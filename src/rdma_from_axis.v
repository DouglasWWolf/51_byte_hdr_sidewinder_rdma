
//====================================================================================
//                        ------->  Revision History  <------
//====================================================================================
//
//   Date     Who   Ver  Changes
//====================================================================================
// 25-Jul-23  DWW  1000  Initial creation
//====================================================================================
/*

    This module formats an AXI stream as a UDP packet.  It does this by buffering up
    an incoming packet (in a FIFO) while it counts the number of bytes in the
    packet.  Once the incoming packet has arrived, the packet-length is written into
    its own FIFO.

    The thread that reads those two FIFOs builds a valid RDMA header header then
    outputs the RDMA header (in its own data-cycle) followed by the packet data.

    <><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
    <> An RDMA header is an ordinary ethernet/IP/UDP header followed by:                          <>
    <>     An 8-byte target address                                                              <>
    <>     A 1-byte AXI-AWLEN value
    <><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>

    The incoming AXI Stream should be byte packed; only the last beat (the beat with
    AXIS_DATA_TLAST asserted) may have a TKEEP value with bits set to 0.
    
    Notable busses:

        AXIS_RDMA feeds the input of the rdma-header FIFO
        AXIS_DATA feeds the input of the packet-data FIFO
        fplin feeds the input of the packet-length FIFO

        fplout is the output of the packet-length FIFO
        fpdout is the output of the packet-data FIFO
        ftaout is the output of the rdma-header FIFO

        AXIS_TX is the output stream containing an *sparse* RDMA packet.  It is the job of
        a downstream module (rdma_pack.v) to byte-pack the sparse RDMA packet into a fully
        packed AXI stream.

*/
module rdma_from_axis # 
(
    // This is the width of the incoming and outgoing data bus in bytes
    parameter STREAM_WB = 64,      

    // The number of bytes in the RDMA header fields
    parameter RDMA_HDR_LEN = 9,

    // Last octet of the source MAC address
    parameter[ 7:0] SRC_MAC = 2,    
    
    // The source IP address
    parameter[ 7:0] SRC_IP0 = 10,
    parameter[ 7:0] SRC_IP1 = 1,
    parameter[ 7:0] SRC_IP2 = 1,
    parameter[ 7:0] SRC_IP3 = 2,

    // The destiniation IP address
    parameter[ 7:0] DST_IP0 = 10,
    parameter[ 7:0] DST_IP1 = 1,
    parameter[ 7:0] DST_IP2 = 1,
    parameter[ 7:0] DST_IP3 = 255,
    
    // The source and destination UDP ports
    parameter[15:0] SRC_PORT = 1000,
    parameter[15:0] DST_PORT = 32002,

    // This must be at least as large as the number of the smallest packets that
    // can fit into the data FIFO.   Min is 16.  
    parameter MAX_PACKET_COUNT = 256,

    // This should be at minimum MAX_PACKET_COUNT * # of data-cycles in the smallest
    // incoming packet.  This number must be large enough to accomodate the number of
    // data cycles in the largest incoming packet.
    parameter DATA_FIFO_SIZE = 256

    //<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
    //>> DATA_FIFO_SIZE / MAX_PACKET_COUNT = # of cycles in the smallest incoming data packet
    //<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
) 
(
    input clk, resetn,

    //=========================  Incoming Packet Data  =========================
    input [STREAM_WB*8-1:0] AXIS_DATA_TDATA,
    input [STREAM_WB  -1:0] AXIS_DATA_TKEEP,
    input                   AXIS_DATA_TVALID,
    input                   AXIS_DATA_TLAST,
    output                  AXIS_DATA_TREADY,
    //==========================================================================


    //===================  Incoming RDMA header fields  ==========================
    input [RDMA_HDR_LEN*8-1:0] AXIS_RDMA_TDATA,
    input                      AXIS_RDMA_TVALID,
    output                     AXIS_RDMA_TREADY,
    //==========================================================================

    
    //========================  The outgoing UDP packet  =======================
    output [STREAM_WB*8-1:0] AXIS_TX_TDATA,
    output [STREAM_WB  -1:0] AXIS_TX_TKEEP,
    output                   AXIS_TX_TVALID,
    output                   AXIS_TX_TLAST,
    input                    AXIS_TX_TREADY
    //==========================================================================

);

//==================  The output of the packet-data FIFO  ==================
wire[STREAM_WB*8-1:0] fpdout_tdata;
wire[STREAM_WB  -1:0] fpdout_tkeep;
wire                  fpdout_tvalid;
wire                  fpdout_tlast;
wire                  fpdout_tready;
//==========================================================================


//=============  This feeds the input of the packet-length FIFO  ===========
wire[15:0] fplin_tdata;
wire       fplin_tvalid;
wire       fplin_tready;
//==========================================================================

//=============  This is the output of the packet-length FIFO  =============
wire[15:0] fplout_tdata;
wire       fplout_tvalid;
wire       fplout_tready;
//==========================================================================

// The length (in bytes) of a standard Ethernet header 
localparam ETH_HDR_LEN = 14;

// The length (in bytes) of a standard header for an IP packet
localparam IP_HDR_LEN = 20;

// The length (in bytes) of a standard header for a UDP packet
localparam UDP_HDR_LEN = 8;

// This is the width (in bytes) of the RDMA header
localparam FULL_HDR_LEN = ETH_HDR_LEN + IP_HDR_LEN + UDP_HDR_LEN + RDMA_HDR_LEN; 

// How many bytes remain in AXIS_TX_TDATA after the Eth/IPv4/UDP header is packed into it
localparam REMAINING_LEN = STREAM_WB - FULL_HDR_LEN;

// These zero bits for REMAINING_LEN bytes and for REMAINING_LEN bits
localparam[REMAINING_LEN*8-1:0] REMAINING_ZBYTES = 0;
localparam[REMAINING_LEN  -1:0] REMAINING_ZBITS  = 0;

// This is TKEEP for the full header
localparam[FULL_HDR_LEN-1:0] FULL_HDR_TKEEP = {FULL_HDR_LEN{1'b1}};

// This is the state of the primary state machine
reg[1:0] fsm_state;

// The statically declared ethernet header fields
localparam[47:0] eth_dst_mac    = {48'hFFFFFFFFFFFF};
localparam[47:0] eth_src_mac    = {40'hC400AD0000, SRC_MAC};
localparam[15:0] eth_frame_type = 16'h0800;

// The statically declared IPv4 header fields
localparam[15:0] ip4_ver_dsf    = 16'h4500;
localparam[15:0] ip4_id         = 16'hDEAD;
localparam[15:0] ip4_flags      = 16'h4000;
localparam[15:0] ip4_ttl_prot   = 16'h4011;
localparam[15:0] ip4_srcip_h    = {SRC_IP0, SRC_IP1};
localparam[15:0] ip4_srcip_l    = {SRC_IP2, SRC_IP3};
localparam[15:0] ip4_dstip_h    = {DST_IP0, DST_IP1};
localparam[15:0] ip4_dstip_l    = {DST_IP2, DST_IP3};

// The statically declared UDP header fields
localparam[15:0] udp_src_port   = SRC_PORT;
localparam[15:0] udp_dst_port   = DST_PORT;
localparam[15:0] udp_checksum   = 0;

// Compute both the IPv4 packet length and UDP packet length
wire[15:0]       ip4_length     = IP_HDR_LEN  + UDP_HDR_LEN + RDMA_HDR_LEN + fplout_tdata;
wire[15:0]       udp_length     =               UDP_HDR_LEN + RDMA_HDR_LEN + fplout_tdata;

// Compute the 32-bit version of the IPv4 header checksum
wire[31:0] ip4_cs32 = ip4_ver_dsf
                    + ip4_id
                    + ip4_flags
                    + ip4_ttl_prot
                    + ip4_srcip_h
                    + ip4_srcip_l
                    + ip4_dstip_h
                    + ip4_dstip_l
                    + ip4_length;

// Compute the 16-bit IPv4 checksum
wire[15:0] ip4_checksum = ~(ip4_cs32[15:0] + ip4_cs32[31:16]);

// This is the output bus of RDMA header fields FIFO
reg [RDMA_HDR_LEN*8-1:0] frhout_tdata_reg;
wire[RDMA_HDR_LEN*8-1:0] frhout_tdata;
wire                     frhout_tvalid;
reg                      frhout_tready;

// The RDMA header is either in the FIFO data output, or has been stored in frhout_tdata_reg
wire[RDMA_HDR_LEN*8-1:0] rdma_hdr_fields = (frhout_tready == 1) ? frhout_tdata : frhout_tdata_reg;

// Extract the target address and burst-length from the RDMA header fields
wire[63:0] target_addr = rdma_hdr_fields[0*8 +: 64];
wire[7:0 ] burst_len   = rdma_hdr_fields[8*8 +:  8];

// This is the 51-byte packet header for an RDMA packet
wire[FULL_HDR_LEN*8-1:0] pkt_header =
{
    // Ethernet header fields
    eth_dst_mac,
    eth_src_mac,
    eth_frame_type,

    // IPv4 header fields
    ip4_ver_dsf,
    ip4_length,
    ip4_id,
    ip4_flags,
    ip4_ttl_prot,
    ip4_checksum,
    ip4_srcip_h,
    ip4_srcip_l,
    ip4_dstip_h,
    ip4_dstip_l,

    // UDP header fields
    udp_src_port,
    udp_dst_port,
    udp_length,
    udp_checksum,
    
    // RDMA header fields
    target_addr,
    burst_len
};


// The Ethernet IP sends the bytes from least-sigificant-byte to most-significant-byte.  
// This means we need to create a little-endian (i.e., reversed) version of our packet 
// header.
wire[FULL_HDR_LEN*8-1:0] pkt_header_le;
genvar i;
for (i=0; i<FULL_HDR_LEN; i=i+1) begin
    assign pkt_header_le[i*8 +:8] = pkt_header[(FULL_HDR_LEN-1-i)*8 +:8];
end 

//=====================================================================================================================
// In state 1, we drive AXIS_TX with the outgoing RDMA header.
// In state 2, AXIS_TX is driven directly from the output of the packet-data FIFO.
//=====================================================================================================================
assign AXIS_TX_TDATA = (fsm_state == 1) ? {REMAINING_ZBYTES, pkt_header_le}
                     : (fsm_state == 2) ? fpdout_tdata
                     : 0;

assign AXIS_TX_TKEEP = (fsm_state == 1) ? {REMAINING_ZBITS, FULL_HDR_TKEEP}
                     : (fsm_state == 2) ? fpdout_tkeep
                     : 0;

assign AXIS_TX_TLAST = (fsm_state == 2 & fpdout_tlast);

assign AXIS_TX_TVALID = (fsm_state == 1) ? (fplout_tvalid & fplout_tready)
                      : (fsm_state == 2) ? fpdout_tvalid
                      : 0;


assign fpdout_tready  = (fsm_state == 2 & AXIS_TX_TREADY);
//=====================================================================================================================




//=====================================================================================================================
// This state machine has 3 states:
//
//   0 = We just came out of reset.  This state initializes things.
//
//   1 = Waiting for a "packet length" to arrive on AXIS_LEN.  When it does, the 50-byte packet header is emitted
//       on AXIS_TX.
//
//   2 = Copying the output of the packet-data FIFO to the AXIS_TX output stream
//
//
// Since logic outside of this routine buffers up the entire packet prior to presenting us with a packet-length on
// the AXIS_LEN bus, we may assume that an incoming cycle of user data (on fpdout_tdata) will be available on every
// consecutive cycle after receiving a packet-length.
//=====================================================================================================================

// We are able to receive data from AXIS_LEN in state 1 only when the TX bus is ready for us to send
assign fplout_tready = (resetn == 1 & fsm_state == 1 & AXIS_TX_TREADY);

always @(posedge clk) begin
    if (resetn == 0) begin
        frhout_tready  <= 0;
        fsm_state      <= 0;
    
    end else case(fsm_state) 
        
        // Here we're coming out of reset
        0:  begin
                frhout_tready <= 1;
                fsm_state     <= 1;
            end


        // Here we're waiting for a packet-length to arrive on the AXIS_LEN bus.  While
        // we're waiting, we will capture the first data-cycle of the AXIS_RX bus.
        1:  begin

                // While we're waiting for data to arrive on AXIS_LEN, read the
                // first cycle of target-address from its FIFO.  We can use the 
                // state of "frhout_tready" to determine whether target-address
                // is sitting in rdma_hdr_fields or in frhout_tdata.
                //
                // The target address could arrive on the same data-cycle as the packet-length,
                // or it could arrive earlier.
                if (frhout_tready & frhout_tvalid) begin
                    frhout_tdata_reg <= frhout_tdata;
                    frhout_tready    <= 0;                    
                end


                // If a packet-length arrives, the packet header is immediately emitted.
                // We know for a fact that the entire packet will be queued up and waiting
                // for us on the AXIS_DATA_BUS by the time we receive a packet-length on the
                // AXIS_LEN bus.
                if (fplout_tready & fplout_tvalid) fsm_state <= 2;
            end


        // When we receive the last data-cycle of the packet, go back to state 1
        2:  if (fpdout_tvalid & fpdout_tready & fpdout_tlast) fsm_state <= 1;
        
    endcase
end
//=====================================================================================================================



//=====================================================================================================================
// This block counts the number of one bits in AXIS_DATA_TKEEP, thereby determining the number of data-bytes in the
// AXIS_DATA_TDATA field. 
//=====================================================================================================================
reg[7:0] data_byte_count;
//---------------------------------------------------------------------------------------------------------------------
integer n;
always @*
begin
    data_byte_count = 0;  
    for (n=0;n<64;n=n+1) begin   
        data_byte_count = data_byte_count + AXIS_DATA_TKEEP[n];
    end
end
//=====================================================================================================================


//=====================================================================================================================
// This state machine writes entries to the packet-length FIFO
//=====================================================================================================================
reg[15:0] packet_size;
//---------------------------------------------------------------------------------------------------------------------

// AXIS_PL contains the measured length of the incoming data packet
assign fplin_tdata = packet_size + data_byte_count;

// AXIS_PL has valid data on the cycle where we see TLAST on the incoming data packet
assign fplin_tvalid = (AXIS_DATA_TREADY & AXIS_DATA_TVALID & AXIS_DATA_TLAST);

always @(posedge clk) begin
    if (resetn == 0) begin
        packet_size <= 0;
    end else begin
        
        // On every beat of incoming packet data, accumulate the packet-length.
        // When we see the last beat of the packet, write the packet-length to the FIFO
        if (AXIS_DATA_TVALID & AXIS_DATA_TREADY) begin
            if (AXIS_DATA_TLAST == 0)
                packet_size <= packet_size + data_byte_count;
            else 
                packet_size <= 0;
        end
    end
end
//=====================================================================================================================


//====================================================================================
// This FIFO holds the incoming packet data
//====================================================================================
xpm_fifo_axis #
(
   .FIFO_DEPTH(DATA_FIFO_SIZE),    // DECIMAL
   .TDATA_WIDTH(STREAM_WB * 8),    // DECIMAL
   .FIFO_MEMORY_TYPE("auto"),      // String
   .PACKET_FIFO("false"),          // String
   .USE_ADV_FEATURES("0000")       // String
)
packet_data_fifo
(
    // Clock and reset
   .s_aclk   (clk   ),                       
   .m_aclk   (clk   ),             
   .s_aresetn(resetn),

    // The input bus to the FIFO
   .s_axis_tdata (AXIS_DATA_TDATA ),
   .s_axis_tkeep (AXIS_DATA_TKEEP ),
   .s_axis_tvalid(AXIS_DATA_TVALID),
   .s_axis_tlast (AXIS_DATA_TLAST ),
   .s_axis_tready(AXIS_DATA_TREADY),

    // The output bus of the FIFO
   .m_axis_tdata (fpdout_tdata ),     
   .m_axis_tkeep (fpdout_tkeep ),
   .m_axis_tvalid(fpdout_tvalid),       
   .m_axis_tlast (fpdout_tlast ),         
   .m_axis_tready(fpdout_tready),


    // Unused input stream signals
   .s_axis_tdest(),
   .s_axis_tid  (),
   .s_axis_tstrb(),
   .s_axis_tuser(),

    // Unused output stream signals
   .m_axis_tdest(),             
   .m_axis_tid  (),               
   .m_axis_tstrb(), 
   .m_axis_tuser(),         

    // Other unused signals
   .almost_empty_axis(),
   .almost_full_axis(), 
   .dbiterr_axis(),          
   .prog_empty_axis(), 
   .prog_full_axis(), 
   .rd_data_count_axis(), 
   .sbiterr_axis(),
   .wr_data_count_axis(),
   .injectdbiterr_axis(),
   .injectsbiterr_axis()
);
//====================================================================================


//====================================================================================
// This FIFO holds the packet-length of the incoming data packets
//====================================================================================
xpm_fifo_axis #
(
   .FIFO_DEPTH(MAX_PACKET_COUNT),  // DECIMAL
   .TDATA_WIDTH(16),               // DECIMAL
   .FIFO_MEMORY_TYPE("auto"),      // String
   .PACKET_FIFO("false"),          // String
   .USE_ADV_FEATURES("0000")       // String
)
packet_length_fifo
(
    // Clock and reset
   .s_aclk   (clk   ),                       
   .m_aclk   (clk   ),             
   .s_aresetn(resetn),

    // The input bus to the FIFO
   .s_axis_tdata (fplin_tdata ),
   .s_axis_tvalid(fplin_tvalid),
   .s_axis_tready(fplin_tready),

    // The output bus of the FIFO
   .m_axis_tdata (fplout_tdata ),     
   .m_axis_tvalid(fplout_tvalid),       
   .m_axis_tready(fplout_tready),     

    // Unused input stream signals
   .s_axis_tdest(),
   .s_axis_tid  (),
   .s_axis_tstrb(),
   .s_axis_tuser(),
   .s_axis_tkeep(),
   .s_axis_tlast(),

    // Unused output stream signals
   .m_axis_tdest(),             
   .m_axis_tid  (),               
   .m_axis_tstrb(), 
   .m_axis_tuser(),         
   .m_axis_tkeep(),           
   .m_axis_tlast(),         

    // Other unused signals
   .almost_empty_axis(),
   .almost_full_axis(), 
   .dbiterr_axis(),          
   .prog_empty_axis(), 
   .prog_full_axis(), 
   .rd_data_count_axis(), 
   .sbiterr_axis(),
   .wr_data_count_axis(),
   .injectdbiterr_axis(),
   .injectsbiterr_axis()
);
//====================================================================================


//====================================================================================
// This FIFO holds the target-address of the incoming data packets
//====================================================================================
xpm_fifo_axis #
(
   .FIFO_DEPTH(MAX_PACKET_COUNT),  // DECIMAL
   .TDATA_WIDTH(RDMA_HDR_LEN*8),   // DECIMAL
   .FIFO_MEMORY_TYPE("auto"),      // String
   .PACKET_FIFO("false"),          // String
   .USE_ADV_FEATURES("0000")       // String
)
rdma_hdr_fifo
(
    // Clock and reset
   .s_aclk   (clk   ),                       
   .m_aclk   (clk   ),             
   .s_aresetn(resetn),

    // The input of this FIFO is wired directly into the module interface
   .s_axis_tdata (AXIS_RDMA_TDATA ),
   .s_axis_tvalid(AXIS_RDMA_TVALID),
   .s_axis_tready(AXIS_RDMA_TREADY),

    // The output bus of the FIFO
   .m_axis_tdata (frhout_tdata ),     
   .m_axis_tvalid(frhout_tvalid),       
   .m_axis_tready(frhout_tready),     

    // Unused input stream signals
   .s_axis_tdest(),
   .s_axis_tid  (),
   .s_axis_tstrb(),
   .s_axis_tuser(),
   .s_axis_tkeep(),
   .s_axis_tlast(),

    // Unused output stream signals
   .m_axis_tdest(),             
   .m_axis_tid  (),               
   .m_axis_tstrb(), 
   .m_axis_tuser(),         
   .m_axis_tkeep(),           
   .m_axis_tlast(),         

    // Other unused signals
   .almost_empty_axis(),
   .almost_full_axis(), 
   .dbiterr_axis(),          
   .prog_empty_axis(), 
   .prog_full_axis(), 
   .rd_data_count_axis(), 
   .sbiterr_axis(),
   .wr_data_count_axis(),
   .injectdbiterr_axis(),
   .injectsbiterr_axis()
);
//====================================================================================


endmodule