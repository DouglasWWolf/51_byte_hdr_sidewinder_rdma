

module rdma_pack #
(
    parameter STREAM_WB = 64,       /* Stream width in bytes              */
    parameter RDMA_HDR_LEN = 51     /* Length of the RDMA header in bytes */
)
(
    // clock and reset
    input  clk, resetn,

    //==========================================================================
    //                         Input AXI-Stream
    //==========================================================================
    input[STREAM_WB*8-1:0] AXIS_RX_TDATA,
    input[STREAM_WB-1:0]   AXIS_RX_TKEEP,
    input                  AXIS_RX_TVALID,
    input                  AXIS_RX_TLAST,
    output                 AXIS_RX_TREADY,
    
    //==========================================================================


    //==========================================================================
    //                         Output AXI-Stream
    //==========================================================================
    output[STREAM_WB*8-1:0] AXIS_TX_TDATA,
    output[STREAM_WB-1:0]   AXIS_TX_TKEEP,
    output                  AXIS_TX_TVALID,
    output                  AXIS_TX_TLAST,
    input                   AXIS_TX_TREADY
    //==========================================================================

 );

// This is the number of bytes left in a data-cycle after the header
localparam REMAINING_LEN = STREAM_WB - RDMA_HDR_LEN;

// These zero bits for REMAINING_LEN bytes and for REMAINING_LEN bits
localparam[REMAINING_LEN*8-1:0] REMAINING_ZBYTES = 0;
localparam[REMAINING_LEN  -1:0] REMAINING_ZBITS  = 0;

// This is the state of the packing state machine
reg[2:0] psm_state;

// In states 2 and 3, this is leftover tdata and tkeep from the prior incoming data cycle
reg[RDMA_HDR_LEN*8-1:0] prior_tdata;
reg[RDMA_HDR_LEN  -1:0] prior_tkeep;

// We've reached the end of the incoming packet if TKEEP indicates there are no more valid bytes 
wire end_of_packet = (AXIS_RX_TKEEP[REMAINING_LEN +: RDMA_HDR_LEN] == 0);


// We're always ready to accept incoming data in state 1, and in state 2 we're ready to accept incoming data
// any time we're clear to immediately write that data to the output
assign AXIS_RX_TREADY = (psm_state == 1) ? 1
                      : (psm_state == 2) ? AXIS_TX_TREADY
                      : 0;

// The output tdata is always the first RDMA_HDR_LEN bytes of the prior cycle, and REMAINING_LEN bytes of this cycle
assign AXIS_TX_TDATA = (psm_state == 2) ? {AXIS_RX_TDATA[0 +: REMAINING_LEN *8], prior_tdata} 
                     : (psm_state == 3) ? {REMAINING_ZBYTES, prior_tdata}
                     : 0;

// The output tkeep is always the first RDMA_HDR_LEN bits of the prior cycle, and REMAINING_LEN bits of this cycle
assign AXIS_TX_TKEEP = (psm_state == 2) ? {AXIS_RX_TKEEP[0 +: REMAINING_LEN   ], prior_tkeep} 
                     : (psm_state == 3) ? {REMAINING_ZBITS, prior_tkeep}
                     : 0;

// We are always outputting valid data in states 2 and 3
assign AXIS_TX_TVALID = (psm_state == 2 || psm_state == 3);

// TLAST is asserted in state 2 on a data-cycle with partial output, or always in state 3
assign AXIS_TX_TLAST = (psm_state == 2 && AXIS_RX_TREADY && AXIS_RX_TVALID && end_of_packet)
                     | (psm_state == 3);


//====================================================================================
// This state machine reads the input stream and packs bytes into the output stream.
// 
// It expects:
//    Once the first cycle of a packet arrives, valid cycles will arrive on every
//    subsequent cycle until the entire packet has arrived.
//
//    The rightmost RDMA_HDR_LEN bytes of the first cycle will be valid.  The 
//    invalid bytes to the left of that header are going to be thrown away.
//====================================================================================
always @(posedge clk) begin
    
    if (resetn == 0) begin
        psm_state <= 0;
    
    end else case(psm_state)

        // In state 0 we're coming out of reset
        0:  begin
                psm_state <= 1;
            end

        // In state 1 we're waiting for the first data-cycle to arrive.  That
        // data cycle is the RDMA header
        1:  if (AXIS_RX_TVALID & AXIS_RX_TREADY) begin
                prior_tdata <= AXIS_RX_TDATA[0 +: RDMA_HDR_LEN * 8];
                prior_tkeep <= {RDMA_HDR_LEN{1'b1}};
                psm_state   <= 2;
            end

        // Here we wait for data-cycles to arrive after the header has arrived
        2:  if (AXIS_RX_TVALID & AXIS_RX_TREADY) begin
                prior_tdata <= AXIS_RX_TDATA[REMAINING_LEN *8 +: RDMA_HDR_LEN * 8];
                prior_tkeep <= AXIS_RX_TKEEP[REMAINING_LEN    +: RDMA_HDR_LEN    ];
                if (AXIS_RX_TLAST) psm_state <= (AXIS_TX_TLAST) ? 1 : 3;
            end

        // Here we're outputting the last cycle of the packet.  Not every byte of
        // this data-cycle will have TKEEP set
        3:  if (AXIS_TX_TVALID & AXIS_TX_TREADY) psm_state <= 1;

    endcase

end
//====================================================================================

endmodule