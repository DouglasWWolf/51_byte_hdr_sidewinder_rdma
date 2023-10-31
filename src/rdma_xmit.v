//===================================================================================================
//                            ------->  Revision History  <------
//===================================================================================================
//
//   Date     Who   Ver  Changes
//===================================================================================================
// 30-Oct-23  DWW     1  Initial creation
//===================================================================================================

/*

This module monitors incoming AXI Write transactions on the AW (address) and W (data) channels of
an AXI-MM slave, and turns them into outgoing AXI-streams.

This module automatically sends a write-acknowledge on the AXI-MM B channel for every incoming
AXI write transaction.

*/

module rdma_xmit #
(
    parameter integer AXI_DATA_WIDTH = 512,
    parameter integer AXI_ADDR_WIDTH = 64,
    parameter integer RDMA_HDR_LEN = (AXI_ADDR_WIDTH + 8)
)
(
    //=================  This is the main AXI4-slave interface  ================
    input wire  clk, resetn,

    // "Specify write address"              -- Master --    -- Slave --
    input[AXI_ADDR_WIDTH-1:0]               S_AXI_AWADDR,
    input                                   S_AXI_AWVALID,
    input[3:0]                              S_AXI_AWID,
    input[7:0]                              S_AXI_AWLEN,
    input[2:0]                              S_AXI_AWSIZE,
    input[1:0]                              S_AXI_AWBURST,
    input                                   S_AXI_AWLOCK,
    input[3:0]                              S_AXI_AWCACHE,
    input[3:0]                              S_AXI_AWQOS,
    input[2:0]                              S_AXI_AWPROT,

    output                                                  S_AXI_AWREADY,

    // "Write Data"                         -- Master --    -- Slave --
    input[AXI_DATA_WIDTH-1:0]               S_AXI_WDATA,
    input                                   S_AXI_WVALID,
    input[(AXI_DATA_WIDTH/8)-1:0]           S_AXI_WSTRB,
    input                                   S_AXI_WLAST,
    output                                                  S_AXI_WREADY,

    // "Send Write Response"                -- Master --    -- Slave --
    output[1:0]                                             S_AXI_BRESP,
    output                                                  S_AXI_BVALID,
    input                                   S_AXI_BREADY,

    // "Specify read address"               -- Master --    -- Slave --
    input[AXI_ADDR_WIDTH-1:0]               S_AXI_ARADDR,
    input                                   S_AXI_ARVALID,
    input[2:0]                              S_AXI_ARPROT,
    input                                   S_AXI_ARLOCK,
    input[3:0]                              S_AXI_ARID,
    input[7:0]                              S_AXI_ARLEN,
    input[1:0]                              S_AXI_ARBURST,
    input[3:0]                              S_AXI_ARCACHE,
    input[3:0]                              S_AXI_ARQOS,
    output                                                  S_AXI_ARREADY,

    // "Read data back to master"           -- Master --    -- Slave --
    output[AXI_DATA_WIDTH-1:0]                              S_AXI_RDATA,
    output                                                  S_AXI_RVALID,
    output[1:0]                                             S_AXI_RRESP,
    output                                                  S_AXI_RLAST,
    input                                   S_AXI_RREADY,

    //==========================================================================


    //==========================================================================
    //                         AXI-Stream output for data
    //==========================================================================
    output[AXI_DATA_WIDTH  -1:0] AXIS_DATA_TDATA,
    output[AXI_DATA_WIDTH/8-1:0] AXIS_DATA_TKEEP,
    output                       AXIS_DATA_TVALID,
    output                       AXIS_DATA_TLAST,
    input                        AXIS_DATA_TREADY,
    //==========================================================================


    //==========================================================================
    //                     AXI-Stream output for addresses
    //==========================================================================
    output[RDMA_HDR_LEN-1:0] AXIS_RDMA_TDATA,
    output                   AXIS_RDMA_TVALID,
    input                    AXIS_RDMA_TREADY
    //==========================================================================

 );


// Wire the AXI-Slave "AW" channel directly to the AXIS_RDMA output stream
assign AXIS_RDMA_TDATA  = {S_AXI_AWLEN, S_AXI_AWADDR};
assign AXIS_RDMA_TVALID = S_AXI_AWVALID;
assign S_AXI_AWREADY    = AXIS_RDMA_TREADY;


// Wire the AXI-Slave "W" channel directly to the AXIS_DATA output stream
assign AXIS_DATA_TDATA  = S_AXI_WDATA;
assign AXIS_DATA_TKEEP  = S_AXI_WSTRB;
assign AXIS_DATA_TVALID = S_AXI_WVALID;
assign AXIS_DATA_TLAST  = S_AXI_WLAST;
assign S_AXI_WREADY     = AXIS_DATA_TREADY;

// The number of transactions received, and the number of transactions responded to
reg[63:0] transactions_rcvd, transactions_resp;


//====================================================================================
// This state machine counts the number of AXI transactions received.
//
// Drives:
//    transactions_rcvd
//====================================================================================
always @(posedge clk) begin
    
    // If we're in reset...
    if (resetn == 0)
        transactions_rcvd <= 0;
    
    // Otherwise, if this is the last beat of a burst...
    else if (S_AXI_WVALID & S_AXI_WREADY & S_AXI_WLAST)
        transactions_rcvd <= transactions_rcvd + 1;
end
//====================================================================================



//====================================================================================
// This state machine ensures that we issue an AXI response for each AXI transaction
// that we receive.
//
// Drives:
//    S_AXI_BVALID
//    transactions_resp
//====================================================================================

// Our BRESP response is always "OKAY"
assign S_AXI_BRESP = 0;

// Our BRESP output is valid so long as we have transactions we haven't responsed to
assign S_AXI_BVALID = (resetn == 1 && transactions_resp < transactions_rcvd);

// Every time we see a valid handshake on the B-channel, it means that
// we have successfully responded to an AXI write transaction
always @(posedge clk) begin
    if (resetn == 0) 
        transactions_resp <= 0;
    else if (S_AXI_BVALID & S_AXI_BREADY)
        transactions_resp <= transactions_resp + 1;
end
//====================================================================================


endmodule