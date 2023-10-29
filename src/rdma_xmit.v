
module rdma_xmit #
(
    parameter integer AXI_DATA_WIDTH = 512,
    parameter integer AXI_ADDR_WIDTH = 64
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
    output reg[1:0]                                         S_AXI_BRESP,
    output reg                                              S_AXI_BVALID,
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
    output[AXI_DATA_WIDTH-1:0] AXIS_DATA_TDATA,
    output                     AXIS_DATA_TVALID,
    output                     AXIS_DATA_TLAST,
    input                      AXIS_DATA_TREADY,
    //==========================================================================


    //==========================================================================
    //                     AXI-Stream output for addresses
    //==========================================================================
    output[AXI_ADDR_WIDTH-1:0] AXIS_ADDR_TDATA,
    output                     AXIS_ADDR_TVALID,
    input                      AXIS_ADDR_TREADY
    //==========================================================================

 );


// Wire the AXI-Slave "AW" channel directly to the "target address" output stream
assign AXIS_ADDR_TDATA  = S_AXI_AWADDR;
assign AXIS_ADDR_TVALID = S_AXI_AWVALID;
assign S_AXI_AWREADY = AXIS_ADDR_TREADY;


// Wire the AXI-Slave "W" channel directly to the streaming output
assign AXIS_DATA_TDATA   = S_AXI_WDATA;
assign AXIS_DATA_TVALID  = S_AXI_WVALID;
assign AXIS_DATA_TLAST   = S_AXI_WLAST;
assign S_AXI_WREADY      = AXIS_DATA_TREADY;

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
reg resp_state;

always @(posedge clk) begin

    // If we're in reset...
    if (resetn == 0) begin
        transactions_resp <= 0;
        S_AXI_BVALID      <= 0;
        resp_state        <= 0;
    
    end else case (resp_state)

    
        // If we've received a new transaction, issue the response, then 
        // go wait for the handshake
        0:  if (transactions_resp < transactions_rcvd) begin
                S_AXI_BRESP  <= 0;
                S_AXI_BVALID <= 1;
                resp_state   <= 1;
            end


        // Here we're waiting for the handshake on the AXI-Slave B channel
        1:  if (S_AXI_BVALID & S_AXI_BREADY) begin
                transactions_resp <= transactions_resp + 1;
                S_AXI_BVALID      <= 0;
                resp_state        <= 0;
            end

    endcase

end
//====================================================================================


endmodule