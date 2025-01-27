`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/10/18 15:47:48
// Design Name: 
// Module Name: WB_stage
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module WB_stage(
    input wire clk,
    input wire reset,
    output wire ws_allowin,
    input wire ms_to_ws_valid,
    input wire [`MS_TO_WS_BUS_WD - 1:0] ms_to_ws_bus,
    output wire [`WS_TO_RF_BUS_WD  - 1:0] ws_to_rf_bus,
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
    );
    wire ws_gr_we;
    wire [ 4:0] ws_dest;
    wire [31:0] ws_final_result;
    wire [31:0] ws_pc;
    
    wire        rf_we   ;
    wire [ 4:0] rf_waddr;
    wire [31:0] rf_wdata;
    
    reg ws_valid;
    wire ws_ready_go;
    reg [`MS_TO_WS_BUS_WD - 1:0] ms_to_ws_bus_r;
    
    assign ws_ready_go = 1'b1;
    assign ws_allowin = !ws_valid || ws_ready_go;
    
    assign {
    ws_gr_we,
    ws_dest,
    ws_final_result,
    ws_pc
    }=ms_to_ws_bus_r;
    
    assign rf_we    = ws_gr_we && ws_valid;
    assign rf_waddr = ws_dest;
    assign rf_wdata = ws_final_result;
    
    assign ws_to_rf_bus={
        rf_we,
        rf_waddr,
        rf_wdata
    };
    
    assign debug_wb_pc       = ws_pc;
    assign debug_wb_rf_we   = {4{rf_we}};
    assign debug_wb_rf_wnum  = ws_dest;
    assign debug_wb_rf_wdata = ws_final_result;
    
    always @(posedge clk)begin
        if(reset)begin
            ws_valid <= 1'b0;
        end
        else if(ws_allowin)begin
            ws_valid <= ms_to_ws_valid;
        end
        if(ms_to_ws_valid && ws_allowin)begin
            ms_to_ws_bus_r <= ms_to_ws_bus;
        end
    end 
    
    
    
endmodule
