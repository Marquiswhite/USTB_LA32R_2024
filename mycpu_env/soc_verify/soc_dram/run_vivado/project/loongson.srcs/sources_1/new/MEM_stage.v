`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/10/18 15:09:05
// Design Name: 
// Module Name: MEM_stage
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


module MEM_stage(
    input wire clk,
    input wire reset,
    input wire ws_allowin,
    output wire ms_allowin,
    input wire es_to_ms_valid,
    input wire [`ES_TO_MS_BUS_WD - 1:0] es_to_ms_bus,
    output wire ms_to_ws_valid,
    output wire [`MS_TO_WS_BUS_WD - 1:0] ms_to_ws_bus,
    input wire [31:0] data_sram_rdata
    );
    wire ms_res_from_mem;
    wire ms_gr_we;
    wire [4:0] ms_dest;
    wire [31:0] ms_alu_result;
    wire [31:0] ms_pc;
    
    wire [31:0] mem_result;
    wire [31:0] final_result;
    
    reg ms_valid;
    wire ms_ready_go;
    reg [`ES_TO_MS_BUS_WD - 1:0] es_to_ms_bus_r;
    
    assign {
    ms_res_from_mem,
    ms_gr_we,
    ms_dest,
    ms_alu_result,
    ms_pc
    }=es_to_ms_bus_r;
    
    assign ms_ready_go = 1'b1;
    assign ms_allowin = !ms_valid || ms_ready_go && ws_allowin;
    assign ms_to_ws_valid = ms_valid && ms_ready_go;
    
    assign mem_result = data_sram_rdata;
    assign final_result = ms_res_from_mem ?mem_result:ms_alu_result;
    
    assign ms_to_ws_bus={
        ms_gr_we,
        ms_dest,
        final_result,
        ms_pc
    };
    
    always @(posedge clk)begin
        if(reset)begin
            ms_valid <= 1'b0;
        end
        else if(ms_allowin)begin
            ms_valid <= es_to_ms_valid;
        end
        if(es_to_ms_valid && ms_allowin)begin
            es_to_ms_bus_r <= es_to_ms_bus;
        end
    end 
    
    
endmodule
