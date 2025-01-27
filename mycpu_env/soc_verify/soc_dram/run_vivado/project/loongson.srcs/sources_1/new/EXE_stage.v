`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/10/18 13:32:05
// Design Name: 
// Module Name: EXE_stage
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


module EXE_stage(
    input wire clk,
    input wire reset,
    input wire ms_allowin,
    output wire es_allowin,
    input wire ds_to_es_valid,
    input wire [`DS_TO_ES_BUS_WD - 1:0] ds_to_es_bus,
    output wire es_to_ms_valid,
    output wire [`ES_TO_MS_BUS_WD - 1:0] es_to_ms_bus,
    output wire data_sram_en,
    output wire [3:0] data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata
    );
    wire        [11:0] es_alu_op;
    wire        es_load_op;
    wire        es_src1_is_pc;
    wire        es_src2_is_imm;
    wire        es_src2_is_4;
    wire        es_gr_we;
    wire        es_mem_we;
    wire [4:0]  es_dest;
    wire [31:0] es_imm;
    wire [31:0] es_rj_value;
    wire [31:0] es_rkd_value;
    wire [31:0] es_pc;
    wire        es_res_from_mem;
    
    reg   es_valid;
    wire  es_ready_go;
    reg   [`DS_TO_ES_BUS_WD - 1:0] ds_to_es_bus_r;
    
    wire [31:0] alu_src1;
    wire [31:0] alu_src2;
    wire [31:0] alu_result;
    
    assign es_ready_go = 1'b1;
    assign es_allowin = !es_valid || es_ready_go && ms_allowin;
    assign es_to_ms_valid = es_valid && es_ready_go;
    
    assign alu_src1 = es_src1_is_pc ?es_pc[31:0]:es_rj_value; 
    assign alu_src2 = es_src2_is_imm?es_imm     :es_rkd_value;
    
    alu u_alu(
    .alu_op(es_alu_op),
    .alu_src1(alu_src1),
    .alu_src2(alu_src2),
    .alu_result(alu_result)
    );
    
    assign {
    es_alu_op,
    es_load_op,
    es_src1_is_pc,
    es_src2_is_imm,
    es_src2_is_4,
    es_gr_we,
    es_mem_we,
    es_dest,
    es_imm,
    es_rj_value,
    es_rkd_value,
    es_pc,
    es_res_from_mem
    }=ds_to_es_bus_r;
    
    assign es_to_ms_bus ={
        es_res_from_mem,
        es_gr_we,
        es_dest,
        alu_result,
        es_pc
    };
    
    assign data_sram_en    = 1'b1;
    assign data_sram_we    = es_mem_we && es_valid ? 4'hf : 4'h0;
    assign data_sram_addr  = alu_result;
    assign data_sram_wdata = es_rkd_value; 
    
    always @(posedge clk)begin
        if(reset)begin
            es_valid <= 1'b0;
        end
        else if(es_allowin)begin
            es_valid <= ds_to_es_valid;
        end
        if(ds_to_es_valid && es_allowin)begin
            ds_to_es_bus_r <= ds_to_es_bus;
        end
    end 
    
endmodule
