`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/10/14 20:34:47
// Design Name: 
// Module Name: ID_stage
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


 //ȡָ�׶ε���Ҫ�����ǽ�ָ��ȡ��
`include "mycpu_head.v"

module IF_stage(
    input wire clk,
    input wire reset,
    input wire ds_allowin,
    input wire [`BR_BUS_WD -1 :0] br_bus,//ע����input  ID->IF
    output wire fs_to_ds_valid,
    
    output wire [`FS_TO_DS_BUS_WD-1 :0] fs_to_ds_bus,
 // inst sram interface
    output wire inst_sram_en,
    output wire [3:0] inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input wire [31:0] inst_sram_rdata
 );

 
 //if�� �� �� �� ��
 reg fs_valid;
 wire fs_ready_go;
 wire fs_allowin;
 wire to_fs_valid;

 wire [31:0] seq_pc; //˳ �� �� �� ַ
 wire [31:0] next_pc; //�� һ �� ִ �� �� �� ַ�� �� �� �� �� ת

 wire br_taken; //�� ת �� ��
 wire [31:0] br_target;//�� ת �� ַ

 assign {br_taken,br_target} = br_bus; //�� ת �� �� �� ��

 wire [31:0] fs_inst; //fs�� �� ��inst��pc,�� �� �� ��
 reg [31:0] fs_pc;

 //if��id�� �� ��bus
 assign fs_to_ds_bus = {fs_inst,fs_pc};

 //pre-if stage
 assign to_fs_valid = ~reset;
 assign seq_pc = fs_pc + 3'h4; //˳ ��
 assign next_pc = (br_taken == 1 ) ? br_target :seq_pc;//Ŀ�ĵ�ַ��˳���ַ

 //IF stage
 assign fs_ready_go = 1'b1; //�� �� �� ��
 assign fs_allowin = !fs_valid || fs_ready_go && ds_allowin; //�� �� �� �룬
 assign fs_to_ds_valid = fs_valid && fs_ready_go;

 always@(posedge clk)begin
 if(reset)begin
 fs_valid <= 1'b0;
 end else if(fs_allowin)begin
 fs_valid <= to_fs_valid;
 end
 end

 always@(posedge clk)begin
 if(reset)begin
 fs_pc <= 32'h1bfffffc;
 end else if(to_fs_valid && fs_allowin)begin

 fs_pc <= next_pc;
 end
 end

 assign inst_sram_en = to_fs_valid && fs_allowin; //
 assign inst_sram_we = 4'h0; //д
 assign inst_sram_addr =next_pc;
 assign inst_sram_wdata = 32'b0;

 assign fs_inst = inst_sram_rdata;

 endmodule
