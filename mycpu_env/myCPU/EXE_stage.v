`include "mycpu_head.v"
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
    output wire [31:0] data_sram_wdata,
    output wire [`ES_TO_DS_BUS_WD - 1:0] es_to_ds_bus,
    input wire ws_ertn_flush
  );
  wire [18:0] es_alu_op;
  wire        es_load_op;
  wire        es_src1_is_pc;
  wire        es_src2_is_imm;
  wire        es_src2_is_4;
  wire        es_gr_we;
  wire        es_mem_we;
  wire [ 4:0] es_dest;
  wire [31:0] es_imm;
  wire [31:0] es_rj_value;
  wire [31:0] es_rkd_value;
  wire [31:0] es_pc;
  wire        es_res_from_mem;

  //前递
  wire [31:0] es_reg_data;
  wire [ 4:0] es_reg_dest;
  wire        es_reg_able;

  //字节写使能信号，用于访存指令
  wire        es_mem_sign;
  wire [ 1:0] es_mem_size;
  wire [ 3:0] bwe_b_result;        //byte_write_enable
  wire [ 3:0] bwe_h_result;
  wire [ 3:0] bwe_w_result;
  wire [31:0] es_result_b;
  wire [31:0] es_result_h;

  reg   es_valid;
  wire  es_ready_go;
  reg   [`DS_TO_ES_BUS_WD - 1:0] ds_to_es_bus_r;

  wire [31:0] alu_src1;
  wire [31:0] alu_src2;
  wire [31:0] alu_result;

  //乘除法的添加
  wire div_delay;

  //异常和例外
  wire es_adef;
  wire es_ine;
  wire es_sys;
  wire es_csr_re;
  wire es_csr_we;
  wire [13:0] es_csr_num;
  wire [31:0] es_csr_wmask;
  wire [31:0] es_csr_wvalue;
  wire es_ale;
  wire es_ertn;
  wire es_res_from_csr;
  reg  es_ertn_delay;
  wire es_break;
  wire [1:0] es_rdcnt_detail;
  wire [31:0] mem_addr;
  wire es_ex;
  wire es_res_from_timer;

  assign es_res_from_timer = es_rdcnt_detail != 2'b00;

  //仍然是异常和例外
  always @(posedge clk )
  begin
    if(reset || ws_ertn_flush)
    begin
      es_ertn_delay <= 1'b0;
    end
    else if(es_ertn | es_adef | es_ale | es_ine | es_sys | es_break)
    begin
      es_ertn_delay <= 1'b1;
    end
  end

  assign es_ale = (es_mem_size == 2'b10) ? (alu_result[0] != 1'b0):
         (es_mem_size == 2'b11 ? (alu_result[1:0] != 2'b00):(1'b0));
  assign es_ex = es_ale | es_adef | es_sys | es_break | es_ine;
  assign es_reg_dest = es_dest & {5{es_valid}};
  assign es_reg_data = alu_result;
  assign es_reg_able = ((es_res_from_csr || es_res_from_mem || es_res_from_timer) && es_valid) ? 1'b0 : 1'b1;
  assign mem_addr = alu_result;
  /*
  assign es_to_ds_bus = {es_reg_able,         //1 1
                         es_reg_dest,         //5 6
                         es_reg_data,        //32 38
                        es_csr_we && es_valid,    //1 39
                        es_csr_num};          //14 53
                        */

  assign es_to_ds_bus = {es_reg_able,         //1 1
                         es_reg_dest,         //5 6
                         es_reg_data,        //32 38
                         es_csr_we & es_valid,    //1 39
                         (es_csr_re | es_csr_we) & es_valid //1 40
                        };

  assign es_ready_go = es_ertn_delay ? 1'b0 :(div_delay ? 1'b0 : 1'b1);
  assign es_allowin = !es_valid || es_ready_go && ms_allowin;
  assign es_to_ms_valid = es_valid && es_ready_go;

  assign alu_src1 = es_src1_is_pc  ? es_pc[31:0]:es_rj_value;
  assign alu_src2 = es_src2_is_imm ? es_imm     :es_rkd_value;


  alu u_alu(
        .clk(clk),
        .reset(reset),
        .alu_op(es_alu_op),
        .alu_src1(alu_src1),
        .alu_src2(alu_src2),
        .alu_result(alu_result),
        .div_delay(div_delay)
      );

  //考虑能否复用ID已经使用的decoder来解决访存指令的添加
  /*
  mux_1 u_mux_1(
  .mux_mem_size(es_mem_size),
  .mux_addr(alu_result[1:0]),
  .mux_result(mux_result)
  );
  */
  byte_write_enable_b u_byte_write_enable_b0(
                        .in(alu_result[1:0]),
                        .out(bwe_b_result)
                      );

  byte_write_enable_h u_byte_write_enable_h0(
                        .in(alu_result[1:0]),
                        .out(bwe_h_result)
                      );

  assign bwe_w_result = 4'b1111;

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
      es_res_from_mem,
      es_mem_size,
      es_mem_sign,
      es_adef,
      es_ine,
      es_csr_re,
      es_csr_we,
      es_csr_num,
      es_csr_wmask,
      es_csr_wvalue,
      es_sys,
      es_ertn,
      es_res_from_csr,
      es_break,
      es_rdcnt_detail
    }=ds_to_es_bus_r;

  assign es_to_ms_bus ={
           es_res_from_mem,        //1 1
           es_gr_we,               //1 2
           es_dest,                //5 7
           alu_result,             //32 39
           es_pc,                  //32 71
           es_mem_size ,          //2 73
           es_mem_sign ,              //1 74
           es_adef,               //1 75
           es_ine,               //1 76
           es_sys,                  //1 77
           es_ale,             //1 78
           es_csr_re,          //1 79
           es_csr_we,          //1 80
           es_csr_num,         //14 94
           es_csr_wmask,           //32 126
           es_csr_wvalue,           //32 158
           es_ertn,                  //1 159
           es_res_from_csr,          //1 160
           es_break,              //1 161
           es_rdcnt_detail,        //2 163
           mem_addr                 //32 195
         };

  assign es_result_b = {{8{bwe_b_result[3]}} & es_rkd_value[ 7: 0],
                        {8{bwe_b_result[2]}} & es_rkd_value[ 7: 0],
                        {8{bwe_b_result[1]}} & es_rkd_value[ 7: 0],
                        {8{bwe_b_result[0]}} & es_rkd_value[ 7: 0]};

  assign es_result_h = {{8{bwe_h_result[3:2]}} & es_rkd_value[15:0],
                        {8{bwe_h_result[1:0]}} & es_rkd_value[15:0]};

  assign data_sram_en    = 1'b1;
  assign data_sram_we    = ((es_mem_we && es_valid) ? (es_mem_size == 2'b01 ? bwe_b_result : (es_mem_size == 2'b10 ? bwe_h_result : bwe_w_result)) :4'b0) & {4{~es_ertn_delay}} & {4{~es_ex}};
  assign data_sram_addr  = alu_result;
  //assign data_sram_wdata = es_rkd_value;
  assign data_sram_wdata = es_mem_size == 2'b01 ? es_result_b : (es_mem_size == 2'b10 ? es_result_h : es_rkd_value);

  always @(posedge clk)
  begin
    if(reset || ws_ertn_flush)
    begin
      es_valid <= 1'b0;
      ds_to_es_bus_r <= `DS_TO_ES_BUS_WD'b0;
    end
    else if (es_ertn_delay)
    begin
      es_valid <= 1'b0;
    end
    else if(es_allowin)
    begin
      es_valid <= ds_to_es_valid;
    end

    if(ds_to_es_valid && es_allowin && ~ws_ertn_flush)
    begin
      ds_to_es_bus_r <= ds_to_es_bus;
    end
    else if(ws_ertn_flush)
    begin
      ds_to_es_bus_r <= `DS_TO_ES_BUS_WD'b0;
    end
  end

endmodule
