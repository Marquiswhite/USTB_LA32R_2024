`include"mycpu_head.v"
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
    input wire [31:0] data_sram_rdata,
    output wire [`MS_TO_DS_BUS_WD - 1:0] ms_to_ds_bus,
    input wire ws_ertn_flush
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

  //前递
  wire [31:0] ms_reg_data;
  wire [ 4:0] ms_reg_dest;
  wire        ms_reg_able;        //记录结果是否出来了

  //用于访存指令的添加
  wire        ms_mem_sign;
  wire [ 1:0] ms_mem_size;
  wire        mem_result_choose;
  wire [ 7:0] mem_result_b;
  wire [15:0] mem_result_h;
  wire [31:0] mem_result_bs;
  wire [31:0] mem_result_hs;
  wire [31:0] mem_result_bu;
  wire [31:0] mem_result_hu;
  wire [ 3:0] mem_byte_b;
  wire [ 3:0] mem_byte_h;
  wire        mem_h_or_b;         //1为h,0为b

  //异常和例外
  wire ms_adef;
  wire ms_ine;
  wire ms_sys;
  wire ms_ale;
  wire ms_csr_re;
  wire ms_csr_we;
  wire [13:0] ms_csr_num;
  wire [31:0] ms_csr_wmask;
  wire [31:0] ms_csr_wvalue;
  wire ms_ertn;
  wire ms_res_from_csr;
  wire ms_break;
  wire [1:0] ms_rdcnt_detail;
  wire [31:0] ms_mem_addr;
  wire ms_res_from_timer;

  assign ms_res_from_timer = ms_rdcnt_detail != 2'b00;

  assign ms_reg_able = (ms_valid && (ms_res_from_csr || ms_res_from_timer)) ? 1'b0 : 1'b1;
  assign ms_reg_dest = ms_dest & {5{ms_valid}};
  assign ms_reg_data = final_result;
  assign ms_to_ds_bus = { ms_reg_able,    //1 1
                          ms_reg_dest,    //5 6
                          ms_reg_data,    //32 38
                          ms_csr_we & ms_valid,    //1  39
                          (ms_csr_re | ms_csr_we) & ms_valid    //1 40
                          }; 


  assign {
      ms_res_from_mem,
      ms_gr_we,
      ms_dest,
      ms_alu_result,
      ms_pc,
      ms_mem_size,
      ms_mem_sign,
      ms_adef,
      ms_ine,
      ms_sys,
      ms_ale,
      ms_csr_re,
      ms_csr_we,
      ms_csr_num,
      ms_csr_wmask,
      ms_csr_wvalue,
      ms_ertn,
      ms_res_from_csr,
      ms_break,
      ms_rdcnt_detail,
      ms_mem_addr
    }=es_to_ms_bus_r;

  assign ms_ready_go = 1'b1;
  assign ms_allowin = !ms_valid || ms_ready_go && ws_allowin;
  assign ms_to_ws_valid = ms_valid && ms_ready_go;

  assign mem_result    = data_sram_rdata;

  //这是认为取回的4个字节是从地址开始往后四位
  /*
  assign mem_result_bs = (ms_mem_size == 2'b01 && ms_mem_sign == 1'b0) ? {{24{mem_result[31]}}, mem_result[31:24]} : 32'b0;
  assign mem_result_hs = (ms_mem_size == 2'b10 && ms_mem_sign == 1'b0) ? {{16{mem_result[31]}}, mem_result[31:16]} : 32'b0;
  assign mem_result_bu = (ms_mem_size == 2'b01 && ms_mem_sign == 1'b1) ? {{24'b0, mem_result[31:24]}} : 32'b0;
  assign mem_result_hu = (ms_mem_size == 2'b10 && ms_mem_sign == 1'b1) ? {{16'b0, mem_result[31:16]}} : 32'b0;
  */
  byte_write_enable_b u_byte_write_enable_b1(
                        .in(ms_alu_result[1:0]),
                        .out(mem_byte_b)
                      );

  byte_write_enable_h u_byte_write_enable_h1(
                        .in(ms_alu_result[1:0]),
                        .out(mem_byte_h)
                      );

  assign mem_h_or_b = ms_mem_size == 2'b10 ? 1'b1 : 1'b0;

  assign mem_result_b = {8{mem_byte_b[3]}}   & mem_result[31:24] |
         {8{mem_byte_b[2]}}   & mem_result[23:16] |
         {8{mem_byte_b[1]}}   & mem_result[15: 8] |
         {8{mem_byte_b[0]}}   & mem_result[ 7: 0];
  assign mem_result_h = {8{mem_byte_h[3:2]}} & mem_result[31:16] |
         {8{mem_byte_h[1:0]}} & mem_result[15:0];

  assign mem_result_bs = {{24{mem_result_b[ 7]}},mem_result_b} & {32{!ms_mem_sign}} & {32{!mem_h_or_b}};
  assign mem_result_hs = {{16{mem_result_h[15]}},mem_result_h} & {32{!ms_mem_sign}} & {32{mem_h_or_b}};
  assign mem_result_bu = {24'b0,mem_result_b} & {32{ms_mem_sign}} & {32{!mem_h_or_b}};
  assign mem_result_hu = {16'b0,mem_result_h} & {32{ms_mem_sign}} & {32{mem_h_or_b}};

  assign mem_result_choose = (ms_mem_size != 2'b11) ? 1'b1 : 1'b0;            //不是w时为1
  assign final_result = ms_res_from_mem ? (mem_result_choose ? (mem_result_bs | mem_result_hs | mem_result_bu | mem_result_hu): mem_result) : ms_alu_result;

  assign ms_to_ws_bus={
           ms_gr_we,           //1 0
           ms_dest,            //5 6
           final_result,       //32 38
           ms_pc,              //32 70
           ms_adef,             //1 71
           ms_ine,              //1 72
           ms_sys,              //1 73
           ms_ale,              //1 74
           ms_csr_re,           //1 75
           ms_csr_we,           //1 76
           ms_csr_num,          //14 90
           ms_csr_wmask,        //32 122
           ms_csr_wvalue ,       //32 154
           ms_ertn,              //1 155
           ms_res_from_csr,      //1 156
           ms_break,              //1 157
           ms_rdcnt_detail,      //2 159
           ms_mem_addr    //32 191
         };

  always @(posedge clk)
  begin
    if(reset || ws_ertn_flush)
    begin
      ms_valid <= 1'b0;
      es_to_ms_bus_r <= `ES_TO_MS_BUS_WD'b0;
    end
    else if(ms_allowin)
    begin
      ms_valid <= es_to_ms_valid;
    end
    if(es_to_ms_valid && ms_allowin)
    begin
      es_to_ms_bus_r <= es_to_ms_bus;
    end
  end


endmodule
