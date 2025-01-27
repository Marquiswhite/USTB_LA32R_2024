
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
`include"mycpu_head.v"

module WB_stage(
    input  wire clk,
    input  wire reset,
    output wire ws_allowin,
    input  wire ms_to_ws_valid,
    input  wire [`MS_TO_WS_BUS_WD - 1:0] ms_to_ws_bus,
    output wire [`WS_TO_RF_BUS_WD  - 1:0] ws_to_rf_bus,
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata,
    output wire [`WS_TO_DS_BUS_WD - 1:0] ws_to_ds_bus,
    output wire ws_ertn_flush,
    output wire [31:0] ertn_pc
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

  //前递-定义
  wire [31:0] ws_reg_data;
  wire [ 4:0] ws_reg_dest;
  wire        ws_reg_able;        //记录结果是否出来了

  //例外和异常
  wire ws_adef;
  wire ws_ine;
  wire ws_sys;
  wire ws_ale;
  wire ws_csr_we;
  wire ws_csr_re;
  wire [13:0] ws_csr_num;
  wire [31:0] ws_csr_wmask;
  wire [31:0] ws_csr_wvalue;
  wire [31:0] ws_csr_rvalue;
  wire ws_ertn;
  wire ws_res_from_csr;
  wire ws_ex;
  wire [5:0] ws_ecode;
  wire ws_esubcode;
  wire [31:0] ws_csr_era_pc;
  wire [31:0] ws_csr_eentry_pc;
  wire ws_break;
  wire [1:0] ws_rdcnt_detail;
  wire [63:0] ws_timer64;
  wire [31:0] ws_tid;
  wire ws_has_int;
  wire [31:0] ws_mem_addr;
  wire ws_badv_we;
  wire [31:0] ws_addr;

  assign ws_ertn_flush = (ws_ertn | ws_ex) & ws_valid | ws_has_int;
  assign ws_ex = ws_sys | ws_adef | ws_ale | ws_ine | ws_break | ws_has_int;
  assign ertn_pc = (ws_sys | ws_adef | ws_ale | ws_ine | ws_break | ws_has_int) ? ws_csr_eentry_pc : ws_csr_era_pc;
  assign ws_badv_we = ws_adef | ws_ale;
assign ws_addr = ws_adef ? ws_pc : ws_mem_addr;

  assign ws_ecode = ws_adef ? 6'h08 :
         ws_sys ? 6'h0b :
         ws_break ? 6'h0c :
         ws_ine ? 6'h0d :
         ws_ale ? 6'h09 : 6'h00;
  assign ws_esubcode = 1'b0;

  //前递-实现
  assign ws_reg_able = 1'b1;
  assign ws_reg_dest = ws_dest & {5{ws_valid}};
  assign ws_reg_data =  ws_rdcnt_detail == 2'b11 ? ws_tid:
  ws_rdcnt_detail == 2'b10 ? ws_timer64[63:32] :
  ws_rdcnt_detail == 2'b01 ? ws_timer64[31:0] :
  ws_res_from_csr ? ws_csr_rvalue : ws_final_result;
  assign ws_to_ds_bus = { ws_reg_able,
                          ws_reg_dest,
                          ws_reg_data,
                          ws_csr_we & ws_valid,
                          (ws_csr_re | ws_csr_we) & ws_valid
                        };

  assign ws_ready_go = 1'b1;
  assign ws_allowin = !ws_valid || ws_ready_go;

  assign {
      ws_gr_we,           //1 1
      ws_dest,            //5 6
      ws_final_result,    //32 38
      ws_pc,              //32 70
      ws_adef,            //1 71
      ws_ine,             //1 72
      ws_sys,             //1 73
      ws_ale,             //1 74
      ws_csr_re,          //1 75
      ws_csr_we,          //1 76
      ws_csr_num,         //14 90
      ws_csr_wmask,       //32 122
      ws_csr_wvalue ,     //32 154
      ws_ertn,             //1 155
      ws_res_from_csr,     //1 156
      ws_break  ,           //1 157
      ws_rdcnt_detail,       //2 159
      ws_mem_addr           //32 191
    }=ms_to_ws_bus_r;

  assign rf_we    = ws_gr_we && ws_valid && ~(ws_has_int | ws_ex | ws_ertn);
  assign rf_waddr = ws_dest;
  assign rf_wdata = ws_rdcnt_detail == 2'b11 ? ws_tid:
  ws_rdcnt_detail == 2'b10 ? ws_timer64[63:32] :
  ws_rdcnt_detail == 2'b01 ? ws_timer64[31:0] :
  ws_res_from_csr ? ws_csr_rvalue : ws_final_result;

  assign ws_to_rf_bus={
           rf_we,
           rf_waddr,
           rf_wdata
         };

  assign debug_wb_pc       = ws_pc;
  assign debug_wb_rf_we   = {4{rf_we}};
  assign debug_wb_rf_wnum  = ws_dest;
  assign debug_wb_rf_wdata = rf_wdata;

  always @(posedge clk)
  begin
    if(reset || ws_ertn_flush)
    begin
      ws_valid <= 1'b0;
    end
    else if(ws_allowin)
    begin
      ws_valid <= ms_to_ws_valid;
    end
    if(ms_to_ws_valid && ws_allowin)
    begin
      ms_to_ws_bus_r <= ms_to_ws_bus;
    end
  end



  regcsr u_regcsr(
           .clk(clk),
           .reset(reset),
           .csr_re(ws_csr_re),
           .csr_num(ws_csr_num),
           .csr_rvalue(ws_csr_rvalue),
           .csr_we(ws_csr_we),
           .csr_wmask(ws_csr_wmask),
           .csr_wvalue(ws_csr_wvalue),
           .wb_ex(ws_ex),
           .ertn_flush(ws_ertn),
           .wb_ecode(ws_ecode),
           .wb_esubcode({8'b0,ws_esubcode}),
           .ws_pc(ws_pc),
           .has_int(ws_has_int),
           .ertn_csr_era_pc(ws_csr_era_pc),
           .ertn_csr_eentry_pc(ws_csr_eentry_pc),
           .csr_tid(ws_tid),
           .csr_timer64(ws_timer64),
           .wb_addr(ws_addr),
           .wb_badv_we(ws_badv_we)
         );


endmodule
