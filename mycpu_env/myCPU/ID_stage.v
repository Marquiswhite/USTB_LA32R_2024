
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
//////////////////////////////////////////////////////////////////////////////////`timescale 1ns / 1ps
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

//译码阶段的主要功能是解析指令生成控制信号并读取通用寄存器堆生成源操作数
`include "mycpu_head.v"

module ID_stage(
    input wire clk,
    input wire reset,
    //allowin
    input wire es_allowin,
    output wire ds_allowin,
    //from if stage
    input wire fs_to_ds_valid,
    input wire [`FS_TO_DS_BUS_WD-1 : 0] fs_to_ds_bus,
    //to ex stage
    output wire ds_to_es_valid,
    output wire [`DS_TO_ES_BUS_WD -1 : 0 ] ds_to_es_bus,
    //to if stage
    output wire [`BR_BUS_WD -1 : 0 ] br_bus ,
    //to rf :write back
    input wire [`WS_TO_RF_BUS_WD -1 :0] ws_to_rf_bus,
    //从寄存器堆中读取源操作数。
    input wire [`ES_TO_DS_BUS_WD - 1:0] es_to_ds_bus,
    input wire [`MS_TO_DS_BUS_WD - 1:0] ms_to_ds_bus,
    input wire [`WS_TO_DS_BUS_WD - 1:0] ws_to_ds_bus,
    input wire ws_ertn_flush
  );

  wire br_taken;
  wire [31:0] br_target;

  wire [31:0] ds_pc;
  wire [31:0] ds_inst;
  // ID 阶段握手信号
  reg ds_valid;
  wire ds_ready_go;

  wire [18:0] alu_op;

  wire load_op; //加 载 信 号，这是之前给的注释，但是这个信号一直没用到，不理解
  wire src1_is_pc; //来 自src为pc地 址
  wire src2_is_imm; //src2为imm
  wire res_from_mem; //result来 自mem
  wire dst_is_r1;
  wire dst_is_rj;
  wire gr_we;
  wire mem_we;
  wire src_reg_is_rd;
  wire [4:0]dest;
  wire rj_eq_rd;
  wire [31:0]rj_value;
  wire [31:0]rkd_value;
  wire [31:0]imm;
  wire [31:0]br_offs;
  wire [31:0]jirl_offs;

  //分 段
  wire [5:0] op_31_26;
  wire [3:0] op_25_22;
  wire [1:0] op_21_20;
  wire [4:0] op_19_15;

  wire [4:0] rd;
  wire [4:0] rj;
  wire [4:0] rk;

  wire [11:0] i12;
  wire [19:0] i20;
  wire [15:0] i16;
  wire [25:0] i26;

  wire [63:0] op_31_26_d;
  wire [15:0] op_25_22_d;
  wire [ 3:0] op_21_20_d;
  wire [31:0] op_19_15_d;

  wire inst_add_w;
  wire inst_sub_w;
  wire inst_slt;
  wire inst_sltu;
  wire inst_nor;
  wire inst_and;
  wire inst_or;
  wire inst_xor;
  wire inst_slli_w;
  wire inst_srli_w;
  wire inst_srai_w;
  wire inst_addi_w;
  wire inst_ld_w;
  wire inst_st_w;
  wire inst_jirl;
  wire inst_b;
  wire inst_bl;
  wire inst_beq;
  wire inst_bne;
  wire inst_lu12i_w;

  //算术逻辑转移类指令的添加
  wire inst_slti;
  wire inst_sltui;
  wire inst_andi;
  wire inst_ori;
  wire inst_xori;
  wire inst_sll_w;
  wire inst_srl_w;
  wire inst_sra_w;
  wire inst_pcaddu12i;
  wire inst_blt;
  wire inst_bge;
  wire inst_bltu;
  wire inst_bgeu;

  //访存指令的添加
  wire inst_ld_b;
  wire inst_ld_h;
  wire inst_ld_bu;
  wire inst_ld_hu;
  wire inst_st_b;
  wire inst_st_h;

  //乘除法指令的添加
  wire inst_mul_w;
  wire inst_mulh_w;
  wire inst_mulh_wu;
  wire inst_div_w;
  wire inst_mod_w;
  wire inst_div_wu;
  wire inst_mod_wu;

  //异常和例外的添加
  wire inst_csrrd;
  wire inst_csrwr;
  wire inst_csrxchg;
  wire inst_syscall;
  wire inst_ertn;
  wire inst_break;
  wire inst_rdcntvl_w;
  wire inst_rdcntvh_w;
  wire inst_rdcntid_w;

  //仍然是异常和例外
  wire [13:0] ds_csr_num;
  wire ds_csr_re;
  wire ds_csr_we;
  wire [31:0] ds_csr_wmask;
  wire [31:0] ds_csr_wvalue;
  wire ds_adef;
  wire ds_ine;
  wire ds_sys;
  wire ds_ertn;
  wire ds_res_from_csr;
  wire ds_csr_en;
  wire ds_es_csr_we;
  wire ds_ms_csr_we;
  wire ds_ws_csr_we;
  wire ds_csr_delay;
  wire ds_es_csr_rw;
  wire ds_ms_csr_rw;
  wire ds_ws_csr_rw;
  wire ds_break;
  wire [1:0] ds_rdcnt_detail;

  //仍然是算术逻辑转移类指令的添加
  wire need_ui12;
  wire rj_greater_rd_signed;     //视为有符号整数比较
  wire rj_smaller_rd_signed;     //视为有符号整数比较
  wire rj_greater_rd_unsigned;   //视为无符号整数比较
  wire rj_smaller_rd_unsigned;   //视为无符号整数比较

  //仍然是访存指令的添加
  wire [1:0] mem_size;           //要访问的内存的大小，01表示b，10表示h，11表示w
  wire       mem_sign;           //0表示符号拓展，1表示零拓展

  //仍然是乘除法指令的添加
  wire need_ui5;
  wire need_si12;
  wire need_si16;
  wire need_si20;
  wire need_si26;
  wire src2_is_4;

  wire [ 4:0] rf_raddr1;
  wire [31:0] rf_rdata1;
  wire [ 4:0] rf_raddr2;
  wire [31:0] rf_rdata2;

  wire rf_we ; //写 使 能
  wire [4:0] rf_waddr;
  wire [31:0] rf_wdata;

  //接受es阶段的数据，用于前递
  wire        inst_reg_nw;
  wire        inst_rj;
  wire        inst_rd;
  wire        inst_rk;
  wire        rj_es_forward;
  wire        rk_es_forward;
  wire        rd_es_forward;
  wire        ds_es_reg_able;
  wire [31:0] ds_es_reg_data;
  wire [ 4:0] ds_es_reg_dest;
  wire        rkd_es_forward;

  //接受ms阶段的数据，用于前递
  wire        rj_ms_forward;
  wire        rk_ms_forward;
  wire        rd_ms_forward;
  wire        ds_ms_reg_able;
  wire [31:0] ds_ms_reg_data;
  wire [ 4:0] ds_ms_reg_dest;
  wire        rkd_ms_forward;

  //接受WS阶段的数据，用于前递
  wire        rj_ws_forward;
  wire        rk_ws_forward;
  wire        rd_ws_forward;
  wire        ds_ws_reg_able;
  wire [31:0] ds_ws_reg_data;
  wire [ 4:0] ds_ws_reg_dest;
  wire        rkd_ws_forward;

  //用于处理EXE/MEM/WB阶段需要前递的结果还没有计算出来的情况-定义
  reg        ds_stall;

  //接受es阶段的数据，用于前递
  assign { ds_es_reg_able,     //这个变量命名有问题，实际上是结果是否来自于MEM阶段
           ds_es_reg_dest,
           ds_es_reg_data,
           ds_es_csr_we ,      //ES阶段是否要写CSR寄存器
           ds_es_csr_rw
         } = es_to_ds_bus;

  //接受ms阶段的数据，用于前递
  assign { ds_ms_reg_able,
           ds_ms_reg_dest,
           ds_ms_reg_data,
           ds_ms_csr_we,
           ds_ms_csr_rw
         } = ms_to_ds_bus;

  //接受WS阶段的数据，用于前递
  assign { ds_ws_reg_able,
           ds_ws_reg_dest,
           ds_ws_reg_data,
           ds_ws_csr_we,
           ds_ws_csr_rw
         } = ws_to_ds_bus;

  /////////////////////////////////
  assign inst_reg_nw = inst_st_w  | inst_b      | inst_beq  | inst_bne  | inst_blt | inst_bge  | inst_bltu | inst_bgeu  | inst_st_b   | inst_st_h   | inst_syscall | inst_ertn | inst_break;      //这里显然是没有把PC看作特殊寄存器的
  assign inst_rd     = inst_beq   | inst_bne    | inst_st_w | inst_blt  | inst_bge | inst_bltu | inst_bgeu | inst_st_b  | inst_st_h   | inst_csrwr  | inst_csrxchg | inst_rdcntvh_w | inst_rdcntvl_w;
  assign inst_rj     = inst_add_w | inst_addi_w | inst_slt  | inst_sltu | inst_and | inst_or   | inst_nor  | inst_xor   | inst_slli_w | inst_srli_w | inst_srai_w | inst_beq   | inst_bne    | inst_sub_w   | inst_ld_w | inst_st_w | inst_jirl | inst_slti | inst_sltui | inst_andi | inst_ori | inst_xori | inst_sll_w | inst_srl_w | inst_sra_w | inst_blt | inst_bge | inst_bltu | inst_bgeu | inst_ld_b |inst_ld_h | inst_ld_bu | inst_ld_hu | inst_st_b | inst_st_h | inst_mul_w | inst_mulh_w | inst_mulh_wu | inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu | inst_csrxchg | inst_rdcntid_w;
  assign inst_rk     = inst_add_w | inst_slt    | inst_sltu | inst_and  | inst_or  | inst_nor  | inst_xor  | inst_sub_w | inst_sll_w  | inst_srl_w  | inst_sra_w  | inst_mul_w | inst_mulh_w | inst_mulh_wu | inst_div_w | inst_mod_w | inst_div_wu | inst_mod_wu;

  assign rj_es_forward = (inst_rj && rj != 5'b00000) && (rj == ds_es_reg_dest);
  assign rk_es_forward = (inst_rk && rk != 5'b00000) && (rk == ds_es_reg_dest);
  assign rd_es_forward = (inst_rd && rd != 5'b00000) && (rd == ds_es_reg_dest);
  assign rkd_es_forward = src_reg_is_rd ? rd_es_forward : rk_es_forward;

  assign rj_ms_forward = (inst_rj && rj != 5'b00000) && (rj == ds_ms_reg_dest);
  assign rk_ms_forward = (inst_rk && rk != 5'b00000) && (rk == ds_ms_reg_dest);
  assign rd_ms_forward = (inst_rd && rd != 5'b00000) && (rd == ds_ms_reg_dest);
  assign rkd_ms_forward = src_reg_is_rd ? rd_ms_forward : rk_ms_forward;

  assign rj_ws_forward = (inst_rj && rj != 5'b00000) && (rj == ds_ws_reg_dest);
  assign rk_ws_forward = (inst_rk && rk != 5'b00000) && (rk == ds_ws_reg_dest);
  assign rd_ws_forward = (inst_rd && rd != 5'b00000) && (rd == ds_ws_reg_dest);
  assign rkd_ws_forward = src_reg_is_rd ? rd_ws_forward : rk_ws_forward;

  always@(ds_es_reg_able, ds_ms_reg_able, ds_ws_reg_able, rj_es_forward, rkd_es_forward, rj_ms_forward, rkd_ms_forward, rj_ws_forward, rkd_ws_forward)
  begin
    if(ds_es_reg_able == 1'b0 && (rj_es_forward || rkd_es_forward)
        || ds_ms_reg_able == 1'b0 && (rj_ms_forward || rkd_ms_forward)
        || ds_ws_reg_able == 1'b0 && (rj_ws_forward || rkd_ws_forward))
    begin
      ds_stall = 1'b1;
    end
    else
    begin
      ds_stall = 1'b0;
    end
  end

  /////////////////////////////////

  //处理新增加的转移指令
  /*
  assign rj_greater_rd_signed = (!rj_value[31] & rkd_value[31] ||
                                (!rj_value[31] & !rkd_value[31]) && (rj_value > rkd_value) ||
                                (rj_value[31]  & rkd_value[31] ) && (rj_value < rkd_value));
  assign rj_smaller_rd_signed = !rj_greater_rd_signed & !rj_eq_rd;
  */
  assign rj_greater_rd_signed = (rj_value[31] ~^ rkd_value[31]) && ( rj_value > rkd_value) ||
         (!rj_value[31] & rkd_value[31]);
  assign rj_smaller_rd_signed = !rj_greater_rd_signed & !rj_eq_rd;
  assign rj_greater_rd_unsigned = rj_value > rkd_value;
  assign rj_smaller_rd_unsigned = rj_value < rkd_value;

  /////////////////////////////////
  //处理访存指令
  assign mem_size = (inst_st_w | inst_ld_w) ? 2'b11 :
         ((inst_st_h | inst_ld_h | inst_ld_hu) ? 2'b10 :
          ((inst_st_b | inst_ld_b | inst_ld_bu) ? 2'b01 : 2'b00));
  assign mem_sign = (inst_ld_bu | inst_ld_hu) ? 1'b1 : 1'b0;  //无符号是1

  /////////////////////////////////
  assign ds_ine = ~(inst_add_w | inst_sub_w | inst_slt | inst_sltu |
                    inst_nor | inst_and | inst_or | inst_xor |
                    inst_slli_w |inst_srli_w| inst_srai_w | inst_addi_w |
                    inst_ld_w | inst_st_w | inst_jirl | inst_b| inst_bl |
                    inst_beq | inst_bne | inst_lu12i_w | inst_slti |
                    inst_sltui | inst_andi | inst_ori | inst_xori | inst_sll_w |
                    inst_srl_w | inst_sra_w | inst_pcaddu12i |inst_blt| inst_bge |
                    inst_bltu| inst_bgeu| inst_ld_b| inst_ld_bu | inst_ld_h| inst_ld_hu|
                    inst_st_b| inst_st_h| inst_mul_w | inst_mulh_w| inst_mulh_wu|
                    inst_div_w | inst_div_wu| inst_mod_w|inst_mod_wu | inst_csrrd | inst_csrwr | inst_csrxchg |
                    inst_syscall | inst_ertn | inst_rdcntid_w | inst_rdcntvh_w | inst_rdcntvl_w
                   );

  ////////////////////////////////

  assign op_31_26 = ds_inst[31:26];
  assign op_25_22 = ds_inst[25:22];
  assign op_21_20 = ds_inst[21:20];
  assign op_19_15 = ds_inst[19:15];

  assign rd = ds_inst[ 4: 0];
  assign rj = ds_inst[ 9: 5];
  assign rk = ds_inst[14:10];

  assign i12 = ds_inst[21:10];
  assign i20 = ds_inst[24: 5];
  assign i16 = ds_inst[25:10];
  assign i26 = {ds_inst[ 9: 0], ds_inst[25:10]};

  decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
  decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
  decoder_2_4 u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
  decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));

  ///////////////////////////////////
  assign inst_add_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
  assign inst_sub_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
  assign inst_slt = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
  assign inst_sltu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
  assign inst_nor = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
  assign inst_and = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
  assign inst_or = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
  assign inst_xor = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
  assign inst_slli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
  assign inst_srli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
  assign inst_srai_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
  assign inst_addi_w = op_31_26_d[6'h00] & op_25_22_d[4'ha];
  assign inst_ld_w = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
  assign inst_st_w = op_31_26_d[6'h0a] & op_25_22_d[4'h6];
  assign inst_jirl = op_31_26_d[6'h13];
  assign inst_b = op_31_26_d[6'h14];
  assign inst_bl = op_31_26_d[6'h15];
  assign inst_beq = op_31_26_d[6'h16];
  assign inst_bne = op_31_26_d[6'h17];
  assign inst_lu12i_w= op_31_26_d[6'h05] & ~ds_inst[25];

  //算术逻辑转移类指令的添加
  assign inst_slti      = op_31_26_d[6'h00] & op_25_22_d[4'h8];
  assign inst_sltui     = op_31_26_d[6'h00] & op_25_22_d[4'h9];
  assign inst_andi      = op_31_26_d[6'h00] & op_25_22_d[4'hd];
  assign inst_ori       = op_31_26_d[6'h00] & op_25_22_d[4'he];
  assign inst_xori      = op_31_26_d[6'h00] & op_25_22_d[4'hf];
  assign inst_sll_w     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0e];
  assign inst_srl_w     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0f];
  assign inst_sra_w     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h10];
  assign inst_pcaddu12i = op_31_26_d[6'h07] & ~ds_inst[25];
  assign inst_blt       = op_31_26_d[6'h18];
  assign inst_bge       = op_31_26_d[6'h19];
  assign inst_bltu      = op_31_26_d[6'h1a];
  assign inst_bgeu      = op_31_26_d[6'h1b];

  //访存类指令的添加
  assign inst_ld_b  = op_31_26_d[6'h0a] & op_25_22_d[4'h0];
  assign inst_ld_h  = op_31_26_d[6'h0a] & op_25_22_d[4'h1];
  assign inst_ld_bu = op_31_26_d[6'h0a] & op_25_22_d[4'h8];
  assign inst_ld_hu = op_31_26_d[6'h0a] & op_25_22_d[4'h9];
  assign inst_st_b  = op_31_26_d[6'h0a] & op_25_22_d[4'h4];
  assign inst_st_h  = op_31_26_d[6'h0a] & op_25_22_d[4'h5];

  //乘除法指令的添加
  assign inst_mul_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h18];
  assign inst_mulh_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h19];
  assign inst_mulh_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h1a];
  assign inst_div_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h00];
  assign inst_mod_w   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h01];
  assign inst_div_wu  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h02];
  assign inst_mod_wu  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h03];

  //例外和异常
  assign inst_break = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h14];
  assign inst_rdcntid_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00] & (rk == 5'b11000) & (rd == 5'h00);
  assign inst_rdcntvl_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00] & (rk == 5'b11000) & (rj == 5'h00);
  assign inst_rdcntvh_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0] & op_19_15_d[5'h00] & (rk == 5'b11001) & (rj == 5'h00);
  assign inst_csrrd = op_31_26_d[6'h01] & ~ds_inst[25] & ~ds_inst[24] & (rj == 5'b00000);
  assign inst_csrwr = op_31_26_d[6'h01] & ~ds_inst[25] & ~ds_inst[24] & (rj == 5'b00001);
  assign inst_csrxchg = op_31_26_d[6'h01] & ~ds_inst[25] & ~ds_inst[24] & (rj != 5'b00000 && rj != 5'b00001);
  assign inst_syscall = op_31_26_d[6'h00] & op_25_22_d[4'h00] & op_21_20_d[2'h2] & op_19_15_d[5'h16];
  assign inst_ertn = (ds_inst == 32'h6483800);
  assign ds_csr_num = ds_inst[23:10];
  assign ds_csr_re = inst_csrrd | inst_csrxchg | inst_csrwr;
  assign ds_csr_we = inst_csrwr | inst_csrxchg;
  assign ds_csr_wmask = inst_csrwr ? 32'hffffffff : rj_value;
  assign ds_csr_wvalue = rkd_value;
  assign ds_sys = inst_syscall;
  assign ds_ertn = inst_ertn | inst_syscall | inst_break;
  assign ds_res_from_csr = inst_csrrd | inst_csrxchg | inst_csrwr;
  assign ds_csr_en = ds_csr_re | ds_csr_we;
  assign ds_csr_delay = (ds_csr_we |ds_csr_re) && (ds_es_csr_we | ds_ms_csr_we | ds_ws_csr_we) ||     //csr指令之间的delay
         ((inst_rj && rj != 5'b00000) && (ds_es_csr_rw && (rj == ds_es_reg_dest) || ds_ms_csr_rw && (rj == ds_ms_reg_dest) || ds_ws_csr_rw && (rj == ds_ws_reg_dest))) ||
         ((inst_rk && rk != 5'b00000) && (ds_es_csr_rw && (rk == ds_es_reg_dest) || ds_ms_csr_rw && (rk == ds_ms_reg_dest) || ds_ws_csr_rw && (rk == ds_ws_reg_dest))) ||
         ((inst_rd && rd != 5'b00000) && (ds_es_csr_rw && (rd == ds_es_reg_dest) || ds_ms_csr_rw && (rd == ds_ms_reg_dest) || ds_ws_csr_rw && (rd == ds_ws_reg_dest)));
  //id要读es阶段的csr的目标寄存器
  assign ds_break = inst_break;
  assign ds_rdcnt_detail = inst_rdcntid_w ? 2'b11:
         inst_rdcntvh_w ? 2'b10:
         inst_rdcntvl_w ? 2'b01 : 2'b00;
  //assign ds_res_from_timer = inst_rdcntid_w | inst_rdcntvh_w | inst_rdcntvl_w;

  /////////////////////////////////
  assign alu_op[ 0] = inst_add_w  | inst_addi_w | inst_ld_w | inst_st_w | inst_jirl | inst_bl | inst_pcaddu12i | inst_st_b | inst_st_h | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu;
  assign alu_op[ 1] = inst_sub_w;
  assign alu_op[ 2] = inst_slt    | inst_slti;
  assign alu_op[ 3] = inst_sltu   | inst_sltui;
  assign alu_op[ 4] = inst_and    | inst_andi;
  assign alu_op[ 5] = inst_nor;
  assign alu_op[ 6] = inst_or     | inst_ori;
  assign alu_op[ 7] = inst_xor    | inst_xori;
  assign alu_op[ 8] = inst_slli_w | inst_sll_w;
  assign alu_op[ 9] = inst_srli_w | inst_srl_w;
  assign alu_op[10] = inst_srai_w | inst_sra_w;
  assign alu_op[11] = inst_lu12i_w;
  assign alu_op[12] = inst_mul_w;
  assign alu_op[13] = inst_mulh_w;
  assign alu_op[14] = inst_mulh_wu;
  assign alu_op[15] = inst_div_w;
  assign alu_op[16] = inst_mod_w;
  assign alu_op[17] = inst_div_wu;
  assign alu_op[18] = inst_mod_wu;

  //信 号
  assign need_ui5  = inst_slli_w  | inst_srli_w | inst_srai_w;
  assign need_si12 = inst_addi_w  | inst_ld_w   | inst_st_w   | inst_slti | inst_sltui | inst_st_b | inst_st_h | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu;
  assign need_si16 = inst_jirl    | inst_beq    | inst_bne    | inst_blt  | inst_bge   | inst_bltu | inst_bgeu;
  assign need_si20 = inst_lu12i_w | inst_pcaddu12i;
  assign need_si26 = inst_b       | inst_bl;
  assign src2_is_4 = inst_jirl    | inst_bl;
  assign need_ui12 = inst_andi    | inst_ori    | inst_xori;

  assign imm = src2_is_4 ? 32'h4 :
         need_si20 ? {i20[19:0], 12'b0} :
         need_ui12 ? {20'b0, i12[11:0]}:
         need_ui5  ? rk : {32{need_si12}} & {{20{i12[11]}}, i12[11:0]} ;  //这里没有使用need_si12，但是实现了指令，出现的是新增加的

  assign br_offs = need_si26 ? {{ 4{i26[25]}}, i26[25:0], 2'b0} :
         {{14{i16[15]}}, i16[15:0], 2'b0} ;         //这里没用need_si16，但是暗示了后面就是si16

  assign jirl_offs = {{14{i16[15]}}, i16[15:0], 2'b0};

  assign src_reg_is_rd = inst_beq | inst_bne | inst_st_w | inst_blt | inst_bge | inst_bltu | inst_bgeu | inst_st_b | inst_st_h | inst_csrwr | inst_csrrd | inst_csrxchg;

  assign src1_is_pc = inst_jirl | inst_bl | inst_pcaddu12i;

  assign src2_is_imm = inst_slli_w   |
         inst_srli_w   |
         inst_srai_w   |
         inst_addi_w   |
         inst_ld_w     |
         inst_st_w     |
         inst_lu12i_w  |
         inst_jirl     |
         inst_bl       |
         inst_slti     |
         inst_sltui    |
         inst_andi     |
         inst_ori      |
         inst_xori     |
         inst_pcaddu12i|
         inst_st_b     |
         inst_st_h     |
         inst_ld_b     |
         inst_ld_h     |
         inst_ld_bu    |
         inst_ld_hu;

  assign res_from_mem = inst_ld_w | inst_ld_b | inst_ld_h | inst_ld_bu | inst_ld_hu;
  assign dst_is_r1 = inst_bl;
  assign dst_is_rj = inst_rdcntid_w;
  assign gr_we = ~inst_st_w & ~inst_beq & ~inst_bne & ~inst_b & ~inst_blt & ~inst_bge & ~inst_bltu & ~inst_bgeu & ~inst_st_b & ~inst_st_h & ~inst_syscall & ~inst_ertn;
  assign mem_we = inst_st_w | inst_st_b | inst_st_h;
  assign dest = dst_is_r1   ? 5'h01 :
         dst_is_rj ? rj :
         inst_reg_nw ? 5'b0 : rd;

  assign rf_raddr1 = rj;
  assign rf_raddr2 = src_reg_is_rd ? rd :rk;

  regfile u_regfile(
            .clk (clk ),
            .raddr1 (rf_raddr1),
            .rdata1 (rf_rdata1),
            .raddr2 (rf_raddr2),
            .rdata2 (rf_rdata2),
            .we (rf_we ),
            .waddr (rf_waddr ),
            .wdata (rf_wdata )
          );

  assign rj_value  = rj_es_forward  ? ds_es_reg_data:(rj_ms_forward  ? ds_ms_reg_data : ( rj_ws_forward ? ds_ws_reg_data : rf_rdata1));
  assign rkd_value = rkd_es_forward ? ds_es_reg_data:(rkd_ms_forward ? ds_ms_reg_data : (rkd_ws_forward ? ds_ws_reg_data : rf_rdata2));

  assign rj_eq_rd = (rj_value == rkd_value);
  assign br_taken = ( inst_beq  && rj_eq_rd
                      || inst_bne  && !rj_eq_rd
                      || inst_jirl
                      || inst_bl
                      || inst_b
                      || inst_blt  && (rj_smaller_rd_signed  )
                      || inst_bge  && (rj_greater_rd_signed   || rj_eq_rd)
                      || inst_bltu && (rj_smaller_rd_unsigned)
                      || inst_bgeu && (rj_greater_rd_unsigned || rj_eq_rd)) && ds_valid;
  assign br_target = (inst_beq || inst_bne || inst_bl || inst_b || inst_blt || inst_bge || inst_bltu || inst_bgeu) ? (ds_pc + br_offs) :
         /*inst_jirl*/ (rj_value + jirl_offs);



  //ID->IF的br_bus
  assign br_bus = {br_taken, br_target};

  reg [`FS_TO_DS_BUS_WD-1 : 0] fs_to_ds_bus_r;

  assign {ds_inst,
          ds_pc,
          ds_adef
         } = fs_to_ds_bus_r;

  assign {
      rf_we, //37:37
      rf_waddr, //36:32
      rf_wdata //31:0
    } = ws_to_rf_bus;

  assign ds_to_es_bus = {alu_op ,                       //19 19
                         load_op ,                      //1 20
                         src1_is_pc ,                   //1 21
                         src2_is_imm ,                  //1 22
                         src2_is_4 ,                    //1 23
                         gr_we ,                        //1 24
                         mem_we ,                       //1 25
                         dest ,                         //5 30
                         imm ,                          //32 62
                         rj_value ,                     //32 94
                         rkd_value ,                    //32 126
                         ds_pc ,                        //32 158
                         res_from_mem ,                 //1 159
                         mem_size ,                     //2 161
                         mem_sign,                       //1 162
                         ds_adef,                       //1 163
                         ds_ine ,                        //1 164
                         ds_csr_re,                  //1 165
                         ds_csr_we,                         //1 166
                         ds_csr_num,                 //14 180
                         ds_csr_wmask,                       //32 212
                         ds_csr_wvalue,                      //32 244
                         ds_sys,                 //1 245
                         ds_ertn,                //1 246
                         ds_res_from_csr,             //1 247
                         ds_break,                  //1 248
                         ds_rdcnt_detail          //2 250
                        };


  //////////////////////////////////////////////////
  assign ds_ready_go = (ds_stall || ds_csr_delay) ? 1'b0 : 1'b1;
  assign ds_allowin = !ds_valid || ds_ready_go && es_allowin;
  assign ds_to_es_valid = ds_valid && ds_ready_go;
  always @(posedge clk)
  begin
    if (reset || ws_ertn_flush)
    begin
      ds_valid <= 1'b0;
    end
    else if (ds_allowin)
    begin
      ds_valid <= fs_to_ds_valid;
    end

    if (fs_to_ds_valid && ds_allowin && ~ws_ertn_flush)
    begin
      fs_to_ds_bus_r <= fs_to_ds_bus;
    end else if(ws_ertn_flush)begin
      fs_to_ds_bus_r <= `FS_TO_DS_BUS_WD'b0;
    end
  end

endmodule
