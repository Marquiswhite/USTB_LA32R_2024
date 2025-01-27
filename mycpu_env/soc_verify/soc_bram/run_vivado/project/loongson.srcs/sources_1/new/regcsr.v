`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2024/11/19 21:31:19
// Design Name:
// Module Name: regcsr
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

module regcsr(
    input  wire clk,
    input  wire reset,
    input  wire csr_re,       //读使能
    input  wire [13:0] csr_num,     //寄存器号
    output wire [31:0] csr_rvalue,       //寄存器读返回值
    input  wire csr_we,     //写使能
    input  wire [31:0] csr_wmask,  //写掩码
    input  wire [31:0] csr_wvalue,    //写数据
    input  wire wb_ex ,      //写回级的异常处理触发信号
    input  wire ertn_flush,
    input  wire [5:0] wb_ecode,
    input  wire [8:0] wb_esubcode,
    input  wire [31:0] ws_pc,
    output wire has_int,
    output wire [31:0] ertn_csr_era_pc,
    output wire [31:0] ertn_csr_eentry_pc,
    output wire [63:0] csr_timer64,
    output wire [31:0] csr_tid,
    input  wire [31:0] wb_addr,
    input  wire wb_badv_we
  );

  reg [63:0] timer64;
  assign csr_timer64 = timer64;
  always @(posedge clk )
  begin
    if(reset)
    begin
      timer64 <= 64'b0;
    end
    else
    begin
      timer64 <= timer64 + 1'b1;
    end
  end

  //crmd 当前模式信息
  reg  [ 1:0] csr_crmd_plv;    //当前特权等级
  reg        csr_crmd_ie;      //当前全局中断使能
  reg        csr_crmd_da;      //直接地址翻译模式的使能
  reg        csr_crmd_pg;      //映射地址模式的使能
  reg  [ 1:0] csr_crmd_datf;   //直接地址翻译模式时取指操作的存储访问类型
  reg  [ 1:0] csr_crmd_datm;   //直接地址翻译模式时，load和store操作的存储访问类型
  wire [31:0] csr_crmd_rvalue; //读寄存器的值

  //prmd 例外前模式信息
  reg [1:0] csr_prmd_pplv;    //触发例外时，记录csr_crmd_plv的旧值
  reg       csr_prmd_pie;     //触发例外时，记录csr_crmd_ie的旧值
  wire [31:0] csr_prmd_rvalue;

  //ecfg 例外控制
  reg [12:0] csr_ecfg_lie;    //局部中断使能位
  wire [31:0] csr_ecfg_rvalue;

  //estat 例外状态
  reg [12:0] csr_estat_is;    //中断状态位
  reg [ 5:0] csr_estat_ecode; //例外类型一级编码
  reg [ 8:0] csr_estat_esubcode;//例外类型二级编码
  wire [31:0] csr_estat_rvalue;

  //era 例外返回地址
  reg [31:0] csr_era_pc;      //记录触发例外的指令的PC
  wire [31:0] csr_era_rvalue;

  //badv 出错虚地址
  reg [31:0] csr_badv_vaddr;  //触发TLB重填例外和地址错误相关例外时，记录出错的虚地址
  wire [31:0] csr_badv_rvalue;

  //eentry 例外入口地址
  reg [25:0] csr_eentry_va;   //例外和总段入口地址的[31:6]位，低6位必须为0
  wire [31:0] csr_eentry_rvalue;

  //save 数据保存
  reg [31:0] csr_save0_data;  //仅供软件读写的数据，除csr指令外，硬件不会修改其内容
  wire [31:0] csr_save0_rvalue;

  reg [31:0] csr_save1_data;
  wire [31:0] csr_save1_rvalue;

  reg [31:0] csr_save2_data;
  wire [31:0] csr_save2_rvalue;

  reg [31:0] csr_save3_data;
  wire [31:0] csr_save3_rvalue;

  //tid 定时器编号
  reg [31:0] csr_tid_tid;     //定时器编号
  wire [31:0] csr_tid_rvalue;

  //tcfg 定时器配置
  reg        csr_tcfg_en;     //定时器使能位
  reg        csr_tcfg_periodic;//定时器循环模式控制位
  reg [31:2] csr_tcfg_initval;//定时器倒计时自减计数的初始值
  wire [31:0] csr_tcfg_rvalue;

  //tval 定时器数值
  reg [31:0] csr_tval_timeval;//当前计数器的计数值
  wire [31:0] tcfg_next_value;
  wire [31:0] csr_tval_rvalue;        //示例中的csr_tval

  //ticlr 定是中断清楚
  //reg        csr_ticlr_clr;   //当该位为1时，将清楚时钟中断标记，读出结果宗伟0
  wire [31:0] csr_ticlr_rvalue;

  //////////////////////////    CRMD 当前状态信息     /////////////////////////////////////
  assign csr_crmd_rvalue = {23'b0,
                            csr_crmd_datm,
                            csr_crmd_datf,
                            csr_crmd_pg,
                            csr_crmd_da,
                            csr_crmd_ie,
                            csr_crmd_plv};
  /////////////////////////    PLV 当前特权等级 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_crmd_plv <= 2'b0;
    end
    else if(wb_ex)
    begin
      csr_crmd_plv <= 2'b0;
    end
    else if(ertn_flush)
    begin
      csr_crmd_plv <= csr_prmd_pplv;
    end
    else if(csr_we & csr_num == `CSR_CRMD)
    begin
      csr_crmd_plv <=  csr_wmask[`CSR_CRMD_PLV] & csr_wvalue[`CSR_CRMD_PLV]
                   | ~csr_wmask[`CSR_CRMD_PLV] & csr_crmd_plv;
    end
  end

  /////////////////////////    IE 当前全局中断使能 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_crmd_ie <= 1'b0;
    end
    else if(wb_ex)
    begin
      csr_crmd_ie <= 1'b0;
    end
    else if(ertn_flush)
    begin
      csr_crmd_ie <= csr_prmd_pie;
    end
    else if(csr_we & csr_num == `CSR_CRMD)
    begin
      csr_crmd_ie <=  csr_wmask[`CSR_CRMD_IE] & csr_wvalue[`CSR_CRMD_IE]
                  | ~csr_wmask[`CSR_CRMD_IE] & csr_crmd_ie;
    end
  end

  /////////////////////////    DA 直接地址翻译模式的使能 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_crmd_da <= 1'b1;
    end
    else if(ertn_flush && csr_estat_ecode == 6'h3f)
    begin
      csr_crmd_da <= 1'b0;
    end
    else if(csr_we & csr_num == `CSR_CRMD)
    begin
      csr_crmd_da <= csr_wmask[`CSR_CRMD_DA] & csr_wvalue[`CSR_CRMD_DA]
                  | ~csr_wmask[`CSR_CRMD_DA] & csr_crmd_da;
    end
  end

  /////////////////////////    PG 映射地址翻译模式的使能 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_crmd_pg <= 1'b0;
    end
    else if(ertn_flush && csr_estat_ecode == 6'h3f)
    begin
      csr_crmd_pg <= 1'b1;
    end
    else if(csr_we & csr_num == `CSR_CRMD)
    begin
      csr_crmd_pg <= csr_wmask[`CSR_CRMD_PG] & csr_wvalue[`CSR_CRMD_PG]
                  | ~csr_wmask[`CSR_CRMD_PG] & csr_crmd_pg;
    end
  end

  /////////////////////////    DATF 直接地址翻译模式时，取指操作的存储访问类型 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_crmd_datf <= 2'b00;
    end
    else if(csr_crmd_da == 1'b1 && csr_crmd_pg == 1'b1)
    begin
      csr_crmd_datf <= 2'b01;
    end
    else if(csr_we & csr_num == `CSR_CRMD)
    begin
      csr_crmd_datf <= csr_wmask[`CSR_CRMD_DATF] & csr_wvalue[`CSR_CRMD_DATF]
                    | ~csr_wmask[`CSR_CRMD_DATF] & csr_crmd_datf;
    end
  end


  /////////////////////////    DATM 直接地址翻译模式时，load 和 store 操作的存储访问类型 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_crmd_datm <= 2'b0;
    end
    else if(csr_crmd_da == 1'b1 && csr_crmd_pg == 1'b1)
    begin
      csr_crmd_datm <= 2'b01;
    end
    else if(csr_we & csr_num == `CSR_CRMD)
    begin
      csr_crmd_datm <= csr_wmask[`CSR_CRMD_DATM] & csr_wvalue[`CSR_CRMD_DATM]
                    | ~csr_wmask[`CSR_CRMD_DATM] & csr_crmd_datm;
    end
  end

  //////////////////////////    PRMD 例外前模式信息     /////////////////////////////////////
  assign csr_prmd_rvalue = {29'b0,
                            csr_prmd_pie,
                            csr_prmd_pplv};
  /////////////////////////    PPLV CSR.CRMD 中 PLV 域的旧值记录 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_prmd_pplv <= 2'b0;
    end
    else if(wb_ex && {csr_crmd_plv,csr_crmd_ie} != 3'b0)
    begin
      csr_prmd_pplv <= csr_crmd_plv;
    end
    else if(csr_we && csr_num == `CSR_PRMD)
    begin
      csr_prmd_pplv <=  csr_wmask[`CSR_PRMD_PPLV] & csr_wvalue[`CSR_PRMD_PPLV]
                    |  ~csr_wmask[`CSR_PRMD_PPLV] & csr_prmd_pplv;
    end
  end

  /////////////////////////    PIE CSR.CRMD 中 IE 域的旧值记录 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_prmd_pie <= 1'b0;
    end
    else if(wb_ex)
    begin
      csr_prmd_pie <= csr_crmd_ie;
    end
    else if(csr_we && csr_num == `CSR_PRMD)
    begin
      csr_prmd_pie <=  csr_wmask[`CSR_PRMD_PIE] & csr_wvalue[`CSR_PRMD_PIE]
                   |  ~csr_wmask[`CSR_PRMD_PIE] & csr_prmd_pie;
    end
  end

  //////////////////////////    ECFG 例外控制     /////////////////////////////////////
  assign csr_ecfg_rvalue = {19'b0,
                            csr_ecfg_lie[12:11],
                            1'b0,                 //没有保证第10位总为0，但是读出来的一定是0
                            csr_ecfg_lie[9:0]};
  /////////////////////////    LIE 局部中断使能位 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_ecfg_lie <= 13'b0;
    end
    else if(csr_we && csr_num == `CSR_ECFG)
    begin
      csr_ecfg_lie <= csr_wmask[`CSR_ECFG_LIE] & csr_wvalue[`CSR_ECFG_LIE]
                   | ~csr_wmask[`CSR_ECFG_LIE] & csr_ecfg_lie;
    end
  end

  //////////////////////////    ESTAT 例外状态     /////////////////////////////////////
  assign csr_estat_rvalue = {1'b0,
                             csr_estat_esubcode,
                             csr_estat_ecode,
                             3'b0,
                             csr_estat_is};
  /////////////////////////    IS 软件和硬件中断的状态位 RW、R
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_estat_is <= 13'b0;
    end
    else if(csr_we && csr_num == `CSR_ESTAT)
    begin
      csr_estat_is[1:0] <= csr_wmask[`CSR_ESTAT_IS_1_0] & csr_wvalue[`CSR_ESTAT_IS_1_0]
                  | ~csr_wmask[`CSR_ESTAT_IS_1_0] & csr_estat_is[1:0];
    end

    if(csr_tcfg_en && csr_tval_timeval == 32'b0)
    begin
      csr_estat_is[11] <= 1'b1;
    end
    else if(csr_we && csr_num ==`CSR_TICLR && (csr_wmask[`CSR_TICLR_CLR] & csr_wvalue[`CSR_TICLR_CLR]))
    begin
      csr_estat_is[11] <= 1'b0;
    end
  end
  /////////////////////////    ECODE 例外类型一级编码 R
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_estat_ecode <= 6'b0;        //注意这是为了避免高阻态的默认值
    end
    else if(wb_ex)
    begin
      csr_estat_ecode <= wb_ecode;
    end
  end
  /////////////////////////    ESUBCODE 例外类型二级编码 R
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_estat_esubcode <= 9'b0;
    end
    else if(wb_ex)
    begin
      csr_estat_esubcode <= wb_esubcode;
    end
  end

  //////////////////////////    ERA 例外返回地址     /////////////////////////////////////
  assign csr_era_rvalue = csr_era_pc;
  /////////////////////////    PC 触发例外的指令的 PC 记录 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_era_pc <= 32'b0;
    end
    else if(wb_ex)
    begin
      csr_era_pc <= ws_pc;
    end
    else if(csr_we && csr_num == `CSR_ERA)
    begin
      csr_era_pc <= csr_wmask[`CSR_ERA_PC] & csr_wvalue[`CSR_ERA_PC]
                 | ~csr_wmask[`CSR_ERA_PC] & csr_era_pc;
    end
  end


  //////////////////////////    BADV 出错虚地址     /////////////////////////////////////
  assign csr_badv_rvalue = csr_badv_vaddr;
  /////////////////////////    VADDR 触发 TLB 重填例外和地址错误相关例外时出错的虚地址记录 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_badv_vaddr <= 32'b0;
    end
    else if(wb_badv_we)
    begin     //!!!!!!!!!!!!!!!!!!!!!!!!!!!有待补全，维护逻辑不完善
      csr_badv_vaddr <= wb_addr;
    end
    else if(csr_we && csr_num == `CSR_BADV)
    begin
      csr_badv_vaddr <= csr_wmask[`CSR_BADV_VADDR] & csr_wvalue[`CSR_BADV_VADDR]
                     | ~csr_wmask[`CSR_BADV_VADDR] & csr_badv_vaddr;
    end
  end

  //////////////////////////    EENTRY 例外入口地址     /////////////////////////////////////
  assign csr_eentry_rvalue = {csr_eentry_va,
                              6'b0};
  /////////////////////////    VA 例外和中断入口地址 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_eentry_va <= 26'b0;     //这里我还没有表示其内容从何处来！！！！！！！！！！！！！！！！！！！！！！！！！
    end
    else if(csr_we && csr_num == `CSR_EENTRY)
    begin
      csr_eentry_va <= csr_wmask[`CSR_EENTRY_VA] & csr_wvalue[`CSR_EENTRY_VA]
                    | ~csr_wmask[`CSR_EENTRY_VA] & csr_eentry_va;
    end
  end

  //////////////////////////    SAVE0 数据保存     /////////////////////////////////////
  assign csr_save0_rvalue = csr_save0_data;
  /////////////////////////    DATA 仅供软件读写的数据 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_save0_data <= 32'b0;
    end
    else if(csr_we && csr_num == `CSR_SAVE0)
    begin
      csr_save0_data <= csr_wmask[`CSR_SAVE0_DATA] & csr_wvalue[`CSR_SAVE0_DATA]
                     | ~csr_wmask[`CSR_SAVE0_DATA] & csr_save0_data;
    end
  end
  //////////////////////////    SAVE1 数据保存     /////////////////////////////////////
  assign csr_save1_rvalue = csr_save1_data;
  /////////////////////////    DATA 仅供软件读写的数据 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_save1_data <= 32'b0;
    end
    else if(csr_we && csr_num == `CSR_SAVE1)
    begin
      csr_save1_data <= csr_wmask[`CSR_SAVE1_DATA] & csr_wvalue[`CSR_SAVE1_DATA]
                     | ~csr_wmask[`CSR_SAVE1_DATA] & csr_save1_data;
    end
  end
  //////////////////////////    SAVE2 数据保存     /////////////////////////////////////
  assign csr_save2_rvalue = csr_save2_data;
  /////////////////////////    DATA 仅供软件读写的数据 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_save2_data <= 32'b0;
    end
    else if(csr_we && csr_num == `CSR_SAVE2)
    begin
      csr_save2_data <= csr_wmask[`CSR_SAVE2_DATA] & csr_wvalue[`CSR_SAVE2_DATA]
                     | ~csr_wmask[`CSR_SAVE2_DATA] & csr_save2_data;
    end
  end
  //////////////////////////    SAVE3 数据保存     /////////////////////////////////////
  assign csr_save3_rvalue = csr_save3_data;
  /////////////////////////    DATA 仅供软件读写的数据 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_save3_data <= 32'b0;
    end
    else if(csr_we && csr_num == `CSR_SAVE3)
    begin
      csr_save3_data <= csr_wmask[`CSR_SAVE3_DATA] & csr_wvalue[`CSR_SAVE3_DATA]
                     | ~csr_wmask[`CSR_SAVE3_DATA] & csr_save3_data;
    end
  end

  //////////////////////////    TID 定时器编号     /////////////////////////////////////
  assign csr_tid_rvalue = csr_tid_tid;
  /////////////////////////    TID 定时器编号 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_tid_tid <= 32'b0;       //这里没有csr_cpuid_coreid
    end
    else if(csr_we && csr_num == `CSR_TID)
    begin
      csr_tid_tid <= csr_wmask[`CSR_TID_TID] & csr_wvalue[`CSR_TID_TID]
                  | ~csr_wmask[`CSR_TID_TID] & csr_tid_tid;
    end
  end
  //////////////////////////    TCFG 定时器配置     /////////////////////////////////////
  assign csr_tcfg_rvalue = {csr_tcfg_initval,
                            csr_tcfg_periodic,
                            csr_tcfg_en};
  /////////////////////////    EN 定时器使能位 RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_tcfg_en <= 1'b0;
    end
    else if(csr_we && csr_num == `CSR_TCFG)
    begin
      csr_tcfg_en <= csr_wmask[`CSR_TCFG_EN] & csr_wvalue[`CSR_TCFG_EN]
                  | ~csr_wmask[`CSR_TCFG_EN] & csr_tcfg_en;
    end
  end
  /////////////////////////    PERIODIC 定时器循环模式控制位 RW
  always @(posedge clk)
  begin
    if (reset)
    begin
      csr_tcfg_periodic <= 1'b0;
    end
    else if (csr_we && csr_num == `CSR_TCFG)
    begin
      csr_tcfg_periodic <= csr_wmask[`CSR_TCFG_PERIODIC] & csr_wvalue[`CSR_TCFG_PERIODIC]
                        | ~csr_wmask[`CSR_TCFG_PERIODIC] & csr_tcfg_periodic;
    end
  end
  /////////////////////////    INITVAL 定时器倒计时自减计数的初始值 RW
  always @(posedge clk)
  begin
    if (reset)
    begin
      csr_tcfg_initval <= 32'b0;
    end
    else if (csr_we && csr_num == `CSR_TCFG)
    begin
      csr_tcfg_initval <= csr_wmask[`CSR_TCFG_INITVAL] & csr_wvalue[`CSR_TCFG_INITVAL]
                       | ~csr_wmask[`CSR_TCFG_INITVAL] & csr_tcfg_initval;
    end
  end

  //////////////////////////    TVAL 定时器数值     /////////////////////////////////////
  assign csr_tval_rvalue = csr_tval_timeval;
  /////////////////////////    TIMEVAL 当前定时器的计数值 R
  assign tcfg_next_value = csr_wmask[31:0] & csr_wvalue[31:0]
         | ~csr_wmask[31:0] &{csr_tcfg_initval, csr_tcfg_periodic, csr_tcfg_en};

  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_tval_timeval <= 32'hffffffff;
    end
    else if(csr_we && csr_num == `CSR_TCFG && csr_wvalue)    //原始条件是csr_we && csr_num == `CSR_TVAL && tcfg_next_value[`CSR_TCFG_EN]
    begin
      csr_tval_timeval <= {tcfg_next_value[`CSR_TCFG_INITVAL],2'b0};
    end
    else if(csr_tcfg_en && csr_tval_timeval != 32'hffffffff)
    begin
      if(csr_tval_timeval[31:0] == 32'b0 && csr_tcfg_periodic)
      begin
        csr_tval_timeval <= {csr_tcfg_initval, 2'b0};
      end
      else
      begin
        csr_tval_timeval <= csr_tval_timeval -1'b1;
      end
    end
  end
  //////////////////////////    TICLR 定时中断清除     /////////////////////////////////////
  assign csr_ticlr_rvalue = 32'b0;
  /////////////////////////    CLR 当对该 bit 写值 1 时，将清除时钟中断标记。该寄存器读出结果总为 0 W1

  assign csr_tid = csr_tid_rvalue;
  assign ertn_csr_era_pc = csr_era_rvalue;
  assign ertn_csr_eentry_pc = csr_eentry_rvalue;
  assign has_int = ((csr_estat_is[12:0] & csr_ecfg_lie[12:0]) != 13'b0) && (csr_crmd_ie == 1'b1);

  assign csr_rvalue = {32{csr_num == `CSR_CRMD  }} & csr_crmd_rvalue
         | {32{csr_num == `CSR_PRMD  }} & csr_prmd_rvalue
         | {32{csr_num == `CSR_ECFG  }} & csr_ecfg_rvalue
         | {32{csr_num == `CSR_ESTAT }} & csr_estat_rvalue
         | {32{csr_num == `CSR_ERA   }} & csr_era_rvalue
         | {32{csr_num == `CSR_BADV  }} & csr_badv_rvalue
         | {32{csr_num == `CSR_EENTRY}} & csr_eentry_rvalue
         | {32{csr_num == `CSR_SAVE0 }} & csr_save0_rvalue
         | {32{csr_num == `CSR_SAVE1 }} & csr_save1_rvalue
         | {32{csr_num == `CSR_SAVE2 }} & csr_save2_rvalue
         | {32{csr_num == `CSR_SAVE3 }} & csr_save3_rvalue
         | {32{csr_num == `CSR_TID   }} & csr_tid_rvalue
         | {32{csr_num == `CSR_TCFG  }} & csr_tcfg_rvalue
         | {32{csr_num == `CSR_TVAL  }} & csr_tval_rvalue
         | {32{csr_num == `CSR_TICLR }} & csr_ticlr_rvalue;

endmodule
