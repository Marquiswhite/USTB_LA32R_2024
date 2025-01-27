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
    input  wire csr_re,       //��ʹ��
    input  wire [13:0] csr_num,     //�Ĵ�����
    output wire [31:0] csr_rvalue,       //�Ĵ���������ֵ
    input  wire csr_we,     //дʹ��
    input  wire [31:0] csr_wmask,  //д����
    input  wire [31:0] csr_wvalue,    //д����
    input  wire wb_ex ,      //д�ؼ����쳣�������ź�
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

  //crmd ��ǰģʽ��Ϣ
  reg  [ 1:0] csr_crmd_plv;    //��ǰ��Ȩ�ȼ�
  reg        csr_crmd_ie;      //��ǰȫ���ж�ʹ��
  reg        csr_crmd_da;      //ֱ�ӵ�ַ����ģʽ��ʹ��
  reg        csr_crmd_pg;      //ӳ���ַģʽ��ʹ��
  reg  [ 1:0] csr_crmd_datf;   //ֱ�ӵ�ַ����ģʽʱȡָ�����Ĵ洢��������
  reg  [ 1:0] csr_crmd_datm;   //ֱ�ӵ�ַ����ģʽʱ��load��store�����Ĵ洢��������
  wire [31:0] csr_crmd_rvalue; //���Ĵ�����ֵ

  //prmd ����ǰģʽ��Ϣ
  reg [1:0] csr_prmd_pplv;    //��������ʱ����¼csr_crmd_plv�ľ�ֵ
  reg       csr_prmd_pie;     //��������ʱ����¼csr_crmd_ie�ľ�ֵ
  wire [31:0] csr_prmd_rvalue;

  //ecfg �������
  reg [12:0] csr_ecfg_lie;    //�ֲ��ж�ʹ��λ
  wire [31:0] csr_ecfg_rvalue;

  //estat ����״̬
  reg [12:0] csr_estat_is;    //�ж�״̬λ
  reg [ 5:0] csr_estat_ecode; //��������һ������
  reg [ 8:0] csr_estat_esubcode;//�������Ͷ�������
  wire [31:0] csr_estat_rvalue;

  //era ���ⷵ�ص�ַ
  reg [31:0] csr_era_pc;      //��¼���������ָ���PC
  wire [31:0] csr_era_rvalue;

  //badv �������ַ
  reg [31:0] csr_badv_vaddr;  //����TLB��������͵�ַ�����������ʱ����¼��������ַ
  wire [31:0] csr_badv_rvalue;

  //eentry ������ڵ�ַ
  reg [25:0] csr_eentry_va;   //������ܶ���ڵ�ַ��[31:6]λ����6λ����Ϊ0
  wire [31:0] csr_eentry_rvalue;

  //save ���ݱ���
  reg [31:0] csr_save0_data;  //���������д�����ݣ���csrָ���⣬Ӳ�������޸�������
  wire [31:0] csr_save0_rvalue;

  reg [31:0] csr_save1_data;
  wire [31:0] csr_save1_rvalue;

  reg [31:0] csr_save2_data;
  wire [31:0] csr_save2_rvalue;

  reg [31:0] csr_save3_data;
  wire [31:0] csr_save3_rvalue;

  //tid ��ʱ�����
  reg [31:0] csr_tid_tid;     //��ʱ�����
  wire [31:0] csr_tid_rvalue;

  //tcfg ��ʱ������
  reg        csr_tcfg_en;     //��ʱ��ʹ��λ
  reg        csr_tcfg_periodic;//��ʱ��ѭ��ģʽ����λ
  reg [31:2] csr_tcfg_initval;//��ʱ������ʱ�Լ������ĳ�ʼֵ
  wire [31:0] csr_tcfg_rvalue;

  //tval ��ʱ����ֵ
  reg [31:0] csr_tval_timeval;//��ǰ�������ļ���ֵ
  wire [31:0] tcfg_next_value;
  wire [31:0] csr_tval_rvalue;        //ʾ���е�csr_tval

  //ticlr �����ж����
  //reg        csr_ticlr_clr;   //����λΪ1ʱ�������ʱ���жϱ�ǣ����������ΰ0
  wire [31:0] csr_ticlr_rvalue;

  //////////////////////////    CRMD ��ǰ״̬��Ϣ     /////////////////////////////////////
  assign csr_crmd_rvalue = {23'b0,
                            csr_crmd_datm,
                            csr_crmd_datf,
                            csr_crmd_pg,
                            csr_crmd_da,
                            csr_crmd_ie,
                            csr_crmd_plv};
  /////////////////////////    PLV ��ǰ��Ȩ�ȼ� RW
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

  /////////////////////////    IE ��ǰȫ���ж�ʹ�� RW
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

  /////////////////////////    DA ֱ�ӵ�ַ����ģʽ��ʹ�� RW
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

  /////////////////////////    PG ӳ���ַ����ģʽ��ʹ�� RW
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

  /////////////////////////    DATF ֱ�ӵ�ַ����ģʽʱ��ȡָ�����Ĵ洢�������� RW
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


  /////////////////////////    DATM ֱ�ӵ�ַ����ģʽʱ��load �� store �����Ĵ洢�������� RW
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

  //////////////////////////    PRMD ����ǰģʽ��Ϣ     /////////////////////////////////////
  assign csr_prmd_rvalue = {29'b0,
                            csr_prmd_pie,
                            csr_prmd_pplv};
  /////////////////////////    PPLV CSR.CRMD �� PLV ��ľ�ֵ��¼ RW
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

  /////////////////////////    PIE CSR.CRMD �� IE ��ľ�ֵ��¼ RW
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

  //////////////////////////    ECFG �������     /////////////////////////////////////
  assign csr_ecfg_rvalue = {19'b0,
                            csr_ecfg_lie[12:11],
                            1'b0,                 //û�б�֤��10λ��Ϊ0�����Ƕ�������һ����0
                            csr_ecfg_lie[9:0]};
  /////////////////////////    LIE �ֲ��ж�ʹ��λ RW
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

  //////////////////////////    ESTAT ����״̬     /////////////////////////////////////
  assign csr_estat_rvalue = {1'b0,
                             csr_estat_esubcode,
                             csr_estat_ecode,
                             3'b0,
                             csr_estat_is};
  /////////////////////////    IS �����Ӳ���жϵ�״̬λ RW��R
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
  /////////////////////////    ECODE ��������һ������ R
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_estat_ecode <= 6'b0;        //ע������Ϊ�˱������̬��Ĭ��ֵ
    end
    else if(wb_ex)
    begin
      csr_estat_ecode <= wb_ecode;
    end
  end
  /////////////////////////    ESUBCODE �������Ͷ������� R
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

  //////////////////////////    ERA ���ⷵ�ص�ַ     /////////////////////////////////////
  assign csr_era_rvalue = csr_era_pc;
  /////////////////////////    PC ���������ָ��� PC ��¼ RW
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


  //////////////////////////    BADV �������ַ     /////////////////////////////////////
  assign csr_badv_rvalue = csr_badv_vaddr;
  /////////////////////////    VADDR ���� TLB ��������͵�ַ�����������ʱ��������ַ��¼ RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_badv_vaddr <= 32'b0;
    end
    else if(wb_badv_we)
    begin     //!!!!!!!!!!!!!!!!!!!!!!!!!!!�д���ȫ��ά���߼�������
      csr_badv_vaddr <= wb_addr;
    end
    else if(csr_we && csr_num == `CSR_BADV)
    begin
      csr_badv_vaddr <= csr_wmask[`CSR_BADV_VADDR] & csr_wvalue[`CSR_BADV_VADDR]
                     | ~csr_wmask[`CSR_BADV_VADDR] & csr_badv_vaddr;
    end
  end

  //////////////////////////    EENTRY ������ڵ�ַ     /////////////////////////////////////
  assign csr_eentry_rvalue = {csr_eentry_va,
                              6'b0};
  /////////////////////////    VA ������ж���ڵ�ַ RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_eentry_va <= 26'b0;     //�����һ�û�б�ʾ�����ݴӺδ�����������������������������������������������������
    end
    else if(csr_we && csr_num == `CSR_EENTRY)
    begin
      csr_eentry_va <= csr_wmask[`CSR_EENTRY_VA] & csr_wvalue[`CSR_EENTRY_VA]
                    | ~csr_wmask[`CSR_EENTRY_VA] & csr_eentry_va;
    end
  end

  //////////////////////////    SAVE0 ���ݱ���     /////////////////////////////////////
  assign csr_save0_rvalue = csr_save0_data;
  /////////////////////////    DATA ���������д������ RW
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
  //////////////////////////    SAVE1 ���ݱ���     /////////////////////////////////////
  assign csr_save1_rvalue = csr_save1_data;
  /////////////////////////    DATA ���������д������ RW
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
  //////////////////////////    SAVE2 ���ݱ���     /////////////////////////////////////
  assign csr_save2_rvalue = csr_save2_data;
  /////////////////////////    DATA ���������д������ RW
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
  //////////////////////////    SAVE3 ���ݱ���     /////////////////////////////////////
  assign csr_save3_rvalue = csr_save3_data;
  /////////////////////////    DATA ���������д������ RW
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

  //////////////////////////    TID ��ʱ�����     /////////////////////////////////////
  assign csr_tid_rvalue = csr_tid_tid;
  /////////////////////////    TID ��ʱ����� RW
  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_tid_tid <= 32'b0;       //����û��csr_cpuid_coreid
    end
    else if(csr_we && csr_num == `CSR_TID)
    begin
      csr_tid_tid <= csr_wmask[`CSR_TID_TID] & csr_wvalue[`CSR_TID_TID]
                  | ~csr_wmask[`CSR_TID_TID] & csr_tid_tid;
    end
  end
  //////////////////////////    TCFG ��ʱ������     /////////////////////////////////////
  assign csr_tcfg_rvalue = {csr_tcfg_initval,
                            csr_tcfg_periodic,
                            csr_tcfg_en};
  /////////////////////////    EN ��ʱ��ʹ��λ RW
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
  /////////////////////////    PERIODIC ��ʱ��ѭ��ģʽ����λ RW
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
  /////////////////////////    INITVAL ��ʱ������ʱ�Լ������ĳ�ʼֵ RW
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

  //////////////////////////    TVAL ��ʱ����ֵ     /////////////////////////////////////
  assign csr_tval_rvalue = csr_tval_timeval;
  /////////////////////////    TIMEVAL ��ǰ��ʱ���ļ���ֵ R
  assign tcfg_next_value = csr_wmask[31:0] & csr_wvalue[31:0]
         | ~csr_wmask[31:0] &{csr_tcfg_initval, csr_tcfg_periodic, csr_tcfg_en};

  always@(posedge clk)
  begin
    if(reset)
    begin
      csr_tval_timeval <= 32'hffffffff;
    end
    else if(csr_we && csr_num == `CSR_TCFG && csr_wvalue)    //ԭʼ������csr_we && csr_num == `CSR_TVAL && tcfg_next_value[`CSR_TCFG_EN]
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
  //////////////////////////    TICLR ��ʱ�ж����     /////////////////////////////////////
  assign csr_ticlr_rvalue = 32'b0;
  /////////////////////////    CLR ���Ը� bit дֵ 1 ʱ�������ʱ���жϱ�ǡ��üĴ������������Ϊ 0 W1

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
