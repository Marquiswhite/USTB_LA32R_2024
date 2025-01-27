`include"mycpu_head.v"

module mycpu_top(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    output wire        inst_sram_en,        /*这里增加了*/
    output wire [3:0]  inst_sram_we,        /*这里改成了4比特*/
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // data sram interface
    output wire        data_sram_en,        /*这里增加了*/
    output wire [3:0]  data_sram_we,        /*这里改成了4比特*/
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);
reg         reset;
always @(posedge clk) reset <= ~resetn;

wire ds_allowin;
wire es_allowin;
wire ms_allowin;
wire ws_allowin;

wire fs_to_ds_valid;
wire ds_to_es_valid;
wire es_to_ms_valid;
wire ms_to_ws_valid;

wire [`BR_BUS_WD -1 :0] br_bus;
wire [`FS_TO_DS_BUS_WD - 1:0] fs_to_ds_bus;
wire [`DS_TO_ES_BUS_WD - 1:0] ds_to_es_bus;
wire [`ES_TO_MS_BUS_WD - 1:0] es_to_ms_bus;
wire [`MS_TO_WS_BUS_WD - 1:0] ms_to_ws_bus;
wire [`WS_TO_RF_BUS_WD - 1:0] ws_to_rf_bus;
wire [`ES_TO_DS_BUS_WD - 1:0] es_to_ds_bus;
wire [`MS_TO_DS_BUS_WD - 1:0] ms_to_ds_bus;
wire [`WS_TO_DS_BUS_WD - 1:0] ws_to_ds_bus;

wire ws_ertn_flush;
wire [31:0] ertn_pc;

IF_stage u_IF_stage(
    .clk(clk),
    .reset(reset),
    .ds_allowin(ds_allowin),
    .br_bus(br_bus),
    .fs_to_ds_valid(fs_to_ds_valid),
    .fs_to_ds_bus(fs_to_ds_bus),
    .inst_sram_en(inst_sram_en),
    .inst_sram_we(inst_sram_we),
    .inst_sram_addr(inst_sram_addr),
    .inst_sram_wdata(inst_sram_wdata),
    .inst_sram_rdata(inst_sram_rdata),
    .ws_ertn_flush(ws_ertn_flush),
    .ertn_pc(ertn_pc)
);

ID_stage u_ID_stage(
    .clk(clk),
    .reset(reset),
    .es_allowin(es_allowin),
    .ds_allowin(ds_allowin),
    .fs_to_ds_valid(fs_to_ds_valid),
    .fs_to_ds_bus(fs_to_ds_bus),
    .ds_to_es_valid(ds_to_es_valid),
    .ds_to_es_bus(ds_to_es_bus),
    .br_bus(br_bus),
    .ws_to_rf_bus(ws_to_rf_bus),
    .es_to_ds_bus(es_to_ds_bus),
    .ms_to_ds_bus(ms_to_ds_bus),
    .ws_to_ds_bus(ws_to_ds_bus),
    .ws_ertn_flush(ws_ertn_flush)
);

EXE_stage u_EXE_stage(
    .clk(clk),
    .reset(reset),
    .ms_allowin(ms_allowin),
    .es_allowin(es_allowin),
    .ds_to_es_valid(ds_to_es_valid),
    .ds_to_es_bus(ds_to_es_bus),
    .es_to_ms_valid(es_to_ms_valid),
    .es_to_ms_bus(es_to_ms_bus),
    .data_sram_en(data_sram_en),
    .data_sram_we(data_sram_we),
    .data_sram_addr(data_sram_addr),
    .data_sram_wdata(data_sram_wdata),
    .es_to_ds_bus(es_to_ds_bus),
    .ws_ertn_flush(ws_ertn_flush)
);

MEM_stage u_MEM_stage(
    .clk(clk),
    .reset(reset),
    .ws_allowin(ws_allowin),
    .ms_allowin(ms_allowin),
    .es_to_ms_valid(es_to_ms_valid),
    .es_to_ms_bus(es_to_ms_bus),
    .ms_to_ws_valid(ms_to_ws_valid),
    .ms_to_ws_bus(ms_to_ws_bus),
    .data_sram_rdata(data_sram_rdata),
    .ms_to_ds_bus(ms_to_ds_bus),
    .ws_ertn_flush(ws_ertn_flush)
);

WB_stage u_WB_stage(
    .clk(clk),
    .reset(reset),
    .ws_allowin(ws_allowin),
    .ms_to_ws_valid(ms_to_ws_valid),
    .ms_to_ws_bus(ms_to_ws_bus),
    .ws_to_rf_bus(ws_to_rf_bus),
    .debug_wb_pc(debug_wb_pc),
    .debug_wb_rf_we(debug_wb_rf_we),
    .debug_wb_rf_wnum(debug_wb_rf_wnum),
    .debug_wb_rf_wdata(debug_wb_rf_wdata),
    .ws_to_ds_bus(ws_to_ds_bus),
    .ws_ertn_flush(ws_ertn_flush),
    .ertn_pc(ertn_pc)
);

endmodule
