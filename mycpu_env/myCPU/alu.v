module alu(
  input  wire clk,
  input  wire [18:0] alu_op,
  input  wire reset,
  input  wire [31:0] alu_src1,
  input  wire [31:0] alu_src2,
  output wire [31:0] alu_result,
  output wire div_delay
);

wire op_add;   //add operation
wire op_sub;   //sub operation
wire op_slt;   //signed compared and set less than
wire op_sltu;  //unsigned compared and set less than
wire op_and;   //bitwise and
wire op_nor;   //bitwise nor
wire op_or;    //bitwise or
wire op_xor;   //bitwise xor
wire op_sll;   //logic left shift
wire op_srl;   //logic right shift
wire op_sra;   //arithmetic right shift
wire op_lui;   //Load Upper Immediate
wire op_mul_high;   //乘法 mul operation 有符号
wire op_mul_low;    //乘法 mul operation 有符号
wire op_mul_highu;  //乘法 mul operation 无符号
wire op_div;
wire op_mod;
wire op_divu;
wire op_modu;


wire signed [31:0] alu_src1_signed;
wire signed [31:0] alu_src2_signed;

assign alu_src1_signed = alu_src1;
assign alu_src2_signed = alu_src2;

// control code decomposition
assign op_add       = alu_op[ 0];
assign op_sub       = alu_op[ 1];
assign op_slt       = alu_op[ 2];
assign op_sltu      = alu_op[ 3];
assign op_and       = alu_op[ 4];
assign op_nor       = alu_op[ 5];
assign op_or        = alu_op[ 6];
assign op_xor       = alu_op[ 7];
assign op_sll       = alu_op[ 8];
assign op_srl       = alu_op[ 9];
assign op_sra       = alu_op[10];
assign op_lui       = alu_op[11];
assign op_mul_low   = alu_op[12];
assign op_mul_high  = alu_op[13];
assign op_mul_highu = alu_op[14];

assign op_div       = alu_op[15];
assign op_mod       = alu_op[16];
assign op_divu      = alu_op[17];
assign op_modu      = alu_op[18];

wire [31:0] add_sub_result;
wire [31:0] slt_result;
wire [31:0] sltu_result;
wire [31:0] and_result;
wire [31:0] nor_result;
wire [31:0] or_result;
wire [31:0] xor_result;
wire [31:0] lui_result;
wire [31:0] sll_result;
wire [63:0] sr64_result;
wire [31:0] sr_result;

wire [63:0] mul_result_signed;
wire [63:0] mul_result_unsigned;



// 32-bit adder
wire [31:0] adder_a;
wire [31:0] adder_b;
wire        adder_cin;
wire [31:0] adder_result;
wire        adder_cout;

//除法IP核的实现
/*
div_gen_0 u_div_gen_0(
.aclk(clk),
.s_axis_divisor_tvalid(),       //除数
.s_axis_divisor_tdata(),
.s_axis_dividend_tvalid(),

);
*/

assign adder_a   = alu_src1;
assign adder_b   = (op_sub | op_slt | op_sltu) ? ~alu_src2 : alu_src2;  //src1 - src2 rj-rk
assign adder_cin = (op_sub | op_slt | op_sltu) ? 1'b1      : 1'b0;
assign {adder_cout, adder_result} = adder_a + adder_b + adder_cin;

// ADD, SUB result
assign add_sub_result = adder_result;

// SLT result
assign slt_result[31:1] = 31'b0;   //rj < rk 1
assign slt_result[0]    = (alu_src1[31] & ~alu_src2[31])
                        | ((alu_src1[31] ~^ alu_src2[31]) & adder_result[31]);

// SLTU result
assign sltu_result[31:1] = 31'b0;
assign sltu_result[0]    = ~adder_cout;

// bitwise operation
assign and_result = alu_src1 & alu_src2;
/*assign or_result  = alu_src1 | alu_src2 | alu_result;*/
assign or_result  = alu_src1 | alu_src2 ;
assign nor_result = ~or_result;
assign xor_result = alu_src1 ^ alu_src2;
assign lui_result = alu_src2;

// SLL result
/*assign sll_result = alu_src2 << alu_src1[4:0];   //rj << i5*/
assign sll_result = alu_src1 << alu_src2[4:0];   

// SRL, SRA result
/*assign sr64_result = {{32{op_sra & alu_src2[31]}}, alu_src2[31:0]} >> alu_src1[4:0]; //rj >> i5*/
assign sr64_result = {{32{op_sra & alu_src1[31]}}, alu_src1[31:0]} >> alu_src2[4:0]; //rj >> i5

/*assign sr_result   = sr64_result[30:0];*/
assign sr_result   = sr64_result[31:0];

// MUL result
assign mul_result_signed = alu_src1_signed * alu_src2_signed;
assign mul_result_unsigned = alu_src1 * alu_src2;

// DIV result
reg  div_src_delay_signed;
reg  div_src_delay_unsigned;

wire dividend_valid_signed;
wire divisor_valid_signed;
wire div_result_valid_signed;
wire [63:0] div_result_signed;
wire div_computing_signed;
wire [31:0] div_quotient_signed;
wire [31:0] div_remainder_signed;

wire dividend_valid_unsigned;
wire divisor_valid_unsigned;
wire div_result_valid_unsigned;
wire [63:0] div_result_unsigned;
wire div_computing_unsigned;
wire [31:0] div_quotient_unsigned;
wire [31:0] div_remainder_unsigned;

assign div_delay = (div_computing_signed | div_computing_unsigned) ? 1'b1 : 1'b0;

assign div_quotient_signed = div_result_signed[63:32];
assign div_remainder_signed = div_result_signed[31:0];
assign div_quotient_unsigned = div_result_unsigned[63:32];
assign div_remainder_unsigned = div_result_unsigned[31:0];

assign div_computing_signed = (op_mod | op_div) & ~div_result_valid_signed;
assign div_computing_unsigned = (op_modu | op_divu) & ~div_result_valid_unsigned;

assign dividend_valid_signed = div_computing_signed & ~div_src_delay_signed;
assign divisor_valid_signed = div_computing_signed & ~div_src_delay_signed;
assign dividend_valid_unsigned = div_computing_unsigned & ~div_src_delay_unsigned;
assign divisor_valid_unsigned = div_computing_unsigned & ~div_src_delay_unsigned;

always@(posedge clk)begin
    if(reset)begin
        div_src_delay_signed <= 1'b0;
    end else if(~div_src_delay_signed & div_computing_signed)begin
        div_src_delay_signed <= 1'b1;
    end else if(div_result_valid_signed) begin
        div_src_delay_signed <= 1'b0;
    end
end

always@(posedge clk)begin
    if(reset)begin
        div_src_delay_unsigned <= 1'b0;
    end else if(~div_src_delay_unsigned & div_computing_unsigned)begin
        div_src_delay_unsigned <= 1'b1;
    end else if(div_result_valid_unsigned)begin
        div_src_delay_unsigned <= 1'b0;
    end
end

div_gen_signed u_div_gen_signed(
.aclk(clk),
.s_axis_dividend_tdata(alu_src1_signed),
.s_axis_dividend_tvalid(dividend_valid_signed),
.s_axis_divisor_tdata(alu_src2_signed),
.s_axis_divisor_tvalid(divisor_valid_signed),
.m_axis_dout_tvalid(div_result_valid_signed),
.m_axis_dout_tdata(div_result_signed)
);

div_gen_unsigned u_div_gen_unsigned(
.aclk(clk),
.s_axis_dividend_tdata(alu_src1),
.s_axis_dividend_tvalid(dividend_valid_unsigned),
.s_axis_divisor_tdata(alu_src2),
.s_axis_divisor_tvalid(divisor_valid_unsigned),
.m_axis_dout_tvalid(div_result_valid_unsigned),
.m_axis_dout_tdata(div_result_unsigned)
);

// final result mux
assign alu_result = ({32{op_add|op_sub}} & add_sub_result)
                  | ({32{op_slt       }} & slt_result)
                  | ({32{op_sltu      }} & sltu_result)
                  | ({32{op_and       }} & and_result)
                  | ({32{op_nor       }} & nor_result)
                  | ({32{op_or        }} & or_result)
                  | ({32{op_xor       }} & xor_result)
                  | ({32{op_lui       }} & lui_result)
                  | ({32{op_sll       }} & sll_result)
                  | ({32{op_srl|op_sra}} & sr_result)
                  | ({32{op_mul_low   }} & mul_result_signed[31:0])
                  | ({32{op_mul_high  }} & mul_result_signed[63:32])
                  | ({32{op_mul_highu }} & mul_result_unsigned[63:32])
                  | ({32{op_div       }} & div_quotient_signed)
                  | ({32{op_mod       }} & div_remainder_signed)
                  | ({32{op_divu      }} & div_quotient_unsigned)
                  | ({32{op_modu      }} & div_remainder_unsigned);

endmodule
