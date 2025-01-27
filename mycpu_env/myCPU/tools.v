module decoder_2_4(
    input  wire [ 1:0] in,
    output wire [ 3:0] out
);

genvar i;
generate for (i=0; i<4; i=i+1) begin : gen_for_dec_2_4
    assign out[i] = (in == i);
end endgenerate

endmodule


module decoder_4_16(
    input  wire [ 3:0] in,
    output wire [15:0] out
);

genvar i;
generate for (i=0; i<16; i=i+1) begin : gen_for_dec_4_16
    assign out[i] = (in == i);
end endgenerate

endmodule


module decoder_5_32(
    input  wire [ 4:0] in,
    output wire [31:0] out
);

genvar i;
generate for (i=0; i<32; i=i+1) begin : gen_for_dec_5_32
    assign out[i] = (in == i);
end endgenerate

endmodule


module decoder_6_64(
    input  wire [ 5:0] in,
    output wire [63:0] out
);

genvar i;
generate for (i=0; i<64; i=i+1) begin : gen_for_dec_6_64
    assign out[i] = (in == i);
end endgenerate

endmodule


module byte_write_enable_b(
    input  wire [1:0] in,
    output wire [3:0] out
);
genvar i;
generate for (i=0; i<4; i=i+1) begin : gen_for_byte_write_enable_b
    assign out[i] = (in == i);
end endgenerate
endmodule

module byte_write_enable_h(         
    input  wire [1:0] in,               //00 01 10 11
    output wire [3:0] out               //0011 0000 1100 0000
);
assign out = (in == 2'b01 || in == 2'b11) ? 4'b0 : (in == 2'b00 ? 4'b0011:4'b1100);
endmodule
