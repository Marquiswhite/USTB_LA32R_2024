module mux_1(
    input  wire [1:0] mux_mem_size,
    input  wire [1:0] mux_addr,
    output reg [3:0] mux_result
);

    // 使用参数化的方式来简化逻辑
    always @(*) begin
        case (mux_mem_size)
            2'b01: begin // size_b
                case (mux_addr)
                    2'b00: mux_result = 4'b0001; // addr_0
                    2'b01: mux_result = 4'b0010; // addr_1
                    2'b10: mux_result = 4'b0100; // addr_2
                    2'b11: mux_result = 4'b1000; // addr_3
                    default: mux_result = 4'b0000;
                endcase
            end
            2'b10: begin // size_h
                case (mux_addr)
                    2'b00: mux_result = 4'b0011; // addr_0
                    2'b10: mux_result = 4'b1100; // addr_2
                    default: mux_result = 4'b0000;
                endcase
            end
            2'b11: begin // size_w
                mux_result = 4'b1111;
            end
            default: mux_result = 4'b0000; // 默认情况
        endcase
    end

endmodule
