`default_nettype wire

module corescore_emitter_uart (
i_clk,
i_rst,
i_data,
i_valid,
o_ready,
o_uart_tx
);

input i_clk;
input i_rst;
input [7:0] i_data;
input i_valid;
output o_ready;
output o_uart_tx;

assign o_ready = 1'b1;
assign o_uart_tx = 1'b1;

endmodule

`default_nettype none
