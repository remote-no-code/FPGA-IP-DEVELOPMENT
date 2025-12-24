`default_nettype none

module gpio_ip (
    input         clk,
    input         resetn,
    input         we,        // write enable
    input         re,        // read enable
    input  [31:0] wdata,
    output [31:0] rdata,
    output [31:0] gpio_out
);

    // ------------------------------------------------------------
    // GPIO register
    // ------------------------------------------------------------
    reg [31:0] gpio_reg;

    // ------------------------------------------------------------
    // Write logic
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            gpio_reg <= 32'b0;
        end else if (we) begin
            gpio_reg <= wdata;
        end
    end

    // ------------------------------------------------------------
    // Read logic
    // ------------------------------------------------------------
    assign rdata = re ? gpio_reg : 32'b0;

    // ------------------------------------------------------------
    // GPIO output
    // ------------------------------------------------------------
    assign gpio_out = gpio_reg;

endmodule
