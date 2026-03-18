`timescale 1ns / 1ps
// =============================================================================
// pwm_gen.v  —  Precision PWM Generator for SG90 Servo
// =============================================================================
// Target  : Cyclone V SoC FPGA fabric
// Clock   : 50 MHz (from HPS PLL via FPGA fabric clock)
//
// SG90 Timing Reference:
//   Period  = 20.0 ms  → 1,000,000 cycles @ 50 MHz
//   Pulse   = 1.0 ms   →    50,000 cycles (  0°)
//   Pulse   = 1.5 ms   →    75,000 cycles ( 90°, center)
//   Pulse   = 2.0 ms   →   100,000 cycles (180°)
//
// The counter increments every clock cycle.  When counter < duty_cycles
// the output is HIGH; when counter >= duty_cycles it is LOW.
// An irq_period pulse (1-cycle wide) fires at counter wrap for the HPS
// to synchronise its control loop exactly to the servo frame rate.
// =============================================================================
module pwm_gen #(
    parameter CLK_FREQ_HZ    = 50_000_000,
    parameter DEFAULT_PERIOD = 1_000_000,   // 50 Hz
    parameter DEFAULT_DUTY   =    75_000    // 1.5 ms – safe centre position
)(
    input  wire        clk,
    input  wire        rst_n,          // active-low synchronous reset

    // Runtime configuration (written by AXI-Lite wrapper)
    input  wire        enable,
    input  wire [31:0] period_cycles,  // total frame length in clocks
    input  wire [31:0] duty_cycles,    // HIGH time in clocks

    // Outputs
    output reg         pwm_out,        // servo signal pin
    output reg         irq_period      // 1-cycle pulse at frame start
);

    reg [31:0] counter;

    always @(posedge clk) begin
        if (!rst_n) begin
            counter    <= 32'd0;
            pwm_out    <= 1'b0;
            irq_period <= 1'b0;
        end else if (!enable) begin
            counter    <= 32'd0;
            pwm_out    <= 1'b0;
            irq_period <= 1'b0;
        end else begin
            irq_period <= 1'b0;                         // default: deasserted

            if (counter >= (period_cycles - 32'd1)) begin
                counter    <= 32'd0;
                irq_period <= 1'b1;                     // notify HPS
            end else begin
                counter <= counter + 32'd1;
            end

            // Combinatorial-style in clocked block: avoids glitch on boundary
            pwm_out <= (counter < duty_cycles) ? 1'b1 : 1'b0;
        end
    end

endmodule
