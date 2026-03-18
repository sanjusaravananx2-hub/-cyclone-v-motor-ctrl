`timescale 1ns / 1ps
// =============================================================================
// axi_pwm_ip.v  —  AXI4-Lite Slave PWM Controller
// =============================================================================
// Wraps pwm_gen with an AXI4-Lite slave interface so the HPS ARM core can
// configure and control the servo PWM from C code.
//
// Accessed via the Lightweight HPS-to-FPGA Bridge (LWH2F):
//   Base address (HPS side): 0xFF200000
//
// Register Map  (word-aligned, 32-bit):
//   Offset 0x00  CTRL    [0] = enable   (1 = run PWM)
//                        [1] = sreset   (1 = synchronous reset counter)
//   Offset 0x04  PERIOD  Period in clock cycles  (default 1,000,000)
//   Offset 0x08  DUTY    Duty  in clock cycles   (default  75,000)
//   Offset 0x0C  STATUS  [0]  = current pwm_out level    (read-only)
//                        [1]  = irq_pending flag          (W1C)
//                        [31:2] reserved / zero
//
// Interrupt:
//   irq_out goes HIGH when irq_pending is set and is cleared by writing 1
//   to STATUS[1].  Connect to a GIC SPI on the HPS for a synchronised
//   PID wakeup without polling.
// =============================================================================
module axi_pwm_ip #(
    parameter CLK_FREQ_HZ          = 50_000_000,
    parameter DEFAULT_PERIOD       = 1_000_000,
    parameter DEFAULT_DUTY         =    75_000,
    parameter C_S_AXI_ADDR_WIDTH   = 4,
    parameter C_S_AXI_DATA_WIDTH   = 32
)(
    // ---- AXI4-Lite Slave -------------------------------------------------- //
    input  wire                              S_AXI_ACLK,
    input  wire                              S_AXI_ARESETN,

    // Write address
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]    S_AXI_AWADDR,
    input  wire                              S_AXI_AWVALID,
    output reg                               S_AXI_AWREADY,

    // Write data
    input  wire [C_S_AXI_DATA_WIDTH-1:0]    S_AXI_WDATA,
    input  wire [C_S_AXI_DATA_WIDTH/8-1:0]  S_AXI_WSTRB,
    input  wire                              S_AXI_WVALID,
    output reg                               S_AXI_WREADY,

    // Write response
    output reg  [1:0]                        S_AXI_BRESP,
    output reg                               S_AXI_BVALID,
    input  wire                              S_AXI_BREADY,

    // Read address
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]    S_AXI_ARADDR,
    input  wire                              S_AXI_ARVALID,
    output reg                               S_AXI_ARREADY,

    // Read data
    output reg  [C_S_AXI_DATA_WIDTH-1:0]    S_AXI_RDATA,
    output reg  [1:0]                        S_AXI_RRESP,
    output reg                               S_AXI_RVALID,
    input  wire                              S_AXI_RREADY,

    // ---- Outputs ---------------------------------------------------------- //
    output wire                              irq_out,
    output wire                              pwm_out
);

    // -------------------------------------------------------------------------
    // Internal registers
    // -------------------------------------------------------------------------
    reg [31:0] reg_ctrl;     // 0x00
    reg [31:0] reg_period;   // 0x04
    reg [31:0] reg_duty;     // 0x08
    reg [31:0] reg_status;   // 0x0C  (irq_pending latch)

    wire pwm_out_w;
    wire irq_period_w;

    // -------------------------------------------------------------------------
    // PWM core instantiation
    // -------------------------------------------------------------------------
    pwm_gen #(
        .CLK_FREQ_HZ    (CLK_FREQ_HZ),
        .DEFAULT_PERIOD (DEFAULT_PERIOD),
        .DEFAULT_DUTY   (DEFAULT_DUTY)
    ) u_pwm (
        .clk           (S_AXI_ACLK),
        .rst_n         (S_AXI_ARESETN & ~reg_ctrl[1]),
        .enable        (reg_ctrl[0]),
        .period_cycles (reg_period),
        .duty_cycles   (reg_duty),
        .pwm_out       (pwm_out_w),
        .irq_period    (irq_period_w)
    );

    assign pwm_out = pwm_out_w;
    assign irq_out = reg_status[1];   // level interrupt to GIC

    // -------------------------------------------------------------------------
    // IRQ pending latch – set by hardware, cleared by W1C from HPS
    // W1C condition: AXI write to STATUS register with bit[1] set
    // -------------------------------------------------------------------------
    wire w1c_clear = S_AXI_WREADY && S_AXI_WVALID
                     && (aw_addr_r[3:2] == 2'd3)
                     && S_AXI_WSTRB[0] && S_AXI_WDATA[1];

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN)
            reg_status[1] <= 1'b0;
        else if (w1c_clear)
            reg_status[1] <= 1'b0;
        else if (irq_period_w)
            reg_status[1] <= 1'b1;

        reg_status[0] <= pwm_out_w;   // live mirror of PWM output
    end

    // -------------------------------------------------------------------------
    // AXI4-Lite Write path
    // -------------------------------------------------------------------------
    reg [C_S_AXI_ADDR_WIDTH-1:0] aw_addr_r;

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            S_AXI_AWREADY <= 1'b0;
            S_AXI_WREADY  <= 1'b0;
            S_AXI_BVALID  <= 1'b0;
            S_AXI_BRESP   <= 2'b00;
            reg_ctrl       <= 32'd1;  // start enabled for standalone test
            reg_period     <= DEFAULT_PERIOD;
            reg_duty       <= DEFAULT_DUTY;
        end else begin

            // Accept write address
            if (S_AXI_AWVALID && !S_AXI_AWREADY) begin
                S_AXI_AWREADY <= 1'b1;
                aw_addr_r     <= S_AXI_AWADDR;
            end else begin
                S_AXI_AWREADY <= 1'b0;
            end

            // Accept write data and perform register write
            if (S_AXI_WVALID && !S_AXI_WREADY) begin
                S_AXI_WREADY <= 1'b1;

                case (aw_addr_r[3:2])
                    2'd0: begin   // CTRL
                        if (S_AXI_WSTRB[0]) reg_ctrl[7:0]   <= S_AXI_WDATA[7:0];
                    end
                    2'd1: begin   // PERIOD
                        if (S_AXI_WSTRB[0]) reg_period[7:0]   <= S_AXI_WDATA[7:0];
                        if (S_AXI_WSTRB[1]) reg_period[15:8]  <= S_AXI_WDATA[15:8];
                        if (S_AXI_WSTRB[2]) reg_period[23:16] <= S_AXI_WDATA[23:16];
                        if (S_AXI_WSTRB[3]) reg_period[31:24] <= S_AXI_WDATA[31:24];
                    end
                    2'd2: begin   // DUTY
                        if (S_AXI_WSTRB[0]) reg_duty[7:0]   <= S_AXI_WDATA[7:0];
                        if (S_AXI_WSTRB[1]) reg_duty[15:8]  <= S_AXI_WDATA[15:8];
                        if (S_AXI_WSTRB[2]) reg_duty[23:16] <= S_AXI_WDATA[23:16];
                        if (S_AXI_WSTRB[3]) reg_duty[31:24] <= S_AXI_WDATA[31:24];
                    end
                    2'd3: begin   // STATUS – W1C handled in IRQ latch block
                    end
                    default: ;
                endcase
            end else begin
                S_AXI_WREADY <= 1'b0;
            end

            // Write response
            if (S_AXI_WREADY && S_AXI_WVALID) begin
                S_AXI_BVALID <= 1'b1;
                S_AXI_BRESP  <= 2'b00; // OKAY
            end else if (S_AXI_BVALID && S_AXI_BREADY) begin
                S_AXI_BVALID <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // AXI4-Lite Read path
    // -------------------------------------------------------------------------
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            S_AXI_ARREADY <= 1'b0;
            S_AXI_RVALID  <= 1'b0;
            S_AXI_RRESP   <= 2'b00;
            S_AXI_RDATA   <= 32'd0;
        end else begin

            if (S_AXI_ARVALID && !S_AXI_ARREADY) begin
                S_AXI_ARREADY <= 1'b1;

                case (S_AXI_ARADDR[3:2])
                    2'd0:    S_AXI_RDATA <= reg_ctrl;
                    2'd1:    S_AXI_RDATA <= reg_period;
                    2'd2:    S_AXI_RDATA <= reg_duty;
                    2'd3:    S_AXI_RDATA <= reg_status;
                    default: S_AXI_RDATA <= 32'hDEADBEEF;
                endcase

                S_AXI_RVALID <= 1'b1;
                S_AXI_RRESP  <= 2'b00;
            end else begin
                S_AXI_ARREADY <= 1'b0;
            end

            if (S_AXI_RVALID && S_AXI_RREADY)
                S_AXI_RVALID <= 1'b0;
        end
    end

endmodule
