`timescale 1ns / 1ps
// =============================================================================
// top.v  —  FPGA Top-Level  (Cyclone V SoC, DE1-SoC)
// =============================================================================
// Connects the HPS hard-macro (via GHRD soc_system) to the custom AXI PWM IP.
//
// Pin assignments for DE1-SoC:
//   GPIO_0[0]  → SG90 signal wire (servo PWM out, 3.3 V)  [JP1 pin 1]
//   KEY[0]     → Active-low reset
//   LEDR[0]    → PWM output mirror (visual feedback)
//   LEDR[9]    → IRQ pending indicator
// =============================================================================
module top (
    // ---- Clock / Reset ---------------------------------------------------- //
    input  wire        CLOCK_50,

    // ---- HPS DDR3 --------------------------------------------------------- //
    output wire [14:0] HPS_DDR3_ADDR,
    output wire [2:0]  HPS_DDR3_BA,
    output wire        HPS_DDR3_CAS_N,
    output wire        HPS_DDR3_CKE,
    output wire        HPS_DDR3_CK_N,
    output wire        HPS_DDR3_CK_P,
    output wire        HPS_DDR3_CS_N,
    inout  wire [3:0]  HPS_DDR3_DM,
    inout  wire [31:0] HPS_DDR3_DQ,
    inout  wire [3:0]  HPS_DDR3_DQS_N,
    inout  wire [3:0]  HPS_DDR3_DQS_P,
    output wire        HPS_DDR3_ODT,
    output wire        HPS_DDR3_RAS_N,
    output wire        HPS_DDR3_RESET_N,
    input  wire        HPS_DDR3_RZQ,
    output wire        HPS_DDR3_WE_N,

    // ---- HPS Ethernet ----------------------------------------------------- //
    output wire        HPS_ENET_GTX_CLK,
    output wire        HPS_ENET_MDC,
    inout  wire        HPS_ENET_MDIO,
    input  wire        HPS_ENET_RX_CLK,
    input  wire        HPS_ENET_RX_DATA0,
    input  wire        HPS_ENET_RX_DATA1,
    input  wire        HPS_ENET_RX_DATA2,
    input  wire        HPS_ENET_RX_DATA3,
    input  wire        HPS_ENET_RX_DV,
    output wire        HPS_ENET_TX_DATA0,
    output wire        HPS_ENET_TX_DATA1,
    output wire        HPS_ENET_TX_DATA2,
    output wire        HPS_ENET_TX_DATA3,
    output wire        HPS_ENET_TX_EN,

    // ---- HPS QSPI Flash -------------------------------------------------- //
    inout  wire        HPS_FLASH_DATA0,
    inout  wire        HPS_FLASH_DATA1,
    inout  wire        HPS_FLASH_DATA2,
    inout  wire        HPS_FLASH_DATA3,
    output wire        HPS_FLASH_DCLK,
    output wire        HPS_FLASH_NCSO,

    // ---- HPS SD Card ------------------------------------------------------ //
    inout  wire        HPS_SD_CMD,
    inout  wire        HPS_SD_DATA0,
    inout  wire        HPS_SD_DATA1,
    inout  wire        HPS_SD_DATA2,
    inout  wire        HPS_SD_DATA3,
    output wire        HPS_SD_CLK,

    // ---- HPS USB ---------------------------------------------------------- //
    inout  wire        HPS_USB_DATA0,
    inout  wire        HPS_USB_DATA1,
    inout  wire        HPS_USB_DATA2,
    inout  wire        HPS_USB_DATA3,
    inout  wire        HPS_USB_DATA4,
    inout  wire        HPS_USB_DATA5,
    inout  wire        HPS_USB_DATA6,
    inout  wire        HPS_USB_DATA7,
    input  wire        HPS_USB_CLKOUT,
    output wire        HPS_USB_STP,
    input  wire        HPS_USB_DIR,
    input  wire        HPS_USB_NXT,

    // ---- HPS SPI ---------------------------------------------------------- //
    output wire        HPS_SPIM_CLK,
    output wire        HPS_SPIM_MOSI,
    input  wire        HPS_SPIM_MISO,
    output wire        HPS_SPIM_SS,

    // ---- HPS UART --------------------------------------------------------- //
    input  wire        HPS_UART_RX,
    output wire        HPS_UART_TX,

    // ---- HPS I2C ---------------------------------------------------------- //
    inout  wire        HPS_I2C0_SDAT,
    inout  wire        HPS_I2C0_SCLK,
    inout  wire        HPS_I2C1_SDAT,
    inout  wire        HPS_I2C1_SCLK,

    // ---- HPS GPIO --------------------------------------------------------- //
    inout  wire        HPS_CONV_USB_N,
    inout  wire        HPS_ENET_INT_N,
    inout  wire        HPS_LTC_GPIO,
    inout  wire        HPS_I2C_CONTROL,
    inout  wire        HPS_LED,
    inout  wire        HPS_KEY,
    inout  wire        HPS_GSENSOR_INT,

    // ---- User I/O --------------------------------------------------------- //
    output wire [9:0]  LEDR,
    input  wire [3:0]  KEY,
    input  wire [9:0]  SW,
    inout  wire [35:0] GPIO_0,
    inout  wire [35:0] GPIO_1
);

    // -------------------------------------------------------------------------
    // Internal wires
    // -------------------------------------------------------------------------
    wire        clk_50;
    wire        hps_reset_n;

    // LWH2F bridge wires (AXI, from HPS to FPGA)
    wire [31:0]  lwh2f_awaddr;
    wire         lwh2f_awvalid;
    wire         lwh2f_awready;
    wire [31:0]  lwh2f_wdata;
    wire [3:0]   lwh2f_wstrb;
    wire         lwh2f_wvalid;
    wire         lwh2f_wready;
    wire [1:0]   lwh2f_bresp;
    wire         lwh2f_bvalid;
    wire         lwh2f_bready;
    wire [31:0]  lwh2f_araddr;
    wire         lwh2f_arvalid;
    wire         lwh2f_arready;
    wire [31:0]  lwh2f_rdata;
    wire [1:0]   lwh2f_rresp;
    wire         lwh2f_rvalid;
    wire         lwh2f_rready;

    wire         pwm_signal;
    wire         pwm_irq;

    assign clk_50 = CLOCK_50;

    // -------------------------------------------------------------------------
    // AXI PWM IP  (mapped at LWH2F offset 0x0000)
    // -------------------------------------------------------------------------
    axi_pwm_ip #(
        .CLK_FREQ_HZ    (50_000_000),
        .DEFAULT_PERIOD (1_000_000),    // 50 Hz
        .DEFAULT_DUTY   (   75_000)     // 1.5 ms — safe power-on position
    ) u_pwm_ip (
        .S_AXI_ACLK     (clk_50),
        .S_AXI_ARESETN  (KEY[0]),  // bypass hps_reset_n for standalone test

        .S_AXI_AWADDR   (lwh2f_awaddr[3:0]),
        .S_AXI_AWVALID  (lwh2f_awvalid),
        .S_AXI_AWREADY  (lwh2f_awready),

        .S_AXI_WDATA    (lwh2f_wdata),
        .S_AXI_WSTRB    (lwh2f_wstrb),
        .S_AXI_WVALID   (lwh2f_wvalid),
        .S_AXI_WREADY   (lwh2f_wready),

        .S_AXI_BRESP    (lwh2f_bresp),
        .S_AXI_BVALID   (lwh2f_bvalid),
        .S_AXI_BREADY   (lwh2f_bready),

        .S_AXI_ARADDR   (lwh2f_araddr[3:0]),
        .S_AXI_ARVALID  (lwh2f_arvalid),
        .S_AXI_ARREADY  (lwh2f_arready),

        .S_AXI_RDATA    (lwh2f_rdata),
        .S_AXI_RRESP    (lwh2f_rresp),
        .S_AXI_RVALID   (lwh2f_rvalid),
        .S_AXI_RREADY   (lwh2f_rready),

        .irq_out        (pwm_irq),
        .pwm_out        (pwm_signal)
    );

    // -------------------------------------------------------------------------
    // Servo control with switches, sweep, and preset angles
    // -------------------------------------------------------------------------
    // SW[9:0] = manual angle control (0-180 degrees)
    // KEY[1]  = auto sweep (10 to 170 degrees, back and forth)
    // KEY[2]  = preset 0 degrees
    // KEY[3]  = preset 180 degrees
    // -------------------------------------------------------------------------

    // Switch-based duty: maps SW 0-1023 to 50000-100127 cycles
    wire [31:0] sw_duty;
    assign sw_duty = 32'd50_000 + ({22'd0, SW[9:0]} * 32'd49);

    // Auto-sweep state machine
    reg [31:0] sweep_duty;
    reg        sweep_dir;       // 0 = increasing, 1 = decreasing
    reg        sweep_active;
    reg [19:0] sweep_timer;     // slows down the sweep speed

    always @(posedge clk_50) begin
        if (!KEY[0]) begin
            sweep_duty   <= 32'd75_000;  // 90 degrees
            sweep_dir    <= 1'b0;
            sweep_active <= 1'b0;
            sweep_timer  <= 20'd0;
        end else begin
            // KEY[1] toggles sweep mode (active low, detect falling edge)
            if (!KEY[1])
                sweep_active <= ~sweep_active;

            if (sweep_active) begin
                sweep_timer <= sweep_timer + 20'd1;
                if (sweep_timer == 20'd0) begin  // update every ~20ms
                    if (!sweep_dir) begin
                        if (sweep_duty < 32'd97_000)  // ~170 degrees
                            sweep_duty <= sweep_duty + 32'd500;
                        else
                            sweep_dir <= 1'b1;
                    end else begin
                        if (sweep_duty > 32'd53_000)  // ~10 degrees
                            sweep_duty <= sweep_duty - 32'd500;
                        else
                            sweep_dir <= 1'b0;
                    end
                end
            end
        end
    end

    // Duty cycle mux: sweep mode or switch mode, with button presets
    reg [31:0] final_duty;
    always @(*) begin
        if (!KEY[2])
            final_duty = 32'd50_000;      // KEY[2] = 0 degrees
        else if (!KEY[3])
            final_duty = 32'd100_000;     // KEY[3] = 180 degrees
        else if (sweep_active)
            final_duty = sweep_duty;       // auto sweep
        else
            final_duty = sw_duty;          // switch control
    end

    wire test_pwm_out;
    wire test_irq;
    pwm_gen #(
        .CLK_FREQ_HZ    (50_000_000),
        .DEFAULT_PERIOD (1_000_000),
        .DEFAULT_DUTY   (   75_000)
    ) u_pwm_test (
        .clk           (clk_50),
        .rst_n         (KEY[0]),
        .enable        (1'b1),
        .period_cycles (32'd1_000_000),
        .duty_cycles   (final_duty),
        .pwm_out       (test_pwm_out),
        .irq_period    (test_irq)
    );

    // Servo moved to GPIO_1[0] (see below)

    // Diagnostic LEDs
    reg [25:0] blink_cnt;
    always @(posedge clk_50) begin
        if (!KEY[0])
            blink_cnt <= 26'd0;
        else
            blink_cnt <= blink_cnt + 26'd1;
    end

    assign LEDR[0]    = 1'b1;           // design loaded
    assign LEDR[1]    = test_pwm_out;   // PWM signal
    assign LEDR[2]    = blink_cnt[25];  // clock alive
    assign LEDR[3]    = sweep_active;   // sweep mode ON
    assign LEDR[9:4]  = SW[9:4];        // show switch state

    // -------------------------------------------------------------------------
    // Angle calculation for display (duty 50000-100000 → 0-180)
    // -------------------------------------------------------------------------
    wire [7:0] display_angle;
    wire [31:0] duty_offset = (final_duty > 32'd50_000) ? (final_duty - 32'd50_000) : 32'd0;
    wire [31:0] angle_raw   = (duty_offset * 32'd180) / 32'd50_000;
    assign display_angle = (angle_raw > 32'd180) ? 8'd180 : angle_raw[7:0];

    // -------------------------------------------------------------------------
    // LT24 LCD Display (GPIO_1 / JP2)
    // -------------------------------------------------------------------------
    wire [15:0] lcd_data;
    wire        lcd_rs, lcd_cs_n, lcd_wr_n, lcd_rd_n, lcd_reset_n, lcd_on;

    lt24_driver u_lt24 (
        .clk        (clk_50),
        .rst_n      (KEY[0]),
        .angle      (display_angle),
        .sweep_on   (sweep_active),
        .lcd_data   (lcd_data),
        .lcd_rs     (lcd_rs),
        .lcd_cs_n   (lcd_cs_n),
        .lcd_wr_n   (lcd_wr_n),
        .lcd_rd_n   (lcd_rd_n),
        .lcd_reset_n(lcd_reset_n),
        .lcd_on     (lcd_on)
    );

    // LT24 on GPIO_0 (JP1) — verified from Leeds Pong project pin mapping:
    //   GPIO_0[8]     = LT24Data[0]     GPIO_0[17]    = LT24Data[8]
    //   GPIO_0[7]     = LT24Data[1]     GPIO_0[18]    = LT24Data[9]
    //   GPIO_0[6]     = LT24Data[2]     GPIO_0[19]    = LT24Data[10]
    //   GPIO_0[5]     = LT24Data[3]     GPIO_0[20]    = LT24Data[11]
    //   GPIO_0[13]    = LT24Data[4]     GPIO_0[21]    = LT24Data[12]
    //   GPIO_0[14]    = LT24Data[5]     GPIO_0[22]    = LT24Data[13]
    //   GPIO_0[15]    = LT24Data[6]     GPIO_0[23]    = LT24Data[14]
    //   GPIO_0[16]    = LT24Data[7]     GPIO_0[24]    = LT24Data[15]
    //   GPIO_0[10]    = LT24Rd_n        GPIO_0[25]    = LT24CS_n
    //   GPIO_0[11]    = LT24Wr_n        GPIO_0[33]    = LT24Reset_n
    //   GPIO_0[12]    = LT24RS          GPIO_0[35]    = LT24LCDOn
    assign GPIO_0[8]  = lcd_data[0];
    assign GPIO_0[7]  = lcd_data[1];
    assign GPIO_0[6]  = lcd_data[2];
    assign GPIO_0[5]  = lcd_data[3];
    assign GPIO_0[13] = lcd_data[4];
    assign GPIO_0[14] = lcd_data[5];
    assign GPIO_0[15] = lcd_data[6];
    assign GPIO_0[16] = lcd_data[7];
    assign GPIO_0[17] = lcd_data[8];
    assign GPIO_0[18] = lcd_data[9];
    assign GPIO_0[19] = lcd_data[10];
    assign GPIO_0[20] = lcd_data[11];
    assign GPIO_0[21] = lcd_data[12];
    assign GPIO_0[22] = lcd_data[13];
    assign GPIO_0[23] = lcd_data[14];
    assign GPIO_0[24] = lcd_data[15];
    assign GPIO_0[10] = lcd_rd_n;
    assign GPIO_0[11] = lcd_wr_n;
    assign GPIO_0[12] = lcd_rs;
    assign GPIO_0[25] = lcd_cs_n;
    assign GPIO_0[33] = lcd_reset_n;
    assign GPIO_0[35] = lcd_on;

    // Servo now on GPIO_1[0] (JP2 pin 1)
    assign GPIO_1[0] = test_pwm_out;

    // -------------------------------------------------------------------------
    // HPS (GHRD soc_system generated by Platform Designer)
    // -------------------------------------------------------------------------
    soc_system u_soc (
        .clk_clk                               (clk_50),
        .reset_reset_n                         (KEY[0]),
        .hps_0_h2f_reset_reset_n               (hps_reset_n),

        // FPGA-to-HPS cold/warm/debug reset (active-low, active-low, active-low)
        .hps_0_f2h_cold_reset_req_reset_n      (1'b1),
        .hps_0_f2h_debug_reset_req_reset_n     (1'b1),
        .hps_0_f2h_warm_reset_req_reset_n      (1'b1),

        // STM hardware events (unused)
        .hps_0_f2h_stm_hw_events_stm_hwevents  (27'b0),

        // LWH2F AXI master → our PWM IP
        .hps_0_h2f_lw_axi_master_awid          (),
        .hps_0_h2f_lw_axi_master_awaddr        (lwh2f_awaddr),
        .hps_0_h2f_lw_axi_master_awlen         (),
        .hps_0_h2f_lw_axi_master_awsize        (),
        .hps_0_h2f_lw_axi_master_awburst       (),
        .hps_0_h2f_lw_axi_master_awlock        (),
        .hps_0_h2f_lw_axi_master_awcache       (),
        .hps_0_h2f_lw_axi_master_awprot        (),
        .hps_0_h2f_lw_axi_master_awvalid       (lwh2f_awvalid),
        .hps_0_h2f_lw_axi_master_awready       (lwh2f_awready),
        .hps_0_h2f_lw_axi_master_wid           (),
        .hps_0_h2f_lw_axi_master_wdata         (lwh2f_wdata),
        .hps_0_h2f_lw_axi_master_wstrb         (lwh2f_wstrb),
        .hps_0_h2f_lw_axi_master_wlast         (),
        .hps_0_h2f_lw_axi_master_wvalid        (lwh2f_wvalid),
        .hps_0_h2f_lw_axi_master_wready        (lwh2f_wready),
        .hps_0_h2f_lw_axi_master_bid           (12'b0),
        .hps_0_h2f_lw_axi_master_bresp         (lwh2f_bresp),
        .hps_0_h2f_lw_axi_master_bvalid        (lwh2f_bvalid),
        .hps_0_h2f_lw_axi_master_bready        (lwh2f_bready),
        .hps_0_h2f_lw_axi_master_arid          (),
        .hps_0_h2f_lw_axi_master_araddr        (lwh2f_araddr),
        .hps_0_h2f_lw_axi_master_arlen         (),
        .hps_0_h2f_lw_axi_master_arsize        (),
        .hps_0_h2f_lw_axi_master_arburst       (),
        .hps_0_h2f_lw_axi_master_arlock        (),
        .hps_0_h2f_lw_axi_master_arcache       (),
        .hps_0_h2f_lw_axi_master_arprot        (),
        .hps_0_h2f_lw_axi_master_arvalid       (lwh2f_arvalid),
        .hps_0_h2f_lw_axi_master_arready       (lwh2f_arready),
        .hps_0_h2f_lw_axi_master_rid           (12'b0),
        .hps_0_h2f_lw_axi_master_rdata         (lwh2f_rdata),
        .hps_0_h2f_lw_axi_master_rresp         (lwh2f_rresp),
        .hps_0_h2f_lw_axi_master_rlast         (1'b1),
        .hps_0_h2f_lw_axi_master_rvalid        (lwh2f_rvalid),
        .hps_0_h2f_lw_axi_master_rready        (lwh2f_rready),

        // FPGA-to-HPS interrupt (PWM frame IRQ → GIC SPI #72)
        .hps_0_f2h_irq0_irq                    ({31'b0, pwm_irq}),

        // DDR3
        .memory_mem_a                          (HPS_DDR3_ADDR),
        .memory_mem_ba                         (HPS_DDR3_BA),
        .memory_mem_ck                         (HPS_DDR3_CK_P),
        .memory_mem_ck_n                       (HPS_DDR3_CK_N),
        .memory_mem_cke                        (HPS_DDR3_CKE),
        .memory_mem_cs_n                       (HPS_DDR3_CS_N),
        .memory_mem_ras_n                      (HPS_DDR3_RAS_N),
        .memory_mem_cas_n                      (HPS_DDR3_CAS_N),
        .memory_mem_we_n                       (HPS_DDR3_WE_N),
        .memory_mem_reset_n                    (HPS_DDR3_RESET_N),
        .memory_mem_dq                         (HPS_DDR3_DQ),
        .memory_mem_dqs                        (HPS_DDR3_DQS_P),
        .memory_mem_dqs_n                      (HPS_DDR3_DQS_N),
        .memory_mem_odt                        (HPS_DDR3_ODT),
        .memory_mem_dm                         (HPS_DDR3_DM),
        .memory_oct_rzqin                      (HPS_DDR3_RZQ),

        // HPS I/O (active peripherals from GHRD)
        .hps_0_hps_io_hps_io_emac1_inst_TX_CLK (HPS_ENET_GTX_CLK),
        .hps_0_hps_io_hps_io_emac1_inst_TXD0   (HPS_ENET_TX_DATA0),
        .hps_0_hps_io_hps_io_emac1_inst_TXD1   (HPS_ENET_TX_DATA1),
        .hps_0_hps_io_hps_io_emac1_inst_TXD2   (HPS_ENET_TX_DATA2),
        .hps_0_hps_io_hps_io_emac1_inst_TXD3   (HPS_ENET_TX_DATA3),
        .hps_0_hps_io_hps_io_emac1_inst_RXD0   (HPS_ENET_RX_DATA0),
        .hps_0_hps_io_hps_io_emac1_inst_MDIO   (HPS_ENET_MDIO),
        .hps_0_hps_io_hps_io_emac1_inst_MDC    (HPS_ENET_MDC),
        .hps_0_hps_io_hps_io_emac1_inst_RX_CTL (HPS_ENET_RX_DV),
        .hps_0_hps_io_hps_io_emac1_inst_TX_CTL (HPS_ENET_TX_EN),
        .hps_0_hps_io_hps_io_emac1_inst_RX_CLK (HPS_ENET_RX_CLK),
        .hps_0_hps_io_hps_io_emac1_inst_RXD1   (HPS_ENET_RX_DATA1),
        .hps_0_hps_io_hps_io_emac1_inst_RXD2   (HPS_ENET_RX_DATA2),
        .hps_0_hps_io_hps_io_emac1_inst_RXD3   (HPS_ENET_RX_DATA3),
        .hps_0_hps_io_hps_io_qspi_inst_IO0     (HPS_FLASH_DATA0),
        .hps_0_hps_io_hps_io_qspi_inst_IO1     (HPS_FLASH_DATA1),
        .hps_0_hps_io_hps_io_qspi_inst_IO2     (HPS_FLASH_DATA2),
        .hps_0_hps_io_hps_io_qspi_inst_IO3     (HPS_FLASH_DATA3),
        .hps_0_hps_io_hps_io_qspi_inst_SS0     (HPS_FLASH_NCSO),
        .hps_0_hps_io_hps_io_qspi_inst_CLK     (HPS_FLASH_DCLK),
        .hps_0_hps_io_hps_io_sdio_inst_CMD     (HPS_SD_CMD),
        .hps_0_hps_io_hps_io_sdio_inst_D0      (HPS_SD_DATA0),
        .hps_0_hps_io_hps_io_sdio_inst_D1      (HPS_SD_DATA1),
        .hps_0_hps_io_hps_io_sdio_inst_CLK     (HPS_SD_CLK),
        .hps_0_hps_io_hps_io_sdio_inst_D2      (HPS_SD_DATA2),
        .hps_0_hps_io_hps_io_sdio_inst_D3      (HPS_SD_DATA3),
        .hps_0_hps_io_hps_io_usb1_inst_D0      (HPS_USB_DATA0),
        .hps_0_hps_io_hps_io_usb1_inst_D1      (HPS_USB_DATA1),
        .hps_0_hps_io_hps_io_usb1_inst_D2      (HPS_USB_DATA2),
        .hps_0_hps_io_hps_io_usb1_inst_D3      (HPS_USB_DATA3),
        .hps_0_hps_io_hps_io_usb1_inst_D4      (HPS_USB_DATA4),
        .hps_0_hps_io_hps_io_usb1_inst_D5      (HPS_USB_DATA5),
        .hps_0_hps_io_hps_io_usb1_inst_D6      (HPS_USB_DATA6),
        .hps_0_hps_io_hps_io_usb1_inst_D7      (HPS_USB_DATA7),
        .hps_0_hps_io_hps_io_usb1_inst_CLK     (HPS_USB_CLKOUT),
        .hps_0_hps_io_hps_io_usb1_inst_STP     (HPS_USB_STP),
        .hps_0_hps_io_hps_io_usb1_inst_DIR     (HPS_USB_DIR),
        .hps_0_hps_io_hps_io_usb1_inst_NXT     (HPS_USB_NXT),
        .hps_0_hps_io_hps_io_spim1_inst_CLK    (HPS_SPIM_CLK),
        .hps_0_hps_io_hps_io_spim1_inst_MOSI   (HPS_SPIM_MOSI),
        .hps_0_hps_io_hps_io_spim1_inst_MISO   (HPS_SPIM_MISO),
        .hps_0_hps_io_hps_io_spim1_inst_SS0    (HPS_SPIM_SS),
        .hps_0_hps_io_hps_io_uart0_inst_RX     (HPS_UART_RX),
        .hps_0_hps_io_hps_io_uart0_inst_TX     (HPS_UART_TX),
        .hps_0_hps_io_hps_io_i2c0_inst_SDA     (HPS_I2C0_SDAT),
        .hps_0_hps_io_hps_io_i2c0_inst_SCL     (HPS_I2C0_SCLK),
        .hps_0_hps_io_hps_io_i2c1_inst_SDA     (HPS_I2C1_SDAT),
        .hps_0_hps_io_hps_io_i2c1_inst_SCL     (HPS_I2C1_SCLK),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO09  (HPS_CONV_USB_N),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO35  (HPS_ENET_INT_N),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO40  (HPS_LTC_GPIO),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO48  (HPS_I2C_CONTROL),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO53  (HPS_LED),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO54  (HPS_KEY),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO61  (HPS_GSENSOR_INT)
    );

endmodule
