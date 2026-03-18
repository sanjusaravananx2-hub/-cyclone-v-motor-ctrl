// ============================================================================
// LT24 LCD Driver (ILI9341, 240x320, 16-bit parallel)
// Minimal version: init + solid colour fill to verify hardware works
// ============================================================================
module lt24_driver (
    input  wire        clk,        // 50 MHz
    input  wire        rst_n,
    input  wire [7:0]  angle,      // 0-180
    input  wire        sweep_on,   // sweep mode active

    // LT24 parallel interface
    output reg  [15:0] lcd_data,
    output reg         lcd_rs,     // 0=command, 1=data
    output reg         lcd_cs_n,
    output reg         lcd_wr_n,
    output wire        lcd_rd_n,
    output reg         lcd_reset_n,
    output wire        lcd_on
);

    assign lcd_on   = 1'b1;
    assign lcd_rd_n = 1'b1;

    // ---- Hardware reset: hold low 10ms, then release ----
    reg [19:0] reset_cnt;
    reg        reset_done;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reset_cnt   <= 20'd0;
            reset_done  <= 1'b0;
            lcd_reset_n <= 1'b0;
        end else if (!reset_done) begin
            reset_cnt <= reset_cnt + 20'd1;
            if (reset_cnt == 20'd500_000) begin
                lcd_reset_n <= 1'b1;
                reset_done  <= 1'b1;
            end
        end
    end

    // ---- Write cycle: setup data, then pulse WR low ----
    // Phase 0: setup (CS low, data valid)
    // Phase 1: WR low
    // Phase 2: WR low (hold)
    // Phase 3: WR low (hold)
    // Phase 4: WR high (latch)
    // Phase 5: done
    reg [2:0] wr_cnt;
    reg       wr_start;
    wire      wr_busy = (wr_cnt != 3'd0) || wr_start;
    wire      wr_done = (wr_cnt == 3'd5);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_cnt   <= 3'd0;
            lcd_wr_n <= 1'b1;
            lcd_cs_n <= 1'b0;  // CS always low (single device)
        end else if (wr_start && wr_cnt == 3'd0) begin
            wr_cnt   <= 3'd1;
        end else if (wr_cnt == 3'd1) begin
            lcd_wr_n <= 1'b0;
            wr_cnt   <= 3'd2;
        end else if (wr_cnt == 3'd2) begin
            wr_cnt <= 3'd3;  // hold
        end else if (wr_cnt == 3'd3) begin
            wr_cnt <= 3'd4;  // hold
        end else if (wr_cnt == 3'd4) begin
            lcd_wr_n <= 1'b1;  // rising edge latches data
            wr_cnt   <= 3'd5;
        end else if (wr_cnt == 3'd5) begin
            wr_cnt   <= 3'd0;
        end
    end

    // ---- State machine ----
    localparam S_WAIT_RESET  = 4'd0;
    localparam S_POST_RESET  = 4'd1;
    localparam S_SEND_CMD    = 4'd2;
    localparam S_WAIT_WR     = 4'd3;
    localparam S_DELAY       = 4'd4;
    localparam S_FILL_PIXEL  = 4'd5;
    localparam S_FILL_WAIT   = 4'd6;

    reg [3:0]  state;
    reg [3:0]  after_wr;     // state to go to after write completes
    reg [3:0]  after_delay;  // state to go to after delay
    reg [23:0] delay_cnt;
    reg [5:0]  cmd_idx;
    reg [16:0] pixel_cnt;

    // Latched angle for current frame
    reg [7:0]  draw_angle;
    reg        draw_sweep;

    // ---- Init + window command table ----
    // {rs, data[7:0]}  rs=0 for command, rs=1 for data
    // Special: idx with value 9'h1FF = insert 120ms delay
    localparam CMD_LEN = 6'd30;
    reg [8:0] cmd_rom;
    always @(*) begin
        case (cmd_idx)
            // Software reset
            6'd0:  cmd_rom = {1'b0, 8'h01};
            6'd1:  cmd_rom = 9'h1FF;          // delay 120ms
            // Sleep out
            6'd2:  cmd_rom = {1'b0, 8'h11};
            6'd3:  cmd_rom = 9'h1FF;          // delay 120ms
            // Pixel format 16-bit
            6'd4:  cmd_rom = {1'b0, 8'h3A};
            6'd5:  cmd_rom = {1'b1, 8'h55};
            // MADCTL: landscape (MX+MV=0x60 or MV+MY=0xA0, try 0x28 = MV)
            6'd6:  cmd_rom = {1'b0, 8'h36};
            6'd7:  cmd_rom = {1'b1, 8'h28};
            // Display ON
            6'd8:  cmd_rom = {1'b0, 8'h29};
            6'd9:  cmd_rom = 9'h1FF;          // delay 50ms
            // Column address set (0 to 319 = 0x013F)
            6'd10: cmd_rom = {1'b0, 8'h2A};
            6'd11: cmd_rom = {1'b1, 8'h00};
            6'd12: cmd_rom = {1'b1, 8'h00};
            6'd13: cmd_rom = {1'b1, 8'h01};
            6'd14: cmd_rom = {1'b1, 8'h3F};
            // Row address set (0 to 239 = 0x00EF)
            6'd15: cmd_rom = {1'b0, 8'h2B};
            6'd16: cmd_rom = {1'b1, 8'h00};
            6'd17: cmd_rom = {1'b1, 8'h00};
            6'd18: cmd_rom = {1'b1, 8'h00};
            6'd19: cmd_rom = {1'b1, 8'hEF};
            // Memory write
            6'd20: cmd_rom = {1'b0, 8'h2C};
            default: cmd_rom = 9'd0;
        endcase
    end

    // ---- Pixel colour ----
    // Needle position: angle 0-180 maps to columns 20-299 (280px range)
    wire [15:0] bar_mul = {8'd0, draw_angle} * 16'd199;
    wire [8:0]  needle_col = 9'd20 + bar_mul[15:7];

    // Pixel counters
    reg [8:0] px_col;
    reg [7:0] px_row;

    // ---- 3x5 digit font for angle display ----
    // Each digit is 3 columns x 5 rows, stored as 15 bits
    reg [14:0] digit_bitmap;
    reg [3:0] digit_val;
    always @(*) begin
        case (digit_val)
            4'd0: digit_bitmap = 15'b111_101_101_101_111;
            4'd1: digit_bitmap = 15'b010_110_010_010_111;
            4'd2: digit_bitmap = 15'b111_001_111_100_111;
            4'd3: digit_bitmap = 15'b111_001_111_001_111;
            4'd4: digit_bitmap = 15'b101_101_111_001_001;
            4'd5: digit_bitmap = 15'b111_100_111_001_111;
            4'd6: digit_bitmap = 15'b111_100_111_101_111;
            4'd7: digit_bitmap = 15'b111_001_001_001_001;
            4'd8: digit_bitmap = 15'b111_101_111_101_111;
            4'd9: digit_bitmap = 15'b111_101_111_001_111;
            default: digit_bitmap = 15'b000_000_000_000_000;
        endcase
    end

    // Digit decomposition: hundreds, tens, ones
    wire [3:0] d_hundreds = (draw_angle >= 8'd100) ? 4'd1 : 4'd0;
    wire [7:0] after_hundreds = draw_angle - (d_hundreds ? 8'd100 : 8'd0);
    wire [3:0] d_tens = (after_hundreds >= 8'd90) ? 4'd9 :
                        (after_hundreds >= 8'd80) ? 4'd8 :
                        (after_hundreds >= 8'd70) ? 4'd7 :
                        (after_hundreds >= 8'd60) ? 4'd6 :
                        (after_hundreds >= 8'd50) ? 4'd5 :
                        (after_hundreds >= 8'd40) ? 4'd4 :
                        (after_hundreds >= 8'd30) ? 4'd3 :
                        (after_hundreds >= 8'd20) ? 4'd2 :
                        (after_hundreds >= 8'd10) ? 4'd1 : 4'd0;
    wire [3:0] d_ones = after_hundreds - (d_tens * 4'd10);

    // Digit display area: 3 digits at (col 140-170, row 30-54), scale 2x
    // Each digit: 6px wide (3*2), 10px tall (5*2), 2px gap between
    wire in_digit_area = (px_row >= 8'd30) && (px_row < 8'd50) &&
                         (px_col >= 9'd140) && (px_col < 9'd164);
    wire [8:0] digit_x = px_col - 9'd140;  // 0-23
    wire [7:0] digit_y = px_row - 8'd30;   // 0-19

    // Which digit (0=hundreds, 1=tens, 2=ones)
    wire [1:0] which_digit = (digit_x < 9'd6) ? 2'd0 :
                             (digit_x < 9'd14 && digit_x >= 9'd8) ? 2'd1 :
                             (digit_x >= 9'd16 && digit_x < 9'd22) ? 2'd2 : 2'd3;

    // Pixel within the digit (0-2 col, 0-4 row, scaled 2x)
    wire [1:0] font_col = (digit_x < 9'd6) ? digit_x[2:1] :
                           (digit_x < 9'd14) ? (digit_x - 9'd8) >> 1 :
                           (digit_x - 9'd16) >> 1;
    wire [2:0] font_row = digit_y[3:1];  // /2

    // Select digit value
    always @(*) begin
        case (which_digit)
            2'd0: digit_val = d_hundreds;
            2'd1: digit_val = d_tens;
            2'd2: digit_val = d_ones;
            default: digit_val = 4'd15; // blank
        endcase
    end

    // Font pixel lookup: row 0 = bits 14,13,12; row 1 = 11,10,9; etc.
    wire [3:0] bit_idx = (4'd4 - font_row) * 4'd3 + (4'd2 - font_col);
    wire font_pixel = (which_digit != 2'd3) && digit_bitmap[bit_idx];

    // ---- Gauge elements ----
    // Gauge track
    wire in_gauge  = (px_row >= 8'd90) && (px_row <= 8'd149);
    wire in_track  = in_gauge && (px_col >= 9'd20) && (px_col <= 9'd299);

    // Needle (6px wide)
    wire at_needle = in_gauge && (px_col >= needle_col) && (px_col < (needle_col + 9'd6));

    // Needle glow (2px on each side)
    wire at_glow = in_gauge && !at_needle &&
        (px_col + 9'd2 >= needle_col) && (px_col < (needle_col + 9'd8));

    // Major ticks (at 0,90,180) — tall
    wire at_major_tick = (px_row >= 8'd85) && (px_row <= 8'd154) &&
        (px_col == 9'd20 || px_col == 9'd159 || px_col == 9'd299);

    // Minor ticks (at 30,60,120,150) — short
    wire at_minor_tick = in_gauge &&
        (px_col == 9'd66  || px_col == 9'd113 ||
         px_col == 9'd206 || px_col == 9'd253);

    // Track gradient colour: blue → cyan → green → yellow → red
    wire [8:0] track_pos = (px_col >= 9'd20) ? (px_col - 9'd20) : 9'd0; // 0-279
    reg [15:0] track_colour;
    always @(*) begin
        if (track_pos < 9'd70)
            track_colour = 16'h0011;  // dark blue
        else if (track_pos < 9'd140)
            track_colour = 16'h0131;  // dark cyan
        else if (track_pos < 9'd210)
            track_colour = 16'h0320;  // dark green
        else
            track_colour = 16'h4100;  // dark red
    end

    // Sweep mode indicator
    wire in_sweep_ind = draw_sweep &&
        (px_row >= 8'd200) && (px_row <= 8'd210) &&
        (px_col >= 9'd100) && (px_col <= 9'd219);

    // Header bar
    wire in_header = (px_row >= 8'd10) && (px_row <= 8'd14) &&
        (px_col >= 9'd20) && (px_col <= 9'd299);

    // Border frame
    wire at_border = ((px_row == 8'd70 || px_row == 8'd169) &&
                      (px_col >= 9'd10) && (px_col <= 9'd309)) ||
                     ((px_col == 9'd10 || px_col == 9'd309) &&
                      (px_row >= 8'd70) && (px_row <= 8'd169));

    // ---- Final colour mux ----
    reg [15:0] pixel_colour;
    always @(*) begin
        if (font_pixel)
            pixel_colour = 16'hFFFF;  // white digits
        else if (in_digit_area && which_digit != 2'd3)
            pixel_colour = 16'h0000;  // digit background
        else if (at_needle)
            pixel_colour = draw_sweep ? 16'hF800 : 16'h07E0;  // red/green needle
        else if (at_glow)
            pixel_colour = draw_sweep ? 16'h4000 : 16'h0200;  // dim glow
        else if (at_major_tick)
            pixel_colour = 16'hFFFF;  // white major ticks
        else if (at_minor_tick)
            pixel_colour = 16'hC618;  // grey minor ticks
        else if (in_track)
            pixel_colour = track_colour;  // gradient track
        else if (at_border)
            pixel_colour = 16'h4208;  // grey border
        else if (in_header)
            pixel_colour = draw_sweep ? 16'hF800 : 16'h07E0;  // mode bar
        else if (in_sweep_ind)
            pixel_colour = 16'hF800;  // sweep indicator
        else
            pixel_colour = 16'h0000;  // black background
    end

    // ---- Main FSM ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_WAIT_RESET;
            after_wr    <= S_WAIT_RESET;
            after_delay <= S_WAIT_RESET;
            delay_cnt   <= 24'd0;
            cmd_idx     <= 6'd0;
            pixel_cnt   <= 17'd0;
            px_col      <= 9'd0;
            px_row      <= 8'd0;
            wr_start    <= 1'b0;
            lcd_data    <= 16'd0;
            lcd_rs      <= 1'b0;
            draw_angle  <= 8'd90;
            draw_sweep  <= 1'b0;
        end else begin
            // Clear wr_start after one cycle
            if (wr_start) wr_start <= 1'b0;

            case (state)

                S_WAIT_RESET: begin
                    if (reset_done) begin
                        delay_cnt   <= 24'd0;
                        after_delay <= S_SEND_CMD;
                        state       <= S_DELAY;
                    end
                end

                S_POST_RESET: begin
                    // unused, kept for compatibility
                    state <= S_SEND_CMD;
                end

                S_SEND_CMD: begin
                    if (cmd_idx == 6'd21) begin
                        // All init + window commands sent, start pixel fill
                        draw_angle <= angle;
                        draw_sweep <= sweep_on;
                        px_col     <= 9'd0;
                        px_row     <= 8'd0;
                        pixel_cnt  <= 17'd0;
                        state      <= S_FILL_PIXEL;
                    end else if (cmd_rom == 9'h1FF) begin
                        // Delay marker
                        delay_cnt   <= 24'd0;
                        after_delay <= S_SEND_CMD;
                        cmd_idx     <= cmd_idx + 6'd1;
                        state       <= S_DELAY;
                    end else begin
                        lcd_rs   <= cmd_rom[8];
                        lcd_data <= {8'h00, cmd_rom[7:0]};
                        wr_start <= 1'b1;
                        after_wr <= S_SEND_CMD;
                        cmd_idx  <= cmd_idx + 6'd1;
                        state    <= S_WAIT_WR;
                    end
                end

                S_WAIT_WR: begin
                    if (wr_done)
                        state <= after_wr;
                end

                S_DELAY: begin
                    delay_cnt <= delay_cnt + 24'd1;
                    if (delay_cnt == 24'd6_000_000)  // 120ms
                        state <= after_delay;
                end

                S_FILL_PIXEL: begin
                    lcd_rs   <= 1'b1;
                    lcd_data <= pixel_colour;
                    wr_start <= 1'b1;
                    after_wr <= S_FILL_WAIT;
                    state    <= S_WAIT_WR;
                end

                S_FILL_WAIT: begin
                    if (px_col == 9'd319 && px_row == 8'd239) begin
                        // Frame done — restart from window setup (cmd_idx 10)
                        cmd_idx <= 6'd10;
                        state   <= S_SEND_CMD;
                    end else if (px_row == 8'd239) begin
                        px_row <= 8'd0;
                        px_col <= px_col + 9'd1;
                        state  <= S_FILL_PIXEL;
                    end else begin
                        px_row <= px_row + 8'd1;
                        state  <= S_FILL_PIXEL;
                    end
                end

                default: state <= S_WAIT_RESET;
            endcase
        end
    end

endmodule
