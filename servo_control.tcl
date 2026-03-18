# =============================================================================
# servo_control.tcl — Control servo via JTAG by writing to PWM registers
# =============================================================================
# After programming the FPGA with the AXI PWM IP design, this script
# writes directly to the PWM registers via the LWH2F bridge.
#
# Register map (base 0xFF200000):
#   0x00 CTRL   - bit[0] = enable
#   0x04 PERIOD - period in clock cycles (1,000,000 = 50 Hz)
#   0x08 DUTY   - duty in clock cycles
#   0x0C STATUS - bit[0] = pwm level, bit[1] = irq pending

proc servo_init {} {
    global master
    set masters [get_service_paths master]
    set ::master [lindex $masters 0]
    open_service master $::master
    puts "Connected to JTAG master"

    # Set period = 1,000,000 (50 Hz)
    master_write_32 $::master 0xFF200004 0x000F4240
    # Set duty = 75,000 (1.5 ms = 90 degrees)
    master_write_32 $::master 0xFF200008 0x000124F8
    # Enable PWM
    master_write_32 $::master 0xFF200000 0x00000001

    puts "PWM enabled at 90 degrees"
}

proc servo_angle {deg} {
    global master
    # Map 0-180 degrees to 50000-100000 clock cycles
    # duty = 50000 + (deg * 278)  (278 = 50000/180)
    set duty [expr {50000 + int($deg * 277.78)}]
    master_write_32 $::master 0xFF200008 $duty
    puts "Servo -> ${deg} degrees (duty=$duty cycles)"
}

proc servo_sweep {} {
    global master
    puts "Sweeping 0 -> 180 -> 0..."
    for {set d 0} {$d <= 180} {incr d 10} {
        servo_angle $d
        after 200
    }
    for {set d 180} {$d >= 0} {incr d -10} {
        servo_angle $d
        after 200
    }
    puts "Sweep complete"
}

proc servo_close {} {
    global master
    close_service master $::master
    puts "Disconnected"
}

# Auto-run demo
servo_init
after 500
servo_sweep
after 500
servo_angle 90
servo_close
