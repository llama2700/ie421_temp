#
# 5_gpio_config.tcl
#
# Manual control of Renesas RC38612 OTP configuration via GPIO[3:0] and reset.
#
# Prerequisites:
#   - Bitstream programmed and debug cores refreshed
#
# VIO probe mapping:
#   probe_out0 = vio_rstn          (I2C sequencer reset, leave alone)
#   probe_out1 = vio_gpio[3:0]    (GPIO config select to Renesas)
#   probe_out2 = vio_jitt_resetn  (active-low Renesas reset)
#
# Usage:
#   gpio_set <3:0 binary>    ;# e.g. gpio_set 1100
#   jitt_reset                ;# pulse reset low for 200ms
#   gpio_config <12-15>      ;# set GPIO + pulse reset in one call
#

proc gpio_set {val} {
    set_property OUTPUT_VALUE $val [get_hw_probes vio_0/probe_out1]
    commit_hw_vio [get_hw_probes vio_0/probe_out1]
    puts "GPIO\[3:0\] = $val"
}

proc jitt_reset {} {
    set_property OUTPUT_VALUE 0 [get_hw_probes vio_0/probe_out2]
    commit_hw_vio [get_hw_probes vio_0/probe_out2]
    puts "jitt_resetn asserted (low)"
    after 200
    set_property OUTPUT_VALUE 1 [get_hw_probes vio_0/probe_out2]
    commit_hw_vio [get_hw_probes vio_0/probe_out2]
    puts "jitt_resetn released (high)"
    puts "Waiting 1s for PLL lock..."
    after 1000
    puts "Ready."
}

proc gpio_config {cfg} {
    switch $cfg {
        12 { gpio_set 1100 }
        13 { gpio_set 1101 }
        14 { gpio_set 1110 }
        15 { gpio_set 1111 }
        default {
            puts "ERROR: config must be 12, 13, 14, or 15 (got $cfg)"
            return
        }
    }
    jitt_reset
    puts "Renesas now running OTP configuration $cfg"
}

puts ""
puts "=== Renesas GPIO Config Control ==="
puts "Commands:"
puts "  gpio_set <binary>     ;# e.g. gpio_set 1100"
puts "  jitt_reset            ;# pulse reset (200ms low, then high)"
puts "  gpio_config <12-15>   ;# set GPIO + reset in one step"
puts ""
puts "Examples:"
puts "  gpio_config 12        ;# load OTP config 12"
puts "  gpio_config 15        ;# load OTP config 15 (default)"
puts ""
puts "Then run:  source Scripts/2_read_freq_counters.tcl"
puts ""
