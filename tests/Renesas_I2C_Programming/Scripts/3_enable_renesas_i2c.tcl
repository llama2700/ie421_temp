#
# 3_enable_renesas_i2c.tcl
#
# Triggers the I2C sequencer to program the Renesas RC38612 jitter cleaner
# with register values from the .coe file stored in BRAM.
#
# ONLY run this when you are ready to reprogram the jitter cleaner.
# After this completes, run 2_read_freq_counters.tcl again to see
# the new frequencies.
#

puts ""
puts "WARNING: This will program the Renesas jitter cleaner via I2C."
puts "         Make sure you are ready to modify the clock outputs."
puts ""

# Trigger VIO probe_out0 = 1 to start I2C sequencer
set_property OUTPUT_VALUE 1 [get_hw_probes vio_0/probe_out0]
commit_hw_vio [get_hw_probes vio_0/probe_out0]

puts "I2C sequencer triggered."
puts "Monitor ILA for SCL/SDA activity and xfer_count progress."
puts ""
puts "Wait for programming to complete (xfer_enable goes low),"
puts "then run: source 2_read_freq_counters.tcl"
