# =========================
# User configuration
# =========================
set PART "xcvu2p-fsvj2104-3-e"   ;# <-- change to your exact part
set TOP  "top"               ;# <-- your top module (note: no dash)

set SRC_DIR "./src"
set BUILD_DIR "./build"

# Create build directory if needed
file mkdir $BUILD_DIR

# =========================
# Parallelization
# =========================
# Use more cores (adjust as needed)
# set_param general.maxThreads 8

# =========================
# In-memory project setup
# =========================
create_project -in_memory -part $PART

# =========================
# Read sources
# =========================

# RTL
read_verilog [glob $SRC_DIR/rtl/*.v]

# Constraints
read_xdc [glob $SRC_DIR/constraints/*.xdc]

set ip_files [glob $SRC_DIR/ip/*.xci]

foreach ip $ip_files {
    import_ip $ip
}

generate_target all [get_ips]
synth_ip [get_ips]

# =========================
# Synthesis
# =========================
synth_design -top $TOP -part $PART

# Write post-synth checkpoint
write_checkpoint -force $BUILD_DIR/post_synth.dcp

# Optional reports
report_timing_summary -file $BUILD_DIR/post_synth_timing.rpt
report_utilization -file $BUILD_DIR/post_synth_util.rpt

# =========================
# Implementation
# =========================
opt_design
place_design
route_design

# Write post-impl checkpoint
write_checkpoint -force $BUILD_DIR/post_impl.dcp

# Reports
report_timing_summary -file $BUILD_DIR/post_impl_timing.rpt
report_utilization -file $BUILD_DIR/post_impl_util.rpt

# =========================
# Bitstream + debug probes
# =========================

# Debug probes (ILA, etc.)
write_debug_probes -force $BUILD_DIR/${TOP}.ltx

# Bitstream
write_bitstream -force $BUILD_DIR/${TOP}.bit

# =========================
# Done
# =========================
puts "Build complete. Outputs in $BUILD_DIR"