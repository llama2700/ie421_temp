# GTF DFE Optimization — Detailed Module Specifications

## Reference Documents
- UG1549 v1.1: UltraScale+ FPGAs GTF Transceivers User Guide
- Corundum NIC documentation (docs/source/)

---

## 1. `gtf_drp_controller`

### Purpose
Serialize and arbitrate DRP (Dynamic Reconfiguration Port) transactions to the GTF_CHANNEL primitive. Multiple internal requestors (calibration FSM, eye scan engine, host direct access) share a single DRP port.

### Parameters
```verilog
parameter DRP_ADDR_WIDTH = 16,
parameter DRP_DATA_WIDTH = 16,
parameter NUM_REQUESTORS = 3   // cal_fsm, eye_scan, host
```

### Ports
```verilog
// Clock/Reset
input  wire        clk,           // DRP clock (same as DRPCLK to GTF)
input  wire        rst,

// DRP master to GTF_CHANNEL
output reg  [DRP_ADDR_WIDTH-1:0]  drp_addr,
output reg  [DRP_DATA_WIDTH-1:0]  drp_di,
output reg                        drp_en,
output reg                        drp_we,
input  wire [DRP_DATA_WIDTH-1:0]  drp_do,
input  wire                       drp_rdy,

// Requestor 0: Calibration FSM
input  wire [DRP_ADDR_WIDTH-1:0]  req0_addr,
input  wire [DRP_DATA_WIDTH-1:0]  req0_wdata,
input  wire                       req0_valid,
input  wire                       req0_wr,
output wire [DRP_DATA_WIDTH-1:0]  req0_rdata,
output wire                       req0_done,
output wire                       req0_grant,

// Requestor 1: Eye Scan Engine
// (same signal pattern as req0)

// Requestor 2: Host Direct Access
// (same signal pattern as req0)
```

### State Machine
```
IDLE → GRANT → ASSERT_EN → WAIT_RDY → DONE → IDLE
```

- **IDLE**: Check requestors in fixed priority order (0 > 1 > 2). Grant first valid.
- **GRANT**: Latch address, data, wr from granted requestor. Assert `drp_en` and `drp_we` (if write).
- **WAIT_RDY**: Hold until `drp_rdy` asserts. Capture `drp_do` for reads.
- **DONE**: Assert `reqN_done` for one cycle. Return to IDLE.

### Timing
- DRP transactions take 1 DRPCLK cycle to initiate, variable cycles to complete (typically 1-2).
- `drp_en` must be a single-cycle pulse.
- `drp_we` held with `drp_en`.
- Back-to-back transactions need at least 1 idle cycle between them.

### DRP Address Map (GTF_CHANNEL, from UG1549 Appendix A)
Key registers for DFE optimization:

| Address | Register | Relevant Bits |
|---------|----------|---------------|
| 0x0040 | RXLPMEN | [0] LPM enable |
| 0x004C | RXDFE_CFG1 | [5] DFE monitor enable, [4:0] loop select |
| 0x0063 | RXDFE_H2_CFG1 | [15:11] TAP2 override value |
| 0x0065 | RXDFE_H3_CFG1 | [15:11] TAP3 override value |
| 0x0067 | RXDFE_H4_CFG1 | [15:11] TAP4 override value |
| 0x0069 | RXDFE_H5_CFG1 | [15:11] TAP5 override value |
| 0x006B | RXDFE_H6_CFG1 | [15:11] TAP6 override value |
| 0x006D | RXDFE_H7_CFG1 | [15:11] TAP7 override value |
| 0x006F | RXDFE_H8_CFG1 | [15:11] TAP8 override value |
| 0x0071 | RXDFE_H9_CFG1 | [15:11] TAP9 override value |
| 0x0073 | RXDFE_HA_CFG1 | [15:11] TAP10 override value |
| 0x0075 | RXDFE_HB_CFG1 | [15:11] TAP11 override value |
| 0x0077 | RXDFE_HC_CFG1 | [15:11] TAP12 override value |
| 0x0079 | RXDFE_HD_CFG1 | [15:11] TAP13 override value |
| 0x007B | RXDFE_HE_CFG1 | [15:11] TAP14 override value |
| 0x007D | RXDFE_HF_CFG1 | [15:11] TAP15 override value |
| 0x0090 | ES_CONTROL | Eye scan control |
| 0x0097 | RX_BUFFER_CFG | Buffer bypass config |

Note: Exact DRP addresses must be verified against the GTF Wizard output for the specific part (UL3422/UL3542). The addresses above are representative; use the generated IP's address map as the source of truth.

---

## 2. `gtf_cal_fsm`

### Purpose
Automated calibration sequence that converges DFE tap weights, reads them, freezes them, and optionally overrides them with stored values.

### Parameters
```verilog
parameter NUM_TAPS = 15,
parameter DMON_WIDTH = 16,
parameter CONVERGENCE_COUNT = 64,    // readings before declaring stable
parameter CONVERGENCE_THRESHOLD = 1, // max delta between readings for "stable"
parameter TIMEOUT_CYCLES = 10000000  // ~15ms at 644MHz, fail if not converged
```

### Ports
```verilog
// Clock/Reset
input  wire        clk,
input  wire        rst,

// Control
input  wire        cal_start,        // pulse to begin calibration
input  wire        override_start,   // pulse to load stored values
input  wire [4:0]  override_val [0:NUM_TAPS-1],  // stored tap values
output reg         cal_done,
output reg         cal_busy,
output reg         cal_error,        // timeout or DRP error
output reg  [3:0]  active_tap_count, // taps with non-zero converged value

// DRP requestor interface (to gtf_drp_controller)
output reg  [15:0] drp_addr,
output reg  [15:0] drp_wdata,
output reg         drp_valid,
output reg         drp_wr,
input  wire [15:0] drp_rdata,
input  wire        drp_done,
input  wire        drp_grant,

// GTF sideband (directly to GTF_CHANNEL ports)
output reg  [NUM_TAPS*2-1:0] tap_hold_ovrden,  // {HOLD,OVRDEN} per tap
output reg                    rxdfelpmreset,
input  wire [DMON_WIDTH-1:0]  dmonitorout,

// Tap readback
output reg  [4:0]  tap_values [0:NUM_TAPS-1],   // converged values
output reg  [14:0] tap_active_mask              // bitmask of non-zero taps
```

### State Machine: Calibration Mode
```
IDLE
  → CAL_RESET_DFE        Assert RXDFELPMRESET for 16 cycles
  → CAL_RELEASE           Deassert RXDFELPMRESET, set all taps to auto-adapt {00}
  → CAL_WAIT_SETTLE       Wait fixed settling time (e.g., 1ms = ~644K cycles)
  → CAL_READ_TAP          Select tap N via RXDFE_CFG1[4:0] DRP write
  → CAL_READ_DMON         Read DMONITOROUT, extract tap value
  → CAL_CHECK_STABLE      Compare with previous reading, increment stable count or reset
  → CAL_NEXT_TAP          If stable count >= CONVERGENCE_COUNT, store value, advance to next tap
                           If all taps read, → CAL_FREEZE
                           If timeout, → CAL_ERROR
  → CAL_FREEZE            Set all taps to {HOLD=1, OVRDEN=0}
  → CAL_MINIMIZE          For taps with |value| < 2, override to 0 and freeze
  → CAL_DONE              Assert cal_done, return to IDLE
```

### State Machine: Override Mode
```
IDLE
  → OVR_WRITE_TAP         For each tap: write override value to RXDFE_Hx_CFG1[15:11] via DRP
  → OVR_SET_OVRDEN        Set {HOLD=x, OVRDEN=1} for all taps
  → OVR_DONE              Assert cal_done, return to IDLE
```

### DMONITOROUT Tap Value Extraction
Per UG1549 Table 41-42:
- Write `RXDFE_CFG1[5] = 1` (enable DFE monitoring)
- Write `RXDFE_CFG1[4:0]` to select the desired adaptation loop
- Read `DMONITOROUT[6:0]` for 7-bit signed value (taps with double-neutral encoding)
- Read `DMONITOROUT[6:2]` for 5-bit value (most taps)
- Read `DMONITOROUT[6:1]` for 6-bit value (TAP2, TAP3)

Mapping (from UG1549 Table 42):
```
RXDFE_CFG1[4:0] = 5'b00000 → RXDFEOS (baseline wander, 7-bit signed)
RXDFE_CFG1[4:0] = 5'b00001 → RXDFEKL (DFE low-freq gain, 5-bit)
RXDFE_CFG1[4:0] = 5'b00010 → RXDFETAP2 (6-bit)
RXDFE_CFG1[4:0] = 5'b00011 → RXDFETAP3 (6-bit)
RXDFE_CFG1[4:0] = 5'b00100 → RXDFEKH (DFE high-freq gain, 5-bit)
...
RXDFE_CFG1[4:0] = 5'b01000 → RXDFETAP4 (5-bit)
RXDFE_CFG1[4:0] = 5'b01001 → RXDFETAP5 (5-bit)
... through TAP15
```

### Convergence Detection
```
For each tap:
  previous_value = 0
  stable_count = 0
  loop:
    current_value = read DMONITOROUT
    if |current_value - previous_value| <= CONVERGENCE_THRESHOLD:
      stable_count++
    else:
      stable_count = 0
    previous_value = current_value
    if stable_count >= CONVERGENCE_COUNT:
      converged_value[tap] = current_value
      break
    if total_cycles > TIMEOUT_CYCLES:
      error
```

---

## 3. `gtf_eye_scan_engine`

### Purpose
Drive the GTF's built-in 2D eye scan hardware to measure eye height, width, and BER at configurable sample points.

### Parameters
```verilog
parameter H_STEPS = 64,      // horizontal scan resolution
parameter V_STEPS = 128,     // vertical scan resolution
parameter BRAM_DEPTH = 8192, // storage for scan results
parameter DEFAULT_PRESCALE = 5
```

### Eye Scan Sequence (per UG1549 RX Margin Analysis)
```
1. Assert EYESCANRESET for 16 cycles, deassert
2. Configure prescale via ES_PRESCALE DRP register
3. For each (h_offset, v_offset) sample point:
   a. Write ES_HORZ_OFFSET via DRP
   b. Write ES_VERT_OFFSET via DRP
   c. Write ES_CONTROL to start measurement
   d. Wait for ES_CONTROL_STATUS = done
   e. Read error count from ES_ERROR_COUNT
   f. Read sample count from ES_SAMPLE_COUNT
   g. Compute BER = error_count / sample_count
   h. Store {h_offset, v_offset, ber} in BRAM
4. Post-process: find eye boundaries (BER < threshold)
5. Compute eye_height = max_v - min_v at h_offset=0
6. Compute eye_width = max_h - min_h at v_offset=0
```

### Quick Scan Mode
Instead of full 2D scan, sample only:
- 5 horizontal points: {-2, -1, 0, +1, +2} around center
- 5 vertical points: {-2, -1, 0, +1, +2} around center
- Total: 25 points instead of H_STEPS * V_STEPS
- Sufficient for LPM/DFE decision and margin monitoring

---

## 4. `gtf_lpm_dfe_selector`

### Purpose
Automatically select between LPM (CTLE-only) and DFE equalization modes based on measured eye margin.

### Logic
```
State machine:
  START_LPM:
    Set RXLPMEN = 1
    Wait for link to stabilize (~10ms)
    Trigger quick eye scan
    → CHECK_LPM

  CHECK_LPM:
    Wait for scan_done
    If eye_height >= threshold:
      Stay in LPM, set recheck timer → MONITOR
    Else:
      → SWITCH_TO_DFE

  SWITCH_TO_DFE:
    Set RXLPMEN = 0
    Trigger calibration FSM
    Wait for cal_done
    Trigger quick eye scan → CHECK_DFE

  CHECK_DFE:
    Wait for scan_done
    Record margin → MONITOR

  MONITOR:
    Decrement recheck timer
    When timer expires: trigger quick scan, → RECHECK

  RECHECK:
    Wait for scan_done
    If margin degraded below threshold and in LPM: → SWITCH_TO_DFE
    Else: reset timer → MONITOR
```

---

## 5. `gtf_tx_align_engine`

### Purpose
Minimize TX latency by aligning packet injection to 64b/66b codeword boundaries using the hard MAC's `TXAXISTCANSTART` signal.

### Logic

**Pass-through mode** (`force_align = 0`):
- Direct wire-through, zero added latency
- Still counts hits/misses for monitoring

**Aligned mode** (`force_align = 1`):
- When `s_axis_tvalid` rises (new packet start), check `txaxistcanstart`
- If `txaxistcanstart = 1`: forward immediately (hit)
- If `txaxistcanstart = 0`: hold `s_axis_tready` low until `txaxistcanstart` asserts (miss)
- Maximum wait: 4 TXUSRCLK cycles (one codeword period)

**Predictive mode** (enhancement):
- Track `txgbseqstart` to predict when next `txaxistcanstart` will occur
- Per UG1549: new codewords start at sequence counts 0, 4, 8, 12, 16, 20, 24, 28
- Optimal injection: one cycle before codeword start (counts 3, 7, 11, 15, 19, 23, 32)
- Pre-assert `s_axis_tready` one cycle early to give upstream logic a head start

---

## 6. `gtf_rx_fcs_bypass`

### Purpose
Disable RX FCS checking in the hard MAC to eliminate the up-to-9-cycle delay between last data byte and `RXAXISTERR`.

From the datasheet, `ctl_rx_check_fcs` and `ctl_rx_delete_fcs` are direct ports on the GTF_CHANNEL primitive (not DRP registers). The module simplifies to driving those ports directly:

```verilog
assign ctl_rx_check_fcs = fcs_check_enable;
assign ctl_rx_delete_fcs = fcs_delete_enable;
```

---

## 7. `gtf_buf_bypass_ctrl`

### Purpose
Configure GTF for TX and RX buffer bypass mode to achieve fixed, deterministic latency through the transceiver.

### Buffer Bypass Sequence (per UG1549)

**TX Buffer Bypass:**
```
1. Configure TX_BUFFER_CFG via DRP for bypass mode
2. After TXRESETDONE:
   a. Assert TXPHDALIGNEN
   b. Wait for TXPHALIGNDONE
   c. Buffer bypass active, latency is now fixed
3. After any reset: must repeat alignment
```

**RX Buffer Bypass:**
```
1. Configure RX_BUFFER_CFG via DRP for bypass mode
2. After RXRESETDONE:
   a. Assert RXPHDALIGNEN
   b. Wait for RXPHALIGNDONE
   c. Buffer bypass active, latency is now fixed
3. After any reset or CDR re-lock: must repeat alignment
```

### Re-alignment Detection
Monitor `RXCDRLOCK` — if it drops and re-asserts, buffer bypass alignment must be repeated. The module automatically re-runs the alignment sequence when this occurs.

---

## 8. `gtf_dfe_opt_core` (Top-Level Integration)

### Purpose
Integrate all submodules, provide unified AXI-Lite register interface.

### Internal Wiring
```
                  ┌──────────────────────┐
   AXI-Lite ──────┤  Register File       │
                  │  (decode, CDC)       │
                  └───┬──┬──┬──┬──┬──┬───┘
                      │  │  │  │  │  │
            ┌─────────┘  │  │  │  │  └──────────┐
            ▼            ▼  │  ▼  ▼              ▼
     ┌──────────┐ ┌──────┐  │ ┌──────┐    ┌──────────┐
     │ cal_fsm  │ │eye   │  │ │lpm/  │    │tx_align  │
     │          │ │scan  │  │ │dfe   │    │          │
     └────┬─────┘ └──┬───┘  │ │sel   │    └──────────┘
          │          │      │ └──────┘
          ▼          ▼      ▼
     ┌────────────────────────┐    ┌──────────┐  ┌──────────┐
     │   drp_controller       │    │fcs_bypass│  │buf_bypass│
     │   (arbitrate)          │    │          │  │          │
     └────────┬───────────────┘    └──────────┘  └──────────┘
              │
              ▼
         GTF DRP port
```

### Clock Domain Crossings
- `clk` (core, ~250 MHz) ↔ `drpclk` (DRP, variable) — for DRP transactions initiated from register writes
- `clk` ↔ `txusrclk` (~645 MHz) — for TX alignment engine control/status
- `clk` ↔ `rxusrclk` (~645 MHz) — for DMONITOROUT sampling

Use `xpm_cdc_*` primitives or simple 2-FF synchronizers for single-bit signals. Use async FIFOs for multi-bit transfers if needed.

---

## 9. `gtf_dfe_opt_corundum_wrapper`

### Purpose
Map the standalone `gtf_dfe_opt_core` into Corundum's application block interface.

### Integration Points in Corundum

1. **`fpga.v`**: Add GTF_CHANNEL sideband ports to the `fpga_core` instance port list
2. **`fpga_core.v`**: Pass sideband ports through to `mqnic_core` instance
3. **`mqnic_core_pcie_us.v`**: Pass sideband ports through to `mqnic_app_block` instance
4. **`mqnic_app_block.v`**: Instantiate `gtf_dfe_opt_core`, connect AXI-Lite from `s_axil_app_ctrl_*` and sideband signals

---

## 10. Linux Driver: `mqnic_gtf_dfe.c`

### sysfs Attributes (read-only monitoring)
```
/sys/class/net/<iface>/device/gtf_dfe/
├── mode              # "lpm" or "dfe" or "auto"
├── active_taps       # integer count
├── eye_height        # integer
├── eye_width         # integer
├── buf_bypass        # "aligned" or "disabled" or "error"
├── tx_align_hits     # integer
├── tx_align_misses   # integer
└── cal_status        # "idle" or "busy" or "done" or "error"
```

### ioctl Commands
```c
#define GTF_DFE_IOC_MAGIC  'G'
#define GTF_DFE_CALIBRATE      _IO(GTF_DFE_IOC_MAGIC, 1)
#define GTF_DFE_EYE_SCAN       _IO(GTF_DFE_IOC_MAGIC, 2)
#define GTF_DFE_GET_TAPS       _IOR(GTF_DFE_IOC_MAGIC, 3, struct gtf_dfe_taps)
#define GTF_DFE_SET_TAPS       _IOW(GTF_DFE_IOC_MAGIC, 4, struct gtf_dfe_taps)
#define GTF_DFE_SET_MODE       _IOW(GTF_DFE_IOC_MAGIC, 5, int)
#define GTF_DFE_GET_STATUS     _IOR(GTF_DFE_IOC_MAGIC, 6, struct gtf_dfe_status)
#define GTF_DFE_DRP_READ       _IOWR(GTF_DFE_IOC_MAGIC, 7, struct gtf_dfe_drp_xfer)
#define GTF_DFE_DRP_WRITE      _IOW(GTF_DFE_IOC_MAGIC, 8, struct gtf_dfe_drp_xfer)
#define GTF_DFE_QUICK_SCAN     _IO(GTF_DFE_IOC_MAGIC, 9)
```

---

## 11. Userspace Library: `libgtf_dfe_opt`

### API
```c
gtf_dfe_handle_t *gtf_dfe_open(const char *device);
void gtf_dfe_close(gtf_dfe_handle_t *h);
int gtf_dfe_calibrate(gtf_dfe_handle_t *h);
int gtf_dfe_eye_scan(gtf_dfe_handle_t *h);
int gtf_dfe_quick_scan(gtf_dfe_handle_t *h);
int gtf_dfe_get_taps(gtf_dfe_handle_t *h, struct gtf_dfe_taps *taps);
int gtf_dfe_set_taps(gtf_dfe_handle_t *h, const struct gtf_dfe_taps *taps);
int gtf_dfe_save_profile(gtf_dfe_handle_t *h, const char *filename);
int gtf_dfe_load_profile(gtf_dfe_handle_t *h, const char *filename);
int gtf_dfe_set_mode(gtf_dfe_handle_t *h, enum gtf_dfe_mode mode);
int gtf_dfe_get_status(gtf_dfe_handle_t *h, struct gtf_dfe_status *status);
int gtf_dfe_drp_read(gtf_dfe_handle_t *h, uint16_t addr, uint16_t *data);
int gtf_dfe_drp_write(gtf_dfe_handle_t *h, uint16_t addr, uint16_t data);
```

### Profile File Format
```
# GTF DFE Tap Profile
mode=dfe
tap_count=15
tap0=12
tap1=7
tap2=3
...
active_mask=0x000F
buf_bypass=1
fcs_check=0
tx_align=1
```
