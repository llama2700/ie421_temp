# GTF DFE Optimization Project — Project Plan

## Context

This project builds a latency-optimized DFE (Decision Feedback Equalizer) control system for the Xilinx GTF transceiver on UltraScale+ FPGAs (UL3422/UL3542), targeting 10G Ethernet in hard MAC mode. The goal is to minimize and stabilize analog receive-path latency for high-frequency trading (HFT) applications by replacing the GTF's default auto-adapt behavior with a calibrated, deterministic equalization configuration.

The system is designed as a standalone RTL module with a thin Corundum integration wrapper, plus a Linux driver extension and userspace tools for runtime control.

### The 7 Optimizations

| # | Optimization | Layer | Hard MAC? |
|---|---|---|---|
| 1 | LPM vs DFE mode selection | PMA | Yes |
| 2 | Buffer bypass (fixed latency) | PMA | Yes |
| 3 | Freeze taps after calibration | PMA | Yes |
| 4 | Minimize active tap count | PMA | Yes |
| 5 | TXAXISTCANSTART alignment | MAC | Yes |
| 6 | Disable RX FCS checking | MAC | Yes |
| 7 | Eye scan sweep for optimal tap values | PMA | Yes |

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  Linux Host                                             │
│  ┌───────────────┐  ┌────────────────────────────────┐  │
│  │ mqnic driver  │  │ gtf_dfe_opt userspace tool     │  │
│  │ (extended)    │  │ - calibration CLI              │  │
│  │               │  │ - eye scan visualization       │  │
│  │ ioctl/sysfs ◄─┼──┤ - tap dump/load                │  │
│  └───────┬───────┘  └────────────────────────────────┘  │
│          │ PCIe BAR (AXI-Lite)                          │
├──────────┼──────────────────────────────────────────────┤
│  FPGA    │                                              │
│  ┌───────▼───────────────────────────────────────────┐   │
│  │ Corundum mqnic_core                               │   │
│  │  ┌──────────────────────────────────────────────┐ │   │
│  │  │ mqnic_app_block (Corundum  wrapper)          │ │   │
│  │  │  ┌────────────────────────────────────────┐  │ │   │
│  │  │  │ gtf_dfe_opt_core (standalone module)   │  │ │   │
│  │  │  │  ┌──────────────┐ ┌─────────────────┐  │  │ │   │
│  │  │  │  │ DRP Control  │ │ Calibration FSM │  │  │ │   │
│  │  │  │  └──────┬───────┘ └────────┬────────┘  │  │ │   │
│  │  │  │  ┌──────┴───────┐ ┌────────┴────────┐  │  │ │   │
│  │  │  │  │ Eye Scan Eng │ │ TX Align Engine │  │  │ │   │
│  │  │  │  └──────────────┘ └─────────────────┘  │  │ │   │
│  │  │  └────────────────────────────────────────┘  │ │   │
│  │  └──────────────────────────────────────────────┘ │   │
│  └───────────────────────────────┬───────────────────┘   │
│                                  │ DRP                   │
│  ┌───────────────────────────────▼───────────────────┐   │
│  │ GTF_CHANNEL (hard silicon)                        │   │
│  │  CTLE → DFE (15 taps) → CDR → PCS → MAC           │   │
│  └───────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────┘
```

---

## Module Breakdown

### Module 1: `gtf_dfe_opt_core` (Standalone Top-Level)

**Purpose**: Self-contained DFE optimization engine, usable in any GTF design.

**Interfaces**:
- AXI-Lite slave (32-bit addr, 32-bit data) for host register access
- DRP master port (16-bit addr, 16-bit data) to GTF_CHANNEL
- GTF sideband signals: `RXLPMEN`, `DMONITOROUT`, `DMONITORCLK`, per-tap `{HOLD, OVRDEN}` ports, `EYESCANRESET`, `RXDFELPMRESET`
- GTF MAC signals: `TXAXISTCANSTART`, `TXAXISTSOF`, `TXAXISTREADY`
- Status/interrupt output

**Submodules**:

#### 1a. `gtf_drp_controller`
- Serializes DRP read/write transactions
- Arbitrates between eye scan engine, calibration FSM, and host-initiated DRP access
- Handles DRP timing (assert `drp_en`, wait for `drp_rdy`)

#### 1b. `gtf_cal_fsm` (Calibration State Machine)
- Implements the boot-time calibration sequence:
  1. Assert `RXDFELPMRESET` to reset DFE
  2. Release, let auto-adapt run on all 15 taps
  3. Poll `DMONITOROUT` until tap values stabilize (convergence detection)
  4. Read converged values for all taps via `DMONITOROUT` cycling
  5. Freeze all taps (`HOLD=1`)
  6. Store values in internal registers
- Implements override mode:
  1. Load pre-stored tap values into `RXDFE_Hx_CFG1[15:11]` via DRP
  2. Set `{HOLD=x, OVRDEN=1}` for all taps
  3. Skip auto-adapt entirely
- Implements tap minimization:
  1. After calibration, identify taps with converged value near zero
  2. Force those taps to zero and freeze
  3. Report number of active taps to host

#### 1c. `gtf_eye_scan_engine`
- Drives the GTF eye scan hardware
- Configurable prescale, horizontal/vertical range
- Collects eye diagram data point by point
- Stores results in internal BRAM (configurable depth)
- Supports targeted sweep mode: only scan a small region around the current operating point for fast characterization
- Reports eye height, eye width, and estimated BER to host registers

#### 1d. `gtf_lpm_dfe_selector`
- Implements the LPM vs DFE decision logic
- Starts in LPM mode (`RXLPMEN=1`)
- Triggers a quick eye scan
- If eye margin > configurable threshold: stay in LPM
- If margin < threshold: switch to DFE (`RXLPMEN=0`), run calibration
- Reports selected mode and margin to host

#### 1e. `gtf_tx_align_engine`
- Monitors `TXAXISTCANSTART` signal from the hard MAC
- Provides a `tx_can_send` output to downstream logic
- Optionally holds TX AXI-Stream data until next codeword boundary
- Counts alignment hits/misses for performance monitoring
- Integrates with `TXGBSEQSTART` (33-cycle gearbox pattern) for predictive alignment

#### 1f. `gtf_rx_fcs_bypass`
- Thin wrapper for RX FCS control
- Sets `ctl_rx_check_fcs = 0` via DRP at initialization
- Optionally keeps `ctl_rx_delete_fcs = 0` so FCS bytes pass through for software verification
- Exposes FCS enable/disable as a runtime register

#### 1g. `gtf_buf_bypass_ctrl`
- Configures the GTF for TX and RX buffer bypass mode
- Handles the alignment sequence required after reset (per UG1549 ch. 3/4)
- Monitors alignment status
- Reports fixed latency value to host

### Module 2: `gtf_dfe_opt_corundum_wrapper`

**Purpose**: Thin wrapper that maps `gtf_dfe_opt_core`'s AXI-Lite interface into Corundum's application block.

**Implementation**:
- Instantiates `gtf_dfe_opt_core`
- Maps Corundum's `s_axil_app_ctrl_*` signals to the core's AXI-Lite slave
- Passes through DRP and GTF sideband signals from `fpga.v` port list
- Connects TX align engine to the direct TX AXI-Stream interface for packet injection timing
- Bridges interrupt output to Corundum's event system

### Module 3: Linux Driver Extension

**Purpose**: Expose DFE optimization controls to userspace through the existing mqnic driver.

**Components**:

#### 3a. `mqnic_gtf_dfe.c` (kernel module extension)
- Registers as a sub-device of the mqnic driver
- Maps the app block's AXI-Lite BAR region
- Provides:
  - `sysfs` attributes for: current mode (LPM/DFE), active tap count, eye height/width, buffer bypass status, FCS bypass status, TX alignment hit rate
  - `ioctl` interface for: trigger calibration, trigger eye scan, read/write individual tap values, load/save tap profiles, set LPM/DFE threshold

#### 3b. `libgtf_dfe_opt` (userspace library)
- C library wrapping the ioctl interface
- Functions: `gtf_calibrate()`, `gtf_eye_scan()`, `gtf_get_taps()`, `gtf_set_taps()`, `gtf_save_profile()`, `gtf_load_profile()`, `gtf_set_mode()`

#### 3c. `gtf-dfe-tool` (CLI utility)
- `gtf-dfe-tool calibrate` — run full calibration sequence
- `gtf-dfe-tool scan` — run eye scan and dump results
- `gtf-dfe-tool taps` — print current tap values
- `gtf-dfe-tool profile save <file>` — save current config to file
- `gtf-dfe-tool profile load <file>` — load config from file
- `gtf-dfe-tool status` — print mode, margins, alignment stats
- `gtf-dfe-tool mode [lpm|dfe|auto]` — force mode or enable auto-selection

---

## Register Map

Base address: configurable (Corundum app block BAR offset)

| Offset | Name | R/W | Description |
|--------|------|-----|-------------|
| 0x0000 | CTRL | RW | Global control: [0] cal_start, [1] eye_scan_start, [2] override_enable, [3] buf_bypass_en, [4] fcs_bypass_en, [5] tx_align_en, [6] lpm_force, [7] dfe_force, [8] auto_mode_en |
| 0x0004 | STATUS | RO | [0] cal_done, [1] cal_busy, [2] eye_scan_done, [3] eye_scan_busy, [4] link_up, [5] lpm_active, [6] dfe_active, [7] buf_bypass_aligned, [8] rx_block_lock |
| 0x0008 | IRQ_EN | RW | Interrupt enable mask |
| 0x000C | IRQ_STATUS | RO/W1C | Interrupt status (write-1-to-clear) |
| 0x0010 | TAP_ACTIVE | RO | Bitmask of taps with non-zero converged values |
| 0x0014 | TAP_COUNT | RO | Number of active taps after minimization |
| 0x0020-0x005C | TAP_VAL[0:15] | RO | Converged/current value per tap (5-bit, zero-extended to 32) |
| 0x0060-0x009C | TAP_OVR[0:15] | RW | Override value per tap (5-bit) |
| 0x00A0 | EYE_HEIGHT | RO | Measured eye height (in mV or codes) |
| 0x00A4 | EYE_WIDTH | RO | Measured eye width (in UI or codes) |
| 0x00A8 | EYE_BER | RO | Estimated BER (log scale, fixed point) |
| 0x00AC | EYE_PRESCALE | RW | Eye scan prescale value |
| 0x00B0 | EYE_H_RANGE | RW | Horizontal scan range [15:0] min, [31:16] max |
| 0x00B4 | EYE_V_RANGE | RW | Vertical scan range [15:0] min, [31:16] max |
| 0x00C0 | TX_ALIGN_HIT | RO | Count of TX packets aligned to codeword boundary |
| 0x00C4 | TX_ALIGN_MISS | RO | Count of TX packets that waited for next codeword |
| 0x00C8 | TX_ALIGN_CTRL | RW | [0] force_align (hold TX until TXAXISTCANSTART) |
| 0x00D0 | LPM_THRESHOLD | RW | Eye margin threshold for LPM/DFE auto-selection |
| 0x00D4 | CAL_TIMEOUT | RW | Convergence detection timeout (in DMONITORCLK cycles) |
| 0x00D8 | CAL_STABILITY | RW | Number of stable readings required for convergence |
| 0x00E0 | DRP_ADDR | RW | Direct DRP access: address |
| 0x00E4 | DRP_DATA | RW | Direct DRP access: write data / read data |
| 0x00E8 | DRP_CTRL | RW | [0] drp_start, [1] drp_wr |
| 0x00EC | DRP_STATUS | RO | [0] drp_done, [1] drp_busy |
| 0x0100 | VERSION | RO | Module version / magic number |
| 0x0104 | SCRATCH | RW | Scratch register for driver probe |

---

## File Structure

```
rtl/custom/
├── rtl/
│   ├── gtf_dfe_opt_core.v           # Standalone top-level
│   ├── gtf_drp_controller.v         # DRP arbitration and serialization
│   ├── gtf_cal_fsm.v                # Calibration state machine
│   ├── gtf_eye_scan_engine.v        # Eye scan driver
│   ├── gtf_lpm_dfe_selector.v       # LPM/DFE auto-selection
│   ├── gtf_tx_align_engine.v        # TXAXISTCANSTART alignment
│   ├── gtf_rx_fcs_bypass.v          # FCS check bypass control
│   ├── gtf_buf_bypass_ctrl.v        # Buffer bypass configuration
│   └── gtf_dfe_opt_corundum_wrapper.v  # Corundum app block wrapper
├── tb/
│   ├── test_gtf_dfe_opt_core.py     # cocotb top-level testbench
│   ├── test_gtf_cal_fsm.py          # Calibration FSM tests
│   ├── test_gtf_eye_scan.py         # Eye scan engine tests
│   ├── test_gtf_tx_align.py         # TX alignment tests
│   └── gtf_channel_model.v          # GTF behavioral model for sim
├── driver/
│   ├── mqnic_gtf_dfe.c              # Kernel module extension
│   ├── mqnic_gtf_dfe.h              # Kernel header (register offsets)
│   └── Makefile
├── lib/
│   ├── libgtf_dfe_opt.c             # Userspace C library
│   ├── libgtf_dfe_opt.h             # Library header
│   └── Makefile
├── tools/
│   ├── gtf-dfe-tool.c               # CLI utility
│   └── Makefile
├── constraints/
│   └── gtf_dfe_opt_timing.xdc       # Timing constraints for DRP/DMONITOR clocks
└── docs/
    └── spec.md                       # Detailed spec (see separate file)
```

---

## Implementation Order

1. **`gtf_drp_controller`** — everything else depends on DRP access
2. **`gtf_buf_bypass_ctrl`** — enables fixed-latency mode, independent of other optimizations
3. **`gtf_rx_fcs_bypass`** — trivial, just a few DRP writes at init
4. **`gtf_cal_fsm`** — core calibration logic, depends on DRP controller
5. **`gtf_lpm_dfe_selector`** — depends on eye scan engine and cal FSM
6. **`gtf_eye_scan_engine`** — depends on DRP controller
7. **`gtf_tx_align_engine`** — independent of DFE path, can be developed in parallel
8. **`gtf_dfe_opt_core`** — integrates all submodules, adds register file
9. **`gtf_dfe_opt_corundum_wrapper`** — thin integration layer
10. **`mqnic_gtf_dfe.c`** + `libgtf_dfe_opt` + `gtf-dfe-tool` — software stack
11. **Testbenches** — developed alongside each module

---

## Verification

### RTL Simulation
- cocotb testbenches using a GTF behavioral model (`gtf_channel_model.v`) that simulates DRP responses, DMONITOROUT values, and eye scan hardware behavior
- Test cases:
  - DRP read/write sequencing and arbitration
  - Calibration convergence detection with various tap profiles
  - Tap freeze and override
  - LPM/DFE auto-selection threshold crossing
  - Eye scan data collection and BER estimation
  - TX alignment hit/miss counting
  - Buffer bypass alignment sequence
  - Register read/write from AXI-Lite

### Hardware Validation
- ILA (Integrated Logic Analyzer) probes on: DRP bus, DMONITOROUT, tap hold/override signals, TXAXISTCANSTART, eye scan state
- Loopback test: near-end PMA loopback to verify calibration without external link
- Live link test: SFP+ DAC cable between two ports, run calibration, compare LPM vs DFE eye margins
- Latency measurement: PTP timestamps or ILA-based measurement of packet-in to packet-out latency in LPM vs DFE, buffer bypass vs normal

### Software Validation
- `gtf-dfe-tool status` reports correct values matching ILA observations
- `gtf-dfe-tool calibrate` completes without error, tap values match DMONITOROUT
- Profile save/load round-trips correctly
- Eye scan produces plausible eye diagram data

---

## Detailed Spec

See `gtf_dfe_opt_spec.md` in this directory for per-module specifications including port lists, state machine diagrams, DRP register sequences, and timing requirements.
