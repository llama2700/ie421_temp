# Custom not-so-802.3 MAC/PCS in fabric

> **VERY IMPORTANT:** we can basically disregard area (unless it's insane), as long as our logic stays close to the hard AFE part of the transceiver.
>
> *side question: does the hard mac get in the way? just curious*

analog front-end: CDR, CTLE/DFE, and SerDes. Everything from the raw deserialized bits up to the host interface is your responsibility in soft logic. This maps to the 10GBASE-R PCS and MAC layers per IEEE 802.3.

---

## Step 1: PCS front-end — Gearbox + Block Alignment/Sync

- GTF SerDes outputs raw bits at fixed (?) parallel width.
- Gearbox converts to aligned 66-bit blocks.
- Searches for repeating `01`/`10` sync header pattern to find block boundaries and locks on.

### Optimizations
- Barrel shifter width and pipeline depth for gearbox (wider = fewer stages but more LUTs)
- Sync. header detection can be speculative across multiple offsets in parallel
- Slip-based vs. shift-based alignment (trade higher area for quicker lock time)
- Minimize cycles from raw data to first valid aligned block

### Placement/Timing
Pblock-constrain the gearbox + sync logic to the SLRs nearest the GTF column. Timing is going to be hard to close without dividing here — the RXUSRCLK2 from raw mode is very fast. Need a width converter (e.g. 32-bit raw → 66-bit blocks at a divided clock) so downstream stages can run at a sane frequency. Continue to use this divided clock for all subsequent stages.

### Instrumentation
State machine accessible via ILA: count sync lock acquisitions, lock losses, total slip count, and time-to-lock (cycles). Once locked, latency through this stage should be deterministic — verify with ILA timestamp delta between input and output.

---

## Step 2: PCS back-end — Descramble + 64b/66b Decode

- 64-bit payload of each block is self-synchronizing descrambled (polynomial is x⁵⁸ + x³⁹ + 1)
- 2-bit header + 64-bit payload: decode into XGMII-equivalent signals
  - 8 bytes of data/control
  - type codes (idle, start, terminate, error, ordered set are used in XGMII, but we don't really need to stick to that)

### Optimizations
- Descrambler and decoder can be fused into a single pipeline stage since descrambling is a simple XOR feedback and decode is combinational table lookup
- Parallel descramble: can unroll LFSR across 64-bit word to compute all the bits in one cycle
- Decode truth table should be sparse so LUT size and mapping can be optimized
- We don't need to adhere to XGMII (since MAC is ours too), so there are tons of cross-functional optimizations we can do with the MAC frame

### Placement/Timing
The unrolled LFSR descrambler creates a wide XOR tree — keep it physically adjacent to the decode LUTs so the combined cone stays in one clock region. If fusing descramble + decode into one stage, verify the total combinational depth still closes timing at the divided clock. Keep it wide and shallow (maximize parallelism, minimize LUT depth).

### Instrumentation
This stage sets the floor for per-block processing latency and should be deterministic once locked. Count decode errors (invalid block types), sync header errors, and total blocks processed. These are cheap counters that give you a continuous BER proxy without needing eye scan.

---

## Step 3: MAC Frame Engine (frame delineation + processing)

- Take decoded "XGMII" stream and perform frame delineation:
  - detect the Start control character → strip preamble + SFD → extract frame data → detect Terminate character → enforce minimum IPG.
  - On TX, it inserts preamble/SFD, enforces minimum frame size, and manages IPG insertion.

### Optimizations
- Cut-through forwarding: start pushing frame bytes to the host interface as soon as the destination MAC/ethertype appears, no need to wait for the full frame
- Preamble/SFD (start of frame delimiter?) detection is a fixed 8-byte pattern match that can be done combinationally
- Terminate detection can be signaled the same cycle it appears (no buffering needed)
- On TX, IPG can be minimized to the IEEE minimum (12 bytes, but deficit idle count allows averaging down to ~9.6 bytes). Since we control both ends, we can go below spec if needed.
- Pipeline depth directly adds to wire-to-host latency, so try to minimize

### Placement/Timing
Lower frequency than stages 1-2 so placement is less critical, but the cut-through forwarding path (Start detect → first byte out) must be as shallow as possible. Keep frame parsing logic physically between the Stage 2 output and Stage 4 CRC input to avoid long routing detours. Preamble/SFD match is a fixed 8-byte compare — one LUT level, no timing concern.

### Instrumentation
Frame counters (good, runt, giant, errored), cut-through latency measurement (cycles from Start detect to first byte forwarded), IPG violation counter on RX. On TX, count deficit idle adjustments. All accessible via AXI-Lite register reads or ILA.

---

## Step 4: CRC/FCS + AXI-Stream Host Interface

- Compute CRC-32 over the frame (everything from dest MAC to end of payload) and compares against the 4-byte FCS trailer. On TX, it appends the computed CRC. The result is packaged into an AXI-Stream interface (tdata, tkeep, tlast, tvalid/tready) for Corundum's ingress pipeline.

### Optimizations
- **FCS bypass:** skip FCS checking entirely to eliminate the delay of waiting for the CRC result before signaling frame-good. In the hard MAC this costs ~9 cycles; in soft logic we simply don't build the wait.
- **Parallel CRC:** instead of serial byte-at-a-time CRC, compute CRC across the full datapath width (e.g. 8 bytes/cycle at 10G with 64-bit bus) in one cycle using a precomputed XOR matrix — single biggest win.
- **Speculative forwarding:** forward the frame immediately and deliver the CRC pass/fail result as a sideband signal after-the-fact (let software or Corundum decide whether to drop)
- AXI-Stream bus width and backpressure handling affect throughput at line rate

### Placement/Timing
The parallel CRC XOR matrix is a wide combinational block — may need one pipeline register if it doesn't close timing at the target clock. The AXI-Stream interface must be placed near Corundum's ingress / PCIe region, so this stage naturally bridges between the transceiver-adjacent PCS logic and the PCIe-adjacent NIC logic. Watch for long routing here.

### Instrumentation
CRC error counter, frame good/bad counters, AXI-Stream backpressure cycles (count cycles where tready is deasserted). End-to-end latency measurement: latch a free-running counter at Stage 1 block sync output, compare at Stage 4 AXI-Stream tlast — gives total soft-logic pipeline latency per frame.

---

## Summary

| Stage | Layer | Key Function | Primary Optimization Lever |
|-------|-------|-------------|---------------------------|
| 1 | PCS front-end | Gearbox + block sync | Parallel candidate alignment, lock latency |
| 2 | PCS back-end | Descramble + 64b/66b decode | Fused single-cycle pipeline, unrolled LFSR |
| 3 | MAC | Frame delineation + IPG | Cut-through forwarding, minimal buffering |
| 4 | MAC / Interface | CRC-32 + AXI-Stream to Corundum | Parallel CRC, FCS bypass, speculative forwarding |

Each stage is a self-contained pipeline block with a clean interface (66-bit blocks between 1→2, "XGMII" between 2→3, framed data between 3→4, AXI-Stream out of 4), so they can be independently optimized and benchmarked for latency.

---

## Final Timing Closure

Two main concerns:

1. **Throughput:** the clock feeding the PCS front-end needs to sustain line rate, timing on the gearbox at this frequency is the throughput bottleneck
2. **Latency:** the total pipeline depth (registers) across all 4 stages is what determines wire-to-software latency. minimizing stages 2-4 at the slower divided clock is where we win on latency

front-end clock limits our throughput but comb. depth after PCS front end limits latency. Need to basically target both.
