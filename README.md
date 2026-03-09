```markdown
# MSDF FC-MNIST Demo (784×10) on Zynq (PYNQ-Z2) — AXI4-Lite Peripheral

This repository contains the **hardware design (VHDL + Vivado Block Design integration)** for an end-to-end demo of an **MNIST fully-connected layer (784 → 10)** implemented using **radix-2 MSDF (Most-Significant-Digit-First) online arithmetic** building blocks.

The design is packaged as a **single AXI4-Lite memory-mapped peripheral** for Zynq-7000 (e.g., **PYNQ-Z2 / Zynq-7020**), enabling the PS (ARM) to:
- write **activations (X)**, **weights (W)**, and **biases (B)** via AXI-Lite windows,
- pulse **START**,
- poll **DONE**,
- read back **logits** and **cycle count**.

> Note: This repo focuses on the **synthesizable design + integration** (bitstream/report generation).  
> Simulation testbenches exist in companion verification repos / versions of the project (tile scan, FC regression, AXI-lite TBs, etc.).

---

## 1) What this design demonstrates (thesis focus)

### MSDF / Online Arithmetic aspects
- **Digit-serial (MSDF) computation** with bounded online delay (δ) and signed-digit behavior in the compute chain.
- **Tile-based SOP**: the 784-dot-product is decomposed into tiles (e.g., 8 lanes per tile), each processed by an MSDF tile engine, followed by reconstruction.
- **DSP-free datapath** for the MSDF arithmetic core (resource use shifts to LUT/FF + BRAM).

### System/FPGA integration aspects
- Full **AXI4-Lite “PS ↔ accelerator”** shell:
  - Register map + load windows (X/W/B) + control + readback (logits/cycles)
- Vivado Block Design (PS7 + AXI interconnect + reset + custom IP)
- Reports for **utilization** and **timing** (post-synth/post-route).

---

## 2) High-level architecture

```

PS (ARM)  ──AXI4-Lite──►  msdf_accel_axi_top
├─ msdf_axi_lite_regs      (address decode + windows)
└─ msdf_accel_top          (compute wrapper)
├─ msdf_actv_mem       (X memory, 784 deep)
├─ msdf_weight_mem     (W memory, [neuron,tile] indexed)
├─ msdf_bias_mem       (B memory, 10 deep)
└─ msdf_fc_ctrl_10     (FC controller, tile sequencing)
└─ msdf_tile_sop + msdf_tile_recon_r2 (MSDF tile engine)

````

---

## 3) Repository contents (typical)

- `rtl/`  
  Synthesizable VHDL modules (accelerator top, AXI regs, memories, controller, MSDF core blocks).
- `bd/` or `vivado/`  
  Vivado block design wrapper / scripts (if included).
- `reports/` or `reports_routed/`  
  Post-synthesis and post-route reports (utilization, timing, clocks, IP, etc.).
- `constraints/`  
  XDC constraints (clock, resets, etc.) if used.
- `vectors/` (optional)  
  Reference text vectors used by simulation TBs / host emulation (w_fixed.txt, x_fixed.txt, b_fixed.txt, logits_exp.txt).

If your folder names differ, adjust paths accordingly.

---

## 4) Build prerequisites

### Tools
- **Vivado 2024.1** (Linux recommended; design verified with Vivado 2024.1 toolchain)
- Board/part: **xc7z020clg400-1** (PYNQ-Z2 / Zynq-7020 class)

### Hardware assumption
- Single clock domain from PS **FCLK_CLK0** (commonly configured at **100 MHz**, 10 ns).

---

## 5) How to build the bitstream (Vivado)

### A) Open project and generate bitstream
1. Open the Vivado project (`.xpr`).
2. Ensure the BD contains:
   - `processing_system7`
   - AXI interconnect
   - `msdf_accel_axi_top` as an AXI4-Lite slave
3. Run:
   - **Synthesis**
   - **Implementation**
   - **Generate Bitstream**

### B) Export XSA (optional, for Vitis)
If you plan to use Vitis later:
- `File → Export → Export Hardware` (include bitstream) to produce an `.xsa`.

---

## 6) Report generation (post-route, routed design)

From **Tools → Tcl Console**:

```tcl
open_run impl_1
set RPT_DIR "/home/bad32576/MSDF_ZIPPED_VERSION/FC_MNIST_DEMO_1/reports_routed"
file mkdir $RPT_DIR

report_timing_summary -delay_type max -report_unconstrained -check_timing_verbose \
  -file "$RPT_DIR/post_route_timing_summary.rpt"

report_timing -delay_type max -max_paths 10 -nworst 10 -path_type full \
  -file "$RPT_DIR/post_route_timing_max10.rpt"

report_timing_summary -delay_type min -report_unconstrained -check_timing_verbose \
  -file "$RPT_DIR/post_route_timing_summary_hold.rpt"

report_timing -delay_type min -max_paths 10 -nworst 10 -path_type full \
  -file "$RPT_DIR/post_route_timing_min10_hold.rpt"

report_utilization -hierarchical \
  -file "$RPT_DIR/post_route_utilization_hierarchical.rpt"

report_clocks -file "$RPT_DIR/post_route_clocks.rpt"
report_clock_utilization -file "$RPT_DIR/post_route_clock_utilization.rpt"
````

---

## 7) AXI4-Lite register map (offsets)

> The peripheral is memory-mapped at a **base address** chosen in Vivado Address Editor
> (example base: `0x4000_0000`).
> The RTL decodes **offsets** (low address bits). Add your base in software.

### Control/Status

| Offset | Name    | Access | Description                               |
| -----: | ------- | ------ | ----------------------------------------- |
| 0x0000 | CTRL    | W      | bit0: START (W1P), bit1: CLEAR_DONE (W1P) |
| 0x0004 | STATUS  | R      | bit0: BUSY, bit1: DONE                    |
| 0x0008 | CYCLES  | R      | cycles_total (32-bit)                     |
| 0x000C | CONFIG0 | R      | (implementation-defined)                  |
| 0x0010 | CONFIG1 | R      | (implementation-defined)                  |

### Logits window (readback)

| Offset range     | Access | Description                        |
| ---------------- | ------ | ---------------------------------- |
| 0x0100 + 8*j + 0 | R      | logit[j] low 32 bits               |
| 0x0100 + 8*j + 4 | R      | logit[j] high bits (sign-extended) |

* `j = 0..9`
* Each logit is **signed ACC_BITS** (default 40-bit).

### Activation window (write)

| Offset range | Access | Description                                  |
| ------------ | ------ | -------------------------------------------- |
| 0x1000 + 4*i | W      | activation X[i] (lower 16 bits used, signed) |

* `i = 0..783`

### Bias window (write)

| Offset range | Access | Description                            |
| ------------ | ------ | -------------------------------------- |
| 0x2000 + 4*j | W      | bias B[j] (lower 16 bits used, signed) |

* `j = 0..9`

### Weight staging + commit (write)

|       Offset | Access | Description                               |
| -----------: | ------ | ----------------------------------------- |
|       0x3000 | W      | select neuron index for weight commit     |
|       0x3004 | W      | select tile index for weight commit       |
| 0x3010 + 4*k | W      | stage lane k (lower 16 bits used, signed) |
|       0x3030 | W      | COMMIT: writes staged lanes as one tile   |

* Tile lanes: `k = 0..TILE_LEN-1` (typically 8)
* Tile index: `t = 0..N_TILES-1`

---

## 8) Data formats (important for software)

* **Activations (X)**: signed 16-bit written via AXI (lower 16 bits taken), resized internally to `w_fixed_t`.
* **Weights (W)**: staged as signed 16-bit per lane, resized internally to `W_TOTAL_BITS`.
* **Bias (B)**: signed 16-bit via AXI (lower 16 bits).
* **Logits**: signed `ACC_BITS` (default 40-bit) read back as two 32-bit words.

> Recommendation: Always write values **sign-extended** into the 32-bit AXI word.
> (i.e., `int16 → int32` sign extension in software)

---

## 9) Typical host flow (PS software / PYNQ)

1. Write all **weights** (tile-wise: select neuron + tile, write lanes, commit).
2. Write all **biases**.
3. For each inference case:

   * write all **activations**
   * pulse `CLEAR_DONE`
   * pulse `START`
   * poll `STATUS.DONE`
   * read logits from `LOGITS` window
   * optionally read `CYCLES`

---

## 10) Timing closure note (demo realism)

This design is functionally verified in simulation; for hardware reliability you must also be **timing-clean** at your target clock.

If WNS is negative at 100 MHz, typical low-risk fixes are:

* add **AXI register slice / interconnect pipelining** in Block Design, and/or
* pipeline AXI read/write response inside `msdf_axi_lite_regs` (AXI-Lite allows variable latency), and/or
* reduce PS FCLK0 frequency for a demo-safe configuration.

See `/reports_routed/` for current timing summary and worst paths.


