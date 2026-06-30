# Hybrid Beamforming RTL File Hierarchy

## Organization Rule

Each SystemVerilog source file contains exactly one module. Compile order is captured in the RTL filelist.

## Top-Level Hierarchy

```text
hybrid_beamforming_arch_top
|
|-- srs_ls_estimator
|-- dft_mmse_estimator
|-- partial_svd_engine
|-- pe_altmin_engine
|   |-- pe_altmin_frf_engine
|       |-- frf_a_builder
|       |-- svd8_procrustes_stage
|       |-- frf_m_builder
|       |-- frf_phase_projector
|       |   |-- frf_phase_projector_lane
|       |-- frf_convergence_check
|-- fbb_solver
|-- coeff_ram
|-- digital_precoder_apply
|-- analog_precoder_apply

pe_altmin_matrix_memory
|-- complex_1w2r_ram
```

## File Map

| File | Module | Role and interaction |
| --- | --- | --- |
| `hybrid_beamforming_arch_top.sv` | `hybrid_beamforming_arch_top` | Top-level architecture. Starts each coefficient update block and connects active coefficients to the runtime precoder path. |
| `srs_ls_estimator.sv` | `srs_ls_estimator` | First update block. Produces the initial channel estimate for the DFT-MMSE estimator. |
| `dft_mmse_estimator.sv` | `dft_mmse_estimator` | Filters the Least Squares estimate and feeds the partial SVD engine. |
| `partial_svd_engine.sv` | `partial_svd_engine` | Produces `F_opt`, the fully digital reference precoder used by PE-AltMin and the baseband solver. |
| `pe_altmin_engine.sv` | `pe_altmin_engine` | Wrapper between the top-level update path and the Radio Frequency precoder engine. Passes warm-start control and previous `F_RF`. |
| `pe_altmin_frf_engine.sv` | `pe_altmin_frf_engine` | Main PE-AltMin controller. Sequences A build, Procrustes, M build, phase projection, and convergence check. |
| `frf_a_builder.sv` | `frf_a_builder` | Computes `A = F_opt^H * F_RF`. Its output feeds the Procrustes stage. |
| `svd8_procrustes_stage.sv` | `svd8_procrustes_stage` | Placeholder for the small SVD/Procrustes operation. Produces `F_DD` for the M Matrix Builder. |
| `frf_m_builder.sv` | `frf_m_builder` | Computes `M = F_opt * F_DD^H`. Its output feeds the phase projection engine. |
| `frf_phase_projector.sv` | `frf_phase_projector` | Walks through `M` and dispatches entries to phase projection lanes to produce updated `F_RF`. |
| `frf_phase_projector_lane.sv` | `frf_phase_projector_lane` | Projects one complex `M` entry into one unit-magnitude `F_RF` coefficient. Intended to be replaced by a true phase lane. |
| `frf_convergence_check.sv` | `frf_convergence_check` | Decides whether the PE-AltMin loop runs another iteration. Currently iteration-count based. |
| `pe_altmin_latency_stage.sv` | `pe_altmin_latency_stage` | Reusable start/busy/done timing stage used by placeholder blocks. |
| `pe_altmin_matrix_memory.sv` | `pe_altmin_matrix_memory` | Groups local storage for `F_opt`, `F_RF`, `A`, `F_DD`, and `M`. Used as the future PE-AltMin memory interface. |
| `complex_1w2r_ram.sv` | `complex_1w2r_ram` | Simple complex memory primitive with one write port and two read ports. Instantiated by local matrix storage. |
| `fbb_solver.sv` | `fbb_solver` | Placeholder for the Baseband Precoder Solver after `F_RF` is available. |
| `coeff_ram.sv` | `coeff_ram` | Stores active `F_RF` and `F_BB` coefficients for runtime use. |
| `digital_precoder_apply.sv` | `digital_precoder_apply` | Runtime path block that applies active `F_BB` to input data streams. |
| `analog_precoder_apply.sv` | `analog_precoder_apply` | Runtime path block that applies active `F_RF` to produce antenna-domain outputs. |

## Filelists

| Filelist | Purpose |
| --- | --- |
| `rtl/hybrid_beamforming_rtl.f` | Full RTL compile order. |
| `tb/pe_altmin_tb.f` | PE-AltMin focused compile order. |

## Visual Datapath

The diagram below shows how coefficient-update data moves through the main RTL building blocks.

![Hybrid beamforming datapath](hybrid_beamforming_architecture_simple.svg)

## Notes

Most estimator, Procrustes, and solver blocks currently keep final interfaces stable while the functional datapaths are filled in. The PE-AltMin controller, matrix builders, phase scheduler, and local memory skeleton are the active design focus.
