# Hybrid Beamforming RTL Progress and Plan

## Current Direction

Build a functional RTL baseline first. Optimize latency only after the full coefficient update path is represented clearly in hardware.

## Main Building Blocks

```text
Least Squares Channel Estimator
-> Discrete Fourier Transform Minimum Mean Square Error Estimator
-> Partial Singular Value Decomposition Engine
-> Phase Extraction Alternating Minimization Engine
-> Baseband Precoder Solver
-> Coefficient Memory
-> Runtime Digital and Radio Frequency Precoder Apply
```

## Progress Snapshot

| Building block | Status | Notes |
| --- | --- | --- |
| Top-Level Coefficient Update Controller | In place | Schedules the update path from channel estimation through coefficient write. |
| Least Squares Channel Estimator | Placeholder | Interface is stable; numeric datapath still needed. |
| Discrete Fourier Transform Minimum Mean Square Error Estimator | Placeholder | Interface is stable; filtering datapath still needed. |
| Partial Singular Value Decomposition Engine | Placeholder | Produces the target fully digital precoder for later stages. |
| Phase Extraction Alternating Minimization Controller | In place | Schedules A build, Procrustes, M build, phase projection, and iteration check. |
| A Matrix Builder | Initial functional design | Fixed-point complex multiply-accumulate scheduler exists. |
| M Matrix Builder | Initial functional design | Fixed-point complex multiply-accumulate scheduler exists. |
| Phase Projection Engine | Initial functional structure | Lane scheduler exists; true angle-to-unit-vector lane is next. |
| Local Matrix Storage | Initial functional structure | Simple local storage exists for the PE-AltMin matrices. |
| Procrustes Stage | Placeholder | Needs a functional small Singular Value Decomposition strategy. |
| Baseband Precoder Solver | Placeholder | Needs a functional solver architecture. |

## Current Focus

The immediate goal is to complete a clear functional path for the Phase Extraction Alternating Minimization update:

```text
F_opt and current F_RF
-> A = F_opt^H * F_RF
-> Procrustes update, F_DD = V * U^H
-> M = F_opt * F_DD^H
-> F_RF = exp(j * angle(M))
-> iteration check
```

## Next Design Steps

1. Connect the A Matrix Builder and M Matrix Builder to local matrix storage.
2. Replace the current simple phase projection lane with a true lookup-table or Coordinate Rotation Digital Computer based phase lane.
3. Define the functional Procrustes stage for the small Singular Value Decomposition operation.
4. Add the Baseband Precoder Solver architecture.
5. Complete one end-to-end coefficient update pass.

## Optimization Later

After the functional baseline is complete, revisit:

```text
Multiply-accumulate lane count
Phase projection lane count
Memory banking
Pipeline depth
Early convergence
Resource sharing
```

## References

The design direction is based on the MATLAB hybrid beamforming model and the supplied hybrid precoding reference papers.