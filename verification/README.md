# Independent numerical recheck

**Audit date:** 26 November 2026.  
**Source:** [ATMD_Quarter_Car_Colab.ipynb at the audited revision](https://github.com/IhsanSA/atmd-quarter-car-control/blob/b412fa0bd8a154fd8d368998faff9544ef75a5ab/ATMD_Quarter_Car_Colab.ipynb) (blob `e44ade6a6dbbdd0d316a0a8fcfd98ae2ccaa1624`).

The Python notebook is the canonical study. This audit is an independently implemented numerical calculation in JavaScript, using the same model, parameters, scenarios, sampled controllers, actuator saturation and end-stop law. It is not a claim that the Python kernel or every notebook cell was executed again.

## Scope and results

| Check | Outcome |
|---|---|
| Passive TMD design search | 336 designs, 121 frequency points each; optimum reproduced |
| Controller tuning | 276 candidates × 2 training scenarios = 552 trajectories; selected PID and LQR pair reproduced |
| Time-domain evaluation | 9 road/parameter scenarios × 4 configurations = 36 trajectories |
| Comparison with saved notebook table | 360 displayed fields checked; 0 mismatches at displayed precision |
| Frequency-domain response | Continuous linear road-to-body gain and active-benefit intervals reproduced |
| Integration step refinement | Six additional traces at 0.25 ms rather than 0.5 ms; controller update period kept at 2 ms |
| Monte Carlo and published reference cases | Saved notebook results retained; these parts were not independently rerun |

The time-domain comparison uses the saved notebook table's two-decimal resolution (±0.005 of the printed unit, plus floating-point tolerance); contact counts, screen results, and absent values are checked exactly. The original eigenvalue-based candidate stability filter was not rerun; all 276 candidates were reported to pass it in the saved notebook, and the same candidate set was re-evaluated.

## Nominal sinusoidal scenario

Road amplitude: 4 mm. Frequency: 1.2 Hz. Simulation duration: 8 s. Peak includes the complete simulation; RMS uses 5.5–8.0 s.

| Configuration | Peak body displacement (mm) | RMS body displacement (mm) | Peak reduction vs passive TMD (%) | RMS actuator force (N) | Peak relative travel (mm) |
|---|---:|---:|---:|---:|---:|
| Standard | 7.643035 | 5.404441 | -11.042246 | 0.000000 | — |
| Passive TMD | 6.882998 | 4.864421 | 0.000000 | 0.000000 | 18.695502 |
| ATMD-PID | 6.097577 | 4.189558 | 11.411034 | 3.513281 | 36.642627 |
| ATMD-LQR | 6.158067 | 4.277445 | 10.532200 | 3.018679 | 27.863792 |

LQR RMS actuator force is 14.08% lower than PID in this case. Force is not equivalent to electrical energy.

## Selected parameters

- Passive TMD: stiffness 488.030665 N/m; damping 23.978432 N·s/m.
- PID: $K_p=800$, $K_i=96$, $K_d=35$, stroke feedback mix 0.
- LQR: force scale 70.94470796 N; stroke weight 0.02. Gain for the state order `[z_s,z_u,z_a,v_s,v_u,v_a]`: `1631.910350, -177.231504, -60.684493, 48.627560, 2.599548, -36.997218`.
- Training target 6.0685%; gap between selected average reductions 0.2721 percentage points. The selected gains match the notebook.

## Frequency response and physical limits

| Configuration | Road-to-body gain at 1.2 Hz (m/m) | Phase (deg) | Interval with lower gain than passive TMD (Hz) |
|---|---:|---:|---|
| Standard | 1.910764 | -53.243 | — |
| TMD | 1.719366 | -50.959 | — |
| PID | 1.480525 | -47.704 | 0.884604–1.700372 |
| LQR | 1.512907 | -53.574 | 0.763446–1.938441 |

For the 30 mm, 1.2 Hz road input, peak displacement was 56.242 mm (TMD), 57.144 mm (PID), and 55.849 mm (LQR). The relative changes were −1.60% for PID and +0.70% for LQR, using positive values to indicate additional reduction. Halving the integration step changed nominal peak displacement by less than 0.000024 mm and high-amplitude peak displacement by less than 0.093 mm for the three attached-damper configurations. Contact counts were unchanged (17, 32, and 32 in the high-amplitude case).

## How to reproduce the independent recheck

From the repository root, with Node.js installed:

```sh
node verification/recheck.mjs
```

To repeat the passive search and controller-training comparison as well:

```sh
node verification/recheck.mjs --tuning
```

These commands require no additional packages, use the source notebook's embedded measured-road data, and return a JSON report. The numerical code was executed in a JavaScript runtime, including the optional tuning calculation. The thin Node.js command-line wrapper was not separately run because a command-line runtime was unavailable during this audit.

The audit is intentionally tied to the current notebook's parameter set, saved output format, and selected design. Review and update it if the underlying study changes. Mechanical modeling and numerical agreement do not establish physical vehicle validation.
