# Legacy course code

Unmodified NMPC formulations from a university course assignment (2023), kept
as the starting point for this project. They are not imported by the
`overtake_nmpc` package.

| File | Notes |
| --- | --- |
| `nmpc_takeover_student.py` | Kinematic-bicycle overtaking NLP in the leader's frame. |
| `nmpc_dragracing_student.py` | Dynamic-bicycle NLP with tire forces. Needs the course `sim` and `utils` modules, which are not included, so it does not import. |
| `case_1.mat` … `case_5.mat` | Warm starts for a 30-step version of the drag-racing NLP. |

## Known issues

Takeover:

- Line 32 computes the slip angle with `atan(δ)` where the kinematic bicycle uses `tan(δ)`.
- Lines 47–48: the lateral-velocity cost terms are not squared.

Drag racing:

- Lines 126–128: tire-saturation and slack costs are `TODO` stubs.
- Line 82: friction-cone slip angles use the model symbol `xm` instead of `x[:, k]`.
- Line 49: constrains `Fyf` and `Fyr` to zero instead of tying them to the auxiliary variables.
- Line 138: the lower bound on `F_x` conflicts with the power limit and minimum speed.
- Line 95: terminal cost uses `x[5, k]` instead of `x[5, N]`.
- Line 118: rear saturation limit reuses front-axle values.
