# pipette_4bar

**English** | [中文](README_zh.md) | [日本語](README_ja.md)

Dimensional synthesis of a **dual four-bar linkage** driven by a single motor. The two linkages (fb1, fb2) share one motor flange; their end points move alternately along a vertical line at x = -30 mm: when one moves down, the other lifts. The script optimizes all link lengths and the motor stroke, then animates the result with the [`one`](https://github.com/wanweiwei07/wrs2) viewer.

![motion](docs/images/motion.gif)

## 1. Coordinate system and naming

![schematic](docs/images/schematic.png)

| Symbol | Meaning |
|---|---|
| O1 = (0, 0) | Motor flange axis, the origin. The cranks of fb1 and fb2 are both fixed on it |
| O2 = (0, f) | Rocker pivot, on the +y axis (f = frame length) |
| A | Crank / connecting-rod joint |
| B | Connecting-rod / rocker joint |
| P | End point, rigidly attached to the rocker (the rocker is a triangle O2-B-P) |
| fb1, fb2 | The two four-bar linkages. fb2's crank angle = fb1's crank angle + δ (fixed offset, same flange) |

Units are mm in the optimization and m in the viewer.

## 2. Design requirements

| Requirement | Value | Parameter in code |
|---|---|---|
| End points move along a vertical line | x = -30 mm | `X_LINE` |
| Forward motor stroke: fb1 end moves -y | ≥ 25 mm, monotonic | `STROKE1` |
| Forward motor stroke: fb2 end lifts (+y) | ≥ 25 mm, monotonic | `STROKE2` |
| Start of fb2 end relative to fb1 end | y₂ − y₁ = −25 mm | `START_OFFSET` |
| Backward motor stroke | reversed: fb1 lifts, fb2 moves -y | (automatic) |
| Both cranks point to +x | crank tip x ≥ 0 over the stroke | `W_CRANK_DIR` |
| Revolute joints A, B stay in band | −15 ≤ x ≤ 20 mm | `JOINT_X_RANGE` |
| Transmission angle | 40° ≤ μ ≤ 140° | `MU_MIN` |

## 3. Design variables

Per linkage (×2): `[f, a, b, c, p, β, σ]`

| Variable | Meaning | Range |
|---|---|---|
| f | frame O1-O2 | 20 – 60 mm |
| a | crank O1-A | 10 – 30 mm |
| b | connecting rod A-B | 10 – 60 mm |
| c | rocker O2-B | 10 – 50 mm |
| p | O2-P distance | 10 – 50 mm |
| β | angle from O2B to O2P | −180° – 180° |
| σ | assembly branch | ±1 |

Shared: motor start angle θ₀, crank offset δ, motor direction (CW/CCW), motor stroke angle (45° – 180°).

## 4. Method

- **Kinematics**: for each motor angle, B is the intersection of circle(A, b) and circle(O2, c) on branch σ; P is obtained by rotating O2B by β and scaling to p. A configuration that cannot be assembled is rejected.
- **Cost** (summed over fb1 and fb2): straightness max|x_P + 30| + penalties for non-monotonic motion, stroke shortage, start-offset error, transmission angle, crank direction, and joint x range.
- **Solver**: `scipy.optimize.differential_evolution` (Sobol init, parallel workers). One run takes about 10 s.

## 5. Result (seed = 0)

| | f | a | b | c | p | β | σ |
|---|---|---|---|---|---|---|---|
| fb1 | 33.34 | 21.48 | 53.16 | 17.21 | 31.30 | 168.0° | −1 |
| fb2 | 33.34 | 13.67 | 32.52 | 12.92 | 31.30 | −42.8° | +1 |

Motor: starts at −83.0°, turns 56.6° CCW; fb2 crank offset δ = 57.4°.

| Item | fb1 | fb2 |
|---|---|---|
| End point travel | 45.84 → 20.84 (−25.00 mm) | 20.84 → 45.84 (+25.00 mm) |
| Max \|x − (−30)\| | 1.30 mm | 1.30 mm |
| Transmission angle | 60.1° – 86.0° | 61.9° – 123.8° |
| Joint A x | 2.60 – 19.23 mm | 11.72 – 13.67 mm |
| Joint B x | 14.01 – 17.21 mm | −12.20 – −5.18 mm |

![poses](docs/images/poses.png)

![performance](docs/images/performance.png)

## 6. Notes

- **Straightness limit.** P is on the rocker, so it always moves on an arc centered at O2. The best an arc can do against a straight segment of length s is a deviation of about s² / (16 p): 0.47 mm for 15 mm, **1.30 mm for 25 mm**, about 1.9 mm (estimated) for 30 mm. The other variables cannot reduce this; to get a straighter path, P would have to be on the connecting rod.
- **Shared pivot.** Since both end points must cover the same segment on the same line, the optimum gives f₁ = f₂ and p₁ = p₂: the two rockers share pivot O2 and both end points travel the same arc. They must be in different planes (z layers) to avoid collision; the viewer draws fb2 6 mm behind fb1.
- **Multiple optima.** Different seeds give different link lengths with the same cost. Add a secondary objective (smaller motor stroke, larger transmission angle, compactness, …) if a unique design is needed.

## 7. Usage

Requirements: Python ≥ 3.12, `numpy`, `scipy`, and [`one`](https://github.com/wanweiwei07/wrs2) (plus `pyglet` and `imgui-bundle` for the viewer).

```bash
# Windows example; make `one` importable
set PYTHONPATH=E:\src_code\one
py -3.12 four_bar_opt.py
```

The script prints the result (see section 5), saves it to `four_bar_result.npy`, and opens the viewer:

![viewer](docs/images/viewer.png)

| Item in the viewer | Meaning |
|---|---|
| Gold / purple / brown | fb1 crank / connecting rod / rocker |
| Carrot orange / pink / slate gray | fb2 crank / connecting rod / rocker |
| Black | frame and joints |
| Magenta | end points P and their paths |
| Orange line | target line x = −30 |
| Silver lines | joint x limits −15 and 20 |
| **Front view** button | look at the xy plane along −z, y up, fit the mechanism |
| **Center view** button | keep the current view direction, re-center and fit |

The motor sweeps forward and back over its stroke. Mouse: left/right drag to orbit, middle drag to pan, scroll to zoom.

To change requirements, edit the constants at the top of [four_bar_opt.py](four_bar_opt.py) (`X_LINE`, `STROKE1`, `STROKE2`, `START_OFFSET`, `JOINT_X_RANGE`, `MU_MIN`, `FB_BOUNDS`, …). To replay a saved result without re-optimizing:

```python
import numpy as np, four_bar_opt as m
x = np.load("four_bar_result.npy")
m.report(x)
m.visualize(x)
```

> On Windows the optimizer uses worker processes (`workers=-1`). Any script that calls `optimize()` must do so under `if __name__ == '__main__':`, otherwise each worker re-runs the script.
