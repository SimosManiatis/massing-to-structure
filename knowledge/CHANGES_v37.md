# StructuralGEN v37 (2026-10-05)

Built from the v35 script pasted in chat (E:\PROJECTS\StructuralGEN was not reachable this session). Copy `structuralgen_v37.py` over `E:\PROJECTS\StructuralGEN\src\structuralgen.py` and add these entries to `knowledge/CHANGELOG.md` / `04_known_issues.md` there.

## v37 — column gaps + merged pile clusters
- Wall mode: frame-line columns now run the full storey height (p[4] to p[5]). They stopped at the slab underside (p[5] − slab), leaving a slab-thick gap at every floor between the column and the one above wherever no beam passed over it (the frame beams end at the column faces). Volumes were already full height, so quantities do not change. (KI-71)
- Pile clusters (v36): clusters of neighbouring heavy columns that overlap (e.g. two corridor columns 1.8 m apart) are merged into one cluster at the load centroid with m = ⌈1.1 × 1.29 ΣP / (rows × Fr;d)⌉ positions. Before, the second cluster's piles were dropped for spacing and its load landed between piles. (KI-72)
- Includes v36: heavy column loads on a funderingsbalk get a pile cluster at 3D (rigid local cap). (KI-70)

## Checks (Rhino-free stub run)
| Case | Foundation | Piles | Column gaps | Balance |
|---|---|---|---|---|
| 25×20 × 18 concrete walls (your run) | FAIL U 1.25 → PASS | 160 × Ø500, U 1.00, 64 tension piles | 0 | 0.000 % |
| 25×30 × 14 concrete walls | PASS | 203 × Ø500, U 0.92, 57 tension | 0 | 0.000 % |
| 25×30 × 7 steel walls | PASS | 105 × Ø500 | 0 | 0.000 % |
| 20×15 × 9 timber walls | PASS | 55 × Ø500, 24 tension | 0 | 0.000 % |
