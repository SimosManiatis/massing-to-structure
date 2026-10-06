# Known issues (open, as of v38)

| ID | Sev | Area | Issue |
|---|---|---|---|
| KI-47 | M | piles | Compression and tension capacities are inputs (or assumed 0.35 Fr;d for tension); no CPT-based calculation, negative skin friction, group effects or horizontal loads. |
| KI-48 | L | foundation | Lift pit, deeper core poer, openings in funderingsbalken not modelled. |
| KI-45 | L | frame beams | 2.60 m clear height assumed (Bbl not checked); band beams wider than the column: punching/torsion not checked. |
| KI-57 | L | poeren | Pile punching without the 2d/a enhancement (conservative). |
| KI-69 | L | wind | cs·cd for h ≥ 50 m or h/b ≥ 5 (NB annex C) only flagged, not computed. |
| KI-42 | M | stability | Door openings in core walls, lintel shear deformation, dynamic response not checked. |
| KI-27/28 | M | programme | No end-facade/corner units; core continuity on setbacks not checked; penthouse cannot use a bay cut by a setback (INCOMPLETE on setback cases). |
| KI-35 | M | programme | Max-NLA mix only; "every type at least one" fixed. |
| KI-41/34 | M | windows | Windows input removed (parked); openings in walls not modelled. |
| KI-73 | L | tests | End-to-end `run()` tests (fake Rhino) exist only on E:; C: tests cover the calculation functions. |
| KI-74 | L | take-off | Internal walls are an estimate per apartment type (no floor plans); wall m2 gross (no openings); steel/timber column m3 not given (hollow/solid sections listed by side only). |
| KI-75 | L | foundation | Pile adding is greedy, not monotonic: timber 25x15x12 gives 89 piles at Ft 900 kN but 92 at Ft 1750 kN. |
| KI-76 | M | programme | Mix is very sensitive to 0.1 m of allowance (studio vs one-bed bays swap between materials); no mix-target input. |
| KI-77 | M | programme | Apartment_GEN 1/2: bay widths come from re-weighted knapsack candidates, not a joint optimisation, so exact counts that need a different bay set can be missed (MIX_DEVIATION reported). Penthouse runs take whole bays on the top storey. |
| KI-78 | M | overhang | Overhangs only in wall mode; cantilever walls use a simple lever-arm chord model (no openings, deflection, vibration, connections); hung frame beams in along-overhangs are simply supported; corner zones carried by the end lines. |
| KI-79 | L | report | Frame-beam headroom equal to the target at 2 decimals (2.40 vs 2.40) is still flagged BELOW TARGET and option (A) reads 'storey height 3.20 m instead of 3.20 m (+0.00 m)'; compare with a 5 mm tolerance. |
| KI-80 | M | overhang | A frame line whose overhang no beam carries still gets a placeholder storey-high wall (reported NOT CLAMPED) to keep the load path; it has nothing to clamp into. Seen in the 6 m timber test. |
| KI-81 | FIXED v56 | programme | Unit boxes are area/width deep against the facade, so they do not reach the corridor (no entrance side); the gap is reported as unused depth / deep strip but has no box. |
| KI-82 | FIXED v56 | overhang | Timber along-overhangs (Simos B, 4.3 m) still fall back to storey-high facade/corridor walls, which do not stack (same concern as v54, along direction). |
| KI-83 | L | programme | Only the outermost overhang end is treated as a facade; intermediate facades of nested overhangs keep a t/2 inset. |
| KI-84 | L | programme | Apartment boxes are full band depth but NLA counts type areas, so box area (e.g. 2703 m2) exceeds NLA (2085 m2) on Simos B; the difference is depth a real plan would use for storage or larger units. |
| KI-85 | M | overhang | Steel inserts in Timber/Concrete: root connection, fire protection and the steel-timber interface are not designed. |
| KI-86 | M | programme | Overhang-storey apartments can be deeper than the 10 m daylight depth (13.07 m on Simos B storeys 3/7); flagged DEEP UNITS, daylight not checked. |
