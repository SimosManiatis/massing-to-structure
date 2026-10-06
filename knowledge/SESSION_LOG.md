# Session log (C: copy)

## 2026-10-05 · Why no beams along X / why frame beams so thick (v38 run, 25×20 × 18 concrete walls)
- Explained: slab spans one way along X between support lines, so no gravity beams along X; facade edge members are concealed 500×250 slab strips (invisible); frame beams across the 9.1 m band are wide band beams (1500–1800×550) because the selector minimises depth for the 2.60 m clear height.
- Proposed (not applied): deep narrow beams on the building-end (gable) frame lines where headroom does not matter; optional visible tie beams along X at facades/corridor edges.

## 2026-10-05 · v39 Clear_Height input + Apartments_N output
- Added the input and output as requested; 11 tests pass; archived v39.

## 2026-10-05 · Which inputs to expose
- Proposed input list (tier 1: Imposed_Load split, Facade_Load, Corridor_Width, Apartment_Depth, Concrete_Class, Core_Count; tier 2: Steel_Grade, Timber_Grade, Wall_Thickness_Min, Drift_Limit, Max_Band_Beam_Width, Common_Ratio, Core_Area; tier 3: Fire_Resistance, Mix_Shares, Typology). Nothing implemented yet.

## 2026-10-05 · v40 quantity take-off outputs
- Added 8 outputs (m1/m2 totals + Takeoff_Data rows) and the internal-wall estimate; 13 tests; archived v40.

## 2026-10-05 · v41 Facade_Load + Core_Count inputs
- Added both inputs; 15 tests; archived v41.

## 2026-10-05 · v42 Facade_Load "does not change anything"
- Verified the calculation responds; made input lookup name-tolerant and added an INPUTS RECEIVED report line to show what arrives. 16 tests; archived v42.

## 2026-10-05 · v41 reports 25x15x12 (C/S/T) reviewed
- Inputs arrive (DBG INPUT facade 1.5, clear 2.4, cores 1). Grid REVIEW_REQUIRED comes from the TENSION PILES issue (timber also headroom), not geometry.
- Stub reproduces the concrete pile count exactly (87). Tension sensitivity (assumed 612 / 900 / 1750 kN): concrete 87/87/87, steel 107/98/98, timber 108/89/92. Steel/timber are uplift-governed (pinned lintels = no coupling, less ballast); 3.7 m wide FB_line5 with 24 piles disappears with Ft >= 900.
- Mix flips between materials (C 23 studios/2 one-bed, S 1 studio/22 one-bed) because the end-line allowance 0.40 vs 0.30 m lets a 4.81 m one-bed bay fit; max-NLA objective. Proposed Mix_Target input (not applied).

## 2026-10-05 · v43 programme inputs
- Added Corridor_Width, Max_Unit_Width, Common_Ratio, Core_Area (empty = defaults). 20 tests; archived v43. Smoke run timber 25x15x12: no limit 98 piles; Max_Unit_Width 5.5 → 92 one-bed + 1 studio, 105 piles; + corridor 2.4 → 69 one-bed + 24 studios, 99 piles.

## 2026-10-05 · v43 Rhino run OK
- Simos's v43 concrete report matches v41 at defaults (inputs arrive; empty Max_Unit_Width/Core_Area use defaults). v44: blank inputs shown as '-' in INPUTS RECEIVED.

## 2026-10-05 · v45 Apartment_GEN
- Added Apartment_GEN 0/1/2 with Type_mix / Apt_Manual_Count. 24 tests; archived v45. Known limit: bays are chosen by weighted knapsack, so some feasible exact counts are missed (10/20/20/10/2 on 25x15x12 → 2-bed 16/20).

## 2026-10-05 · v46 overhangs
- Added cantilevering storeys in wall mode (both directions, auto beam/wall by length, bays extended). 27 tests; archived v46. Frame mode still nested-only.

## 2026-10-05 · v47 overhang Breps
- Simos modelled overhangs as extra Breps at the same storey heights → "Storeys must be contiguous; one Brep per elevation". v47 merges Breps with the same z-range into one storey if they form a rectangle (error for L/T shapes or Breps overlapping in height). 28 tests; archived v47.

## 2026-10-05 · v48 Brep slicing
- v47 still gave "Storeys must be contiguous" in Rhino (likely the old script in the component or overhang boxes not exactly on storey heights). v48: all Brep z-values snapped (2 cm) into storey planes; every Brep is sliced into the storeys it spans and merged per storey; errors list the received Brep heights; ERROR line shows the script version. 28 tests; archived v48.

## 2026-10-05 · v49 Breps diagnostics
- v48 error showed 7 Breps received (storeys 19.2-22.4 and 25.6-28.8 missing) although the Rhino model has 9 boxes; the two missing are exactly the two extended (overhang) storeys. v49: counts items on the Breps wire incl. nulls and errors if some are null; gap error names the count. Awaiting Simos.

## 2026-10-05 · v50 clamped cantilevers
- Simos agreed: cantilever walls only on party/core/gable lines (continuous), frame lines beams with hogging-designed frame beam behind, along-overhangs beams with back-span beams. 28 tests; archived v50.

## 2026-10-05 · v51
- Tip edge members, units stretched into narrow overhang strips, back-span headroom check. 29 tests; archived v51.

## 2026-10-05 · v51 Rhino run 20x20x9 with overhangs (Type_mix 30/30/20/15/5)
- Runs, balance exact, all PASS. Findings: NLA/GFA 32.9%: band depth 9.07 m with 25-40 m2 units (min frontage 4.0 m -> a studio slot is 38 m2), mode-1 fills 7.45/7.62 m bays with ONE small unit each (bay set from weighted knapsack, no small bays), core bay side 1 never used (4.4 m x 9.07 m per storey), 4.00 m left overhang strip empty (< 4.2 m, neighbour is the core so no stretch), top storey only 2 penthouses. Back-span beams RC 600x1150 in the core bay (2.05 m clear) duplicate the core long walls. Proposed (not applied): exhaustive bay-set enumeration in Apartment_GEN 1/2 with slot-efficiency tie-break, units on the free side of the core bay, Apartment_Depth input / full-depth units, overhang strips with facade end need only 4.0+t/2, core long walls as back-span clamp.

## 2026-10-05 · same 20x20x9 overhang building in Timber/Concrete/Steel (v51)
- Programme identical in all three (NLA/GFA 32.9%) -> programme fixes are material-independent. Piles T 77 / C 76 / S 82; back-span headroom C 2.05 (RC 600x1150), S 2.15 (BOX 800x350); Timber: no along-overhang beam passes -> storey-high CLT facade/corridor walls (U up to 0.96) whose back-span pieces in the core bay sit on the CLT core long walls (overlap) -> fix #5 (core walls as clamp) also removes this. Timber frame-beam headroom 2.18 m (needs 3.42 m storey). Timber pile tension U 1.00 (assumed Ft). Awaiting go for the 5 fixes.

## 2026-10-05 · v52 U1
- Exhaustive bay sets + full-programme check of the top 8; Type_mix objective excess>area>deviation; common-slot units bug fixed. Simos B (mix 30/19/41/15/5) 34.5% -> 45.1%. 32 tests; archived v52. Next proposed: U2 (core bay free side), U5, U12.

## 2026-10-05 · Rhino test loop (computer use)
- Simos chose option B. The StructureGeneration Script component (structural_module_paratool.gh) now contains only a loader: exec(compile(io.open(r"C:\PROJECTS\StructuralGEN\src\structuralgen.py", encoding="utf-8").read(), path, "exec"), globals()). Every solve runs the current src file.
- Procedure: Grasshopper on monitor "U2790B (2)", Script Editor opens on "display 2900983654"; run with the green play button in the editor; read the Report panel (top right, yellow) via right-click > Copy Data Only + clipboard read.
- First Rhino run of v52 on Simos B (Type_mix 30/19/41/15/5): 43 apartments, NLA/GFA 45.1%, balance exact, 73 piles - identical to the stub prediction.

## 2026-10-05 · v53 U2-U12
- All implemented, 38 tests green (2 hash seeds).
- Rhino run (Simos B, Steel, Type_mix 30/19/41/15/5, Clear_Height 2.4, Core_Count 1): MATERIALS v53, 51 apartments (12/10/21/6/2), NLA 1980 m2, NLA/GFA 50.7% (v52 45.1%), target met within 5%, 70 piles (v52 73), max U 0.9994, walls 0.53, stability 0.35, balance exact, all statuses PASS. New EFFICIENCY, unit-depth, overhang root moment and deflection lines present; core walls hold 2 back-spans.
- Seen in Rhino: KI-79 (headroom equality flagged BELOW TARGET with '+0.00 m'), back-span beam 2.15 m clear on storey 0, storey 0 only 38% (common area + small units).

## 2026-10-05 · v54 overhang walls
- Simos: a load-bearing wall that does not stack vertically does not sit right. Chose 'Beams first'.
- Removed the 3 m cap; walls only when no beam passes, flagged REVIEW. 38 tests green (seeds 1, 99). Concrete beams pass up to 8 m in the 12-storey test; timber falls back to walls at 6 m.
- Rhino run v54 (Simos B, Steel, Type_mix): no cantilever walls left (wall pieces 74 -> 63; storeys 3 and 7 back to 7 pieces, all stacking); 4.0 m Y overhangs carried by 5 cantilever beams each (BOX 475x325 to 625x325, U 0.96-0.99) + 1500x250 tip strip; concrete walls 372 -> 334 m3 (-97 t); applied load 54867 -> 53994 kN; piles 70; max U 0.9995; balance exact; all PASS. End-line allowance 0.40 -> 0.38 m shifted the bays: 50 units (12/9/21/6/2), NLA/GFA 49.9% (v53 51 units, 50.7%); mix target still met. Frame-beam headroom now 2.42 m, so KI-79 does not show here (bug still open).

## 2026-10-05 · v55 support boxes
- Simos: support boxes feel broken; update the script only, no computer use.
- Found: core box shifted on storey 7, corridor 4.87 m on storeys 3/7, overhang bays empty (clear-width mismatch). Fixed all three; gable stays a frame line when only an overhang unit shares it. 39 tests green (seeds 1, 99). Not run in Rhino (asked not to).

## 2026-10-05 · v56 KI-81 + KI-82
- Simos: 'yes proceed' (both). Steel inserts before walls; full-depth apartment/common boxes; unlettable deep-strip boxes; fake_rhino hollow steel. 39 tests green (seeds 1, 99). Not run in Rhino.

## 2026-10-05 · v57 revert branch {3}
- Simos: v56 'completely broke it' - extra Support_Boxes branch, everything shows as corridor. Removed branch {3}; 39 tests green. Lesson: never add/rename output branches or columns without asking.

## 2026-10-05 · v58 deep units
- Simos asked about gaps (the v57 deep strips without box). Chose option 1: deeper apartments to the corridor, daylight flagged. 39 tests green.

## 2026-10-05 · v59 settings + Message
- Simos: suppress orange warning, compact Message output, internalise loads/slab/clear height/corridor/max unit width/common ratio/core area at the top. Done; 39 tests green. Soil pressure: no such input (piles only) - question to Simos. He must add a 'Message' output and may delete the 8 inputs on the component.
