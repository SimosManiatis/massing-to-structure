# Changelog (C: copy; v9–v35 history on E:)

## v59 · 2026-10-05
- No orange warnings: removed the 'Review Report and Clash_Geometry.' and programme-status runtime warnings; only a real exception gives the red error. Everything stays in Report/Debug.
- New output variable Message (add an output named Message to the component): line 1 OK/CHECK/FAIL | apartments | NLA/GFA | piles | max U; line 2 PROGRAMME status: fits / target met / MIX OFF / NOT PLACED / ROOM LEFT (vacant m2 in manual count, empty band length); optional FAIL line; GEOMETRY/STRUCTURE: ok or the review headlines (from run issues, programme issues excluded). Exception: 'ERROR v59: ...'.
- Internalised as settings at the top of the script (section 0): FLOOR_LOAD 10.0, ROOF_LOAD 8.5, SLAB_THICKNESS 0.25, CLEAR_HEIGHT 2.40, CORRIDOR_WIDTH 1.80, MAX_UNIT_WIDTH_SETTING None, COMMON_RATIO 0.05, CORE_AREA 25.0. Removed from INPUT_NAMES; still-connected inputs with those names are ignored and listed as IGNORED in the INPUTS RECEIVED line. 'Soil pressure' does not exist (piles only) - asked Simos.
- Tests: InputLookup updated; Regression checks no runtime messages and a Message line.

## v58 · 2026-10-05
- Deep strips closed (Simos: gaps beside the corridor on storeys 3 and 7): apartment boxes on overhang storeys run from the facade to the corridor (depth = depths + strip, 13.07 m on Simos B). Unit types are still chosen for the capped 10 m depth. Report line DEEP UNITS (REVIEW daylight) replaces DEEP STRIPS; EFFICIENCY marks 'deep units on side n'. No output structure change.

## v57 · 2026-10-05
- Reverted the v56 Support_Boxes branch {3} 'unlettable' (Simos: it broke his definition, "everything is corridor"). SUPPORT_KINDS is again (core, circulation, common); deep strips are reported in text only (no box). Output tree structures must not change without asking.
- Script data checked in the harness with Simos's inputs (Steel, Apartment_GEN 0): corridor = 1.80 m strip on every storey, cores stacked. Mode 0 mix (1-bedroom heavy) is unchanged from v54 behaviour.

## v56 · 2026-10-05
- KI-82: steel inserts. When no Timber/Concrete cantilever (across or along) or back-span beam passes, a steel box section is tried before any storey-high wall (cantilever_beam(..., back) tries insert_materials(); back-span picked together with the cantilever). Members carry material='Steel', profile=True; reported as 'steel insert ...', issue OVERHANG STEEL INSERTS; Material_Quantities row 'Beams,Steel (overhang inserts)' and VOLUMES suffix. Simos B timber: 14 inserts, wall pieces 77 -> 63, no along walls.
- KI-81: apartment and common boxes run from the facade to the corridor (full band depth); NLA stays the type areas; Apartment_Data gains box_area_m2 (last column); report line 'Apartment boxes run from the facade to the corridor'. Deep strips get support kind 'unlettable' (Support_Boxes branch {3}).
- tests/fake_rhino: Brep.CreateBooleanDifference subtracts the void (hollow volume), so Steel runs end-to-end. Steel run reproduces the v54 Rhino run exactly (70 piles, U 0.9995). New regression case simos_steel_mix (70 / 53.4 / 0.9995). Overhang test: 6 m timber expects inserts; wall fallback tested by forcing select_cantilever to fail.

## v55 · 2026-10-05
- Support boxes fixed (Simos: "the way support boxes are created is a bit broken"):
  - Core boxes stack at the ground-floor position on every storey (storey 7 of Simos B had its core shifted 4 m out with a cross-overhang); clipped to the storey footprint.
  - Corridor (circulation) stays on the ground-floor corridor zone on overhang storeys (bay_geometry anchored case keeps base['zone']); it used to widen to 4.87 m and eat the overhang band. Depth beyond the 10 m daylight depth is reported as DEEP STRIPS / "deep strip" in EFFICIENCY (no box).
  - Outer overhang bay ends are facades: half() returns -te/2 there (facade_ends), so clear_width matches the U4 fill and boxes run to the facade. Simos B overhang bays now hold units (empty 8.0 m on storeys 1/5 gone).
  - A gable line shared only by an overhang unit stays a frame line (NON-load-bearing partition, reported) instead of becoming a full-height party wall.
  - Layout line reports the core box area next to Core_Area.
- Simos B concrete mix: 50 -> 55 units, NLA/GFA 49.9 -> 53.4%, structure unchanged (73 piles). Regression re-baselined (simos 53.4, simos2 53.2).
- New test SupportBoxes; removed unused mix_rows.

## v54 · 2026-10-05
- Overhangs across the building: beams first. The 3.0 m OVERHANG_BEAM_MAX cap is removed; every support line gets a cantilever beam at any length when a section passes. Storey-high cantilever walls are only a last resort on party/core/gable lines and are flagged `OVERHANG WALL (REVIEW)` because they do not stack (Simos: load-bearing walls must stack).
- Simos B: the 4 x storey-high walls on the 4.0 m Y overhangs (U 0.30) become cantilever beams.
- Tests: Overhangs case 4 m concrete expects beams and no walls; new 6 m timber case expects the flagged wall fallback.

## v53 — U2–U12 (2026-10-05)
- U2: the free side of every core bay holds apartments (largest fitting type in mode 0; optional/vacant in modes 1/2). 25x15x12 mode 0: 68 -> 79 units; Simos B Type_mix 45% -> ~50-52%; 25x15x12 counts 10/20/20/10/2 now exact.
- Modes 1/2: a type with target 0 is never placed.
- U3: Apartment_Depth input (Float, Item; empty/0 = automatic, >= 4.0 m) caps the band depth; Report line for types smaller than 4.0 m x band depth.
- U4: the outermost overhang strip (facade end) is filled with need - t/2 - te/2, so a 4.00 m strip holds a studio.
- U5: along-overhang cantilevers whose back-span bay is a core bay on the core long-wall lines are clamped by the core walls (no back-span beam; V into the core walls). Report counts them.
- U7/U8: overhang report gives the largest root moment (joint not designed) and the deflection utilisation (equivalent 2L check, explained as conservative).
- U9: EFFICIENCY line per storey (NLA/GFA, empty band length, unused slot depth).
- U10: pile_line tries 1-4 rows and keeps the fewest piles, then removes unneeded inserted piles; timber 25x15x12 107 -> 98 piles. Not strictly monotonic in Pile_Tension_Capacity (poer size changes the neighbouring beam segment, +-3%).
- U11: tests/fake_rhino.py runs the whole component (run(), outputs, null-Brep check) without Rhino (Concrete/Timber; Steel hollow geometry not faked).
- U12: Regression test on 5 reference buildings (piles +-2%, NLA +-2 pp, U <= 1, balance exact).
- Determinism fix: equal-width bay candidates sorted by name (results depended on PYTHONHASHSEED).
- U6 verified: frame beams behind cross cantilevers are in the apartment headroom check.
- Tests: 38.

## v52 — U1 exhaustive bay sets for Apartment_GEN 1/2 (2026-10-05)
- Candidate bay sets = every maximal combination of type bay widths per free segment (BAY_SET_LIMIT 400, combined across segments) + the old weighted-knapsack variants; light evaluation (assign on bay classes, share tolerance x BAY_SET_MARGIN 0.5) ranks them, the best BAY_SET_FULL_CHECKS (8) are run through the full programme (forced_bays) and the best real result is kept.
- Type_mix objective: first total share excess beyond 5 pp, then apartment area, then total deviation (was: deviation then area).
- Bug fixes: units placed beside the ground-floor common area were counted in the mix search but never placed (now placed); a top-only type is no longer "missing" when at least one copy is placed in modes 1/2.
- Simos's 20x20x9 (overhangs on storeys 1/3/5/7), Type_mix 30/19/41/15/5: NLA/GFA 34.5% -> 45.1%, target met; counts 8/8/8/8/2 exact; 25x15x12 Type_mix 30/30/20/15/5: 56.1%; 25x15x12 counts 10/20/20/10/2: 2-bed 17/20 (was 16; the bay set that fits all, 9.29+6.26+4.75 m, is 1 cm longer than the segment). Tests: 32.

## v51 — overhang tip edges, stretched units, back-span headroom (2026-10-05)
- Tip edge members along every overhanging long facade between the cantilever tips (select_edge_member on the largest tip spacing; concealed strip or downstand), own weight added to the cantilever tip loads (balance exact).
- Programme: an overhang strip narrower than a unit (spare bay) is added to the adjacent unit on the storeys that reach it (unit spans two bay pieces, area + strip x band depth; type name kept). 25x15, +2 m end: 15 units stretched, +198 m2.
- Headroom: lowest clear height under the along-overhang back-span beams reported; issue when below Clear_Height. Frame beams on the old gable line inside stretched units now count for the apartment headroom check.
- Tests: 29.

## v50 — clamped cantilevers only (2026-10-05)
- Simos's render showed storey-high cantilever walls floating on frame lines. Rule now:
  - Across: storey-high cantilever walls only on party/core lines (clamped in the wall) and gable end lines (continued inside over the band depth as a gable wall, piece `_2`); frame lines get cantilever beams only, and the frame beam behind is designed for the hogging moment (w_design = max(w, 8M/b²)); no beam → "OVERHANG NOT CLAMPED" issue; back-span beam fails → "OVERHANG BACK-SPAN FAILS".
  - Along: cantilever beams on the long facade/corridor lines (any length) + back-span beams over the last bay designed for M; only if no beam passes, storey-high facade/corridor walls continued over the last bay (issue note).
- Balance stays exact; 25×15×12 concrete +6 m end now on RC 700×1400 cantilever beams + 500×1000 back-span beams (110 piles). Tests: 28.

## v48 — robust Brep → storey conversion (2026-10-05)
- Storey planes = all Brep bottom/top heights snapped within 2 cm (STOREY_SNAP); each Brep is sliced into every storey it spans (a two-storey overhang box works), then merged per storey into one rectangle.
- Clear errors: gap between storeys, thin storey (< 2.0 m, STOREY_MIN_HEIGHT) from a Brep not ending on a storey height, L/T shapes; each lists the Brep heights received. Report ERROR line now shows "ERROR (v48)". Tests: 28.

## v47 — several Breps per storey (2026-10-05)
- `merge_storey_boxes`: Breps with the same z0/z1 (within tolerance) are joined into one storey when together they fill their bounding rectangle; L/T shapes and Breps spanning other storeys' heights raise clear errors. Report "Storeys" line notes when Breps were merged. Tests: 28.

## v46 — overhangs / cantilevering storeys (wall mode) (2026-10-05)
- Simos's choices: both directions, auto system by length, apartments extend into the overhang.
- `normalize_levels` accepts storeys larger than the one below (must still overlap it); frame mode raises a clear error when an overhang is present.
- Programme: corridor zone anchored to the ground storey on storeys that overhang across the band (per-side band depths `depths`); bays extended into along-overhang zones (`overhang` bays, valid only on the storeys that reach there).
- Wall design: support box SB[k] = intersection of storeys 0..k; existing structure designed on SB; zone Z = storey − SB carried as cantilevers. Loads: Floor_Load on the overhanging floor + Roof_Load where nothing above + facade on the overhang perimeter + self-weight (exposed floors added to the load balance; balance stays exact).
  - Across (beyond long facade): L <= 3.0 m cantilever beams (select_cantilever) on every support line under the floor of the storey; longer: storey-high cantilever walls (`select_cantilever_wall`: lever 0.7 h, chord bars <= 4% of 0.2 h zone / CLT horizontal layers, VRd,max) on party/core and end lines, deep beams on internal frame lines.
  - Along (beyond gable): hung frame beams + lintels on the extended bay lines, carried by cantilever beams (<= 3 m) or storey-high walls in the long facade and corridor lines.
  - Reactions V + M/b at the SB-edge line, −M/b at the next line (columns) or inside the wall; uplift columns reported.
- Outputs: Walls branch {2} cantilever; Beam_Data rows for overhang/hung members; take-off roles "overhang cantilever beam", "overhang hung beam", "cantilever wall".
- 25×15×12 concrete, storeys 4–11 larger: +2 m end → 94 piles (87 without); +6 m end → 110; +2 m side → 92; +4 m side → 103; all PASS, balance exact. Timber +6 m end: CLT root wall fails (error now names the overhang). Tests: 27.

## v45 — Apartment_GEN with Type_mix / Apt_Manual_Count (2026-10-05)
- `Apartment_GEN` (Integer, Item; empty = 0): 0 = previous max-lettable-area mix (unchanged, every type at least one); 1 = `Type_mix` (Float, List; one value per type, fractions or %, normalised to the sum) = target share of UNITS; 2 = `Apt_Manual_Count` (Integer, List; one whole number per type) = exact unit counts.
- Modes 1/2: bay widths from the area knapsack re-weighted (≈30 weightings; best by light evaluation kept); then every slot (bay × side × storey) gets 0–4 units side by side (several only where they fit; NON-load-bearing walls between them) by a local search (single moves + pair moves) minimising deviation (mode 1: Σ|share−target|, mode 2: Σ|count−target|), then maximising area. Top-only types: count from the target (mode 1: share × unit slots, ≥1 if share > 0). Mode 2 may leave slots vacant (reported as "Vacant slot area"). "Every type at least one" not applied.
- Status MIX_DEVIATION (+ GH warning, PROGRAMME MIX DEVIATION issue) when mode 2 is not exact or a mode 1 share is off by more than 5 percentage points. Report line "APARTMENT_GEN …: requested vs designed".
- 25×15×12: Type_mix 30/30/20/15/5 → 19/19/12/10/2 (all within 5 pp); counts 0/40/0/0/1 exact (vacant 1609 m2); 10/20/20/10/2 → 2-bedroom 16 of 20 (bays 10.1 + 10.2 m cannot hold them: MIX_DEVIATION). Tests: 24.

## v44 — INPUTS RECEIVED shows empty inputs as "-" (2026-10-05)
- Simos's v43 run: Max_Unit_Width and Core_Area were connected but empty and printed as blanks; defaults were used correctly. Now shown as "-". Results identical to v41 at default inputs (verified: 87 piles, 68 units). Tests: 20.

## v43 — Corridor_Width, Max_Unit_Width, Common_Ratio, Core_Area inputs (2026-10-05)
- All Float, Item access; empty = default (1.8 m / no limit / 0.05 / 25 m2). `optional_number()` reads them.
- Corridor_Width drives the band depth (bay_geometry), wall-mode corridor lintels and the report.
- Max_Unit_Width: types whose area / band depth (+ wall allowance) exceeds it are left out (stack and top-only types); leftover slot length is never spread beyond the cap (becomes spare instead); status DESIGNED_LIMITED / INCOMPLETE_LIMITED + PROGRAMME LIMITED issue + GH warning. Below the slab span it removes frame lines from apartments (timber 25×15×12: 5.5 m → only studio / 1-bedroom).
- Common_Ratio 0–1 (values > 1 read as %); Core_Area > 0.
- DBG INPUT and the Apartments_N report line list the four values. Tests: 20.

## v42 — robust input names + INPUTS RECEIVED line (2026-10-05)
- Simos: "facade load does not change anything". The calculation responds (25×20 × 18 walls: Facade_Load 1.5 → 5.0 gives facade 6 885 → 22 950 kN, piles 164 → 199, funderingsbalken 234 → 296 m3), so the value most likely never reached the script (input named differently, or the old script still in the component).
- `inp(name)`: inputs are found by exact name, else by a case/space/underscore-insensitive match (e.g. `facade_load`, `Facade Load`), never by an ALL-CAPS internal constant. `number()` and all input reads use it.
- Report line 3: `INPUTS RECEIVED` lists every optional input with the value the script actually got (`-` = not connected, default used).
- Tests: 16.

## v41 — Facade_Load and Core_Count inputs (2026-10-05)
- `Facade_Load` (Float, Item, kN/m2 of elevation, default 1.5): drives every facade line load (floor edges, gables, frame-mode perimeter beams, ground-storey foundation beams, load balance).
- `Core_Count` (Integer, Item; empty or 0 = automatic, one core per 600 m2 of footprint): number of programme cores, used by the wall system and by the frame-mode core stability system; error if the core bays do not fit the building length.
- 25×20 × 18 concrete walls: 1 core U 0.77, core tension 17.7 MN → 2 cores U 0.52, tension 3.2/3.0 MN, piles 164 → 153.
- Tests: 15.

## v40 — quantity take-off outputs (2026-10-05)
- New outputs 32–39 (Item / List): `Foundation_Beams_m1`, `Piles_m1`, `Party_Walls_m2`, `Core_Walls_m2`, `Internal_Walls_m2`, `Columns_m1`, `Beams_m1`, `Takeoff_Data` (List of CSV rows `category,item,dimensions,count,m1,m2,m3`: funderingsbalken per b x h, piles per diameter and length, walls per kind/thickness/material, columns per side, beams per role and section, concealed edge strips separately, internal walls per apartment type).
- `quantity_takeoff()` (section 7d2). Walls m2 = length x storey height (one face, gross). Columns m1 = sum of storey heights. Beams m1 = spans of frame beams, lintels, downstand edge members and gallery cantilevers; concealed slab edge strips listed but not counted.
- Internal (non-load-bearing) walls are not modelled: estimate per apartment = 4.0 m + 4.5 m per bedroom + 0.05 m per m2 unit area (`INTERNAL_WALL_BASE`, `INTERNAL_WALL_PER_BEDROOM`, `INTERNAL_WALL_PER_M2`), times the clear storey height. Bedrooms from the type name (Studio 0, "n bedroom" n, Penthouse 3, otherwise area/25 − 1). About 0.2–0.37 m wall per m2 of unit.
- Report line QUANTITY TAKE-OFF; DBG|TAKEOFF line. Tests: 13.

## v39 — Clear_Height input + Apartments_N output (2026-10-05)
- New input `Clear_Height` (Float, Item access, m; default 2.60). Drives the frame-beam selection (lightest section keeping the clear height, else shallowest), the HEADROOM warning/options and the storey height needed. Must be below the storey height.
- New output `Apartments_N` (31st output): DataTree, one integer per branch = number of apartments of that type, same branch order as Apartment_Boxes ({0} Studio, {1} 1 bedroom, {2} 2 bedroom, {3} 3 bedroom, {4} Penthouse with the default names). A unit spanning several bays counts once.
- Tests: 11.

## v38 — tension piles checked (2026-10-05)
- New optional input `Pile_Tension_Capacity` (Fr;t;d per pile, kN, from the CPTs / NEN 9997-1). Without it the script ASSUMES 0.35 × Fr;d (`PILE_TENSION_SHARE`) and says so in the report.
- Pile tension now governs design like compression: `pile_line` utilisation = max(compression, tension); `pile_group` only accepts layouts whose worst wind-case tension ≤ Fr;t;d; `add_pile` U includes tension; report line (b) and the TENSION PILES issue give U and the source of Fr;t;d. Pile self-weight ignored (conservative).
- Pile rows under a funderingsbalk: up to 4 (`PILE_MAX_ROWS`, was 3) so overturning-heavy lines can resolve.
- Results (stub runs, default capacities): 25×20 × 18 concrete walls PASS 164 × Ø500, 72 tension piles at U 0.95; 20×15 × 12 timber walls failed tension (U 1.15) with 3 rows → PASS with 4 rows (76 × Ø500, U 0.97); 25×30 × 6 steel walls no tension.
- New project skeleton on C: (`src/`, `archive/`, `tests/` with Rhino stubs, `knowledge/`, `CLAUDE.md`). Tests: 9.

## v37 — column gaps + merged pile clusters (2026-10-05)
- Wall-mode frame-line columns run the full storey height (they stopped at the slab underside → visible gaps at every floor).
- Overlapping pile clusters of neighbouring heavy columns merge into one cluster at the load centroid.

## v36 — pile clusters under heavy columns (2026-10-03)
- A column load that one pile position cannot carry gets m positions at 3D centred on the column (rigid local cap).
