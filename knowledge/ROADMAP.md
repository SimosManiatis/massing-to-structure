# Roadmap (v38)

## Next
1. **Rhino run of v38** with `Debug` = True; add `Pile_Tension_Capacity` input (kN) to the component (optional).
2. **Merge E: and C:** — copy the E: knowledge base and `tests/fake_rhino.py` here (or point me at it) so both machines have the full history.
3. **Programme efficiency on deep plans** — 25 m deep plans leave a 5 m circulation zone (NLA/GFA ~45–48 %). Decision needed: deeper apartments, or a narrower corridor with storage/balconies.
4. **Penthouse on setback storeys** — allow the top-only type to use a bay cut by the setback.
5. **Pile capacity from CPT** (KI-47): Koppejan-type input, negative skin friction.

## Then
6. Core foundation (lift pit, deeper core poer) — KI-48.
7. Band-beam punching/torsion — KI-45, KI-57.
8. Stability remainder (door openings in cores, annex C cs·cd) — KI-42, KI-69.
9. Fire minimum sizes (steel protection, timber charring).

## Later
Windows back in; mix control; layout realism (end units, galleries, Bbl lift/stair rules); continuous concrete beams; embodied carbon (MKI) and cost.

## Decisions needed from Simos
- Clear height: now an input (`Clear_Height`, default 2.60 m); Bbl value still to confirm.
- Keep "every type at least one"?
- Item 3 (circulation depth).
- Real Fr;t;d per pile (currently assumed 0.35 Fr;d).
- Proposed (awaiting Simos): Mix_Target input (share per type) to stop the max-NLA mix flipping between materials.
- 2026-10-05 input candidates re-listed for Simos (tier 1: Mix_Target, Corridor_Width, Apartment_Depth/Max_Unit_Width, Imposed_Load split, Consequence_Class; tier 2: Concrete_Class, Steel_Grade, Timber_Grade, Common_Ratio, Core_Area, Drift_Limit, Ground_Floor_Type; tier 3: Wind_Correlation, Rebar ratios, Pile catalogue). Awaiting choice.
- v43 done: Corridor_Width, Max_Unit_Width, Common_Ratio, Core_Area. Still open from the input list: Mix_Target, Imposed_Load split, Steel_Grade, Concrete_Class, Timber_Grade, Consequence_Class, Drift_Limit, Ground_Floor_Type, Pile_Type.

## Proposed v52+ (2026-10-05, awaiting Simos)
P1 programme efficiency: U1 exhaustive bay sets (Apartment_GEN 1/2, slot-efficiency tie-break); U2 units on the free side of the core bay; U3 Apartment_Depth input + full-depth units / too-small-type note; U4 overhang strips with facade end need 4.0+t/2.
P2 overhang structure: U5 core long walls as back-span clamp; U6 stretched-unit frame beam headroom also for cross overhangs; U7 overhang deflection L/250 (cantilever 2L/250); U8 back-span frame beam continuity hogging also checked at the column joint (report only).
P3 quality: U9 NLA/GFA + efficiency report line per storey (slot waste); U10 pile choice monotonic (KI-75); U11 Mix_Target for mode 0 dropped (covered by Apartment_GEN); U12 end-to-end run() smoke test with a fake Rhino (port fake_rhino from E:).
Tests per update listed in the session reply.
- Done (v56): unit boxes span facade-to-corridor; unlettable support kind for deep strips.
- Done (v56): steel inserts before walls.
- Proposal: NLA option counting full box area (type areas as minimums) (KI-84).
