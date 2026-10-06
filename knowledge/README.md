# StructuralGEN knowledge base (C: copy)

**Current script version:** v42 (Report header `MATERIALS v42`) · 4344 lines · sha256 `3b12d39053870c26…` · archive: v37–v42.

| File | Content |
|---|---|
| CHANGELOG.md | Version history from v36 (older history on E:). |
| KNOWN_ISSUES.md | Open issues. |
| ROADMAP.md | Next steps and decisions needed. |
| WIND_NB.md | NEN-EN 1991-1-4 + NB:2011 values used. |
| CHANGES_v37.md | Notes delivered with v37. |

Grasshopper inputs (v38): Breps, Floor_Load, Roof_Load, Construction, Slab_Thickness, AreaPerTypeOfApartment, Apartment_Names, Load_Bearing_Walls, Wind_Area, Wind_Terrain, Pile_Diameter, Pile_Capacity, **Pile_Tension_Capacity (new, optional, kN per pile)**, Peil_NAP, Pile_Tip_NAP, **Clear_Height (v39, Float, Item, default 2.60 m)**, **Facade_Load (v41, Float, Item, default 1.5 kN/m2)**, **Core_Count (v41, Integer, Item, empty = auto)**, **Corridor_Width (v43, Float, Item, default 1.8 m)**, **Max_Unit_Width (v43, Float, Item, empty/0 = no limit)**, **Common_Ratio (v43, Float, Item, default 0.05; >1 read as %)**, **Core_Area (v43, Float, Item, default 25 m2)**, **Apartment_GEN (v45, Integer, Item, 0/1/2, empty = 0)**, **Type_mix (v45, Float, List, shares per type)**, **Apt_Manual_Count (v45, Integer, List, units per type)**, Debug. 39 outputs: the 30 earlier ones + **Apartments_N** (31, v39) + take-off (v40): **Foundation_Beams_m1, Piles_m1, Party_Walls_m2, Core_Walls_m2, Internal_Walls_m2, Columns_m1, Beams_m1, Takeoff_Data** (32–39).

v46: storeys may overhang the storey below (wall mode only): cantilever beams up to 3 m, storey-high cantilever walls beyond (v54: beams first at any length, walls only as a flagged last resort); apartments extend into the overhang. Walls tree gains branch {2} cantilever.

v53: new input Apartment_Depth (Float, Item, empty = automatic). Tests: python3 -m unittest test_core in tests/ (stub + fake_rhino end-to-end).
v55: support boxes stack (core at the ground-floor position, corridor on the ground-floor corridor zone); overhang bays hold units up to the facade; deep strips beyond the 10 m daylight depth reported.
v56: overhang beams try a steel insert before any wall (Timber/Concrete); apartment/common boxes run facade to corridor (Apartment_Data box_area_m2); Support_Boxes branch {3} unlettable (deep strips); fake_rhino runs Steel.
v57: Support_Boxes back to three branches {0} core {1} circulation {2} common (v56 branch {3} removed); deep strips reported in text only.
v58: overhang-storey apartments run to the corridor (no gaps), flagged DEEP UNITS when deeper than the daylight depth.
v59: Floor_Load, Roof_Load, Slab_Thickness, Clear_Height, Corridor_Width, Max_Unit_Width, Common_Ratio and Core_Area are settings at the top of src/structuralgen.py (section 0), not inputs. New output Message (compact status). No orange runtime warnings.
