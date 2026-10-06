# StructuralGEN — working rules (C:\PROJECTS\StructuralGEN)

- Grasshopper Script component, Rhino 8, Python 3 only. No inline comments. Deliver the full script (`src/structuralgen.py`). Never stop mid-task.
- Every change: archive the previous version in `archive/`, bump `SCRIPT_VERSION`, run `python3 tests/test_core.py` and `python3 -m pyflakes src/structuralgen.py`, update `knowledge/` (CHANGELOG, KNOWN_ISSUES, ROADMAP, README).
- Scope rules from Simos: load-bearing walls ONLY between apartments and at cores; frame lines elsewhere. Foundation = piles only (funderingsbalken, poeren, kanaalplaat). Windows input removed. Never extend a defined scope — propose instead.
- Tests use `tests/stub.py` (Rhino modules mocked); `run()` itself needs Rhino — Simos sends Reports with `Debug` = True for end-to-end checks.
- The full history (v9–v35 knowledge base, fake_rhino end-to-end tests) is on the other machine at E:\PROJECTS\StructuralGEN; this folder continues from v37.

## Rhino testing
- The Grasshopper Script component loads src/structuralgen.py from disk (loader stub); edit the file, then re-run the component (computer use) and read the Report panel via Copy Data Only.
- Never add, remove or reorder output branches or data columns (Support_Boxes, Apartment_Boxes, *_Data) without asking Simos first: his Grasshopper definition depends on them (v56 branch {3} broke it).
- Git: repo https://github.com/SimosManiatis/massing-to-structure (public). The connected folder cannot delete files, so never run git inside it from the VM (locks/temp objects get stuck). Work in $HOME/sg with --work-tree=<folder>, then cp -r .git into the folder; Simos pushes from Windows. archive/ and _to_delete/ are ignored.
