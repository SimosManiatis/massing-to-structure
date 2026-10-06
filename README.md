# massing-to-structure (StructuralGEN)

Grasshopper (Rhino 8, Python 3) concept generator: from stacked massing Breps it designs an apartment programme, load-bearing party/core walls and frame lines, overhangs, wind stability and a Dutch pile foundation (screw displacement piles, funderingsbalken, poeren, kanaalplaat over a crawl space). Concept estimate only - not structural design.

## Use in Grasshopper

Put a Python 3 Script component on the canvas with only this loader in it:

```python
import io
_sg_path = r'C:\PROJECTS\StructuralGEN\src\structuralgen.py'
exec(compile(io.open(_sg_path, encoding='utf-8').read(), _sg_path, 'exec'), globals())
```

Project settings (loads, slab thickness, clear height, corridor width, max unit width, common ratio, core area) are at the top of `src/structuralgen.py` (section 0). Inputs, outputs and the full method are described in `knowledge/README.md`; changes per version in `knowledge/CHANGELOG.md`.

## Tests

```
python3 -m unittest tests.test_core
```

`tests/fake_rhino.py` stands in for RhinoCommon/Grasshopper so the whole component runs outside Rhino.

Author: Simos Maniatis
