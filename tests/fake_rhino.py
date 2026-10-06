import io
import math
import os
import sys
import types


class Point3d(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.X, self.Y, self.Z = float(x), float(y), float(z)


Point3d.Origin = Point3d()


class Vector3d(Point3d):
    pass


Vector3d.XAxis, Vector3d.YAxis, Vector3d.ZAxis = Vector3d(1, 0, 0), Vector3d(0, 1, 0), Vector3d(0, 0, 1)


class Interval(object):
    def __init__(self, a, b):
        self.T0, self.T1 = float(a), float(b)


class Plane(object):
    def __init__(self, origin=None, a=None, b=None):
        self.Origin = origin or Point3d()


Plane.WorldXY = Plane()


class Transform(object):
    def __init__(self, fn):
        self.fn = fn

    @staticmethod
    def Rotation(angle, axis, centre):
        c, s = math.cos(angle), math.sin(angle)
        return Transform(lambda p: (p[0]*c-p[1]*s, p[0]*s+p[1]*c, p[2]))

    @staticmethod
    def Scale(centre, factor):
        return Transform(lambda p: (p[0]*factor, p[1]*factor, p[2]*factor))

    @staticmethod
    def PlaneToPlane(a, b):
        o = b.Origin
        return Transform(lambda p: (p[0]+o.X, p[1]+o.Y, p[2]+o.Z))


class BoundingBox(object):
    def __init__(self, lo, hi):
        self.Min, self.Max = Point3d(*lo), Point3d(*hi)


class Edge(object):
    def __init__(self, a, b):
        self.PointAtStart, self.PointAtEnd = Point3d(*a), Point3d(*b)

    def IsLinear(self, tol):
        return True


class Brep(object):
    IsValid = True
    IsSolid = True

    def __init__(self, lo, hi, hollow=0.0):
        self.corners = [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        self.hollow = hollow

    def bounds(self):
        xs, ys, zs = zip(*self.corners)
        return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))

    def GetBoundingBox(self, accurate):
        return BoundingBox(*self.bounds())

    def DuplicateBrep(self):
        lo, hi = self.bounds()
        return Brep(lo, hi, self.hollow)

    def Transform(self, xf):
        self.corners = [xf.fn(p) for p in self.corners]
        return True

    @property
    def Edges(self):
        lo, hi = self.bounds()
        out = []
        for z in (lo[2], hi[2]):
            out += [Edge((lo[0], lo[1], z), (hi[0], lo[1], z)), Edge((hi[0], lo[1], z), (hi[0], hi[1], z)),
                    Edge((hi[0], hi[1], z), (lo[0], hi[1], z)), Edge((lo[0], hi[1], z), (lo[0], lo[1], z))]
        return out

    @staticmethod
    def CreateBooleanDifference(a, b, tol):
        (alo, ahi), (blo, bhi) = a.bounds(), b.bounds()
        overlap = 1.0
        for n in range(3):
            overlap *= max(0.0, min(ahi[n], bhi[n])-max(alo[n], blo[n]))
        return [Brep(alo, ahi, a.hollow+overlap)]


class Extrusion(object):
    pass


class Box(object):
    def __init__(self, plane, ix, iy, iz):
        o = plane.Origin
        self.lo = (ix.T0+o.X, iy.T0+o.Y, iz.T0+o.Z)
        self.hi = (ix.T1+o.X, iy.T1+o.Y, iz.T1+o.Z)

    def ToBrep(self):
        return Brep(self.lo, self.hi)


class Circle(object):
    def __init__(self, plane, radius):
        self.plane, self.radius = plane, radius


class Cylinder(object):
    def __init__(self, circle, height):
        self.circle, self.height = circle, height

    def ToBrep(self, a, b):
        o, r = self.circle.plane.Origin, self.circle.radius
        return Brep((o.X-r, o.Y-r, o.Z), (o.X+r, o.Y+r, o.Z+self.height))


class VolumeMassProperties(object):
    def __init__(self, volume):
        self.Volume = volume

    @staticmethod
    def Compute(b):
        lo, hi = b.bounds()
        return VolumeMassProperties((hi[0]-lo[0])*(hi[1]-lo[1])*(hi[2]-lo[2])-getattr(b, 'hollow', 0.0))

    def Dispose(self):
        pass


class PolylineCurve(object):
    def __init__(self, points):
        self.points = points


class DataTreeType(object):
    def __init__(self):
        self.branches = {}

    def EnsurePath(self, path):
        self.branches.setdefault(path.index, [])

    def Add(self, item, path):
        self.branches.setdefault(path.index, []).append(item)


class DataTreeFactory(object):
    def __getitem__(self, kind):
        return DataTreeType


class GH_Path(object):
    def __init__(self, index):
        self.index = index


class Param(object):
    def __init__(self, name, value):
        self.Name = self.NickName = name
        items = value if isinstance(value, (list, tuple)) else [value]
        self.VolatileData = types.SimpleNamespace(AllData=lambda skip: list(items))


class Component(object):
    def __init__(self, inputs):
        self.messages = []
        self.Params = types.SimpleNamespace(Input=[Param(k, v) for k, v in inputs.items()])

    def AddRuntimeMessage(self, level, text):
        self.messages.append((level, text))


def install():
    rhino = types.ModuleType('Rhino')
    geometry = types.ModuleType('Rhino.Geometry')
    for name, obj in list(globals().items()):
        if isinstance(obj, type) and name in ('Point3d', 'Vector3d', 'Interval', 'Plane', 'Transform', 'Brep', 'Extrusion', 'Box', 'Circle', 'Cylinder', 'VolumeMassProperties', 'PolylineCurve'):
            setattr(geometry, name, obj)
    rhino.Geometry = geometry
    rhino.RhinoDoc = types.SimpleNamespace(ActiveDoc=types.SimpleNamespace(ModelUnitSystem='UnitSystem.Meters', ModelAbsoluteTolerance=0.001))
    rhino.RhinoMath = types.SimpleNamespace(UnitScale=lambda a, b: 1.0)
    rhino.UnitSystem = types.SimpleNamespace(Meters='UnitSystem.Meters')
    grasshopper = types.ModuleType('Grasshopper')
    kernel = types.ModuleType('Grasshopper.Kernel')
    kernel.GH_RuntimeMessageLevel = types.SimpleNamespace(Warning='Warning', Error='Error', Remark='Remark')
    data = types.ModuleType('Grasshopper.Kernel.Data')
    data.GH_Path = GH_Path
    grasshopper.DataTree = DataTreeFactory()
    grasshopper.Kernel = kernel
    kernel.Data = data
    system = types.ModuleType('System')
    system.Object = object
    for name, module in (('Rhino', rhino), ('Rhino.Geometry', geometry), ('Grasshopper', grasshopper), ('Grasshopper.Kernel', kernel),
                         ('Grasshopper.Kernel.Data', data), ('System', system)):
        sys.modules[name] = module


def run_component(boxes, **inputs):
    install()
    breps = [Brep((b[0], b[2], b[4]), (b[1], b[3], b[5])) for b in boxes]
    values = dict(Breps=breps)
    values.update(inputs)
    component = Component(values)
    namespace = dict(values)
    namespace['__name__'] = 'structuralgen'
    namespace['ghenv'] = types.SimpleNamespace(Component=component)
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'structuralgen.py')
    exec(compile(io.open(path, encoding='utf-8').read(), path, 'exec'), namespace)
    namespace['_messages'] = component.messages
    return namespace
