"""
Author: Simos Maniatis
TU DELFT / MSc Building Technology / February 2024
"""

from __future__ import division

import json
import math
from collections import defaultdict
import Rhino
import Rhino.Geometry as rg
import Grasshopper.Kernel as ghk
import System
from Grasshopper import DataTree
from Grasshopper.Kernel.Data import GH_Path

# 0. Project settings - edit these values here (they are no longer component inputs)

FLOOR_LOAD = 10.0
ROOF_LOAD = 8.5
SLAB_THICKNESS = 0.25
CLEAR_HEIGHT = 2.40
CORRIDOR_WIDTH = 1.80
MAX_UNIT_WIDTH_SETTING = None
COMMON_RATIO = 0.05
CORE_AREA = 25.0

# 1. Internal settings

CONCRETE_BEAM_RATIO = 12.0
CONCRETE_COLUMN_STRESS_MPA = 8.0

SPAN_LIMITS = {'Concrete': 8.0, 'Timber': 15.0, 'Steel': 21.0}
SEED_TARGETS = {'Concrete': (8.0, 7.0, 6.0, 5.0, 4.0),
                'Timber': (15.0, 12.0, 9.0, 6.0, 5.0, 4.0, 3.5, 3.0, 2.5),
                'Steel': (21.0, 16.0, 12.0, 9.0, 6.0, 5.0, 4.0, 3.5, 3.0, 2.5)}
SHORT_SPAN_LIMITS = {'Concrete': 8.0, 'Timber': 6.0, 'Steel': 6.0}
FACE_INDEX_BIN = 6.0
MIN_MOVABLE_BAY = 2.50
SUPPORT_KINDS = ('core', 'circulation', 'common')
SEARCH_PASSES = 3
SIZE_ITERATIONS = 6
MAX_GRID_LINES = 20000
MAX_COLUMN_SEGMENTS = 20000
MAX_FACES = 200000
DENSITY = 25.0
MAX_EDGE_SUPPORT_OFFSET = 0.75  # m; geometric search cap, NOT cantilever capacity
ONE_WAY_ASPECT = 2.0
RC_COVER_TO_BAR_CENTRE_MM = 52.5
RC_COLUMN_STEEL_RATIO = 0.01
RC_FLANGED_FACTOR = 0.8
RC_MAX_COLUMN = 1.50


# Concrete edge-strip scenario
EDGE_FCK = 30.0
EDGE_FYK = 500.0
EDGE_GAMMA_C = 1.50
EDGE_GAMMA_S = 1.15
EDGE_ALPHA_CC = 0.85
EDGE_COVER_MM = 35.0
EDGE_AGGREGATE_MM = 20.0
EDGE_ES = 200000.0
EDGE_CREEP = 2.0
SCRIPT_VERSION = '59'
PERMANENT_SHARE = 0.7
ULS_GRAVITY = 1.2*PERMANENT_SHARE+1.5*(1.0-PERMANENT_SHARE)
WIND_ULS_FACTOR = 1.5
EDGE_ULS_FACTOR = ULS_GRAVITY
EDGE_DEFLECTION_RATIO = 250.0
EDGE_DIAMETERS_MM = (8.0, 10.0, 12.0, 16.0)
EDGE_SPACINGS_MM = (250.0, 200.0, 175.0, 150.0, 125.0, 100.0, 75.0)


# 2. Geometry and facade checks

def isfinite(value):
    return not (math.isinf(value) or math.isnan(value))


def aperture_axis(b, tol):
    flat = max(4.0*tol,1e-5)
    dx,dy,dz = b[1]-b[0],b[3]-b[2],b[5]-b[4]
    if dz <= flat:
        return 'horizontal'
    if dx <= flat and dy > flat:
        return 0
    if dy <= flat and dx > flat:
        return 2
    return None


def angle_from_segments(segments, tol):
    """Pure geometric orientation: horizontal vectors in consistent units."""
    horizontal = [(dx,dy) for dx,dy,dz in segments
                  if abs(dz) <= tol and math.hypot(dx,dy) > tol]
    if not horizontal:
        raise ValueError('No straight horizontal Brep edges found for orientation.')
    dx,dy = max(horizontal,key=lambda v: math.hypot(*v))
    degrees = (math.degrees(math.atan2(dy,dx))+45.0)%90.0-45.0
    return 0.0 if abs(degrees) < 1e-10 else degrees


def infer_grid_angle(source, scale, tol):
    valid = []
    for i,item in enumerate(source):
        b = item.ToBrep() if isinstance(item,rg.Extrusion) else item
        if not isinstance(b,rg.Brep) or not b.IsValid or not b.IsSolid:
            raise ValueError('Storey {} is not a valid closed Brep.'.format(i))
        valid.append(b)
    lowest = min(valid,key=lambda b:b.GetBoundingBox(True).Min.Z)
    segments = []
    for edge in lowest.Edges:
        if not edge.IsLinear(tol/scale):
            continue
        a,b = edge.PointAtStart,edge.PointAtEnd
        segments.append(((b.X-a.X)*scale,(b.Y-a.Y)*scale,(b.Z-a.Z)*scale))
    return angle_from_segments(segments,tol)


def facade_groups(levels, records, tol):
    groups = defaultdict(list)
    unmatched = set()
    flat_tol = max(4.0*tol,1e-5)
    for face,b in records:
        axis = aperture_axis(b,tol)
        if axis == 'horizontal':
            continue
        if axis is None:
            unmatched.add(face)
            continue
        assigned = False
        od = 2-axis
        for k,p in enumerate(levels):
            if b[5] <= p[4]+tol or b[4] >= p[5]-tol:
                continue
            if b[od+1] < p[od]-tol or b[od] > p[od+1]+tol:
                continue
            centre = (b[axis]+b[axis+1])/2.0
            side = axis if abs(centre-p[axis]) <= abs(centre-p[axis+1]) else axis+1
            if abs(centre-p[side]) > MAX_EDGE_SUPPORT_OFFSET+flat_tol:
                continue
            groups[k,side].append((face,b))
            assigned = True
        if not assigned:
            unmatched.add(face)
    return groups,sorted(unmatched)


def facade_column_hits(frame, supports, groups, clearance, tol):
    """Project only the perimeter columns onto their associated facade."""
    result = {}
    margin = clearance+tol
    for name,k,b in frame['columns']:
        x,y = (b[0]+b[1])/2.0,(b[2]+b[3])/2.0
        p = supports[k]
        faces = set()
        for side in range(4):
            normal = x if side < 2 else y
            if abs(normal-p[side]) > tol:
                continue
            d = 2 if side < 2 else 0
            for face,w in groups.get((k,side),()):
                if (b[d]-margin <= w[d+1] and b[d+1]+margin >= w[d]
                        and b[4]-margin <= w[5] and b[5]+margin >= w[4]):
                    faces.add(face)
        if faces:
            result[name] = sorted(faces)
    return result


def projected_beam_faces(bounds, storey, support, groups, clearance, tol):
    """Check perimeter beams AND beam ends reaching a support facade.

    Depth behind glazing is not a clearance solution. Distant interior
    members that do not reach a perimeter support line are excluded.
    """
    result = set()
    margin = clearance+tol
    for side in range(4):
        normal = 0 if side < 2 else 2
        tangent = 2-normal
        if not bounds[normal]-tol <= support[side] <= bounds[normal+1]+tol:
            continue
        for face,w in groups.get((storey,side),()):
            if (bounds[tangent]-margin <= w[tangent+1] and bounds[tangent+1]+margin >= w[tangent]
                    and bounds[4]-margin <= w[5] and bounds[5]+margin >= w[4]):
                result.add(face)
    return sorted(result)


def facade_frame_hits(frame, supports, groups, clearance, tol):
    result = facade_column_hits(frame,supports,groups,clearance,tol)
    for name,k,b in frame['beams']:
        hits = projected_beam_faces(b,k,supports[k],groups,clearance,tol)
        if hits:
            result[name] = hits
    return result


def material_beam_bounds(a, b, xs, ys, beam_top, section, direction):
    ax,ay,bx,by = xs[a[0]],ys[a[1]],xs[b[0]],ys[b[1]]
    width,depth = section['b'],section['h']
    return ((ax,bx,ay-width/2,ay+width/2,beam_top-depth,beam_top) if direction=='X'
            else (ax-width/2,ax+width/2,ay,by,beam_top-depth,beam_top))


# 3. Input helpers and grid layout

INPUT_NAMES = ('Breps', 'Construction', 'AreaPerTypeOfApartment', 'Apartment_Names',
               'Load_Bearing_Walls', 'Wind_Area', 'Wind_Terrain', 'Pile_Diameter', 'Pile_Capacity', 'Pile_Tension_Capacity',
               'Peil_NAP', 'Pile_Tip_NAP', 'Facade_Load', 'Core_Count',
               'Apartment_GEN', 'Type_mix', 'Apt_Manual_Count', 'Apartment_Depth', 'Debug')
SETTING_INPUTS = ('Floor_Load', 'Roof_Load', 'Slab_Thickness', 'Clear_Height', 'Corridor_Width', 'Max_Unit_Width', 'Common_Ratio', 'Core_Area')
RUN_INFO = {}


def input_key(name):
    return ''.join(ch for ch in name.lower() if ch.isalnum())


def inp(name):
    space = globals()
    if space.get(name) is not None:
        return space[name]
    wanted = input_key(name)
    for key, value in list(space.items()):
        if value is not None and key != name and not key.isupper() and input_key(key) == wanted and not callable(value):
            return value
    return None


def inputs_line():
    parts = []
    for name in INPUT_NAMES[1:]:
        value = inp(name)
        if value is None or (isinstance(value, str) and not value.strip()):
            parts.append('{} -'.format(name))
        elif isinstance(value, (list, tuple)) or (hasattr(value, '__iter__') and not isinstance(value, str)):
            parts.append('{} [{} items]'.format(name, len(list(value))))
        else:
            parts.append('{} {}'.format(name, value))
    ignored = [name for name in SETTING_INPUTS if inp(name) is not None and str(inp(name)).strip() != '']
    return ('INPUTS RECEIVED (- = not connected, default used): '+' | '.join(parts)+'.'+
            ' SETTINGS (top of the script): Floor_Load {} | Roof_Load {} | Slab_Thickness {} | Clear_Height {} | Corridor_Width {} | Max_Unit_Width {} | Common_Ratio {} | Core_Area {}.'.format(
                FLOOR_LOAD, ROOF_LOAD, SLAB_THICKNESS, CLEAR_HEIGHT, CORRIDOR_WIDTH, MAX_UNIT_WIDTH_SETTING if MAX_UNIT_WIDTH_SETTING else '-', COMMON_RATIO, CORE_AREA)+
            (' IGNORED component inputs (now set at the top of the script): '+', '.join(ignored)+'.' if ignored else ''))


def optional_number(name, default):
    v = inp(name)
    if v is None or str(v).strip() == '':
        return default
    v = float(v)
    if not isfinite(v):
        raise ValueError(name+' must be finite.')
    return v


def number(name, default, positive=True):
    v = inp(name)
    v = default if v is None else float(v)
    if not isfinite(v) or (positive and v <= 0):
        raise ValueError(name + ' must be finite' + (' and > 0.' if positive else '.'))
    return v


def up(v):
    return math.ceil(v / 0.05 - 1e-10) * 0.05


def unique(values, tol):
    out = []
    for v in sorted(values):
        if not out or v - out[-1] > tol:
            out.append(v)
    return out


def overlap(a, b):
    return all(a[d] <= b[d+1] and a[d+1] >= b[d] for d in (0, 2, 4))


def expand(b, r):
    return tuple(v + (-r if i % 2 == 0 else r) for i, v in enumerate(b))


def normalize_levels(levels, tol):
    out = [list(p) for p in sorted(levels, key=lambda p: p[4])]
    for k, p in enumerate(out):
        if min(p[1]-p[0], p[3]-p[2], p[5]-p[4]) <= tol:
            raise ValueError('Degenerate storey.')
        if k:
            q = out[k-1]
            if abs(p[4]-q[5]) > tol:
                raise ValueError('Storeys must be contiguous: storey {} ends at z {:.3f} m but the next starts at {:.3f} m.'.format(k-1, q[5], p[4]))
            if p[1] <= q[0]+tol or p[0] >= q[1]-tol or p[3] <= q[2]+tol or p[2] >= q[3]-tol:
                raise ValueError('Storey {} does not overlap the storey below; a floating block cannot be carried.'.format(k))
            p[4] = q[5]
            for j in range(4):
                if abs(p[j]-q[j]) <= tol:
                    p[j] = q[j]
    # Canonicalize tolerance-sized boundary discrepancies across all levels.
    for indices in ((0, 1), (2, 3)):
        boundaries = unique([p[j] for p in out for j in indices], tol)
        for p in out:
            for j in indices:
                p[j] = min(boundaries, key=lambda v: abs(v-p[j]))
    return out


def merge_storey_boxes(boxes, tol):
    snap = max(tol, STOREY_SNAP)
    zs = unique([v for b in boxes for v in (b[4], b[5])], snap)
    ranges = ', '.join('{:.3f}-{:.3f}'.format(b[4], b[5]) for b in sorted(boxes, key=lambda b: (b[4], b[5])))
    merged = []
    for za, zb in zip(zs, zs[1:]):
        g = [b for b in boxes if b[4] <= za+snap and b[5] >= zb-snap]
        if not g:
            raise ValueError('No Brep between z {:.3f} and {:.3f} m: the storeys must stack without gaps. {} Brep(s) received, heights (m): {}. If the model has a box there, it is not reaching the Breps input (not referenced in the parameter, filtered or null upstream).'.format(za, zb, len(boxes), ranges))
        if zb-za < STOREY_MIN_HEIGHT:
            raise ValueError('The Brep heights create a {:.3f} m thin storey between z {:.3f} and {:.3f} m: every Brep must start and end on storey heights (an overhang box exactly as high as its storey). Brep heights received (m): {}.'.format(zb-za, za, zb, ranges))
        x0, x1 = min(b[0] for b in g), max(b[1] for b in g)
        y0, y1 = min(b[2] for b in g), max(b[3] for b in g)
        xs = unique([v for b in g for v in (b[0], b[1])], tol)
        ys = unique([v for b in g for v in (b[2], b[3])], tol)
        missing = 0.0
        for xa, xb in zip(xs, xs[1:]):
            for ya, yb in zip(ys, ys[1:]):
                cx, cy = (xa+xb)/2.0, (ya+yb)/2.0
                if not any(b[0]-tol <= cx <= b[1]+tol and b[2]-tol <= cy <= b[3]+tol for b in g):
                    missing += (xb-xa)*(yb-ya)
        if missing > max(tol*(x1-x0+y1-y0), 1e-6):
            raise ValueError('The {} Breps between z {:.2f} and {:.2f} m do not form one rectangle in plan ({:.1f} m2 of their bounding rectangle is empty): L/T-shaped storeys are not supported; add the missing corner box or model the storey as one box.'.format(len(g), za, zb, missing))
        merged.append((x0, x1, y0, y1, za, zb))
    return merged


def brep_wire_count():
    try:
        for param in ghenv.Component.Params.Input:
            if input_key(param.NickName) == input_key('Breps') or input_key(param.Name) == input_key('Breps'):
                return sum(1 for item in param.VolatileData.AllData(False))
    except Exception:
        return None
    return None


def has_overhang(levels, tol):
    return any(p[0] < q[0]-tol or p[1] > q[1]+tol or p[2] < q[2]-tol or p[3] > q[3]+tol for q, p in zip(levels, levels[1:]))


def seed_axis(boundaries, tol, target, max_span):
    boundaries = unique(boundaries, tol)
    coords, fixed = [boundaries[0]], {0}
    for a, b in zip(boundaries, boundaries[1:]):
        minimum_count = max(1,int(math.ceil((b-a)/max_span-1e-10)))
        maximum_count = max(1,int(math.floor((b-a)/MIN_MOVABLE_BAY+1e-10)))
        n = max(minimum_count,min(maximum_count,int(math.ceil((b-a)/target-1e-10))))
        if len(coords)+n > MAX_GRID_LINES:
            raise ValueError('Grid exceeds size limit.')
        coords.extend(a+(b-a)*i/n for i in range(1, n+1))
        fixed.add(len(coords)-1)
    return coords, fixed


def validate_axis(coords, fixed, tol, max_span):
    for i, (a, b) in enumerate(zip(coords, coords[1:])):
        if b-a > max_span+tol or b-a <= tol:
            raise ValueError('Internal maximum-span/order invariant failed.')
        if b-a < MIN_MOVABLE_BAY-tol and not (i in fixed and i+1 in fixed):
            raise ValueError('Internal minimum movable-bay invariant failed.')


def prune_axis(coords, fixed, tol, max_span):
    """Remove only redundant movable lines; remap mandatory indices."""
    removed = 0
    while True:
        options = [i for i in range(1, len(coords)-1)
                   if i not in fixed and coords[i+1]-coords[i-1] <= max_span+tol]
        if not options:
            break
        i = min(options, key=lambda j: min(coords[j]-coords[j-1], coords[j+1]-coords[j]))
        del coords[i]
        revised = {j if j < i else j-1 for j in fixed}
        fixed.clear()
        fixed.update(revised)
        removed += 1
    validate_axis(coords, fixed, tol, max_span)
    return removed


def axis_limits(construction, targets):
    short = SHORT_SPAN_LIMITS[construction]
    full = SPAN_LIMITS[construction]
    return tuple(full if t > short+1e-9 else short for t in targets)


def one_way_pair(construction, targets):
    short = SHORT_SPAN_LIMITS[construction]
    return sum(t > short+1e-9 for t in targets) <= 1


def check_one_way(xs, ys, construction, tol):
    short = SHORT_SPAN_LIMITS[construction]
    long_x = max(b-a for a,b in zip(xs,xs[1:])) > short+tol
    long_y = max(b-a for a,b in zip(ys,ys[1:])) > short+tol
    if long_x and long_y:
        raise ValueError('Two-way long span: bays exceed {:.1f} m in both X and Y.'.format(short))


class FaceIndex:
    def __init__(self, records):
        self.records = records
        self.bins = defaultdict(list)
        count = 0
        for index, (face, b) in enumerate(records):
            i0, i1, j0, j1 = self.extents(b)
            count += (i1-i0+1)*(j1-j0+1)
            if count > 1000000:
                raise ValueError('Aperture index too large; check geometry units.')
            for i in range(i0, i1+1):
                for j in range(j0, j1+1):
                    self.bins[i, j].append(index)

    def extents(self, b):
        return tuple(int(math.floor(b[i]/FACE_INDEX_BIN)) for i in (0, 1, 2, 3))

    def hits(self, b):
        i0, i1, j0, j1 = self.extents(b)
        if (i1-i0+1)*(j1-j0+1) > 100000:
            candidates = range(len(self.records))
        else:
            candidates = set()
            for i in range(i0, i1+1):
                for j in range(j0, j1+1):
                    candidates.update(self.bins.get((i, j), ()))
        return [self.records[i] for i in candidates if overlap(b, self.records[i][1])]


def move_axes(levels, xs, ys, fixed_x, fixed_y, index, reserve, clearance, tol, limits, facades=None):
    """Score actual column locations, never projected global forbidden strips."""
    axes = (xs, ys)
    margin = reserve/2 + clearance + tol
    epsilon = max(tol*2, 1e-5)
    for sweep in range(SEARCH_PASSES):
        changed = False
        for axis, fixed in ((0, fixed_x), (1, fixed_y)):
            coords, other = axes[axis], axes[1-axis]
            d, od = axis*2, (1-axis)*2
            for i in range(1, len(coords)-1):
                if i in fixed:
                    continue
                previous, current, following = coords[i-1:i+2]
                minimum = MIN_MOVABLE_BAY
                lo = max(previous+minimum, following-limits[axis])
                hi = min(following-minimum, previous+limits[axis])
                if current < lo-tol or current > hi+tol:
                    raise ValueError('Movable line violates minimum/maximum spacing constraints.')
                if hi-lo <= tol:
                    continue
                ideal = (previous+following)/2
                # Only storeys and perpendicular coordinates on this line.
                contexts = []
                candidates = [lo, hi, current, min(hi, max(lo, ideal))]
                for k,p in enumerate(levels):
                    if not p[d]-tol <= current <= p[d+1]+tol:
                        continue
                    # Moving X changes positions along the Y facades; moving
                    # Y changes positions along the X facades. Preserve the
                    # projection constraint even for an inboard perimeter.
                    for side in ((2,3) if axis == 0 else (0,1)):
                        records = (facades or {}).get((k,side),())
                        if records:
                            contexts.append(records)
                            for face,box in records:
                                candidates.extend((box[d]-margin-epsilon,box[d+1]+margin+epsilon))
                    for v in other:
                        if not p[od]-tol <= v <= p[od+1]+tol:
                            continue
                        b = [0.0]*6
                        b[d], b[d+1] = lo-margin, hi+margin
                        b[od], b[od+1] = v-margin, v+margin
                        b[4], b[5] = p[4]-clearance-tol, p[5]+clearance+tol
                        records = index.hits(b)
                        if records:
                            contexts.append(records)
                            for face, box in records:
                                candidates.extend((box[d]-margin-epsilon, box[d+1]+margin+epsilon))
                candidates.extend(lo+(hi-lo)*j/12 for j in range(1, 12))
                candidates = unique([v for v in candidates if lo <= v <= hi], tol/10)

                def score(v):
                    hit_columns, penetration = 0, 0.0
                    for records in contexts:
                        lengths = [max(0.0, min(v+margin, b[d+1])-max(v-margin, b[d]))
                                   for face, b in records
                                   if v-margin <= b[d+1] and v+margin >= b[d]]
                        if lengths:
                            hit_columns += 1
                            penetration += max(lengths)
                    return (hit_columns, round(penetration, 9), abs(v-ideal))

                best = min(candidates, key=score)
                if score(best) < score(current) and abs(best-current) > tol:
                    coords[i] = best
                    changed = True
        if not changed:
            break
    for coords, fixed, limit in ((xs, fixed_x, limits[0]), (ys, fixed_y, limits[1])):
        validate_axis(coords, fixed, tol, limit)


def models_for(levels, xs, ys, tol, support_levels=None):
    if len(xs)*len(ys)*len(levels) > MAX_COLUMN_SEGMENTS:
        raise ValueError('Grid exceeds {:,} potential column segments.'.format(MAX_COLUMN_SEGMENTS))
    models = []
    support_levels = levels if support_levels is None else support_levels
    for k, p in enumerate(levels):
        support = support_levels[k]
        ix = [i for i, x in enumerate(xs) if support[0]-tol <= x <= support[1]+tol]
        iy = [j for j, y in enumerate(ys) if support[2]-tol <= y <= support[3]+tol]
        if len(ix) < 2 or len(iy) < 2:
            raise ValueError('Each storey requires at least two support lines in each direction.')
        edges = [((a,j),(b,j),xs[b]-xs[a],'X') for a,b in zip(ix,ix[1:]) for j in iy]
        edges += [((i,a),(i,b),ys[b]-ys[a],'Y') for i in ix for a,b in zip(iy,iy[1:])]
        area, covered, exposed = {}, {}, {}
        upper = levels[k+1] if k+1 < len(levels) else None
        for u, i in enumerate(ix):
            left = p[0] if u == 0 else (xs[ix[u-1]]+xs[i])/2
            right = p[1] if u == len(ix)-1 else (xs[i]+xs[ix[u+1]])/2
            for v, j in enumerate(iy):
                front = p[2] if v == 0 else (ys[iy[v-1]]+ys[j])/2
                back = p[3] if v == len(iy)-1 else (ys[j]+ys[iy[v+1]])/2
                a = (right-left)*(back-front)
                c = 0 if upper is None else (max(0,min(right,upper[1])-max(left,upper[0]))*
                                              max(0,min(back,upper[3])-max(front,upper[2])))
                area[i,j], covered[i,j], exposed[i,j] = a, c, max(0,a-c)
        models.append(dict(nodes=list(area), edges=edges, area=area, covered=covered, exposed=exposed))
    for k in range(1,len(models)):
        if not set(models[k]['nodes']).issubset(models[k-1]['nodes']):
            raise ValueError('Unsupported column at setback.')
    return models


# Internal material scenarios. Strength/stiffness units below: kN, m, kN/m2.
# No product catalogue is implied by these generated trial dimensions.
MATERIAL_CATALOG_CACHE = {}
MATERIAL_BEAM_CACHE = {}
MATERIAL_CLEAR_CACHE = {}
MATERIAL_ULS = ULS_GRAVITY
MATERIAL_DEFLECTION_RATIO = 300.0


# 4. Steel and timber member sizing

def construction_name(value):
    if value is None:
        return 'Concrete'
    value = str(value).strip().strip('"').strip("'").strip().lower()
    for name in ('Concrete','Steel','Timber'):
        if value == name.lower():
            return name
    raise ValueError('Construction must be Concrete, Steel or Timber (String hint, ITEM access).')


def material_properties(construction):
    if construction == 'Steel':
        return dict(rho=7850.0,weight=7850.0*9.81/1000,E=210e6,G=210e6/2.6,
                    E05=210e6,fc=235000.0,fm=235000.0,fv=235000.0/math.sqrt(3),
                    creep=0.0,grade='S235 assumed; plates 4-40 mm')
    if construction == 'Timber':
        # EN 14080 GL24h; permanent load, service class 2 scenario.
        # Entire service load sustained: kmod=.60, kdef=.80, gammaM=1.25.
        return dict(rho=420.0,weight=420.0*9.81/1000,E=11.5e6,G=650000.0,
                    E05=9.6e6,fc=24000.0*.60/1.25,fm=24000.0*.60/1.25,
                    fv=3500.0*.60/1.25,creep=.80,grade='GL24h assumed; protected service class 2')
    raise ValueError('Material member checks are available for Steel and Timber.')


def section_properties(construction, width, depth, wall=0.0):
    if min(width,depth) <= 0 or wall < 0 or 2*wall >= min(width,depth):
        raise ValueError('Invalid section dimensions.')
    if construction == 'Steel':
        if wall <= 0 or wall > .040+1e-10:
            raise ValueError('Steel trial plates must be >0 and <=40 mm.')
        fy = 235000.0 if wall <= .016+1e-10 else 225000.0
        # All four plates screened as uniform compression, conservative for
        # the bending webs: internal plate c/t <=42*sqrt(235/fy).
        if max(width-2*wall,depth-2*wall)/wall > 42*math.sqrt(235000.0/fy)+1e-9:
            return None
        extra = dict(fy=fy)
        area = width*depth-(width-2*wall)*(depth-2*wall)
        iy = (width*depth**3-(width-2*wall)*(depth-2*wall)**3)/12
        iz = (depth*width**3-(depth-2*wall)*(width-2*wall)**3)/12
        av = 2*wall*(depth-2*wall)  # two clear webs; conservatively omit flanges
        label = 'BOX {:.0f}x{:.0f}x{:.0f}'.format(depth*1000,width*1000,wall*1000)
    else:
        area,iy,iz = width*depth,width*depth**3/12,depth*width**3/12
        av = .67*area/1.5  # crack-reduced effective shear area, rectangular tau_max
        label = 'GL24h {:.0f}x{:.0f}'.format(width*1000,depth*1000)
        extra = {}
    return dict(b=width,h=depth,t=wall,A=area,Iy=iy,Iz=iz,W=2*iy/depth,Av=av,section=label,**extra)


def material_catalog(construction, kind):
    key = construction,kind
    if key not in MATERIAL_CATALOG_CACHE:
        sections = []
        if construction == 'Steel':
            for h in list(range(100,651,25))+list(range(700,1501,50)):
                if kind == 'Column' and h > 1000:
                    continue
                widths = [h] if kind == 'Column' else range(100,min(h,600)+1,25 if h <= 650 else 50)
                for b in widths:
                    if kind == 'Beam' and h > 3*b:
                        continue
                    for t in (4,5,6,8,10,12,16,20,25,30,35,40):
                        s = section_properties(construction,b/1000,h/1000,t/1000)
                        if s:
                            sections.append(s)
        else:
            if kind == 'Column':
                sections = [section_properties(construction,b/1000,b/1000) for b in range(140,1001,20)]
            else:
                for b in range(140,601,20):
                    for h in range(180,1441,45):
                        if b <= 2*h and h <= 6*b:
                            sections.append(section_properties(construction,b/1000,h/1000))
        MATERIAL_CATALOG_CACHE[key] = sorted(sections,key=lambda s:(s['A'],s['h'],s['b'],s['t']))
    return MATERIAL_CATALOG_CACHE[key]


def compression_check(construction, section, height, service_load):
    p = material_properties(construction)
    strength = section.get('fy',p['fc'])
    ncr = math.pi**2*p['E05']*min(section['Iy'],section['Iz'])/height**2
    if construction == 'Steel':
        slenderness = math.sqrt(section['A']*strength/ncr)
        phi = .5*(1+.49*(slenderness-.2)+slenderness**2)
        reduction = min(1.0,1/(phi+math.sqrt(max(0.0,phi**2-slenderness**2))))
    else:
        slenderness = math.sqrt(section['A']*24000.0/ncr)
        phi = .5*(1+.10*(slenderness-.3)+slenderness**2)
        reduction = 1.0 if slenderness <= .3 else min(1.0,1/(phi+math.sqrt(max(0.0,phi**2-slenderness**2))))
    resistance = reduction*section['A']*strength
    return dict(N_service=service_load,NEd=MATERIAL_ULS*service_load,NbRd=resistance,
                Ncr=ncr,slenderness=slenderness,reduction=reduction,
                U_N=MATERIAL_ULS*service_load/resistance,
                EI=p['E']*min(section['Iy'],section['Iz'])/(1+p['creep']))


def beam_check(construction, section, span, area_line_load):
    p = material_properties(construction)
    q = area_line_load+p['weight']*section['A']
    med,ved = MATERIAL_ULS*q*span**2/8,MATERIAL_ULS*q*span/2
    fm = section.get('fy',p['fm'])
    fv = fm/math.sqrt(3) if 'fy' in section else p['fv']
    mrd,vrd = fm*section['W'],fv*section['Av']
    if construction == 'Steel':
        # Conservative whole-section reduction when VEd>0.5 VRd.
        rho = min(1.0,max(0.0,2*ved/vrd-1)**2)
        mrd *= 1-rho
        shear_stiffness = p['G']*section['Av']
    else:
        shear_stiffness = p['G']*(5.0/6.0)*section['A']
    ei = p['E']*section['Iy']/(1+p['creep'])
    ga = shear_stiffness/(1+p['creep'])
    displacement = 5*q*span**4/(384*ei)+q*span**2/(8*ga)
    limit = span/MATERIAL_DEFLECTION_RATIO
    return dict(qSLS=q,MEd=med,MRd=mrd,VEd=ved,VRd=vrd,EI=ei,
                deflection_mm=1000*displacement,limit_mm=1000*limit,
                U_M=med/mrd if mrd>0 else float('inf'),U_V=ved/vrd,U_deflection=displacement/limit)


def blocked(section, constraints):
    return any(section['b'] >= bmin-1e-12 and section['h'] >= hmin-1e-12 for bmin,hmin in constraints)


def beam_constraints(a, b, xs, ys, beam_top, direction, nearby, storey, support, groups, available_depth, clearance, tol):
    r = clearance+tol
    if direction == 'X':
        lo,hi,centre,along,lateral = xs[a[0]],xs[b[0]],ys[a[1]],0,2
    else:
        lo,hi,centre,along,lateral = ys[a[1]],ys[b[1]],xs[a[0]],2,0
    found = set()
    def add(bmin, w):
        hmin = beam_top-r-w[5]
        if beam_top+r >= w[4] and hmin < available_depth:
            found.add((round(bmin,9),round(hmin,9)))
    for face,w in nearby:
        if lo-r <= w[along+1] and hi+r >= w[along]:
            add(2*max(centre-r-w[lateral+1],w[lateral]-centre-r),w)
    for side in range(4):
        normal = 0 if side < 2 else 2
        position = support[side]
        for face,w in groups.get((storey,side),()):
            if normal == along:
                if lo-tol <= position <= hi+tol:
                    add(2*max(centre-r-w[lateral+1],w[lateral]-centre-r),w)
            elif lo-r <= w[along+1] and hi+r >= w[along]:
                add(2*max(centre-tol-position,position-centre-tol),w)
    return tuple(sorted(found))


def select_material_beam(construction, span, q, available_depth, constraints=()):
    key = construction,span,q,available_depth
    if key not in MATERIAL_BEAM_CACHE:
        for section in material_catalog(construction,'Beam'):
            if section['h'] >= available_depth:
                continue
            check = beam_check(construction,section,span,q)
            if max(check['U_M'],check['U_V'],check['U_deflection']) <= 1.0:
                MATERIAL_BEAM_CACHE[key] = (section,check)
                break
        else:
            raise ValueError('{} beam: no trial section passes at span {:.3f} m, area-load envelope {:.3f} kN/m and available depth {:.3f} m.'.format(construction,span,q,available_depth))
    fallback = MATERIAL_BEAM_CACHE[key]
    if not constraints or not blocked(fallback[0],constraints):
        return fallback
    clear_key = key+(constraints,)
    if clear_key not in MATERIAL_CLEAR_CACHE:
        MATERIAL_CLEAR_CACHE[clear_key] = fallback
        catalog = material_catalog(construction,'Beam')
        smallest = dict(b=min(c['b'] for c in catalog),h=min(c['h'] for c in catalog))
        for section in ([] if blocked(smallest,constraints) else catalog):
            if section['h'] >= available_depth or section['A'] < fallback[0]['A']-1e-12 or blocked(section,constraints):
                continue
            check = beam_check(construction,section,span,q)
            if max(check['U_M'],check['U_V'],check['U_deflection']) <= 1.0:
                MATERIAL_CLEAR_CACHE[clear_key] = (section,check)
                break
    return MATERIAL_CLEAR_CACHE[clear_key]


def material_check_row(name, construction, section, check):
    fields = [name,construction,section['section']]+[section[k] for k in ('b','h','t','A','Iy','Iz')]
    fields += [check.get(k,'') for k in ('EI','N_service','NEd','NbRd','Ncr','slenderness','reduction',
               'qSLS','MEd','MRd','VEd','VRd','deflection_mm','limit_mm','U_N','U_M','U_V','U_deflection',
               'As_req','As_prov','U_As','span_depth','span_depth_limit','lambda_lim','MEd_column','MRd_column')]
    fields.append('PASS_ASSUMED_MEMBER_MODEL')
    return ','.join('{:.9g}'.format(v) if isinstance(v,(float,int)) else v for v in fields)


FRAME_TIE_MIN = 75.0
SECONDARY_DEPTH_RATIO = 25.0
SECONDARY_MIN_DEPTH = 0.25
GL24H_FT0K = 19200.0


def frame_beam_extras(a, b, xs, ys, p, direction, qarea, height, span, upper=None):
    if direction == 'X':
        lines, value, lo, hi = ys, ys[a[1]], p[2], p[3]
    else:
        lines, value, lo, hi = xs, xs[a[0]], p[0], p[1]
    perimeter = abs(value-lo) < 1e-6 or abs(value-hi) < 1e-6
    inside = sorted(v for v in lines if lo-1e-6 <= v <= hi+1e-6)
    j = min(range(len(inside)), key=lambda i: abs(inside[i]-value))
    spacing = ((value-inside[j-1]) if j > 0 else 0.0)/2.0+((inside[j+1]-value) if j+1 < len(inside) else 0.0)/2.0
    facade = 0.0
    if upper is not None:
        if direction == 'X':
            on = abs(value-upper[2]) < 1e-6 or abs(value-upper[3]) < 1e-6
            r0, r1, u0, u1 = min(xs[a[0]], xs[b[0]]), max(xs[a[0]], xs[b[0]]), upper[0], upper[1]
        else:
            on = abs(value-upper[0]) < 1e-6 or abs(value-upper[1]) < 1e-6
            r0, r1, u0, u1 = min(ys[a[1]], ys[b[1]]), max(ys[a[1]], ys[b[1]]), upper[2], upper[3]
        if on and u0-1e-6 <= r0 and r1 <= u1+1e-6:
            facade = FACADE_LOAD*height
    tie = max(FRAME_TIE_MIN, (0.4 if perimeter else 0.8)*qarea*spacing*span)
    return facade, tie, perimeter


def tie_capacity(construction, section):
    if construction == 'Steel':
        return section['A']*section.get('fy', material_properties('Steel')['fm'])
    return section['A']*GL24H_FT0K


def select_frame_beam_member(construction, span, line_load, available, constraints, tie):
    hmin = min(max(SECONDARY_MIN_DEPTH, span/SECONDARY_DEPTH_RATIO), available-1e-6)
    section, check = select_material_beam(construction, span, line_load, available, constraints)
    if section['h'] >= hmin-1e-9 and tie_capacity(construction, section) >= tie:
        return section, check, 'load'
    for trial in material_catalog(construction, 'Beam'):
        if trial['h'] >= available or trial['h'] < hmin-1e-9 or blocked(trial, constraints):
            continue
        if tie_capacity(construction, trial) < tie:
            continue
        trial_check = beam_check(construction, trial, span, line_load)
        if max(trial_check['U_M'], trial_check['U_V'], trial_check['U_deflection']) <= 1.0:
            return trial, trial_check, 'minimum depth' if section['h'] < hmin-1e-9 else 'tie'
    return section, check, 'load'


def size_material_frame(levels, xs, ys, models, floor, roof, slab, tol, construction, aperture_context=None):
    pmat = material_properties(construction)
    loads = {n:0.0 for n in models[0]['nodes']}
    sides = {n:0.0 for n in loads}
    above_area = {n:0.0 for n in loads}
    columns,beams,cd,bd,notes,rows,checks = [],[],[],[],[],[],[]
    profiles = {}
    vc = vb = beam_weight = max_utilization = facade_total = 0.0
    governed, tie_max, tie_u = defaultdict(int), 0.0, 0.0
    widened = [0]
    for k in reversed(range(len(levels))):
        p,m = levels[k],models[k]
        height = p[5]-p[4]
        bearing = {}
        qarea = roof if k == len(levels)-1 else max(floor,roof)
        line_loads = slab_line_loads(m,xs,ys,p,qarea)
        for n in m['nodes']:
            loads[n] += m['covered'][n]*floor+m['exposed'][n]*roof
        for a,b,span,direction in m['edges']:
            facade,tie,perimeter = frame_beam_extras(a,b,xs,ys,p,direction,qarea,(levels[k+1][5]-levels[k+1][4]) if k+1 < len(levels) else 0.0,span,levels[k+1] if k+1 < len(levels) else None)
            line_load = line_loads[a,b]+facade
            facade_total += facade*span
            loads[a] += facade*span/2
            loads[b] += facade*span/2
            beam_top = p[5]-slab
            constraints = ()
            if aperture_context is not None:
                aperture_index,facades,supports,clearance = aperture_context
                query_section = dict(b=.65,h=height-slab)
                query = material_beam_bounds(a,b,xs,ys,beam_top,query_section,direction)
                nearby = aperture_index.hits(expand(query,clearance+tol))
                constraints = beam_constraints(a,b,xs,ys,beam_top,direction,nearby,k,supports[k],facades,
                                               height-slab-tol,clearance,tol)
            section,check,reason = select_frame_beam_member(construction,span,line_load,height-slab-tol,constraints,tie)
            governed[reason] += 1
            tie_max = max(tie_max,tie)
            tie_u = max(tie_u,tie/tie_capacity(construction,section))
            width,depth = section['b'],section['h']
            bounds = material_beam_bounds(a,b,xs,ys,beam_top,section,direction)
            name = 'B_{}_{}_{}_{}_{}'.format(k,a[0],a[1],b[0],b[1])
            beams.append((name,k,bounds))
            profiles[name] = dict(section,direction=direction,length=span)
            checks.append(material_check_row(name,construction,section,check))
            max_utilization = max(max_utilization,check['U_M'],check['U_V'],check['U_deflection'])
            vb += section['A']*span
            weight = section['A']*span*pmat['weight']
            beam_weight += weight
            loads[a] += weight/2
            loads[b] += weight/2
            bd.append('{},{},{},{},{},{:.3f},{:.3f},{:.3f}'.format(k,a[0],a[1],b[0],b[1],span,width,depth))
            bearing[a] = max(bearing.get(a,0.0),width)
            bearing[b] = max(bearing.get(b,0.0),width)
        for n in m['nodes']:
            if bearing.get(n,0.0) > sides[n]+1e-9:
                widened[0] += 1
            for section in material_catalog(construction,'Column'):
                if section['b'] < max(sides[n],bearing.get(n,0.0))-1e-10 or section['A'] < above_area[n]-1e-10:
                    continue
                own_weight = section['A']*height*pmat['weight']
                check = compression_check(construction,section,height,loads[n]+own_weight)
                if check['U_N'] <= 1.0:
                    break
            else:
                raise ValueError('{} column: no trial section passes at storey {}, node {}, height {:.3f} m and service load {:.3f} kN.'.format(construction,k,n,height,loads[n]))
            side = section['b']
            sides[n],above_area[n] = side,section['A']
            x,y = xs[n[0]],ys[n[1]]
            name = 'C_{}_{}_{}'.format(k,n[0],n[1])
            columns.append((name,k,(x-side/2,x+side/2,y-side/2,y+side/2,p[4],p[5])))
            profiles[name] = dict(section,direction='Z',length=height)
            checks.append(material_check_row(name,construction,section,check))
            max_utilization = max(max_utilization,check['U_N'])
            vc += section['A']*height
            loads[n] += own_weight
            cd.append('{},{},{},{:.3f},{:.3f},{:.3f},{:.3f}'.format(k,n[0],n[1],height,side,m['area'][n],loads[n]))
        if any((sides[a]+sides[b])/2 > s+tol for a,b,s,d in m['edges']):
            notes.append('COLUMN OVERLAP at storey {}; required setback lines need review.'.format(k))
        rows.append('Storey {}: area {:.3f} m2 | covered {:.3f} | exposed {:.3f} | columns {} | beams {}'.format(
            k,sum(m['area'].values()),sum(m['covered'].values()),sum(m['exposed'].values()),len(m['nodes']),len(m['edges'])))
    fa,ra = sum(sum(m['covered'].values()) for m in models),sum(sum(m['exposed'].values()) for m in models)
    residual = sum(loads.values())-(fa*floor+ra*roof+beam_weight+pmat['weight']*vc+facade_total)
    if abs(residual)>max(1e-6,sum(loads.values())*1e-9):
        raise ValueError('Internal material load balance failed.')
    rule = ('FRAME BEAMS (all directions): perimeter beams carry the facade {:.1f} kN/m2 x storey height ({:.0f} kN in total); every beam is a robustness tie (EN 1991-1-7 A.5.1: 0.8 q s L internal, 0.4 q s L perimeter, >= {:.0f} kN; max {:.0f} kN, tension U {:.2f} with A x {}); minimum depth max({:.2f} m, span/{:.0f}) for practical connections. '
            'Governed by: load {} | minimum depth {} | tie {}. Beams parallel to a one-way slab span carry no slab load and are sized by these rules. Columns are never narrower than the widest beam they carry ({} column-storeys widened).').format(
            FACADE_LOAD,facade_total,FRAME_TIE_MIN,tie_max,tie_u,'fy' if construction == 'Steel' else 'ft,0,k {:.1f} MPa (accidental, gammaM 1.0)'.format(GL24H_FT0K/1000),
            SECONDARY_MIN_DEPTH,SECONDARY_DEPTH_RATIO,governed['load'],governed['minimum depth'],governed['tie'],widened[0])
    return dict(columns=columns,beams=beams,cd=cd,bd=bd,loads=loads,sides=sides,vc=vc,vb=vb,
                fa=fa,ra=ra,residual=residual,notes=notes,rows=[rule]+list(reversed(rows)),
                profiles=profiles,checks=checks,max_utilization=max_utilization)


def steel_member_brep(bounds, section, tol):
    """Extruded rectangular ring: one closed hollow Brep per steel member."""
    b,h,t = section['b'],section['h'],section['t']
    outer = rg.PolylineCurve([rg.Point3d(-b/2,-h/2,0),rg.Point3d(b/2,-h/2,0),
                             rg.Point3d(b/2,h/2,0),rg.Point3d(-b/2,h/2,0),rg.Point3d(-b/2,-h/2,0)])
    # Inner loop clockwise when viewed from +Z.
    inner = rg.PolylineCurve([rg.Point3d(-b/2+t,-h/2+t,0),rg.Point3d(-b/2+t,h/2-t,0),
                             rg.Point3d(b/2-t,h/2-t,0),rg.Point3d(b/2-t,-h/2+t,0),rg.Point3d(-b/2+t,-h/2+t,0)])
    # Set the path and profile coordinate system explicitly; split polygon
    # kinks while converting to Brep. Do not rely on Create/ToBrep defaults.
    extrusion = rg.Extrusion()
    shape = None
    try:
        good = extrusion.SetPathAndUp(rg.Point3d.Origin,rg.Point3d(0,0,section['length']),rg.Vector3d.YAxis)
        good = good and extrusion.SetOuterProfile(outer,True) and extrusion.AddInnerProfile(inner)
        if good:
            shape = extrusion.ToBrep(True)
    except Exception:
        shape = None
    if shape is None or not shape.IsSolid or not shape.IsValid:
        # Exact box-minus-through-void fallback in local SI coordinates.
        # Cap planes of the cutter extend beyond both member ends to avoid
        # coincident faces. Tolerance stays small compared with wall thickness.
        geom_tol = min(max(tol,1e-8),t/100.0)
        extension = max(.01,10*geom_tol)
        outer_solid = rg.Box(rg.Plane.WorldXY,rg.Interval(-b/2,b/2),rg.Interval(-h/2,h/2),
                             rg.Interval(0,section['length'])).ToBrep()
        void = rg.Box(rg.Plane.WorldXY,rg.Interval(-b/2+t,b/2-t),rg.Interval(-h/2+t,h/2-t),
                      rg.Interval(-extension,section['length']+extension)).ToBrep()
        pieces = rg.Brep.CreateBooleanDifference(outer_solid,void,geom_tol)
        shape = pieces[0] if pieces is not None and len(pieces)==1 else None
        if shape is None or not shape.IsSolid or not shape.IsValid:
            raise ValueError('Hollow steel construction failed for {} at length {:.6f} m; both explicit extrusion and through-void Boolean failed.'.format(section['section'],section['length']))
    x0,x1,y0,y1,z0,z1 = bounds
    direction = section['direction']
    if direction == 'X':
        plane = rg.Plane(rg.Point3d(x0,(y0+y1)/2,(z0+z1)/2),rg.Vector3d.YAxis,rg.Vector3d.ZAxis)
    elif direction == 'Y':
        plane = rg.Plane(rg.Point3d((x0+x1)/2,y0,(z0+z1)/2),rg.Vector3d(-1,0,0),rg.Vector3d.ZAxis)
    else:
        plane = rg.Plane(rg.Point3d((x0+x1)/2,(y0+y1)/2,z0),rg.Vector3d.XAxis,rg.Vector3d.YAxis)
    if not shape.Transform(rg.Transform.PlaneToPlane(rg.Plane.WorldXY,plane)) or not shape.IsSolid or not shape.IsValid:
        raise ValueError('Hollow steel placement failed for {} on {} axis.'.format(section['section'],direction))
    # Catch profile orientation/capping failures rather than returning solid boxes.
    measured = rg.VolumeMassProperties.Compute(shape)
    if measured is None:
        raise ValueError('Cannot verify steel member volume.')
    volume = abs(measured.Volume)
    measured.Dispose()
    expected = section['A']*section['length']
    if abs(volume-expected)>max(1e-9,expected*1e-6):
        raise ValueError('Hollow steel geometry volume differs from calculated material volume.')
    return shape


# 5. Slab loads, concrete member design and layout search

def slab_line_loads(model, xs, ys, p, q):
    ix = sorted(set(n[0] for n in model['nodes']))
    iy = sorted(set(n[1] for n in model['nodes']))
    loads = dict(((a,b),0.0) for a,b,span,direction in model['edges'])
    for u in range(len(ix)-1):
        for v in range(len(iy)-1):
            i0,i1,j0,j1 = ix[u],ix[u+1],iy[v],iy[v+1]
            lx,ly = xs[i1]-xs[i0],ys[j1]-ys[j0]
            short,long_side = min(lx,ly),max(lx,ly)
            if long_side >= ONE_WAY_ASPECT*short:
                w_long,w_short = q*short/2.0,0.0
            else:
                w_long,w_short = q*short/2.0*(1.0-(short/long_side)**2/3.0),q*short/3.0
            wx,wy = (w_long,w_short) if lx >= ly else (w_short,w_long)
            loads[(i0,j0),(i1,j0)] += wx
            loads[(i0,j1),(i1,j1)] += wx
            loads[(i0,j0),(i0,j1)] += wy
            loads[(i1,j0),(i1,j1)] += wy
    strips = (ys[iy[0]]-p[2],p[3]-ys[iy[-1]],xs[ix[0]]-p[0],p[1]-xs[ix[-1]])
    for u in range(len(ix)-1):
        loads[(ix[u],iy[0]),(ix[u+1],iy[0])] += q*max(0.0,strips[0])
        loads[(ix[u],iy[-1]),(ix[u+1],iy[-1])] += q*max(0.0,strips[1])
    for v in range(len(iy)-1):
        loads[(ix[0],iy[v]),(ix[0],iy[v+1])] += q*max(0.0,strips[2])
        loads[(ix[-1],iy[v]),(ix[-1],iy[v+1])] += q*max(0.0,strips[3])
    return loads


def rc_section(width, depth):
    return dict(b=width,h=depth,t=0.0,A=width*depth,Iy=width*depth**3/12,Iz=depth*width**3/12,
                section='RC {:.0f}x{:.0f}'.format(width*1000,depth*1000))


def rc_strengths():
    fcd = EDGE_ALPHA_CC*EDGE_FCK/EDGE_GAMMA_C
    fyd = EDGE_FYK/EDGE_GAMMA_S
    fctm = 0.30*EDGE_FCK**(2.0/3.0)
    return fcd,fyd,fctm


def rc_beam_check(width, depth, slab, span, line_load):
    fcd,fyd,fctm = rc_strengths()
    q = line_load+DENSITY*width*max(0.0,depth-slab)
    med = EDGE_ULS_FACTOR*q*span**2/8.0
    ved = EDGE_ULS_FACTOR*q*span/2.0
    b = width*1000.0
    d = depth*1000.0-RC_COVER_TO_BAR_CENTRE_MM
    if d <= 0:
        raise ValueError('Concrete beam depth smaller than cover.')
    x_lim = 0.45*d
    mrd = 0.8*x_lim*b*fcd*(d-0.4*x_lim)/1e6
    disc = d*d-2.0*med*1e6/(b*fcd)
    as_req = b*(d-math.sqrt(disc))*fcd/fyd if disc >= 0 else float('inf')
    as_min = max(0.26*fctm/EDGE_FYK,0.0013)*b*d
    as_prov = max(as_req,as_min)
    as_max = 0.04*b*depth*1000.0
    vrd = b*0.9*d*0.6*(1.0-EDGE_FCK/250.0)*fcd/2.0/1000.0
    rho0 = math.sqrt(EDGE_FCK)*1e-3
    rho = as_prov/(b*d)
    if rho <= rho0:
        base = 11.0+1.5*math.sqrt(EDGE_FCK)*rho0/rho+3.2*math.sqrt(EDGE_FCK)*(rho0/rho-1.0)**1.5
    else:
        base = 11.0+1.5*math.sqrt(EDGE_FCK)*rho0/rho
    limit = base*RC_FLANGED_FACTOR*min(1.0,7.0/span)
    ratio = span*1000.0/d
    return dict(qSLS=q,MEd=med,MRd=mrd,VEd=ved,VRd=vrd,As_req=as_req,As_prov=as_prov,
                span_depth=ratio,span_depth_limit=limit,
                U_M=med/mrd,U_V=ved/vrd,U_deflection=ratio/limit,U_As=as_prov/as_max)


def select_rc_beam(span, line_load, slab, height, tol, ratio):
    depth = up(max(0.30,span/ratio,slab+0.10))
    while depth < height-tol:
        width = up(max(0.25,depth/2))
        check = rc_beam_check(width,depth,slab,span,line_load)
        if max(check['U_M'],check['U_V'],check['U_deflection'],check['U_As']) <= 1.0:
            return width,depth,check
        depth = round(depth+0.05,10)
    raise ValueError('Concrete beam: no depth below storey height {:.3f} m passes at span {:.3f} m and slab line load {:.3f} kN/m.'.format(height,span,line_load))


def rc_column_check(side, height, service_load):
    fcd,fyd,fctm = rc_strengths()
    h = side*1000.0
    ac = h*h
    steel = RC_COLUMN_STEEL_RATIO*ac
    ned = EDGE_ULS_FACTOR*service_load
    nrd = (fcd*(ac-steel)+fyd*steel)/1000.0
    d2 = RC_COVER_TO_BAR_CENTRE_MM
    d = h-d2
    l0 = height*1000.0
    slenderness = l0/(h/math.sqrt(12.0))
    n = ned*1000.0/(ac*fcd)
    lambda_lim = 20.0*0.7*1.1*0.7/math.sqrt(n) if n > 0 else float('inf')
    e1 = max(h/30.0,20.0,l0/400.0)
    e2 = 0.0
    if slenderness > lambda_lim:
        eyd = fyd/EDGE_ES
        omega = steel*fyd/(ac*fcd)
        kr = max(0.0,min(1.0,(1.0+omega-n)/(1.0+omega-0.4)))
        kphi = max(1.0,1.0+(0.35+EDGE_FCK/200.0-slenderness/150.0)*EDGE_CREEP)
        e2 = kr*kphi*eyd/(0.45*d)*l0**2/10.0
    med = ned*(e1+e2)/1000.0
    m0 = steel/2.0*fyd*(d-d2)/1e6
    x_bal = 0.0035/(0.0035+fyd/EDGE_ES)*d
    n_bal = 0.8*x_bal*h*fcd/1000.0
    m_bal = n_bal*(h/2.0-0.4*x_bal)/1000.0+m0
    if ned >= n_bal:
        capacity = m_bal*max(0.0,nrd-ned)/(nrd-n_bal)
    else:
        capacity = m0+(m_bal-m0)*ned/n_bal
    utilization = max(ned/nrd,med/capacity if capacity > 0 else float('inf'))
    return dict(N_service=service_load,NEd=ned,NbRd=nrd,slenderness=slenderness,lambda_lim=lambda_lim,
                MEd_column=med,MRd_column=capacity,U_N=utilization)


def size_frame(levels, xs, ys, models, floor, roof, ratio, slab, stress, tol, construction='Concrete', aperture_context=None):
    if construction != 'Concrete':
        return size_material_frame(levels,xs,ys,models,floor,roof,slab,tol,construction,aperture_context)
    loads = dict((n,0.0) for n in models[0]['nodes'])
    sides = dict((n,0.30) for n in loads)
    columns,beams,cd,bd,notes,rows,checks = [],[],[],[],[],[],[]
    profiles = {}
    vc = vb = vb_gross = beam_weight = max_utilization = facade_total = 0.0
    tie_max, tie_steel = 0.0, 0.0
    widened = [0]
    for k in reversed(range(len(levels))):
        p,m = levels[k],models[k]
        height = p[5]-p[4]
        bearing = {}
        qarea = roof if k == len(levels)-1 else max(floor,roof)
        line_loads = slab_line_loads(m,xs,ys,p,qarea)
        for n in m['nodes']:
            loads[n] += m['covered'][n]*floor+m['exposed'][n]*roof
        for a,b,span,direction in m['edges']:
            facade,tie,perimeter = frame_beam_extras(a,b,xs,ys,p,direction,qarea,(levels[k+1][5]-levels[k+1][4]) if k+1 < len(levels) else 0.0,span,levels[k+1] if k+1 < len(levels) else None)
            facade_total += facade*span
            loads[a] += facade*span/2
            loads[b] += facade*span/2
            tie_max = max(tie_max,tie)
            tie_steel = max(tie_steel,tie*1000.0/EDGE_FYK)
            width,depth,check = select_rc_beam(span,line_loads[a,b]+facade,slab,height,tol,ratio)
            section = rc_section(width,depth)
            ax,ay,bx,by = xs[a[0]],ys[a[1]],xs[b[0]],ys[b[1]]
            bounds = ((ax,bx,ay-width/2,ay+width/2,p[5]-depth,p[5]) if direction == 'X'
                      else (ax-width/2,ax+width/2,ay,by,p[5]-depth,p[5]))
            name = 'B_{}_{}_{}_{}_{}'.format(k,a[0],a[1],b[0],b[1])
            beams.append((name,k,bounds))
            profiles[name] = dict(section,direction=direction,length=span)
            checks.append(material_check_row(name,'Concrete',section,check))
            max_utilization = max(max_utilization,check['U_M'],check['U_V'],check['U_deflection'],check['U_As'])
            downstand = width*max(0.0,depth-slab)*span
            vb += downstand
            vb_gross += width*depth*span
            weight = downstand*DENSITY
            beam_weight += weight
            loads[a] += weight/2
            loads[b] += weight/2
            bd.append('{},{},{},{},{},{:.3f},{:.3f},{:.3f}'.format(k,a[0],a[1],b[0],b[1],span,width,depth))
            if width <= depth+1e-9:
                bearing[a] = max(bearing.get(a,0.0),width)
                bearing[b] = max(bearing.get(b,0.0),width)
        for n in m['nodes']:
            denominator = stress-DENSITY*height
            if denominator <= 0:
                raise ValueError('Internal concrete sizing stress is too low for storey height.')
            if bearing.get(n,0.0) > max(0.30,sides[n],math.sqrt(loads[n]/denominator))+1e-9:
                widened[0] += 1
            side = up(max(0.30,sides[n],bearing.get(n,0.0),math.sqrt(loads[n]/denominator)))
            while True:
                own_weight = side*side*height*DENSITY
                check = rc_column_check(side,height,loads[n]+own_weight)
                if check['U_N'] <= 1.0:
                    break
                side = round(side+0.05,10)
                if side > RC_MAX_COLUMN+1e-9:
                    raise ValueError('Concrete column: no side up to {:.2f} m passes at storey {}, node {}, service load {:.3f} kN.'.format(RC_MAX_COLUMN,k,n,loads[n]))
            sides[n] = side
            section = rc_section(side,side)
            x,y = xs[n[0]],ys[n[1]]
            name = 'C_{}_{}_{}'.format(k,n[0],n[1])
            columns.append((name,k,(x-side/2,x+side/2,y-side/2,y+side/2,p[4],p[5])))
            profiles[name] = dict(section,direction='Z',length=height)
            checks.append(material_check_row(name,'Concrete',section,check))
            max_utilization = max(max_utilization,check['U_N'])
            vc += side*side*height
            loads[n] += own_weight
            cd.append('{},{},{},{:.3f},{:.3f},{:.3f},{:.3f}'.format(k,n[0],n[1],height,side,m['area'][n],loads[n]))
        if any((sides[a]+sides[b])/2 > s+tol for a,b,s,d in m['edges']):
            notes.append('COLUMN OVERLAP at storey {}; required setback lines need review.'.format(k))
        rows.append('Storey {}: area {:.3f} m2 | covered {:.3f} | exposed {:.3f} | columns {} | beams {}'.format(
            k,sum(m['area'].values()),sum(m['covered'].values()),sum(m['exposed'].values()),len(m['nodes']),len(m['edges'])))
    fa = sum(sum(m['covered'].values()) for m in models)
    ra = sum(sum(m['exposed'].values()) for m in models)
    residual = sum(loads.values())-(fa*floor+ra*roof+beam_weight+DENSITY*vc+facade_total)
    if abs(residual) > max(1e-6,sum(loads.values())*1e-9):
        raise ValueError('Internal load balance failed.')
    rule = ('FRAME BEAMS (all directions): perimeter beams carry the facade {:.1f} kN/m2 x storey height ({:.0f} kN in total); every beam is a robustness tie (EN 1991-1-7 A.5.1, >= {:.0f} kN; max {:.0f} kN -> continuous tie bars {:.0f} mm2 at fyk in addition to the bending steel); concrete beams keep the span/12, >= 0.30 m minimum depth, so beams parallel to a one-way slab span are not undersized; columns are never narrower than the widest beam they carry ({} column-storeys widened).').format(
            FACADE_LOAD,facade_total,FRAME_TIE_MIN,tie_max,tie_steel,widened[0])
    return dict(columns=columns,beams=beams,cd=cd,bd=bd,loads=loads,sides=sides,vc=vc,vb=vb,vb_gross=vb_gross,
                fa=fa,ra=ra,residual=residual,notes=notes,rows=[rule]+list(reversed(rows)),
                profiles=profiles,checks=checks,max_utilization=max_utilization)


def support_offset(levels, records, reserve, ratio, slab, clearance, tol, construction, max_span):
    """Uniform inboard support-axis distance computed from the current inputs.

    Vertical aligned aperture faces are assigned to the nearest parallel
    facade of the overlapping storey. Deep/internal and unaligned apertures
    are left to final clash checking, never assumed to be cleared.
    """
    if construction == 'Concrete':
        depth = up(max(0.30, max_span/ratio, slab+0.10))
        half_member = max(reserve, up(max(0.25, depth/2)))/2
    else:
        half_member = reserve/2
    inward_depth = 0.0
    flat = max(4.0*tol, 1e-5)
    for face, b in records:
        axis = aperture_axis(b, tol)
        if axis == 'horizontal' or axis is None:
            continue
        for p in levels:
            # Use strict height overlap to avoid assigning the next-storey
            # facade to the terrace directly below it.
            if b[5] <= p[4]+tol or b[4] >= p[5]-tol:
                continue
            od = 2-axis
            if b[od+1] < p[od]-tol or b[od] > p[od+1]+tol:
                continue
            centre = (b[axis]+b[axis+1])/2
            near_low = abs(centre-p[axis]) <= abs(centre-p[axis+1])
            intrusion = b[axis+1]-p[axis] if near_low else p[axis+1]-b[axis]
            if -flat <= intrusion <= MAX_EDGE_SUPPORT_OFFSET+flat:
                inward_depth = max(inward_depth, intrusion)
    offset = inward_depth+half_member+clearance+max(4*tol, 1e-4)
    if offset > MAX_EDGE_SUPPORT_OFFSET:
        raise ValueError('Required support-axis offset {:.3f} m exceeds internal geometric cap {:.3f} m.'.format(offset, MAX_EDGE_SUPPORT_OFFSET))
    return offset


def frame_clash_count(frame, index, clearance, tol):
    return sum(bool(index.hits(expand(b,clearance+tol)))
               for name,k,b in frame['columns']+frame['beams'])



def long_axis(levels):
    base = levels[0]
    return 0 if base[1]-base[0] >= base[3]-base[2] else 2


def aligned_axis(levels, axis, apartments, limit, tol):
    mandatory = unique([p[j] for p in levels for j in (axis, axis+1)], tol)
    walls = defaultdict(set)
    spans = []
    for name, k, b in apartments:
        lo, hi = b[axis], b[axis+1]
        walls[round(lo, 3)].add(k)
        walls[round(hi, 3)].add(k)
        spans.append((lo, hi))
    def gain(v):
        inside = sum(1 for lo, hi in spans if lo+0.05 < v < hi-0.05)
        return (-inside, -1, len(walls.get(round(v, 3), ())))
    candidates = unique(list(walls)+mandatory, tol)
    coords = [mandatory[0]]
    for a, b in zip(mandatory, mandatory[1:]):
        inner = [v for v in candidates if a+tol < v < b-tol]
        points = [a]+inner+[b]
        best = {0: ((0, 0, 0), None)}
        for i in range(1, len(points)):
            options = []
            for j in range(i):
                gap = points[i]-points[j]
                if j not in best or gap > limit+tol:
                    continue
                if gap < MIN_MOVABLE_BAY-tol and not (j == 0 and i == len(points)-1):
                    continue
                value = best[j][0]
                step = (0, 0, 0) if i == len(points)-1 else gain(points[i])
                options.append((tuple(x+y for x, y in zip(value, step)), j))
            if options:
                best[i] = max(options)
        last = len(points)-1
        if last not in best:
            return None
        chain, i = [], last
        while i:
            chain.append(points[i])
            i = best[i][1]
        coords.extend(reversed(chain))
    return coords, set(range(len(coords)))


def column_intrusions(columns, apartments):
    by_storey = defaultdict(list)
    for name, k, b in apartments:
        by_storey[k].append(b)
    count = 0
    for name, k, b in columns:
        x, y = (b[0]+b[1])/2.0, (b[2]+b[3])/2.0
        if any(a[0]+1e-3 < x < a[1]-1e-3 and a[2]+1e-3 < y < a[3]-1e-3 for a in by_storey[k]):
            count += 1
    return count


def solve_layout(levels, index, floor, roof, ratio, slab, stress, clearance, tol, inboard=False, targets=(6.0,6.0), facades=None, construction="Concrete", aligned=None):
    reserve, converged = 0.30, False
    max_span = SPAN_LIMITS[construction]
    limits = axis_limits(construction, targets)
    removed_lines = 0
    previous_offset = None
    xs = ys = fx = fy = None
    for iteration in range(SIZE_ITERATIONS):
        offset = support_offset(levels,index.records,reserve,ratio,slab,clearance,tol,construction,max_span) if inboard else 0.0
        supports = [[p[0]+offset,p[1]-offset,p[2]+offset,p[3]-offset,p[4],p[5]] for p in levels]
        if any(min(p[1]-p[0],p[3]-p[2]) <= tol for p in supports):
            raise ValueError('Inboard support offset consumes a storey footprint.')
        # Uniform offset preserves nested support rectangles. Rebuild the
        # shared grid whenever member resizing changes that offset.
        if previous_offset is None or abs(offset-previous_offset) > tol:
            xs,fx = seed_axis([p[j] for p in supports for j in (0,1)],tol,targets[0],limits[0])
            ys,fy = seed_axis([p[j] for p in supports for j in (2,3)],tol,targets[1],limits[1])
            if aligned is not None:
                if aligned[0] == 0:
                    xs,fx = list(aligned[1]),set(range(len(aligned[1])))
                else:
                    ys,fy = list(aligned[1]),set(range(len(aligned[1])))
        previous_offset = offset
        if len(xs)*len(ys)*len(levels) > MAX_COLUMN_SEGMENTS:
            raise ValueError('Grid exceeds size limit.')
        move_axes(supports,xs,ys,fx,fy,index,reserve,clearance,tol,limits,facades)
        # Short steel/timber bays may be necessary for resistance or window
        # headroom. Never merge them using the concrete 6 m criterion alone.
        if construction == 'Concrete':
            removed_lines += prune_axis(xs,fx,tol,limits[0])+prune_axis(ys,fy,tol,limits[1])
        else:
            validate_axis(xs,fx,tol,limits[0])
            validate_axis(ys,fy,tol,limits[1])
        check_one_way(xs,ys,construction,tol)
        models = models_for(levels,xs,ys,tol,supports)
        frame = size_frame(levels,xs,ys,models,floor,roof,ratio,slab,stress,tol,construction,
                           (index,facades or {},supports,clearance))
        required = max(frame['sides'].values())
        if inboard:
            required = max(required,max(s['b'] for s in frame['profiles'].values()))
        if required <= reserve+tol:
            converged = True
            break
        if iteration < SIZE_ITERATIONS-1:
            reserve = max(reserve,required)
    return dict(xs=xs,ys=ys,fx=fx,fy=fy,models=models,frame=frame,
                iteration=iteration,reserve=reserve,converged=converged,
                removed_lines=removed_lines,offset=offset,supports=supports,
                mode='INBOARD_FRAME' if inboard else ('BOUNDARY_FRAME_APARTMENT_ALIGNED' if aligned is not None else 'BOUNDARY_FRAME'),
                conflicts=frame_clash_count(frame,index,clearance,tol),targets=targets,limits=limits,
                facade_hits=facade_frame_hits(frame,supports,facades or {},clearance,tol))


def choose_layout(levels, index, floor, roof, ratio, slab, stress, clearance, tol, construction="Concrete", apartments=None):
    args = (levels,index,floor,roof,ratio,slab,stress,clearance,tol)
    groups,unmatched = facade_groups(levels,index.records,tol)
    apartments = apartments or []
    selected = None
    attempts = []
    targets = SEED_TARGETS[construction]
    pairs = sorted(((a,b) for a in targets for b in targets),
                   key=lambda p:targets.index(p[0])+targets.index(p[1]))
    pairs = [p for p in pairs if one_way_pair(construction,p)]
    jobs = []
    if apartments:
        axis = long_axis(levels)
        short = SHORT_SPAN_LIMITS[construction]
        aligned = aligned_axis(levels,axis,apartments,short,tol)
        if aligned is not None:
            for t in targets:
                pair = (short,t) if axis == 0 else (t,short)
                if one_way_pair(construction,pair):
                    jobs.append((pair,False,(axis,aligned[0])))
    jobs += [(pair,inboard,None) for pair in pairs for inboard in (False,True)]
    seen = set()
    def rank(c):
        projected = len(c['facade_hits'])
        return (c['conflicts']+projected,c['conflicts'],projected,
                len(c['frame']['notes']),not c['converged'],c['intrusions'],
                len(c['frame']['columns']),c['offset'])
    for pair,inboard,aligned in jobs:
        if not inboard and aligned is None:
            signature = tuple(tuple(seed_axis([p[j] for p in levels for j in axis],tol,t,limit)[0])
                              for axis,t,limit in zip(((0,1),(2,3)),pair,axis_limits(construction,pair)))
            if signature in seen:
                continue
            seen.add(signature)
        label = 'Aligned' if aligned is not None else ('Inboard' if inboard else 'Boundary')
        try:
            candidate = solve_layout(*args,inboard=inboard,targets=pair,facades=groups,construction=construction,aligned=aligned)
        except ValueError as error:
            attempts.append('{} target {} unavailable: {}'.format(label,pair,error))
            continue
        candidate['intrusions'] = column_intrusions(candidate['frame']['columns'],apartments)
        attempts.append('{} target {}: physical conflicts {}; facade-member conflicts {}; columns inside apartments {}.'.format(
            candidate['mode'],pair,candidate['conflicts'],len(candidate['facade_hits']),candidate['intrusions']))
        if selected is None or rank(candidate) < rank(selected):
            selected = candidate
        if (aligned is None and candidate['conflicts']==0 and not candidate['facade_hits'] and candidate['converged']
                and not candidate['frame']['notes'] and candidate['intrusions']==0):
            selected['unmatched_facades'] = unmatched
            return selected,attempts
    if selected is None:
        raise ValueError('No valid candidate grid. '+' | '.join(attempts[:3]))
    selected['unmatched_facades'] = unmatched
    return selected,attempts


# 6. Concrete edge checks

def shear_resistance(fck, gamma_c, width_mm, d_mm, area_mm2):
    k = min(2.0,1.0+math.sqrt(200.0/d_mm))
    rho = min(0.02,area_mm2/(width_mm*d_mm))
    v = max((0.18/gamma_c)*k*(100.0*rho*fck)**(1.0/3.0),
            0.035*k**1.5*math.sqrt(fck))
    return v*width_mm*d_mm/1000.0  # kN for the strip


def cracked_stiffness(width_mm, height_mm, d_mm, area_mm2, e_concrete, e_steel):
    alpha = e_steel/e_concrete
    a = alpha*area_mm2
    # Stable positive root of b*x^2/2 + alpha*As*x - alpha*As*d = 0.
    x = (2.0*a*d_mm)/(a+math.sqrt(a*a+2.0*width_mm*a*d_mm))
    inertia = width_mm*x**3/3.0 + alpha*area_mm2*(d_mm-x)**2
    # Do not claim stiffness greater than the gross concrete section.
    inertia = min(inertia,width_mm*height_mm**3/12.0)
    return x,e_concrete*inertia  # mm, N mm2


def design_edge_strip(length_m, slab_m, service_load, back_bay_m):
    """Automatic bar selection for a 1 m slab strip, with explicit scope.

    All forces below are magnitudes. Top steel resists the hogging moment.
    The function is pure Python and makes no changes to the frame geometry.
    """
    if not all(isfinite(v) for v in (length_m,slab_m,service_load,back_bay_m)):
        raise ValueError('Nonfinite edge-strip data.')
    if min(length_m,slab_m,back_bay_m) <= 0 or service_load < 0:
        raise ValueError('Invalid edge-strip dimensions or load.')
    width,h,L = 1000.0,slab_m*1000.0,length_m*1000.0
    fcd = EDGE_ALPHA_CC*EDGE_FCK/EDGE_GAMMA_C
    fyd = EDGE_FYK/EDGE_GAMMA_S
    fctm = 0.30*EDGE_FCK**(2.0/3.0)
    ecm = 22000.0*((EDGE_FCK+8.0)/10.0)**0.3
    e_eff = ecm/(1.0+EDGE_CREEP)
    qd = EDGE_ULS_FACTOR*service_load
    med = qd*length_m**2/2.0  # kNm per 1 m strip
    ved = qd*length_m          # kN per 1 m strip
    limit = L/EDGE_DEFLECTION_RATIO  # mm
    candidates = []
    for diameter in EDGE_DIAMETERS_MM:
        # Lower of two crossing top bar layers, used for both directions.
        d = h-EDGE_COVER_MM-max(EDGE_DIAMETERS_MM)-diameter/2.0
        if d <= 0 or h-2*EDGE_COVER_MM-max(EDGE_DIAMETERS_MM)-diameter < 0:
            continue
        disc = d*d-2.0*med*1e6/(width*fcd)
        req = (2.0*med*1e6/(fyd*(d+math.sqrt(disc)))) if disc >= 0 else float('inf')
        amin = max(0.26*fctm/EDGE_FYK,0.0013)*width*d
        for spacing in EDGE_SPACINGS_MM:
            if spacing > min(2.0*h,250.0):
                continue
            if spacing-diameter < max(diameter,20.0,EDGE_AGGREGATE_MM+5.0):
                continue
            area = math.pi*diameter**2/4.0*1000.0/spacing
            if area > 0.04*width*h:
                continue
            x_uls = area*fyd/(0.8*width*fcd)
            z = d-0.4*x_uls
            if z <= 0:
                continue
            mrd = area*fyd*z/1e6
            vrd = shear_resistance(EDGE_FCK,EDGE_GAMMA_C,width,d,area)
            x_cr,ei = cracked_stiffness(width,h,d,area,e_eff,EDGE_ES)
            # 1 kN/m == 1 N/mm for a 1 m strip.
            delta = service_load*L**4/(8.0*ei)
            sigma_sls = service_load*length_m**2/2.0*1e6/(area*(d-x_cr/3.0))
            # Required inward straight anchorage at actual ULS steel stress;
            # no beneficial shape/cover/confinement reductions are applied.
            sigma_uls = med*1e6/(area*z)
            fbd = 2.25*0.7*(0.7*fctm/EDGE_GAMMA_C)
            lb_req = diameter/4.0*sigma_uls/fbd
            lbd = max(lb_req,0.3*lb_req,10.0*diameter,100.0)
            available = max(0.0,back_bay_m*1000.0-EDGE_COVER_MM)
            util = dict(bending=med/mrd,shear=ved/vrd,deflection=delta/limit,
                        steel_min=max(req,amin)/area,ductility=x_uls/(0.45*d),
                        anchorage=lbd/available if available > 0 else float('inf'),
                        service_steel=sigma_sls/EDGE_FYK)
            failed = [key for key,value in util.items() if value > 1.0+1e-9]
            if service_load+1e-9 < DENSITY*slab_m:
                failed.append('service_load_below_slab_self_weight')
            candidates.append(dict(status='FAIL' if failed else 'PASS_ASSUMED_MODEL',
                reasons=failed,diameter=diameter,spacing=spacing,As_req=req,As_min=amin,
                As_provided=area,d=d,MEd=med,MRd=mrd,VEd=ved,VRdc=vrd,
                Ecm=ecm,Eeff=e_eff,EI=ei*1e-9,x_cr=x_cr,deflection=delta,
                deflection_limit=limit,anchorage=lbd,available_anchorage=available,
                utilisation=util,qd=qd,qs=service_load,length=length_m))
    if not candidates:
        return dict(status='FAIL',reasons=['no_bar_arrangement_fits_slab'])
    passed = [r for r in candidates if r['status']=='PASS_ASSUMED_MODEL']
    if passed:
        return min(passed,key=lambda r:(r['As_provided'],r['diameter'],-r['spacing']))
    return min(candidates,key=lambda r:(max(r['utilisation'].values()),r['As_provided']))


def check_edges(levels, selected, slab, floor, roof, tol):
    offset = selected['offset']
    if offset <= tol:
        return 'NOT_APPLICABLE',[],['EDGE CHECK: no inward-offset slab strip.'],[]
    xs,ys = selected['xs'],selected['ys']
    rows,summary,results = [],[],[]
    for k,support in enumerate(selected['supports']):
        active_x = [x for x in xs if support[0]-tol <= x <= support[1]+tol]
        active_y = [y for y in ys if support[2]-tol <= y <= support[3]+tol]
        backs = [('X_MIN',active_x[1]-active_x[0]),('X_MAX',active_x[-1]-active_x[-2]),
                 ('Y_MIN',active_y[1]-active_y[0]),('Y_MAX',active_y[-1]-active_y[-2])]
        # An adverse UDL envelope, not an inferred permanent/variable split.
        qs = roof if k == len(levels)-1 else max(floor,roof)
        for side,back in backs:
            r = design_edge_strip(offset,slab,qs,back)
            results.append(r)
            if 'As_provided' not in r:
                rows.append(','.join([str(k),side,'FAIL']+['']*20+[';'.join(r['reasons'])]))
                summary.append('Edge {} {}: FAIL - {}'.format(k,side,';'.join(r['reasons'])))
                continue
            vals = [k,side,r['status'],r['length'],r['qs'],r['qd'],r['diameter'],r['spacing'],
                    r['As_req'],r['As_min'],r['As_provided'],r['d'],r['MEd'],r['MRd'],r['VEd'],r['VRdc'],
                    r['EI'],r['deflection'],r['deflection_limit'],r['anchorage'],
                    r['utilisation']['bending'],r['utilisation']['shear'],r['utilisation']['deflection'],
                    ';'.join(r['reasons'])]
            rows.append(','.join('{:.6g}'.format(v) if isinstance(v,float) else str(v) for v in vals))
            summary.append('Edge {} {}: {} | top phi{:.0f}@{:.0f} mm | As {:.1f} mm2/m | M {:.3f}/{:.3f} kNm/m | V {:.3f}/{:.3f} kN/m | EI {:.2f} kNm2 per 1m strip | deflection {:.4f}/{:.4f} mm | inward anchorage {:.0f} mm{}'.format(
                k,side,r['status'],r['diameter'],r['spacing'],r['As_provided'],r['MEd'],r['MRd'],r['VEd'],r['VRdc'],
                r['EI'],r['deflection'],r['deflection_limit'],r['anchorage'],
                (' | failed: '+';'.join(r['reasons'])) if r['reasons'] else ''))
    overall = 'FAIL' if any(r['status']=='FAIL' for r in results) else 'PASS_ASSUMED_MODEL'
    prefix = [
        'EDGE CHECK: '+overall+' - one-metre fixed-root strip, automatically selected top bars.',
        'Edge material scenario: C30/37; B500; cover 35 mm; gamma_c=1.50; gamma_s=1.15; alpha_cc=0.85.',
        'ULS edge UDL = '+'{:.2f}'.format(ULS_GRAVITY)+' x total service UDL (EN 1990 6.10b, 70 % permanent assumed); not a verified NL load combination.',
        'Edge stiffness: fully cracked transformed section; Ecm calculated from fck; assumed creep=2; entire service UDL sustained.',
        'Bending/shear/deflection, minimum steel, x/d and inward anchorage-space comparisons are performed.',
        'Deflection target L/250 is internal. Support rotation, shrinkage, shear deformation, corners, point/facade line loads and crack width are excluded.',
        'PASS applies only to this assumed strip model. Dutch National Annex selection, full detailing and whole-frame analysis remain unverified.',
        'Selected bars are for the edge strips only, not the complete slab or frame reinforcement.',
        'Edge_Check_Data: storey,side,status,L_m,qSLS_kNm2,qULS_kNm2,bar_mm,spacing_mm,As_req_mm2m,As_min_mm2m,As_provided_mm2m,d_mm,MEd_kNm_per_m,MRd_kNm_per_m,VEd_kN_per_m,VRdc_kN_per_m,EI_kNm2,deflection_mm,limit_mm,inward_anchor_mm,U_M,U_V,U_deflection,failed_checks'
    ]
    return overall,rows,prefix+summary,results


# 7b. Apartment programme fit

CORRIDOR_WIDTH_DEFAULT = 1.8
PROGRAMME_CORRIDOR_WIDTH = CORRIDOR_WIDTH_DEFAULT
PROGRAMME_MIN_UNIT_DEPTH = 6.0
PROGRAMME_CORE_MIN_WIDTH = 3.0
COMMON_RATIO_DEFAULT = 0.05
PROGRAMME_COMMON_RATIO = COMMON_RATIO_DEFAULT
CORE_AREA_DEFAULT = 25.0
PROGRAMME_CORE_AREA = CORE_AREA_DEFAULT
MAX_UNIT_WIDTH = None
APARTMENT_GEN = 0
APARTMENT_DEPTH = None
TYPE_MIX = None
APT_MANUAL_COUNT = None
MIX_MAX_UNITS_PER_SLOT = 4
MIX_SHARE_TOLERANCE = 0.05
BAY_SET_LIMIT = 400
BAY_SET_MARGIN = 0.5
BAY_SET_FULL_CHECKS = 8
PROGRAMME_CORE_SERVED_AREA = 600.0
PROGRAMME_MARGIN = 0.05
APARTMENT_TYPE_NAMES = ('Studio', '1 bedroom', '2 bedroom', '3 bedroom', 'Penthouse')
TOP_STOREY_KEYWORD = 'penthouse'
PROGRAMME_DAYLIGHT_DEPTH = 10.0
PROGRAMME_MIN_FRONTAGE = 4.0
PROGRAMME_DATA_HEADER = 'storey,footprint_m2,typology,band_depth_m,band_length_m,used_length_m,spare_length_m,circulation_m2,cores_m2,common_m2,placed_units,placed_area_m2'
APARTMENT_DATA_HEADER = 'apartment_id,storey,type,area_m2,width_m,depth_m,facade_band,x0,x1,y0,y1,z0,z1,box_area_m2'
SUPPORT_DATA_HEADER = 'support_id,kind,storey,area_m2,x0,x1,y0,y1,z0,z1'


def as_list(value):
    if value is None:
        return []
    if isinstance(value, (str, bytes)) or not hasattr(value, '__iter__'):
        return [value]
    return [v for v in value if v is not None]


def read_programme():
    areas = as_list(inp('AreaPerTypeOfApartment'))
    names = [str(n).strip() for n in as_list(inp('Apartment_Names'))]
    if not areas:
        return None
    if names and len(names) != len(areas):
        raise ValueError('Apartment_Names has {} items but AreaPerTypeOfApartment has {}; supply one name per apartment type or leave it empty.'.format(len(names), len(areas)))
    if any(not n for n in names):
        raise ValueError('Apartment_Names contains an empty name.')
    parsed = []
    for i, a in enumerate(areas):
        a = float(a)
        if not isfinite(a) or a <= 0:
            raise ValueError('AreaPerTypeOfApartment item {} must be > 0 m2.'.format(i))
        parsed.append(a)
    return parsed, names or None


def names_order_for_mix(types):
    return [x['name'] for x in types]


def apartment_types(areas, names=None):
    if names is None:
        if len(areas) == len(APARTMENT_TYPE_NAMES):
            names = APARTMENT_TYPE_NAMES
        else:
            names = tuple('Type {}'.format(i+1) for i in range(len(areas)))
    return [dict(name=n, area=a, top=TOP_STOREY_KEYWORD in n.lower()) for n, a in zip(names, areas)]


def plan_box(along, a0, a1, c0, c1, z0, z1):
    return (a0,a1,c0,c1,z0,z1) if along == 0 else (c0,c1,a0,a1,z0,z1)


def knapsack_bays(length, options):
    cap = int(math.floor(length*100.0+1e-6))
    items = [(name, int(math.ceil(width*100.0-1e-6)), width, value) for name, width, value in options if width <= length+1e-9]
    best, pick = [0.0]*(cap+1), [None]*(cap+1)
    for x in range(1, cap+1):
        best[x] = best[x-1]
        for index, (name, w, width, value) in enumerate(items):
            if w <= x and best[x-w]+value > best[x]+1e-9:
                best[x], pick[x] = best[x-w]+value, index
    chosen, x = [], cap
    while x > 0:
        if pick[x] is None or best[x] <= best[x-1]+1e-12 and pick[x] is None:
            x -= 1
            continue
        name, w, width, value = items[pick[x]]
        chosen.append((name, width))
        x -= w
    return sorted(chosen, key=lambda c: -c[1])


def bay_geometry(levels):
    along = long_axis(levels)
    cross = 2-along
    corridor = PROGRAMME_CORRIDOR_WIDTH
    geo = []
    for k, p in enumerate(levels):
        c0, c1 = p[cross], p[cross+1]
        width = c1-c0
        anchored = False
        cap = min(PROGRAMME_DAYLIGHT_DEPTH, APARTMENT_DEPTH or PROGRAMME_DAYLIGHT_DEPTH)
        strip = (0.0, 0.0)
        if width >= 2*PROGRAMME_MIN_UNIT_DEPTH+corridor:
            typology, depth, sides = 'DOUBLE_LOADED_CORRIDOR', min(cap, (width-corridor)/2.0), (0, 1)
            zone = (c0+depth, c1-depth)
            depths = (depth, depth)
            if k and geo[0]['typology'] == typology:
                base = geo[0]
                if (c0 < base['c0']-1e-6 or c1 > base['c1']+1e-6) and c0 <= base['zone'][0]+1e-6 and c1 >= base['zone'][1]-1e-6:
                    r0, r1 = base['zone'][0]-c0, c1-base['zone'][1]
                    d0, d1 = min(cap, r0), min(cap, r1)
                    zone, depths, depth, anchored = base['zone'], (d0, d1), min(d0, d1), True
                    strip = (r0-d0, r1-d1)
        else:
            depth = max(0.0, min(PROGRAMME_DAYLIGHT_DEPTH, width-corridor))
            typology, sides = 'SINGLE_LOADED_GALLERY', ((0,) if depth > 0 else ())
            zone = (c1-corridor, c1) if depth > 0 else (c0, c1)
            depths = (depth, 0.0)
        geo.append(dict(typology=typology, depth=depth, depths=depths, anchored=anchored, strip=strip, sides=sides, c0=c0, c1=c1, zone=zone,
                        a0=p[along], a1=p[along+1], z0=p[4], z1=p[5]))
    return along, geo


def programme_check(levels, columns, areas, names=None, wall_thickness=0.0, max_bay=None, end_thickness=None, forced_bays=None):
    types = apartment_types(areas, names)
    storeys = len(levels)
    along, geo = bay_geometry(levels)
    g0 = geo[0]
    t = wall_thickness
    te = t if end_thickness is None else end_thickness
    ext = (te-t)/2.0
    footprint = [(p[1]-p[0])*(p[3]-p[2]) for p in levels]
    typical_depth = geo[1]['depth'] if storeys > 1 and geo[1]['depth'] > 0 else g0['depth']
    def need(area, depth):
        if depth <= 0:
            return float('inf')
        return max(PROGRAMME_MIN_FRONTAGE, area/depth)+t
    cap = MAX_UNIT_WIDTH+t if MAX_UNIT_WIDTH else float('inf')
    def fits(area, depth):
        return need(area, depth) <= cap+1e-9
    area_of_type = dict((x['name'], x['area']) for x in types)
    cores = CORE_COUNT or max(1, int(math.ceil(footprint[0]/PROGRAMME_CORE_SERVED_AREA-1e-9)))
    core_len = PROGRAMME_CORE_AREA/g0['depth'] if g0['depth'] > 0 else 0.0
    core_bay = max(core_len, PROGRAMME_MIN_FRONTAGE)+t
    a0, a1 = g0['a0']+te/2.0, g0['a1']-te/2.0
    if cores*core_bay > a1-a0+1e-9:
        raise ValueError('Core_Count {}: {} core bays of {:.2f} m do not fit in the {:.2f} m building length.'.format(cores, cores, core_bay, a1-a0))
    facade_ends = []
    def half(pos):
        if any(abs(pos-f) < 1e-6 for f in facade_ends):
            return -te/2.0
        return te/2.0 if abs(pos-a0) < 1e-6 or abs(pos-a1) < 1e-6 else t/2.0
    core_starts = [a0] if cores == 1 else [a0+(a1-a0-core_bay)*i/(cores-1.0) for i in range(cores)]
    bays, segments, cursor = [], [], a0
    for cs in core_starts:
        ce = cs+core_bay+(ext if abs(cs-a0) < 1e-9 else 0.0)
        if abs(cs+core_bay-a1) < 1e-9 and abs(cs-a0) > 1e-9:
            cs -= ext
            ce = a1
        if cs-cursor > 1e-9:
            segments.append((cursor, cs))
        bays.append(dict(start=cs, end=ce, kind='core'))
        cursor = ce
    if a1-cursor > 1e-9:
        segments.append((cursor, a1))
    segments = [(st+(ext if abs(st-a0) < 1e-9 else 0.0), en-(ext if abs(en-a1) < 1e-9 else 0.0)) for st, en in segments]
    slots_per_bay = len(g0['sides'])*storeys
    excluded = [x['name'] for x in types if not fits(x['area'], typical_depth if not x['top'] else geo[storeys-1]['depth'])]
    stack_types = [x for x in types if not x['top'] and x['name'] not in excluded]
    options = [(x['name'], need(x['area'], typical_depth), x['area']*slots_per_bay+1e-3) for x in stack_types]
    gen = APARTMENT_GEN if APARTMENT_GEN in (1, 2) else 0
    names_all = [x['name'] for x in types]
    if gen == 1:
        total_mix = float(sum(TYPE_MIX))
        target = dict((n, v/total_mix) for n, v in zip(names_all, TYPE_MIX))
    elif gen == 2:
        target = dict((n, int(v)) for n, v in zip(names_all, APT_MANUAL_COUNT))
    else:
        target = {}
    combo_cache = {}
    def combos(cap, depth):
        key = (round(cap, 4), round(depth, 4))
        if key not in combo_cache:
            allowed = sorted([(x['name'], need(x['area'], depth), x['area']) for x in stack_types if fits(x['area'], depth) and target.get(x['name'], 0) > 0], key=lambda a: -a[1])
            out = []
            def walk(start, used, chosen):
                if chosen:
                    out.append(tuple(chosen))
                if len(chosen) >= MIX_MAX_UNITS_PER_SLOT:
                    return
                for j in range(start, len(allowed)):
                    if used+allowed[j][1] <= cap+1e-9:
                        walk(j, used+allowed[j][1], chosen+[allowed[j][0]])
            walk(0, 0.0, [])
            combo_cache[key] = out
        return combo_cache[key]
    share_tolerance = [MIX_SHARE_TOLERANCE]
    def mix_score(counts):
        if gen == 2:
            deviation = sum(abs(counts.get(n, 0)-target.get(n, 0)) for n in names_all)
        else:
            total = sum(counts.values())
            if not total:
                return (float('inf'), 0.0, float('inf'))
            gaps = [abs(counts.get(n, 0)/float(total)-target.get(n, 0.0)) for n in names_all]
            excess = sum(max(0.0, gap-share_tolerance[0]) for gap in gaps)
            return (round(excess, 9), -sum(counts.get(n, 0)*area_of_type[n] for n in names_all), round(sum(gaps), 9))
        return (round(deviation, 9), -sum(counts.get(n, 0)*area_of_type[n] for n in names_all))
    def assign(classes, fixed):
        opts = {}
        for key, count in classes.items():
            cap, depth, empty = key
            choices = list(combos(cap, depth))
            if empty or not choices:
                choices.append(())
            opts[key] = choices
        best = None
        starts = ['area', 'empty'] if gen == 2 else ['area']
        for start in starts:
            dist, counts = {}, defaultdict(int)
            for n, c in fixed.items():
                counts[n] += c
            for key, count in classes.items():
                if start == 'empty' and () in opts[key]:
                    first = ()
                else:
                    first = max(opts[key], key=lambda f: sum(area_of_type[n] for n in f))
                dist[key] = defaultdict(int)
                dist[key][first] = count
                for n in first:
                    counts[n] += count
            current = mix_score(counts)
            while True:
                move = None
                for key in dist:
                    for old, held in list(dist[key].items()):
                        if held <= 0:
                            continue
                        for new in opts[key]:
                            if new == old:
                                continue
                            for n in old:
                                counts[n] -= 1
                            for n in new:
                                counts[n] += 1
                            score = mix_score(counts)
                            for n in new:
                                counts[n] -= 1
                            for n in old:
                                counts[n] += 1
                            if score < current and (move is None or score < move[0]):
                                move = (score, key, old, new)
                if move is None:
                    singles = [(key, old, new) for key in dist for old, held in dist[key].items() if held > 0 for new in opts[key] if new != old]
                    pair = None
                    for ia, (ka, oa, na) in enumerate(singles):
                        for n in oa:
                            counts[n] -= 1
                        for n in na:
                            counts[n] += 1
                        for kb, ob, nb in singles[ia+1:]:
                            if (kb, ob) == (ka, oa) and dist[ka][oa] < 2:
                                continue
                            for n in ob:
                                counts[n] -= 1
                            for n in nb:
                                counts[n] += 1
                            score = mix_score(counts)
                            for n in nb:
                                counts[n] -= 1
                            for n in ob:
                                counts[n] += 1
                            if score < current and (pair is None or score < pair[0]):
                                pair = (score, (ka, oa, na), (kb, ob, nb))
                        for n in na:
                            counts[n] -= 1
                        for n in oa:
                            counts[n] += 1
                    if pair is None:
                        break
                    current = pair[0]
                    for key, old, new in pair[1:]:
                        dist[key][old] -= 1
                        dist[key][new] += 1
                        for n in old:
                            counts[n] -= 1
                        for n in new:
                            counts[n] += 1
                    continue
                score, key, old, new = move
                while dist[key][old] > 0:
                    for n in old:
                        counts[n] -= 1
                    for n in new:
                        counts[n] += 1
                    trial = mix_score(counts)
                    if trial < current:
                        current = trial
                        dist[key][old] -= 1
                        dist[key][new] += 1
                    else:
                        for n in new:
                            counts[n] -= 1
                        for n in old:
                            counts[n] += 1
                        break
            if best is None or current < best[0]:
                best = (current, dist)
        return best
    def fill(free, forced, opts=None):
        result = []
        pending = sorted(forced, key=lambda f: -f[1])
        free = sorted(free, key=lambda sg: -(sg[1]-sg[0]))
        for st, en in free:
            placed_here, cursor = [], st
            for item in list(pending):
                if en-cursor >= item[1]-1e-9:
                    placed_here.append(dict(start=cursor, end=cursor+item[1], kind='unit', type=item[0]))
                    cursor += item[1]
                    pending.remove(item)
            for name, width in knapsack_bays(en-cursor, opts or options):
                placed_here.append(dict(start=cursor, end=cursor+width, kind='unit', type=name))
                cursor += width
            remainder = en-cursor
            if remainder > 1e-6 and placed_here and max_bay:
                widths = [b['end']-b['start'] for b in placed_here]
                parts = [int(math.ceil(w/max_bay-1e-9)) for w in widths]
                def limit(p):
                    return min(p*max_bay, cap)
                while sum(max(0.0, limit(p)-w) for p, w in zip(parts, widths)) < remainder-1e-9:
                    growable = [i for i in range(len(widths)) if parts[i]*max_bay < cap-1e-9]
                    if not growable:
                        break
                    widest = max(growable, key=lambda i: widths[i]/parts[i])
                    parts[widest] += 1
                room = [max(0.0, limit(p)-w) for p, w in zip(parts, widths)]
                total_room = sum(room)
                share = min(remainder, total_room)
                if total_room > 1e-9:
                    widths = [w+share*r/total_room for w, r in zip(widths, room)]
                cursor = st
                for b, w in zip(placed_here, widths):
                    b['start'], b['end'] = cursor, cursor+w
                    cursor += w
                remainder = en-cursor
            if remainder > 1e-6:
                if placed_here and remainder < PROGRAMME_MIN_FRONTAGE+t and placed_here[-1]['end']-placed_here[-1]['start']+remainder <= cap+1e-9:
                    placed_here[-1]['end'] = en
                else:
                    placed_here.append(dict(start=cursor, end=en, kind='spare'))
            result += placed_here
        return result
    top_names = [x['name'] for x in types if x['top']]
    if gen and forced_bays is not None:
        unit_bays = [dict(b) for b in forced_bays]
    elif gen:
        shares = dict(target) if gen == 1 else dict((n, c/float(max(1, sum(target.values())))) for n, c in target.items())
        weights = [{}, dict((x['name'], 1.0+4.0*shares.get(x['name'], 0.0)) for x in stack_types)]
        for x in stack_types:
            for factor in (1.5, 2.0, 5.0, 20.0):
                weights.append({x['name']: factor})
        for ia, xa in enumerate(stack_types):
            for xb in stack_types[ia+1:]:
                for factor in (2.0, 5.0):
                    weights.append({xa['name']: factor, xb['name']: factor})
        for factor in (0.5, 0.2):
            weights.append(dict((x['name'], factor) for x in stack_types if shares.get(x['name'], 0.0) <= 0.0))
        seen, ranked = set(), []
        trials = [fill(segments, [], [(n, w, v*weight.get(n, 1.0)) for n, w, v in options]) for weight in weights]
        items = sorted(set((round(w, 4), n) for n, w, v in options), key=lambda it: (-it[0], it[1]))
        widths_seen, unique_items = set(), []
        for w, n in items:
            if w not in widths_seen:
                widths_seen.add(w)
                unique_items.append((w, n))
        per_segment = []
        for st, en in segments:
            bay_sets = []
            def walk(i, left, chosen):
                if len(bay_sets) > BAY_SET_LIMIT:
                    return
                grown = False
                for j in range(i, len(unique_items)):
                    if unique_items[j][0] <= left+1e-9:
                        grown = True
                        walk(j, left-unique_items[j][0], chosen+[unique_items[j]])
                if not grown and chosen:
                    bay_sets.append((left, chosen))
            walk(0, en-st, [])
            bay_sets.sort(key=lambda c: c[0])
            per_segment.append([c[1] for c in bay_sets])
        budget = max(1, int(BAY_SET_LIMIT**(1.0/max(1, len(per_segment)))))
        sets = [[]]
        for seg_index, bay_sets in enumerate(per_segment):
            sets = [prev+[(seg_index, combo)] for prev in sets for combo in (bay_sets[:budget] or [[]])]
        no_opts = [('none', 1e9, 0.0)]
        for chosen_set in sets:
            trial = []
            for seg_index, combo in chosen_set:
                trial += fill([segments[seg_index]], [(n, w) for w, n in combo], no_opts)
            trials.append(trial)
        share_tolerance[0] = MIX_SHARE_TOLERANCE*BAY_SET_MARGIN
        for trial_bays in trials:
            signature = tuple((b['kind'], round(b['end']-b['start'], 3)) for b in trial_bays)
            if signature in seen:
                continue
            seen.add(signature)
            classes = defaultdict(int)
            for b in trial_bays:
                if b['kind'] == 'unit':
                    classes[round(b['end']-b['start'], 4), round(typical_depth, 4), gen == 2] += slots_per_bay
            slots_total = sum(classes.values())
            if gen == 2:
                fixed_estimate = dict((n, target.get(n, 0)) for n in top_names)
            else:
                fixed_estimate = dict((n, int(round(target.get(n, 0.0)*slots_total))) for n in top_names)
            result = assign(classes, fixed_estimate) if classes else ((float('inf'), 0.0, float('inf')), {})
            ranked.append((result[0], len(ranked), trial_bays))
        share_tolerance[0] = MIX_SHARE_TOLERANCE
        ranked.sort(key=lambda r: (r[0], r[1]))
        best = None
        for light, order, trial_bays in ranked[:BAY_SET_FULL_CHECKS]:
            full = programme_check(levels, columns, areas, names, wall_thickness, max_bay, end_thickness, forced_bays=[dict(b) for b in trial_bays])
            counts = dict((n, c) for n, c in full[4].get('mix', {}).items() if c)
            score = mix_score(defaultdict(int, counts))
            if best is None or score < best[0]:
                best = (score, trial_bays)
        unit_bays = [dict(b) for b in best[1]] if best else fill(segments, [])
    else:
        unit_bays = fill(segments, [])
        widest = max([b['end']-b['start'] for b in unit_bays if b['kind'] == 'unit'] or [0.0])
        forced = [(x['name'], need(x['area'], typical_depth)) for x in stack_types if need(x['area'], typical_depth) > widest+1e-9]
        if forced:
            unit_bays = fill(segments, forced)
    for b in unit_bays:
        if abs(b['start']-(a0+ext)) < 1e-9 and ext:
            b['start'] = a0
        if abs(b['end']-(a1-ext)) < 1e-9 and ext:
            b['end'] = a1
    bays += unit_bays
    over_pieces = []
    if max(g['a1'] for g in geo) > g0['a1']+1e-6:
        marks = sorted(set([a1]+[g['a1']-te/2.0 for g in geo if g['a1'] > g0['a1']+1e-6]))
        over_pieces += list(zip(marks, marks[1:]))
    if min(g['a0'] for g in geo) < g0['a0']-1e-6:
        marks = sorted(set([a0]+[g['a0']+te/2.0 for g in geo if g['a0'] < g0['a0']-1e-6]))
        over_pieces += list(zip(marks, marks[1:]))
    outer_right = max([en for st, en in over_pieces if st >= a1-1e-6] or [None])
    outer_left = min([st for st, en in over_pieces if en <= a0+1e-6] or [None])
    facade_gain = te/2.0+t/2.0+1e-6
    facade_ends += [x for x in (outer_left, outer_right) if x is not None]
    for st, en in over_pieces:
        if en-st < 1e-6:
            continue
        lo, hi = st, en
        if outer_right is not None and abs(en-outer_right) < 1e-6:
            hi = en+facade_gain
        if outer_left is not None and abs(st-outer_left) < 1e-6:
            lo = st-facade_gain
        piece = [b for b in fill([(lo, hi)], []) if min(b['end'], en)-max(b['start'], st) > 1e-6]
        for b in piece:
            b['start'], b['end'] = max(b['start'], st), min(b['end'], en)
        piece = piece or [dict(start=st, end=en, kind='spare')]
        for b in piece:
            b['overhang'] = True
        bays += piece
    bays.sort(key=lambda b: b['start'])
    for b in bays:
        width = b['end']-b['start']
        parts = int(math.ceil(width/max_bay-1e-9)) if max_bay else 1
        b['splits'] = [b['start']+width*j/parts for j in range(1, parts)] if parts > 1 else []
    def clear_width(i):
        return bays[i]['end']-bays[i]['start']-half(bays[i]['start'])-half(bays[i]['end'])+t
    def valid(i, side, k):
        b, g = bays[i], geo[k]
        return (side in g['sides'] and g['depths'][side] > 0 and b['start'] >= g['a0']-1e-6 and b['end'] <= g['a1']+1e-6)
    occupied = {}
    for i, b in enumerate(bays):
        if b['kind'] == 'core':
            for k in range(storeys):
                if valid(i, 0, k):
                    occupied[i, 0, k] = ('core', None, None)
                for side in geo[k]['sides']:
                    if side == 0 or not valid(i, side, k):
                        continue
                    fitting = [x for x in stack_types if need(x['area'], geo[k]['depths'][side]) <= clear_width(i)+1e-9]
                    if fitting:
                        best_type = max(fitting, key=lambda x: x['area'])
                        occupied[i, side, k] = ('unit', best_type['name'], best_type['area'])
        elif b['kind'] == 'unit':
            for k in range(storeys):
                for side in geo[k]['sides']:
                    if valid(i, side, k) and need(area_of_type[b['type']], geo[k]['depths'][side]) <= clear_width(i)+1e-9:
                        occupied[i, side, k] = ('unit', b['type'], area_of_type[b['type']])
    def designed_area():
        return sum(v[2] for v in occupied.values() if v[0] == 'unit')
    common = PROGRAMME_COMMON_RATIO*designed_area()
    common_ok, common_slots = True, []
    if common > 0:
        common_ok = False
        need_len, length = (common/g0['depth'] if g0['depth'] > 0 else float('inf')), 0.0
        for side in g0['sides']:
            for i, b in enumerate(bays):
                if b['kind'] == 'core' or not valid(i, side, 0) or clear_width(i) < PROGRAMME_MIN_FRONTAGE+t-1e-9:
                    continue
                use = min(clear_width(i)-t, need_len-length)
                common_slots.append((i, side, use))
                length += use
                if length >= need_len-1e-9:
                    break
            if length >= need_len-1e-9:
                common_ok = True
                break
        if not common_ok:
            common_slots = []
        for i, side, use in common_slots:
            occupied[i, side, 0] = ('common', None, use)
    top = storeys-1
    spans = {}
    missing = []
    unit_slots = sum(1 for v in occupied.values() if v[0] == 'unit')
    def top_target(x):
        if gen == 2:
            return target.get(x['name'], 0)
        if gen == 1:
            share = target.get(x['name'], 0.0)
            return max(1, int(round(share*unit_slots))) if share > 0 else 0
        return 1
    top_runs = []
    for x in [x for x in types if x['top'] and x['name'] not in excluded for copy in range(top_target(x))]:
        top_runs.append(x)
    for x in top_runs:
        placed_top = False
        for side in geo[top]['sides']:
            need_len = need(x['area'], geo[top]['depths'][side])
            run_bays, length = [], 0.0
            for i in reversed(range(len(bays))):
                slot = occupied.get((i, side, top))
                if bays[i]['kind'] == 'core' or not valid(i, side, top) or (slot is not None and slot[0] != 'unit'):
                    run_bays, length = [], 0.0
                    continue
                run_bays.insert(0, i)
                length += bays[i]['end']-bays[i]['start']
                if length >= need_len-1e-9:
                    break
            if length >= need_len-1e-9:
                for i in run_bays:
                    occupied[i, side, top] = ('top', x['name'], x['area'])
                spans[run_bays[0], side, top] = (run_bays, x['name'], x['area'])
                placed_top = True
                break
        if not placed_top and not (gen and any(name == x['name'] for run, name, area in spans.values())):
            missing.append(x['name'])
    present = set(v[1] for v in occupied.values() if v[0] in ('unit', 'top'))
    for x in sorted([x for x in stack_types if x['name'] not in present and not gen], key=lambda x: -x['area']):
        options_swap = []
        for (i, side, k), v in occupied.items():
            if v[0] != 'unit' or sum(1 for w in occupied.values() if w[0] == 'unit' and w[1] == v[1]) <= 1:
                continue
            if need(x['area'], geo[k]['depths'][side]) <= clear_width(i)+1e-9:
                left = clear_width(i)-need(x['area'], geo[k]['depths'][side])
                bonus = sum(area_of_type[n] for n, w in knapsack_bays(left, [(y['name'], need(y['area'], geo[k]['depths'][side]), y['area']) for y in stack_types])) if left > 0 else 0.0
                options_swap.append((v[2]-x['area']-bonus if v[2] >= x['area'] else 1e6+x['area']-v[2], k, i, side))
        for i, b in enumerate(bays):
            if b['kind'] == 'spare':
                for k in range(storeys):
                    for side in geo[k]['sides']:
                        if valid(i, side, k) and (i, side, k) not in occupied and need(x['area'], geo[k]['depths'][side]) <= clear_width(i)+1e-9:
                            options_swap.append((-x['area'], k, i, side))
        if options_swap:
            loss, k, i, side = min(options_swap)
            occupied[i, side, k] = ('unit', x['name'], x['area'])
        else:
            missing.append(x['name'])
    placements, extras, split_slots, common_cuts = [], 0, set(), {}
    vacant_area = 0.0
    if gen:
        fixed_top = defaultdict(int)
        for run_bays, name, area in spans.values():
            fixed_top[name] += 1
        slot_key, classes = {}, defaultdict(int)
        for (i, side, k), v in sorted(occupied.items()):
            if v[0] not in ('unit', 'common'):
                continue
            prefix = v[2]+t if v[0] == 'common' else 0.0
            key = (round(clear_width(i)-prefix, 4), round(geo[k]['depths'][side], 4), gen == 2 or v[0] == 'common' or bays[i]['kind'] == 'core')
            slot_key[i, side, k] = key
            classes[key] += 1
        score, dist = assign(classes, fixed_top) if classes else ((0.0, 0.0), {})
        pools = dict((key, sorted([f for f, c in held.items() for copy in range(c)], key=lambda f: (-len(f), f))) for key, held in dist.items())
        for (i, side, k) in sorted(slot_key, key=lambda s: (s[0], s[1], s[2])):
            key = slot_key[i, side, k]
            fill_now = pools[key].pop(0) if pools.get(key) else ()
            v = occupied[i, side, k]
            start, end = bays[i]['start'], bays[i]['end']
            fill_now = sorted(fill_now, key=lambda n: -need(area_of_type[n], geo[k]['depths'][side]))
            cuts = [start]
            if v[0] == 'common':
                cuts.append(start+v[2]+t+half(start)-t/2.0)
            for j, n in enumerate(fill_now):
                width = need(area_of_type[n], geo[k]['depths'][side])
                cuts.append(cuts[-1]+width+(half(start)-t/2.0 if j == 0 and v[0] != 'common' else 0.0))
            if fill_now:
                cuts[-1] = end
            ranges = list(zip(cuts, cuts[1:]))
            if v[0] == 'common':
                if fill_now:
                    common_cuts[i, side] = ranges.pop(0)
                    extras += len(fill_now)
                    for n, rng in zip(fill_now, ranges):
                        placements.append(([i], side, k, area_of_type[n], n, [rng]))
                continue
            if not fill_now:
                vacant_area += (clear_width(i)-t)*geo[k]['depths'][side]
                del occupied[i, side, k]
                continue
            extras += len(fill_now)-1
            for n, rng in zip(fill_now, ranges):
                placements.append(([i], side, k, area_of_type[n], n, [rng]))
        split_slots = set(slot_key)
    for (i, side, k), v in list(occupied.items()):
        if gen or v[0] not in ('unit', 'common'):
            continue
        start, end = bays[i]['start'], bays[i]['end']
        first = need(v[2], geo[k]['depths'][side]) if v[0] == 'unit' else v[2]+t
        left = clear_width(i)-first
        opts = [(x['name'], need(x['area'], geo[k]['depths'][side]), x['area']) for x in stack_types]
        picks = knapsack_bays(left, opts) if opts and left >= min(o[1] for o in opts)-1e-9 else []
        if not picks:
            continue
        cuts = [start, start+first+half(start)-t/2.0]
        for name, width in picks:
            cuts.append(cuts[-1]+width)
        cuts[-1] = end
        ranges = list(zip(cuts, cuts[1:]))
        units = [(name, area_of_type[name]) for name, width in picks]
        if v[0] == 'unit':
            units.insert(0, (v[1], v[2]))
            split_slots.add((i, side, k))
        else:
            common_cuts[i, side] = ranges.pop(0)
        for (name, area), rng in zip(units, ranges):
            placements.append(([i], side, k, area, name, [rng]))
        extras += len(picks)
    placements += [([i], side, k, v[2], v[1], [(bays[i]['start'], bays[i]['end'])]) for (i, side, k), v in occupied.items() if v[0] == 'unit' and (i, side, k) not in split_slots]
    placements += [(run_bays, side, k, area, name, [(bays[i]['start'], bays[i]['end']) for i in run_bays]) for (i, side, k), (run_bays, name, area) in spans.items()]
    stretched, stretched_area = 0, 0.0
    for i, b in enumerate(bays):
        if b['kind'] != 'spare' or not b.get('overhang'):
            continue
        j = next((j for j in (i-1, i+1) if 0 <= j < len(bays) and bays[j]['kind'] == 'unit' and (abs(bays[j]['end']-b['start']) < 1e-6 or abs(bays[j]['start']-b['end']) < 1e-6)), None)
        if j is None:
            continue
        for n, (run_bays, side, k, area, name, ranges) in enumerate(placements):
            if run_bays != [j] or len(ranges) != 1 or abs(ranges[0][0]-bays[j]['start']) > 1e-6 or abs(ranges[0][1]-bays[j]['end']) > 1e-6 or not valid(i, side, k):
                continue
            extra = (b['end']-b['start'])*geo[k]['depths'][side]
            placements[n] = (sorted([j, i]), side, k, area+extra, name, sorted([ranges[0], (b['start'], b['end'])]))
            stretched += 1
            stretched_area += extra
    apartments = []
    spanning = 0
    wide = 0
    for number, (run_bays, side, k, area, name, ranges) in enumerate(sorted(placements, key=lambda p: (p[2], p[1], p[5][0][0]))):
        g = geo[k]
        pieces = [(st+half(st), en-half(en)) for st, en in ranges]
        wide += any(bays[i].get('splits') for i in run_bays) and len(ranges) == len(run_bays) and all(abs(r[1]-r[0]-(bays[i]['end']-bays[i]['start'])) < 1e-6 for r, i in zip(ranges, run_bays))
        width = sum(b-a for a, b in pieces)
        depth = g['depths'][side]+g['strip'][side]
        c0, c1 = (g['c0'], g['c0']+depth) if side == 0 else (g['c1']-depth, g['c1'])
        spanning += len(pieces) > 1
        for piece, (start, end) in enumerate(pieces):
            label = 'A_{}_{}'.format(k, number)+('.{}'.format(piece) if len(pieces) > 1 else '')
            apartments.append((label, k, name, area*(end-start)/width, end-start, depth, side,
                               plan_box(along, start, end, c0, c1, g['z0'], g['z1'])))
    supports = []
    for k in range(storeys):
        g = geo[k]
        for i, b in enumerate(bays):
            if b['kind'] == 'core' and valid(i, 0, k):
                cc0, cc1 = max(g0['c0'], g['c0']), min(g0['c0']+g0['depth'], g['c1'])
                if cc1-cc0 > 1e-6:
                    supports.append(('core', k, plan_box(along, b['start']+half(b['start']), b['end']-half(b['end']), cc0, cc1, g['z0'], g['z1'])))
        if g['depth'] > 0:
            supports.append(('circulation', k, plan_box(along, g['a0'], g['a1'], g['zone'][0], g['zone'][1], g['z0'], g['z1'])))
    if common_ok and common > 0 and common_slots:
        pieces = []
        for i, side, use in common_slots:
            st, en = common_cuts.get((i, side), (bays[i]['start'], bays[i]['end']))
            pieces.append((st+half(st), en-half(en), side))
        for start, end, side in pieces:
            depth = g0['depths'][side]
            c0, c1 = (g0['c0'], g0['c0']+depth) if side == 0 else (g0['c1']-depth, g0['c1'])
            supports.append(('common', 0, plan_box(along, start, end, c0, c1, g0['z0'], g0['z1'])))
    supports = [('S_{}_{}_{}'.format(k, kind, i), k, kind, b) for i, (kind, k, b) in enumerate(supports)]
    lines = []
    overhang_partitions = []
    edges = sorted(set([round(b['start'], 6) for b in bays]+[round(b['end'], 6) for b in bays]))
    for pos in edges:
        left = [i for i, b in enumerate(bays) if abs(b['end']-pos) < 1e-6]
        right = [i for i, b in enumerate(bays) if abs(b['start']-pos) < 1e-6]
        shared = any((l, side, k) in occupied and (r, side, k) in occupied
                     for l in left for r in right if not bays[l].get('overhang') and not bays[r].get('overhang') for k in range(storeys) for side in (0, 1))
        overhang_split = any((l, side, k) in occupied and (r, side, k) in occupied
                             for l in left for r in right if bays[l].get('overhang') or bays[r].get('overhang') for k in range(storeys) for side in (0, 1))
        if overhang_split and not shared:
            overhang_partitions.append(pos)
        lines.append((pos, 'wall' if shared else 'frame'))
    for b in bays:
        for pos in b.get('splits', []):
            lines.append((pos, 'internal'))
    lines.sort()
    mix = defaultdict(int)
    placed = defaultdict(lambda: defaultdict(int))
    for run_bays, side, k, area, name, ranges in placements:
        mix[name] += 1
        placed[k][name] += 1
    def plan_area(b):
        return (b[1]-b[0])*(b[3]-b[2])
    area_of = defaultdict(float)
    for sid, k, kind, b in supports:
        area_of[kind, k] += plan_area(b)
    gfa = sum(footprint)
    placed_area = sum(pl[3] for pl in placements)
    units_total = len(placements)
    spare_length = sum(b['end']-b['start'] for b in bays if b['kind'] == 'spare')
    status = 'DESIGNED' if not missing and common_ok else 'INCOMPLETE'
    mix_off = []
    if gen:
        for n in names_order_for_mix(types):
            got = mix[n]
            if gen == 2:
                if got != target.get(n, 0):
                    mix_off.append('{} {} vs {} requested'.format(n, got, target.get(n, 0)))
            else:
                share = got/float(units_total) if units_total else 0.0
                if abs(share-target.get(n, 0.0)) > MIX_SHARE_TOLERANCE+1e-9:
                    mix_off.append('{} {:.0%} vs {:.0%} requested'.format(n, share, target.get(n, 0.0)))
        if mix_off and status == 'DESIGNED':
            status = 'MIX_DEVIATION'
    if excluded:
        status += '_LIMITED'
    names_order = [x['name'] for x in types]
    out = [
        'Programme design (max lettable area, stacked bays): {} | {} apartments, NLA {:.1f} m2 | GFA {:.1f} m2 | NLA/GFA {:.1%}.'.format(
            status, units_total, placed_area, gfa, placed_area/gfa if gfa > 0 else 0.0),
        'Designed mix: '+'; '.join('{} {} x {:g} m2 ({:.0%} of units)'.format(n, mix[n], area_of_type[n], mix[n]/float(units_total) if units_total else 0.0) for n in names_order),
        'Layout: {} along local {} | band depth {:.2f} m | {} {:.1f} m | bays (identical on every storey): {} | {} core bay(s) {:.2f} m (core {:.0f} m2 asked; core box {:.1f} m2 = clear bay x band depth, the minimum {:.1f} m frontage sets the width) | common {:.1f} m2 on storey 0{}.'.format(
            g0['typology'], 'X' if along == 0 else 'Y', g0['depth'],
            'corridor zone (min {:.1f} m)'.format(PROGRAMME_CORRIDOR_WIDTH) if g0['typology'] == 'DOUBLE_LOADED_CORRIDOR' else 'gallery', (g0['zone'][1]-g0['zone'][0]) if g0['typology'] == 'DOUBLE_LOADED_CORRIDOR' else PROGRAMME_CORRIDOR_WIDTH,
            ', '.join('{}:{:.2f}'.format(b.get('type', b['kind']), b['end']-b['start']) for b in bays),
            cores, core_bay, PROGRAMME_CORE_AREA, area_of['core', 0]/float(cores) if cores else 0.0, PROGRAMME_MIN_FRONTAGE, common, '' if common_ok else ' (DOES NOT FIT in a ground-floor bay run)'),
        'Bay lines (local {}): {} | party-wall lines {} | wall thickness allowance {:.2f} m | spare length {:.2f} m.'.format(
            'X' if along == 0 else 'Y', ', '.join('{:.2f}{}'.format(p, 'W' if kind == 'wall' else ('I' if kind == 'internal' else '')) for p, kind in lines),
            sum(1 for p, kind in lines if kind == 'wall'), t, spare_length)+(' W = party-wall line (load-bearing wall); I = split inside a unit wider than the slab span; unmarked = end/spare line. With Load_Bearing_Walls, I and unmarked lines are frame lines (core walls where a core sits).' if max_bay else ''),
        'Support space (from boxes): circulation {:.1f} m2 (the corridor zone of the ground floor, stacked on every storey that spans it, and run out into along-overhangs) + cores {:.1f} m2 (stacked at the ground-floor position) + common {:.1f} m2 (box: the common slots full band depth, {:.1f} m2 asked).'.format(
            sum(v for (kd, k), v in area_of.items() if kd == 'circulation'), sum(v for (kd, k), v in area_of.items() if kd == 'core'),
            sum(v for (kd, k), v in area_of.items() if kd == 'common'), common),
        'Apartment boxes run from the facade to the corridor (full band depth, so every unit touches the corridor): box area {:.1f} m2 vs NLA {:.1f} m2 by type area; Apartment_Data box_area_m2 per box.'.format(
            sum(a[4]*a[5] for a in apartments), placed_area)]
    for k in range(storeys):
        line = ', '.join('{} {}'.format(placed[k][n], n) for n in names_order if placed[k][n]) or 'none'
        out.append('Programme storey {}: {} | band depth {:.2f} m | {}'.format(k, geo[k]['typology'], geo[k]['depth'], line))
    efficiency = []
    for k in range(storeys):
        nla_k = sum(pl[3] for pl in placements if pl[2] == k)
        slot_k = sum(sum(r[1]-r[0] for r in pl[5])*(geo[k]['depths'][pl[1]]+geo[k]['strip'][pl[1]]) for pl in placements if pl[2] == k)
        band_k = sum(b['end']-b['start'] for i, b in enumerate(bays) for side in geo[k]['sides'] if valid(i, side, k))
        used_k = sum(bays[i]['end']-bays[i]['start'] for (i, side, kk), v in occupied.items() if kk == k and v[0] in ('unit', 'common', 'top', 'core'))
        efficiency.append('{} {:.0%} (empty {:.1f} m, box beyond type area {:.0f} m2{})'.format(k, nla_k/footprint[k] if footprint[k] > 0 else 0.0, max(0.0, band_k-used_k), max(0.0, slot_k-nla_k),
                                                                                    ', deep units on side {}'.format(' and '.join(str(sd) for sd in (0, 1) if geo[k]['strip'][sd] > 1e-6)) if any(geo[k]['strip'][sd] > 1e-6 for sd in (0, 1)) else ''))
    out.append('EFFICIENCY per storey (NLA/GFA with NLA = type areas, band length with no apartment or common area, apartment box area beyond the type area because every box runs from the facade to the corridor, deep units = apartments deeper than the daylight depth, see DEEP UNITS): '+' | '.join(efficiency)+'.')
    deep = ['storey {} side {} {:.2f} m deep'.format(k, side, geo[k]['depths'][side]+geo[k]['strip'][side]) for k in range(storeys) for side in (0, 1) if geo[k]['strip'][side] > 1e-6]
    if deep:
        out.append('DEEP UNITS (REVIEW daylight): on overhang storeys the corridor stays where it is on the ground floor, so the band from the corridor to the overhanging facade is deeper than the {:.1f} m daylight depth; the apartments there run from the facade to the corridor (entrance side) and are deeper than the daylight depth: {}. Unit types are still chosen for {:.1f} m depth; the extra depth suits storage, bathrooms or a deeper living room - check daylight, or shorten the overhang.'.format(
            PROGRAMME_DAYLIGHT_DEPTH, '; '.join(deep), min(PROGRAMME_DAYLIGHT_DEPTH, APARTMENT_DEPTH or PROGRAMME_DAYLIGHT_DEPTH)))
    if missing:
        out.append('Types that could not be placed: '+', '.join(missing)+' (no bay or top-storey run wide enough).')
    if gen:
        out.append('APARTMENT_GEN {} ({}): requested vs designed: {}.{}{}'.format(
            gen, 'Type_mix = target share of units per type' if gen == 1 else 'Apt_Manual_Count = exact number of units per type',
            '; '.join('{} {} -> {}'.format(n, '{:.0%}'.format(target.get(n, 0.0)) if gen == 1 else target.get(n, 0),
                                          '{} ({:.0%})'.format(mix[n], mix[n]/float(units_total) if units_total else 0.0)) for n in names_order),
            ' Deviation: '+'; '.join(mix_off)+'.' if mix_off else ' Target met'+(' (every type within {:.0%} of its share).'.format(MIX_SHARE_TOLERANCE) if gen == 1 else ' exactly.'),
            ' Vacant slot area {:.1f} m2 (fewer units requested than the bays hold; left unallocated, the walls stay where the bays are).'.format(vacant_area) if vacant_area > 1e-6 else ''))
    small = [x for x in stack_types if x['area'] < PROGRAMME_MIN_FRONTAGE*typical_depth-1e-9]
    if small and typical_depth > 0:
        out.append('Unit types smaller than one minimum frontage x band depth ({:.1f} m x {:.2f} m = {:.1f} m2): {}; their boxes still run the full band depth (facade to corridor, so every unit has an entrance side), so the box is larger than the type area; Apartment_Depth shortens the bands.'.format(
            PROGRAMME_MIN_FRONTAGE, typical_depth, PROGRAMME_MIN_FRONTAGE*typical_depth, ', '.join('{} {:g} m2'.format(x['name'], x['area']) for x in small)))
    if excluded:
        out.append('Max_Unit_Width {:.2f} m: types left out because area / band depth ({:.2f} m) needs a wider unit: {}. Every placed unit is at most {:.2f} m wide (clear between the walls), so no frame line crosses an apartment when this is below the slab span.'.format(
            MAX_UNIT_WIDTH, typical_depth, ', '.join('{} ({:.2f} m)'.format(n, need(area_of_type[n], typical_depth)-t) for n in excluded), MAX_UNIT_WIDTH))
    mix_text = ('Mix design (APARTMENT_GEN {}): bay widths: every combination of apartment-type bay widths that fills each free segment (up to {} sets) plus area-knapsack variants re-weighted towards the target; the set whose filled slots come closest to the target is kept ({}); every slot (bay x side x storey) is then filled with 0-{} units side by side (one per slot where possible; several only where they fit the bay, separated by NON-load-bearing walls) by a local search that minimises the deviation from the target and then maximises area; top-only types ("Penthouse") on the top storey, {}; common area = {:.0%} of the designed apartment area on the ground floor; "every type at least one" is not applied.'.format(
        gen, BAY_SET_LIMIT, 'Type_mix: first every type within {:.0%} of its share, then the most apartment area, then the smallest total deviation'.format(MIX_SHARE_TOLERANCE) if gen == 1 else 'Apt_Manual_Count: smallest count deviation, then the most apartment area', MIX_MAX_UNITS_PER_SLOT, 'their count from the target' if gen else '', PROGRAMME_COMMON_RATIO) if gen else None)
    out += [
        mix_text or 'Mix design: per free segment between cores an exact knapsack (1 cm steps) chooses bay stacks, one type per stack over all storeys, maximising apartment area; bay width = max({:.1f} m, area / band depth) + wall allowance. Every type gets at least one unit (a dedicated bay is reserved first for a type no other bay is wide enough for; otherwise it is swapped into the slot that loses least area); one unit per top-only type ("Penthouse") on the top storey; common area = {:.0%} of the designed apartment area in ground-floor slots from the start (first band, then the second band if needed).'.format(
            PROGRAMME_MIN_FRONTAGE, PROGRAMME_COMMON_RATIO),
        ('{} unit(s) span several bays: one box per bay piece, ids A_storey_n.piece.'.format(spanning) if spanning else 'No unit spans several bays.')+
        (' {} unit(s) extended into overhang strips narrower than a unit (+{:.1f} m2; the frame line at the old gable then crosses the unit).'.format(stretched, stretched_area) if stretched else '')+
        (' Overhang units are separated from the unit inside by a NON-load-bearing wall on the old gable line{} {} (it stays a frame line: one storey overhanging does not make the gable a full-height party wall).'.format('s' if len(overhang_partitions) > 1 else '', ', '.join('{:.2f}'.format(x) for x in overhang_partitions)) if overhang_partitions else '')+
        (' {} extra unit(s) fill leftover slot length (forced small types, common area) — separated by NON-load-bearing walls because they do not stack.'.format(extras) if extras else '')+
        (' {} unit(s) are wider than the {:.2f} m slab span: a frame line (beam on columns in the facade and corridor walls) crosses each of them — no wall inside the unit.'.format(wide, max_bay) if max_bay and wide else ''),
        'Not modelled: end-facade units, balconies, external galleries, shafts, escape distances, lift rules, BBL'+('.' if gen else ', market or policy mix targets (set Apartment_GEN 1 or 2).'),
        'Programme_Data: '+PROGRAMME_DATA_HEADER+' (one row per storey).',
        'Apartment_Data: '+APARTMENT_DATA_HEADER+' | Support_Data: '+SUPPORT_DATA_HEADER]
    data = []
    for k in range(storeys):
        g = geo[k]
        band_length = sum(b['end']-b['start'] for i, b in enumerate(bays) for side in g['sides'] if valid(i, side, k))
        used = sum(bays[i]['end']-bays[i]['start'] for (i, side, kk) in occupied if kk == k)
        data.append('{},{:.3f},{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{},{:.3f}'.format(
            k, footprint[k], g['typology'], g['depth'], band_length, used, band_length-used,
            area_of['circulation', k], area_of['core', k], area_of['common', k],
            sum(placed[k].values()), sum(pl[3] for pl in placements if pl[2] == k)))
    apartment_rows = ['{},{},{},{:.3f},{:.3f},{:.3f},{},{},{:.3f}'.format(i, k, n, a, w, d, side, ','.join('{:.4f}'.format(v) for v in b), w*d)
                      for i, k, n, a, w, d, side, b in apartments]
    support_rows = ['{},{},{},{:.3f},{}'.format(i, kind, k, plan_area(b), ','.join('{:.4f}'.format(v) for v in b))
                    for i, k, kind, b in supports]
    issues = []
    if missing:
        issues.append('PROGRAMME INCOMPLETE: types not placed: {}.'.format(', '.join(missing)))
    if mix_off:
        issues.append('PROGRAMME MIX DEVIATION (APARTMENT_GEN {}): {}.'.format(gen, '; '.join(mix_off)))
    if excluded:
        issues.append('PROGRAMME LIMITED BY Max_Unit_Width {:.2f} m: {} not placed (they need a wider unit at {:.2f} m band depth).'.format(MAX_UNIT_WIDTH, ', '.join(excluded), typical_depth))
    if not common_ok:
        issues.append('PROGRAMME: common area {:.1f} m2 does not fit in a ground-floor bay run.'.format(common))
    layout = dict(apartments=[(a[0], a[1], a[7]) for a in apartments], apartment_rows=apartment_rows,
                  apartment_types=[a[2] for a in apartments], type_order=names_order,
                  supports=[(i, k, b) for i, k, kind, b in supports], support_rows=support_rows,
                  support_kinds=[kind for i, k, kind, b in supports], mix=dict(mix),
                  bays=bays, lines=lines, along=along, geo=geo, core_len=core_len, core_depth=g0['depth'], wall_t=t, end_t=te, extras=extras)
    return status, out, issues, data, layout


def programme_report(levels, columns, wall_thickness=0.0, max_bay=None, end_thickness=None):
    empty = dict(apartments=[], apartment_rows=[], apartment_types=[], type_order=[], supports=[], support_rows=[], support_kinds=[], bays=[], lines=[], mix={})
    try:
        programme = read_programme()
    except (ValueError, TypeError) as error:
        return 'INVALID_INPUT', ['Programme design: INVALID_INPUT - {}'.format(error)], ['PROGRAMME INPUT INVALID: {}'.format(error)], [], empty
    if programme is None:
        return 'NOT_DESIGNED_NO_PROGRAMME', ['Programme design: not run; connect AreaPerTypeOfApartment (List access, Integer hint); Apartment_Names optional (List access, Text hint).'], [], [], empty
    return programme_check(levels, columns, *programme, wall_thickness=wall_thickness, max_bay=max_bay, end_thickness=end_thickness)



def branch_items(items, keys, order):
    return [[item for item, key in zip(items, keys) if key == wanted] for wanted in order]


def to_tree(branches):
    tree = DataTree[System.Object]()
    for i, items in enumerate(branches):
        path = GH_Path(i)
        tree.EnsurePath(path)
        for item in items:
            tree.Add(item, path)
    return tree


# 7c. Load-bearing wall system

WALL_START = {'Concrete': (0.20, 0.25), 'Steel': (0.20, 0.25), 'Timber': (0.12, 0.16)}
WALL_MAX_CONCRETE = 0.60
CLT_THICKNESSES = (0.10, 0.12, 0.14, 0.16, 0.20, 0.24, 0.28, 0.32, 0.36, 0.40)
CLT_FC0K = 21000.0
CLT_E05 = 7.4e6
CLT_E_MEAN = 11.0e6
CLT_FMK = 24000.0
CLT_VERTICAL_SHARE = 0.6
CLT_DENSITY = 4.6
CLT_KMOD = 0.60
CLT_GAMMA_M = 1.25
CLT_KDEF = 0.80
CONCRETE_WALL_ALPHA = 0.8
WALL_KINDS = ('core', 'party', 'cantilever')
STOREY_SNAP = 0.02
STOREY_MIN_HEIGHT = 2.0
CANTILEVER_WALL_LEVER = 0.7
CANTILEVER_CHORD_SHARE = 0.2
CANTILEVER_MAX_STEEL = 0.04
FACADE_LOAD_DEFAULT = 1.5
FACADE_LOAD = FACADE_LOAD_DEFAULT
CORE_COUNT = None
EDGE_STRIP_WIDTHS = (0.50, 0.75, 1.00, 1.25, 1.50)
CLT_FVRK = 1100.0
TIE_BASE = 20.0
TIE_PER_STOREY = 4.0
TIE_MAX = 60.0
TIE_VERTICAL_MIN = 100.0
TARGET_CLEAR_HEIGHT_DEFAULT = 2.60
TARGET_CLEAR_HEIGHT = TARGET_CLEAR_HEIGHT_DEFAULT
BAND_BEAM_WIDTHS = (0.30, 0.45, 0.60, 0.75, 0.90, 1.05, 1.20, 1.35, 1.50, 1.65, 1.80)
WIND_AREA_VB0 = {1: 29.5, 2: 27.0, 3: 24.5}
WIND_TERRAIN = {'sea': (0.005, 1.0, 'zee/kust (category 0)'), 'open': (0.2, 4.0, 'onbebouwd (category II)'), 'built': (0.5, 7.0, 'bebouwd (category III)')}
WIND_TERRAIN_ALIASES = {'sea': 'sea', 'zee': 'sea', 'kust': 'sea', 'coast': 'sea', 'coastal': 'sea', '0': 'sea',
                        'open': 'open', 'onbebouwd': 'open', 'rural': 'open', 'ii': 'open', '2': 'open',
                        'built': 'built', 'bebouwd': 'built', 'urban': 'built', 'iii': 'built', '3': 'built'}
WIND_Z0_II = 0.05
AIR_DENSITY = 1.25
WIND_CPE_TABLE = ((1.0, 0.8, -0.5), (5.0, 0.8, -0.7))
WIND_CORRELATION = 0.85
WIND_DYNAMIC_HEIGHT = 50.0
WIND_DYNAMIC_SLENDERNESS = 5.0
SECOND_ORDER_XI = 7.8
SECOND_ORDER_LIMIT = 0.1
SECOND_ORDER_E_CONCRETE = 11.0e6
COUPLING_CRACKED = 0.5
COUPLING_STEPS = 200
COUPLING_SHARES = (1.0, 0.5, 0.25, 0.1, 0.0)
WIND_ECCENTRICITY = 0.10
DRIFT_RATIO = 500.0
PSI0_IMPOSED = 0.4
WIND_GRAVITY_FACTOR = 1.2*PERMANENT_SHARE+1.5*PSI0_IMPOSED*(1.0-PERMANENT_SHARE)
CLT_KMOD_WIND = 0.9
CONCRETE_E_CRACKED = 16.5e6
CLT_FV_PLANE = 2500.0
WALL_DATA_HEADER = 'wall_id,kind,storey,position_m,length_m,thickness_m,height_m,N_service_kN_per_m,NEd_kN_per_m,NRd_kN_per_m,U,material'


def wall_material(construction):
    return 'CLT' if construction == 'Timber' else 'Concrete'


def wall_thicknesses(construction, kind):
    start = WALL_START[construction][1 if kind == 'core' else 0]
    if wall_material(construction) == 'CLT':
        return [t for t in CLT_THICKNESSES if t >= start-1e-9]
    count = int(round((WALL_MAX_CONCRETE-start)/0.05))+1
    return [round(start+0.05*i, 10) for i in range(count)]


def wall_unit_weight(construction):
    return CLT_DENSITY if wall_material(construction) == 'CLT' else DENSITY


def wall_check(construction, thickness, height, service_load):
    ned = EDGE_ULS_FACTOR*service_load
    if wall_material(construction) == 'CLT':
        area = thickness*CLT_VERTICAL_SHARE
        inertia = thickness**3/12.0*CLT_VERTICAL_SHARE
        ncr = math.pi**2*CLT_E05*inertia/height**2
        slenderness = math.sqrt(area*CLT_FC0K/ncr)
        phi = 0.5*(1+0.2*(slenderness-0.3)+slenderness**2)
        kc = 1.0 if slenderness <= 0.3 else min(1.0, 1.0/(phi+math.sqrt(max(0.0, phi**2-slenderness**2))))
        nrd = kc*area*CLT_FC0K*CLT_KMOD/CLT_GAMMA_M
        return dict(NEd=ned, NRd=nrd, U=ned/nrd, slenderness=slenderness, factor=kc)
    fcd = CONCRETE_WALL_ALPHA*EDGE_FCK/EDGE_GAMMA_C*1000.0
    eccentricity = max(thickness/30.0, 0.02)+height/400.0
    phi = min(1.14*(1-2*eccentricity/thickness)-0.02*height/thickness, 1-2*eccentricity/thickness)
    nrd = thickness*fcd*max(0.0, phi)
    return dict(NEd=ned, NRd=nrd, U=ned/nrd if nrd > 0 else float('inf'), slenderness=height/thickness, factor=phi)


def slab_span_limit(construction, slab, q):
    q = max(q, 1e-9)
    if construction == 'Timber':
        ei = CLT_E_MEAN*0.85*slab**3/12.0/(1.0+CLT_KDEF)
        deflection = (384.0*ei/(5.0*300.0*q))**(1.0/3.0)
        mrd = CLT_FMK*CLT_KMOD/CLT_GAMMA_M*0.85*slab**2/6.0
        bending = math.sqrt(8.0*mrd/(EDGE_ULS_FACTOR*q))
        return min(deflection, bending), 'CLT slab: L/300 with creep and bending'
    fcd, fyd, fctm = rc_strengths()
    d = slab*1000.0-30.0
    rho0 = math.sqrt(EDGE_FCK)*1e-3
    rho = 0.005
    base = 11.0+1.5*math.sqrt(EDGE_FCK)*rho0/rho+3.2*math.sqrt(EDGE_FCK)*(rho0/rho-1.0)**1.5
    ratio = 1.3*base*d/1000.0
    mrd = 0.01*1000.0*d*fyd*(d-0.4*0.01*d*fyd/(0.8*fcd))/1e6
    bending = math.sqrt(8.0*mrd/(EDGE_ULS_FACTOR*q))
    return min(ratio, bending), 'concrete slab: EC2 7.4.2 span/depth (K=1.3, rho=0.5%) and bending with 1% steel'


def size_frame_column(construction, height, load, minimum):
    if construction == 'Concrete':
        side = up(max(0.30, minimum))
        while True:
            own = side*side*height*DENSITY
            check = rc_column_check(side, height, load+own)
            if check['U_N'] <= 1.0:
                return rc_section(side, side), check, own
            side = round(side+0.05, 10)
            if side > RC_MAX_COLUMN+1e-9:
                raise ValueError('Concrete frame-line column: no side up to {:.2f} m passes for {:.1f} kN.'.format(RC_MAX_COLUMN, load))
    weight = material_properties(construction)['weight']
    for section in material_catalog(construction, 'Column'):
        if section['b'] < minimum-1e-10:
            continue
        own = section['A']*height*weight
        check = compression_check(construction, section, height, load+own)
        if check['U_N'] <= 1.0:
            return section, check, own
    raise ValueError('{} frame-line column: no trial section passes for {:.1f} kN.'.format(construction, load))


def edge_strip_check(construction, width, slab, span, q, facade_line):
    w = q*width+facade_line
    med = EDGE_ULS_FACTOR*w*span**2/8.0
    ved = EDGE_ULS_FACTOR*w*span/2.0
    label = 'CLT' if construction == 'Timber' else 'RC'
    section = dict(b=width, h=slab, t=0.0, A=width*slab, Iy=width*slab**3/12.0, Iz=slab*width**3/12.0,
                   section='{} slab strip {:.0f}x{:.0f}'.format(label, width*1000, slab*1000))
    if construction == 'Timber':
        ei = CLT_E_MEAN*0.85*width*slab**3/12.0/(1.0+CLT_KDEF)
        mrd = CLT_FMK*CLT_KMOD/CLT_GAMMA_M*0.85*width*slab**2/6.0
        vrd = CLT_FVRK*CLT_KMOD/CLT_GAMMA_M*width*slab/1.5
        deflection = 5.0*w*span**4/(384.0*ei)
        limit = span/300.0
        check = dict(qSLS=w, MEd=med, MRd=mrd, VEd=ved, VRd=vrd, EI=ei, deflection_mm=1000*deflection, limit_mm=1000*limit,
                     U_M=med/mrd, U_V=ved/vrd, U_deflection=deflection/limit)
        return section, check
    fcd, fyd, fctm = rc_strengths()
    b = width*1000.0
    d = slab*1000.0-30.0
    x_lim = 0.45*d
    mrd = 0.8*x_lim*b*fcd*(d-0.4*x_lim)/1e6
    disc = d*d-2.0*med*1e6/(b*fcd)
    as_req = b*(d-math.sqrt(disc))*fcd/fyd if disc >= 0 else float('inf')
    as_prov = max(as_req, max(0.26*fctm/EDGE_FYK, 0.0013)*b*d)
    vrd = shear_resistance(EDGE_FCK, EDGE_GAMMA_C, b, d, min(as_prov, 0.02*b*d))
    rho0 = math.sqrt(EDGE_FCK)*1e-3
    rho = as_prov/(b*d) if as_prov < float('inf') else 1.0
    if rho <= rho0:
        base = 11.0+1.5*math.sqrt(EDGE_FCK)*rho0/rho+3.2*math.sqrt(EDGE_FCK)*(rho0/rho-1.0)**1.5
    else:
        base = 11.0+1.5*math.sqrt(EDGE_FCK)*rho0/rho
    limit = 1.3*base*min(1.0, 7.0/span)
    ratio = span*1000.0/d
    check = dict(qSLS=w, MEd=med, MRd=mrd, VEd=ved, VRd=vrd, As_req=as_req, As_prov=as_prov,
                 span_depth=ratio, span_depth_limit=limit, U_M=med/mrd, U_V=ved/vrd, U_deflection=ratio/limit,
                 U_As=as_prov/(0.04*b*slab*1000.0))
    return section, check


def select_edge_member(construction, span, q, facade_line, slab, height, tol):
    for width in EDGE_STRIP_WIDTHS:
        section, check = edge_strip_check(construction, width, slab, span, q, facade_line)
        if max(check['U_M'], check['U_V'], check['U_deflection'], check.get('U_As', 0.0)) <= 1.0:
            return dict(kind='concealed', section=section, check=check, material='CLT' if construction == 'Timber' else 'Concrete',
                        depth=slab, top=0.0, volume=0.0)
    line_load = q*EDGE_STRIP_WIDTHS[0]+facade_line
    if construction == 'Timber':
        section, check = select_material_beam('Timber', span, line_load, height-slab-tol)
        return dict(kind='downstand', section=section, check=check, material='Timber', depth=section['h'], top=slab,
                    volume=section['A']*span)
    width, depth, check = select_rc_beam(span, line_load, slab, height, tol, CONCRETE_BEAM_RATIO)
    return dict(kind='downstand', section=rc_section(width, depth), check=check, material='Concrete', depth=depth, top=0.0,
                volume=width*max(0.0, depth-slab)*span)


def select_frame_beam(construction, span, line_load, slab, height, tol):
    if construction == 'Concrete':
        best = None
        depth = up(slab+0.05)
        while depth < height-tol:
            for width in BAND_BEAM_WIDTHS:
                check = rc_beam_check(width, depth, slab, span, line_load)
                if max(check['U_M'], check['U_V'], check['U_deflection'], check['U_As']) <= 1.0:
                    clear_ok = height-depth >= TARGET_CLEAR_HEIGHT-1e-9
                    key = (0 if clear_ok else 1, width*max(0.0, depth-slab) if clear_ok else depth, width)
                    if best is None or key < best[0]:
                        best = (key, width, depth, check)
                    break
            depth = round(depth+0.05, 10)
        if best is None:
            raise ValueError('Concrete frame beam: no section up to storey height passes at span {:.2f} m.'.format(span))
        key, width, depth, check = best
        return rc_section(width, depth), check, 0.0, width*max(0.0, depth-slab)*span, DENSITY*width*max(0.0, depth-slab)
    weight = material_properties(construction)['weight']
    target = height-slab-TARGET_CLEAR_HEIGHT
    if target > 0.18:
        try:
            section, check = select_material_beam(construction, span, line_load, target+1e-6)
            return section, check, slab, section['A']*span, weight*section['A']
        except ValueError:
            pass
    cap = 0.20
    while cap < height-slab-tol+1e-9:
        try:
            section, check = select_material_beam(construction, span, line_load, cap)
            return section, check, slab, section['A']*span, weight*section['A']
        except ValueError:
            cap = round(cap+0.05, 10)
    section, check = select_material_beam(construction, span, line_load, height-slab-tol)
    return section, check, slab, section['A']*span, weight*section['A']


def wall_elements(rects, groups):
    out = []
    for name, members in groups:
        parts = [rects[m] for m in members]
        area = sum((r[1]-r[0])*(r[3]-r[2]) for r in parts)
        am = sum((r[1]-r[0])*(r[3]-r[2])*(r[0]+r[1])/2.0 for r in parts)/area
        cm = sum((r[1]-r[0])*(r[3]-r[2])*(r[2]+r[3])/2.0 for r in parts)/area
        ia = sum((r[3]-r[2])*(r[1]-r[0])**3/12.0+(r[1]-r[0])*(r[3]-r[2])*((r[0]+r[1])/2.0-am)**2 for r in parts)
        ic = sum((r[1]-r[0])*(r[3]-r[2])**3/12.0+(r[1]-r[0])*(r[3]-r[2])*((r[2]+r[3])/2.0-cm)**2 for r in parts)
        out.append(dict(name=name, A=area, a=am, c=cm, Ia=ia, Ic=ic,
                        web_a=sum((r[1]-r[0])*(r[3]-r[2]) for r in parts if r[1]-r[0] >= r[3]-r[2]),
                        web_c=sum((r[1]-r[0])*(r[3]-r[2]) for r in parts if r[3]-r[2] > r[1]-r[0]),
                        ea=max(max(abs(r[0]-am), abs(r[1]-am)) for r in parts), ec=max(max(abs(r[2]-cm), abs(r[3]-cm)) for r in parts),
                        depth_a=max(r[1] for r in parts)-min(r[0] for r in parts), depth_c=max(r[3] for r in parts)-min(r[2] for r in parts),
                        N=sum(r[4] for r in parts)))
    return out


def wind_terrain(value):
    key = str(value if value is not None else 'open').strip().strip('"').strip("'").lower()
    if key not in WIND_TERRAIN_ALIASES:
        raise ValueError('Wind_Terrain must be sea/coast (zee/kust), open (onbebouwd) or built (bebouwd).')
    return WIND_TERRAIN_ALIASES[key]


def peak_pressure(z, area, terrain):
    vb = WIND_AREA_VB0[area]
    z0, zmin, label = WIND_TERRAIN[terrain]
    kr = 0.19*(z0/WIND_Z0_II)**0.07
    zz = max(z, zmin)
    cr = kr*math.log(zz/z0)
    vm = cr*vb
    iv = 1.0/math.log(zz/z0)
    return (1.0+7.0*iv)*0.5*AIR_DENSITY*vm**2/1000.0


def wind_force_coefficient(height, depth):
    ratio = height/max(depth, 1e-6)
    table = WIND_CPE_TABLE
    if ratio <= table[0][0]:
        cd, ce = table[0][1], table[0][2]
    elif ratio >= table[-1][0]:
        cd, ce = table[-1][1], table[-1][2]
    else:
        for (r0, d0, e0), (r1, d1, e1) in zip(table, table[1:]):
            if r0 <= ratio <= r1:
                f = (ratio-r0)/(r1-r0)
                cd, ce = d0+f*(d1-d0), e0+f*(e1-e0)
                break
    corr = WIND_CORRELATION
    return (cd-ce)*corr, cd, ce, corr, ratio


def wind_slices(z0, z1, height, breadth):
    if height <= breadth:
        return [(z0, z1, height)]
    marks = (breadth,) if height <= 2.0*breadth else (breadth, height-breadth)
    cuts = sorted(set([z0, z1]+[c for c in marks if z0 < c < z1]))
    out = []
    for a, b in zip(cuts, cuts[1:]):
        mid = (a+b)/2.0
        if mid <= breadth:
            ze = breadth
        elif height <= 2.0*breadth or mid >= height-breadth:
            ze = height
        else:
            ze = b
        out.append((a, b, ze))
    return out


def storey_moment(shape, height, s):
    return sum(f*(z-s) for z, f in shape if z > s+1e-12)


def coupling_solution(a1, a2, i1, i2, lever, clear, storey_h, ic_beam, shape, height):
    inertia = i1+i2
    if ic_beam <= 0 or lever <= 0 or clear <= 0 or inertia <= 0:
        return dict(eta=0.0, factor=1.0, v_per_m=0.0)
    alpha2 = 12.0*ic_beam*lever**2/(storey_h*clear**3*inertia)
    k2 = 1.0+(a1+a2)*inertia/(a1*a2*lever**2)
    big = k2*alpha2
    n = COUPLING_STEPS
    dx = height/n
    xs = [dx*j for j in range(n+1)]
    moment = [storey_moment(shape, height, height-x) for x in xs]
    lower, diag, upper, rhs = [0.0]*(n+1), [0.0]*(n+1), [0.0]*(n+1), [0.0]*(n+1)
    diag[0], rhs[0] = 1.0, 0.0
    for j in range(1, n):
        lower[j], diag[j], upper[j] = 1.0/dx**2, -2.0/dx**2-big, 1.0/dx**2
        rhs[j] = -alpha2*moment[j]/lever
    lower[n], diag[n], rhs[n] = 2.0/dx**2, -2.0/dx**2-big, -alpha2*moment[n]/lever
    for j in range(1, n+1):
        w = lower[j]/diag[j-1]
        diag[j] -= w*upper[j-1]
        rhs[j] -= w*rhs[j-1]
    t = [0.0]*(n+1)
    t[n] = rhs[n]/diag[n]
    for j in range(n-1, -1, -1):
        t[j] = (rhs[j]-upper[j]*t[j+1])/diag[j]
    base = moment[n]
    if base <= 0:
        return dict(eta=0.0, factor=1.0, v_per_m=0.0)
    free = sum(moment[j]*xs[j] for j in range(n+1))
    coupled = sum((moment[j]-t[j]*lever)*xs[j] for j in range(n+1))
    flow = max(abs(t[j+1]-t[j])/dx for j in range(n))
    return dict(eta=t[n]*lever/base, factor=free/coupled if coupled > 1e-12 else 1.0, v_per_m=flow*storey_h/base)


def lateral_check(levels, elements, material, n_total, members, extents=None, wind=(2, 'open'), faces=None):
    base = levels[0]
    height = levels[-1][5]-levels[0][4]
    a0, a1, c0, c1 = extents if extents else (base[0], base[1], base[2], base[3])
    widths = {'a': c1-c0, 'c': a1-a0}
    depths = {'a': a1-a0, 'c': c1-c0}
    centre = {'a': (a0+a1)/2.0, 'c': (c0+c1)/2.0}
    if faces is None:
        faces = [(p[4]-base[4], p[5]-base[4], c1-c0, a1-a0) for p in levels]
    area, terrain = wind
    coefficients = dict((d, wind_force_coefficient(height, depths[d])) for d in ('a', 'c'))
    breadths = {'a': max(f[2] for f in faces), 'c': max(f[3] for f in faces)}
    def wind_load(direction):
        force, moment, shape = 0.0, 0.0, []
        cf = coefficients[direction][0]
        for z0, z1, width_a, width_c in faces:
            for s0, s1, ze in wind_slices(z0, z1, height, breadths[direction]):
                f = cf*peak_pressure(ze, area, terrain)*(width_a if direction == 'a' else width_c)*(s1-s0)
                force += f
                moment += f*(s0+s1)/2.0
                shape.append(((s0+s1)/2.0, f))
        return force, moment, shape
    alpha_h = min(1.0, max(2.0/3.0, 2.0/math.sqrt(height)))
    alpha_m = math.sqrt(0.5*(1.0+1.0/max(1, members)))
    theta = alpha_h*alpha_m/200.0
    notional = theta*WIND_GRAVITY_FACTOR*n_total
    if material == 'CLT':
        fc = CLT_FC0K*CLT_KMOD_WIND/CLT_GAMMA_M*CLT_VERTICAL_SHARE
        fv = CLT_FV_PLANE*CLT_KMOD_WIND/CLT_GAMMA_M
        stiffness = CLT_E_MEAN*CLT_VERTICAL_SHARE
        design_e = stiffness/CLT_GAMMA_M
    else:
        fcd, fyd, fctm = rc_strengths()
        fc = fcd*1000.0
        fv = 0.5*0.6*(1.0-EDGE_FCK/250.0)*fcd*1000.0*0.9
        stiffness = CONCRETE_E_CRACKED
        design_e = SECOND_ORDER_E_CONCRETE
    state = dict((e['name'], 0) for e in elements if e.get('coupled'))
    def inertia(e, key):
        if key == 'Ic' and e.get('coupled'):
            return e['Ic_free']*e['coupled']['options'][state[e['name']]]['factor']
        return e[key]
    def evaluate():
        results, worst = {}, 0.0
        sum_ia = sum(inertia(e, 'Ia') for e in elements)
        sum_ic = sum(inertia(e, 'Ic') for e in elements)
        if sum_ia <= 0 or sum_ic <= 0:
            return None
        c_s = sum(inertia(e, 'Ia')*e['c'] for e in elements)/sum_ia
        a_s = sum(inertia(e, 'Ic')*e['a'] for e in elements)/sum_ic
        torsion_stiffness = sum(inertia(e, 'Ia')*(e['c']-c_s)**2 for e in elements)+sum(inertia(e, 'Ic')*(e['a']-a_s)**2 for e in elements)
        per = dict((e['name'], dict(U=0.0, tension=0.0, M=[], lintel_U=0.0)) for e in elements)
        for direction in ('a', 'c'):
            wind_f, wind_moment, shape = wind_load(direction)
            total = sum_ia if direction == 'a' else sum_ic
            vertical = WIND_GRAVITY_FACTOR*n_total
            critical = SECOND_ORDER_XI*design_e*total/height**2
            ratio = vertical/critical if critical > 0 else float('inf')
            amp = 1.0 if ratio <= SECOND_ORDER_LIMIT else (1.0/(1.0-ratio) if ratio < 1.0 else float('inf'))
            force = (WIND_ULS_FACTOR*wind_f+notional)*amp
            moment = (WIND_ULS_FACTOR*wind_moment+notional*height/2.0)*amp
            own, other = ('Ia', 'Ic') if direction == 'a' else ('Ic', 'Ia')
            lever_key, centre_s = ('c', c_s) if direction == 'a' else ('a', a_s)
            ecc = abs(centre[lever_key]-centre_s)+WIND_ECCENTRICITY*widths[direction]
            torque = force*ecc
            drift = wind_f/height*height**4/(8.0*stiffness*total)
            u_drift = drift/(height/DRIFT_RATIO)
            worst_here = max(u_drift, 0.0 if amp != float('inf') else float('inf'))
            for e in elements:
                i_own, i_other = inertia(e, own), inertia(e, other)
                v_main = force*i_own/total+(torque*i_own*abs(e[lever_key]-centre_s)/torsion_stiffness if torsion_stiffness > 0 else 0.0)
                v_side = torque*i_other*abs(e['a' if direction == 'c' else 'c']-(a_s if direction == 'c' else c_s))/torsion_stiffness if torsion_stiffness > 0 else 0.0
                m_main, m_side = (moment*v_main/force, moment*v_side/force) if force > 0 else (0.0, 0.0)
                w_side = i_other/(e['ec'] if direction == 'a' else e['ea']) if i_other > 0 else 0.0
                side_bend = m_side/w_side if w_side > 0 else 0.0
                if e.get('coupled') and direction == 'c':
                    cp = e['coupled']
                    opt = cp['options'][state[e['name']]]
                    pull = opt['eta']*m_main/cp['lever']
                    u, tension = 0.0, 0.0
                    for part in cp['parts']:
                        m_part = (1.0-opt['eta'])*m_main*part['I']/cp['I']
                        bend = m_part*part['e']/part['I']+side_bend
                        sigma_max = (WIND_GRAVITY_FACTOR*part['N']+pull)/part['A']+bend
                        sigma_min = (0.9*PERMANENT_SHARE*part['N']-pull)/part['A']-bend
                        shear = v_main*part['I']/cp['I']/part['A']
                        u = max(u, sigma_max/fc, shear/fv)
                        if sigma_min < 0:
                            tension = max(tension, m_part/(0.8*part['L'])+pull/2.0-0.9*PERMANENT_SHARE*part['N']/2.0)
                    v_lintel = opt['v_per_m']*m_main
                    lintel_u = max(v_lintel*cp['clear']/2.0/cp['MRd'], (cp['Vg']+v_lintel)/cp['VRd']) if opt['v_per_m'] > 0 else 0.0
                    per[e['name']]['lintel_U'] = lintel_u
                    per[e['name']]['coupling'] = dict(eta=opt['eta'], factor=opt['factor'], share=opt['share'], lintel_U=lintel_u, lintel_V=v_lintel)
                    u = max(u, lintel_u)
                    tension = max(0.0, tension)
                else:
                    w_main = i_own/(e['ea'] if direction == 'a' else e['ec']) if i_own > 0 else 0.0
                    bend = (m_main/w_main if w_main > 0 else 0.0)+side_bend
                    sigma_max = WIND_GRAVITY_FACTOR*e['N']/e['A']+bend
                    sigma_min = 0.9*PERMANENT_SHARE*e['N']/e['A']-bend
                    web_main = e['web_a'] if direction == 'a' else e['web_c']
                    web_side = e['web_c'] if direction == 'a' else e['web_a']
                    shear = max(v_main/(web_main or e['A']), v_side/(web_side or e['A']))
                    depth = e['depth_a'] if direction == 'a' else e['depth_c']
                    tension = max(0.0, m_main/(0.8*depth)-0.9*PERMANENT_SHARE*e['N']/2.0) if depth > 0 and sigma_min < 0 else 0.0
                    u = max(sigma_max/fc, shear/fv)
                per[e['name']]['U'] = max(per[e['name']]['U'], u)
                per[e['name']]['tension'] = max(per[e['name']]['tension'], tension)
                per[e['name']]['M'].append((m_main, m_side) if direction == 'a' else (m_side, m_main))
                worst_here = max(worst_here, u)
            worst = max(worst, worst_here)
            results[direction] = dict(force=force, wind=wind_f, notional=notional, ecc=ecc, drift=drift, u_drift=u_drift, worst=worst_here,
                                      cf=coefficients[direction][0], cpe_d=coefficients[direction][1], cpe_e=coefficients[direction][2],
                                      corr=coefficients[direction][3], h_d=coefficients[direction][4], fv_ratio=ratio, amp=amp)
        return results, worst, per, c_s, a_s
    while True:
        out = evaluate()
        if out is None:
            return dict(status='FAIL', worst=float('inf'), lines=['LATERAL STABILITY: no wall resists one of the directions.'], per={})
        results, worst, per, c_s, a_s = out
        moved = False
        for e in elements:
            if e.get('coupled') and per[e['name']]['lintel_U'] > 1.0+1e-9 and state[e['name']]+1 < len(e['coupled']['options']):
                state[e['name']] += 1
                moved = True
        if not moved:
            break
    status = 'PASS_ASSUMED_MODEL' if worst <= 1.0+1e-9 else 'FAIL'
    dynamic = [d for d in ('a', 'c') if height >= WIND_DYNAMIC_HEIGHT or height/max(sum(f[2 if d == 'a' else 3]*(f[1]-f[0]) for f in faces)/height, 1e-6) >= WIND_DYNAMIC_SLENDERNESS]
    return dict(status=status, worst=worst, results=results, per=per, theta=theta, c_s=c_s, a_s=a_s, height=height, dynamic=dynamic,
                qp_top=peak_pressure(height, area, terrain), qp_10=peak_pressure(10.0, area, terrain), wind=wind)


def stability_report(stability, wind, span_dir, cross_dir, material, grown, elements_text, extra_fail=''):
    def name(d):
        return span_dir if d == 'a' else cross_dir
    res = stability.get('results', {})
    lines = ['LATERAL STABILITY: {} | stability elements: {}. Wind NEN-EN 1991-1-4 + NB: wind area {} (vb,0 {:.1f} m/s), terrain {} (z0 {:.3f} m, zmin {:.0f} m); qp(ze) = (1 + 7 Iv) 0.5 rho vm(ze)^2 with the reference heights of EN 1991-1-4 Figure 7.4 (h <= b: ze = h over the whole face; b < h <= 2b: ze = b below b, h above; h > 2b: b at the base, h in the top b, storey tops between): {:.2f} kN/m2 at 10 m, {:.2f} kN/m2 at the roof ({:.1f} m); force = cf x qp x storey face area with cf = (cpe,D - cpe,E) x 0.85 (NEN-EN 1991-1-4 NB:2011 table NB.6 - 7.1, h/d <= 1: D +0.8, E -0.5; h/d 5: E -0.7, linear between; 7.2.2(4) lack of correlation 0.85){}; asymmetric eccentricity b/{:.0f} (7.1.2); notional theta_i = {:.5f} (EN 1992-1-1 5.2) on {:.2f} x total gravity (floors, walls, columns, beams, facades); ULS 1.5 wind + notional; global second order EN 1992-1-1 Annex H (FV,BB = {:.1f} E I / H2, ignored when FV,Ed/FV,BB <= {:.1f}, otherwise forces x 1/(1 - FV,Ed/FV,BB)).'.format(
        stability['status'], elements_text, wind[0], WIND_AREA_VB0[wind[0]], WIND_TERRAIN[wind[1]][2], WIND_TERRAIN[wind[1]][0], WIND_TERRAIN[wind[1]][1],
        stability.get('qp_10', 0.0), stability.get('qp_top', 0.0), stability.get('height', 0.0),
        ''.join('; along local {} h/d {:.2f}: D {:+.2f}, E {:+.2f}, factor {:.2f} -> cf {:.2f}'.format(name(d), r['h_d'], r['cpe_d'], r['cpe_e'], r['corr'], r['cf']) for d, r in sorted(res.items()))
        +('; cs cd = 1 is NOT allowed along local {} (NB 6.1: h >= 50 m or h/b >= 5) — determine cs cd with annex C (>= 0.85), not done here'.format(', '.join(name(d) for d in stability.get('dynamic', []))) if stability.get('dynamic') else '; cs cd = 1 (NB 6.1: h < 50 m and h/b < 5)'),
        1.0/WIND_ECCENTRICITY, stability.get('theta', 0.0), WIND_GRAVITY_FACTOR, SECOND_ORDER_XI, SECOND_ORDER_LIMIT)]
    lines += ['  along local {}: base shear {:.0f} kN (wind {:.0f} service), FV,Ed/FV,BB {:.3f}{}, eccentricity to stiffness centre {:.2f} m, governing element U {:.2f}, wind drift {:.1f} mm vs H/{:.0f} = {:.1f} mm.'.format(
        name(d), r['force'], r['wind'], r['fv_ratio'], ' (second order ignored)' if r['amp'] == 1.0 else ' -> second-order factor {:.3f}'.format(r['amp']),
        r['ecc'], r['worst'], 1000*r['drift'], DRIFT_RATIO, 1000*stability['height']/DRIFT_RATIO) for d, r in sorted(res.items())]
    per = stability.get('per', {})
    lines.append('  elements: '+'; '.join('{} U {:.2f}{}{}'.format(n, v['U'],
        ' (coupled by the lintels: {:.0%} of the overturning as push-pull, lintel stiffness used {:.0%}, lintel U {:.2f})'.format(v['coupling']['eta'], v['coupling']['share'], v['coupling']['lintel_U']) if v.get('coupling') else '',
        ', tension {:.0f} kN ({})'.format(v['tension'], 'hold-downs' if material == 'CLT' else '{:.0f} mm2 end bars'.format(v['tension']*1000/435.0)) if v['tension'] > 0 else '') for n, v in sorted(per.items())))
    failing = [d for d in ('a', 'c') if res.get(d, {}).get('worst', 0.0) > 1.0+1e-9]
    lines.append(('  walls thickened for stability (one wall at a time, the one giving the largest utilisation drop per m3 added; stiffness centre and torsion recomputed each step): '+', '.join(grown)+'.' if grown else '  no wall thickened for stability.')
                 +(' STILL FAILS along local {}{}.'.format(', '.join(name(d) for d in failing), extra_fail) if stability['status'] != 'PASS_ASSUMED_MODEL' else ''))
    lines.append('  Assumptions: cdir = cseason = 1 (NB), cprob = c0 = 1 (flat terrain, 50-year return), vb,0 NB table NB.1, z0/zmin NB table NB.3 - 4.1 (qp equals NB table NB.5), rho 1.25 kg/m3, cpe,10 NB table NB.6 - 7.1 with ze per figure 7.4 on all faces (NB 7.2.2(1)), kr = 0.19 (z0/0.05)^0.07; elastic uncracked stresses for strength, cracked E for drift (concrete {:.1f} GPa, CLT {:.1f} GPa vertical layers, connection slip ignored), second order with E {:.1f} GPa (concrete 0.4 Ecd) or CLT E/gammaM; favourable gravity 0.9 x {:.0%} permanent share; shear vs VRd,max with horizontal bars (concrete) or CLT in-plane fv,k {:.1f} MPa. Coupled walls (concrete party-wall pairs over the corridor, monolithic RC lintels): continuous-connection method (Stafford Smith & Coull), lintel I = {:.0%} of gross (cracked), lintel stiffness reduced in steps until the lintel resists the coupling shear (MRd at x/d 0.45 hogging = sagging, VRd,max; coupling bars and links not designed); steel and timber lintels are pinned (no coupling). Overturning goes into the piles (FOUNDATION wind case). Not checked: openings in stability walls, shear deformation of the lintels, dynamic response.'.format(
        CONCRETE_E_CRACKED/1e6, CLT_E_MEAN*CLT_VERTICAL_SHARE/1e6, SECOND_ORDER_E_CONCRETE/1e6, PERMANENT_SHARE, CLT_FV_PLANE/1000.0, COUPLING_CRACKED))
    return lines


def select_cantilever(construction, length, w, tip, slab, height, tol):
    span, line_load = 2.0*length, w+2.0*tip/length
    if construction == 'Concrete':
        width, depth, check = select_rc_beam(span, line_load, slab, height, tol, CONCRETE_BEAM_RATIO)
        return rc_section(width, depth), check, 0.0, width*max(0.0, depth-slab)*length
    section, check = select_material_beam(construction, span, line_load, height-slab-tol)
    return section, check, slab, section['A']*length


def select_cantilever_wall(construction, length, moment, shear, height):
    material = wall_material(construction)
    z = CANTILEVER_WALL_LEVER*height
    best = None
    for t in wall_thicknesses(construction, 'party'):
        own = t*height*length*wall_unit_weight(construction)
        m = ULS_GRAVITY*(moment+own*length/2.0)
        v = ULS_GRAVITY*(shear+own)
        if material == 'Concrete':
            tension = m/z
            steel = tension*1000.0/(EDGE_FYK/EDGE_GAMMA_S)
            chord = t*CANTILEVER_CHORD_SHARE*height*1e6
            u_t = steel/(CANTILEVER_MAX_STEEL*chord)
            fcd = EDGE_ALPHA_CC*EDGE_FCK/EDGE_GAMMA_C
            vrd = 0.5*0.6*(1.0-EDGE_FCK/250.0)*fcd*1000.0*t*z
            u_v = v/vrd
        else:
            tension = m/z
            steel = 0.0
            u_t = m/((1.0-CLT_VERTICAL_SHARE)*t*height**2/6.0*CLT_KMOD*CLT_FMK/CLT_GAMMA_M)
            u_v = 1.5*v/(t*height)/(CLT_KMOD*CLT_FV_PLANE/CLT_GAMMA_M)
        res = dict(t=t, own=own, M=m, V=v, tension=tension, steel=steel, U=max(u_t, u_v), material=material)
        best = res
        if res['U'] <= 1.0:
            break
    return best


def tie_design(levels, spans, segment_lengths, q, wall_t, construction):
    storeys = len(levels)
    clear = min(p[5]-p[4] for p in levels)
    ft = min(TIE_BASE+TIE_PER_STOREY*storeys, TIE_MAX)
    along = max(ft, ft*q/7.5*min(5.0*clear, max(spans))/5.0)
    across = max(ft, ft*q/7.5*min(5.0*clear, max(segment_lengths))/5.0)
    vertical = max(TIE_VERTICAL_MIN, 34.0*wall_t*1e6/8000.0*(clear/wall_t)**2/1000.0)
    cls = '2A' if storeys <= 4 else ('2B' if storeys <= 15 else '3')
    return dict(ft=ft, along=along, across=across, peripheral=ft, vertical=vertical if cls != '2A' else 0.0, cls=cls)


def iterate_wall_design(levels, construction, floor, roof, slab, tol, layout_for, wind=(2, 'open'), piles=None):
    layout, system, allowance = iterate_wall_pass(levels, construction, floor, roof, slab, tol, layout_for, wind, piles, 0.0)
    if system is not None:
        b = system['balance']
        extra = b['columns']+b['frame_beams']+b['facade']+b['members']
        if extra > 0:
            layout, system, allowance = iterate_wall_pass(levels, construction, floor, roof, slab, tol, layout_for, wind, piles, extra)
    return layout, system, allowance


def iterate_wall_pass(levels, construction, floor, roof, slab, tol, layout_for, wind, piles, extra_gravity):
    allowance = (WALL_START[construction][0], WALL_START[construction][0])
    max_bay = slab_span_limit(construction, slab, max(floor, roof))[0]
    layout, system = None, None
    for attempt in range(6):
        layout = layout_for(allowance[0], max_bay, allowance[1])
        if not layout['bays']:
            return layout, None, allowance
        system = design_wall_system(levels, layout, construction, floor, roof, slab, tol, wind, piles, extra_gravity)
        needed = system['allowance']
        if needed[0] <= allowance[0]+1e-9 and needed[1] <= allowance[1]+1e-9:
            break
        allowance = (max(allowance[0], needed[0]), max(allowance[1], needed[1]))
    system['lines'].insert(2, 'Layout allowances per line type (iterated until every member fits): party-wall lines {:.2f} m, building-end lines {:.2f} m (core wall / end-frame column); slab-span splits inside units need none.'.format(*allowance))
    return layout, system, allowance


def design_wall_system(levels, layout, construction, floor, roof, slab, tol, wind=(2, 'open'), piles=None, extra_gravity=0.0):
    piles = piles or dict(diameter=None, capacity=None, peil=PEIL_NAP_DEFAULT, tip=PILE_TIP_NAP_DEFAULT)
    along, geo_full = layout['along'], layout['geo']
    storeys = len(levels)
    geo = []
    for k, gf in enumerate(geo_full):
        g = dict(gf)
        if k:
            sb = geo[k-1]
            g['a0'], g['a1'] = max(gf['a0'], sb['a0']), min(gf['a1'], sb['a1'])
            g['c0'], g['c1'] = max(gf['c0'], sb['c0']), min(gf['c1'], sb['c1'])
            if any(abs(g[key]-gf[key]) > 1e-9 for key in ('a0', 'a1', 'c0', 'c1')) or gf.get('anchored'):
                if g['typology'] == 'DOUBLE_LOADED_CORRIDOR':
                    if g['c0'] <= geo[0]['zone'][0]+1e-6 and g['c1'] >= geo[0]['zone'][1]-1e-6:
                        g['zone'] = geo[0]['zone']
                    g['depth'] = max(0.0, min(g['zone'][0]-g['c0'], g['c1']-g['zone'][1]))
                else:
                    g['zone'] = (g['c1']-PROGRAMME_CORRIDOR_WIDTH, g['c1'])
                    g['depth'] = max(0.0, min(PROGRAMME_DAYLIGHT_DEPTH, g['c1']-g['c0']-PROGRAMME_CORRIDOR_WIDTH))
        geo.append(g)
    g0 = geo[0]
    q_design = max(floor, roof)
    limit, limit_basis = slab_span_limit(construction, slab, q_design)
    material = wall_material(construction)
    unit_weight = wall_unit_weight(construction)
    corridor = PROGRAMME_CORRIDOR_WIDTH
    positions = unique([p for p, kind in layout['lines']], tol)
    kind_at = dict((round(p, 6), kind) for p, kind in layout['lines'])
    line_kind = [kind_at.get(round(p, 6), 'frame') for p in positions]
    spans = [b-a for a, b in zip(positions, positions[1:])]
    span_dir = 'X' if along == 0 else 'Y'
    cross_dir = 'Y' if along == 0 else 'X'
    def plan(a0, a1, c0, c1, z0, z1):
        return plan_box(along, a0, a1, c0, c1, z0, z1)
    def segments_of(g):
        if g['depth'] <= 0:
            return [], None, 0.0
        if g['typology'] == 'DOUBLE_LOADED_CORRIDOR':
            lane = (g['zone'][0], g['zone'][1])
            return [(g['c0'], lane[0]), (lane[1], g['c1'])], lane, 0.0
        return [(g['c0'], g['c1']-corridor)], None, corridor
    def cross_index(s, end):
        return 2*s+end
    def along_extent(i, g, width):
        lo, hi = positions[i]-width/2.0, positions[i]+width/2.0
        inside = valid_lines(g) or [i]
        if i == inside[0] and abs(positions[i]-g['a0']) < 1.0:
            lo, hi = g['a0'], g['a0']+width
        if i == inside[-1] and abs(g['a1']-positions[i]) < 1.0:
            lo, hi = g['a1']-width, g['a1']
        if lo < g['a0']:
            lo, hi = g['a0'], g['a0']+width
        if hi > g['a1']:
            lo, hi = g['a1']-width, g['a1']
        return lo, hi
    def valid_lines(g):
        return [i for i, pos in enumerate(positions) if g['a0']-1e-6 <= pos <= g['a1']+1e-6]
    def tribs(valid, g=None):
        out = {}
        for u, i in enumerate(valid):
            left = (positions[valid[u-1]]+positions[i])/2.0 if u > 0 else (min(g['a0'], positions[i]) if g else positions[i])
            right = (positions[valid[u+1]]+positions[i])/2.0 if u+1 < len(valid) else (max(g['a1'], positions[i]) if g else positions[i])
            out[i] = right-left
        return out
    def facade_line(k):
        return FACADE_LOAD*(levels[k+1][5]-levels[k+1][4]) if k+1 < storeys else 0.0
    def upper(k):
        return geo[k+1] if k+1 < storeys and geo[k+1]['depth'] >= 0 else None
    def long_facade(k, i, e):
        u = upper(k)
        if u is None or not (u['a0']-1e-6 <= positions[i] <= u['a1']+1e-6):
            return 0.0
        uf = geo_full[k+1]
        hit = (abs(e-u['c0']) < 1e-6 and abs(u['c0']-uf['c0']) < 1e-6) or (abs(e-u['c1']) < 1e-6 and abs(u['c1']-uf['c1']) < 1e-6)
        return facade_line(k) if hit else 0.0
    def gable_line(k, g, i, valid):
        u = upper(k)
        if u is None:
            return 0.0
        uf = geo_full[k+1]
        edges = [edge for edge, full in ((u['a0'], uf['a0']), (u['a1'], uf['a1'])) if abs(edge-full) < 1e-6]
        ends = set(min(valid, key=lambda j: abs(positions[j]-edge)) for edge in edges)
        return facade_line(k) if i in ends else 0.0
    def end_load(k, g, e, q, trib, lane, cantilever, gable=0.0, i=None):
        if abs(e-g['c0']) < 1e-6 or (cantilever == 0 and abs(e-g['c1']) < 1e-6):
            return long_facade(k, i, e)*trib+edge_own_at(k, i)
        if lane is not None and (abs(e-lane[0]) < 1e-6 or abs(e-lane[1]) < 1e-6):
            return (q_at(k, i, lane[0], lane[1])*trib+gable+lintel_for(k, i)['own'])*(lane[1]-lane[0])/2.0
        if cantilever > 0 and abs(e-(g['c1']-cantilever)) < 1e-6:
            return q_at(k, i, g['c1']-cantilever, g['c1'])*trib*cantilever+long_facade(k, i, g['c1'])*trib+gallery_for(k, i)['own']+edge_own_at(k, i)
        return 0.0
    valid_cache = {}
    def valid_of(k):
        if k not in valid_cache:
            valid_cache[k] = valid_lines(geo[k])
        return valid_cache[k]
    def trib_range(k, i):
        g, valid = geo[k], valid_of(k)
        u = valid.index(i)
        left = (positions[valid[u-1]]+positions[i])/2.0 if u > 0 else min(g['a0'], positions[i])
        right = (positions[valid[u+1]]+positions[i])/2.0 if u+1 < len(valid) else max(g['a1'], positions[i])
        return left, right
    q_cache = {}
    def q_at(k, i, c0, c1):
        key = (k, i, round(c0, 6), round(c1, 6))
        if key not in q_cache:
            u = geo[k+1] if k+1 < storeys else None
            if u is None:
                q_cache[key] = roof
            else:
                left, right = trib_range(k, i)
                fa = max(0.0, min(right, u['a1'])-max(left, u['a0']))/(right-left) if right > left+1e-9 else (1.0 if u['a0']-1e-6 <= positions[i] <= u['a1']+1e-6 else 0.0)
                fc = max(0.0, min(c1, u['c1'])-max(c0, u['c0']))/(c1-c0) if c1 > c0+1e-9 else 1.0
                q_cache[key] = roof+(floor-roof)*fa*fc
        return q_cache[key]
    def member_weight(material_name, volume):
        if material_name == 'Concrete':
            return DENSITY*volume
        if material_name == 'CLT':
            return CLT_DENSITY*volume
        return material_properties(material_name)['weight']*volume
    def beam_for(span, line_load, height, material=None):
        material = material or construction
        if material == 'Concrete':
            width, depth, check = select_rc_beam(span, line_load, slab, height, tol, CONCRETE_BEAM_RATIO)
            return rc_section(width, depth), check, 0.0, width*max(0.0, depth-slab)*span, DENSITY*width*max(0.0, depth-slab)
        section, check, reason = select_frame_beam_member(material, span, line_load, height-slab-tol, (), FRAME_TIE_MIN)
        return section, check, slab, section['A']*span, material_properties(material)['weight']*section['A']
    lintel_cache = {}
    def lintel_for(k, i):
        if (k, i) not in lintel_cache:
            g, p = geo[k], levels[k]
            lane = segments_of(g)[1]
            span = lane[1]-lane[0]
            load = q_at(k, i, lane[0], lane[1])*tribs(valid_of(k), g)[i]
            section, check, drop, volume, own_line = beam_for(span, load, p[5]-p[4])
            lintel_cache[k, i] = dict(section=section, check=check, drop=drop, volume=volume, own=own_line, span=span, load=load)
        return lintel_cache[k, i]
    gallery_cache = {}
    def gallery_for(k, i):
        if (k, i) not in gallery_cache:
            g, p = geo[k], levels[k]
            cantilever = segments_of(g)[2]
            trib_i = tribs(valid_of(k), g)[i]
            section, check, drop, volume = select_cantilever(construction, cantilever, q_at(k, i, g['c1']-cantilever, g['c1'])*trib_i, long_facade(k, i, g['c1'])*trib_i, slab, p[5]-p[4], tol)
            gallery_cache[k, i] = dict(section=section, check=check, drop=drop, volume=volume, own=member_weight(construction, volume))
        return gallery_cache[k, i]
    edge_cache = {}
    def edge_for(k, u):
        if (k, u) not in edge_cache:
            g, p = geo[k], levels[k]
            valid = valid_of(k)
            i, j = valid[u], valid[u+1]
            span = positions[j]-positions[i]
            q_edge = roof if k == storeys-1 else q_design
            member = select_edge_member(construction, span, q_edge, max(long_facade(k, i, g['c0']), long_facade(k, j, g['c0']), long_facade(k, i, g['c1']), long_facade(k, j, g['c1'])), slab, p[5]-p[4], tol)
            edge_cache[k, u] = dict(member=member, span=span, own=member_weight(member['material'], member['volume'])/span if span > 0 else 0.0)
        return edge_cache[k, u]
    def edge_own_at(k, i):
        valid = valid_of(k)
        if i not in valid:
            return 0.0
        u = valid.index(i)
        total = 0.0
        for v in (u-1, u):
            if 0 <= v < len(valid)-1:
                e = edge_for(k, v)
                total += e['own']*e['span']/2.0
        return total
    core_boxes = [b for b in layout['bays'] if b['kind'] == 'core']
    core_depth = layout['core_depth']
    def nearest(v):
        return min(range(len(positions)), key=lambda j: abs(positions[j]-v))
    core_cover = set()
    for b in core_boxes:
        core_cover.update(((nearest(b['start']), 0), (nearest(b['end']), 0)))
    def walled(i, s):
        return line_kind[i] == 'wall' or (i, s) in core_cover
    wall_ids = sorted(set(i for i in range(len(positions)) if any(walled(i, s) for s in (0, 1))))
    keys = [('line', i) for i in wall_ids]+[('core', c) for c in range(len(core_boxes))]
    options = dict((key, wall_thicknesses(construction, 'party' if key[0] == 'line' and line_kind[key[1]] == 'wall' else 'core')) for key in keys)
    chosen = dict((key, 0) for key in keys)
    def thickness(key):
        return options[key][chosen[key]]
    over_wall, over_col, back_moment, over_core = defaultdict(float), defaultdict(float), defaultdict(float), defaultdict(float)
    core_clamps, overhang_roots = [0], [0.0]
    def core_clamp(root_i, inner_i, s_idx, e):
        if inner_i is None or s_idx != 0:
            return None
        for c, b in enumerate(core_boxes):
            if set((nearest(b['start']), nearest(b['end']))) == set((root_i, inner_i)) and (abs(e-geo[0]['c0']) < 0.05 or abs(e-(geo[0]['c0']+layout['core_depth'])) < 0.05):
                return c
        return None
    overhang_beams, overhang_walls, overhang_text, overhang_issues = [], [], {}, []
    inserts = [0]
    overhang_weight, overhang_u, walls_along, overhang_clear = [0.0], [0.0], [False], [float('inf'), 0, '']
    def rect_area(r):
        return max(0.0, r[1]-r[0])*max(0.0, r[3]-r[2])
    def footprint_of(k):
        g = geo_full[k]
        return (g['a0'], g['a1'], g['c0'], g['c1'])
    def overlap_area(r, f):
        return max(0.0, min(r[1], f[1])-max(r[0], f[0]))*max(0.0, min(r[3], f[3])-max(r[2], f[2]))
    def zone_load(k, r):
        area = rect_area(r)
        if area <= 1e-12:
            return 0.0
        covered = overlap_area(r, footprint_of(k+1))/area if k+1 < storeys else 0.0
        return floor+roof*(1.0-covered)
    def deposit(k, i, s, end, load, couple=0.0, span=0.0):
        if walled(i, s):
            over_wall[k, i, s] += load
            return
        over_col[k, i, cross_index(s, end)] += load+(couple/span if span > 1e-9 else 0.0)
        if span > 1e-9:
            over_col[k, i, cross_index(s, 1-end)] -= couple/span
    def insert_materials(first=None):
        first = first or construction
        return [first] if first == 'Steel' else [first, 'Steel']
    def cantilever_beam(L, w, P, height, back=None):
        for mat in insert_materials():
            own = 0.0
            out = None
            for sweep in range(2):
                try:
                    section, check, drop, volume = select_cantilever(mat, L, w+own, P, slab, height, tol)
                except ValueError:
                    out = None
                    break
                own = member_weight(mat, volume)/L
                out = dict(section=section, check=check, drop=drop, volume=volume, own=own, material=mat)
            if out is None:
                continue
            if back is not None:
                span_b, extra_b = back
                out['back'] = None
                for mat_b in insert_materials(mat):
                    try:
                        out['back'] = beam_for(span_b, max(8.0*(extra_b+out['own']*L*L/2.0)/(span_b*span_b), 1.0), height, mat_b)+(mat_b,)
                        break
                    except ValueError:
                        continue
                if out['back'] is None:
                    continue
            return out
        return None
    def insert_label(mat):
        return '' if mat == construction else 'steel insert '
    def split_load(V, M, L):
        w = max(0.0, 2.0*(V*L-M)/(L*L))
        return w, max(0.0, V-w*L)
    def beam_u(check):
        return max(check['U_M'], check['U_V'], check['U_deflection'], check.get('U_As', 0.0))
    for k in range(1, storeys):
        fp, sb = geo_full[k], geo[k]
        p = levels[k]
        height_k, height_b = p[5]-p[4], levels[k-1][5]-levels[k-1][4]
        fac = FACADE_LOAD*height_k
        valid = valid_of(k)
        if not valid:
            continue
        segs, lane, gallery = segments_of(sb)
        notes = []
        for side in (0, 1):
            L_strip = (sb['c0']-fp['c0']) if side == 0 else (fp['c1']-sb['c1'])
            if L_strip <= 1e-6 or not segs:
                continue
            s_idx, end = (0, 0) if side == 0 else (len(segs)-1, 1)
            root = segs[s_idx][end]
            tip = fp['c0'] if side == 0 else fp['c1']
            L = abs(tip-root)
            rect = (fp['a0'], fp['a1'], fp['c0'], sb['c0']) if side == 0 else (fp['a0'], fp['a1'], sb['c1'], fp['c1'])
            q = zone_load(k, rect)*rect_area(rect)/(L*(fp['a1']-fp['a0']))
            kinds = defaultdict(list)
            stops = [fp['a0']]+[positions[i] for i in valid]+[fp['a1']]
            span_e = max(b-a for a, b in zip(stops, stops[1:]))
            try:
                em = select_edge_member(construction, span_e, q, fac, slab, height_b, tol)
            except ValueError:
                em = None
                overhang_issues.append('OVERHANG TIP EDGE: storey {} side {}: no edge member spans {:.2f} m between the cantilever tips.'.format(k, side, span_e))
            edge_own = member_weight(em['material'], em['volume'])/span_e if em is not None and span_e > 1e-9 else 0.0
            if em is not None:
                width_e, depth_e = em['section']['b'], em['depth']
                top_e = p[4]-em['top']
                ce0, ce1 = (tip, tip+width_e) if side == 0 else (tip-width_e, tip)
                for n_e, (lo_e, hi_e) in enumerate(zip(stops, stops[1:])):
                    if hi_e-lo_e <= 1e-6:
                        continue
                    overhang_beams.append(dict(name='B_{}_overhangEdge{}_{}'.format(k, side, n_e), k=k, box=plan(lo_e, hi_e, ce0, ce1, top_e-depth_e, top_e), section=em['section'], check=em['check'],
                                               direction=span_dir, length=hi_e-lo_e, volume=em['volume']*(hi_e-lo_e)/span_e, material=em['material'], profile=False,
                                               row='{},{},{},{},{},{:.3f},{:.3f},{:.3f}'.format(k, -1, -1, -1, -1, hi_e-lo_e, width_e, depth_e)))
                overhang_weight[0] += edge_own*(fp['a1']-fp['a0'])
                kinds['tip edge {} {}'.format(em['kind'], em['section']['section'])].append(beam_u(em['check']))
            for u, i in enumerate(valid):
                left = (positions[valid[u-1]]+positions[i])/2.0 if u > 0 else fp['a0']
                right = (positions[valid[u+1]]+positions[i])/2.0 if u+1 < len(valid) else fp['a1']
                trib = right-left
                w = q*trib+(fac if u in (0, len(valid)-1) else 0.0)
                P = (fac+edge_own)*trib
                V0, M0 = w*L+P, w*L*L/2.0+P*L
                gable = u in (0, len(valid)-1) and abs(positions[i]-(fp['a0'] if u == 0 else fp['a1'])) < 1.0
                wall_line = walled(i, s_idx) or gable
                beam = cantilever_beam(L, w, P, height_b)
                c_lo, c_hi = (tip, root) if side == 0 else (root, tip)
                back = segs[s_idx][1]-segs[s_idx][0]
                if beam is None and not wall_line:
                    overhang_issues.append('OVERHANG NOT CLAMPED: storey {} line {:.2f} m (frame line) cantilevers {:.2f} m across and no cantilever beam passes; a storey-high wall there would have nothing to clamp into. Move a party wall onto this line or shorten the overhang (a wall is drawn only to keep the load path).'.format(k, positions[i], L))
                if beam is not None and not walled(i, s_idx):
                    back_moment[k-1, i, s_idx] = max(back_moment[k-1, i, s_idx], M0+beam['own']*L*L/2.0)
                if beam is not None:
                    section, check = beam['section'], beam['check']
                    V, M = V0+beam['own']*L, M0+beam['own']*L*L/2.0
                    top = p[4]-beam['drop']
                    lo, hi = along_extent(i, sb, section['b'])
                    overhang_beams.append(dict(name='B_{}_{}_overhangC{}'.format(k, i, side), k=k, box=plan(lo, hi, c_lo, c_hi, top-section['h'], top), section=section, check=check,
                                               direction=cross_dir, length=L, volume=beam['volume'], material=beam['material'], profile=beam['material'] != 'Concrete',
                                               row='{},{},{},{},{},{:.3f},{:.3f},{:.3f}'.format(k, i, cross_index(s_idx, end), i, -1, L, section['b'], section['h'])))
                    overhang_weight[0] += beam['own']*L
                    deposit(k-1, i, s_idx, end, V, M, segs[s_idx][1]-segs[s_idx][0])
                    overhang_roots[0] = max(overhang_roots[0], M)
                    kinds[insert_label(beam['material'])+'cantilever beam '+section['section']].append(beam_u(check))
                    inserts[0] += beam['material'] != construction
                else:
                    res = select_cantilever_wall(construction, L, M0, V0, height_k)
                    overhang_issues.append('OVERHANG WALL (REVIEW): storey {} line {:.2f} m cantilevers {:.2f} m across and no cantilever beam passes, so a storey-high {} wall {:.2f} m carries it as a wall-beam clamped into the wall it continues; it does NOT stack (nothing below or above it in the overhang). Shorten the overhang, use a lighter floor or accept the transfer wall.'.format(k, positions[i], L, res['material'], res['t']))
                    V, M = V0+res['own'], M0+res['own']*L/2.0
                    lo, hi = along_extent(i, sb, res['t'])
                    overhang_walls.append(dict(name='W_cantilever_{}_{}_{}_0'.format(i, 2+side, k), k=k, box=plan(lo, hi, c_lo, c_hi, p[4], p[5]), t=res['t'], length=L, height=height_k, res=res, pos=positions[i]))
                    deposit(k, i, s_idx, end, V, M, back)
                    if not walled(i, s_idx):
                        own_back = res['t']*height_k*back*unit_weight
                        b0, b1 = segs[s_idx]
                        overhang_walls.append(dict(name='W_cantilever_{}_{}_{}_2'.format(i, 2+side, k), k=k, box=plan(lo, hi, b0, b1, p[4], p[5]), t=res['t'], length=back, height=height_k, res=res, pos=positions[i]))
                        deposit(k, i, s_idx, 0, own_back/2.0)
                        deposit(k, i, s_idx, 1, own_back/2.0)
                    kinds['storey-high {} wall {:.2f} m'.format(res['material'], res['t'])].append(res['U'])
                    overhang_u[0] = max(overhang_u[0], res['U'])
            notes.append('local {} side {} {:.2f} m ({})'.format(cross_dir, side, L, ', '.join('{} x {} U {:.2f}'.format(len(us), name, max(us)) for name, us in sorted(kinds.items()))))
        for end in (0, 1):
            L_strip = (sb['a0']-fp['a0']) if end == 0 else (fp['a1']-sb['a1'])
            if L_strip <= 1e-6 or not segs:
                continue
            root_i = valid[0] if end == 0 else valid[-1]
            inner_i = (valid[1] if end == 0 else valid[-2]) if len(valid) > 1 else None
            root = positions[root_i]
            edge = sb['a0'] if end == 0 else sb['a1']
            tip = fp['a0'] if end == 0 else fp['a1']
            Lc = abs(tip-root)
            rect = (fp['a0'], sb['a0'], sb['c0'], sb['c1']) if end == 0 else (sb['a1'], fp['a1'], sb['c0'], sb['c1'])
            q = zone_load(k, rect)
            if end == 0:
                hung = sorted([x for x in positions if fp['a0']-1e-6 <= x < sb['a0']-1e-6], reverse=True)
            else:
                hung = sorted([x for x in positions if sb['a1']+1e-6 < x <= fp['a1']+1e-6])
            if not hung:
                hung = [tip]
            dist = [abs(x-root) for x in hung]
            bounds = [abs(edge-root)]+[(a+b)/2.0 for a, b in zip(dist, dist[1:])]+[Lc]
            points = defaultdict(list)
            for n, d in enumerate(dist):
                trib = bounds[n+1]-bounds[n]
                w_line = q*trib+(fac if n == len(dist)-1 else 0.0)
                pieces = [(seg[0], seg[1], (s, 0), (s, 1)) for s, seg in enumerate(segs)]
                if lane is not None:
                    pieces.append((lane[0], lane[1], (0, 1), (1, 0)))
                for c0p, c1p, ka, kb in pieces:
                    span = c1p-c0p
                    if span <= 1e-6:
                        continue
                    section, check, drop, volume, own_line = beam_for(span, w_line, height_k)
                    R = (w_line+own_line)*span/2.0
                    points[ka].append((d, R))
                    points[kb].append((d, R))
                    overhang_weight[0] += own_line*span
                    pos = hung[n]
                    top = p[4]-drop
                    lo, hi = pos-section['b']/2.0, pos+section['b']/2.0
                    if n == len(dist)-1 and abs(pos-tip) < 1.0:
                        lo, hi = (tip, tip+section['b']) if end == 0 else (tip-section['b'], tip)
                    overhang_beams.append(dict(name='B_{}_hung{}_{}'.format(k, end, len(overhang_beams)), k=k, box=plan(lo, hi, c0p, c1p, top-section['h'], top), section=section, check=check,
                                               direction=cross_dir, length=span, volume=volume,
                                               row='{},{},{},{},{},{:.3f},{:.3f},{:.3f}'.format(k, -1, -1, -1, -1, span, section['b'], section['h'])))
                if gallery > 0:
                    points[len(segs)-1, 1].append((d, (q*trib+(fac if n == len(dist)-1 else 0.0))*gallery))
            run0, run1 = sorted((abs(edge-root), Lc))
            if abs(sb['c0']-fp['c0']) < 1e-6:
                points[0, 0].append(((run0+run1)/2.0, fac*(run1-run0)))
            if abs(sb['c1']-fp['c1']) < 1e-6:
                points[len(segs)-1, 1].append(((run0+run1)/2.0, fac*(run1-run0)))
            kinds = defaultdict(list)
            for (s_idx, s_end), loads in sorted(points.items()):
                V0 = sum(R for d, R in loads)
                M0 = sum(R*d for d, R in loads)
                if V0 <= 1e-9:
                    continue
                e = segs[s_idx][s_end]
                a_lo, a_hi = sorted((root, tip))
                w_eq, P_eq = split_load(V0, M0, Lc)
                clamp = core_clamp(root_i, inner_i, s_idx, e)
                b_in = abs(positions[inner_i]-root) if inner_i is not None and clamp is None else 0.0
                in_lo, in_hi = sorted((root, positions[inner_i])) if inner_i is not None else (root, root)
                if b_in <= 1e-6 and clamp is None:
                    overhang_issues.append('OVERHANG NOT CLAMPED: storey {} local {} end {}: no inner support line for the back-span of the cantilever.'.format(k, span_dir, end))
                beam = cantilever_beam(Lc, w_eq, P_eq, height_b, (b_in, M0) if b_in > 1e-6 else None)
                back_beam = beam['back'][:5] if beam is not None and b_in > 1e-6 else None
                back_mat = beam['back'][5] if beam is not None and b_in > 1e-6 else None
                if beam is not None:
                    section, check = beam['section'], beam['check']
                    V, M = V0+beam['own']*Lc, M0+beam['own']*Lc*Lc/2.0
                    c_lo, c_hi = (e, e+section['b']) if s_end == 0 else (e-section['b'], e)
                    top = p[4]-beam['drop']
                    overhang_beams.append(dict(name='B_{}_{}_overhangA{}{}'.format(k, root_i, end, cross_index(s_idx, s_end)), k=k, box=plan(a_lo, a_hi, c_lo, c_hi, top-section['h'], top), section=section, check=check,
                                               direction=span_dir, length=Lc, volume=beam['volume'], material=beam['material'], profile=beam['material'] != 'Concrete',
                                               row='{},{},{},{},{},{:.3f},{:.3f},{:.3f}'.format(k, root_i, cross_index(s_idx, s_end), -1, cross_index(s_idx, s_end), Lc, section['b'], section['h'])))
                    overhang_weight[0] += beam['own']*Lc
                    dk = k-1
                    kinds[insert_label(beam['material'])+'cantilever beam '+section['section']].append(beam_u(check))
                    inserts[0] += beam['material'] != construction
                    if b_in > 1e-6:
                        bsec, bcheck, bdrop, bvol, bown = back_beam
                        clear_back = (p[4]-bdrop-bsec['h'])-levels[k-1][4]
                        if clear_back < overhang_clear[0]:
                            overhang_clear[:] = [clear_back, k-1, bsec['section']]
                        bc_lo, bc_hi = (e, e+bsec['b']) if s_end == 0 else (e-bsec['b'], e)
                        btop = p[4]-bdrop
                        overhang_beams.append(dict(name='B_{}_{}_overhangB{}{}'.format(k, root_i, end, cross_index(s_idx, s_end)), k=k, box=plan(in_lo, in_hi, bc_lo, bc_hi, btop-bsec['h'], btop), section=bsec, check=bcheck,
                                                   direction=span_dir, length=b_in, volume=bvol, material=back_mat, profile=back_mat != 'Concrete',
                                                   row='{},{},{},{},{},{:.3f},{:.3f},{:.3f}'.format(k, inner_i, cross_index(s_idx, s_end), root_i, cross_index(s_idx, s_end), b_in, bsec['b'], bsec['h'])))
                        overhang_weight[0] += bown*b_in
                        deposit(dk, root_i, s_idx, s_end, bown*b_in/2.0)
                        deposit(dk, inner_i, s_idx, s_end, bown*b_in/2.0)
                        kinds[insert_label(back_mat)+'back-span beam '+bsec['section']].append(beam_u(bcheck))
                        inserts[0] += back_mat != construction
                else:
                    res = select_cantilever_wall(construction, Lc, M0, V0, height_k)
                    V, M = V0+res['own'], M0+res['own']*Lc/2.0
                    c_lo, c_hi = (e, e+res['t']) if s_end == 0 else (e-res['t'], e)
                    overhang_walls.append(dict(name='W_cantilever_{}_{}_{}_1'.format(root_i, 4+cross_index(s_idx, s_end), k), k=k, box=plan(a_lo, a_hi, c_lo, c_hi, p[4], p[5]), t=res['t'], length=Lc, height=height_k, res=res, pos=e))
                    dk = k
                    if b_in > 1e-6:
                        own_back = res['t']*height_k*b_in*unit_weight
                        overhang_walls.append(dict(name='W_cantilever_{}_{}_{}_3'.format(root_i, 4+cross_index(s_idx, s_end), k), k=k, box=plan(in_lo, in_hi, c_lo, c_hi, p[4], p[5]), t=res['t'], length=b_in, height=height_k, res=res, pos=e))
                        deposit(dk, root_i, s_idx, s_end, own_back/2.0)
                        deposit(dk, inner_i, s_idx, s_end, own_back/2.0)
                        walls_along[0] = True
                    kinds['storey-high {} wall {:.2f} m'.format(res['material'], res['t'])].append(res['U'])
                    overhang_u[0] = max(overhang_u[0], res['U'])
                overhang_roots[0] = max(overhang_roots[0], M)
                if clamp is not None:
                    over_core[dk, clamp] += V
                    core_clamps[0] += 1
                elif inner_i is not None and b_in > 1e-6:
                    deposit(dk, root_i, s_idx, s_end, V+M/b_in)
                    deposit(dk, inner_i, s_idx, s_end, -M/b_in)
                else:
                    deposit(dk, root_i, s_idx, s_end, V)
            notes.append('local {} end {} {:.2f} m ({} hung line(s) at {} m carried on the long facade and corridor lines: {})'.format(
                span_dir, end, Lc, len(hung), ', '.join('{:.2f}'.format(x) for x in hung),
                ', '.join('{} x {} U {:.2f}'.format(len(us), name, max(us)) for name, us in sorted(kinds.items()))))
        if notes:
            overhang_text[k] = '; '.join(notes)
    for b in overhang_beams:
        overhang_u[0] = max(overhang_u[0], beam_u(b['check']))
    if overhang_clear[0] < TARGET_CLEAR_HEIGHT-1e-9:
        overhang_issues.append('OVERHANG BACK-SPAN HEADROOM: {:.2f} m clear under the back-span beam {} in the last bay of storey {} (target {:.2f} m); the beam sits in the facade/corridor wall line, so it cuts into door and window heads there.'.format(overhang_clear[0], overhang_clear[2], overhang_clear[1], TARGET_CLEAR_HEIGHT))
    if inserts[0]:
        overhang_issues.append('OVERHANG STEEL INSERTS: {} cantilever/back-span beam(s) under the overhangs are steel (S235 box sections) because no {} beam passes; steel-{} connections at the root and fire protection are not designed.'.format(inserts[0], construction, 'timber' if construction == 'Timber' else 'concrete'))
    if walls_along[0]:
        overhang_issues.append('OVERHANG ALONG THE BUILDING: no cantilever beam passes, so storey-high walls in the long facade and corridor walls carry it and continue over the last bay inside as their back-span; these are facade/corridor walls, not walls between apartments; check windows and doors there.')
    if overhang_u[0] > 1.0+1e-9:
        overhang_issues.append('OVERHANG MEMBER FAILS: max utilisation {:.2f} (cantilever beam or storey-high cantilever wall); shorten the overhang or add a deeper/stronger cantilever system.'.format(overhang_u[0]))
    def compute():
        n = defaultdict(float)
        rows = {}
        for k in reversed(range(storeys)):
            p, g = levels[k], geo[k]
            height = p[5]-p[4]
            q = roof if k == storeys-1 else q_design
            segs, lane, cantilever = segments_of(g)
            valid = valid_lines(g)
            trib = tribs(valid, g)
            for i in valid:
                for s, (c0, c1) in enumerate(segs):
                    if not walled(i, s):
                        continue
                    length = c1-c0
                    extra = sum(end_load(k, g, e, q, trib[i], lane, cantilever, gable_line(k, g, i, valid), i) for e in (c0, c1))/length
                    n['line', i, s] += q_at(k, i, c0, c1)*trib[i]+gable_line(k, g, i, valid)+extra+thickness(('line', i))*height*unit_weight+over_wall.get((k, i, s), 0.0)/length
                    rows['line', i, s, k] = (n['line', i, s], c0, c1)
            for c, b in enumerate(core_boxes):
                if not (g['a0']-1e-6 <= b['start'] and b['end'] <= g['a1']+1e-6):
                    continue
                first_c, last_c = nearest(b['start']), nearest(b['end'])
                inner_len = along_extent(last_c, g, thickness(('line', last_c)) if ('line', last_c) in options else WALL_START[construction][1])[0]-along_extent(first_c, g, thickness(('line', first_c)) if ('line', first_c) in options else WALL_START[construction][1])[1]
                n['core', c] += thickness(('core', c))*height*unit_weight+(over_core.get((k, c), 0.0)/(2.0*inner_len) if inner_len > 1e-6 else 0.0)
                rows['core', c, 0, k] = (n['core', c], g['c0'], g['c0']+core_depth)
        return rows
    for attempt in range(60):
        rows = compute()
        changed = False
        for key in keys:
            worst = max([wall_check(construction, thickness(key), levels[r[3]][5]-levels[r[3]][4], v[0])['U']
                         for r, v in rows.items() if (r[0], r[1]) == key] or [0.0])
            if worst > 1.0:
                if chosen[key]+1 >= len(options[key]):
                    hint = ' It carries the root reaction of an overhang (V + M/b of the cantilevers above): shorten the overhang, use Concrete walls or give the overhang its own lighter build-up.' if key[0] == 'line' and any(kk[1] == key[1] and v > 0 for kk, v in over_wall.items()) else ''
                    raise ValueError('{} wall {} {}: no thickness up to {:.2f} m passes.{}'.format(material, key[0], key[1], options[key][-1], hint))
                chosen[key] += 1
                changed = True
        if not changed:
            break
    def covered(k):
        if k+1 >= storeys:
            return 0.0
        p, u = levels[k], levels[k+1]
        return max(0.0, min(p[1], u[1])-max(p[0], u[0]))*max(0.0, min(p[3], u[3])-max(p[2], u[2]))
    floor_total = sum((p[1]-p[0])*(p[3]-p[2])*roof+(floor-roof)*covered(k) for k, p in enumerate(levels))
    exposed_floor = sum(floor*(rect_area(footprint_of(k))-overlap_area(footprint_of(k), footprint_of(k-1))) for k in range(1, storeys))
    floor_total += exposed_floor
    def wall_weight():
        total = 0.0
        for k, p in enumerate(levels):
            g, height = geo[k], p[5]-p[4]
            segs = segments_of(g)[0]
            for i in valid_lines(g):
                for s, (c0, c1) in enumerate(segs):
                    if walled(i, s):
                        total += thickness(('line', i))*height*unit_weight*(c1-c0)
            for c, b in enumerate(core_boxes):
                if g['a0']-1e-6 <= b['start'] and b['end'] <= g['a1']+1e-6:
                    first, last = nearest(b['start']), nearest(b['end'])
                    inner = along_extent(last, g, thickness(('line', last)))[0]-along_extent(first, g, thickness(('line', first)))[1]
                    total += 2.0*thickness(('core', c))*height*unit_weight*inner
        return total
    coupled_lines = []
    lane_g0 = segments_of(g0)[1]
    if construction == 'Concrete' and lane_g0 is not None:
        coupled_lines = [i for i in valid_lines(g0) if walled(i, 0) and walled(i, 1) and (i, 0) not in core_cover and (i, 1) not in core_cover]
    height_total = levels[-1][5]-levels[0][4]
    breadth_c = max(geo_full[k]['a1']-geo_full[k]['a0'] for k in range(storeys))
    wind_shape = [((s0+s1)/2.0, peak_pressure(ze, wind[0], wind[1])*(geo_full[k]['a1']-geo_full[k]['a0'])*(s1-s0))
                  for k in range(storeys) for s0, s1, ze in wind_slices(levels[k][4]-levels[0][4], levels[k][5]-levels[0][4], height_total, breadth_c)]
    coupling_cache = {}
    def coupling_for(i, r0, r1):
        key = (i, round(r0[1]-r0[0], 4), round(r1[1]-r1[0], 4))
        if key not in coupling_cache:
            lins = [lintel_for(k, i) for k in range(storeys) if segments_of(geo[k])[1] is not None and i in valid_of(k)]
            ic = COUPLING_CRACKED*min(l['section']['b']*l['section']['h']**3/12.0 for l in lins)
            parts = []
            for r in (r0, r1):
                length, thick = r[3]-r[2], r[1]-r[0]
                parts.append(dict(A=thick*length, I=thick*length**3/12.0, e=length/2.0, L=length, N=r[4]))
            lever = (r1[2]+r1[3])/2.0-(r0[2]+r0[3])/2.0
            clear = lane_g0[1]-lane_g0[0]
            storey_h = height_total/storeys
            options = []
            for share in COUPLING_SHARES:
                sol = coupling_solution(parts[0]['A'], parts[1]['A'], parts[0]['I'], parts[1]['I'], lever, clear, storey_h, ic*share, wind_shape, height_total)
                sol['share'] = share
                options.append(sol)
            coupling_cache[key] = dict(parts=parts, I=parts[0]['I']+parts[1]['I'], lever=lever, clear=clear, options=options,
                                       MRd=min(l['check']['MRd'] for l in lins), VRd=min(l['check']['VRd'] for l in lins), Vg=max(l['check']['VEd'] for l in lins))
        cp = coupling_cache[key]
        for part, r in zip(cp['parts'], (r0, r1)):
            part['N'] = r[4]
        return cp
    def lateral():
        rows0 = compute()
        rects, groups, used = {}, [], set()
        for (kind, i, s, k), (n_service, c0, c1) in rows0.items():
            if k != 0:
                continue
            if kind == 'line':
                t = thickness(('line', i))
                a0, a1 = along_extent(i, g0, t)
                rects['line', i, s] = (a0, a1, c0, c1, n_service*(c1-c0))
            else:
                t = thickness(('core', i))
                b = core_boxes[i]
                first, last = nearest(b['start']), nearest(b['end'])
                s0 = along_extent(first, g0, thickness(('line', first)))[1]
                s1 = along_extent(last, g0, thickness(('line', last)))[0]
                rects['core', i, 0] = (s0, s1, c0, c0+t, n_service*(s1-s0))
                rects['core', i, 1] = (s0, s1, c1-t, c1, n_service*(s1-s0))
        for c, b in enumerate(core_boxes):
            members = [key for key in rects if key[0] == 'core' and key[1] == c]
            members += [('line', i, 0) for i in (nearest(b['start']), nearest(b['end'])) if ('line', i, 0) in rects]
            used.update(members)
            groups.append(('core {}'.format(c), members))
        for i in coupled_lines:
            if ('line', i, 0) in rects and ('line', i, 1) in rects and ('line', i, 0) not in used and ('line', i, 1) not in used:
                used.update([('line', i, 0), ('line', i, 1)])
                groups.append(('wall line {} pair'.format(i), [('line', i, 0), ('line', i, 1)]))
        for key in sorted(rects):
            if key not in used:
                groups.append(('wall line {} seg {}'.format(key[1], key[2]), [key]))
        frames = sum(1 for i in valid_lines(g0) for s in range(len(segments_of(g0)[0])) if not walled(i, s))
        faces = [(levels[k][4]-levels[0][4], levels[k][5]-levels[0][4], geo_full[k]['c1']-geo_full[k]['c0'], geo_full[k]['a1']-geo_full[k]['a0']) for k in range(storeys)]
        elements = wall_elements(rects, groups)
        for e in elements:
            if e['name'].endswith(' pair'):
                i = int(e['name'].split()[2])
                e['coupled'] = coupling_for(i, rects['line', i, 0], rects['line', i, 1])
                e['Ic_free'] = e['coupled']['I']
        return lateral_check(levels, elements, material, floor_total+wall_weight()+extra_gravity, len(rects)+2*frames, (g0['a0'], g0['a1'], g0['c0'], g0['c1']), wind, faces)
    stability = lateral()
    gravity_choice = dict(chosen)
    def key_length(key):
        if key[0] == 'line':
            return sum(c1-c0 for s, (c0, c1) in enumerate(segments_of(g0)[0]) if walled(key[1], s)) or 1.0
        b = core_boxes[key[1]]
        return 2.0*max(b['end']-b['start'], 1e-3)
    steps = 0
    while stability['status'] != 'PASS_ASSUMED_MODEL' and steps < 200:
        steps += 1
        best = None
        for key in keys:
            if chosen[key]+1 >= len(options[key]):
                continue
            added = (options[key][chosen[key]+1]-options[key][chosen[key]])*key_length(key)
            chosen[key] += 1
            trial = lateral()
            chosen[key] -= 1
            gain = stability['worst']-trial['worst']
            if gain <= 1e-6:
                continue
            score = gain/added
            if best is None or score > best[0]:
                best = (score, key, trial)
        if best is None:
            break
        chosen[best[1]] += 1
        stability = best[2]
    def key_label(key):
        return 'core {} long walls'.format(key[1]) if key[0] == 'core' else 'line {:.2f} m'.format(positions[key[1]])
    grown = ['{} {:.2f}->{:.2f} m'.format(key_label(key), options[key][gravity_choice[key]], options[key][chosen[key]])
             for key in keys if chosen[key] != gravity_choice[key]]
    rows = compute()
    walls, wall_rows, max_wall_u, wall_volume = [], [], 0.0, 0.0
    base_segments = []
    member_face = {}
    end_walls = set()
    for (kind, i, s, k), (n_service, c0, c1) in sorted(rows.items(), key=lambda item: (item[0][3], item[0][0], item[0][1], item[0][2])):
        p, g = levels[k], geo[k]
        height = p[5]-p[4]
        if kind == 'line':
            t = thickness(('line', i))
            if i in (valid_of(0)[0], valid_of(0)[-1]):
                end_walls.add(i)
            a0, a1 = along_extent(i, g, t)
            pieces = [(a0, a1, c0, c1)]
            length = c1-c0
            pos = positions[i]
            name_kind = 'party' if line_kind[i] == 'wall' else 'core'
            member_face[k, i, cross_index(s, 0)] = member_face[k, i, cross_index(s, 1)] = (a0, a1)
        else:
            t = thickness(('core', i))
            b = core_boxes[i]
            first, last = nearest(b['start']), nearest(b['end'])
            s0 = along_extent(first, g, thickness(('line', first)))[1]
            s1 = along_extent(last, g, thickness(('line', last)))[0]
            pieces = [(s0, s1, c0, c0+t), (s0, s1, c1-t, c1)]
            length = 2.0*(s1-s0)
            pos = b['start']
            name_kind = 'core'
        check = wall_check(construction, t, height, n_service)
        max_wall_u = max(max_wall_u, check['U'])
        wid = 'W_{}_{}_{}_{}'.format(name_kind if kind == 'line' else 'corelong', i, s, k)
        for n_piece, (a0, a1, c0p, c1p) in enumerate(pieces):
            walls.append(('{}_{}'.format(wid, n_piece), k, name_kind, plan(a0, a1, c0p, c1p, p[4], p[5])))
            wall_volume += (a1-a0)*(c1p-c0p)*height
            if k == 0:
                base_segments.append((name_kind, n_service, t, a0, a1, c0p, c1p, wid))
        wall_rows.append([wid, name_kind, k, pos, length, t, height, n_service, check['NEd'], check['NRd'], check['U'], material])
    for ow in overhang_walls:
        walls.append((ow['name'], ow['k'], 'cantilever', ow['box']))
        b = ow['box']
        wall_volume += (b[1]-b[0])*(b[3]-b[2])*(b[5]-b[4])
        wall_rows.append([ow['name'][:-2], 'cantilever', ow['k'], ow['pos'], ow['length'], ow['t'], ow['height'], 0.0, ow['res']['tension'], 0.0, ow['res']['U'], material])
        max_wall_u = max(max_wall_u, ow['res']['U'])
    beams, bd, profiles, checks = [], [], {}, []
    columns, cd = [], []
    vb, vc, max_u = 0.0, 0.0, max_wall_u
    vb_steel = 0.0
    member_count = defaultdict(int)
    edge_kinds = defaultdict(int)
    col_load, col_side, base_columns = defaultdict(float), {}, []
    frame_sections = defaultdict(int)
    min_clear = float('inf')
    min_inside = [float('inf'), 0.0, 0]
    width_governed = [0]
    frame_beam_weight = 0.0
    max_frame_width = {}
    def add_member(name, k, box, section, check, material_name, direction, length, volume, profile, row):
        beams.append((name, k, box))
        if profile:
            profiles[name] = dict(section, direction=direction, length=length)
        checks.append(material_check_row(name, material_name, section, check))
        bd.append(row)
        return max(check['U_M'], check['U_V'], check['U_deflection'], check.get('U_As', 0.0)), volume
    for k in reversed(range(storeys)):
        p, g = levels[k], geo[k]
        height = p[5]-p[4]
        q = roof if k == storeys-1 else q_design
        segs, lane, cantilever = segments_of(g)
        valid = valid_lines(g)
        trib = tribs(valid, g)
        for i in valid:
            for s, (c0, c1) in enumerate(segs):
                if walled(i, s):
                    continue
                length = c1-c0
                w = q_at(k, i, c0, c1)*trib[i]+gable_line(k, g, i, valid)
                w_design = max(w, 8.0*back_moment.get((k, i, s), 0.0)/(length*length)) if length > 1e-9 else w
                try:
                    section, check, drop, volume, own_line = select_frame_beam(construction, length, w_design, slab, height, tol)
                except ValueError:
                    section, check, drop, volume, own_line = select_frame_beam(construction, length, w, slab, height, tol)
                    overhang_issues.append('OVERHANG BACK-SPAN FAILS: storey {} frame line {:.2f} m: no frame beam carries the cantilever hogging moment {:.0f} kNm; move a party wall onto this line or shorten the overhang.'.format(k, positions[i], back_moment[k, i, s]))
                frame_beam_weight += own_line*length
                clear_here = height-section['h']-drop
                min_clear = min(min_clear, clear_here)
                inside_unit = line_kind[i] == 'internal' or (geo_full[k]['a0']+1.0 < positions[i] < geo_full[k]['a1']-1.0 and not (g0['a0']+1.0 < positions[i] < g0['a1']-1.0))
                if inside_unit and clear_here < min_inside[0]:
                    min_inside[:] = [clear_here, height, k]
                ends = []
                for end, e in enumerate((c0, c1)):
                    j = cross_index(s, end)
                    col_load[i, j] += (w+own_line)*length/2.0+end_load(k, g, e, q, trib[i], lane, cantilever, gable_line(k, g, i, valid), i)+over_col.get((k, i, j), 0.0)
                    carried = [section]
                    if lane is not None and (abs(e-lane[0]) < 1e-6 or abs(e-lane[1]) < 1e-6):
                        carried.append(lintel_for(k, i)['section'])
                    if cantilever > 0 and abs(e-(g['c1']-cantilever)) < 1e-6:
                        carried.append(gallery_for(k, i)['section'])
                    bearing = max([sec['b'] for sec in carried if construction != 'Concrete' or sec['b'] <= sec['h']+1e-9] or [0.0])
                    if bearing > col_side.get((i, j), 0.0)+1e-9:
                        width_governed[0] += 1
                    csec, ccheck, own = size_frame_column(construction, height, max(0.0, col_load[i, j]), max(col_side.get((i, j), 0.0), bearing))
                    col_load[i, j] += own
                    side = csec['b']
                    col_side[i, j] = side
                    a0, a1 = along_extent(i, g, side)
                    cc0, cc1 = (e, e+side) if end == 0 else (e-side, e)
                    ni, nj = (i, j) if along == 0 else (j, i)
                    name = 'C_{}_{}_{}'.format(k, ni, nj)
                    columns.append((name, k, plan(a0, a1, cc0, cc1, p[4], p[5])))
                    if construction == 'Steel':
                        profiles[name] = dict(csec, direction='Z', length=height)
                    checks.append(material_check_row(name, construction, csec, ccheck))
                    max_u = max(max_u, ccheck.get('U_N', 0.0), ccheck.get('U_M', 0.0))
                    vc += csec['A']*height
                    cd.append('{},{},{},{:.3f},{:.3f},{:.3f},{:.3f}'.format(k, ni, nj, height, side, trib[i]*length/2.0, col_load[i, j]))
                    member_face[k, i, j] = (a0, a1)
                    ends.append(cc1 if end == 0 else cc0)
                    max_frame_width[i] = max(max_frame_width.get(i, 0.0), side)
                    if k == 0:
                        base_columns.append((name, col_load[i, j], side, (a0+a1)/2.0, (cc0+cc1)/2.0))
                width, depth = section['b'], section['h']
                top = p[5]-drop
                a0, a1 = along_extent(i, g, width)
                u_max, v = add_member('B_{}_{}_frame{}'.format(k, i, s), k, plan(a0, a1, ends[0], ends[1], top-depth, top), section, check,
                                      construction, cross_dir, ends[1]-ends[0], volume, construction != 'Concrete',
                                      '{},{},{},{},{},{:.3f},{:.3f},{:.3f}'.format(k, i, cross_index(s, 0), i, cross_index(s, 1), length, width, depth))
                max_u, vb = max(max_u, u_max), vb+v
                member_count['frame'] += 1
                frame_sections[section['section']] += 1
        tips = {}
        if cantilever > 0:
            for i in valid:
                gal = gallery_for(k, i)
                section, check, drop, volume = gal['section'], gal['check'], gal['drop'], gal['volume']
                width, depth = section['b'], section['h']
                lo, hi = along_extent(i, g, width)
                tips[i] = (lo, hi)
                top = p[5]-drop
                u_max, v = add_member('B_{}_{}_gallery'.format(k, i), k, plan(lo, hi, g['c1']-cantilever, g['c1'], top-depth, top), section, check,
                                      construction, cross_dir, cantilever, volume, construction != 'Concrete',
                                      '{},{},{},{},{},{:.3f},{:.3f},{:.3f}'.format(k, i, 1, i, 2, cantilever, width, depth))
                max_u, vb = max(max_u, u_max), vb+v
                member_count['gallery'] += 1
        last_j = cross_index(len(segs)-1, 1)
        for u in range(len(valid)-1):
            i, j = valid[u], valid[u+1]
            span = positions[j]-positions[i]
            member = edge_for(k, u)['member']
            section, check = member['section'], member['check']
            width, depth = section['b'], member['depth']
            top = p[5]-member['top']
            for side, edge in ((0, g['c0']), (1, g['c1'])):
                if side == 1 and cantilever > 0:
                    lo, hi = tips[i][1], tips[j][0]
                else:
                    jj = 0 if side == 0 else last_j
                    lo, hi = member_face.get((k, i, jj), (positions[i], positions[i]))[1], member_face.get((k, j, jj), (positions[j], positions[j]))[0]
                c0b, c1b = (edge, edge+width) if side == 0 else (edge-width, edge)
                u_max, v = add_member('B_{}_{}_edge{}'.format(k, i, side), k, plan(lo, hi, c0b, c1b, top-depth, top), section, check, member['material'], span_dir,
                                      hi-lo, member['volume']*(hi-lo)/span, False,
                                      '{},{},{},{},{},{:.3f},{:.3f},{:.3f}'.format(k, i, 0 if side == 0 else last_j, j, 0 if side == 0 else last_j, span, width, depth))
                max_u, vb = max(max_u, u_max), vb+v
                member_count['edge'] += 1
                edge_kinds[member['kind'], member['section']['section']] += 1
        if lane is None:
            continue
        for i in valid:
            span = lane[1]-lane[0]
            lin = lintel_for(k, i)
            section, check, drop, volume, own_line = lin['section'], lin['check'], lin['drop'], lin['volume'], lin['own']
            width, depth = section['b'], section['h']
            top = p[5]-drop
            lo, hi = along_extent(i, g, width)
            u_max, v = add_member('B_{}_{}_lintel'.format(k, i), k, plan(lo, hi, lane[0], lane[1], top-depth, top), section, check,
                                  construction, cross_dir, span, volume, construction != 'Concrete',
                                  '{},{},{},{},{},{:.3f},{:.3f},{:.3f}'.format(k, i, 1, i, 2, span, width, depth))
            max_u, vb = max(max_u, u_max), vb+v
            member_count['lintel'] += 1
    for ob in overhang_beams:
        u_max, v = add_member(ob['name'], ob['k'], ob['box'], ob['section'], ob['check'], ob.get('material', construction), ob['direction'], ob['length'], ob['volume'], ob.get('profile', construction != 'Concrete'), ob['row'])
        max_u = max(max_u, u_max)
        if ob.get('material', construction) == 'Steel' and construction != 'Steel':
            vb_steel += v
        else:
            vb += v
        member_count['overhang'] += 1
    max_u = max(max_u, overhang_u[0])
    uplift_columns = sorted(set(key for key, value in col_load.items() if value < -1e-6))
    if True:
        storey0_h = levels[0][5]-levels[0][4]
        valid0 = valid_lines(g0)
        trib0 = tribs(valid0, g0)
        core_regions = []
        for c, b in enumerate(core_boxes):
            first, last = nearest(b['start']), nearest(b['end'])
            lo = along_extent(first, g0, thickness(('line', first)) if ('line', first) in options else WALL_START[construction][1])[0]
            hi = along_extent(last, g0, thickness(('line', last)) if ('line', last) in options else WALL_START[construction][1])[1]
            core_regions.append(dict(c=c, first=first, last=last, a0=lo, a1=hi, c0=g0['c0'], c1=max(g0['c0']+core_depth, segments_of(g0)[0][0][1] if segments_of(g0)[0] else 0.0)))
        core_lines = dict((r['first'], r) for r in core_regions)
        core_lines.update((r['last'], r) for r in core_regions)
        seg_info = []
        for kind, n_service, t, a0, a1, c0p, c1p, wid in base_segments:
            parts = wid.split('_')
            seg_info.append((parts[1], int(parts[2]), int(parts[3]), n_service, t, a0, a1, c0p, c1p))
        per = stability.get('per', {})
        def moments_of(name):
            return per.get(name, {}).get('M', [])
        def wall_wind(i, s, c0p, c1p, t0):
            if 'wall line {} pair'.format(i) in per:
                if s != 0:
                    return []
                return [(max(g0['c0'], t0), g0['c1'], m_c) for m_a, m_c in moments_of('wall line {} pair'.format(i))]
            return [(max(c0p, t0), c1p, m_c) for m_a, m_c in moments_of('wall line {} seg {}'.format(i, s))]
        def design_piles(diameter, capacity, label):
            issues = []
            gf_span = max([positions[valid0[u+1]]-positions[valid0[u]] for u in range(len(valid0)-1)] or [0.0])
            gf_t, gf_w, gf_label = ground_floor_type(gf_span-FB_MIN_WIDTH)
            facade_w = FACADE_LOAD*storey0_h
            point_extra = defaultdict(list)
            poer_extra = defaultdict(float)
            beams = []
            for u in range(len(valid0)-1):
                i, j = valid0[u], valid0[u+1]
                L = positions[j]-positions[i]
                for edge, side in ((g0['c0'], 0), (g0['c1'], 1)):
                    width, h = FB_MIN_WIDTH, FB_H_MIN
                    while True:
                        ws = facade_w+DENSITY*width*h
                        check = foundation_beam_check(width, h, FOUNDATION_ULS*ws*L**2/8.0, FOUNDATION_ULS*ws*L/2.0)
                        if check['U'] <= 1.0 or h >= FB_H_MAX:
                            break
                        h = round(h+0.05, 10)
                    for line in (i, j):
                        region = core_lines.get(line)
                        if region is not None and side == 0:
                            poer_extra[region['c']] += ws*L/2.0
                        else:
                            point_extra[line].append((edge, ws*L/2.0))
                    centre = edge+width/2.0 if side == 0 else edge-width/2.0
                    beams.append(dict(name='FB_facade{}_{}'.format(side, i), axis='a', t0=positions[i], t1=positions[j], centre=centre,
                                      across=(lambda bw, e=edge, sd=side: (e, e+bw) if sd == 0 else (e-bw, e)),
                                      result=dict(piles=[], rows=1, smin=0.0, width=width, height=h, check=check, uls=[], wmax=[], wmin=[],
                                                  u_pile=0.0, spacing=L, status='PASS_ASSUMED_MODEL' if check['U'] <= 1.0 else 'FAIL')))
            poers = []
            poer_end = {}
            for r in core_regions:
                n_load = poer_extra[r['c']]
                for typ, i, s, n_service, t, a0, a1, c0p, c1p in seg_info:
                    if (typ == 'corelong' and i == r['c']) or (typ != 'corelong' and i in (r['first'], r['last']) and s == 0):
                        n_load += n_service*max(a1-a0, c1p-c0p)
                for name, load, side, am, cm in base_columns:
                    if r['a0']-1e-6 <= am <= r['a1']+1e-6 and r['c0']-1e-6 <= cm <= r['c1']+1e-6:
                        n_load += load
                n_load += floor*(r['c1']-r['c0'])*(trib0.get(r['first'], 0.0)+trib0.get(r['last'], 0.0))
                for line in (r['first'], r['last']):
                    if line in (valid0[0], valid0[-1]) and abs(positions[line]-(g0['a0'] if line == valid0[0] else g0['a1'])) < 1.0:
                        n_load += facade_w*(r['c1']-r['c0'])
                ms = []
                for m_a, m_c in moments_of('core {}'.format(r['c'])):
                    ms.append((m_a, m_c))
                res = pile_group((r['a0'], r['a1'], r['c0'], r['c1']), n_load, ms, diameter, capacity, min_h=POER_H_MIN,
                                 grow=(0.0, 0.3, 0.0, 0.3), limits=(g0['a0'], g0['a1'], g0['c0'], g0['c1']))
                poers.append(dict(name='FP_core{}'.format(r['c']), result=res))
                for line in (r['first'], r['last']):
                    poer_end[line] = max(poer_end.get(line, g0['c0']), res['box'][3])
            for i in valid0:
                t0 = poer_end.get(i, g0['c0'])
                t1 = g0['c1']
                dist, points, wind, widths = [], list(point_extra[i]), [], [0.0]
                dist.append((t0, t1, floor*trib0[i]+(facade_w if i in (valid0[0], valid0[-1]) and abs(positions[i]-(g0['a0'] if i == valid0[0] else g0['a1'])) < 1.0 else 0.0)))
                for typ, li, s, n_service, t, a0, a1, c0p, c1p in seg_info:
                    if typ == 'corelong' or li != i or c1p <= t0+1e-6 or (s == 0 and i in core_lines):
                        continue
                    dist.append((max(c0p, t0), c1p, n_service))
                    widths.append(t+0.2)
                    wind.extend(wall_wind(i, s, c0p, c1p, t0))
                fixed = []
                for name, load, side, am, cm in base_columns:
                    if nearest(am) == i and t0-1e-6 <= cm <= t1+1e-6:
                        points.append((cm, load))
                        fixed.append(cm)
                        widths.append(side+0.1)
                if i in poer_end:
                    points = [(max(x, t0), p) for x, p in points]
                if t1-t0 < 0.5:
                    continue
                guess_w = max(widths+[FB_MIN_WIDTH, diameter+2.0*PILE_BEAM_EDGE])
                res = None
                for sweep in range(3):
                    h_guess = res['height'] if res else FB_H_MIN
                    w_guess = res['width'] if res else guess_w
                    self_w = DENSITY*w_guess*h_guess
                    res = pile_line(t0, t1, fixed, dist+[(t0, t1, self_w)], points, wind, diameter, capacity, max(widths))
                    if abs(res['height']-h_guess) < 1e-6 and abs(res['width']-w_guess) < 1e-6:
                        break
                centre = (along_extent(i, g0, res['width'])[0]+along_extent(i, g0, res['width'])[1])/2.0
                beams.append(dict(name='FB_line{}'.format(i), axis='c', t0=t0, t1=t1, centre=centre,
                                  across=(lambda bw, li=i: along_extent(li, g0, bw)), result=res))
            panels = []
            for u in range(len(valid0)-1):
                i, j = valid0[u], valid0[u+1]
                bi = next(b['result']['width'] for b in beams if b['name'] == 'FB_line{}'.format(i)) if any(b['name'] == 'FB_line{}'.format(i) for b in beams) else 0.0
                bj = next(b['result']['width'] for b in beams if b['name'] == 'FB_line{}'.format(j)) if any(b['name'] == 'FB_line{}'.format(j) for b in beams) else 0.0
                a_lo = along_extent(i, g0, bi)[1]
                a_hi = along_extent(j, g0, bj)[0]
                c_lo = max(g0['c0']+FB_MIN_WIDTH, max([p['result']['box'][3] for p, r in zip(poers, core_regions) if r['first'] == i and r['last'] == j] or [g0['c0']]))
                panels.append(('GF_{}'.format(i), a_lo, a_hi, c_lo, g0['c1']-FB_MIN_WIDTH))
            result = assemble_foundation(piles, diameter, capacity, label, max(PILE_SPACING_FACTOR*diameter, PILE_MIN_SPACING), beams, poers, panels,
                                         (gf_t, gf_w, gf_label, gf_span-FB_MIN_WIDTH), storeys, plan, g0['z0'], issues)
            result['issues'] = issues
            return result
        pile_result = choose_pile_design(piles, design_piles)
        foundation = 'PILES'
        footing_fail = pile_result['status'] != 'PASS_ASSUMED_MODEL'
        footings = pile_result['footings']
        footing_kinds = pile_result['kinds']
        pile_rows, element_rows = pile_result['pile_rows'], pile_result['element_rows']
        foundation_lines = foundation_summary_lines(pile_result, piles)
    out_rows = []
    for r in wall_rows:
        out_rows.append('{},{},{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.4f},{}'.format(*r))
    cross_lines = [g0['c0']]
    segs0, lane0, cantilever0 = segments_of(g0)
    if lane0 is not None:
        cross_lines += [lane0[0], lane0[1]]
    elif cantilever0 > 0:
        cross_lines += [g0['c1']-cantilever0]
    cross_lines.append(g0['c1'])
    xs, ys = (positions, cross_lines) if along == 0 else (cross_lines, positions)
    fa = sum((min(levels[k][1], levels[k+1][1])-max(levels[k][0], levels[k+1][0]))*(min(levels[k][3], levels[k+1][3])-max(levels[k][2], levels[k+1][2])) for k in range(storeys-1))
    ra = sum((p[1]-p[0])*(p[3]-p[2]) for p in levels)-fa
    frame = dict(columns=columns, beams=beams, cd=cd, bd=bd, loads={}, sides=dict(col_side), vc=vc, vb=vb, vb_steel=vb_steel, fa=fa, ra=ra, residual=0.0,
                 notes=[], rows=['Storey {}: support lines {} | wall pieces {} | columns {} | frame beams {} | corridor lintels {} | facade edge members {} | gallery cantilevers {}'.format(
                     k, len(valid_lines(geo[k])), sum(1 for w in walls if w[1] == k), sum(1 for c in columns if c[1] == k),
                     sum(1 for b in beams if b[1] == k and '_frame' in b[0]), sum(1 for b in beams if b[1] == k and b[0].endswith('lintel')),
                     sum(1 for b in beams if b[1] == k and '_edge' in b[0]), sum(1 for b in beams if b[1] == k and b[0].endswith('gallery'))) for k in reversed(range(storeys))],
                 profiles=profiles, checks=checks, max_utilization=max_u)
    over = [s for s in spans if s > limit+1e-6]
    unit_weight_t = unit_weight/9.81
    segment_lengths = [c1-c0 for c0, c1 in segs0] or [g0['depth']]
    ties = tie_design(levels, spans, segment_lengths, q_design, min([thickness(key) for key in keys] or [WALL_START[construction][0]]), construction)
    tie_steel = lambda force: force*1000.0/EDGE_FYK
    if material == 'CLT':
        tie_means = 'CLT/glulam: provide these as steel strap/screw connections across every panel joint, along the facades, from floors into walls and through every column splice (connections not designed).'
    else:
        tie_means = 'Concrete: internal ties as continuous slab bars {:.0f} mm2/m along local {} and {:.0f} mm2/m along local {}; peripheral tie {:.0f} mm2 in each facade edge member{}; columns tied vertically for the largest single-storey reaction.'.format(
            tie_steel(ties['along']), span_dir, tie_steel(ties['across']), cross_dir, tie_steel(ties['peripheral']),
            '; vertical wall ties {:.0f} mm2/m continuous through every floor'.format(tie_steel(ties['vertical'])) if ties['vertical'] else '')
    edge_summary = ', '.join('{} x {} ({})'.format(n, sec, kind) for (kind, sec), n in sorted(edge_kinds.items(), key=lambda item: -item[1]))
    def label(i):
        if not (g0['a0']-1e-6 <= positions[i] <= g0['a1']+1e-6):
            return 'hung line (overhang)'
        if line_kind[i] == 'wall':
            return 'P wall {:.2f}m'.format(thickness(('line', i)))
        parts = []
        if (i, 0) in core_cover:
            parts.append('core wall {:.2f}m'.format(thickness(('line', i))))
        if i in max_frame_width:
            parts.append('frame{}'.format(' (inside unit)' if line_kind[i] == 'internal' else ''))
        return ' + '.join(parts) or 'frame'
    column_sides = list(col_side.values())
    overhang_lines = []
    if overhang_text:
        groups_txt = []
        for k in sorted(overhang_text):
            if groups_txt and groups_txt[-1][2] == overhang_text[k] and groups_txt[-1][1] == k-1:
                groups_txt[-1][1] = k
            else:
                groups_txt.append([k, k, overhang_text[k]])
        if overhang_clear[0] < float('inf'):
            overhang_text_clear = ' Lowest clear height under a back-span beam (last bay inside, facade/corridor wall lines): {:.2f} m on storey {} vs target {:.2f} m{}.'.format(overhang_clear[0], overhang_clear[1], TARGET_CLEAR_HEIGHT, '' if overhang_clear[0] >= TARGET_CLEAR_HEIGHT-1e-9 else ' — BELOW TARGET')
        else:
            overhang_text_clear = ''
        overhang_lines.append('OVERHANGS (storeys larger than the storey below; the support box of a storey is the part standing on every storey below, the rest is carried as a cantilever): '+' | '.join(
            'storey {}{}: {}'.format(a, '' if a == b else '-{}'.format(b), txt) for a, b, txt in groups_txt)+'.'+overhang_text_clear+(' Back-span held by the core walls on {} cantilever line(s) (no back-span beam; core walls take V).'.format(core_clamps[0]) if core_clamps[0] else '')+' Largest cantilever root moment {:.0f} kNm (service; transferred into the wall or column at the support-box edge, joint not designed). Cantilever beam deflection is checked on the equivalent 2L simply supported beam with w + 2P/L (5w(2L)^4/384EI = 0.21 wL^4/EI against the cantilever value 0.125 wL^4/EI, limit 2L/300 = L/150): largest deflection utilisation {:.2f}.'.format(overhang_roots[0], max([b['check'].get('U_deflection', 0.0) for b in overhang_beams if '_overhang' in b['name']] or [0.0]))+' Tip edge members span between the cantilever tips along every overhanging long facade (slab edge strip or downstand, carrying the edge strip and the facade).')
        overhang_lines.append('  Method: across the building (beyond a long facade): every support line gets a cantilever beam under the floor, at any length, as long as a section passes (beams first: load-bearing walls must stack; in Timber or Concrete a steel box section is tried as an insert before any wall); only where no beam passes on a party/core or building-end (gable) line, a storey-high cantilever wall ({}) is used as a last resort and flagged OVERHANG WALL (REVIEW) because it does not stack; it is clamped by the party/core wall it continues, or on a gable line it continues inside over the band depth as a gable wall; frame lines only ever get a cantilever beam under the floor, with the frame beam behind it designed for the hogging moment at the column (w = max(w, 8M/b2)); a frame line whose overhang no beam can carry is reported as NOT CLAMPED (move a party wall onto it). Along the building (beyond a gable): hung frame beams and lintels on the extended bay lines rest on cantilever beams in the long facade and corridor lines, each continued over the last bay inside as a back-span beam designed for the cantilever moment (steel insert in Timber/Concrete when the own material does not pass); only if no beam passes, storey-high facade/corridor walls (continued over the last bay). Loads: Floor_Load on the overhanging floor + Roof_Load where nothing is above + facade {:.1f} kN/m2 on the overhang perimeter + self-weight; reactions V + M/b at the support-box edge and -M/b at the next support (frame columns), kept inside the wall for party/core walls. Storey-high walls: lever arm {:.1f} x storey height, chord tension {}, shear vs VRd,max. Not checked: openings in cantilever walls, deflection and vibration of the overhang, local bearing, beam-column joint moment at the root, connections.'.format(
            material, FACADE_LOAD, CANTILEVER_WALL_LEVER,
            'carried by horizontal bars (at most {:.0%} of a chord zone {:.0%} of the height)'.format(CANTILEVER_MAX_STEEL, CANTILEVER_CHORD_SHARE) if material == 'Concrete' else 'in the horizontal CLT layers'))
        if uplift_columns:
            overhang_lines.append('  UPLIFT: {} frame column position(s) end in tension from the overhang back-span couple; tension columns and anchorage needed.'.format(len(uplift_columns)))
    headroom_lines = []
    if min_inside[0] < TARGET_CLEAR_HEIGHT-1e-9:
        wide = defaultdict(list)
        internal = [positions[i] for i in range(len(positions)) if line_kind[i] == 'internal']
        for b in layout['bays']:
            if b['kind'] == 'unit' and any(b['start']+1e-6 < x < b['end']-1e-6 for x in internal):
                wide[b.get('type', 'unit')].append(b['end']-b['start'])
        headroom_lines.append('HEADROOM OPTIONS (frame beams inside apartments give {:.2f} m clear vs {:.2f} m target): (A) storey height {:.2f} m instead of {:.2f} m (+{:.2f} m on every storey with a frame line inside a unit, or a thicker slab with the same total); (B) proposal, not applied (party walls stay only between apartments): units no wider than the {:.2f} m slab span need no frame line inside — the units crossed now are {}; narrower units of these types, or a slab that spans their width (thicker slab or a lighter floor), remove the frame beams from the apartments.'.format(
            min_inside[0], TARGET_CLEAR_HEIGHT, min_inside[1]+TARGET_CLEAR_HEIGHT-min_inside[0], min_inside[1], TARGET_CLEAR_HEIGHT-min_inside[0], limit,
            ', '.join('{} ({} bay(s), {:.2f} m wide)'.format(t, len(w), max(w)) for t, w in sorted(wide.items())) or 'none'))
    lines_out = [
        'STRUCTURAL SYSTEM: WALLS + FRAME LINES — load-bearing {} walls ONLY between apartments (party walls) and around the cores; every other support line (building ends, units wider than the slab span, spare bays) is a frame line: a beam across the band on columns standing in the facade and in the corridor/gallery wall. Slab spans one way along local {} between the support lines.'.format(material, span_dir),
        'Slab {:.3f} m: span limit {:.2f} m ({}); support-line spacing {:.2f}-{:.2f} m{}.'.format(slab, limit, limit_basis, min(spans), max(spans), '' if not over else ' — EXCEEDED on {} bay(s)'.format(len(over))),
        'Support lines (local {}): {}.'.format(span_dir, ', '.join('{:.2f} {}'.format(p, label(i)) for i, p in enumerate(positions))),
        ('Frame lines: {} columns ({} sections, sides at the base {:.2f}-{:.2f} m, smaller higher up), {} beams across the band ({}). Columns sit in the facade and corridor walls, never free-standing in a room; a frame beam crosses the ceiling of units wider than the slab span.'.format(
            len(columns), construction, min(column_sides), max(column_sides), member_count['frame'], ', '.join('{} x {}'.format(n, sec) for sec, n in sorted(frame_sections.items(), key=lambda item: -item[1])))
         +' Frame beams: lightest section that keeps the clear-height target; if none can, the shallowest that works (concrete band beams up to {:.2f} m wide): lowest clear height under a frame beam inside an apartment {}; frame beams on the building-end lines sit in the gable facade (lowest {:.2f} m, no room headroom impact).'.format(
             BAND_BEAM_WIDTHS[-1], 'none (no frame line inside a unit)' if min_inside[0] == float('inf') else '{:.2f} m vs target {:.2f} m{}'.format(min_inside[0], TARGET_CLEAR_HEIGHT, '' if min_inside[0] >= TARGET_CLEAR_HEIGHT-1e-9 else ' — BELOW TARGET'), min_clear)
         +' Columns are never narrower than the beam, lintel or gallery cantilever they carry (concrete band beams wider than deep excepted: punching/torsion at the column not checked); {} column-storeys widened by this rule.'.format(width_governed[0])
         if columns else 'Frame lines: none needed (every support line is a party or core wall).'),
        *headroom_lines,
        ('Corridor: walls and frame columns stop at the {:.1f} m corridor; a lintel beam spans it at every support line and storey (designed as {} beam); the slab spans local {} across the corridor between the lintels. The corridor is the whole circulation zone between the apartment bands, so columns and wall ends sit in the apartment corridor walls.'.format(g0['zone'][1]-g0['zone'][0], construction, span_dir) if g0['typology'] == 'DOUBLE_LOADED_CORRIDOR' else
         'Gallery: walls and columns stop at the {:.1f} m gallery; cantilever beams at every support line carry it (span 2L with w + 2P/L check, designed as {} beams), the gallery slab spans local {} between them.'.format(corridor, construction, span_dir)),
        'Core: cross walls on the two core-bay lines over the core depth (the party wall where that line is one — no duplicate walls) + two longitudinal core walls {} m.'.format(', '.join('{:.2f}'.format(thickness(('core', c))) for c in range(len(core_boxes)))),
        'MEMBERS ALONG LOCAL {0}: no gravity beams are needed because the slab spans local {0}; facade edge members on both long facades in every bay (slab edge strip carrying q x strip width + facade {1:.1f} kN/m2 x storey height; concealed in the slab depth when bending, shear without links and EC2 7.4.2 span/depth pass, otherwise a downstand beam): {2}.'.format(
            span_dir, FACADE_LOAD, edge_summary or 'none'),
        'ROBUSTNESS TIES (EN 1991-1-7 Annex A.5/A.6, consequence class {}): Ft = min(20 + 4 x {} storeys, 60) = {:.0f} kN/m; internal ties Ti = Ft x q/7.5 x z/5 >= Ft: along local {} {:.1f} kN/m, along local {} {:.1f} kN/m; peripheral ties {:.0f} kN{}. q = total service area load (conservative for gk + psi qk). {}'.format(
            ties['cls'], storeys, ties['ft'], span_dir, ties['along'], cross_dir, ties['across'], ties['peripheral'],
            '; vertical wall ties T = 34A/8000 (h/t)^2 >= 100 kN/m: {:.0f} kN/m of wall{}'.format(ties['vertical'], ' (A.6 formula written for masonry/concrete walls; indicative for CLT)' if material == 'CLT' else '') if ties['vertical'] else ' (class 2A: no vertical ties required)',
            tie_means if ties['cls'] != '3' else 'Class 3 (> 15 storeys): a systematic risk assessment is required; ties alone are not sufficient.'),
        'Walls: max utilisation {:.3f} (gravity); volume {:.3f} m3 ({:.1f} t).'.format(max_wall_u, wall_volume, wall_volume*unit_weight_t),
        *stability_report(stability, wind, span_dir, cross_dir, material, grown,
                          'cores as composite box/channel sections (corners continuous) + every party-wall segment as a plane cantilever wall{}; frame lines are pinned (no sway resistance) and braced by the slab diaphragm'.format(
                              ', concrete party-wall pairs coupled over the corridor by the RC lintels' if coupled_lines else ''),
                          ': '+'; '.join((['along the building: more core wall length along local {} (longer/second core)'.format(span_dir)] if stability['results']['a']['worst'] > 1.0+1e-9 else [])
                                         +(['across the building: torsion from an off-centre core overloads the party walls furthest from it — options: a core at the other end or the middle, a concrete core (hybrid) or deeper corridor lintels for more coupling'] if stability['results']['c']['worst'] > 1.0+1e-9 else []))
                          if stability['status'] != 'PASS_ASSUMED_MODEL' and stability.get('results') else ''),
        ('Wall check: CLT, vertical layers {:.0%} of thickness, fc,0,k {:.0f} MPa, E0,05 {:.1f} GPa, kmod {:.2f}, gammaM {:.2f}, beta_c 0.2, l0 = storey height.'.format(
            CLT_VERTICAL_SHARE, CLT_FC0K/1000, CLT_E05/1e6, CLT_KMOD, CLT_GAMMA_M) if material == 'CLT' else
         'Wall check: plain/lightly reinforced concrete wall EN 1992-1-1 12.6.5.2, fcd,pl = {:.2f} fck/gamma_c, e = max(t/30, 20 mm) + l0/400, l0 = storey height.'.format(CONCRETE_WALL_ALPHA)),
        'Loads: walls per metre and frame beams per metre = q x half the bays each side; plus lintel / gallery / facade reactions at the segment ends; self-weight; accumulated from the top; ULS '+'{:.2f}'.format(ULS_GRAVITY)+' x service (EN 1990 6.10b, 70 % permanent assumed). Facades: each floor edge carries the facade of the storey above, gable facades load the end lines, the ground storey sits on the foundation beams. Columns take both frame-beam reactions + end reactions, sized per storey (never smaller than the column above).',
        *foundation_lines,
        *overhang_lines,
        'Wall_Data: '+WALL_DATA_HEADER+' | Walls / Wall_Data branches: {0} core, {1} party, {2} cantilever (storey-high cantilever walls under overhangs: NEd column = chord tension kN, U = max(chord, shear)).',
        'Not checked: wall openings (doors/windows), column/beam connections and joint moments, frame-beam torsion, concentrated loads, fire, acoustics, differential settlement; edge strips: free-edge U-bars, continuity.']
    column_weight = vc*(DENSITY if construction == 'Concrete' else material_properties(construction)['weight'])
    facade_applied = sum(facade_line(k)*(2.0*(geo_full[k+1]['a1']-geo_full[k+1]['a0'])+2.0*(geo_full[k+1]['c1']-geo_full[k+1]['c0'])) for k in range(storeys-1))
    members_weight = sum(v['own']*v['span'] for v in lintel_cache.values())+sum(2.0*v['own']*v['span'] for v in edge_cache.values())+sum(v['own'] for v in gallery_cache.values())+overhang_weight[0]
    applied = floor_total+wall_volume*unit_weight+column_weight+frame_beam_weight+facade_applied+members_weight
    reactions = sum(r[7]*r[4] for r in wall_rows if r[2] == 0)+sum(c[1] for c in base_columns)
    balance = dict(floors=floor_total, walls=wall_volume*unit_weight, columns=column_weight, frame_beams=frame_beam_weight, facade=facade_applied, members=members_weight,
                   applied=applied, reactions=reactions, difference=(reactions-applied)/applied if applied > 0 else 0.0)
    lines_out.append('LOAD BALANCE (service, gravity): floors {:.0f} (Floor_Load under the storey above, Roof_Load on roofs and open terraces) + walls {:.0f} + columns {:.0f} + frame beams {:.0f} + lintels/edge members/galleries {:.0f} + facades {:.0f} = {:.0f} kN applied vs {:.0f} kN at the base of walls and columns ({:+.1%}).'.format(
        floor_total, balance['walls'], column_weight, frame_beam_weight, members_weight, facade_applied, applied, reactions, balance['difference']))
    selected = dict(xs=xs, ys=ys, fx=set(range(len(xs))), fy=set(range(len(ys))), models=[], frame=frame,
                    iteration=0, reserve=0.0, converged=True, removed_lines=0, offset=0.0, supports=[list(p) for p in levels],
                    mode='CROSS_WALL_SYSTEM', targets=(limit, None), limits=(limit, limit), intrusions=0,
                    conflicts=0, facade_hits={})
    base_wall_load = sum(seg[1]*max(seg[6]-seg[5], seg[4]-seg[3]) for seg in base_segments)
    ends = [valid_of(0)[0], valid_of(0)[-1]]
    party_allowance = max([thickness(('line', i)) for i in wall_ids if i not in ends] or [WALL_START[construction][0]])
    end_allowance = max([thickness(('line', i)) for i in wall_ids if i in ends]+[max_frame_width[i] for i in ends if i in max_frame_width] or [party_allowance])
    allowance = (party_allowance, end_allowance)
    return dict(overhang_issues=overhang_issues+(['OVERHANG UPLIFT: {} frame column position(s) in tension from the overhang back-span.'.format(len(uplift_columns))] if uplift_columns else []), selected=selected, walls=walls, wall_rows=out_rows, wall_footings=footings, base_wall_load=base_wall_load,
                wall_volume=wall_volume, wall_mass=wall_volume*unit_weight_t*1000.0, material=material,
                footing_fail=footing_fail, foundation=foundation, lines=lines_out, max_wall_u=max_wall_u, limit=limit,
                ties=ties, member_count=dict(member_count), balance=balance, wall_kind=dict((name, 'gable' if int(name.split('_')[2]) in end_walls and name.split('_')[1] != 'corelong' else kind) for name, k, kind, b in walls),
                allowance=allowance, element_rows=element_rows, pile_rows=pile_rows, columns=len(columns), stability=stability, min_clear=min_inside[0], storey_needed=(min_inside[1]+TARGET_CLEAR_HEIGHT-min_inside[0]) if min_inside[0] < float('inf') else None,
                footing_kinds=footing_kinds, pile_result=pile_result)
# 7d. Pile foundation (Dutch practice: piles, funderingsbalken, poeren, kanaalplaat ground floor)

FOUNDATION_KINDS = ('pile', 'beam', 'poer', 'ground floor')
PILE_CATALOGUE = ((0.38, 1000.0, 'screw displacement pile (Fundex type) 380'), (0.50, 1750.0, 'screw displacement pile (Fundex type) 500'))
PILE_SPACING_FACTOR = 3.0
PILE_MIN_SPACING = 1.0
PILE_MAX_SPACING = 4.0
PILE_CHOICE_TOLERANCE = 1.10
PILE_CLUSTER_MARGIN = 1.10
PILE_TENSION_SHARE = 0.35
PILE_MAX_ROWS = 4
PILE_TENSION = dict(value=None)
CORE_MERGE = 0.5
PILE_BEAM_EDGE = 0.10
PILE_POER_EDGE = 0.15
PEIL_NAP_DEFAULT = 1.40
PILE_TIP_NAP_DEFAULT = -25.5
GF_FINISH = 0.09
GF_BEARING = 0.11
GF_TYPES = ((0.20, 7.5, 2.9, 'kanaalplaat 200'), (0.26, 9.5, 3.8, 'kanaalplaat 260'), (0.32, 11.5, 4.1, 'kanaalplaat 320'), (0.40, 14.0, 5.0, 'kanaalplaat 400'))
FOUNDATION_ULS = ULS_GRAVITY
FB_H_MIN = 0.60
FB_H_TARGET = 1.20
FB_H_MAX = 2.50
FB_COVER = 0.06
FB_MIN_WIDTH = 0.45
POER_H_MIN = 0.80
REBAR_BEAM_LOW = 90.0
REBAR_BEAM_HIGH = 110.0
REBAR_POER = 200.0
HIGH_RISE_STOREYS = 5
PILE_DATA_HEADER = 'pile_id,element,x,y,diameter_m,top_NAP_m,tip_NAP_m,length_m,R_ULS_kN,R_wind_max_kN,R_wind_min_kN,Fr_d_kN,U'
FOUNDATION_ELEMENT_HEADER = 'element_id,kind,b_m,h_m,length_m,piles,span_or_spacing_m,MEd_kNm,MRd_kNm,VEd_kN,VRd_kN,U,volume_m3,rebar_kg,status'


def defaultdict_count(items):
    out = defaultdict(int)
    for item in items:
        out[tuple(round(v, 2) if isinstance(v, float) else v for v in item)] += 1
    return out


def ground_floor_type(span):
    for t, limit, weight, label in GF_TYPES:
        if span <= limit+1e-9:
            return t, weight, label
    t, limit, weight, label = GF_TYPES[-1]
    return t, weight, label+' (span beyond table)'


def foundation_beam_check(b, h, med, ved):
    fcd, fyd, fctm = rc_strengths()
    bb, d = b*1000.0, (h-FB_COVER)*1000.0
    x_lim = 0.45*d
    mrd = 0.8*x_lim*bb*fcd*(d-0.4*x_lim)/1e6
    disc = d*d-2.0*med*1e6/(bb*fcd)
    as_req = bb*(d-math.sqrt(disc))*fcd/fyd if disc >= 0 else float('inf')
    as_prov = max(as_req, max(0.26*fctm/EDGE_FYK, 0.0013)*bb*d)
    vrd = bb*0.9*d*0.6*(1.0-EDGE_FCK/250.0)*fcd/2.0/1000.0
    return dict(MEd=med, MRd=mrd, VEd=ved, VRd=vrd, As=as_prov,
                U=max(med/mrd, ved/vrd, as_prov/(0.04*bb*h*1000.0)))


def punching_depth(reaction, diameter):
    d = 0.30
    while d < 4.0:
        k = min(2.0, 1.0+math.sqrt(200.0/(d*1000.0)))
        v = max(0.18/EDGE_GAMMA_C*k*(100.0*0.005*EDGE_FCK)**(1.0/3.0), 0.035*k**1.5*math.sqrt(EDGE_FCK))
        if v*1000.0*math.pi*(diameter+d)*d >= reaction:
            return d
        d = round(d+0.05, 10)
    return d


def beam_reactions(piles, dist, points):
    n = len(piles)
    reactions = [0.0]*n
    def share(x, load):
        if x <= piles[0]:
            reactions[0] += load
            return
        if x >= piles[-1]:
            reactions[-1] += load
            return
        for j in range(n-1):
            if piles[j] <= x <= piles[j+1]:
                span = piles[j+1]-piles[j]
                reactions[j] += load*(piles[j+1]-x)/span
                reactions[j+1] += load*(x-piles[j])/span
                return
    bounds = [-1e9]+list(piles)+[1e9]
    for ta, tb, w in dist:
        for lo, hi in zip(bounds, bounds[1:]):
            s0, s1 = max(ta, lo), min(tb, hi)
            if s1 > s0+1e-12:
                share((s0+s1)/2.0, w*(s1-s0))
    for x, load in points:
        share(x, load)
    return reactions


def wind_pile_terms(piles, terms):
    extra = [0.0]*len(piles)
    for ta, tb, moment in terms:
        idx = [j for j, p in enumerate(piles) if ta-1e-6 <= p <= tb+1e-6]
        if len(idx) < 2:
            mid = (ta+tb)/2.0
            idx = sorted(range(len(piles)), key=lambda j: abs(piles[j]-mid))[:2]
        if len(idx) < 2:
            continue
        mean = sum(piles[j] for j in idx)/len(idx)
        inertia = sum((piles[j]-mean)**2 for j in idx)
        if inertia <= 1e-9:
            continue
        for j in idx:
            extra[j] += abs(moment)*abs(piles[j]-mean)/inertia
    return extra


def tension_capacity(capacity):
    return PILE_TENSION['value'] or PILE_TENSION_SHARE*capacity


def pile_line(t0, t1, fixed, dist, points, wind, diameter, capacity, min_width):
    smin = max(PILE_SPACING_FACTOR*diameter, PILE_MIN_SPACING)
    base_fixed = sorted(f for f in fixed if t0-1e-9 <= f <= t1+1e-9)
    clusters = []
    def arrange(rows):
        fx, pts, groups = list(base_fixed), [], []
        limit = max(1, int((t1-t0)/smin+1e-9)+1)
        def need(load):
            return min(limit, max(1, int(math.ceil(PILE_CLUSTER_MARGIN*FOUNDATION_ULS*load/(rows*capacity)-1e-9)))) if load > 0 else 1
        def spots_of(x, m):
            span = (m-1)*smin
            lo = max(t0, min(x-span/2.0, t1-span))
            return [lo+k*smin for k in range(m)]
        heavy = sorted([(x, load) for x, load in points if need(load) > 1])
        pts = [(x, load) for x, load in points if need(load) <= 1]
        merged = []
        for x, load in heavy:
            if merged:
                mx, ml, members = merged[-1]
                prev = spots_of(mx, need(ml))
                mine = spots_of(x, need(load))
                if mine[0] < prev[-1]+smin-1e-6:
                    total = ml+load
                    merged[-1] = ((mx*ml+x*load)/total, total, members+[x])
                    continue
            merged.append((x, load, [x]))
        for x, load, members in merged:
            m = need(load)
            spots = spots_of(x, m)
            fx = [f for f in fx if all(abs(f-mx) > 1e-6 for mx in members)]+spots
            pts += [(xx, load/m) for xx in spots]
            groups.append((x, m))
        kept = []
        for f in sorted(fx):
            if not kept or f-kept[-1] >= smin-1e-6:
                kept.append(f)
        ends = [e for e in (t0, t1) if all(abs(e-f) >= smin-1e-9 for f in kept)]
        start = sorted(set(round(v, 9) for v in kept+ends))
        filled = []
        for p0, p1 in zip(start, start[1:]):
            parts = int(math.ceil((p1-p0)/PILE_MAX_SPACING-1e-9))
            filled += [p0+(p1-p0)*k/parts for k in range(parts)]
        state['protected'] = set(round(v, 6) for v in kept)
        return (filled+start[-1:] if start else start), pts, groups
    state = {}
    def evaluate(piles, rows):
        service = beam_reactions(piles, dist, state['pts'])
        extra = wind_pile_terms(piles, wind)
        uls = [FOUNDATION_ULS*r for r in service]
        wmax = [WIND_GRAVITY_FACTOR*r+x for r, x in zip(service, extra)]
        wmin = [0.9*PERMANENT_SHARE*r-x for r, x in zip(service, extra)]
        u_comp = max(max(u, w) for u, w in zip(uls, wmax))/(rows*capacity)
        u_tens = max([0.0]+[-w for w in wmin])/(rows*tension_capacity(capacity))
        return service, uls, wmax, wmin, max(u_comp, u_tens)
    def longest_gap(piles):
        gaps = [(piles[j+1]-piles[j], j) for j in range(len(piles)-1) if piles[j+1]-piles[j] >= 2.0*smin-1e-9]
        return max(gaps) if gaps else None
    options = []
    for rows in range(1, PILE_MAX_ROWS+1):
        start, state['pts'], groups = arrange(rows)
        piles = list(start)
        while True:
            service, uls, wmax, wmin, u = evaluate(piles, rows)
            if u <= 1.0+1e-9:
                break
            worst = max(range(len(piles)), key=lambda j: max(uls[j], wmax[j]))
            near = [(piles[j+1]-piles[j], j) for j in (worst-1, worst) if 0 <= j < len(piles)-1 and piles[j+1]-piles[j] >= 2.0*smin-1e-9]
            gap = max(near) if near else longest_gap(piles)
            if gap is None:
                break
            piles.insert(gap[1]+1, (piles[gap[1]]+piles[gap[1]+1])/2.0)
        if u <= 1.0+1e-9:
            j = len(piles)-2
            while j >= 1:
                if round(piles[j], 6) not in state['protected'] and piles[j+1]-piles[j-1] <= PILE_MAX_SPACING+1e-9:
                    trial = piles[:j]+piles[j+1:]
                    if evaluate(trial, rows)[4] <= 1.0+1e-9:
                        piles = trial
                j -= 1
            u = evaluate(piles, rows)[4]
        options.append((u > 1.0+1e-9, rows*len(piles), rows, list(piles), state['pts'], groups, u))
    passing = [o for o in options if not o[0]]
    chosen = min(passing, key=lambda o: (o[1], o[2])) if passing else options[-1]
    failed, count, rows, piles, state['pts'], groups, u = chosen
    clusters[:] = groups
    width = max(min_width, FB_MIN_WIDTH, diameter+2.0*PILE_BEAM_EDGE+(rows-1)*smin)
    while True:
        service, uls, wmax, wmin, u = evaluate(piles, rows)
        med, ved = 0.0, max(max(uls), max(wmax))/2.0
        spans = list(zip(piles, piles[1:]))
        for p0, p1 in spans:
            span = p1-p0
            w = sum(FOUNDATION_ULS*ww*max(0.0, min(tb, p1)-max(ta, p0)) for ta, tb, ww in dist)/span
            factor = 10.0 if len(spans) > 1 else 8.0
            m = w*span**2/factor+sum(FOUNDATION_ULS*load*(x-p0)*(p1-x)/span for x, load in state['pts'] if p0 < x < p1)
            med = max(med, m)
            ved = max(ved, w*span/2.0+sum(FOUNDATION_ULS*load for x, load in state['pts'] if p0 < x < p1))
        for edge, sign in ((piles[0], -1), (piles[-1], 1)):
            reach = (t0 if sign < 0 else t1)
            arm = abs(edge-reach)
            if arm > 1e-6:
                w = max([FOUNDATION_ULS*ww for ta, tb, ww in dist if min(tb, max(edge, reach))-max(ta, min(edge, reach)) > 1e-9] or [0.0])
                med = max(med, w*arm**2/2.0)
        height, check = max(FB_H_MIN, up(punching_depth(max(max(uls), max(wmax))/rows, diameter)+FB_COVER+0.04)), None
        while height <= FB_H_MAX+1e-9:
            check = foundation_beam_check(width, height, med, ved)
            if check['U'] <= 1.0:
                break
            height = round(height+0.05, 10)
        gap = longest_gap(piles)
        if (check['U'] > 1.0 or height > FB_H_TARGET+1e-9) and gap is not None:
            piles.insert(gap[1]+1, (piles[gap[1]]+piles[gap[1]+1])/2.0)
            continue
        break
    spacing = max([b-a for a, b in zip(piles, piles[1:])] or [0.0])
    return dict(piles=piles, rows=rows, smin=smin, width=width, height=height, check=check, uls=uls, wmax=wmax, wmin=wmin,
                u_pile=u, spacing=spacing, clusters=list(clusters), status='PASS_ASSUMED_MODEL' if u <= 1.0+1e-9 and check['U'] <= 1.0 else 'FAIL')


def pile_group(box, n_service, moments, diameter, capacity, min_h=POER_H_MIN, grow=(0.0, 0.3, 0.0, 0.3), limits=None):
    smin = max(PILE_SPACING_FACTOR*diameter, PILE_MIN_SPACING)
    edge = diameter/2.0+PILE_POER_EDGE
    x0, x1, y0, y1 = box
    height = min_h
    for attempt in range(40):
        lx, ly = x1-x0, y1-y0
        found = None
        if lx >= 2.0*edge-1e-9 and ly >= 2.0*edge-1e-9:
            nx_max = max(1, int((lx-2.0*edge)/smin+1e-9)+1)
            ny_max = max(1, int((ly-2.0*edge)/smin+1e-9)+1)
            weight = DENSITY*lx*ly*height
            total = n_service+weight
            for count, nx, ny in sorted((nx*ny, nx, ny) for nx in range(1, nx_max+1) for ny in range(1, ny_max+1)):
                px = [x0+edge+(lx-2.0*edge)*i/(nx-1) for i in range(nx)] if nx > 1 else [(x0+x1)/2.0]
                py = [y0+edge+(ly-2.0*edge)*j/(ny-1) for j in range(ny)] if ny > 1 else [(y0+y1)/2.0]
                mx, my = sum(px)/nx, sum(py)/ny
                ix = ny*sum((p-mx)**2 for p in px)
                iy = nx*sum((p-my)**2 for p in py)
                worst_max, worst_min, ok = FOUNDATION_ULS*total/count, 0.0, True
                for m_x, m_y in moments:
                    if (abs(m_x) > 1e-6 and ix <= 1e-9) or (abs(m_y) > 1e-6 and iy <= 1e-9):
                        ok = False
                        break
                    dx = abs(m_x)*max(abs(p-mx) for p in px)/ix if ix > 1e-9 else 0.0
                    dy = abs(m_y)*max(abs(p-my) for p in py)/iy if iy > 1e-9 else 0.0
                    worst_max = max(worst_max, WIND_GRAVITY_FACTOR*total/count+dx+dy)
                    worst_min = min(worst_min, 0.9*PERMANENT_SHARE*total/count-dx-dy)
                if ok and worst_max <= capacity+1e-9 and -worst_min <= tension_capacity(capacity)+1e-9:
                    found = (count, px, py, worst_max, worst_min, total)
                    break
        if found:
            need = max(min_h, up(punching_depth(found[3], diameter)+FB_COVER+0.04))
            if need > height+1e-9:
                height = need
                continue
            count, px, py, rmax, rmin, total = found
            return dict(box=(x0, x1, y0, y1), height=height, piles=[(x, y) for x in px for y in py], rmax=rmax, rmin=rmin,
                        total=total, smin=smin, status='PASS_ASSUMED_MODEL')
        x0, x1, y0, y1 = x0-grow[0], x1+grow[1], y0-grow[2], y1+grow[3]
        if limits:
            x0, x1, y0, y1 = max(x0, limits[0]), min(x1, limits[1]), max(y0, limits[2]), min(y1, limits[3])
    return dict(box=(x0, x1, y0, y1), height=height, piles=[((x0+x1)/2.0, (y0+y1)/2.0)], rmax=float('inf'), rmin=0.0,
                total=n_service, smin=smin, status='FAIL')


def pile_options(cfg):
    if cfg.get('diameter'):
        d = cfg['diameter']
        cap = cfg.get('capacity') or next((c for dd, c, n in PILE_CATALOGUE if abs(dd-d) < 1e-6), None)
        if not cap:
            raise ValueError('Pile_Capacity (Fr;d per pile, kN) is required for a pile diameter outside the catalogue ({}).'.format(
                ', '.join('{:.0f} mm'.format(dd*1000) for dd, c, n in PILE_CATALOGUE)))
        label = next((n for dd, c, n in PILE_CATALOGUE if abs(dd-d) < 1e-6), 'screw displacement pile {:.0f}'.format(d*1000))
        return [(d, cap, label)]
    if cfg.get('capacity'):
        return [(dd, cfg['capacity'], n) for dd, c, n in PILE_CATALOGUE]
    return list(PILE_CATALOGUE)


def foundation_summary_lines(res, cfg):
    q = res['quantities']
    lines = [
        'FOUNDATION: PILED (Dutch practice, as in the reference projects) — {} {} piles, {} on {} funderingsbalken and {} poer(en); ground floor {} spanning between the beams over a crawl space (not ground-bearing). '.format(
            q['piles'], res['label'], 'Fr;d {:.0f} kN each'.format(res['capacity']), q['beams'], q['poers'], res['gf_label']),
        '  (a) GROUND FLOOR: {} d = {:.0f} mm, max span {:.2f} m between beam faces, area {:.1f} m2 (gross volume {:.1f} m3); top at PEIL -{:.2f} m finish, bearing on the beams.'.format(
            res['gf_label'], res['gf_t']*1000, res['gf_span'], q['gf_m2'], q['gf_m3'], GF_FINISH),
        '  (b) PILES: {} x {} (D {:.0f} mm, Fr;d {:.0f} kN{}), tip {:.2f} m NAP, PEIL {:+.2f} m NAP, length {:.1f}-{:.1f} m, total {:.1f} m1 ({:.1f} m3); max ULS reaction {:.0f} kN (U {:.2f}); spacing >= {:.2f} m (3D); {}.'.format(
            q['piles'], res['label'], res['diameter']*1000, res['capacity'], ', from Pile_Capacity' if cfg.get('capacity') else ', reference-project value',
            cfg['tip'], cfg['peil'], q['len_min'], q['len_max'], q['pile_m1'], q['pile_m3'], q['r_max'], q['u_pile'], res['smin'],
            'TENSION in {} pile(s) under wind (min {:.0f} kN) checked against Fr;t;d {:.0f} kN ({}), U {:.2f}; pile self-weight ignored (conservative)'.format(
                q['tension'], q['r_min'], tension_capacity(res['capacity']), 'Pile_Tension_Capacity input' if PILE_TENSION['value'] else 'ASSUMED {:.0%} of Fr;d — enter Pile_Tension_Capacity from the CPTs'.format(PILE_TENSION_SHARE), q.get('u_tension', 0.0))
            if q['tension'] else 'no pile in tension'),
        '  (c) FUNDERINGSBALKEN: {} beams, {:.1f} m1, {:.1f} m3 ({}); POEREN: {} ({:.1f} m3); rebar indication {:.0f} kg (beams {:.0f} kg/m3, poeren {:.0f} kg/m3).'.format(
            q['beams'], q['beam_m1'], q['beam_m3'], ', '.join('{} x {:.0f}x{:.0f}'.format(n, b*1000, h*1000) for (b, h), n in sorted(q['beam_sizes'].items(), key=lambda item: -item[1])),
            ', '.join('{} x {:.2f}x{:.2f}x{:.2f} m ({} piles)'.format(n, *p) for p, n in sorted(defaultdict_count(q['poer_list']).items(), key=lambda item: -item[1])) or 'none', q['poer_m3'], q['rebar'], q['rebar_beam'], REBAR_POER),
        '  Method: loads from the walls/columns at ground level + ground floor (Floor_Load on the slab spans) + facade at ground storey + self-weight; ULS 6.10b {:.2f} x service (70 % permanent); wind case {:.2f} x service + 1.5 W overturning from the stability elements (pile rows under each party wall / pile group under each core). Piles added where the worst pile exceeds Fr;d, up to {} rows when 3D spacing is exhausted; a column load larger than one pile position can carry gets a cluster of positions at 3D centred on the column (rigid local cap, load shared equally); beams continuous over the piles (M = wL2/10, point loads PAB/L), bending x/d <= 0.45 and VRd,max (links); depth from {:.2f} m in 0.05 m steps, extra piles if a beam would exceed {:.2f} m; poer depth from pile punching (vRd,c, rho 0.5 %) >= {:.2f} m.'.format(
            FOUNDATION_ULS, WIND_GRAVITY_FACTOR, PILE_MAX_ROWS, FB_H_MIN, FB_H_TARGET, POER_H_MIN),
        '  Pile capacity and tip level are inputs from the CPTs / geotechnical advice (NEN 9997-1); negative skin friction, settlement, pile group effects and horizontal pile loads are not checked. Foundation_Data = piles ({}); Foundation_Check_Data = elements ({}).'.format(PILE_DATA_HEADER, FOUNDATION_ELEMENT_HEADER)]
    return lines


def assemble_foundation(cfg, diameter, capacity, label, smin, beams, poers, panels, gf, storeys, plan_fn, base_z, issues_out):
    top_depth = GF_FINISH+gf[0]+GF_BEARING
    footings, kinds, pile_rows, element_rows = [], [], [], []
    rebar_rate = REBAR_BEAM_HIGH if storeys >= HIGH_RISE_STOREYS else REBAR_BEAM_LOW
    q = dict(piles=0, pile_m1=0.0, pile_m3=0.0, beams=0, beam_m1=0.0, beam_m3=0.0, poers=0, poer_m3=0.0, poer_list=[], beam_sizes=defaultdict(int),
             rebar=0.0, rebar_beam=rebar_rate, r_max=0.0, u_pile=0.0, tension=0, r_min=0.0, len_min=1e9, len_max=0.0)
    status = 'PASS_ASSUMED_MODEL'
    def add_pile(x, y, under, depth, r_uls, r_wmax, r_wmin, rows_capacity):
        top = cfg['peil']-depth
        length = top-cfg['tip']
        if length <= 0:
            raise ValueError('Pile_Tip_NAP ({:.2f}) must lie below the pile head ({:.2f} m NAP).'.format(cfg['tip'], top))
        n = q['piles']
        q['piles'] += 1
        q['pile_m1'] += length
        q['pile_m3'] += math.pi*diameter**2/4.0*length
        q['len_min'], q['len_max'] = min(q['len_min'], length), max(q['len_max'], length)
        u_tension = max(0.0, -r_wmin)/tension_capacity(rows_capacity)
        u = max(max(r_uls, r_wmax)/rows_capacity, u_tension)
        q['r_max'] = max(q['r_max'], max(r_uls, r_wmax))
        q['u_pile'] = max(q['u_pile'], u)
        if r_wmin < 0:
            q['tension'] += 1
            q['r_min'] = min(q['r_min'], r_wmin)
            q['u_tension'] = max(q.get('u_tension', 0.0), u_tension)
        bottom = base_z-depth
        box = plan_fn(x-diameter/2.0, x+diameter/2.0, y-diameter/2.0, y+diameter/2.0, bottom-length, bottom)
        footings.append(('P_{}'.format(n), -1, box))
        kinds.append('pile')
        pile_rows.append('P_{},{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.1f},{:.1f},{:.1f},{:.1f},{:.3f}'.format(
            n, under, (box[0]+box[1])/2.0, (box[2]+box[3])/2.0, diameter, top, cfg['tip'], length, r_uls, r_wmax, r_wmin, capacity, u))
    for beam in beams:
        res = beam['result']
        b, h = res['width'], res['height']
        depth = top_depth+h
        length = beam['t1']-beam['t0']
        vol = b*h*length
        q['beams'] += 1
        q['beam_m1'] += length
        q['beam_m3'] += vol
        q['beam_sizes'][round(b, 2), round(h, 2)] += 1
        q['rebar'] += vol*rebar_rate
        lo, hi = beam['across'](b)
        footings.append((beam['name'], -1, plan_fn(lo, hi, beam['t0'], beam['t1'], base_z-depth, base_z-top_depth) if beam['axis'] == 'c' else
                         plan_fn(beam['t0'], beam['t1'], lo, hi, base_z-depth, base_z-top_depth)))
        kinds.append('beam')
        ch = res['check']
        if res['status'] != 'PASS_ASSUMED_MODEL':
            status = 'FAIL'
            issues_out.append('FOUNDATION BEAM {} FAILS (pile U {:.2f}, beam U {:.2f}).'.format(beam['name'], res['u_pile'], ch['U']))
        element_rows.append('{},beam,{:.3f},{:.3f},{:.3f},{},{:.3f},{:.1f},{:.1f},{:.1f},{:.1f},{:.3f},{:.3f},{:.0f},{}'.format(
            beam['name'], b, h, length, len(res['piles'])*res['rows'] if res['piles'] else 0, res['spacing'], ch['MEd'], ch['MRd'], ch['VEd'], ch['VRd'], max(ch['U'], res['u_pile']), vol, vol*rebar_rate, res['status']))
        if res['piles']:
            cross = beam['centre']
            for j, t in enumerate(res['piles']):
                for r in range(res['rows']):
                    off = cross+(r-(res['rows']-1)/2.0)*res['smin']
                    x, y = (off, t) if beam['axis'] == 'c' else (t, off)
                    add_pile(x, y, beam['name'], depth, res['uls'][j]/res['rows'], res['wmax'][j]/res['rows'], res['wmin'][j]/res['rows'], capacity)
    for poer in poers:
        res = poer['result']
        x0, x1, y0, y1 = res['box']
        h = res['height']
        vol = (x1-x0)*(y1-y0)*h
        q['poers'] += 1
        q['poer_m3'] += vol
        q['rebar'] += vol*REBAR_POER
        q['poer_list'].append((x1-x0, y1-y0, h, len(res['piles'])))
        footings.append((poer['name'], -1, plan_fn(x0, x1, y0, y1, base_z-top_depth-h, base_z-top_depth)))
        kinds.append('poer')
        if res['status'] != 'PASS_ASSUMED_MODEL':
            status = 'FAIL'
            issues_out.append('POER {} FAILS: no pile layout within the search fits Fr;d {:.0f} kN.'.format(poer['name'], capacity))
        n = max(1, len(res['piles']))
        element_rows.append('{},poer,{:.3f},{:.3f},{:.3f},{},{:.3f},,,,,{:.3f},{:.3f},{:.0f},{}'.format(
            poer['name'], x1-x0, h, y1-y0, len(res['piles']), res['smin'], res['rmax']/capacity, vol, vol*REBAR_POER, res['status']))
        for x, y in res['piles']:
            add_pile(x, y, poer['name'], top_depth+h, FOUNDATION_ULS*res['total']/n, res['rmax'], res['rmin'], capacity)
    gf_m2 = 0.0
    for name, x0, x1, y0, y1 in panels:
        if x1-x0 <= 1e-6 or y1-y0 <= 1e-6:
            continue
        gf_m2 += (x1-x0)*(y1-y0)
        footings.append((name, -1, plan_fn(x0, x1, y0, y1, base_z-GF_FINISH-gf[0], base_z-GF_FINISH)))
        kinds.append('ground floor')
    q['gf_m2'], q['gf_m3'] = gf_m2, gf_m2*gf[0]
    element_rows.append('ground_floor,slab,,{:.3f},,0,{:.3f},,,,,,{:.3f},0,PASS_ASSUMED_MODEL'.format(gf[0], gf[3], gf_m2*gf[0]))
    q['beam_sizes'] = dict(q['beam_sizes'])
    if q['tension']:
        issues_out.append('TENSION PILES: {} pile(s) pull out under wind (min {:.0f} kN, U {:.2f} vs Fr;t;d {:.0f} kN{}); tension reinforcement and anchorage into the beams/poeren needed.'.format(
            q['tension'], q['r_min'], q.get('u_tension', 0.0), tension_capacity(capacity), '' if PILE_TENSION['value'] else ', ASSUMED — enter Pile_Tension_Capacity'))
    if q['piles'] == 0:
        q['len_min'] = 0.0
    return dict(footings=footings, kinds=kinds, pile_rows=pile_rows, element_rows=element_rows, quantities=q, status=status,
                diameter=diameter, capacity=capacity, label=label, smin=smin, gf_t=gf[0], gf_span=gf[3], gf_label=gf[2])


def choose_pile_design(cfg, design):
    results = [design(diameter, capacity, label) for diameter, capacity, label in pile_options(cfg)]
    passing = [r for r in results if r['status'] == 'PASS_ASSUMED_MODEL'] or results
    def volume(r):
        q = r['quantities']
        return q['pile_m3']+q['beam_m3']+q['poer_m3']
    best = min(volume(r) for r in passing)
    near = [r for r in passing if volume(r) <= PILE_CHOICE_TOLERANCE*best+1e-9]
    return min(near, key=lambda r: (r['quantities']['piles'], volume(r)))


def design_frame_piles(levels, xs, ys, loads, sides, floor, cfg, cores=None):
    base = levels[0]
    base_z = base[4]
    storeys = len(levels)
    along = long_axis(levels)
    nodes = sorted(loads)
    height0 = base[5]-base[4]
    def plan_fn(x0, x1, y0, y1, z0, z1):
        return (x0, x1, y0, y1, z0, z1)
    span_coords = xs if along == 0 else ys
    spacing = max([b-a for a, b in zip(span_coords, span_coords[1:])] or [0.0])
    def design(diameter, capacity, label):
        issues = []
        gf_span = max(spacing-FB_MIN_WIDTH, 0.0)
        gf_t, gf_w, gf_label = ground_floor_type(gf_span)
        node_load = defaultdict(float)
        for n in nodes:
            node_load[n] += loads[n]
        beams = []
        node_set = set(nodes)
        for axis_dir in ('x', 'y'):
            for (i, j) in nodes:
                nxt = (i+1, j) if axis_dir == 'x' else (i, j+1)
                if nxt not in node_set:
                    continue
                p0 = xs[i] if axis_dir == 'x' else ys[j]
                p1 = xs[nxt[0]] if axis_dir == 'x' else ys[nxt[1]]
                L = p1-p0
                carries_floor = (axis_dir == 'x') != (along == 0)
                if carries_floor:
                    lines = xs if axis_dir == 'y' else ys
                    k = i if axis_dir == 'y' else j
                    trib = ((lines[k]-lines[k-1]) if k > 0 else 0.0)/2.0+((lines[k+1]-lines[k]) if k+1 < len(lines) else 0.0)/2.0
                    w = floor*trib
                else:
                    w = 0.0
                perimeter = (axis_dir == 'x' and (j == 0 or j == len(ys)-1)) or (axis_dir == 'y' and (i == 0 or i == len(xs)-1))
                if perimeter:
                    w += FACADE_LOAD*height0
                width = max(FB_MIN_WIDTH, sides.get((i, j), 0.0)+0.1)
                h = FB_H_MIN
                while True:
                    ws = w+DENSITY*width*h
                    check = foundation_beam_check(width, h, FOUNDATION_ULS*ws*L**2/8.0, FOUNDATION_ULS*ws*L/2.0)
                    if check['U'] <= 1.0 or h >= FB_H_MAX:
                        break
                    h = round(h+0.05, 10)
                node_load[i, j] += ws*L/2.0
                node_load[nxt] += ws*L/2.0
                centre = ys[j] if axis_dir == 'x' else xs[i]
                beams.append(dict(name='FB_{}_{}_{}'.format(axis_dir, i, j), axis='a' if axis_dir == 'x' else 'c', t0=p0, t1=p1, centre=centre,
                                  across=(lambda bw, c=centre: (c-bw/2.0, c+bw/2.0)),
                                  result=dict(piles=[], rows=1, smin=0.0, width=width, height=h, check=check, uls=[], wmax=[], wmin=[], u_pile=0.0,
                                              spacing=L, status='PASS_ASSUMED_MODEL' if check['U'] <= 1.0 else 'FAIL')))
        poers = []
        merged = set()
        for core in (cores or []):
            x0, x1, y0, y1 = core['box']
            inside = [n for n in nodes if x0-CORE_MERGE <= xs[n[0]] <= x1+CORE_MERGE and y0-CORE_MERGE <= ys[n[1]] <= y1+CORE_MERGE]
            merged.update(inside)
            load = core['N']+sum(node_load[n] for n in inside)
            lo_x = min([x0]+[xs[n[0]] for n in inside])
            hi_x = max([x1]+[xs[n[0]] for n in inside])
            lo_y = min([y0]+[ys[n[1]] for n in inside])
            hi_y = max([y1]+[ys[n[1]] for n in inside])
            res = pile_group((lo_x-0.3, hi_x+0.3, lo_y-0.3, hi_y+0.3), load, core['moments'], diameter, capacity, grow=(0.3, 0.3, 0.3, 0.3))
            poers.append(dict(name='FP_core{}'.format(core['c']), result=res))
        for (i, j) in nodes:
            if (i, j) in merged:
                continue
            side = max(sides.get((i, j), 0.3)+0.2, diameter+2.0*PILE_POER_EDGE)
            smin = max(PILE_SPACING_FACTOR*diameter, PILE_MIN_SPACING)
            need = max(1, int(math.ceil(FOUNDATION_ULS*node_load[i, j]*1.05/capacity-1e-9)))
            nx = int(math.ceil(math.sqrt(need)-1e-9))
            ny = int(math.ceil(need/float(nx)-1e-9))
            sx = max(side, (nx-1)*smin+diameter+2.0*PILE_POER_EDGE)
            sy = max(side, (ny-1)*smin+diameter+2.0*PILE_POER_EDGE)
            if (along == 0) != (sx >= sy):
                sx, sy = sy, sx
            box = (xs[i]-sx/2.0, xs[i]+sx/2.0, ys[j]-sy/2.0, ys[j]+sy/2.0)
            res = pile_group(box, node_load[i, j], [], diameter, capacity, grow=(0.3, 0.3, 0.3, 0.3))
            poers.append(dict(name='FP_{}_{}'.format(i, j), result=res))
        panels = []
        for a, b in zip(xs, xs[1:]):
            for c, d in zip(ys, ys[1:]):
                panels.append(('GF_{:.2f}_{:.2f}'.format(a, c), a+FB_MIN_WIDTH/2.0, b-FB_MIN_WIDTH/2.0, c+FB_MIN_WIDTH/2.0, d-FB_MIN_WIDTH/2.0))
        result = assemble_foundation(cfg, diameter, capacity, label, max(PILE_SPACING_FACTOR*diameter, PILE_MIN_SPACING), beams, poers, panels,
                                     (gf_t, gf_w, gf_label, gf_span), storeys, plan_fn, base_z, issues)
        result['issues'] = issues
        return result
    return choose_pile_design(cfg, design)


def design_frame_cores(levels, layout, construction, wind, gravity, members):
    per_storey = defaultdict(list)
    for (name, k, b), kind in zip(layout.get('supports', []), layout.get('support_kinds', [])):
        if kind == 'core':
            per_storey[k].append(b)
    if not per_storey.get(0):
        return None
    storeys = len(levels)
    material = wall_material(construction)
    unit = wall_unit_weight(construction)
    cores = list(range(len(per_storey[0])))
    options = wall_thicknesses(construction, 'core')
    chosen = dict((c, 0) for c in cores)
    def pieces(box, t):
        x0, x1, y0, y1 = box[0], box[1], box[2], box[3]
        return [(x0, x1, y0, y0+t), (x0, x1, y1-t, y1), (x0, x0+t, y0+t, y1-t), (x1-t, x1, y0+t, y1-t)]
    def present(c):
        return [k for k in range(storeys) if c < len(per_storey.get(k, []))]
    def core_weight(c):
        t = options[chosen[c]]
        return sum(sum((r[1]-r[0])*(r[3]-r[2]) for r in pieces(per_storey[k][c], t))*(levels[k][5]-levels[k][4])*unit for k in present(c))
    base = levels[0]
    faces = [(p[4]-base[4], p[5]-base[4], p[3]-p[2], p[1]-p[0]) for p in levels]
    def check():
        rects, groups = {}, []
        for c in cores:
            t = options[chosen[c]]
            ps = pieces(per_storey[0][c], t)
            area = sum((r[1]-r[0])*(r[3]-r[2]) for r in ps)
            w = core_weight(c)
            keys = []
            for n, r in enumerate(ps):
                rects['core', c, n] = (r[0], r[1], r[2], r[3], w*(r[1]-r[0])*(r[3]-r[2])/area)
                keys.append(('core', c, n))
            groups.append(('core {}'.format(c), keys))
        total = gravity+sum(core_weight(c) for c in cores)
        return lateral_check(levels, wall_elements(rects, groups), material, total, members+len(cores), (base[0], base[1], base[2], base[3]), wind, faces)
    stability = check()
    start = dict(chosen)
    steps = 0
    while stability['status'] != 'PASS_ASSUMED_MODEL' and steps < 200:
        steps += 1
        best = None
        for c in cores:
            if chosen[c]+1 >= len(options):
                continue
            before = core_weight(c)
            chosen[c] += 1
            added = core_weight(c)-before
            trial = check()
            chosen[c] -= 1
            gain = stability['worst']-trial['worst']
            if gain <= 1e-6:
                continue
            score = gain/max(added, 1e-9)
            if best is None or score > best[0]:
                best = (score, c, trial)
        if best is None:
            break
        chosen[best[1]] += 1
        stability = best[2]
    grown = ['core {} walls {:.2f}->{:.2f} m'.format(c, options[start[c]], options[chosen[c]]) for c in cores if chosen[c] != start[c]]
    walls, rows, volume, max_u = [], [], 0.0, 0.0
    for c in cores:
        t = options[chosen[c]]
        above = 0.0
        for k in reversed(present(c)):
            p = levels[k]
            height = p[5]-p[4]
            above += t*height*unit
            check_k = wall_check(construction, t, height, above)
            max_u = max(max_u, check_k['U'])
            ps = pieces(per_storey[k][c], t)
            length = sum(max(r[1]-r[0], r[3]-r[2]) for r in ps)
            for n, r in enumerate(ps):
                walls.append(('W_core_{}_{}_{}'.format(c, n, k), k, 'core', (r[0], r[1], r[2], r[3], p[4], p[5])))
                volume += (r[1]-r[0])*(r[3]-r[2])*height
            box = per_storey[k][c]
            rows.append('W_core_{}_{},core,{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.4f},{}'.format(
                c, k, k, (box[0]+box[1])/2.0, length, t, height, above, check_k['NEd'], check_k['NRd'], check_k['U'], material))
    per = stability.get('per', {})
    foundation = [dict(c=c, box=tuple(per_storey[0][c][:4]), N=core_weight(c), moments=list(per.get('core {}'.format(c), {}).get('M', []))) for c in cores]
    mass = volume*unit/9.81*1000.0
    lines = stability_report(stability, wind, 'X', 'Y', material, grown,
                             '{} {} core(s) from the programme as closed box sections (4 walls, corners continuous, door openings ignored); the {} frame is pinned and braced by the slab diaphragm'.format(len(cores), material, construction),
                             ': more or longer core walls (a third core, or core walls along the facade)' if stability['status'] != 'PASS_ASSUMED_MODEL' else '')
    lines.insert(0, 'CORES (frame mode stability system): {} {:.1f} m3 ({:.1f} t), wall thickness {}; the cores carry their own weight only (floors stay on the frame) and all wind; gravity check max U {:.3f}. Core poeren in the FOUNDATION block take the core weight, the frame columns standing inside the core and the overturning.'.format(
        material, volume, mass/1000.0, ', '.join('core {} {:.2f} m'.format(c, options[chosen[c]]) for c in cores), max_u))
    return dict(walls=walls, rows=rows, volume=volume, mass=mass, material=material, stability=stability, lines=lines, foundation=foundation, max_u=max_u)


# 7d2. Quantity take-off

INTERNAL_WALL_BASE = 4.0
INTERNAL_WALL_PER_BEDROOM = 4.5
INTERNAL_WALL_PER_M2 = 0.05
INTERNAL_WALL_BEDROOMS = {'studio': 0, 'penthouse': 3}
TAKEOFF_HEADER = 'category,item,dimensions,count,m1,m2,m3'


def bedrooms_of(name, area):
    key = name.strip().lower()
    for word, n in INTERNAL_WALL_BEDROOMS.items():
        if word in key:
            return n
    digits = [int(part) for part in key.replace('-', ' ').split() if part.isdigit()]
    if digits and 'bed' in key:
        return digits[0]
    return max(0, int(round(area/25.0))-1)


def internal_wall_length(name, area):
    return INTERNAL_WALL_BASE+INTERNAL_WALL_PER_BEDROOM*bedrooms_of(name, area)+INTERNAL_WALL_PER_M2*area


def quantity_takeoff(levels, slab, foundation_rows, pile_rows, wall_rows, column_rows, beam_rows, beam_names, check_rows, apartment_rows):
    rows = []
    def add(category, item, dims, count, m1=0.0, m2=0.0, m3=0.0):
        rows.append('{},{},{},{},{:.3f},{:.3f},{:.3f}'.format(category, item, dims, count, m1, m2, m3))
    beams_fb = defaultdict(lambda: [0, 0.0, 0.0])
    for r in foundation_rows:
        f = r.split(',')
        if len(f) > 4 and f[1] == 'beam':
            b, h, length = float(f[2]), float(f[3]), float(f[4])
            g = beams_fb[round(b, 3), round(h, 3)]
            g[0] += 1
            g[1] += length
            g[2] += b*h*length
    fb_m1 = sum(g[1] for g in beams_fb.values())
    for (b, h), (n, m1, m3) in sorted(beams_fb.items(), key=lambda item: -item[1][1]):
        add('foundation beam', 'funderingsbalk', '{:.0f}x{:.0f} mm (b x h)'.format(b*1000, h*1000), n, m1, 0.0, m3)
    piles = defaultdict(lambda: [0, 0.0])
    for r in pile_rows:
        f = r.split(',')
        d, length = float(f[4]), float(f[7])
        g = piles[round(d, 3), round(length, 2)]
        g[0] += 1
        g[1] += length
    pile_m1 = sum(g[1] for g in piles.values())
    for (d, length), (n, m1) in sorted(piles.items()):
        add('pile', 'D{:.0f}'.format(d*1000), 'length {:.2f} m'.format(length), n, m1, 0.0, math.pi*d*d/4.0*m1)
    walls = defaultdict(lambda: [0, 0.0, 0.0])
    for r in wall_rows:
        f = r.split(',')
        kind, length, t, height = f[1], float(f[4]), float(f[5]), float(f[6])
        g = walls[kind, round(t, 3), f[11] if len(f) > 11 else '']
        g[0] += 1
        g[1] += length*height
        g[2] += length*height*t
    party_m2 = sum(g[1] for (kind, t, mat), g in walls.items() if kind == 'party')
    core_m2 = sum(g[1] for (kind, t, mat), g in walls.items() if kind == 'core')
    for (kind, t, mat), (n, m2, m3) in sorted(walls.items()):
        add({'party': 'party wall', 'cantilever': 'cantilever wall'}.get(kind, 'core wall'), mat, 't {:.0f} mm'.format(t*1000), n, 0.0, m2, m3)
    sections = {}
    for r in check_rows:
        f = r.split(',')
        sections[f[0]] = f[2]
    columns = defaultdict(lambda: [0, 0.0])
    for r in column_rows:
        f = r.split(',')
        height, side = float(f[3]), float(f[4])
        g = columns[round(side, 3)]
        g[0] += 1
        g[1] += height
    col_m1 = sum(g[1] for g in columns.values())
    for side, (n, m1) in sorted(columns.items()):
        add('column', 'column', 'side {:.0f} mm'.format(side*1000), n, m1, 0.0, 0.0)
    beams = defaultdict(lambda: [0, 0.0, 0.0])
    strips = defaultdict(lambda: [0, 0.0, 0.0])
    for name, r in zip(beam_names, beam_rows):
        f = r.split(',')
        span, width, depth = float(f[5]), float(f[6]), float(f[7])
        label = sections.get(name, '{:.0f}x{:.0f}'.format(width*1000, depth*1000))
        role = 'lintel' if name.endswith('lintel') else ('gallery cantilever' if name.endswith('gallery') else ('facade edge member' if '_edge' in name else ('frame beam' if '_frame' in name else ('overhang cantilever beam' if '_overhang' in name else ('overhang hung beam' if '_hung' in name else 'beam')))))
        if 'slab strip' in label:
            g = strips[role, label]
        else:
            g = beams[role, label]
        g[0] += 1
        g[1] += span
        g[2] += width*depth*span
    beam_m1 = sum(g[1] for g in beams.values())
    for (role, label), (n, m1, m3) in sorted(beams.items(), key=lambda item: -item[1][1]):
        add('beam', role, label, n, m1, 0.0, m3)
    for (role, label), (n, m1, m3) in sorted(strips.items()):
        add('edge strip (inside slab; not a beam)', role, label, n, m1, 0.0, m3)
    units = {}
    for r in apartment_rows:
        f = r.split(',')
        uid = f[0].split('.')[0]
        storey = int(f[1])
        if uid not in units:
            units[uid] = [f[2], 0.0, storey]
        units[uid][1] += float(f[3])
    internal = defaultdict(lambda: [0, 0.0, 0.0])
    for uid, (name, area, storey) in units.items():
        p = levels[storey]
        clear = max(0.0, p[5]-p[4]-slab)
        length = internal_wall_length(name, area)
        g = internal[name]
        g[0] += 1
        g[1] += length
        g[2] += length*clear
    internal_m2 = sum(g[2] for g in internal.values())
    for name, (n, m1, m2) in sorted(internal.items()):
        add('internal wall (estimate)', name, '{:.1f} m per unit; {:.0f} bedroom(s)'.format(m1/n, bedrooms_of(name, 0.0)), n, m1, m2, 0.0)
    totals = dict(foundation_beams_m1=fb_m1, piles_m1=pile_m1, party_walls_m2=party_m2, core_walls_m2=core_m2,
                  internal_walls_m2=internal_m2, columns_m1=col_m1, beams_m1=beam_m1)
    lines = ['QUANTITY TAKE-OFF: funderingsbalken {:.1f} m1 | piles {:.1f} m1 | party walls {:.1f} m2 | core walls {:.1f} m2 | internal walls (estimate) {:.1f} m2 | columns {:.1f} m1 | beams {:.1f} m1.'.format(
                 fb_m1, pile_m1, party_m2, core_m2, internal_m2, col_m1, beam_m1),
             '  Walls in m2 = length x storey height (one face, gross, openings not deducted); columns m1 = sum of storey heights; beams m1 = spans, m3 = gross width x depth x span (frame beams, lintels, downstand edge members, gallery cantilevers; concealed slab edge strips listed separately and not counted). Internal walls are NOT modelled: per apartment {:.1f} m + {:.1f} m per bedroom + {:.2f} m per m2 of unit (bathroom/entry/storage walls + bedroom walls), times the clear storey height (storey height - slab) — an assumption to replace with plan data. Takeoff_Data: {}.'.format(
                 INTERNAL_WALL_BASE, INTERNAL_WALL_PER_BEDROOM, INTERNAL_WALL_PER_M2, TAKEOFF_HEADER)]
    return dict(totals=totals, rows=rows, lines=lines)


# 7e. Debug output

def debug_round(value):
    if isinstance(value, float):
        return round(value, 4)
    if isinstance(value, dict):
        return dict((str(k), debug_round(v)) for k, v in value.items())
    if isinstance(value, (list, tuple)):
        return [debug_round(v) for v in value]
    return value


def governing_rows(rows, u_fields=(23, 24, 25, 26, 29)):
    best = {}
    for row in rows:
        f = row.split(',')
        family = f[0].split('_')[0]+('_'+f[0].split('_')[-1] if f[0].startswith('B_') and not f[0].split('_')[-1].isdigit() else '')
        values = []
        for i in u_fields:
            try:
                values.append(float(f[i]))
            except (ValueError, IndexError):
                pass
        u = max(values or [0.0])
        if family not in best or u > best[family][1]:
            best[family] = (f[0], u, f[2])
    return dict((k, dict(member=v[0], U=v[1], section=v[2])) for k, v in best.items())


def debug_lines(info):
    lines = ['DEBUG (copy everything from here to the end and send it): one JSON object per line.']
    for key in ('INPUT', 'BALANCE', 'GOVERNING', 'STABILITY', 'FOUNDATION', 'PROGRAMME', 'TAKEOFF'):
        if info.get(key) is not None:
            lines.append('DBG|{}|{}'.format(key, json.dumps(debug_round(info[key]), sort_keys=True, separators=(',', ':'))))
    return lines


# 8. Material reporting

def material_report_lines(construction, frame, material_status):
    common = [
        'Material_Quantities: category,material,volume_m3,mass_kg. No row for foundations when not estimated. Concrete beam volume is the downstand below the slab.',
        'Construction changes frame members only. Supply appropriate Floor_Load/Roof_Load for the actual floor build-up; these loads are not reduced automatically.',
        'Grid_Status concerns geometry and listed layout checks; Material_Check_Status concerns only the stated member model. Neither certifies the whole structure.',
        'Maximum checked member utilization {:.4f}; status {}.'.format(frame['max_utilization'],material_status),
        'Beam loads: slab load distributed per bay. Bays with long/short >= {:.1f} span one way onto the beams along the long sides (q x short/2 each); other bays two-way (trapezoid q x short/2 x (1-(short/long)^2/3) on long sides, triangle q x short/3 on short sides, moment-equivalent UDLs). Slab strips outside the outer grid lines load the parallel edge beam.'.format(ONE_WAY_ASPECT),
        'Member_Check_Data: member_id,material,section,width_m,depth_m,wall_m,A_m2,Iy_m4,Iz_m4,EI_effective_kNm2,N_service_kN,NEd_kN,NbRd_kN,Ncr_kN,relative_slenderness,buckling_factor,qSLS_kNm,MEd_kNm,MRd_kNm,VEd_kN,VRd_kN,deflection_mm,limit_mm,U_N,U_M,U_V,U_deflection,As_req_mm2,As_prov_mm2,U_As,span_depth,span_depth_limit,lambda_lim,MEd_column_kNm,MRd_column_kNm,status',
        'Blank Member_Check_Data fields are not applicable to that material or member type.'
    ]
    if construction == 'Concrete':
        return common+[
            'Concrete members: C30/37, B500, cover to bar centre {:.1f} mm; ULS {:.2f} x total service load including self-weight (EN 1990 6.10b with 70 % permanent assumed).'.format(RC_COVER_TO_BAR_CENTRE_MM,ULS_GRAVITY),
            'Concrete beams: simply supported rectangle width x total depth (T-flange ignored); bending with x/d <= 0.45, web crushing VRd,max (theta 45 deg; links not designed), As <= 4%, EC2 7.4.2 span/depth (K=1, flanged factor {:.1f}, 7/L for L > 7 m). Width = max(0.25 m, depth/2).'.format(RC_FLANGED_FACTOR),
            'Concrete columns: {:.0%} longitudinal steel assumed; minimum eccentricity max(h/30, 20 mm, l0/400); l0 = storey height (braced); nominal-curvature second order when lambda > lambda_lim (creep 2.0); trilinear N-M interaction through the balance point.'.format(RC_COLUMN_STEEL_RATIO),
            'Not checked for concrete: link design, crack width, continuity and hogging, punching, torsion, fire, durability, joints and sway.',
            'Concrete reference: first-generation EN 1992-1-1 expressions (6.1, 5.8.3.1, 5.8.8, 7.4.2); Dutch National Annex not verified.'
        ]
    p = material_properties(construction)
    lines = common+[
        'Assumed grade: {} | density {:.1f} kg/m3 | unit weight {:.5f} kN/m3.'.format(p['grade'],p['rho'],p['weight']),
        'Internal concrete sizing assumptions do not influence '+construction+' members.',
        'Selected generated trial sections; no claim of supplier/catalogue availability. Select the lowest-area section passing member checks AND checked aperture clearance where possible; unresolved conflicts remain flagged.',
        'Gravity design scenario: {:.2f} x TOTAL service load including frame self-weight (EN 1990 6.10b: 1.2 G + 1.5 Q with 70 % permanent assumed).'.format(ULS_GRAVITY),
        'Beams: simply supported. Column reactions retain the once-only tributary-area model. This is not an analysis of a floor plate or beam-column frame.',
        'Beam self-weight uses the full material section below the floor zone; hollow steel interiors carry no weight.',
        'Beam deflection includes bending and shear; internal L/300 limit; continuous lateral restraint assumed.',
        'Columns: concentric axial resistance with flexural buckling about both axes, effective length = storey height (braced pin-ended assumption).',
        'Columns and beams are pinned and braced: the core walls (frame mode with a programme) or a separately designed bracing system take the wind; beam restraint is assumed.',
        'Not checked: column joint moments/eccentricity and N-M interaction, sway, connection forces/detailing, support bearing/crippling, torsion, vibration, fire or floor design.',
        'Material assumptions and first-generation Eurocode expressions are listed in the script; current Dutch National Annex/compliance not verified.'
    ]
    if construction == 'Steel':
        lines += [
            'Steel geometry: hollow sharp-corner fabricated box sections; hollow-ring Brep volume is checked against calculated section area x length.',
            'Steel: E=210000 MPa; fy=235 MPa (t<=16 mm) or 225 MPa (16<t<=40 mm); plates 4-40 mm; boxes up to 1500 mm deep (columns 1000 mm); gammaM0=gammaM1=1.0; buckling curve c (welded box, EN 1993-1-1 Table 6.2), alpha=0.49.',
            'All plates satisfy c/t <=42 epsilon under uniform compression. Elastic bending and web shear calculated; high shear reduces moment resistance.',
            'Steel reference: https://steelconstruction.info/topics/design/member-design/'
        ]
    else:
        lines += [
            'Glulam GL24h: fm,k=fc,0,k=24 MPa; fv,k=3.5 MPa; Emean=11500 MPa; E05=9600 MPa; Gmean=650 MPa.',
            'Timber scenario: protected service class 2; kmod=0.60; gammaM=1.25; kdef=0.80; beta_c=0.10; no size benefit; shear crack factor=0.67.',
            'Effective bending/shear stiffness includes creep on the entire service load. Joint slip, wet/exposed timber and notched/drilled sections are not covered.',
            'Timber reference: https://www.swedishwood.com/siteassets/5-publikationer/pdfer/sw-design-of-timber-structures-vol2-2022.pdf'
        ]
    return lines


# 9. Grasshopper calculation

def run():
    global TARGET_CLEAR_HEIGHT, FACADE_LOAD, CORE_COUNT, PROGRAMME_CORRIDOR_WIDTH, PROGRAMME_COMMON_RATIO, PROGRAMME_CORE_AREA, MAX_UNIT_WIDTH, APARTMENT_GEN, TYPE_MIX, APT_MANUAL_COUNT, APARTMENT_DEPTH
    MATERIAL_CATALOG_CACHE.clear()
    MATERIAL_BEAM_CACHE.clear()
    MATERIAL_CLEAR_CACHE.clear()
    source = inp('Breps')
    if source is None:
        raise ValueError('Connect Breps with LIST access.')
    if isinstance(source,(rg.Brep,rg.Extrusion)):
        source = [source]
    source = [item for item in list(source) if item is not None]
    if not source:
        raise ValueError('Breps list is empty.')
    wire_count = brep_wire_count()
    if wire_count is not None and wire_count > len(source):
        raise ValueError('{} items arrive on the Breps wire but only {} are geometry: {} are empty (null). The component feeding Breps produced nothing for them (for example a failed Extrude/Boolean/Move); check that component, a Panel on the wire shows the empty items as <null>.'.format(wire_count, len(source), wire_count-len(source)))
    clearance = 0.0
    floor,roof = float(FLOOR_LOAD),float(ROOF_LOAD)
    construction = construction_name(inp('Construction'))
    slab = float(SLAB_THICKNESS)
    if slab <= 0:
        raise ValueError('SLAB_THICKNESS at the top of the script must be > 0 m.')
    ratio = CONCRETE_BEAM_RATIO
    stress = CONCRETE_COLUMN_STRESS_MPA*1000.0
    if min(clearance,floor,roof) < 0:
        raise ValueError('Clearance and loads cannot be negative.')
    pile_diameter = number('Pile_Diameter',0,False)
    pile_diameter = pile_diameter/1000.0 if pile_diameter > 5 else pile_diameter
    pile_capacity = number('Pile_Capacity',0,False)
    if pile_diameter < 0 or pile_capacity < 0:
        raise ValueError('Pile_Diameter and Pile_Capacity cannot be negative.')
    pile_tension = number('Pile_Tension_Capacity',0,False)
    if pile_tension < 0:
        raise ValueError('Pile_Tension_Capacity cannot be negative.')
    PILE_TENSION['value'] = pile_tension or None
    pile_cfg = dict(diameter=pile_diameter or None,capacity=pile_capacity or None,tension=pile_tension or None,
                    peil=number('Peil_NAP',PEIL_NAP_DEFAULT,False),tip=number('Pile_Tip_NAP',PILE_TIP_NAP_DEFAULT,False))
    doc = Rhino.RhinoDoc.ActiveDoc
    if doc is None or str(doc.ModelUnitSystem).split('.')[-1].rstrip('_') in ('None','Unset','CustomUnits'):
        raise ValueError('Set Rhino document units.')
    scale = Rhino.RhinoMath.UnitScale(doc.ModelUnitSystem,Rhino.UnitSystem.Meters)
    if not isfinite(scale) or scale <= 0:
        raise ValueError('Invalid unit conversion.')
    tol = max(doc.ModelAbsoluteTolerance*scale,1e-6)
    angle = infer_grid_angle(source,scale,tol)
    local = rg.Transform.Rotation(-math.radians(angle),rg.Vector3d.ZAxis,rg.Point3d.Origin)
    world = rg.Transform.Rotation(math.radians(angle),rg.Vector3d.ZAxis,rg.Point3d.Origin)
    to_m = rg.Transform.Scale(rg.Point3d.Origin,scale)
    levels = []
    for i,item in enumerate(source):
        if isinstance(item,rg.Extrusion):
            item = item.ToBrep()
        if not isinstance(item,rg.Brep) or not item.IsValid or not item.IsSolid:
            raise ValueError('Storey {} is not a valid closed Brep.'.format(i))
        b = item.DuplicateBrep()
        if not b.Transform(to_m) or not b.Transform(local):
            raise ValueError('Storey transform failed.')
        bb = b.GetBoundingBox(True)
        dx,dy,dz = bb.Max.X-bb.Min.X,bb.Max.Y-bb.Min.Y,bb.Max.Z-bb.Min.Z
        vp = rg.VolumeMassProperties.Compute(b)
        if vp is None:
            raise ValueError('Cannot measure storey volume.')
        volume = abs(vp.Volume)
        vp.Dispose()
        if min(dx,dy,dz) <= tol or abs(volume-dx*dy*dz) > max(dx*dy*dz*1e-6,2*tol*(dx*dy+dx*dz+dy*dz)):
            raise ValueError('Storey {} is not a rectangular box aligned with the detected orientation; independently rotated/irregular storeys are unsupported.'.format(i))
        levels.append((bb.Min.X,bb.Max.X,bb.Min.Y,bb.Max.Y,bb.Min.Z,bb.Max.Z))
    brep_count = len(levels)
    levels = normalize_levels(merge_storey_boxes(levels,tol),tol)
    records = []
    index = FaceIndex(records)
    walls_on = bool(inp('Load_Bearing_Walls') or False)
    TARGET_CLEAR_HEIGHT = float(CLEAR_HEIGHT)
    if TARGET_CLEAR_HEIGHT <= 0:
        raise ValueError('CLEAR_HEIGHT at the top of the script must be > 0 m.')
    FACADE_LOAD = number('Facade_Load',FACADE_LOAD_DEFAULT,False)
    if FACADE_LOAD < 0:
        raise ValueError('Facade_Load cannot be negative.')
    core_input = inp('Core_Count')
    CORE_COUNT = None
    if core_input is not None and str(core_input).strip() not in ('', '0'):
        CORE_COUNT = int(round(float(core_input)))
        if CORE_COUNT < 1:
            raise ValueError('Core_Count must be 1 or more (leave empty or 0 for automatic: one core per {:.0f} m2 of footprint).'.format(PROGRAMME_CORE_SERVED_AREA))
    PROGRAMME_CORRIDOR_WIDTH = float(CORRIDOR_WIDTH)
    if PROGRAMME_CORRIDOR_WIDTH <= 0:
        raise ValueError('CORRIDOR_WIDTH at the top of the script must be > 0 m.')
    PROGRAMME_CORE_AREA = float(CORE_AREA)
    if PROGRAMME_CORE_AREA <= 0:
        raise ValueError('CORE_AREA at the top of the script must be > 0 m2.')
    PROGRAMME_COMMON_RATIO = float(COMMON_RATIO)
    if PROGRAMME_COMMON_RATIO > 1.0:
        PROGRAMME_COMMON_RATIO /= 100.0
    if not 0.0 <= PROGRAMME_COMMON_RATIO < 1.0:
        raise ValueError('COMMON_RATIO at the top of the script must be between 0 and 1 (e.g. 0.05).')
    MAX_UNIT_WIDTH = float(MAX_UNIT_WIDTH_SETTING) if MAX_UNIT_WIDTH_SETTING else None
    if MAX_UNIT_WIDTH is not None and MAX_UNIT_WIDTH < PROGRAMME_MIN_FRONTAGE:
        raise ValueError('MAX_UNIT_WIDTH_SETTING {:.2f} m at the top of the script is below the minimum unit frontage {:.1f} m (None = no limit).'.format(MAX_UNIT_WIDTH, PROGRAMME_MIN_FRONTAGE))
    APARTMENT_DEPTH = optional_number('Apartment_Depth', None)
    if APARTMENT_DEPTH is not None and APARTMENT_DEPTH <= 0:
        APARTMENT_DEPTH = None
    if APARTMENT_DEPTH is not None and APARTMENT_DEPTH < PROGRAMME_MIN_FRONTAGE:
        raise ValueError('Apartment_Depth {:.2f} m is below {:.1f} m (leave empty or 0 for automatic: half the floor width minus the corridor, at most {:.0f} m).'.format(APARTMENT_DEPTH, PROGRAMME_MIN_FRONTAGE, PROGRAMME_DAYLIGHT_DEPTH))
    type_count = len(as_list(inp('AreaPerTypeOfApartment')))
    APARTMENT_GEN = int(round(optional_number('Apartment_GEN', 0)))
    TYPE_MIX, APT_MANUAL_COUNT = None, None
    if APARTMENT_GEN not in (0, 1, 2):
        raise ValueError('Apartment_GEN must be 0 (maximum lettable area), 1 (Type_mix shares) or 2 (Apt_Manual_Count numbers).')
    if APARTMENT_GEN == 1:
        TYPE_MIX = [float(v) for v in as_list(inp('Type_mix')) if str(v).strip() != '']
        if len(TYPE_MIX) != type_count:
            raise ValueError('Apartment_GEN = 1 needs Type_mix (Float, List access) with one share per apartment type: got {} values for {} types (fractions like 0.3 or percentages like 30; they are normalised to their sum).'.format(len(TYPE_MIX), type_count))
        if any(v < 0 or not isfinite(v) for v in TYPE_MIX) or sum(TYPE_MIX) <= 0:
            raise ValueError('Type_mix values must be >= 0 with a positive sum.')
    elif APARTMENT_GEN == 2:
        raw = [float(v) for v in as_list(inp('Apt_Manual_Count')) if str(v).strip() != '']
        if len(raw) != type_count:
            raise ValueError('Apartment_GEN = 2 needs Apt_Manual_Count (Integer, List access) with one number per apartment type: got {} values for {} types.'.format(len(raw), type_count))
        if any(v < 0 or not isfinite(v) or abs(v-round(v)) > 1e-9 for v in raw) or sum(raw) <= 0:
            raise ValueError('Apt_Manual_Count values must be whole numbers >= 0 with at least one apartment.')
        APT_MANUAL_COUNT = [int(round(v)) for v in raw]
    if TARGET_CLEAR_HEIGHT >= max(p[5]-p[4] for p in levels):
        raise ValueError('Clear_Height {:.2f} m must be below the storey height.'.format(TARGET_CLEAR_HEIGHT))
    wind_area = int(round(number('Wind_Area',2)))
    if wind_area not in WIND_AREA_VB0:
        raise ValueError('Wind_Area must be 1, 2 or 3 (NEN-EN 1991-1-4 NB wind areas).')
    wind = (wind_area,wind_terrain(inp('Wind_Terrain')))
    wall_t0 = WALL_START[construction][0] if walls_on else 0.0
    end_t0 = wall_t0
    max_bay = slab_span_limit(construction,slab,max(floor,roof))[0] if walls_on else None
    wall_system = None
    if walls_on:
        early_layout,wall_system,(wall_t0,end_t0) = iterate_wall_design(levels,construction,floor,roof,slab,tol,
                                                                        lambda party,limit,end: programme_report(levels,[],party,limit,end)[4],wind,pile_cfg)
    early_status,early_lines,early_issues,early_data,early_layout = programme_report(levels,[],wall_t0,max_bay,end_t0)
    wall_note = []
    if wall_system is not None:
        selected = wall_system['selected']
        groups,unmatched = facade_groups(levels,index.records,tol)
        selected['facade_hits'] = facade_frame_hits(selected['frame'],selected['supports'],groups,clearance,tol)
        selected['conflicts'] = frame_clash_count(selected['frame'],index,clearance,tol)
        selected['unmatched_facades'] = unmatched
        selected['intrusions'] = column_intrusions(selected['frame']['columns'],early_layout['apartments'])
        attempts = ['CROSS_WALL_SYSTEM: grid = stacked bay lines from the apartment layout plus frame lines; aperture/facade checks applied to frame members and walls.']
    else:
        if has_overhang(levels,tol):
            raise ValueError('Overhanging storeys (a storey larger than the storey below) are designed in wall mode only: set Load_Bearing_Walls = True and connect a valid AreaPerTypeOfApartment.')
        if walls_on:
            wall_note = ['LOAD-BEARING WALLS NOT MODELLED: Load_Bearing_Walls needs a valid programme (AreaPerTypeOfApartment); frame-only design used.']
        selected, attempts = choose_layout(levels,index,floor,roof,ratio,slab,stress,clearance,tol,construction,early_layout['apartments'])
    xs,ys,fx,fy = (selected[key] for key in ('xs','ys','fx','fy'))
    models,frame = selected['models'],selected['frame']
    iteration,reserve,converged = (selected[key] for key in ('iteration','reserve','converged'))
    removed_lines = selected['removed_lines']
    offset = selected['offset']
    issues = list(frame['notes'])+(wall_system.get('overhang_issues',[]) if wall_system is not None else [])
    if construction != 'Concrete' and offset > tol:
        edge_status,edge_data,edge_results = 'NOT_CHECKED_FLOOR_SYSTEM',[],[]
        edge_report = ['EDGE CHECK: floor edge cantilever {:.3f} m; Construction selects the frame only. Floor build-up and restraint are not specified, so the RC strip calculation is not applied to this material option.'.format(offset)]
        issues.append('FLOOR EDGE SUPPORT UNVERIFIED for inboard Steel/Timber frame. See edge report.')
    else:
        edge_status,edge_data,edge_report,edge_results = check_edges(levels,selected,slab,floor,roof,tol)
    if edge_status == 'FAIL':
        issues.append('EDGE CHECK FAILED. See Edge_Check_Data and detailed edge report; geometry retained for diagnosis.')
    elif edge_status == 'PASS_ASSUMED_MODEL':
        issues.append('EDGE CHECK PASSED UNDER STATED ASSUMPTIONS. Full restraint/detailing, corner effects and NL compliance remain unverified.')
    for label, coords, fixed in ((('X', xs, fx), ('Y', ys, fy)) if wall_system is None else ()):
        for i, (a, b) in enumerate(zip(coords, coords[1:])):
            if b-a < MIN_MOVABLE_BAY-tol:
                issues.append('MANDATORY SHORT BAY {} {}: {:.3f} m between required support-rectangle boundaries; requires a different structural layout.'.format(label,i,b-a))
    if not converged:
        issues.append('Column reservation iteration limit reached; final actual members still checked.')
    if any(not any(overlap(b,expand(p,tol+clearance)) for p in levels) for face,b in records):
        issues.append('Some aperture face boxes do not overlap any storey; check mesh placement.')
    foundation_kinds,pile_result,pile_lines,foundation_notes = None,None,[],[]
    frame_cores = None
    if wall_system is None:
        frame_cores = design_frame_cores(levels,early_layout,construction,wind,sum(frame['loads'].values()),len(models[0]['nodes']))
        pile_result = design_frame_piles(levels,xs,ys,frame['loads'],frame['sides'],floor,pile_cfg,frame_cores['foundation'] if frame_cores else None)
        foundation_items,fd,foundation_checks = pile_result['footings'],pile_result['pile_rows'],pile_result['element_rows']
        foundation_status = 'FAIL' if pile_result['status'] != 'PASS_ASSUMED_MODEL' else 'PASS_ASSUMED_MODEL'
        vf = pile_result['quantities']['beam_m3']+pile_result['quantities']['poer_m3']
        foundation_notes = list(pile_result['issues'])
        foundation_kinds = pile_result['kinds']
        pile_lines = foundation_summary_lines(pile_result,pile_cfg)
    issues.extend(foundation_notes)
    issues.extend(wall_note)
    walls = [(name,k,b) for name,k,kind,b in frame_cores['walls']] if frame_cores else []
    if frame_cores and frame_cores['stability']['status'] != 'PASS_ASSUMED_MODEL':
        issues.append('LATERAL STABILITY FAILS in frame mode (utilisation {:.2f}) even after thickening the core walls. See the LATERAL STABILITY block.'.format(frame_cores['stability']['worst']))
    if wall_system is not None:
        walls = [(name,k,b) for name,k,kind,b in wall_system['walls']]
        pile_result = wall_system['pile_result']
        foundation_items,fd,foundation_checks = wall_system['wall_footings'],wall_system['pile_rows'],wall_system['element_rows']
        foundation_kinds = wall_system['footing_kinds']
        vf = pile_result['quantities']['beam_m3']+pile_result['quantities']['poer_m3']
        foundation_status = 'FAIL' if wall_system['footing_fail'] else 'PASS_ASSUMED_MODEL'
        issues.extend(pile_result['issues'])
        if wall_system['footing_fail']:
            issues.append('PILE FOUNDATION CHECK FAILED. See the FOUNDATION block.')
    programme_status,programme_lines,programme_issues,programme_data,programme_layout = programme_report(levels,frame['columns'],wall_t0,max_bay,end_t0)
    clashes,clash_ids = [],set()
    counts = dict(Column=0,Beam=0,Foundation=0,Wall=0,GableOpening=0)
    fixed_column_clashes = 0
    wall_kind = wall_system['wall_kind'] if wall_system is not None else {}
    for group,members in (('Column',frame['columns']),('Beam',frame['beams']),('Foundation',foundation_items),('Wall',walls)):
        for name,k,b in members:
            hits = index.hits(expand(b,clearance+tol))
            kind = 'GableOpening' if group == 'Wall' and wall_kind.get(name) == 'gable' else group
            if hits:
                counts[kind] += 1
                if kind == 'Column':
                    _, storey_id, ni, nj = name.split('_')
                    if int(ni) in fx and int(nj) in fy:
                        fixed_column_clashes += 1
                clash_ids.add(name)
                clashes.append('{},{},{},{},{}'.format(kind,name,k,len(hits),min(f for f,b in hits)))
    if counts['Column']+counts['Beam']+counts['Foundation']+counts['Wall']:
        issues.append('APERTURE BOX CONFLICTS remain. See Clash_Data/Clash_Geometry. Fixed boundaries and beam heads may require another layout.'
                      if wall_system is None else
                      'APERTURE BOX CONFLICTS: {} wall ends / {} beams meet windows. A cross wall or slab edge member cannot move freely: shift the window, or the bay line in the design. See Clash_Data (member_type Wall/Beam).'.format(counts['Wall'],counts['Beam']))
    if wall_system is not None and wall_system['stability']['status'] != 'PASS_ASSUMED_MODEL':
        issues.append('LATERAL STABILITY FAILS (utilisation {:.2f}) even after thickening the stability walls. See the LATERAL STABILITY block for the failing direction and options.'.format(wall_system['stability']['worst']))
    if wall_system is not None and wall_system['min_clear'] < TARGET_CLEAR_HEIGHT-1e-9:
        issues.append('FRAME BEAM HEADROOM: {:.2f} m clear under the deepest frame beam inside an apartment, below the {:.2f} m target: storey height {:.2f} m needed, or narrower units (see HEADROOM OPTIONS).'.format(wall_system['min_clear'],TARGET_CLEAR_HEIGHT,wall_system['storey_needed']))
    if counts['GableOpening']:
        issues.append('GABLE OPENINGS: {} gable-wall storeys contain windows. Openings with lintels are needed in those load-bearing walls (not designed); listed in Clash_Data as GableOpening.'.format(counts['GableOpening']))
    facade_rows = ['{},{},{},{}'.format(name,int(name.split('_')[1]),len(hits),min(hits)) for name,hits in sorted(selected['facade_hits'].items())]
    facade_status = 'NOT_CHECKED_NO_APERTURES'
    clash_ids.update(selected['facade_hits'])
    if facade_rows:
        issues.append('FACADE ALIGNMENT CONFLICTS: {} perimeter members/beam ends overlap windows in elevation. Inward depth does not resolve these conflicts. No clear layout found within the candidate search and support constraints.'.format(len(facade_rows)))
    if wall_system is None and construction != 'Concrete' and (counts['Beam'] or any(n.startswith('B_') for n in selected['facade_hits'])):
        issues.append('BEAM CLEARANCE UNRESOLVED: no fully clear layout was found among the tested sections and '+'{:.1f}-{:.1f}'.format(min(SEED_TARGETS[construction]),SPAN_LIMITS[construction])+' m seed-bay assumptions. Wider/shallow sections and shorter seed bays were considered. Resistance-adequate conflicting members remain visible; no undersized section is substituted.')
    if selected['unmatched_facades']:
        issues.append('FACADE CHECK INCOMPLETE: {} vertical aperture faces could not be associated with an aligned facade; physical checks still include them.'.format(len(selected['unmatched_facades'])))
    clash_geometry = []
    def geometry(members):
        out = []
        for name,k,b in members:
            if construction == 'Steel' and name in frame.get('profiles',{}):
                try:
                    shape = steel_member_brep(b,frame['profiles'][name],tol)
                except Exception as error:
                    raise ValueError('{}: {}'.format(name,error))
            elif name.startswith('P_') and foundation_kinds:
                circle = rg.Circle(rg.Plane(rg.Point3d((b[0]+b[1])/2.0,(b[2]+b[3])/2.0,b[4]),rg.Vector3d.ZAxis),(b[1]-b[0])/2.0)
                shape = rg.Cylinder(circle,b[5]-b[4]).ToBrep(True,True)
            else:
                shape = rg.Box(rg.Plane.WorldXY,rg.Interval(b[0],b[1]),rg.Interval(b[2],b[3]),rg.Interval(b[4],b[5])).ToBrep()
            if shape is None or not shape.Transform(world) or not shape.Transform(rg.Transform.Scale(rg.Point3d.Origin,1/scale)):
                raise ValueError('Member geometry generation failed.')
            out.append(shape)
            if name in clash_ids:
                clash_geometry.append(shape)
        return out
    columns,beams,foundations = geometry(frame['columns']),geometry(frame['beams']),geometry(foundation_items)
    foundation_count = len(foundations)
    if foundation_kinds:
        foundations = to_tree(branch_items(foundations,foundation_kinds,FOUNDATION_KINDS))
    apartment_boxes,support_boxes = geometry(programme_layout['apartments']),geometry(programme_layout['supports'])
    wall_breps = geometry(walls)
    wall_kinds = [kind for name,k,kind,b in wall_system['walls']] if wall_system is not None else ([kind for name,k,kind,b in frame_cores['walls']] if frame_cores else [])
    wall_tree = to_tree(branch_items(wall_breps,wall_kinds,WALL_KINDS))
    wall_rows_all = wall_system['wall_rows'] if wall_system is not None else (frame_cores['rows'] if frame_cores else [])
    wall_row_kinds = [row.split(',')[1] for row in wall_rows_all]
    wall_data_tree = to_tree(branch_items(wall_rows_all,wall_row_kinds,WALL_KINDS))
    type_order,kinds = programme_layout['type_order'],programme_layout['apartment_types']
    apartment_tree = to_tree(branch_items(apartment_boxes,kinds,type_order))
    apartments_n = to_tree([[int(programme_layout.get('mix',{}).get(n,0))] for n in type_order])
    apartment_data_tree = to_tree(branch_items(programme_layout['apartment_rows'],kinds,type_order))
    support_tree = to_tree(branch_items(support_boxes,programme_layout['support_kinds'],SUPPORT_KINDS))
    support_data_tree = to_tree(branch_items(programme_layout['support_rows'],programme_layout['support_kinds'],SUPPORT_KINDS))
    intrusions = column_intrusions(frame['columns'],programme_layout['apartments'])
    branch_lines = ['Apartment_Boxes / Apartment_Data branches: '+', '.join('{{{}}} {}'.format(i,n) for i,n in enumerate(type_order)) if type_order else 'Apartment_Boxes: empty (no programme).',
                    'Support_Boxes / Support_Data branches: '+', '.join('{{{}}} {}'.format(i,n) for i,n in enumerate(SUPPORT_KINDS))+' | Apartments_N: one integer per branch, same branch order as Apartment_Boxes (number of apartments of that type). Clear_Height {:.2f} m (script setting; was {:.2f} m default); Facade_Load {:.2f} kN/m2 of elevation (input, default {:.2f}); Core_Count {} (input; empty = automatic, one core per {:.0f} m2 of footprint); Corridor_Width {:.2f} m (script setting; default {:.2f}); Max_Unit_Width {} (script setting; None = no limit); Common_Ratio {:.1%} (script setting; default {:.0%}); Core_Area {:.1f} m2 (script setting; default {:.0f}); Apartment_GEN {} (0 = max lettable area, 1 = Type_mix shares, 2 = Apt_Manual_Count numbers; empty = 0); Apartment_Depth {} (input; empty = automatic).'.format(TARGET_CLEAR_HEIGHT,TARGET_CLEAR_HEIGHT_DEFAULT,FACADE_LOAD,FACADE_LOAD_DEFAULT,CORE_COUNT or 'automatic',PROGRAMME_CORE_SERVED_AREA,
                        PROGRAMME_CORRIDOR_WIDTH,CORRIDOR_WIDTH_DEFAULT,'{:.2f} m'.format(MAX_UNIT_WIDTH) if MAX_UNIT_WIDTH else 'none',PROGRAMME_COMMON_RATIO,COMMON_RATIO_DEFAULT,PROGRAMME_CORE_AREA,CORE_AREA_DEFAULT,APARTMENT_GEN,'{:.2f} m'.format(APARTMENT_DEPTH) if APARTMENT_DEPTH else 'automatic'),
                    'Grid alignment: {} | columns inside apartment boxes: {} column-storeys. Aligned candidates follow party walls along the long axis (span <= short limit {:.1f} m; fewest lines through apartments, then fewest lines, then most party walls hit) and compete under the normal ranking: clashes, notes, convergence, columns inside apartments, column count.'.format(
                        'APARTMENT_ALIGNED selected' if selected['mode'].endswith('APARTMENT_ALIGNED') else 'regular grid selected', intrusions, SHORT_SPAN_LIMITS[construction])]
    status = 'REVIEW_REQUIRED' if issues else 'CLEAR_CONSERVATIVE_CHECK'
    sx,sy = [b-a for a,b in zip(xs,xs[1:])],[b-a for a,b in zip(ys,ys[1:])]
    material_status = 'PASS_ASSUMED_MEMBER_MODEL'
    material_checks = frame.get('checks',[])
    mass_density = DENSITY*1000/9.81 if construction=='Concrete' else material_properties(construction)['rho']
    quantities = ['Columns,{},{:.6f},{:.3f}'.format(construction,frame['vc'],frame['vc']*mass_density),
                  'Beams,{},{:.6f},{:.3f}'.format(construction,frame['vb'],frame['vb']*mass_density)]
    if frame.get('vb_steel', 0.0) > 1e-9:
        quantities.append('Beams,Steel (overhang inserts),{:.6f},{:.3f}'.format(frame['vb_steel'],frame['vb_steel']*material_properties('Steel')['rho']))
    if True:
        pq = pile_result['quantities']
        concrete_mass = DENSITY*1000/9.81
        quantities += ['Piles,Concrete {} ({} piles; {:.1f} m1),{:.6f},{:.3f}'.format(pile_result['label'],pq['piles'],pq['pile_m1'],pq['pile_m3'],pq['pile_m3']*concrete_mass),
                       'Foundation beams,Concrete ({:.1f} m1),{:.6f},{:.3f}'.format(pq['beam_m1'],pq['beam_m3'],pq['beam_m3']*concrete_mass),
                       'Pile caps,Concrete,{:.6f},{:.3f}'.format(pq['poer_m3'],pq['poer_m3']*concrete_mass),
                       'Ground floor,{} ({:.1f} m2; gross volume),{:.6f},{:.3f}'.format(pile_result['gf_label'],pq['gf_m2'],pq['gf_m3'],pq['gf_m2']*GF_TYPES[[g[0] for g in GF_TYPES].index(pile_result['gf_t'])][2]*1000/9.81 if pile_result['gf_t'] in [g[0] for g in GF_TYPES] else 0.0),
                       'Foundation reinforcement,B500 (indicative kg/m3),0.000000,{:.3f}'.format(pq['rebar'])]
    if wall_system is not None:
        quantities.append('Walls,{},{:.6f},{:.3f}'.format(wall_system['material'],wall_system['wall_volume'],wall_system['wall_mass']))
    elif frame_cores:
        quantities.append('Core walls,{},{:.6f},{:.3f}'.format(frame_cores['material'],frame_cores['volume'],frame_cores['mass']))
    material_report = material_report_lines(construction,frame,material_status)
    report = [
        construction.upper()+' FRAME CONCEPT ESTIMATE - MATERIALS v'+SCRIPT_VERSION+' - NOT STRUCTURAL DESIGN',
        'Author: Simos Maniatis',
        'Construction: '+construction+' frame; piled foundation. Floor loads/build-up are not automatically changed.',
        'Material check status: '+material_status,
        'Grid status: '+status,
        'Programme status: '+programme_status,
        'Storeys {} | base {:.3f} x {:.3f} m{}'.format(len(levels),levels[0][1]-levels[0][0],levels[0][3]-levels[0][2],' | {} Breps merged into {} storeys (boxes at the same storey heights joined into one rectangle each)'.format(brep_count,len(levels)) if brep_count != len(levels) else ''),
        'Automatically detected grid angle {:.6f} degrees (modulo 90).'.format(angle),
        'Layout mode: '+selected['mode'],
        'Facade alignment status: '+facade_status+' | projected column conflicts: {} | projected beam conflicts: {}'.format(sum(n.startswith('C_') for n in selected['facade_hits']),sum(n.startswith('B_') for n in selected['facade_hits'])),
        'Selected seed targets X/Y: {} m; final spans and members are checked after resizing.'.format(selected['targets']),
        'Apertures: NOT CHECKED — the Apertures_Mesh input was removed (v22); clash and facade checks are idle.',
        'Computed inward support-axis offset {:.3f} m | geometric search cap {:.3f} m (not a cantilever capacity).'.format(offset,MAX_EDGE_SUPPORT_OFFSET),
        'Interior supports retained; physical checks plus facade-projected checks for perimeter columns, beams and beam ends.',
        ('Concrete beams start at span/12 and columns at 8 MPa service stress; both are enlarged in 50 mm steps until the RC member checks pass.' if construction=='Concrete' else 'Beam sections selected for bending, shear and deflection; columns for braced axial buckling. Aperture clearance is checked separately.'),
        'Material span limits: Concrete {:.1f} m | Timber {:.1f} m | Steel {:.1f} m. Active {} limit {:.1f} m; seed targets {} m.'.format(SPAN_LIMITS['Concrete'],SPAN_LIMITS['Timber'],SPAN_LIMITS['Steel'],construction,SPAN_LIMITS[construction],', '.join('{:g}'.format(t) for t in SEED_TARGETS[construction])),
        ('One-way long-span rule: not applied to {} (two-way bays up to {:.1f} m allowed).'.format(construction,SPAN_LIMITS[construction])
         if SHORT_SPAN_LIMITS[construction] >= SPAN_LIMITS[construction]-1e-9 else
         'One-way long-span rule: if bays exceed {:.1f} m in one direction, all bays in the other direction stay <= {:.1f} m. Selected axis limits X/Y: {:.1f}/{:.1f} m.'.format(SHORT_SPAN_LIMITS[construction],SHORT_SPAN_LIMITS[construction],*selected['limits'])),
        'Internal bay limits: movable minimum {:.3f} m; maximum {:.3f} m. Layout assumptions, not design rules.'.format(MIN_MOVABLE_BAY,SPAN_LIMITS[construction]),
        'Alternative seed bay counts searched; redundant movable lines removed: {}.'.format(removed_lines),
        'Base bays {} x {} | X spans {:.3f}-{:.3f} m | Y spans {:.3f}-{:.3f} m'.format(len(sx),len(sy),min(sx),max(sx),min(sy),max(sy)),
        'Local X coordinates [m]: '+', '.join('{:.6f}'.format(v) for v in xs),
        'Local Y coordinates [m]: '+', '.join('{:.6f}'.format(v) for v in ys),
        'Sizing iterations {} | reservation {:.3f} m | converged {}'.format(iteration+1,reserve,converged),
        'Largest column {:.3f} m | extra aperture clearance {:.3f} m'.format(max(list(frame['sides'].values()) or [0.0]),clearance),
        'Columns {} | beams {} | foundation elements {}'.format(len(columns),len(beams),foundation_count),
        'Column aperture conflicts at intersections fixed in both axes: {}. Remaining column conflicts: {}.'.format(fixed_column_clashes,counts['Column']-fixed_column_clashes),
        'Area loads floors {:.3f}, roofs {:.3f} kN/m2.'.format(floor,roof)+(' Concrete sizing stress {:.3f} MPa.'.format(stress/1000) if construction=='Concrete' else ''),
        'Covered-floor area {:.3f} m2 | exposed-roof area {:.3f} m2'.format(frame['fa'],frame['ra']),
        'Base service reaction {:.3f} kN excluding foundations | load residual {:.9f} kN'.format(sum(frame['loads'].values()),frame['residual']),
        'MEMBER MATERIAL VOLUMES: columns {:.3f} m3 | beams {:.3f} m3{} | foundation beams + poeren {}'.format(frame['vc'],frame['vb'],
            ' (downstand below slab only; {:.3f} m3 including slab zone)'.format(frame['vb_gross']) if 'vb_gross' in frame else '',
            '{:.3f} m3'.format(vf) if foundation_items else 'not estimated'),
        ('LOAD-BEARING WALLS: {} {:.3f} m3 ({:.1f} t); foundation in the FOUNDATION block.'.format(wall_system['material'],wall_system['wall_volume'],wall_system['wall_mass']/1000.0) if wall_system is not None else ('Load-bearing walls: off; core walls only (stability): {} {:.3f} m3 ({:.1f} t).'.format(frame_cores['material'],frame_cores['volume'],frame_cores['mass']/1000.0) if frame_cores else 'Load-bearing walls: not modelled (Load_Bearing_Walls off); no programme cores, so no stability system is modelled.')),
        ('Volumes: concrete beams = downstand below the slab; steel hollow interiors excluded; frame members are gross sums. Slabs, reinforcement and connections excluded; joint overlaps and conflicting members remain included.' if construction == 'Concrete' else
         'Volumes: steel hollow interiors excluded; timber and steel beams are full sections below the floor zone. Slabs, reinforcement and connections excluded; joint overlaps and conflicting members remain included.'),
        'Frame material mass {:.3f} tonnes, excluding foundations and connections.'.format((frame['vc']+frame['vb'])*mass_density/1000),
        'Shared support-rectangle boundaries extend downwards; original slab footprints and loaded areas are preserved.',
        ('Search moves grid lines to improve column clearance; concrete beam depths follow the RC checks, not aperture clearance.' if construction=='Concrete' else 'Section search checks actual beam width/depth against aperture boxes and facade projection, trying wider/shallow sections before smaller seed bays down to 2.5 m.'),
        'Face bounding boxes are conservative, especially for irregular apertures. CLEAR means only the listed concept checks passed.',
        'Boundary-frame axes lie on envelope edges; inboard-frame axes are offset inward. '+('Lateral stability: the programme cores (see CORES / LATERAL STABILITY).' if frame_cores else 'No lateral stability system is included.'),
        'Ground slab ground-bearing; each top loaded once. Exposed terraces use Roof_Load.',
        'Foundation check status: '+foundation_status,
        'Supported: contiguous rectangular storeys, common automatically detected orientation; frame mode: nested storeys only (no overhangs); wall mode: overhangs carried as cantilevers (see OVERHANGS), no transfer structures.',
        'CSV units m,m2,kN,kN/m2; storey 0 lowest; member rows top-down.',
        'Column_Data: storey,i,j,height,side,tributary_area,base_service_load',
        'Beam_Data: storey,i1,j1,i2,j2,span,width,depth (concrete includes slab zone; steel/timber depth is below floor underside)',
        'Foundation_Data: '+PILE_DATA_HEADER+' | Foundation_Check_Data: '+FOUNDATION_ELEMENT_HEADER+' | Foundations tree {0} piles {1} funderingsbalken {2} poeren {3} ground floor.',
        'Clash_Data: member_type,member_id,storey,face_box_hit_count,first_mesh_face_index',
        'Facade_Clash_Data: member_id,storey,face_hit_count,first_mesh_face_index',
        'Programme status is reported separately (Programme_Status) and does not change Grid_Status.'
    ]+programme_lines+branch_lines+pile_lines+(frame_cores['lines'] if frame_cores else [])+material_report+attempts+frame['rows']+edge_report+issues+programme_issues
    if wall_system is not None:
        mc = wall_system['member_count']
        wall_material_report = [line for line in material_report if (('olumn' not in line) or line.startswith('Member_Check_Data')) and not line.startswith('Beam loads') and not line.startswith('Construction changes')]
        report = [
            construction.upper()+' WALL + FRAME CONCEPT ESTIMATE - MATERIALS v'+SCRIPT_VERSION+' - NOT STRUCTURAL DESIGN',
            'Author: Simos Maniatis',
            'Construction: {}; load-bearing walls {}; floors span one way between the walls; piled foundation. Floor loads/build-up are not automatically changed.'.format(construction,wall_system['material']),
            'Material check status: '+material_status,
            'Grid status: '+status,
            'Programme status: '+programme_status,
            'Foundation check status: '+foundation_status,
            'Storeys {} | base {:.3f} x {:.3f} m{}'.format(len(levels),levels[0][1]-levels[0][0],levels[0][3]-levels[0][2],' | {} Breps merged into {} storeys (boxes at the same storey heights joined into one rectangle each)'.format(brep_count,len(levels)) if brep_count != len(levels) else ''),
            'Automatically detected grid angle {:.6f} degrees (modulo 90).'.format(angle),
            'Layout mode: WALLS_AND_FRAME_LINES (Load_Bearing_Walls = True) | Grid_X/Grid_Y = support lines / facade, corridor and gallery lines.',
            'Apertures: NOT CHECKED — the Apertures_Mesh input was removed (v22); window clashes are ignored for now.',
            'Members: wall pieces {} | columns {} | frame beams {} | corridor lintels {} | facade edge members {} | gallery cantilevers {} | foundations {}'.format(len(walls),len(columns),mc.get('frame',0),mc.get('lintel',0),mc.get('edge',0),mc.get('gallery',0),foundation_count),
            'Columns inside apartment boxes: {} column-storeys (columns in the facade or corridor wall of a unit crossed by a frame line).'.format(intrusions),
            'Local X coordinates [m]: '+', '.join('{:.6f}'.format(v) for v in xs),
            'Local Y coordinates [m]: '+', '.join('{:.6f}'.format(v) for v in ys),
            'Area loads floors {:.3f}, roofs {:.3f} kN/m2 | facade {:.1f} kN/m2 of elevation.'.format(floor,roof,FACADE_LOAD),
            'Covered-floor area {:.3f} m2 | exposed-roof area {:.3f} m2'.format(frame['fa'],frame['ra']),
            'VOLUMES: walls {} {:.3f} m3 ({:.1f} t) | columns {:.3f} m3 | beams {:.3f} m3 (concrete = downstand below the slab; concealed slab strips are inside the slab){} | foundations {:.3f} m3. Slabs, reinforcement and connections excluded.'.format(
                wall_system['material'],wall_system['wall_volume'],wall_system['wall_mass']/1000.0,frame['vc'],frame['vb'],
                ' + steel overhang inserts {:.3f} m3 ({:.1f} t)'.format(frame['vb_steel'],frame['vb_steel']*material_properties('Steel')['rho']/1000.0) if frame.get('vb_steel',0.0) > 1e-9 else '',vf),
            'Supported: contiguous rectangular storeys, common automatically detected orientation; frame mode: nested storeys only (no overhangs); wall mode: overhangs carried as cantilevers (see OVERHANGS), no transfer structures.',
            'CSV units m,m2,kN,kN/m2; storey 0 lowest; member rows top-down.',
            'Column_Data: storey,i,j,height,side,tributary_area,service_load_accumulated | j = cross line index (facade 0, corridor edges 1-2, facade 3; gallery plan: facade 0, gallery edge 1).',
            'Beam_Data: storey,i1,j1,i2,j2,span,width,depth | frame beam across a band, corridor lintel j 1-2, facade edge member j 0-0 / last-last between lines i1-i2, gallery cantilever j 1-2.',
            'Foundation_Data: '+PILE_DATA_HEADER+' | Foundation_Check_Data: '+FOUNDATION_ELEMENT_HEADER+' | Foundations tree {0} piles {1} funderingsbalken {2} poeren {3} ground floor.',
            'Clash_Data: member_type,member_id,storey,face_box_hit_count,first_mesh_face_index (member_type Wall, GableOpening, Beam, Foundation).',
            'Programme status is reported separately (Programme_Status) and does not change Grid_Status.'
        ]+programme_lines+branch_lines[:2]+wall_system['lines']+wall_material_report+frame['rows']+issues+programme_issues
    takeoff = quantity_takeoff(levels,slab,foundation_checks,fd,wall_rows_all,frame['cd'],frame['bd'],[name for name,k,b in frame['beams']],material_checks,programme_layout['apartment_rows'])
    tt = takeoff['totals']
    report = report[:2]+[inputs_line()]+report[2:]+takeoff['lines']
    RUN_INFO.clear()
    RUN_INFO.update(issues=[i for i in issues if not str(i).startswith('PROGRAMME')], max_u=frame.get('max_utilization'),
                    piles=(pile_result['quantities'].get('piles') if pile_result is not None else None))
    if inp('Debug'):
        stability = wall_system['stability'] if wall_system is not None else (frame_cores['stability'] if frame_cores else None)
        pq = pile_result['quantities'] if pile_result is not None else {}
        info = dict(
            INPUT=dict(version=SCRIPT_VERSION,clear_height=TARGET_CLEAR_HEIGHT,facade_load=FACADE_LOAD,core_count=CORE_COUNT,corridor_width=PROGRAMME_CORRIDOR_WIDTH,max_unit_width=MAX_UNIT_WIDTH,common_ratio=PROGRAMME_COMMON_RATIO,core_area=PROGRAMME_CORE_AREA,apartment_gen=APARTMENT_GEN,apartment_depth=APARTMENT_DEPTH,type_mix=TYPE_MIX,apt_manual_count=APT_MANUAL_COUNT,construction=construction,floor=floor,roof=roof,slab=slab,walls=walls_on,wind=list(wind),
                       piles=pile_cfg,areas=list(as_list(inp('AreaPerTypeOfApartment'))),names=[str(n) for n in as_list(inp('Apartment_Names'))],
                       angle=angle,scale=scale,levels=[list(p) for p in levels]),
            BALANCE=(wall_system['balance'] if wall_system is not None else dict(base_reaction=sum(frame['loads'].values()),cores=(frame_cores['mass']*9.81/1000.0 if frame_cores else 0.0),residual=frame['residual'],fa=frame['fa'],ra=frame['ra'])),
            GOVERNING=dict(members=governing_rows(material_checks),walls=governing_rows([','.join([r.split(',')[0],'',r.split(',')[5]]+['']*7+[r.split(',')[10]]) for r in wall_rows_all],(10,)),
                           max_member_U=frame['max_utilization'],max_wall_U=(wall_system['max_wall_u'] if wall_system is not None else (frame_cores['max_u'] if frame_cores else None))),
            STABILITY=(dict(status=stability['status'],worst=stability['worst'],results=stability.get('results'),
                            elements=dict((n,dict(U=v['U'],tension=v['tension'])) for n,v in stability.get('per',{}).items())) if stability else None),
            FOUNDATION=dict(status=foundation_status,label=pile_result['label'] if pile_result else None,diameter=pile_result['diameter'] if pile_result else None,
                            capacity=pile_result['capacity'] if pile_result else None,quantities=dict((k,v) for k,v in pq.items() if k not in ('beam_sizes','poer_list')),
                            pile_service_total=sum(float(r.split(',')[8]) for r in fd)/FOUNDATION_ULS if fd else 0.0,
                            elements=foundation_checks),
            PROGRAMME=dict(status=programme_status,data=programme_data),
            TAKEOFF=tt)
        report = report+debug_lines(info)
    return (columns,beams,foundations,frame['cd'],frame['bd'],fd,'\n'.join(report),xs,ys,status,clashes,clash_geometry,edge_status,edge_data,angle,facade_status,facade_rows,material_status,material_checks,quantities,foundation_status,foundation_checks,programme_status,programme_data,apartment_tree,apartment_data_tree,support_tree,support_data_tree,wall_tree,wall_data_tree,apartments_n,
            tt['foundation_beams_m1'],tt['piles_m1'],tt['party_walls_m2'],tt['core_walls_m2'],tt['internal_walls_m2'],tt['columns_m1'],tt['beams_m1'],takeoff['rows'])


# 10. Outputs and runtime messages

Columns,Beams,Foundations = [],[],[]
Column_Data,Beam_Data,Foundation_Data = [],[],[]
Grid_X_Coordinates,Grid_Y_Coordinates = [],[]
Clash_Data,Clash_Geometry = [],[]
Grid_Status,Report = 'ERROR',''
Edge_Check_Status,Edge_Check_Data = 'ERROR',[]
Detected_Grid_Angle,Facade_Status,Facade_Clash_Data = None,'ERROR',[]
Material_Check_Status,Member_Check_Data,Material_Quantities = 'ERROR',[],[]
Foundation_Check_Status,Foundation_Check_Data = 'ERROR',[]
Programme_Status,Programme_Data = 'ERROR',[]
Apartment_Boxes,Apartment_Data,Support_Boxes,Support_Data = [],[],[],[]
Walls,Wall_Data = [],[]
Apartments_N = []
Foundation_Beams_m1,Piles_m1,Party_Walls_m2,Core_Walls_m2,Internal_Walls_m2,Columns_m1,Beams_m1,Takeoff_Data = None,None,None,None,None,None,None,[]
def compact_message(report, grid_status, material_status, foundation_status, programme_status):
    lines = report.split('\n')
    def line_of(prefix):
        return next((l for l in lines if l.startswith(prefix)), '')
    def between(text, a, b):
        if a not in text:
            return ''
        rest = text.split(a, 1)[1]
        return rest.split(b, 1)[0] if b in rest else rest
    prog = line_of('Programme design')
    units = between(prog, '| ', ' apartments').strip()
    ratio = between(prog, 'NLA/GFA ', '%').strip()
    gen_line = line_of('APARTMENT_GEN ')
    eff = line_of('EFFICIENCY per storey')
    empty = 0.0
    for part in eff.split('(empty ')[1:]:
        try:
            empty += float(part.split(' m')[0])
        except ValueError:
            pass
    vacant = between(gen_line, 'Vacant slot area ', ' m2').strip()
    fit = []
    if programme_status.startswith('INCOMPLETE'):
        fit.append('NOT PLACED: '+between(line_of('Types that could not be placed'), ': ', ' (').strip())
    if programme_status.startswith('MIX_DEVIATION'):
        fit.append('MIX OFF: '+between(gen_line, 'Deviation: ', '.').strip())
    elif 'Target met' in gen_line:
        fit.append('target met')
    if programme_status.endswith('_LIMITED'):
        fit.append('types left out by MAX_UNIT_WIDTH')
    if vacant:
        fit.append('ROOM LEFT {} m2 vacant{} (fewer units asked than the bays hold)'.format(vacant, ', {:.0f} m of band'.format(empty) if empty > 1e-6 else ''))
    elif empty > 1e-6:
        fit.append('ROOM LEFT {:.1f} m of band empty'.format(empty))
    if not fit:
        fit.append('fits, no room left')
    fails = [n for n, v in (('members', material_status), ('foundation', foundation_status)) if 'FAIL' in str(v)]
    heads = []
    for issue in RUN_INFO.get('issues', []):
        head = str(issue).split(':', 1)[0].strip()
        head = head.split(' (')[0].strip().lower()
        if head and head not in heads:
            heads.append(head)
    state = 'FAIL' if fails or programme_status.startswith(('INCOMPLETE', 'INVALID')) else ('CHECK' if heads or programme_status != 'DESIGNED' else 'OK')
    top = '{} v{} | {} apts | NLA/GFA {}% | {} piles | U {:.3f}'.format(state, SCRIPT_VERSION, units or '-', ratio or '-', RUN_INFO.get('piles') if RUN_INFO.get('piles') is not None else '-', RUN_INFO.get('max_u') or 0.0)
    out = [top, 'PROGRAMME {}: {}'.format(programme_status, '; '.join(fit))]
    if fails:
        out.append('FAIL: '+', '.join(fails)+' (see Report)')
    out.append('GEOMETRY/STRUCTURE: '+('ok' if not heads else '; '.join(heads[:6])+(' (+{} more)'.format(len(heads)-6) if len(heads) > 6 else '')))
    return '\n'.join(out)


Message = ''
try:
    (Columns,Beams,Foundations,Column_Data,Beam_Data,Foundation_Data,Report,
     Grid_X_Coordinates,Grid_Y_Coordinates,Grid_Status,Clash_Data,Clash_Geometry,Edge_Check_Status,Edge_Check_Data,Detected_Grid_Angle,Facade_Status,Facade_Clash_Data,Material_Check_Status,Member_Check_Data,Material_Quantities,Foundation_Check_Status,Foundation_Check_Data,Programme_Status,Programme_Data,
     Apartment_Boxes,Apartment_Data,Support_Boxes,Support_Data,Walls,Wall_Data,Apartments_N,
     Foundation_Beams_m1,Piles_m1,Party_Walls_m2,Core_Walls_m2,Internal_Walls_m2,Columns_m1,Beams_m1,Takeoff_Data) = run()
    Message = compact_message(Report, Grid_Status, Material_Check_Status, Foundation_Check_Status, Programme_Status)
except Exception as error:
    Report = 'ERROR (v'+SCRIPT_VERSION+'): '+str(error)
    Message = 'ERROR v'+SCRIPT_VERSION+': '+str(error)
    ghenv.Component.AddRuntimeMessage(ghk.GH_RuntimeMessageLevel.Error,Report)
