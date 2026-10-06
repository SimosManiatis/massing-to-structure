import io
import os
import sys
import unittest
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stub

sg = stub.load()
TOL = 0.001


def walls(x, y, n, construction, h=3.0, setback=False):
    levels = [(0, x, 0, y, h * k, h * k + h) for k in range(n)]
    if setback:
        levels.append((0, x, 0, y / 2.0, h * n, h * n + h))
    levels = sg.normalize_levels(levels, TOL)
    lay, ws, t = sg.iterate_wall_design(levels, construction, 10, 8.5, 0.25, TOL,
                                        lambda a, m, e: sg.programme_check(levels, [], [25, 30, 40, 60, 90], wall_thickness=a, max_bay=m, end_thickness=e)[4])
    return levels, lay, ws


class Version(unittest.TestCase):
    def test_version(self):
        self.assertEqual(sg.SCRIPT_VERSION, '59')


class WindAnnex(unittest.TestCase):
    def test_qp_equals_table_nb5(self):
        table = {(1, 'sea'): {1: 0.93, 10: 1.58, 100: 2.38}, (2, 'open'): {4: 0.60, 10: 0.85, 30: 1.20, 100: 1.64},
                 (2, 'built'): {10: 0.68, 150: 1.65}, (3, 'built'): {10: 0.56, 200: 1.46}}
        for (area, terrain), rows in table.items():
            for z, qp in rows.items():
                self.assertAlmostEqual(sg.peak_pressure(z, area, terrain), qp, delta=0.006)

    def test_cpe_table_nb6(self):
        self.assertAlmostEqual(sg.wind_force_coefficient(1.0, 4.0)[0], 1.3 * 0.85)
        self.assertAlmostEqual(sg.wind_force_coefficient(5.0, 1.0)[0], 1.5 * 0.85)

    def test_figure_7_4(self):
        self.assertEqual(sg.wind_slices(0.0, 3.0, 21.0, 25.0), [(0.0, 3.0, 21.0)])
        self.assertEqual(sg.wind_slices(9.0, 12.0, 15.0, 10.0), [(9.0, 10.0, 10.0), (10.0, 12.0, 15.0)])


class WallMode(unittest.TestCase):
    def test_balance_and_full_height_columns(self):
        for c in ('Concrete', 'Steel', 'Timber'):
            levels, lay, ws = walls(25, 30, 6, c, setback=True)
            self.assertLess(abs(ws['balance']['difference']), 0.001)
            for name, k, b in ws['selected']['frame']['columns']:
                self.assertAlmostEqual(b[5], levels[k][5])
                self.assertAlmostEqual(b[4], levels[k][4])

    def test_tall_buildings_pass_foundation(self):
        for x, y, n in ((25, 20, 18), (25, 30, 14)):
            levels, lay, ws = walls(x, y, n, 'Concrete')
            self.assertEqual(ws['pile_result']['status'], 'PASS_ASSUMED_MODEL', (x, y, n, ws['pile_result']['quantities']['u_pile']))

    def test_tension_checked(self):
        levels, lay, ws = walls(25, 20, 18, 'Concrete')
        q = ws['pile_result']['quantities']
        if q['tension']:
            self.assertLessEqual(q['u_tension'], 1.0 + 1e-9)
            self.assertTrue(any('checked against Fr;t;d' in l for l in ws['lines']))


class PileClusters(unittest.TestCase):
    def test_overlapping_columns_merge(self):
        res = sg.pile_line(0.0, 20.0, [0.3, 8.8, 11.2, 19.7], [(0.0, 20.0, 40.0)],
                           [(0.3, 4400.0), (8.8, 4700.0), (11.2, 4700.0), (19.7, 4400.0)], [], 0.5, 1750.0, 0.45)
        self.assertEqual(res['status'], 'PASS_ASSUMED_MODEL')
        gaps = [b - a for a, b in zip(res['piles'], res['piles'][1:])]
        self.assertGreaterEqual(min(gaps), res['smin'] - 1e-6)

    def test_tension_limits_pile_group(self):
        sg.PILE_TENSION['value'] = 50.0
        try:
            res = sg.pile_group((0, 4, 0, 4), 1000.0, [(4000.0, 0.0)], 0.5, 1750.0)
            self.assertGreaterEqual(-res['rmin'], -50.0 - 1e-6)
            self.assertLessEqual(-res['rmin'], 50.0 + 1e-6)
        finally:
            sg.PILE_TENSION['value'] = None


class ClearHeightAndCounts(unittest.TestCase):
    def test_clear_height_drives_frame_beams(self):
        try:
            sg._ns['TARGET_CLEAR_HEIGHT'] = 2.40
            levels, lay, ws = walls(25, 20, 6, 'Concrete')
            text = ' '.join(ws['lines'])
            self.assertIn('vs target 2.40 m', text)
        finally:
            sg._ns['TARGET_CLEAR_HEIGHT'] = sg.TARGET_CLEAR_HEIGHT_DEFAULT

    def test_apartment_counts_match_mix(self):
        levels = sg.normalize_levels([(0, 25, 0, 20, 3.0 * k, 3.0 * k + 3) for k in range(6)], TOL)
        status, lines, issues, data, lay = sg.programme_check(levels, [], [25, 30, 40, 60, 90])
        counts = [lay['mix'].get(n, 0) for n in lay['type_order']]
        self.assertEqual(len(counts), 5)
        self.assertEqual(sum(counts), len(set(a[0].split('.')[0] for a in lay['apartments'])))


class Takeoff(unittest.TestCase):
    def test_takeoff_totals(self):
        levels, lay, ws = walls(25, 20, 6, 'Concrete')
        f = ws['selected']['frame']
        prog = sg.programme_check(levels, [], [25, 30, 40, 60, 90])[4]
        t = sg.quantity_takeoff(levels, 0.25, ws['element_rows'], ws['pile_rows'], ws['wall_rows'], f['cd'], f['bd'],
                                [n for n, k, b in f['beams']], f['checks'], prog['apartment_rows'])
        tt = t['totals']
        q = ws['pile_result']['quantities']
        self.assertAlmostEqual(tt['piles_m1'], q['pile_m1'], delta=0.05)
        self.assertAlmostEqual(tt['foundation_beams_m1'], q['beam_m1'], delta=0.05)
        self.assertAlmostEqual(tt['columns_m1'], 3.0 * len(f['cd']), places=6)
        walls_m3 = sum(float(r.split(',')[6]) for r in t['rows'] if r.endswith('wall') is False and ('party wall' in r or 'core wall' in r))
        self.assertAlmostEqual(walls_m3, ws['wall_volume'], delta=1e-3)
        self.assertGreater(tt['internal_walls_m2'], 0.0)
        self.assertTrue(all(len(r.split(',')) == 7 for r in t['rows']))

    def test_internal_wall_rule(self):
        self.assertEqual(sg.bedrooms_of('3 bedroom', 60), 3)
        self.assertEqual(sg.bedrooms_of('Studio', 25), 0)
        self.assertAlmostEqual(sg.internal_wall_length('2 bedroom', 40), 4.0 + 9.0 + 2.0)


class FacadeAndCores(unittest.TestCase):
    def test_facade_load_scales_balance(self):
        levels, lay, ws = walls(25, 20, 6, 'Concrete')
        base = ws['balance']['facade']
        try:
            sg._ns['FACADE_LOAD'] = 3.0
            levels, lay, ws = walls(25, 20, 6, 'Concrete')
            self.assertAlmostEqual(ws['balance']['facade'], 2.0 * base, delta=1e-6)
            self.assertLess(abs(ws['balance']['difference']), 0.001)
        finally:
            sg._ns['FACADE_LOAD'] = sg.FACADE_LOAD_DEFAULT

    def test_core_count(self):
        levels = sg.normalize_levels([(0, 25, 0, 20, 3.0 * k, 3.0 * k + 3) for k in range(18)], TOL)
        try:
            for n in (1, 2, 3):
                sg._ns['CORE_COUNT'] = n
                lay = sg.programme_check(levels, [], [25, 30, 40, 60, 90])[4]
                self.assertEqual(sum(1 for b in lay['bays'] if b['kind'] == 'core'), n)
            sg._ns['CORE_COUNT'] = 2
            levels2, lay, ws = walls(25, 20, 18, 'Concrete')
            self.assertEqual(sum(1 for n in ws['stability']['per'] if n.startswith('core')), 2)
            sg._ns['CORE_COUNT'] = 9
            self.assertRaises(ValueError, sg.programme_check, levels, [], [25, 30, 40, 60, 90])
        finally:
            sg._ns['CORE_COUNT'] = None


class InputLookup(unittest.TestCase):
    def test_names_case_and_spaces(self):
        try:
            sg._ns['facade_load'] = 3.5
            self.assertEqual(sg.number('Facade_Load', 1.5, False), 3.5)
            self.assertIn('Facade_Load 3.5', sg.inputs_line())
        finally:
            del sg._ns['facade_load']
        self.assertEqual(sg.number('Facade_Load', 1.5, False), 1.5)
        try:
            sg._ns['Apartment_Depth'] = ''
            self.assertIn('Apartment_Depth -', sg.inputs_line())
            self.assertIsNone(sg.optional_number('Apartment_Depth', None))
        finally:
            del sg._ns['Apartment_Depth']
        try:
            sg._ns['Core_Area'] = 30
            self.assertIn('IGNORED component inputs (now set at the top of the script): Core_Area', sg.inputs_line())
        finally:
            del sg._ns['Core_Area']
        self.assertNotIn('Floor_Load', sg.INPUT_NAMES)


class ProgrammeInputs(unittest.TestCase):
    AREAS = [25, 30, 40, 60, 90]

    def levels(self):
        return sg.normalize_levels([(0, 25, 0, 15, 3.2 * k, 3.2 * k + 3.2) for k in range(12)], TOL)

    def test_corridor_width(self):
        levels = self.levels()
        try:
            sg._ns['PROGRAMME_CORRIDOR_WIDTH'] = 2.4
            lay = sg.programme_check(levels, [], self.AREAS)[4]
            self.assertAlmostEqual(lay['core_depth'], (15 - 2.4) / 2.0)
        finally:
            sg._ns['PROGRAMME_CORRIDOR_WIDTH'] = sg.CORRIDOR_WIDTH_DEFAULT
        lay = sg.programme_check(levels, [], self.AREAS)[4]
        self.assertAlmostEqual(lay['core_depth'], 6.6)

    def test_core_area(self):
        levels = self.levels()
        try:
            sg._ns['PROGRAMME_CORE_AREA'] = 40.0
            lay = sg.programme_check(levels, [], self.AREAS)[4]
            core = [b for b in lay['bays'] if b['kind'] == 'core'][0]
            self.assertAlmostEqual(core['end'] - core['start'], 40.0 / 6.6, delta=0.01)
        finally:
            sg._ns['PROGRAMME_CORE_AREA'] = sg.CORE_AREA_DEFAULT

    def test_common_ratio(self):
        levels = self.levels()
        try:
            sg._ns['PROGRAMME_COMMON_RATIO'] = 0.0
            lay = sg.programme_check(levels, [], self.AREAS)[4]
            self.assertNotIn('common', lay['support_kinds'])
            sg._ns['PROGRAMME_COMMON_RATIO'] = 0.08
            st, out, issues, data, lay = sg.programme_check(levels, [], self.AREAS)
            self.assertIn('common', lay['support_kinds'])
            self.assertGreater(float(data[0].split(',')[9]), 160.0)
        finally:
            sg._ns['PROGRAMME_COMMON_RATIO'] = sg.COMMON_RATIO_DEFAULT

    def test_max_unit_width(self):
        levels = self.levels()
        try:
            sg._ns['MAX_UNIT_WIDTH'] = 5.5
            st, out, issues, data, lay = sg.programme_check(levels, [], self.AREAS, wall_thickness=0.2, max_bay=5.87, end_thickness=0.4)
            self.assertTrue(st.endswith('_LIMITED'))
            for row in lay['apartment_rows']:
                self.assertLessEqual(float(row.split(',')[4]), 5.5 + 1e-6)
            self.assertEqual(lay['mix'].get('3 bedroom', 0), 0)
            self.assertFalse([p for p, kind in lay['lines'] if kind == 'internal'])
            self.assertTrue(any('Max_Unit_Width' in i for i in issues))
        finally:
            sg._ns['MAX_UNIT_WIDTH'] = None


class ApartmentGen(unittest.TestCase):
    AREAS = [25, 30, 40, 60, 90]

    def run_gen(self, gen, mix=None, count=None):
        levels = sg.normalize_levels([(0, 25, 0, 15, 3.2 * k, 3.2 * k + 3.2) for k in range(12)], TOL)
        try:
            sg._ns['APARTMENT_GEN'], sg._ns['TYPE_MIX'], sg._ns['APT_MANUAL_COUNT'] = gen, mix, count
            return sg.programme_check(levels, [], self.AREAS, wall_thickness=0.2, max_bay=5.87, end_thickness=0.4)
        finally:
            sg._ns['APARTMENT_GEN'], sg._ns['TYPE_MIX'], sg._ns['APT_MANUAL_COUNT'] = 0, None, None

    def test_type_mix_shares(self):
        st, out, issues, data, lay = self.run_gen(1, mix=[30, 30, 20, 15, 5])
        self.assertEqual(st, 'DESIGNED')
        total = float(sum(lay['mix'].values()))
        for name, share in zip(sg.APARTMENT_TYPE_NAMES, [0.30, 0.30, 0.20, 0.15, 0.05]):
            self.assertLessEqual(abs(lay['mix'].get(name, 0) / total - share), sg.MIX_SHARE_TOLERANCE + 1e-9)
        st, out, issues, data, lay = self.run_gen(1, mix=[0, 0.5, 0.5, 0, 0])
        self.assertEqual(lay['mix'].get('Studio', 0), 0)
        total = float(sum(lay['mix'].values()))
        self.assertLessEqual(abs(lay['mix'].get('1 bedroom', 0) / total - 0.5), sg.MIX_SHARE_TOLERANCE + 1e-9)
        self.assertLessEqual(abs(lay['mix'].get('2 bedroom', 0) / total - 0.5), sg.MIX_SHARE_TOLERANCE + 1e-9)

    def test_manual_count_exact(self):
        st, out, issues, data, lay = self.run_gen(2, count=[0, 40, 0, 0, 1])
        self.assertEqual(st, 'DESIGNED')
        self.assertEqual(lay['mix'].get('1 bedroom', 0), 40)
        self.assertEqual(lay['mix'].get('Penthouse', 0), 1)
        self.assertEqual(sum(lay['mix'].values()), 41)
        self.assertTrue(any('Vacant slot area' in line for line in out))

    def test_manual_count_impossible(self):
        st, out, issues, data, lay = self.run_gen(2, count=[200, 0, 0, 0, 0])
        self.assertEqual(st, 'MIX_DEVIATION')
        self.assertTrue(any('MIX DEVIATION' in i for i in issues))

    def test_mode_zero_unchanged(self):
        st, out, issues, data, lay = self.run_gen(0)
        self.assertEqual(sum(lay['mix'].values()), 79)


class Overhangs(unittest.TestCase):
    def design(self, upper, construction='Concrete'):
        levels = sg.normalize_levels([(0, 25, 0, 15, 3.2 * k, 3.2 * k + 3.2) if k < 4 else upper(k) for k in range(12)], TOL)
        cfg = dict(diameter=None, capacity=None, tension=None, peil=1.4, tip=-25.5)
        lay, ws, a = sg.iterate_wall_design(levels, construction, 10, 8.5, 0.25, TOL,
                                            lambda a, m, e: sg.programme_check(levels, [], [25, 30, 40, 60, 90], wall_thickness=a, max_bay=m, end_thickness=e)[4], (2, 'open'), cfg)
        return levels, lay, ws

    def test_levels_accept_overhang(self):
        levels = sg.normalize_levels([(0, 25, 0, 15, 0, 3), (0, 28, -2, 15, 3, 6)], TOL)
        self.assertTrue(sg.has_overhang(levels, TOL))
        self.assertRaises(ValueError, sg.normalize_levels, [(0, 25, 0, 15, 0, 3), (30, 40, 0, 15, 3, 6)], TOL)

    def test_balance_and_members(self):
        cases = [lambda k: (0, 27, 0, 15, 3.2 * k, 3.2 * k + 3.2),
                 lambda k: (0, 31, 0, 15, 3.2 * k, 3.2 * k + 3.2),
                 lambda k: (0, 25, 0, 19, 3.2 * k, 3.2 * k + 3.2),
                 lambda k: (-3, 25, -1.5, 16.5, 3.2 * k, 3.2 * k + 3.2)]
        for upper in cases:
            levels, lay, ws = self.design(upper)
            self.assertLess(abs(ws['balance']['difference']), 1e-6)
            self.assertTrue(any(line.startswith('OVERHANGS') for line in ws['lines']))
            self.assertEqual(ws['pile_result']['status'], 'PASS_ASSUMED_MODEL')
        levels, lay, ws = self.design(cases[1])
        self.assertTrue(any(b.get('overhang') for b in lay['bays']))
        self.assertTrue(any('_overhangB' in name for name, k, b in ws['selected']['frame']['beams']))
        levels, lay, ws = self.design(cases[2])
        self.assertFalse([name for name, k, kind, b in ws['walls'] if kind == 'cantilever'])
        self.assertTrue(any('_overhangC' in name for name, k, b in ws['selected']['frame']['beams']))
        self.assertFalse([i for i in ws['overhang_issues'] if 'OVERHANG WALL' in i])
        levels, lay, ws = self.design(lambda k: (0, 25, 0, 21, 3.2 * k, 3.2 * k + 3.2), 'Timber')
        self.assertFalse([name for name, k, kind, b in ws['walls'] if kind == 'cantilever'])
        self.assertTrue([i for i in ws['overhang_issues'] if 'OVERHANG STEEL INSERTS' in i])
        original = sg._ns['select_cantilever']
        def no_beam(*a, **kw):
            raise ValueError('forced')
        try:
            sg._ns['select_cantilever'] = no_beam
            levels, lay, ws = self.design(cases[2])
        finally:
            sg._ns['select_cantilever'] = original
        cant = [name for name, k, kind, b in ws['walls'] if kind == 'cantilever']
        self.assertTrue(cant)
        frame_lines = [i for i, (pos, kind) in enumerate(lay['lines']) if kind == 'internal']
        if [n for n in cant if int(n.split('_')[2]) in frame_lines]:
            self.assertTrue([i for i in ws['overhang_issues'] if 'OVERHANG NOT CLAMPED' in i])
        self.assertTrue([i for i in ws['overhang_issues'] if 'OVERHANG WALL (REVIEW)' in i])
        levels, lay, ws = self.design(cases[0])
        self.assertTrue(any('_overhangA' in name for name, k, b in ws['selected']['frame']['beams']))

    def test_merge_boxes_per_storey(self):
        boxes = [(0, 25, 0, 15, 3.0 * k, 3.0 * k + 3) for k in range(4)] + [(0, 25, 15, 18, 6, 9), (25, 28, 0, 15, 9, 12)]
        merged = sg.merge_storey_boxes(boxes, TOL)
        self.assertEqual(len(merged), 4)
        self.assertEqual(tuple(merged[2]), (0, 25, 0, 18, 6, 9))
        self.assertEqual(tuple(merged[3]), (0, 28, 0, 15, 9, 12))
        levels = sg.normalize_levels(merged, TOL)
        self.assertTrue(sg.has_overhang(levels, TOL))
        self.assertRaises(ValueError, sg.merge_storey_boxes, [(0, 25, 0, 15, 0, 3), (25, 28, 0, 5, 0, 3)], TOL)
        tall = sg.merge_storey_boxes([(0, 25, 0, 15, 0, 3), (0, 25, 0, 15, 3, 6), (25, 28, 0, 15, 0, 6.005)], TOL)
        self.assertEqual([tuple(b) for b in tall], [(0, 28, 0, 15, 0, 3), (0, 28, 0, 15, 3, 6)])
        self.assertRaises(ValueError, sg.merge_storey_boxes, [(0, 25, 0, 15, 0, 3), (0, 25, 0, 15, 3, 6), (25, 28, 0, 15, 3, 3.3)], TOL)
        self.assertRaises(ValueError, sg.merge_storey_boxes, [(0, 25, 0, 15, 0, 3), (0, 25, 0, 15, 4, 7)], TOL)

    def test_tip_edges_stretch_and_headroom(self):
        levels, lay, ws = self.design(lambda k: (0, 25, -2, 15, 3.2 * k, 3.2 * k + 3.2))
        self.assertTrue(any('_overhangEdge' in name for name, k, b in ws['selected']['frame']['beams']))
        self.assertLess(abs(ws['balance']['difference']), 1e-6)
        levels = sg.normalize_levels([(0, 25, 0, 15, 3.2 * k, 3.2 * k + 3.2) if k < 4 else (0, 27, 0, 15, 3.2 * k, 3.2 * k + 3.2) for k in range(12)], TOL)
        st, out, issues, data, lay = sg.programme_check(levels, [], [25, 30, 40, 60, 90], wall_thickness=0.2, max_bay=5.87, end_thickness=0.4)
        self.assertTrue(any('extended into overhang strips' in line for line in out))
        self.assertTrue(any(float(r.split(',')[8]) > 25.0 for r in lay['apartment_rows'] if r.split(',')[1] == '5'))
        levels, lay, ws = self.design(lambda k: (0, 27, 0, 15, 3.2 * k, 3.2 * k + 3.2), 'Timber')
        self.assertTrue(any('BACK-SPAN HEADROOM' in i for i in ws['overhang_issues']))

    def test_no_overhang_unchanged(self):
        levels, lay, ws = self.design(lambda k: (0, 25, 0, 15, 3.2 * k, 3.2 * k + 3.2))
        self.assertEqual(ws['pile_result']['quantities']['piles'], 87)
        self.assertFalse(any(line.startswith('OVERHANGS') for line in ws['lines']))


class BaySetSearch(unittest.TestCase):
    SIMOS_B = [[4.197, 24.197, 2.9525, 22.883, 0.0, 3.2], [4.197, 28.197, 2.9525, 22.883, 3.2, 6.4], [4.197, 24.197, 2.9525, 22.883, 6.4, 9.6],
               [4.197, 24.197, 2.9525, 26.883, 9.6, 12.8], [4.197, 24.197, 2.9525, 22.883, 12.8, 16.0], [0.197, 24.197, 2.9525, 22.883, 16.0, 19.2],
               [4.197, 24.197, 2.9525, 22.883, 19.2, 22.4], [4.197, 24.197, -1.0475, 22.883, 22.4, 25.6], [4.197, 24.197, 2.9525, 22.883, 25.6, 28.8]]

    def run_gen(self, levels, gen, mix=None, count=None):
        levels = sg.normalize_levels(levels, TOL)
        try:
            sg._ns['APARTMENT_GEN'], sg._ns['TYPE_MIX'], sg._ns['APT_MANUAL_COUNT'] = gen, mix, count
            return sg.programme_check(levels, [], [25, 30, 40, 60, 90], wall_thickness=0.2, max_bay=5.87, end_thickness=0.4)
        finally:
            sg._ns['APARTMENT_GEN'], sg._ns['TYPE_MIX'], sg._ns['APT_MANUAL_COUNT'] = 0, None, None

    def nla_ratio(self, out):
        return float(out[0].split('NLA/GFA ')[1].rstrip('%.')) / 100.0

    def test_simos_building_type_mix(self):
        st, out, issues, data, lay = self.run_gen(self.SIMOS_B, 1, mix=[0.3, 0.19, 0.41, 0.15, 0.05])
        self.assertEqual(st, 'DESIGNED')
        self.assertGreaterEqual(self.nla_ratio(out), 0.44)

    def test_simos_building_counts_exact(self):
        st, out, issues, data, lay = self.run_gen(self.SIMOS_B, 2, count=[8, 8, 8, 8, 2])
        self.assertEqual(st, 'DESIGNED')
        self.assertEqual([lay['mix'].get(n, 0) for n in sg.APARTMENT_TYPE_NAMES], [8, 8, 8, 8, 2])

    def test_rectangular_type_mix_efficiency(self):
        st, out, issues, data, lay = self.run_gen([[0, 25, 0, 15, 3.2 * k, 3.2 * k + 3.2] for k in range(12)], 1, mix=[30, 30, 20, 15, 5])
        self.assertEqual(st, 'DESIGNED')
        self.assertGreaterEqual(self.nla_ratio(out), 0.55)
        self.assertEqual(sum(lay['mix'].values()), len(set(r.split(',')[0].split('.')[0] for r in lay['apartment_rows'])))


B_SIMOS = [[4.197, 24.197, 2.9525, 22.883, 0.0, 3.2], [4.197, 28.197, 2.9525, 22.883, 3.2, 6.4], [4.197, 24.197, 2.9525, 22.883, 6.4, 9.6],
           [4.197, 24.197, 2.9525, 26.883, 9.6, 12.8], [4.197, 24.197, 2.9525, 22.883, 12.8, 16.0], [0.197, 24.197, 2.9525, 22.883, 16.0, 19.2],
           [4.197, 24.197, 2.9525, 22.883, 19.2, 22.4], [4.197, 24.197, -1.0475, 22.883, 22.4, 25.6], [4.197, 24.197, 2.9525, 22.883, 25.6, 28.8]]
B_SIMOS_2 = [[4.197, 24.197, 2.9525, 22.883, 0.0, 3.2], [4.197, 24.197, 2.9525, 22.883, 3.2, 6.4], [4.197, 24.197, 2.9525, 22.883, 6.4, 9.6],
             [0.197, 24.197, 2.9525, 22.883, 9.6, 12.8], [0.197, 24.197, 2.9525, 22.883, 12.8, 16.0], [0.197, 24.197, 2.9525, 22.883, 16.0, 19.2],
             [4.197, 27.197, 2.9525, 22.883, 19.2, 22.4], [4.197, 24.197, 2.9525, 22.883, 22.4, 25.6], [4.197, 24.197, -0.0475, 22.883, 25.6, 28.8]]
B_RECT = [[0, 25, 0, 15, 3.2 * k, 3.2 * k + 3.2] for k in range(12)]
BASE_INPUTS = dict(Floor_Load=10.0, Roof_Load=8.5, Slab_Thickness=0.25, AreaPerTypeOfApartment=[25, 30, 40, 60, 90], Load_Bearing_Walls=True,
                   Clear_Height=2.4, Facade_Load=1.5, Core_Count=1, Common_Ratio=0.05, Debug=True)


def run_full(boxes, construction, **extra):
    import json
    import fake_rhino
    kw = dict(BASE_INPUTS)
    kw.update(extra)
    kw['Construction'] = construction
    ns = fake_rhino.run_component(boxes, **kw)
    report = ns['Report']
    dbg = {}
    for line in report.split('\n'):
        if line.startswith('DBG|'):
            key, value = line[4:].split('|', 1)
            dbg[key] = json.loads(value)
    return ns, report, dbg


class SupportBoxes(unittest.TestCase):
    def test_core_and_corridor_stack_and_overhang_bays_fill(self):
        levels = sg.normalize_levels(B_SIMOS, TOL)
        try:
            sg._ns['APARTMENT_GEN'] = 1
            sg._ns['TYPE_MIX'] = [0.3, 0.19, 0.41, 0.15, 0.05]
            st, out, issues, data, lay = sg.programme_check(levels, [], [25, 30, 40, 60, 90], wall_thickness=0.2, max_bay=5.87, end_thickness=0.38)
        finally:
            sg._ns['APARTMENT_GEN'] = 0
            sg._ns['TYPE_MIX'] = None
        rows = [r.split(',') for r in lay['support_rows']]
        for kind in ('core', 'circulation'):
            spans = set((round(float(r[6]), 3), round(float(r[7]), 3)) for r in rows if r[1] == kind)
            self.assertEqual(len(spans), 1, kind)
        self.assertTrue(any(line.startswith('DEEP UNITS') for line in out))
        self.assertEqual(sorted(set(r[1] for r in rows)), ['circulation', 'common', 'core'])
        self.assertEqual(sg.SUPPORT_KINDS, ('core', 'circulation', 'common'))
        geo = lay['geo']
        for a in [r.split(',') for r in lay['apartment_rows']]:
            k, side, y0, y1 = int(a[1]), int(a[6]), float(a[9]), float(a[10])
            g = geo[k]
            inner = g['zone'][0] if side == 0 else g['zone'][1]
            self.assertAlmostEqual(y1 if side == 0 else y0, inner, places=3)
            self.assertAlmostEqual(float(a[13]), float(a[4]) * float(a[5]), delta=0.05)
        over = [b for b in lay['bays'] if b.get('overhang')]
        self.assertTrue(over)
        apts = [r.split(',') for r in lay['apartment_rows']]
        for b in over:
            self.assertTrue([a for a in apts if float(a[7]) >= b['start'] - 0.5 and float(a[8]) <= b['end'] + 0.5])
        self.assertFalse([kind for pos, kind in lay['lines'] if kind == 'wall' and any(abs(pos - b['start']) < 1e-6 or abs(pos - b['end']) < 1e-6 for b in over)])


class CoreBayAndDepth(unittest.TestCase):
    def test_core_bay_free_side_holds_units(self):
        levels = sg.normalize_levels(B_RECT, TOL)
        st, out, issues, data, lay = sg.programme_check(levels, [], [25, 30, 40, 60, 90], wall_thickness=0.2, max_bay=5.87, end_thickness=0.4)
        core = [b for b in lay['bays'] if b['kind'] == 'core'][0]
        on_core_side = [r for r in lay['apartment_rows'] if r.split(',')[6] == '1' and core['start'] - 0.01 <= float(r.split(',')[7]) and float(r.split(',')[8]) <= core['end'] + 0.01]
        self.assertEqual(len(on_core_side), 12)
        self.assertTrue(any(line.startswith('EFFICIENCY') for line in out))

    def test_apartment_depth(self):
        levels = sg.normalize_levels([[0, 20, 0, 20, 3.2 * k, 3.2 * k + 3.2] for k in range(6)], TOL)
        try:
            sg._ns['APARTMENT_DEPTH'] = 7.0
            st, out, issues, data, lay = sg.programme_check(levels, [], [25, 30, 40, 60, 90])
            self.assertAlmostEqual(lay['core_depth'], 7.0)
        finally:
            sg._ns['APARTMENT_DEPTH'] = None
        st, out, issues, data, lay = sg.programme_check(levels, [], [25, 30, 40, 60, 90])
        self.assertAlmostEqual(lay['core_depth'], 9.1)
        self.assertTrue(any(line.startswith('Unit types smaller') for line in out))

    def test_overhang_strip_with_facade_end_holds_units(self):
        levels = sg.normalize_levels(B_SIMOS, TOL)
        st, out, issues, data, lay = sg.programme_check(levels, [], [25, 30, 40, 60, 90], wall_thickness=0.2, max_bay=5.87, end_thickness=0.4)
        strips = [b for b in lay['bays'] if b.get('overhang')]
        self.assertTrue(strips)
        self.assertTrue(all(b['kind'] == 'unit' for b in strips))


class EndToEnd(unittest.TestCase):
    def test_rectangular_concrete_outputs(self):
        ns, report, dbg = run_full(B_RECT, 'Concrete')
        self.assertIn('MATERIALS v' + sg.SCRIPT_VERSION, report)
        self.assertEqual(len(ns['Walls'].branches), 3)
        self.assertEqual(len(ns['Apartments_N'].branches), 5)
        self.assertGreater(len(ns['Columns']), 0)
        self.assertLess(abs(dbg['BALANCE']['difference']), 1e-6)

    def test_null_brep_reported(self):
        import fake_rhino
        fake_rhino.install()
        boxes = [fake_rhino.Brep((0, 0, 3.2 * k), (25, 15, 3.2 * k + 3.2)) for k in range(4)]
        values = dict(BASE_INPUTS, Construction='Concrete', Breps=boxes + [None])
        component = fake_rhino.Component(values)
        ns = dict(values, __name__='structuralgen', ghenv=fake_rhino.types.SimpleNamespace(Component=component))
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'structuralgen.py')
        exec(compile(io.open(path, encoding='utf-8').read(), path, 'exec'), ns)
        self.assertIn('empty (null)', ns['Report'])


class Regression(unittest.TestCase):
    CASES = {
        'rect_concrete': (B_RECT, 'Concrete', {}, dict(piles=87, nla=68.3, umax=0.9976)),
        'rect_timber': (B_RECT, 'Timber', {}, dict(piles=98, nla=68.3, umax=0.9979)),
        'simos_concrete_mix': (B_SIMOS, 'Concrete', dict(Apartment_GEN=1, Type_mix=[0.3, 0.19, 0.41, 0.15, 0.05]), dict(piles=73, nla=53.4, umax=0.9983)),
        'simos_steel_mix': (B_SIMOS, 'Steel', dict(Apartment_GEN=1, Type_mix=[0.3, 0.19, 0.41, 0.15, 0.05]), dict(piles=70, nla=53.4, umax=0.9995)),
        'simos2_concrete_mix': (B_SIMOS_2, 'Concrete', dict(Apartment_GEN=1, Type_mix=[0.3, 0.3, 0.2, 0.15, 0.05]), dict(piles=71, nla=53.2, umax=0.9936)),
        'simos2_timber_mix': (B_SIMOS_2, 'Timber', dict(Apartment_GEN=1, Type_mix=[0.3, 0.3, 0.2, 0.15, 0.05]), dict(piles=65, nla=53.2, umax=0.9999)),
    }

    def test_reference_buildings(self):
        for name, (boxes, construction, extra, expected) in self.CASES.items():
            ns, report, dbg = run_full(boxes, construction, **extra)
            nla = float(report.split('NLA/GFA ')[1].split('%')[0])
            self.assertLessEqual(abs(dbg['FOUNDATION']['quantities']['piles'] - expected['piles']), max(1, 0.02 * expected['piles']), name)
            self.assertLessEqual(abs(nla - expected['nla']), 2.0, name)
            self.assertLessEqual(dbg['GOVERNING']['max_member_U'], 1.0, name)
            self.assertLess(abs(dbg['BALANCE']['difference']), 1e-6, name)
            self.assertIn('PASS', report.split('Material check status: ')[1].split('\n')[0], name)
            self.assertFalse(ns['_messages'], name)
            self.assertTrue(ns['Message'].split(' ')[0] in ('OK', 'CHECK'), name)
            self.assertIn('PROGRAMME ', ns['Message'], name)


if __name__ == '__main__':
    unittest.main(verbosity=1)
