"""Extract the published curves of Figs 6-19 from the paper's vector graphics.

The figures in the published PDF are vector drawings exported by MATLAB, so
the plotted polylines can be read back exactly instead of digitised from a
raster.  Each panel is calibrated from its own grid lines: the 11 vertical
grid lines are t = 0, 0.5, ..., 5 s and the horizontal grid lines sit at the
y ticks, whose labelled values are listed in PANELS below (read from the
rendered page; the tick labels themselves are glyph outlines, not text).

Usage:  python3 tools/extract_paper_curves.py path/to/paper.pdf
Writes  data/paper_curves/<fig>_<panel>_<series>.csv  with columns t,y, and
        data/paper_grid/<step|sine>_<series>_<q|e|tau>.csv  on the 0.01 s grid.
"""
import csv, os, sys
from collections import defaultdict
import pymupdf as fitz

# (page index 0-based, figure, panel, lowest labelled y tick, y tick step,
#  number of y ticks, colour -> series)
STEP_A = {'r': 'dir', 'g': 'PID', 'b': 'FOPID', 'k': 'FBPA'}
STEP_B = {'g': 'PID', 'b': 'FOPID', 'k': 'FBPA'}
SINE_A = {'r': 'dir', 'g': 'PID', 'b': 'FOPID', 'k': 'FBPA'}
SINE_B = {'g': 'PID', 'r': 'FOPID', 'k': 'FBPA'}
TORQ12 = {'g': 'PID', 'b': 'FOPID', 'k': 'FBPA'}
TORQ19 = {'g': 'PID', 'r': 'FOPID', 'k': 'FBPA'}

# panels listed top-to-bottom, left-to-right on each page
PANELS = {
    7:  [('fig06', 'a', 0, .5, STEP_A), ('fig06', 'b', -.5, .5, STEP_B),
         ('fig07', 'a', 0, .5, STEP_A), ('fig07', 'b', -.6, .2, STEP_B),
         ('fig08', 'a', -.2, .2, STEP_A), ('fig08', 'b', -.2, .2, STEP_B)],
    8:  [('fig09', 'a', -1.5, .5, STEP_A), ('fig09', 'b', -.5, .5, STEP_B),
         ('fig10', 'a', -.5, .5, STEP_A), ('fig10', 'b', -2, .5, STEP_B),
         ('fig11', 'a', 0, .2, STEP_A), ('fig11', 'b', -.8, .2, STEP_B)],
    9:  [('fig12', 'tau1', -5e9, 1e9, TORQ12), ('fig12', 'tau2', -5e9, 1e9, TORQ12),
         ('fig12', 'tau3', -8e8, 2e8, TORQ12), ('fig12', 'tau4', -2e8, 2e8, TORQ12),
         ('fig12', 'tau5', -8e7, 2e7, TORQ12), ('fig12', 'tau6', -2e7, 1e7, TORQ12),
         ('fig13', 'a', -1.5, .5, SINE_A), ('fig13', 'b', -.6, .2, SINE_B)],
    10: [('fig14', 'a', -1.5, .5, SINE_A), ('fig14', 'b', -.1, .05, SINE_B),
         ('fig15', 'a', -1.5, .5, SINE_A), ('fig15', 'b', -.4, .1, SINE_B),
         ('fig16', 'a', -1, .5, SINE_A), ('fig16', 'b', -.02, .02, SINE_B),
         ('fig17', 'a', -1.5, .5, SINE_A), ('fig17', 'b', -.5, .1, SINE_B)],
    11: [('fig18', 'a', -1.5, .5, SINE_A), ('fig18', 'b', -.05, .01, SINE_B),
         ('fig19', 'tau1', -1e4, .5e4, TORQ19), ('fig19', 'tau2', -3500, 500, TORQ19),
         ('fig19', 'tau3', -3000, 1000, TORQ19), ('fig19', 'tau4', 0, 200, TORQ19),
         ('fig19', 'tau5', -200, 50, TORQ19), ('fig19', 'tau6', -.8, .2, TORQ19)],
}

BEZIER_SAMPLES = 24

COLOURS = {(1, 0, 0): 'r', (0, 1, 0): 'g', (0, 0, 1): 'b', (0, 0, 0): 'k'}


def colour_key(c):
    if c is None:
        return None
    return COLOURS.get(tuple(round(x) for x in c)) if all(
        abs(x - round(x)) < 0.02 for x in c) else None


def find_panels(drawings):
    """Group the light-grey grid lines into panels."""
    vert = defaultdict(list)   # (y0, y1) -> x positions
    horiz = defaultdict(list)  # (x0, x1) -> y positions
    for d in drawings:
        c = d.get('color')
        if c is None or abs(c[0] - 0.87) > 0.02 or d.get('fill') is not None:
            continue
        for it in d['items']:
            if it[0] != 'l':
                continue
            p, q = it[1], it[2]
            if abs(p.x - q.x) < 0.05:
                vert[(round(min(p.y, q.y), 1), round(max(p.y, q.y), 1))].append(p.x)
            elif abs(p.y - q.y) < 0.05:
                horiz[(round(min(p.x, q.x), 1), round(max(p.x, q.x), 1))].append(p.y)
    panels = []
    groups = []
    for (y0, y1), xs in vert.items():
        xs = sorted(set(round(x, 2) for x in xs))
        # two panels side by side can share the same vertical extent: every
        # panel has exactly 11 vertical grid lines, so split in runs of 11
        for k in range(0, len(xs) - 10, 11):
            groups.append((y0, y1, xs[k:k + 11]))
    for (y0, y1, xs) in groups:
        if len(xs) != 11:
            continue
        # horizontal grid lines spanning the same x range
        ys = []
        for (x0, x1), yy in horiz.items():
            if abs(x0 - xs[0]) < 1 and abs(x1 - xs[-1]) < 1:
                ys += yy
        ys = sorted(set(round(y, 2) for y in ys if y0 - 0.5 <= y <= y1 + 0.5), reverse=True)
        panels.append(dict(x=xs, yticks=ys, top=y0, bottom=y1))
    # order: top-to-bottom, then left-to-right
    panels.sort(key=lambda P: (round(P['top'] / 50), P['x'][0]))
    return panels


def legend_boxes(drawings, P):
    boxes = []
    for d in drawings:
        c = d.get('color')
        if c is None or abs(c[0] - 0.15) > 0.03:
            continue
        for it in d['items']:
            if it[0] == 're':
                r = it[1]
                if P['x'][0] <= r.x0 and r.x1 <= P['x'][-1] + 1 and P['top'] - 1 <= r.y0 and r.y1 <= P['bottom']:
                    boxes.append(r)
    return boxes


def extract(pdf, outdir):
    doc = fitz.open(pdf)
    os.makedirs(outdir, exist_ok=True)
    report = []
    for pno, specs in PANELS.items():
        page = doc[pno]
        drawings = page.get_drawings()
        panels = find_panels(drawings)
        if len(panels) != len(specs):
            raise RuntimeError(f'page {pno+1}: found {len(panels)} panels, expected {len(specs)}')
        for P, (fig, pan, ylo, ystep, cmap) in zip(panels, specs):
            xs = P['x']
            ax = (xs[-1] - xs[0]) / 5.0
            yt = P['yticks']                      # page y, bottom tick first
            # map page-y -> data: lowest tick = ylo, next = ylo + ystep
            dy = (yt[0] - yt[-1]) / (len(yt) - 1)  # page units per tick
            to_t = lambda x: (x - xs[0]) / ax
            to_y = lambda y: ylo + (yt[0] - y) / dy * ystep
            legends = legend_boxes(drawings, P)
            series = defaultdict(list)
            for d in drawings:
                key = colour_key(d.get('color'))
                if key not in cmap or d.get('fill') is not None:
                    continue
                r = d['rect']
                if r.x1 < xs[0] - 1 or r.x0 > xs[-1] + 1 or r.y1 < P['top'] - 2 or r.y0 > P['bottom'] + 2:
                    continue
                if any(L.x0 - 0.5 <= r.x0 and r.x1 <= L.x1 + 0.5 and L.y0 - 0.5 <= r.y0 and r.y1 <= L.y1 + 0.5 for L in legends):
                    continue
                pts = []
                for it in d['items']:
                    if it[0] == 'l':
                        seg = [it[1], it[2]]
                    elif it[0] == 'c':
                        # cubic Bezier (the publisher's PDF pipeline curve-fitted
                        # MATLAB's polylines): sample it densely
                        p0, p1, p2, p3 = it[1], it[2], it[3], it[4]
                        seg = []
                        for k in range(BEZIER_SAMPLES + 1):
                            s = k / BEZIER_SAMPLES
                            a, b, c, e = (1-s)**3, 3*(1-s)**2*s, 3*(1-s)*s**2, s**3
                            seg.append(fitz.Point(a*p0.x + b*p1.x + c*p2.x + e*p3.x,
                                                  a*p0.y + b*p1.y + c*p2.y + e*p3.y))
                    else:
                        continue
                    for p in seg:
                        if not pts or (abs(pts[-1][0] - p.x) > 1e-6 or abs(pts[-1][1] - p.y) > 1e-6):
                            pts.append((p.x, p.y))
                series[cmap[key]].append(pts)
            for name, chunks in series.items():
                rows = [(to_t(x), to_y(y)) for chunk in chunks for (x, y) in chunk]
                fn = os.path.join(outdir, f'{fig}_{pan}_{name}.csv')
                with open(fn, 'w', newline='') as f:
                    w = csv.writer(f)
                    w.writerow(['t', 'y'])
                    for t, y in rows:
                        w.writerow([f'{t:.6f}', f'{y:.8g}'])
                report.append((fig, pan, name, len(rows), len(yt),
                               min(r[1] for r in rows), max(r[1] for r in rows)))
    return report


if __name__ == '__main__':
    pdf = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(__file__), '..', 'data', 'paper_curves')
    for r in extract(pdf, out):
        print('%s %-5s %-6s n=%5d yticks=%2d  y in [%.4g, %.4g]' % r)
    # the same curves on the paper's 0.01 s logging grid, for the Octave code
    import paper_data
    paper_data.write_grid()
    print('wrote data/paper_grid/*.csv')
