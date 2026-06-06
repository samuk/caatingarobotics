import numpy as np, sys
sys.path.insert(0, '.')
from triangle_scan import detect_central_row, TSMParams

H = W = 512
def blank(): return np.zeros((H, W), np.uint8)

def draw_row(mask, x_top, x_bot, width=12):
    """Draw a thick line from (x_top,0) to (x_bot,H-1)."""
    for y in range(H):
        x = int(round(x_top + (x_bot - x_top) * y / (H - 1)))
        mask[y, max(0,x-width//2):min(W,x+width//2)] = 255
    return mask

passed = failed = 0
def check(name, cond, detail=""):
    global passed, failed
    if cond: passed += 1; print(f"  PASS {name}")
    else:    failed += 1; print(f"  FAIL {name}  {detail}")

print("=== T1: single vertical centre row ===")
m = draw_row(blank(), 256, 256)
r = detect_central_row(m)
check("valid", r.valid)
check("anchor near centre", abs(r.anchor_xy[0]-256) <= 20, f"ax={r.anchor_xy[0]}")
check("base near centre", abs(r.bottom_x-256) <= 20, f"px={r.bottom_x}")
check("slope ~0", abs(r.slope) < 0.05, f"m={r.slope:.4f}")

print("=== T2: tilted row (top centre, bottom right) ===")
m = draw_row(blank(), 256, 400)
r = detect_central_row(m)
check("valid", r.valid)
check("base shifted right", r.bottom_x > 320, f"px={r.bottom_x}")
# node convention x=m*y+b: at y=0 x=ax(256), at y=H-1 x=px(400) => m>0
check("slope positive", r.slope > 0.1, f"m={r.slope:.4f}")
# reconstruct: predicted bottom_x = m*(H-1)+b
recon = r.slope*(H-1)+r.intercept
check("line reconstructs to base", abs(recon - r.bottom_x) < 2, f"recon={recon:.1f} px={r.bottom_x}")

print("=== T3: three rows, central should win line-scan ===")
m = blank()
draw_row(m, 100, 60)    # left row
draw_row(m, 256, 256)   # centre
draw_row(m, 412, 452)   # right row
r = detect_central_row(m)
check("valid", r.valid)
check("picks centre at bottom", abs(r.bottom_x-256) <= 30, f"px={r.bottom_x}")

print("=== T4: empty mask -> invalid ===")
r = detect_central_row(blank())
check("invalid on empty", not r.valid)

print("=== T5: end-of-row, plants only in lower half -> strip shift finds them ===")
m = blank()
for y in range(H):
    if y > H//2:                      # nothing in top strip initially
        x = 256
        m[y, x-6:x+6] = 255
r = detect_central_row(m)
check("valid via strip shift", r.valid, f"valid={r.valid}")
check("anchor near centre col", abs(r.anchor_xy[0]-256) <= 25, f"ax={r.anchor_xy[0]}")

print(f"\nCORE: {passed} passed, {failed} failed")

# ---- extended: morph denoise + temporal filter ----
from triangle_scan import TSMParams, TSMFilterParams, TSMFilter, detect_central_row as _d
import numpy as _np

print("=== T6: morph opening removes speckle, keeps row ===")
m = draw_row(blank(), 256, 256)
rng = _np.random.default_rng(0)
# sprinkle 1-2px salt noise off to the left where there's no row
for _ in range(400):
    yy = rng.integers(0, H); xx = rng.integers(0, 120)
    m[yy, xx] = 255
r_raw = _d(m, TSMParams(morph_kernel=0))
r_open = _d(m, TSMParams(morph_kernel=5))
check("row still found after open", r_open.valid and abs(r_open.bottom_x-256) <= 25, f"px={r_open.bottom_x}")

print("=== T7: filter EMA smooths jitter ===")
f = TSMFilter(TSMFilterParams(alpha=0.4, max_hold=2))
xs_in = [250, 262, 248, 258, 252]
out = []
for xb in xs_in:
    res = type(r_open)((256,0),(xb,H-1),0.0,float(256),float(xb),True)
    out.append(f.update(res, W, H).bottom_x)
# filtered variance < raw variance
check("EMA reduces variance", _np.var(out) < _np.var(xs_in), f"var_out={_np.var(out):.1f} var_in={_np.var(xs_in):.1f}")

print("=== T8: outlier gate rejects a single wild jump ===")
f = TSMFilter(TSMFilterParams(alpha=0.5, jump_gate_frac=0.3, max_hold=2))
for xb in [256, 256, 256]:
    f.update(type(r_open)((256,0),(xb,H-1),0.0,256.0,float(xb),True), W, H)
spike = f.update(type(r_open)((460,0),(460,H-1),0.0,460.0,460.0,True), W, H)  # jump 204px > 0.3*512=153
check("spike held near prior, not jumped", abs(spike.bottom_x-256) < 50, f"px={spike.bottom_x}")

print("=== T9: bounded hold then invalid (heartbeat must die) ===")
f = TSMFilter(TSMFilterParams(max_hold=2))
for xb in [256, 256]:
    f.update(type(r_open)((256,0),(xb,H-1),0.0,256.0,float(xb),True), W, H)
inv = type(r_open)((0,0),(0,0),0.0,0.0,0.0,False)
h1 = f.update(inv, W, H); h2 = f.update(inv, W, H); h3 = f.update(inv, W, H)
check("holds frame 1", h1.valid)
check("holds frame 2", h2.valid)
check("invalid after max_hold exceeded", not h3.valid, f"valid={h3.valid}")

print(f"\nEXTENDED: {passed} passed, {failed} failed")
import sys as _s; _s.exit(1 if failed else 0)
