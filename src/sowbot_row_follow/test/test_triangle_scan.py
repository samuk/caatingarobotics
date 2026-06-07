import numpy as np, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sowbot_row_follow'))
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

# ---- extended: multi-row anchor selection ----
from triangle_scan import anchor_scan as _ascan, _normalise_mask as _nm

def _converging(l_top,l_bot,r_top,r_bot,width=12):
    mm=_np.zeros((H,W),_np.uint8)
    for y in range(H):
        t=y/(H-1)
        lx=int(round(l_top+(l_bot-l_top)*t)); rx=int(round(r_top+(r_bot-r_top)*t))
        mm[y,max(0,lx-width//2):lx+width//2]=255
        mm[y,max(0,rx-width//2):rx+width//2]=255
    return mm

print("=== T10: argmax flips between rows; leftmost_peak does not ===")
af=[]; lf=[]
for fr in range(8):
    mm=_converging(230,120,290,400)
    if fr%2: mm[:,285:295]=255
    else:    mm[:,225:235]=255
    af.append(int(_d(mm,TSMParams(anchor_select="argmax",amin_frac=0.0,amax_frac=1.0)).anchor_xy[0]))
    lf.append(int(_d(mm,TSMParams(anchor_select="leftmost_peak",peak_rel_thresh=0.5,amin_frac=0.0,amax_frac=1.0)).anchor_xy[0]))
check("argmax DOES flip (reproduces bug)", max(af)-min(af) > 40, f"range={max(af)-min(af)}")
check("leftmost_peak stable", max(lf)-min(lf) < 40, f"range={max(lf)-min(lf)}")

print("=== T11: rightmost_peak mirrors to right row ===")
mm=_converging(230,120,290,400)
ar=_d(mm,TSMParams(anchor_select="rightmost_peak",peak_rel_thresh=0.5,amin_frac=0.0,amax_frac=1.0))
al=_d(mm,TSMParams(anchor_select="leftmost_peak",peak_rel_thresh=0.5,amin_frac=0.0,amax_frac=1.0))
check("rightmost > leftmost", ar.anchor_xy[0] > al.anchor_xy[0], f"r={ar.anchor_xy[0]} l={al.anchor_xy[0]}")

print("=== T12: spatial_prior seeded left stays left through flips ===")
ff=TSMFilter(TSMFilterParams(alpha=0.5,jump_gate_frac=1.0,max_hold=9))
pp=TSMParams(anchor_select="spatial_prior",prior_lambda=0.02,prior_init_frac=0.31,amin_frac=0.0,amax_frac=1.0)
sp=[]
for fr in range(8):
    mm=_converging(230,120,290,400)
    if fr%2: mm[:,285:295]=255
    else:    mm[:,225:235]=255
    rr=ff.update(_d(mm,pp,prev_anchor=ff.prev_anchor),W,H)
    sp.append(int(rr.bottom_x))
left_n=sum(1 for x in sp if x < W//2)
check("spatial_prior stays left-side >=7/8", left_n>=7, f"left={left_n}/8")

print(f"\nMULTIROW: {passed} passed, {failed} failed")

import sys
print(f"\nTOTAL: {passed} passed, {failed} failed")
sys.exit(1 if failed else 0)
