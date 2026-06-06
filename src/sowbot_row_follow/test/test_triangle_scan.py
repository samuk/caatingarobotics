import numpy as np, sys
import os
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

print(f"\n{passed} passed, {failed} failed")
sys.exit(1 if failed else 0)
