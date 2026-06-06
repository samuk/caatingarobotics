# ===========================================================================
# Triangle Scan Method (TSM) — central crop-row detection
# ===========================================================================
# Clean-room reimplementation written from the published method description,
# NOT derived from the reference implementation at
# github.com/rajithadesilva/TSM (which is CC BY-NC-ND 4.0). Only the algorithm
# (uncopyrightable) is reproduced here; no source code from that repository was
# used. This file is therefore original work and carries the package licence
# (MIT), keeping it clear of the upstream NonCommercial/NoDerivatives terms.
#
# Method reference:
#   de Silva, Cielniak, Gao (2024). "Vision based crop row navigation under
#   varying field conditions in arable fields." Computers and Electronics in
#   Agriculture, 217, 108581.  arXiv:2209.14003
#
# Two-step process on a binary crop-row mask:
#   1. Anchor Scan: find apex A on the top edge (column with max vertical
#      pixel-sum in a top ROI strip of height s*H, over columns [Amin, Amax]).
#      If the strip is empty (below threshold), shift it down by h up to twice
#      (end-of-row handling).
#   2. Line Scan: sweep P along the bottom edge from B to C; P_r is the P that
#      maximises the pixel-sum along the line A->P. The line A->P_r is the
#      detected central crop row.
# ===========================================================================

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class TSMParams:
    """Tunables mirroring the paper's parameters (defaults for 512x512)."""
    s: float = 0.2          # anchor ROI height fraction (h = s*H)
    amin_frac: float = 0.2  # anchor column search start, fraction of W
    amax_frac: float = 0.7  # anchor column search end, fraction of W
    b_frac: float = 0.0     # Begin point x on bottom edge, fraction of W
    c_frac: float = 1.0     # Cease point x on bottom edge, fraction of W
    anchor_min_sum: float = 1.0   # min column-sum (in mask units) to accept A
    max_strip_shifts: int = 2     # extra downward ROI shifts on empty strip
    line_samples: int = 0   # P candidates along BC; 0 => every integer column


@dataclass
class TSMResult:
    """Detected central row. Coordinates in image pixels, origin top-left."""
    anchor_xy: tuple        # A = (x, top_y)
    base_xy: tuple          # P_r = (x, bottom_y)
    slope: float            # m in x = m*y + b  (node convention)
    intercept: float        # b in x = m*y + b
    bottom_x: float         # x where row meets bottom edge (== P_r.x)
    valid: bool


def _normalise_mask(mask: np.ndarray) -> np.ndarray:
    """Accept 0/255 uint8 or boolean; return float32 0/1, single channel."""
    if mask.ndim == 3:
        mask = mask[:, :, 0]
    m = (mask > 0).astype(np.float32)
    return m


def anchor_scan(mask01: np.ndarray, p: TSMParams) -> Optional[int]:
    """Return anchor column x (apex A) on the top edge, or None.

    Sums each column over a top ROI strip of height h = s*H, restricted to
    columns [Amin, Amax]. A is the argmax column. If the best column-sum is
    below anchor_min_sum, shift the strip down by h and retry (up to
    max_strip_shifts times) to handle end-of-row where plants recede.
    """
    h_img, w_img = mask01.shape
    h = max(1, int(round(p.s * h_img)))
    amin = int(round(p.amin_frac * w_img))
    amax = int(round(p.amax_frac * w_img))
    amin = max(0, min(amin, w_img - 1))
    amax = max(amin + 1, min(amax, w_img))

    for shift in range(p.max_strip_shifts + 1):
        top = shift * h
        bot = top + h
        if top >= h_img:
            break
        strip = mask01[top:min(bot, h_img), amin:amax]
        if strip.size == 0:
            break
        col_sums = strip.sum(axis=0)          # length (amax-amin)
        best_local = int(np.argmax(col_sums))
        if col_sums[best_local] >= p.anchor_min_sum:
            return amin + best_local
    return None


def _line_pixel_sum(mask01: np.ndarray, ax: int, ay: int,
                    px: int, py: int) -> float:
    """Sum of mask values along the segment A=(ax,ay) -> P=(px,py).

    Samples one point per row between ay and py (rows are the dense axis for a
    near-vertical row line), bilinear-free nearest-column sampling. Equivalent
    to integrating I along AP as in the paper's Eq. 2.
    """
    h_img, w_img = mask01.shape
    if py == ay:
        return 0.0
    ys = np.arange(min(ay, py), max(ay, py) + 1)
    # x as a function of y along the segment
    xs = ax + (px - ax) * (ys - ay) / float(py - ay)
    xi = np.clip(np.round(xs).astype(int), 0, w_img - 1)
    yi = np.clip(ys, 0, h_img - 1)
    return float(mask01[yi, xi].sum())


def line_scan(mask01: np.ndarray, anchor_x: int, p: TSMParams) -> int:
    """Return P_r.x: bottom-edge column maximising pixel-sum along A->P."""
    h_img, w_img = mask01.shape
    ay = 0                       # A lies on the top edge
    py = h_img - 1               # B, C, P_r lie on the bottom edge
    b_x = int(round(p.b_frac * w_img))
    c_x = int(round(p.c_frac * w_img))
    b_x = max(0, min(b_x, w_img - 1))
    c_x = max(0, min(c_x, w_img - 1))
    lo, hi = min(b_x, c_x), max(b_x, c_x)

    if p.line_samples and p.line_samples > 1:
        cands = np.linspace(lo, hi, p.line_samples).round().astype(int)
    else:
        cands = np.arange(lo, hi + 1)

    best_x = lo
    best_sum = -1.0
    for px in cands:
        s = _line_pixel_sum(mask01, anchor_x, ay, int(px), py)
        if s > best_sum:
            best_sum = s
            best_x = int(px)
    return best_x


def detect_central_row(mask: np.ndarray,
                       params: Optional[TSMParams] = None) -> TSMResult:
    """Run TSM on a crop-row mask. Returns the central row in node convention.

    Node convention: x = m*y + b  (x as a function of y), matching the existing
    crop_row_node fit_line / detect_crop_rows output so downstream visual
    servoing is unchanged. Here A=(ax, 0) and P_r=(px, H-1):
        m = (px - ax) / (H - 1)
        b = ax                       (since x at y=0 is ax)
    bottom_x = px.
    """
    p = params or TSMParams()
    mask01 = _normalise_mask(mask)
    h_img, _ = mask01.shape

    ax = anchor_scan(mask01, p)
    if ax is None:
        return TSMResult((0, 0), (0, 0), 0.0, 0.0, 0.0, False)

    px = line_scan(mask01, ax, p)
    denom = float(h_img - 1) if h_img > 1 else 1.0
    m = (px - ax) / denom
    b = float(ax)
    return TSMResult(
        anchor_xy=(ax, 0),
        base_xy=(px, h_img - 1),
        slope=m,
        intercept=b,
        bottom_x=float(px),
        valid=True,
    )
