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
    # Mask denoising (engineering addition, NOT from the paper). The paper
    # assumes a clean U-Net mask; an ExG+Otsu mask carries shadow/weed speckle
    # that can outvote the true row in the column-sum. A morphological opening
    # removes small fragments while preserving the solid vertical row mass —
    # the right denoise for a pixel-sum method (contour-free, unlike scanwin's
    # min_contour_area). Set 0 to disable and get literal paper behaviour.
    morph_kernel: int = 5   # opening kernel (px); 0 = off
    # ── Anchor selection mode ────────────────────────────────────────────────
    # When two crop rows are visible the column-sum has two near-equal peaks and
    # a plain argmax flips between them frame-to-frame. anchor_select biases the
    # *detection* (not just the output filter) toward one row. Modes:
    #   "argmax"        — tallest column-sum wins (paper default; no bias).
    #   "leftmost_peak" — leftmost peak whose sum >= peak_rel_thresh * global max
    #                     (prefer left row even when slightly weaker).
    #   "rightmost_peak"— mirror of the above (prefer right row).
    #   "spatial_prior" — argmax of (col_sum - prior_lambda*|col - prev_anchor|):
    #                     sticky tracking that resists switching rows once locked.
    #                     Seeded by prior_init_frac on the first frame.
    #   "weighted"      — argmax of (col_sum * side_weight), where side_weight
    #                     ramps linearly from 1+weight_k on the preferred side to
    #                     1 on the other. A much stronger opposite row can still
    #                     win (soft bias, unlike a hard search-window clamp).
    # NOTE: a hard left/right restriction needs no mode — just narrow
    # amin_frac/amax_frac so the unwanted row falls outside the search range.
    anchor_select: str = "argmax"
    peak_rel_thresh: float = 0.6   # leftmost/rightmost_peak: frac of global max
    prior_lambda: float = 0.01     # spatial_prior: px-distance penalty weight
    prior_init_frac: float = 0.3   # spatial_prior: first-frame seed (frac of W)
    weight_k: float = 0.5          # weighted: bias strength on preferred side
    weight_side: str = "left"      # weighted: "left" or "right" preferred


@dataclass
class TSMResult:
    """Detected central row. Coordinates in image pixels, origin top-left."""
    anchor_xy: tuple        # A = (x, top_y)
    base_xy: tuple          # P_r = (x, bottom_y)
    slope: float            # m in x = m*y + b  (node convention)
    intercept: float        # b in x = m*y + b
    bottom_x: float         # x where row meets bottom edge (== P_r.x)
    valid: bool


@dataclass
class TSMFilterParams:
    """Temporal filter tunables (engineering addition, NOT the paper's filter).

    The reference implementation's complementary filter equations are not
    reproduced here (its media doc was not used); this is an independent EMA +
    outlier-gate + bounded-hold design serving the same smoothing purpose on a
    video stream. Operates on the two output parameters (anchor_x, base_x).
    """
    enable: bool = True
    alpha: float = 0.4          # EMA weight on the NEW frame (0..1); lower = smoother
    # Outlier gate: reject a frame whose anchor_x or base_x jumps more than this
    # fraction of image width from the current filtered estimate. The rejected
    # frame counts toward the hold limit below.
    jump_gate_frac: float = 0.35
    # Bounded hold: smooth across brief dropouts, but after this many CONSECUTIVE
    # invalid/rejected frames, stop holding and report invalid so the node's
    # heartbeat goes silent as designed (do NOT mask sustained detection loss —
    # the Limbic action server relies on heartbeat loss to disable Neo safely).
    max_hold: int = 2



def _normalise_mask(mask: np.ndarray, morph_kernel: int = 0) -> np.ndarray:
    """Accept 0/255 uint8 or boolean; return float32 0/1, single channel.

    If morph_kernel > 0, apply a morphological opening (erode then dilate) to
    strip small speckle / thin weed fragments before scanning. Falls back
    silently if OpenCV is unavailable (e.g. unit tests on a bare interpreter).
    """
    if mask.ndim == 3:
        mask = mask[:, :, 0]
    binary = (mask > 0).astype(np.uint8)
    if morph_kernel and morph_kernel > 1:
        try:
            import cv2
            k = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (morph_kernel, morph_kernel))
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, k)
        except Exception:
            pass  # no OpenCV: proceed unfiltered rather than fail
    return binary.astype(np.float32)


def _find_peaks(col_sums: np.ndarray, rel_thresh: float) -> list:
    """Local maxima with value >= rel_thresh * global max. Returns indices."""
    n = len(col_sums)
    if n == 0:
        return []
    gmax = float(col_sums.max())
    if gmax <= 0:
        return []
    floor = rel_thresh * gmax
    peaks = []
    for i in range(n):
        v = col_sums[i]
        if v < floor:
            continue
        left_ok = (i == 0) or (col_sums[i - 1] <= v)
        right_ok = (i == n - 1) or (col_sums[i + 1] <= v)
        if left_ok and right_ok:
            peaks.append(i)
    # Fallback: if the plateau logic found nothing, take the global argmax.
    return peaks or [int(np.argmax(col_sums))]


def _select_anchor_col(col_sums: np.ndarray, p: TSMParams,
                       amin: int, prev_anchor: Optional[int]) -> int:
    """Pick the local anchor column index within col_sums per anchor_select.

    col_sums is indexed from amin; returned index is also local (add amin for
    image coords). prev_anchor is in IMAGE coords (or None on first frame).
    """
    mode = p.anchor_select
    n = len(col_sums)

    if mode == "leftmost_peak":
        return min(_find_peaks(col_sums, p.peak_rel_thresh))
    if mode == "rightmost_peak":
        return max(_find_peaks(col_sums, p.peak_rel_thresh))

    if mode == "spatial_prior":
        idx = np.arange(n)
        if prev_anchor is None:
            prev_local = p.prior_init_frac * n   # seed (e.g. left third)
        else:
            prev_local = prev_anchor - amin
        score = col_sums - p.prior_lambda * np.abs(idx - prev_local)
        return int(np.argmax(score))

    if mode == "weighted":
        idx = np.arange(n)
        if n > 1:
            ramp = idx / float(n - 1)            # 0 at left .. 1 at right
        else:
            ramp = np.zeros(1)
        if p.weight_side == "right":
            w = 1.0 + p.weight_k * ramp
        else:  # left (default)
            w = 1.0 + p.weight_k * (1.0 - ramp)
        return int(np.argmax(col_sums * w))

    # "argmax" (default) and any unknown mode
    return int(np.argmax(col_sums))


def anchor_scan(mask01: np.ndarray, p: TSMParams,
                prev_anchor: Optional[int] = None) -> Optional[int]:
    """Return anchor column x (apex A) on the top edge, or None.

    Sums each column over a top ROI strip of height h = s*H, restricted to
    columns [Amin, Amax]. The winning column is chosen per p.anchor_select
    (see TSMParams). If the best column-sum is below anchor_min_sum, shift the
    strip down by h and retry (up to max_strip_shifts times) for end-of-row.
    prev_anchor (image coords) is only used by the "spatial_prior" mode.
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
        best_local = _select_anchor_col(col_sums, p, amin, prev_anchor)
        # Accept only if the CHOSEN column clears the strength threshold (use
        # the chosen column, not the global max, so a biased pick that lands on
        # a too-weak column correctly triggers the strip shift / end-of-row).
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
                       params: Optional[TSMParams] = None,
                       prev_anchor: Optional[int] = None) -> TSMResult:
    """Run TSM on a crop-row mask. Returns the central row in node convention.

    Node convention: x = m*y + b  (x as a function of y), matching the existing
    crop_row_node fit_line / detect_crop_rows output so downstream visual
    servoing is unchanged. Here A=(ax, 0) and P_r=(px, H-1):
        m = (px - ax) / (H - 1)
        b = ax                       (since x at y=0 is ax)
    bottom_x = px.

    prev_anchor (image coords, from the previous accepted frame) is consumed
    only by the "spatial_prior" anchor_select mode for sticky row tracking.
    """
    p = params or TSMParams()
    mask01 = _normalise_mask(mask, p.morph_kernel)
    h_img, _ = mask01.shape

    ax = anchor_scan(mask01, p, prev_anchor)
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


class TSMFilter:
    """Stateful temporal filter over successive TSMResults on a video stream.

    Engineering addition (NOT the paper's complementary filter). Applies, in
    order, to each new detection:
      1. Outlier gate  — if anchor_x or base_x jumps more than jump_gate_frac*W
                          from the current estimate, treat the frame as a miss.
      2. EMA           — blend an accepted frame into the running estimate.
      3. Bounded hold  — on a miss (invalid OR gated), keep returning the last
                          estimate for up to max_hold consecutive misses; beyond
                          that, return invalid so the caller's heartbeat dies.

    The bounded hold is deliberate: it smooths 1–2 frame dropouts without
    masking sustained detection loss that the safety layer must see.
    """

    def __init__(self, fp: Optional["TSMFilterParams"] = None):
        self.fp = fp or TSMFilterParams()
        self._ax: Optional[float] = None    # filtered anchor x
        self._px: Optional[float] = None    # filtered base x
        self._misses = 0

    def reset(self):
        self._ax = None
        self._px = None
        self._misses = 0

    @property
    def prev_anchor(self) -> Optional[int]:
        """Current filtered anchor x in image coords, or None before lock.

        Feed back into detect_central_row(prev_anchor=...) for the
        "spatial_prior" anchor_select mode so sticky tracking follows the
        smoothed estimate rather than the raw per-frame anchor.
        """
        return None if self._ax is None else int(round(self._ax))

    def _emit(self, h_img: int) -> TSMResult:
        denom = float(h_img - 1) if h_img > 1 else 1.0
        m = (self._px - self._ax) / denom
        return TSMResult(
            anchor_xy=(int(round(self._ax)), 0),
            base_xy=(int(round(self._px)), h_img - 1),
            slope=m,
            intercept=float(self._ax),
            bottom_x=float(self._px),
            valid=True,
        )

    def update(self, result: TSMResult, img_w: int, img_h: int) -> TSMResult:
        if not self.fp.enable:
            return result

        # No estimate yet: accept the first valid detection verbatim.
        if self._ax is None or self._px is None:
            if result.valid:
                self._ax = float(result.anchor_xy[0])
                self._px = result.bottom_x
                self._misses = 0
                return self._emit(img_h)
            return result  # still invalid, nothing to hold

        miss = not result.valid
        if not miss:
            gate = self.fp.jump_gate_frac * img_w
            new_ax = float(result.anchor_xy[0])
            new_px = result.bottom_x
            if abs(new_ax - self._ax) > gate or abs(new_px - self._px) > gate:
                miss = True   # outlier: reject this frame

        if miss:
            self._misses += 1
            if self._misses > self.fp.max_hold:
                self.reset()
                return TSMResult((0, 0), (0, 0), 0.0, 0.0, 0.0, False)
            return self._emit(img_h)   # bounded hold

        # Accepted frame: EMA blend.
        a = self.fp.alpha
        self._ax = a * float(result.anchor_xy[0]) + (1.0 - a) * self._ax
        self._px = a * result.bottom_x + (1.0 - a) * self._px
        self._misses = 0
        return self._emit(img_h)
