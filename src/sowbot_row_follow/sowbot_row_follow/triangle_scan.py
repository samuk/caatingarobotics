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
    # ── Near-vertical prior ──────────────────────────────────────────────────
    # Reject candidate lines more than max_angle_deg off image-vertical by
    # restricting the line-scan bottom point to a cone around the apex:
    #   |P.x - A.x| <= (H-1) * tan(max_angle_deg).
    # The line CANNOT be chosen steeper than this (constraint on the search, not
    # a veto on the result), so a sparse/noisy frame can't tilt the fit to a
    # diagonal. Set 0 (or >=90) to disable and allow any angle. Raise it if the
    # row legitimately appears tilted (steep camera angle, headland re-acquire).
    max_angle_deg: float = 15.0
    # ── Fit mode ──────────────────────────────────────────────────────────────
    # "tsm"    — paper-faithful anchor_scan + line_scan (single free endpoint
    #            P, apex A fixed by anchor_scan). Default; unchanged behaviour.
    # "ransac" — engineering addition. anchor_scan still runs first (so band
    #            search / row-swap / multi-row bookkeeping upstream is
    #            unaffected); the LINE itself is then a RANSAC fit directly
    #            on raw mask pixel coordinates within [amin_frac, amax_frac],
    #            constrained to the same near-vertical max_angle_deg cone.
    #            Crucially this scores candidates by inlier pixel count within
    #            a narrow x-corridor of the candidate line (ransac_corridor_px)
    #            — NOT by total mass in a wide band — so clutter that is
    #            spatially separated from the candidate line contributes
    #            nothing regardless of how much mass it has. This preserves
    #            the spatial-locality property that makes anchor_scan/
    #            line_scan robust, while fitting through many pixels instead
    #            of one fixed apex + one free endpoint.
    fit_mode: str = "tsm"
    ransac_iters: int = 60            # number of random 2-pixel candidate lines to try
    ransac_corridor_px: float = 6.0   # inlier band half-width (px) around a candidate line
    ransac_min_inliers: int = 30      # min inlier pixel count to trust the RANSAC fit
    ransac_max_points: int = 2000     # subsample mask pixels above this count (speed)
    # ── Line-scan minimum support ────────────────────────────────────────────
    # anchor_scan only checks the TOP strip. At end-of-row the top strip can
    # still clear anchor_min_sum (residual soil/horizon texture) while the
    # BOTTOM of the mask has no row structure at all. Without this check
    # line_scan/ransac_line_fit's "best" candidate degenerates to whichever
    # column happens to be tried first (zero real pixel support) and
    # _detect_from_mask01 would still report valid=True — a phantom row
    # pinned to the edge of the search range. line_min_sum is the minimum
    # pixel-sum (same units as anchor_min_sum) the WINNING line_scan
    # candidate must clear to be trusted; below this, the frame is reported
    # invalid instead of silently degrading.
    line_min_sum: float = 1.0


@dataclass
class TSMResult:
    """Detected central row. Coordinates in image pixels, origin top-left."""
    anchor_xy: tuple        # A = (x, top_y)
    base_xy: tuple          # P_r = (x, bottom_y)
    slope: float            # m in x = m*y + b  (node convention)
    intercept: float        # b in x = m*y + b
    bottom_x: float         # x where row meets bottom edge (== P_r.x)
    valid: bool
    # Raw line_scan pixel-sum for the winning candidate (0.0 where not
    # computed, e.g. no-anchor / ransac / filtered-estimate results).
    # Surfaced so tsm_line_min_sum can be calibrated against real numbers
    # instead of guessed — see crop_row_node.py's throttled log of this.
    line_sum: float = 0.0


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
    if n == 0:
        return 0  # empty strip; caller's anchor_min_sum check rejects it

    if mode == "leftmost_peak":
        peaks = _find_peaks(col_sums, p.peak_rel_thresh)
        return min(peaks) if peaks else int(np.argmax(col_sums))
    if mode == "rightmost_peak":
        peaks = _find_peaks(col_sums, p.peak_rel_thresh)
        return max(peaks) if peaks else int(np.argmax(col_sums))

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
                prev_anchor: Optional[int] = None) -> Optional[tuple]:
    """Return (ax, ay): apex A's column and its real strip row, or None.

    Sums each column over a top ROI strip of height h = s*H, restricted to
    columns [Amin, Amax]. The winning column is chosen per p.anchor_select
    (see TSMParams). If the best column-sum is below anchor_min_sum, shift the
    strip down by h and retry (up to max_strip_shifts times) for end-of-row.
    prev_anchor (image coords) is only used by the "spatial_prior" mode.

    ay is the top row of whichever shifted strip actually produced the
    accepted anchor (0 on the first, unshifted strip; shift*h on later ones).
    Callers MUST use this real ay — e.g. as the origin for line_scan's
    A->P sweep — rather than assuming the anchor always sits on row 0. An
    anchor accepted from a shifted strip legitimately lives partway down the
    frame; treating it as row 0 fabricates a full-height line through empty
    space above the strip and widens line_scan's search cone (proportional
    to the A->P vertical span) well beyond what the real detection supports.
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
        # Empty / below-threshold strip: don't run selection on dead data —
        # shift the ROI down (end-of-row handling) or fall through to None.
        if col_sums.size == 0 or float(col_sums.max()) < p.anchor_min_sum:
            continue
        best_local = _select_anchor_col(col_sums, p, amin, prev_anchor)
        # Accept only if the CHOSEN column clears the strength threshold (use
        # the chosen column, not the global max, so a biased pick that lands on
        # a too-weak column correctly triggers the strip shift / end-of-row).
        if col_sums[best_local] >= p.anchor_min_sum:
            return amin + best_local, top
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


def line_scan(mask01: np.ndarray, anchor_x: int, p: TSMParams,
             anchor_y: int = 0) -> tuple:
    """Return (P_r.x, best_sum): bottom-edge column maximising pixel-sum
    along A->P, and the pixel-sum that column achieved.

    anchor_y is A's REAL row (from anchor_scan's returned ay), not always 0.
    An anchor accepted from a shifted strip (end-of-row handling) sits
    partway down the frame; sweeping from row 0 in that case would sum mask
    values over empty space above the strip and inflate the angle cone
    (proportional to (H-1) instead of the true (H-1-anchor_y) span), letting
    noise far from the real detection outvote the true row.

    The candidate range is the configured B-C span intersected with a
    near-vertical cone around the apex: |P.x - anchor_x| <= (py-ay)*tan(angle),
    so the chosen line cannot exceed p.max_angle_deg off image-vertical.

    best_sum is returned (rather than swallowed) so callers can tell a real
    detection apart from a degenerate "winner" chosen among all-zero
    candidates (e.g. end-of-row, where the top strip still has enough mass
    to pass anchor_scan but nothing exists along any bottom-edge candidate).
    """
    import math
    h_img, w_img = mask01.shape
    ay = anchor_y                # A lies on its real strip row
    py = h_img - 1               # B, C, P_r lie on the bottom edge
    b_x = int(round(p.b_frac * w_img))
    c_x = int(round(p.c_frac * w_img))
    b_x = max(0, min(b_x, w_img - 1))
    c_x = max(0, min(c_x, w_img - 1))
    lo, hi = min(b_x, c_x), max(b_x, c_x)

    # Near-vertical prior: clamp the bottom-point range to the angle cone.
    # Proportional to the REAL vertical span (py-ay), not the full frame
    # height, so an anchor found deep in the frame gets a proportionally
    # tighter search cone instead of a full-height one.
    if p.max_angle_deg and 0 < p.max_angle_deg < 90:
        dx = max(0, py - ay) * math.tan(math.radians(p.max_angle_deg))
        lo = max(lo, int(math.floor(anchor_x - dx)))
        hi = min(hi, int(math.ceil(anchor_x + dx)))
        if lo > hi:               # cone falls entirely outside B-C: best effort
            lo = hi = int(max(0, min(anchor_x, w_img - 1)))

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
    return best_x, max(best_sum, 0.0)


def ransac_line_fit(mask01: np.ndarray, p: TSMParams,
                    anchor_x: int, amin: int, amax: int) -> Optional[tuple]:
    """RANSAC fit of x = m*y + b directly on raw mask pixel coordinates.

    Engineering addition (NOT from the paper). Unlike line_scan (one fixed
    apex + one free endpoint argmax) or naive band-centroid averaging (which
    collapses each row-band's full width to one point, so a stray pixel
    anywhere in a wide band can drag that band's centroid), this fits
    through every individual mask pixel in [amin, amax] but SCORES each
    candidate line only by pixels that fall within ransac_corridor_px of it.
    A clutter clump that sits outside that corridor contributes nothing to
    the winning candidate's score no matter how much mass it has — the same
    spatial-locality property that makes the paper's method robust, applied
    here to a real multi-point fit instead of a single free endpoint.

    Candidates are constrained to the near-vertical cone around anchor_x
    (same max_angle_deg as line_scan), and EVERY candidate line must be seeded
    by one point drawn from the apex strip (the same top ROI anchor_scan used)
    and one point from elsewhere in the mask. This is the key difference from
    a naive "two random pixels" RANSAC: checking only the extrapolated y=0
    intercept of an arbitrary pixel pair is not a real constraint — two
    points from a clump far from the apex can still extrapolate to land near
    anchor_x by chance, then win on corridor mass alone. Anchoring one seed
    point to genuine apex-region evidence closes that loophole, mirroring
    line_scan's fixed-apex/free-endpoint structure while still fitting
    through every inlier pixel rather than one fixed point and one free one.

    Returns (m, b, n_inliers) in node convention (x = m*y + b), or None if
    the best candidate has fewer than p.ransac_min_inliers support (caller
    should fall back to the "tsm" line_scan in that case).
    """
    import math
    h_img, w_img = mask01.shape
    amin = max(0, min(amin, w_img - 1))
    amax = max(amin + 1, min(amax, w_img))

    ys_all, xs_all = np.nonzero(mask01[:, amin:amax])
    if ys_all.size == 0:
        return None
    xs_all = xs_all + amin

    # Subsample for speed on dense masks; doesn't change which structures are
    # present, only how many points represent them.
    if xs_all.size > p.ransac_max_points:
        rng_sub = np.random.default_rng(1)
        idx = rng_sub.choice(xs_all.size, size=p.ransac_max_points, replace=False)
        ys_all = ys_all[idx]
        xs_all = xs_all[idx]

    ys_f = ys_all.astype(np.float64)
    xs_f = xs_all.astype(np.float64)
    n_pts = xs_f.size
    denom_h = float(h_img - 1) if h_img > 1 else 1.0

    # Apex-strip seed pool: same top ROI height anchor_scan used (h = s*H),
    # further restricted to points near anchor_x (within the same cone used
    # below). This is necessary, not just belt-and-braces: if an off-row
    # clump happens to also intersect the apex strip (e.g. a wide weed patch
    # near the top of frame), pairing two points drawn from inside that
    # clump is itself a valid "apex-seeded" candidate and can still win on
    # corridor mass. anchor_scan already identified the correct row's apex
    # column (anchor_x) via its own column-sum logic; restricting seed pairs
    # to genuinely originate near that column — not just "somewhere in the
    # top strip" — is what actually closes the loophole.
    apex_h = max(1, int(round(p.s * h_img)))
    if p.max_angle_deg and 0 < p.max_angle_deg < 90:
        apex_cone_px = apex_h * math.tan(math.radians(p.max_angle_deg))
    else:
        apex_cone_px = float(w_img)  # disabled: no angular restriction
    near_anchor = np.abs(xs_f - anchor_x) <= apex_cone_px
    apex_pool = np.nonzero((ys_f < apex_h) & near_anchor)[0]
    if apex_pool.size == 0:
        return None  # no apex-region support at all; let caller fall back

    # Near-vertical cone around the apex, same convention as line_scan.
    max_dx = denom_h
    if p.max_angle_deg and 0 < p.max_angle_deg < 90:
        max_dx = denom_h * math.tan(math.radians(p.max_angle_deg))

    rng = np.random.default_rng(0)  # deterministic: same mask -> same fit
    best_m, best_b, best_n = 0.0, float(anchor_x), -1

    n_iters = max(1, int(p.ransac_iters))
    for _ in range(n_iters):
        i = int(rng.choice(apex_pool))
        j = int(rng.integers(0, n_pts))
        if ys_f[i] == ys_f[j]:
            continue
        cm = (xs_f[j] - xs_f[i]) / (ys_f[j] - ys_f[i])
        cb = xs_f[i] - cm * ys_f[i]
        # Belt-and-braces: still bound the apex intercept to the cone (cheap
        # to check, catches the rare case where the apex-pool point itself
        # sits at the edge of the search range away from anchor_x).
        if abs(cb - anchor_x) > max_dx + p.ransac_corridor_px:
            continue
        # Corridor inlier count: perpendicular-ish distance approximated as
        # horizontal offset at each pixel's own y (cheap, and exact for the
        # near-vertical lines this method is restricted to).
        pred_x = cm * ys_f + cb
        n_in = int((np.abs(xs_f - pred_x) <= p.ransac_corridor_px).sum())
        if n_in > best_n:
            best_n = n_in
            best_m, best_b = float(cm), float(cb)

    if best_n < p.ransac_min_inliers:
        return None

    # Polish: refit by ordinary least squares over the winning inlier set
    # only, so the final line isn't just one of the two seed pixels' exact
    # line but the best fit through all pixels RANSAC identified as inliers.
    pred_x = best_m * ys_f + best_b
    inliers = np.abs(xs_f - pred_x) <= p.ransac_corridor_px
    yin = ys_f[inliers]
    xin = xs_f[inliers]
    if yin.size >= 2 and np.ptp(yin) > 1e-6:
        m_fit, b_fit = np.polyfit(yin, xin, 1)
        return (float(m_fit), float(b_fit), int(inliers.sum()))
    return (best_m, best_b, int(inliers.sum()))


def _detect_from_mask01(mask01: np.ndarray,
                        p: TSMParams,
                        prev_anchor: Optional[int] = None) -> TSMResult:
    """Run TSM on an already-normalised float32 mask (morph_kernel is ignored).

    Used by detect_central_row and detect_rows_multi to avoid redundant mask
    normalisation when running multiple bands on the same frame.

    anchor_scan always runs first regardless of p.fit_mode: it establishes
    whether the band has a valid row at all, and the anchor_x used for
    row-swap debounce / sticky tracking / multi-row band bookkeeping
    upstream. fit_mode only changes how the LINE through that band is
    computed once anchor_scan has confirmed a row is present:
      "tsm" (default) — line_scan's single free-endpoint argmax (unchanged).
      "ransac"         — ransac_line_fit over raw mask pixels in the same
                          band, falling back to the "tsm" line if support is
                          too thin (sparse mask, end-of-row, etc.).
    """
    h_img, w_img = mask01.shape
    anchor = anchor_scan(mask01, p, prev_anchor)
    if anchor is None:
        return TSMResult((0, 0), (0, 0), 0.0, 0.0, 0.0, False)
    ax, ay = anchor

    if p.fit_mode == "ransac":
        amin = int(round(p.amin_frac * w_img))
        amax = int(round(p.amax_frac * w_img))
        fit = ransac_line_fit(mask01, p, ax, amin, amax)
        if fit is not None:
            m, b, _n_in = fit
            denom = float(h_img - 1) if h_img > 1 else 1.0
            px = b + m * denom
            return TSMResult(
                anchor_xy=(int(round(b)), 0),
                base_xy=(int(round(px)), h_img - 1),
                slope=m,
                intercept=b,
                bottom_x=float(px),
                valid=True,
            )
        # Too few inliers (sparse mask, end-of-row, etc.) — fall through to
        # the tsm line_scan below. That fallback is now also support-gated
        # (see line_min_sum below), so this no longer silently produces a
        # phantom "valid" detection either.

    px, best_sum = line_scan(mask01, ax, p, anchor_y=ay)
    if best_sum < p.line_min_sum:
        # anchor_scan found something in the top strip, but no candidate
        # bottom-edge line has real pixel support — e.g. end-of-row, where
        # residual top-strip texture clears anchor_min_sum but the row
        # itself has run out. Report invalid instead of returning the
        # degenerate zero-support "winner" (which was previously always the
        # leftmost/edge candidate, i.e. a line pinned to the frame corner).
        return TSMResult((0, 0), (0, 0), 0.0, 0.0, 0.0, False,
                          line_sum=best_sum)

    denom = float(h_img - 1 - ay) if h_img - 1 > ay else 1.0
    m = (px - ax) / denom
    # Node convention keeps b as the intercept at y=0 (x = m*y + b) for
    # downstream compatibility, extrapolated from the real anchor row rather
    # than assumed to sit there — see anchor_scan's docstring.
    b = float(ax) - m * ay
    return TSMResult(
        anchor_xy=(ax, ay),
        base_xy=(px, h_img - 1),
        slope=m,
        intercept=b,
        bottom_x=float(px),
        valid=True,
        line_sum=best_sum,
    )



def detect_central_row(mask: np.ndarray,
                       params: Optional[TSMParams] = None,
                       prev_anchor: Optional[int] = None) -> TSMResult:
    """Run TSM on a crop-row mask. Returns the detected row in node convention.

    Node convention: x = m*y + b  (x as a function of y), matching the existing
    crop_row_node fit_line / detect_crop_rows output so downstream visual
    servoing is unchanged. A=(ax, ay) and P_r=(px, H-1), where ay is the
    anchor's REAL strip row (0 on an unshifted strip, but > 0 when
    anchor_scan accepted a shifted strip during end-of-row handling):
        m = (px - ax) / (H - 1 - ay)
        b = ax - m*ay                (x extrapolated to y=0, node convention)
    bottom_x = px. anchor_xy is (ax, ay) — do not assume ay == 0.

    prev_anchor (image coords, from the previous accepted frame) is consumed
    only by the "spatial_prior" anchor_select mode for sticky row tracking.
    """
    p = params or TSMParams()
    mask01 = _normalise_mask(mask, p.morph_kernel)
    return _detect_from_mask01(mask01, p, prev_anchor)


def detect_rows_multi(
    mask: np.ndarray,
    params: Optional[TSMParams] = None,
    n_rows: int = 1,
    prev_anchors: Optional[list] = None,
) -> list:
    """Run TSM N times with horizontally-partitioned anchor search bands.

    Partitions [params.amin_frac, params.amax_frac] into n_rows equal-width
    horizontal bands and runs an independent TSM scan in each band. Returns a
    list of TSMResult in left-to-right band order; invalid detections are
    included in-place (caller handles per-slot temporal filtering).

    The mask is normalised and denoised exactly once (using params.morph_kernel)
    and reused across all band scans — so this costs little more than a single
    detect_central_row call.

    When n_rows == 1 this is exactly equivalent to detect_central_row.

    Note on search range: for multi-row use, widen tsm_amin_frac / tsm_amax_frac
    to span all expected rows (e.g. 0.05 / 0.95) since this function partitions
    that range into equal bands.
    """
    from dataclasses import replace as _replace

    p = params or TSMParams()
    n = max(1, int(n_rows))

    # Normalise + denoise once; band calls skip the opening (morph_kernel=0).
    mask01 = _normalise_mask(mask, p.morph_kernel)

    if n == 1:
        prev = prev_anchors[0] if (prev_anchors and len(prev_anchors) > 0) else None
        p0 = _replace(p, morph_kernel=0)
        return [_detect_from_mask01(mask01, p0, prev)]

    lo = p.amin_frac
    hi = p.amax_frac
    if hi <= lo:          # degenerate config: expand by one pixel fraction
        hi = lo + 1.0 / max(mask01.shape[1], 1)
    band_w = (hi - lo) / n

    results = []
    for i in range(n):
        band_p = _replace(
            p,
            amin_frac=lo + i * band_w,
            amax_frac=lo + (i + 1) * band_w,
            morph_kernel=0,
        )
        prev = (prev_anchors[i]
                if (prev_anchors and i < len(prev_anchors))
                else None)
        results.append(_detect_from_mask01(mask01, band_p, prev))

    return results


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
        self._ay: Optional[float] = None    # filtered anchor row (real row,
                                             # NOT always 0 — see anchor_scan)
        self._px: Optional[float] = None    # filtered base x
        self._misses = 0

    def reset(self):
        self._ax = None
        self._ay = None
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
        ay = self._ay if self._ay is not None else 0.0
        denom = float(h_img - 1 - ay) if (h_img - 1) > ay else 1.0
        m = (self._px - self._ax) / denom
        # Node convention: b is the intercept at y=0, extrapolated from the
        # real (filtered) anchor row — see anchor_scan/_detect_from_mask01.
        b = self._ax - m * ay
        return TSMResult(
            anchor_xy=(int(round(self._ax)), int(round(ay))),
            base_xy=(int(round(self._px)), h_img - 1),
            slope=m,
            intercept=b,
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
                self._ay = float(result.anchor_xy[1])
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
        self._ay = a * float(result.anchor_xy[1]) + (1.0 - a) * self._ay
        self._px = a * result.bottom_x + (1.0 - a) * self._px
        self._misses = 0
        return self._emit(img_h)
