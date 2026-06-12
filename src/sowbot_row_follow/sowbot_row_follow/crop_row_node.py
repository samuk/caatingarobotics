#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2022 Agricultural-Robotics-Bonn
# Copyright 2026 Agroecology Lab Ltd
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# crop_row_node.py
# ================
# Derivado de / Derived from:
#   visual-multi-crop-row-navigation
#   https://github.com/Agricultural-Robotics-Bonn/visual-multi-crop-row-navigation
#   Autores / Authors: Alireza Ahmadi, Michael Halstead, Chris McCool (Agricultural-Robotics-Bonn)
#   Porta ROS2 / ROS2 port: Agroecology Lab Ltd
#   Referência / Reference: Ahmadi et al., "Towards Autonomous Crop-Agnostic Visual Navigation
#              in Arable Fields", arXiv:2109.11936, 2021.
#              Ahmadi et al., ICRA 2020.
#
# Adaptado para o projeto Sowbot por Agroecology Lab Ltd.
# Adapted for the Sowbot project by Agroecology Lab Ltd.
#
# Modificações / Modifications:
#   Reduzido para câmera RGB única, monocular, seguimento de fileira apenas para frente.
#   Stripped to single RGB camera, monocular, forward row-following only.
#   Removido / Removed: profundidade, estéreo, correspondência de características,
#     modos de navegação, troca de fileiras, câmera dupla.
#     (depth, stereo, feature matching, navigation modes, row switching, dual-camera)
#   Preservado verbatim ou quase verbatim / Preserved verbatim or near-verbatim:
#     - Índice ExG + limiar Otsu         (imageProc.getExgMask)
#     - Extração de centros de contorno  (imageProc.processRGBImage)
#     - Ajuste de linha por janela       (imageProc.findLinesInImage)
#     - Transformação câmera→imagem      (imageProc.cameraToImage)
#     - Matriz de interação visual       (controller.visualServoingCtl)
#     - Modelo de câmera                 (camera.Camera)
#
# Subscreve / Subscribes:
#   /caatinga_vision/row_nav/image_raw  (sensor_msgs/Image) — da / from camera_source_node
#
# Publica / Publishes:
#   /aoc/conditions/row_offset        (std_msgs/Float32)  deslocamento lateral [-1,1] / lateral offset
#   /aoc/conditions/row_heading_error (std_msgs/Float32)  erro de rumo em radianos / heading error in radians
#   /aoc/heartbeat/neo_vision         (std_msgs/Bool)     True enquanto fileiras detectadas (M10)
#   /caatinga_vision/row_nav/debug_image (sensor_msgs/Image)  frame anotado / annotated frame (Foxglove)
#
# Publica sempre / Always publishes (gated by /row_follow/enable service):
#   /cmd_vel  (geometry_msgs/Twist)

from __future__ import division, print_function
import math
from typing import Optional
import cv2 as cv
import numpy as np
import rclpy
import rclpy.duration
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool
# TSM detector (optional backend). Imported at module load, not per-frame.
from sowbot_row_follow.triangle_scan import detect_central_row, detect_rows_multi


# ===========================================================================
# Modelo de câmera / Camera model  (de / from camera.py — BSD-2)
# ===========================================================================
class Camera:
    def __init__(self, f=1, deltaz=1.2, deltay=0, scale=1,
                 tilt_angle=np.deg2rad(-80), pixel_size=0.96,
                 cx=0, cy=0, ar=1):
        self.f = f
        self.deltaz = deltaz
        self.deltay = deltay
        self.scale = scale
        self.tilt_angle = tilt_angle
        self.pixel_size = pixel_size
        self.cx = cx
        self.cy = cy
        self.ar = ar


# ===========================================================================
# Controlador de servo visual / Visual servoing controller  (de / from controller.py — BSD-2)
# Matriz de interação de Cherubini & Chaumette, preservada exatamente.
# ===========================================================================
def wrap_to_pi(theta: float) -> float:
    while theta < -math.pi:
        theta += 2 * math.pi
    while theta > math.pi:
        theta -= 2 * math.pi
    return theta


def visual_servoing_ctl(camera: Camera,
                        desired_state: np.ndarray,
                        actual_state: np.ndarray,
                        v_des: float) -> float:
    x = actual_state[0]
    y = actual_state[1]
    theta = actual_state[2]
    lambda_x_1 = 10
    lambda_w_1 = 3000
    lambdavec = np.array([lambda_x_1, lambda_w_1])
    controller_type = 0
    angle = camera.tilt_angle
    delta_z = camera.deltaz
    IntMat = np.array([
        [(-np.sin(angle) - y * np.cos(angle)) / delta_z, 0,
         x * (np.sin(angle) + y * np.cos(angle)) / delta_z,
         x * y, -1 - x**2, y],
        [0, -(np.sin(angle) + y * np.cos(angle)) / delta_z,
         y * (np.sin(angle) + y * np.cos(angle)) / delta_z,
         1 + y**2, -x * y, -x],
        [np.cos(angle) * np.cos(theta)**2 / delta_z,
         np.cos(angle) * np.cos(theta) * np.sin(theta) / delta_z,
         -(np.cos(angle) * np.cos(theta) * (y * np.sin(theta) + x * np.cos(theta))) / delta_z,
         -(y * np.sin(theta) + x * np.cos(theta)) * np.cos(theta),
         -(y * np.sin(theta) + x * np.cos(theta)) * np.sin(theta), -1]
    ])
    delta_y = camera.deltay
    TransfMat = np.array([
        [0,               -delta_y],
        [-np.sin(angle),   0],
        [np.cos(angle),    0],
        [0,                0],
        [0,               -np.cos(angle)],
        [0,               -np.sin(angle)],
    ])
    Trans_vel = TransfMat[:, 0]
    Trans_ang = TransfMat[:, 1]
    Jac = np.array([IntMat[controller_type, :], IntMat[2, :]])
    Jac_vel = np.matmul(Jac, Trans_vel)
    Jac_ang = np.matmul(Jac, Trans_ang)
    Jac_ang_pi = np.linalg.pinv([Jac_ang])
    trans_delta = actual_state[controller_type] - desired_state[controller_type]
    ang_delta = actual_state[2] - desired_state[2]
    delta = np.array([trans_delta, wrap_to_pi(ang_delta)])
    temp = lambdavec * delta
    ang_fb = np.matmul(-Jac_ang_pi.T, (temp + Jac_vel * v_des))
    return float(np.squeeze(ang_fb))


# ===========================================================================
# Índice de vegetação ExG + extração de contornos  (de / from imageProc.py — BSD-2)
# ===========================================================================
def compute_exg_mask(bgr_img: np.ndarray):
    img = bgr_img.astype("int32")
    b, g, r = img[:, :, 0], img[:, :, 1], img[:, :, 2]
    exg = 2 * g - r - b
    exg[exg < 0] = 0
    exg = exg.astype("uint8")
    blur = cv.GaussianBlur(exg, (5, 5), 0)
    _, mask = cv.threshold(blur, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    kernel = np.ones((10, 10), np.uint8)
    mask = cv.dilate(mask, kernel, iterations=1)
    return mask


def get_plant_centers(mask: np.ndarray, min_area: float) -> np.ndarray:
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    centers = []
    for c in contours:
        if cv.contourArea(c) >= min_area:
            M = cv.moments(c)
            if M["m00"] > 0:
                centers.append((int(M["m10"] / M["m00"]),
                                 int(M["m01"] / M["m00"])))
    return np.array(centers) if centers else np.empty((0, 2), dtype=int)


# ===========================================================================
# Ajuste de linha por janela de varredura  (de / from imageProc.findLinesInImage — BSD-2)
# ===========================================================================
def fit_line(points_xy: np.ndarray):
    if len(points_xy) < 2:
        return None, None
    y = points_xy[:, 1].astype(float)
    x = points_xy[:, 0].astype(float)
    if np.std(y) < 1.0:
        return None, None
    m, b = np.polyfit(y, x, 1)
    return m, b


def detect_crop_rows(centers: np.ndarray,
                     img_w: int, img_h: int,
                     n_windows: int, win_w: int) -> list:
    if len(centers) == 0:
        return []
    step = img_w / n_windows
    rows = []
    for i in range(n_windows):
        left = i * step
        right = left + win_w
        in_win = centers[(centers[:, 0] >= left) & (centers[:, 0] < right)]
        if len(in_win) < 2:
            continue
        m, b = fit_line(in_win)
        if m is None:
            continue
        bx = m * img_h + b
        if 0 <= bx <= img_w:
            rows.append((bx, m, b))
    return rows


# ===========================================================================
# Visualização de depuração  (de / from imageProc.drawGraphics — BSD-2)
# ===========================================================================
def draw_debug(bgr: np.ndarray, centers: np.ndarray,
               rows: list, img_w: int, img_h: int,
               held: bool = False, swap_remaining_s: float = 0.0,
               avg_row: Optional[tuple] = None,
               row_offset_active: bool = False) -> np.ndarray:
    """Draw debug overlay.

    rows             — individual detected rows; green, amber when swap-hold active.
    avg_row          — (bx, m, b) averaged heading. Supplied only for tsm_n_rows > 1;
                       None for single-row and scanwin paths.
                         red    — all bands valid, heading is a true average.
                         yellow — single-row offset active (one band missing; heading
                                  is the detected row shifted by _last_row_spacing_px/2
                                  toward the missing band).
    row_offset_active — True when the single-row offset has been applied this frame.
    The vertical centre reference is drawn in grey so it does not clash with
    the averaged-heading line.
    """
    out = bgr.copy()
    for (cx, cy) in centers:
        cv.circle(out, (cx, cy), 4, (255, 0, 255), -1)
    # Individual rows: green, amber if swap-hold active
    line_colour = (0, 165, 255) if held else (0, 255, 0)
    for (bx, m, b) in rows:
        x_top = int(m * 0 + b)
        x_bot = int(bx)
        cv.line(out, (x_top, 0), (x_bot, img_h), line_colour, 2)
    # Averaged heading (multi-row TSM only):
    #   red    — full average (all bands valid)
    #   yellow — single-row offset applied (one or more bands missing)
    if avg_row is not None:
        bx, m, b = avg_row
        avg_colour = (0, 255, 255) if row_offset_active else (0, 0, 255)
        cv.line(out, (int(b), 0), (int(bx), img_h), avg_colour, 2)
    # Vertical centre reference: thin grey (offset reference, not heading)
    cv.line(out, (img_w // 2, 0), (img_w // 2, img_h), (128, 128, 128), 1)
    if held and swap_remaining_s > 0.0:
        label = "HOLD %.1fs" % swap_remaining_s
        cv.putText(out, label, (8, 24),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2, cv.LINE_AA)
    if row_offset_active:
        cv.putText(out, "1-ROW", (8, 48),
                   cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv.LINE_AA)
    return out


# ===========================================================================
# Nó ROS 2 / ROS 2 Node
# ===========================================================================
class CropRowNode(Node):
    def __init__(self):
        super().__init__("crop_row_node")
        self.bridge = CvBridge()

        # -------------------------------------------------------------------
        # Parâmetros
        # -------------------------------------------------------------------
        self.declare_parameter("image_topic", "/caatinga_vision/row_nav/image_raw")
        self.declare_parameter("debug_image_topic", "/caatinga_vision/row_nav/debug_image")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("n_scan_windows", 8)
        self.declare_parameter("window_width", 80)
        self.declare_parameter("min_contour_area", 10.0)
        self.declare_parameter("camera_height_m", 1.2)
        self.declare_parameter("camera_tilt_deg", -80.0)
        self.declare_parameter("linear_vel", 0.15)
        self.declare_parameter("omega_scaler", 0.1)
        self.declare_parameter("max_omega", 0.4)
        self.declare_parameter("heartbeat_timeout_s", 2.0)
        self.declare_parameter("use_direct_cmd_vel", True)
        self.declare_parameter("detector", "scanwin")
        self.declare_parameter("tsm_s", 0.2)
        self.declare_parameter("tsm_amin_frac", 0.2)
        self.declare_parameter("tsm_amax_frac", 0.7)
        self.declare_parameter("tsm_b_frac", 0.0)
        self.declare_parameter("tsm_c_frac", 1.0)
        self.declare_parameter("tsm_anchor_min_sum", 1.0)
        self.declare_parameter("tsm_morph_kernel", 5)
        self.declare_parameter("tsm_anchor_select", "argmax")
        self.declare_parameter("tsm_peak_rel_thresh", 0.6)
        self.declare_parameter("tsm_prior_lambda", 0.01)
        self.declare_parameter("tsm_prior_init_frac", 0.3)
        self.declare_parameter("tsm_weight_k", 0.5)
        self.declare_parameter("tsm_weight_side", "left")
        self.declare_parameter("tsm_max_angle_deg", 15.0)
        self.declare_parameter("tsm_n_rows", 1)
        self.declare_parameter("tsm_filter_enable", True)
        self.declare_parameter("tsm_filter_alpha", 0.4)
        self.declare_parameter("tsm_filter_jump_gate_frac", 0.35)
        self.declare_parameter("tsm_filter_max_hold", 2)
        self.declare_parameter("tsm_swap_hold_s", 6.0)
        self.declare_parameter("tsm_swap_threshold_frac", 0.15)
        self.declare_parameter("tsm_single_row_offset_px", 95)

        p = self.get_parameter
        self.image_topic: str    = p("image_topic").value
        self.debug_topic: str    = p("debug_image_topic").value
        self.img_w: int          = p("image_width").value
        self.img_h: int          = p("image_height").value
        self.n_windows: int      = p("n_scan_windows").value
        self.win_w: int          = p("window_width").value
        self.min_area: float     = p("min_contour_area").value
        self.linear_vel: float   = p("linear_vel").value
        self.omega_scaler: float = p("omega_scaler").value
        self.max_omega: float    = p("max_omega").value
        self.hb_timeout: float   = p("heartbeat_timeout_s").value

        self.detector: str = str(p("detector").value).lower()
        if self.detector not in ("scanwin", "tsm"):
            self.get_logger().warn(
                "detector='%s' unrecognised — falling back to 'scanwin'" % self.detector)
            self.detector = "scanwin"

        if self.detector == "tsm":
            from sowbot_row_follow.triangle_scan import (
                TSMParams, TSMFilterParams, TSMFilter)
            self.tsm_n_rows: int = max(1, int(p("tsm_n_rows").value))
            self.tsm_params = TSMParams(
                s=p("tsm_s").value,
                amin_frac=p("tsm_amin_frac").value,
                amax_frac=p("tsm_amax_frac").value,
                b_frac=p("tsm_b_frac").value,
                c_frac=p("tsm_c_frac").value,
                anchor_min_sum=p("tsm_anchor_min_sum").value,
                morph_kernel=p("tsm_morph_kernel").value,
                anchor_select=str(p("tsm_anchor_select").value),
                peak_rel_thresh=p("tsm_peak_rel_thresh").value,
                prior_lambda=p("tsm_prior_lambda").value,
                prior_init_frac=p("tsm_prior_init_frac").value,
                weight_k=p("tsm_weight_k").value,
                weight_side=str(p("tsm_weight_side").value),
                max_angle_deg=p("tsm_max_angle_deg").value,
            )
            _fp = TSMFilterParams(
                enable=p("tsm_filter_enable").value,
                alpha=p("tsm_filter_alpha").value,
                jump_gate_frac=p("tsm_filter_jump_gate_frac").value,
                max_hold=p("tsm_filter_max_hold").value,
            )
            # One independent filter per row slot; all share the same params.
            self.tsm_filters: list = [
                TSMFilter(_fp) for _ in range(self.tsm_n_rows)
            ]
            # Row-swap hold state (single-row path only; unused when tsm_n_rows > 1
            # because band-partitioned detection prevents inter-row flipping).
            self._swap_hold_s: float = float(p("tsm_swap_hold_s").value)
            self._swap_thresh_px: float = p("tsm_swap_threshold_frac").value * self.img_w
            self._held_anchor_x: Optional[float] = None
            self._held_bottom_x: Optional[float] = None
            self._swap_deadline: Optional[rclpy.time.Time] = None

            # Option C single-row offset (multi-row path only).
            # Tracks the observed pixel distance between the outermost valid
            # rows from frames where all tsm_n_rows bands fired.  When only a
            # subset of bands fire, avg_row is shifted by half this spacing
            # toward the missing band(s) so the servo aims between the rows
            # rather than directly at the one visible row.
            # Seeded to 2 × tsm_single_row_offset_px so the first single-row
            # frame uses a reasonable value before any two-row frame is seen.
            _offset_seed = float(p("tsm_single_row_offset_px").value)
            self._last_row_spacing_px: float = _offset_seed * 2.0

        self.camera = Camera(
            deltaz=p("camera_height_m").value,
            tilt_angle=np.deg2rad(p("camera_tilt_deg").value),
        )

        self.get_logger().info(
            "crop_row_node started | subscribing to %s | "
            "cmd_vel gated by /row_follow/enable service" % self.image_topic
        )

        # -------------------------------------------------------------------
        # Subscriber
        # -------------------------------------------------------------------
        self.sub = self.create_subscription(
            Image, self.image_topic, self._on_image, 10)

        # -------------------------------------------------------------------
        # Publishers
        # -------------------------------------------------------------------
        self.pub_offset = self.create_publisher(
            Float32, "/aoc/conditions/row_offset", 10)
        self.pub_heading = self.create_publisher(
            Float32, "/aoc/conditions/row_heading_error", 10)
        self.pub_heartbeat = self.create_publisher(
            Bool, "/aoc/heartbeat/neo_vision", 10)
        self.pub_debug = self.create_publisher(
            Image, self.debug_topic, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        self._enabled = False
        self.srv_enable = self.create_service(
            SetBool, "/row_follow/enable", self._on_enable_service)

        self._last_detection_time = None
        self._hb_timer = self.create_timer(0.2, self._publish_heartbeat)

    # -----------------------------------------------------------------------
    # TSM row-swap hold logic  (single-row path only)
    # -----------------------------------------------------------------------
    def _tsm_apply_swap_hold(self, tsm):
        """Apply row-swap debounce to a raw TSMResult (before TSMFilter.update()).

        Must be called on the raw detector output so the full anchor jump is
        visible — TSMFilter's EMA would smear a 240px row-switch into ~80px
        increments that never cross the threshold.

        Holds the last accepted row's anchor_x AND bottom_x together so that
        _make_held_tsm reconstructs the correct frozen line geometry — not a
        chimera of the old anchor and the new row's base point.

        Returns (tsm_to_use, held, swap_remaining_s).
        """
        if not tsm.valid:
            return tsm, False, 0.0

        new_ax = float(tsm.anchor_xy[0])
        new_bx = float(tsm.bottom_x)

        # First valid detection — lock on unconditionally.
        if self._held_anchor_x is None:
            self._held_anchor_x = new_ax
            self._held_bottom_x = new_bx
            self._swap_deadline = None
            return tsm, False, 0.0

        now = self.get_clock().now()
        shift = abs(new_ax - self._held_anchor_x)

        if shift <= self._swap_thresh_px:
            # Same row — EMA-track both endpoints (slow: operating on raw
            # detections which have higher frame-to-frame jitter than
            # post-filter values) and reset any pending swap timer.
            self._held_anchor_x = 0.95 * self._held_anchor_x + 0.05 * new_ax
            self._held_bottom_x = 0.95 * self._held_bottom_x + 0.05 * new_bx
            self._swap_deadline = None
            return tsm, False, 0.0

        # Anchor has jumped — possible row switch.
        if self._swap_deadline is None:
            self._swap_deadline = now + rclpy.duration.Duration(
                seconds=self._swap_hold_s)
            self.get_logger().info(
                "TSM row-swap hold armed: anchor shifted %.0fpx (threshold %.0fpx), "
                "holding current row for %.1fs"
                % (shift, self._swap_thresh_px, self._swap_hold_s))

        remaining_ns = (self._swap_deadline - now).nanoseconds
        if remaining_ns > 0:
            remaining_s = remaining_ns * 1e-9
            return self._make_held_tsm(), True, remaining_s

        # Timer expired — accept the new row.
        self.get_logger().info(
            "TSM row-swap hold expired: accepting new row at anchor_x=%.0f" % new_ax)
        self._held_anchor_x = new_ax
        self._held_bottom_x = new_bx
        self._swap_deadline = None
        return tsm, False, 0.0

    def _make_held_tsm(self):
        """Reconstruct a TSMResult from the held anchor + bottom endpoint pair.

        Both points belong to the same (last accepted) row, so slope and
        intercept correctly describe that row's frozen geometry.
        """
        from sowbot_row_follow.triangle_scan import TSMResult
        ax = self._held_anchor_x
        px = self._held_bottom_x
        denom = float(self.img_h - 1) if self.img_h > 1 else 1.0
        m = (px - ax) / denom
        return TSMResult(
            anchor_xy=(int(round(ax)), 0),
            base_xy=(int(round(px)), self.img_h - 1),
            slope=m,
            intercept=float(ax),
            bottom_x=float(px),
            valid=True,
        )

    def _reset_swap_hold(self):
        """Clear all row-swap hold state (called on disable and re-enable)."""
        self._held_anchor_x = None
        self._held_bottom_x = None
        self._swap_deadline = None

    # -----------------------------------------------------------------------
    # Image callback
    # -----------------------------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error("CV bridge error: %s" % str(e))
            return

        h, w = bgr.shape[:2]
        if w != self.img_w or h != self.img_h:
            bgr = cv.resize(bgr, (self.img_w, self.img_h))

        mask = compute_exg_mask(bgr)
        centers = get_plant_centers(mask, self.min_area)

        held = False
        swap_remaining_s = 0.0
        # avg_row: (bx, m, b) heading used for visual servoing and red/yellow
        # overlay in debug_image.  Set only for tsm_n_rows > 1; the single-row
        # paths leave it None so draw_debug omits the separate overlay line.
        avg_row: Optional[tuple] = None
        # row_offset_active: True when avg_row has been shifted by the single-row
        # offset (one or more bands missing).  Triggers yellow overlay + label.
        row_offset_active: bool = False

        if self.detector == "tsm":
            if self.tsm_n_rows > 1:
                # Multi-row path: detect_rows_multi normalises the mask once and
                # runs an independent TSM scan in each horizontal band.
                raw_results = detect_rows_multi(
                    mask, self.tsm_params,
                    n_rows=self.tsm_n_rows,
                    prev_anchors=[f.prev_anchor for f in self.tsm_filters],
                )
                filtered = [
                    self.tsm_filters[i].update(r, self.img_w, self.img_h)
                    for i, r in enumerate(raw_results)
                ]
                detected_rows = [
                    (r.bottom_x, r.slope, r.intercept)
                    for r in filtered if r.valid
                ]
                if detected_rows:
                    n_valid = len(detected_rows)
                    avg_bx = float(np.mean([r[0] for r in detected_rows]))
                    avg_m  = float(np.mean([r[1] for r in detected_rows]))
                    avg_b  = float(np.mean([r[2] for r in detected_rows]))

                    if n_valid == self.tsm_n_rows:
                        # All bands valid: update observed inter-row spacing.
                        # Guard against degenerate frames where both rows overlap.
                        bx_vals = [r[0] for r in detected_rows]
                        spacing = float(max(bx_vals) - min(bx_vals))
                        if spacing > 20.0:
                            self._last_row_spacing_px = spacing
                    else:
                        # Partial detection (Option C): shift avg_row by half
                        # the last observed spacing toward the missing band(s).
                        valid_idx   = [i for i, r in enumerate(filtered) if r.valid]
                        missing_idx = [i for i in range(self.tsm_n_rows)
                                       if i not in set(valid_idx)]
                        valid_c   = sum(valid_idx)   / len(valid_idx)
                        missing_c = sum(missing_idx) / len(missing_idx)
                        sign = +1.0 if missing_c > valid_c else -1.0
                        offset = sign * self._last_row_spacing_px / 2.0
                        avg_bx += offset
                        avg_b  += offset
                        row_offset_active = True
                        self.get_logger().debug(
                            "single-row offset: %d/%d bands valid, shift %.1fpx (%s)"
                            % (n_valid, self.tsm_n_rows, abs(offset),
                               "right" if sign > 0 else "left"))

                    avg_row = (avg_bx, avg_m, avg_b)
            else:
                # Single-row path with swap-hold debounce.
                tsm_raw = detect_central_row(
                    mask, self.tsm_params,
                    prev_anchor=self.tsm_filters[0].prev_anchor,
                )
                tsm_raw, held, swap_remaining_s = self._tsm_apply_swap_hold(tsm_raw)
                tsm = self.tsm_filters[0].update(tsm_raw, self.img_w, self.img_h)
                detected_rows = (
                    [(tsm.bottom_x, tsm.slope, tsm.intercept)] if tsm.valid else []
                )
                # avg_row stays None: single green line is the heading; no red overlay.
        else:
            detected_rows = detect_crop_rows(
                centers, self.img_w, self.img_h, self.n_windows, self.win_w)

        debug_img = draw_debug(
            bgr, centers, detected_rows,
            self.img_w, self.img_h,
            held=held, swap_remaining_s=swap_remaining_s,
            avg_row=avg_row,
            row_offset_active=row_offset_active,
        )
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
        debug_msg.header = msg.header
        self.pub_debug.publish(debug_msg)

        if not detected_rows:
            return

        # Servo uses avg_row when pre-computed (multi-row TSM); otherwise
        # compute it from detected_rows (single-row TSM, scanwin).
        if avg_row is not None:
            avg_bx, avg_slope = avg_row[0], avg_row[1]
        else:
            avg_bx = float(np.mean([r[0] for r in detected_rows]))
            avg_slope = float(np.mean([r[1] for r in detected_rows]))
        heading_error_rad = float(np.arctan(avg_slope))

        actual_feature = np.array([
            avg_bx - self.img_w / 2.0,
            self.img_h / 2.0,
            heading_error_rad,
        ])
        desired_feature = np.array([0.0, actual_feature[1], 0.0])

        omega_raw = visual_servoing_ctl(
            self.camera, desired_feature, actual_feature, self.linear_vel)
        omega = float(np.clip(
            self.omega_scaler * omega_raw, -self.max_omega, self.max_omega))

        norm_offset = float(np.clip(
            actual_feature[0] / (self.img_w / 2.0), -1.0, 1.0))

        self._last_detection_time = self.get_clock().now()

        off_msg = Float32()
        off_msg.data = norm_offset
        self.pub_offset.publish(off_msg)

        hdg_msg = Float32()
        hdg_msg.data = heading_error_rad
        self.pub_heading.publish(hdg_msg)

        if self._enabled:
            twist = Twist()
            twist.linear.x = self.linear_vel
            twist.angular.z = omega
            self.pub_cmd_vel.publish(twist)

        self.get_logger().debug(
            "rows=%d offset=%.3f heading=%.1fdeg omega=%.3f hold=%s"
            % (len(detected_rows), norm_offset,
               math.degrees(heading_error_rad), omega, held)
        )

    # -----------------------------------------------------------------------
    # Enable / disable service
    # -----------------------------------------------------------------------
    def _on_enable_service(self, request: SetBool.Request,
                           response: SetBool.Response) -> SetBool.Response:
        self._enabled = request.data
        if not self._enabled:
            self._publish_zero_cmd_vel()
            if self.detector == "tsm":
                self._reset_swap_hold()
        response.success = True
        response.message = "enabled" if self._enabled else "disabled"
        self.get_logger().info("row_follow/enable → %s" % response.message)
        return response

    def _publish_zero_cmd_vel(self) -> None:
        self.pub_cmd_vel.publish(Twist())

    # -----------------------------------------------------------------------
    # Heartbeat M10
    # -----------------------------------------------------------------------
    def _publish_heartbeat(self) -> None:
        alive = False
        if self._last_detection_time is not None:
            elapsed = (
                self.get_clock().now() - self._last_detection_time
            ).nanoseconds * 1e-9
            alive = elapsed < self.hb_timeout
        if not alive and self._last_detection_time is not None:
            self.get_logger().warn(
                "crop_row_node: no row detection — heartbeat DEAD. "
                "Limbic System should enter PERCEPTION_DEGRADED (M10).",
                throttle_duration_sec=5.0,
            )
        hb = Bool()
        hb.data = alive
        self.pub_heartbeat.publish(hb)


def main(args=None):
    rclpy.init(args=args)
    node = CropRowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
