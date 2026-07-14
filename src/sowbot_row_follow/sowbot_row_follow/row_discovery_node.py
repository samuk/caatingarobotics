#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2026 Agroecology Lab Ltd
# BSD-2-Clause License

# row_discovery_node.py
# ======================
# ROS 2 node on Limbic implementing 'auto' / discovery-mode row following:
# no pre-existing topo map is required. The robot follows the row it's
# currently facing (reusing Neo's existing visual servo + heartbeat), and
# when the row ends it performs a headland turn — alternating left/right
# each time, boustrophedon-style — until Neo re-detects a row, then resumes
# following. A topo node is dropped at every row-exit and row-entry point
# using the *same* node/edge schema ui_node.py's drop_topo_node() writes, so
# the resulting map is immediately usable by the existing topological_
# navigation stack for subsequent (non-discovery) runs — row edges use the
# 'limbic_row_follow' action exactly like a pre-planned F2C map would.
#
# `map_name` defaults to 'maize_map' — the SAME file topological_map_
# manager2 is already serving live (see TMAP2_FILE) — so discovered nodes
# land directly in the map the rest of the stack is using, not a separate
# file. They coexist with whatever F2C-authored nodes are already in that
# file as their own connected sub-graph (DISC_R*_IN/_OUT); nothing wires
# the origin node to the pre-existing nodes automatically. Every node drop
# hot-reloads the live map via switch_topological_map with no_alias=True,
# which REPLACES whatever topo map Nav2/topological_navigation currently
# has loaded — don't run discovery concurrently with active navigation on
# the same map for this reason.
#
# An origin node (row_role='entry', same schema as every other node) is
# dropped at the robot's actual pose the instant start_discovery is called
# — before any motion — so the very first row has a recorded start point
# the same way every subsequent row does.
#
# Requires (same as limbic_row_follow_node.py):
#   /row_follow/enable        (std_srvs/SetBool)   — Neo's crop_row_node
#   /aoc/heartbeat/neo_vision (std_msgs/Bool)       — Neo perception health
#                                                      (kept alive even while
#                                                      disabled — see notes)
#   /cmd_vel                  (geometry_msgs/Twist) — direct drive during the
#                                                      headland maneuver only
#   TF map->base_link                               — pose/heading/odometry
#                                                      for the maneuver and
#                                                      for dropped node poses
#
# Also requires (matches ui_node.py's WriteTopologicalMap usage):
#   /topological_map_manager2/switch_topological_map (topological_navigation_msgs/srv/WriteTopologicalMap)
#
# Headland maneuver is five open-loop TF-monitored segments, alternating
# turn direction each time so consecutive rows are traversed in opposite
# directions (classic boustrophedon / Ω-turn), and IS NOT itself driven by
# vision — Neo is disabled throughout (mirrors the Phase 0 align pattern in
# limbic_row_follow_node.py, which is also the only other place in the
# stack that drives /cmd_vel directly while Neo is disabled):
#   1. HEADLAND_CLEARANCE: drive straight forward `headland_clearance_m`
#      (default 0.5m) — clears the implement/tracks past the row-end
#      canopy edge before pivoting, so the turn doesn't cut through
#      standing crop right at the row end.
#   2. TURN_1: turn in place ~90 deg (direction alternates each maneuver)
#   3. LATERAL_STEP: drive forward `lateral_step_m` (default 1.0m) — this
#      is what actually produces the sideways offset onto the next row,
#      i.e. the "1m along the headland" travel.
#   4. TURN_2: turn in place ~90 deg the SAME direction (net 180 deg from
#      the original heading — now facing back down the field into the
#      new row).
#   5. SEEK_ROW: creep forward at `creep_linear_vel`, still with Neo
#      disabled but still watching /aoc/heartbeat/neo_vision (crop_row_node
#      computes detections/heartbeat independent of the enable flag — only
#      cmd_vel publishing is gated), until the heartbeat is alive
#      continuously for `row_reacquire_confirm_s`. That confirms a row,
#      not a single noisy frame, is what's in view before handing back
#      control.
#
# If SEEK_ROW's straight creep doesn't reacquire within
# `max_search_distance_m`, that's NOT immediately treated as the field
# boundary — the turn onto the new row may just be a bit misaligned. It
# falls through to:
#   6. WIGGLE_SEEK: an expanding fan scan, alternating single-track nudges
#      (left track forward ~`wiggle_step_m` [default 0.10m], then right
#      track forward the same, growing further out each cycle) up to
#      ±`wiggle_max_sweep_deg` (default 90 deg) either side of the heading
#      TURN_2 left it facing, checking for reacquisition after every single
#      nudge. Each nudge is real diff-drive kinematics for a single track
#      moving and the other stationary (v=v_side/2, w=∓v_side/track_sep),
#      closed-loop against actual TF yaw rather than open-loop timing.
#      `track_separation_m` (default 0.698m) matches devkit_simulation's
#      default sim model (sowbot_01.xacro) — override it if driving a
#      different chassis (e.g. robo_caatinga.urdf.xacro is 1.85m), or the
#      wiggle angles will be wrong for that vehicle.
#
# Only if the wiggle scan ALSO exhausts its full ±90 deg sweep without
# reacquiring is this treated as having discovered the end of the field
# (not just a bad frame or a misaligned turn), and discovery stops cleanly
# with the map so far intact.
#
# Row-end detection during FOLLOW_ROW is the mirror image: a heartbeat that
# goes (and stays) false for `row_lost_confirm_s` is treated as reaching
# the end of the row, not a transient occlusion — same debounce reasoning
# as limbic_row_follow_node.py's own heartbeat-staleness monitor, just
# applied as a trigger instead of an abort.

import copy
import json
import math
import os
import threading
import time
from datetime import datetime, timezone
from enum import Enum, auto

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

try:
    from topological_navigation_msgs.srv import WriteTopologicalMap
    _TOPO_SRV_OK = True
except ImportError:
    _TOPO_SRV_OK = False

_ROW_ACTION = 'limbic_row_follow'
_NAV_ACTION = 'navigate_to_pose'


class _State(Enum):
    IDLE = auto()
    FOLLOW_ROW = auto()
    HEADLAND_CLEARANCE = auto()  # straight creep clear of row-end canopy
    TURN_1 = auto()          # first 90 deg turn
    LATERAL_STEP = auto()    # drive forward lateral_step_m
    TURN_2 = auto()          # second 90 deg turn (net 180 deg)
    SEEK_ROW = auto()        # creep forward until heartbeat reacquired
    WIGGLE_SEEK = auto()     # fan-scan fallback if straight seek fails
    COMPLETE = auto()
    FAULT = auto()


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class RowDiscoveryNode(Node):

    def __init__(self):
        super().__init__('row_discovery_node')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('headland_clearance_m', 0.5)
        self.declare_parameter('lateral_step_m', 1.0)
        self.declare_parameter('turn_speed_rad_s', 0.3)
        self.declare_parameter('creep_linear_vel', 0.12)
        self.declare_parameter('row_lost_confirm_s', 1.5)
        self.declare_parameter('row_reacquire_confirm_s', 0.75)
        self.declare_parameter('max_search_distance_m', 4.0)
        self.declare_parameter('wiggle_step_m', 0.10)
        self.declare_parameter('wiggle_speed_mps', 0.15)
        self.declare_parameter('wiggle_max_sweep_deg', 90.0)
        self.declare_parameter('track_separation_m', 0.698)  # sowbot_01.xacro
        self.declare_parameter('enable_service_timeout_s', 2.0)
        self.declare_parameter('monitor_rate_hz', 10.0)
        self.declare_parameter('first_turn_direction', 'right')  # 'right'|'left'
        self.declare_parameter('map_name', 'maize_map')
        self.declare_parameter('maps_dir', '/workspace/maps')
        self.declare_parameter('node_name_prefix', 'DISC_R')

        self._headland_clearance: float = self.get_parameter('headland_clearance_m').value
        self._lateral_step: float = self.get_parameter('lateral_step_m').value
        self._turn_speed: float = self.get_parameter('turn_speed_rad_s').value
        self._creep_vel: float = self.get_parameter('creep_linear_vel').value
        self._row_lost_confirm_s: float = self.get_parameter('row_lost_confirm_s').value
        self._row_reacquire_confirm_s: float = self.get_parameter(
            'row_reacquire_confirm_s').value
        self._max_search_m: float = self.get_parameter('max_search_distance_m').value
        self._wiggle_step_m: float = self.get_parameter('wiggle_step_m').value
        self._wiggle_speed_mps: float = self.get_parameter('wiggle_speed_mps').value
        self._wiggle_max_sweep_rad: float = math.radians(
            self.get_parameter('wiggle_max_sweep_deg').value)
        self._track_sep: float = self.get_parameter('track_separation_m').value
        self._enable_timeout: float = self.get_parameter('enable_service_timeout_s').value
        self._map_name: str = self.get_parameter('map_name').value
        self._maps_dir: str = self.get_parameter('maps_dir').value
        self._name_prefix: str = self.get_parameter('node_name_prefix').value

        monitor_rate: float = self.get_parameter('monitor_rate_hz').value
        self._dt = 1.0 / monitor_rate

        # turn_right == True means "turn clockwise (negative angular.z) first".
        # Alternates every maneuver — row 1->2 goes one way, row 2->3 the
        # other, matching how a real boustrophedon pattern must alternate
        # or successive rows would walk off in the same direction forever.
        self._turn_right: bool = (
            self.get_parameter('first_turn_direction').value.lower() != 'left')

        self._cbg = ReentrantCallbackGroup()

        # ------------------------------------------------------------------
        # TF
        # ------------------------------------------------------------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._map_frame = 'map'

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._state: _State = _State.IDLE
        self._heartbeat_alive: bool = False
        self._heartbeat_state_since: float = 0.0   # monotonic time of last flip
        self._maneuver_start_pose: tuple | None = None   # (x, y, yaw)
        self._maneuver_target_yaw: float | None = None
        self._seek_start_pose: tuple | None = None
        self._wiggle_targets: list[float] = []   # queued yaw offsets, rad
        self._wiggle_ref_yaw: float | None = None
        self._wiggle_target_yaw: float | None = None
        self._wiggle_side: str | None = None     # 'left' | 'right' track fwd
        self._row_index: int = 1
        self._last_node_name: str | None = None
        self._topo_doc: dict = {}
        self._status_msg: str = 'idle'

        # ------------------------------------------------------------------
        # Pub/Sub
        # ------------------------------------------------------------------
        self.create_subscription(
            Bool, '/aoc/heartbeat/neo_vision', self._on_heartbeat, 10,
            callback_group=self._cbg)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._status_pub = self.create_publisher(String, '/row_discovery/status', 10)

        self._enable_client = self.create_client(
            SetBool, '/row_follow/enable', callback_group=self._cbg)

        if _TOPO_SRV_OK:
            self._switch_map_cli = self.create_client(
                WriteTopologicalMap,
                '/topological_map_manager2/switch_topological_map',
                callback_group=self._cbg)
        else:
            self._switch_map_cli = None
            self.get_logger().warn(
                'topological_navigation_msgs not available — discovered map '
                'will be written to disk but not hot-loaded')

        # ------------------------------------------------------------------
        # Services to start/stop discovery — off by default. This does not
        # auto-run on node bring-up; an operator (or a mission step) must
        # explicitly kick it off once the robot is manually positioned
        # facing down the first row.
        # ------------------------------------------------------------------
        self.create_service(Trigger, '~/start_discovery', self._on_start,
                            callback_group=self._cbg)
        self.create_service(Trigger, '~/stop_discovery', self._on_stop,
                            callback_group=self._cbg)

        self._timer = self.create_timer(self._dt, self._tick,
                                          callback_group=self._cbg)

        self.get_logger().info('row_discovery_node ready (idle)')

    # ------------------------------------------------------------------
    # Heartbeat tracking — mirrors limbic_row_follow_node's staleness
    # monitor, but we track the flip time so FOLLOW_ROW/SEEK_ROW can debounce
    # against a *duration* of true/false rather than acting on one sample.
    # ------------------------------------------------------------------

    def _on_heartbeat(self, msg: Bool) -> None:
        if msg.data != self._heartbeat_alive:
            self._heartbeat_alive = msg.data
            self._heartbeat_state_since = time.monotonic()

    def _heartbeat_state_duration(self) -> float:
        return time.monotonic() - self._heartbeat_state_since

    # ------------------------------------------------------------------
    # Start/stop services
    # ------------------------------------------------------------------

    def _on_start(self, request, response):
        if self._state not in (_State.IDLE, _State.COMPLETE, _State.FAULT):
            response.success = False
            response.message = f'already running (state={self._state.name})'
            return response
        self._row_index = 1
        self._last_node_name = None
        self._turn_right = (
            self.get_parameter('first_turn_direction').value.lower() != 'left')
        self._drop_origin_node()
        self._enter_follow_row()
        response.success = True
        response.message = 'discovery started'
        return response

    def _on_stop(self, request, response):
        self._safe_disable()
        self._publish_zero_twist()
        self._state = _State.IDLE
        self._set_status('stopped by operator')
        response.success = True
        response.message = 'discovery stopped'
        return response

    def _set_status(self, msg: str) -> None:
        self._status_msg = msg
        m = String()
        m.data = f'[row {self._row_index}] {self._state.name}: {msg}'
        self._status_pub.publish(m)
        self.get_logger().info(m.data)

    # ------------------------------------------------------------------
    # Pose helper — same pattern as limbic_row_follow_node._pose_in_frame,
    # staleness-gated so a transient TF gap can't corrupt a maneuver.
    # ------------------------------------------------------------------

    _TF_STALENESS_LIMIT_S = 1.0

    def _pose(self) -> tuple | None:
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, 'base_link', Time())
        except (LookupException, ConnectivityException,
                ExtrapolationException) as e:
            self.get_logger().warn(
                f'_pose: TF lookup failed ({type(e).__name__}): {e}',
                throttle_duration_sec=2.0)
            return None
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - stamp > self._TF_STALENESS_LIMIT_S:
            return None
        p, q = t.transform.translation, t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))
        return (p.x, p.y, yaw)

    def _publish_zero_twist(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    # ------------------------------------------------------------------
    # Neo enable/disable — synchronous, mirrors limbic_row_follow_node.
    # ------------------------------------------------------------------

    def _call_enable(self, enable: bool) -> bool:
        if not self._enable_client.wait_for_service(
                timeout_sec=self._enable_timeout):
            self.get_logger().error(
                'row_discovery: /row_follow/enable service not available')
            return False
        req = SetBool.Request()
        req.data = enable
        future = self._enable_client.call_async(req)
        start = self.get_clock().now()
        timeout = Duration(seconds=self._enable_timeout)
        while not future.done():
            if self.get_clock().now() - start > timeout:
                self.get_logger().error('row_discovery: enable call timed out')
                return False
            time.sleep(0.02)
        resp = future.result()
        return bool(resp and resp.success)

    def _safe_disable(self) -> bool:
        return self._call_enable(False)

    # ------------------------------------------------------------------
    # State machine entry points
    # ------------------------------------------------------------------

    def _enter_follow_row(self) -> None:
        if not self._call_enable(True):
            self._set_status('enable failed — fault')
            self._state = _State.FAULT
            return
        self._state = _State.FOLLOW_ROW
        self._set_status('following row')

    def _enter_headland_clearance(self) -> None:
        self._safe_disable()
        self._publish_zero_twist()
        pose = self._pose()
        if pose is None:
            self._set_status('no TF at row end — fault')
            self._state = _State.FAULT
            return
        self._maneuver_start_pose = pose
        self._state = _State.HEADLAND_CLEARANCE
        self._set_status(
            f'headland clearance {self._headland_clearance:.2f}m straight')

    def _enter_turn_1(self) -> None:
        self._safe_disable()
        self._publish_zero_twist()
        pose = self._pose()
        if pose is None:
            self._set_status('no TF at row end — fault')
            self._state = _State.FAULT
            return
        _x, _y, yaw = pose
        sign = -1.0 if self._turn_right else 1.0
        self._maneuver_target_yaw = _wrap_pi(yaw + sign * (math.pi / 2.0))
        self._state = _State.TURN_1
        self._set_status(f'headland turn 1/2 ({"right" if self._turn_right else "left"})')

    def _enter_lateral_step(self) -> None:
        pose = self._pose()
        if pose is None:
            self._state = _State.FAULT
            return
        self._maneuver_start_pose = pose
        self._state = _State.LATERAL_STEP
        self._set_status(f'lateral step {self._lateral_step:.2f}m')

    def _enter_turn_2(self) -> None:
        pose = self._pose()
        if pose is None:
            self._state = _State.FAULT
            return
        _x, _y, yaw = pose
        sign = -1.0 if self._turn_right else 1.0
        self._maneuver_target_yaw = _wrap_pi(yaw + sign * (math.pi / 2.0))
        self._state = _State.TURN_2
        self._set_status('headland turn 2/2')

    def _enter_seek_row(self) -> None:
        pose = self._pose()
        if pose is None:
            self._state = _State.FAULT
            return
        self._seek_start_pose = pose
        self._state = _State.SEEK_ROW
        self._set_status('creeping forward, seeking next row')

    def _enter_wiggle_seek(self) -> None:
        pose = self._pose()
        if pose is None:
            self._state = _State.FAULT
            return
        self._publish_zero_twist()
        _x, _y, yaw = pose
        self._wiggle_ref_yaw = yaw
        # Fan-scan schedule: alternating -delta,+delta,-2*delta,+2*delta,...
        # 'left wheels forward' first (negative offset == turn right, see
        # _tick_wiggle_seek), then 'right wheels forward' (positive
        # offset), each cycle stepping wiggle_step_m further out, capped at
        # +/- wiggle_max_sweep_deg either side.
        delta = self._wiggle_step_m / self._track_sep
        targets: list[float] = []
        n = 1
        while n * delta <= self._wiggle_max_sweep_rad + 1e-6:
            targets.append(-n * delta)
            targets.append(n * delta)
            n += 1
        self._wiggle_targets = targets
        self._state = _State.WIGGLE_SEEK
        self._set_status(
            f'straight seek failed — wiggle scan up to '
            f'+/-{math.degrees(self._wiggle_max_sweep_rad):.0f} deg')
        self._advance_wiggle_target()

    def _advance_wiggle_target(self) -> None:
        if not self._wiggle_targets:
            # Full fan sweep exhausted on both sides without reacquiring —
            # NOW treat this as the field boundary, not before.
            self._publish_zero_twist()
            self._safe_disable()
            self._state = _State.COMPLETE
            self._set_status(
                f'no row found within +/-'
                f'{math.degrees(self._wiggle_max_sweep_rad):.0f} deg wiggle '
                'scan — assuming field boundary reached; discovery complete')
            self._write_map(final=True)
            return
        offset = self._wiggle_targets.pop(0)
        self._wiggle_target_yaw = _wrap_pi(self._wiggle_ref_yaw + offset)
        self._wiggle_side = 'left' if offset < 0 else 'right'

    def _on_row_reacquired(self) -> None:
        self._publish_zero_twist()
        self._turn_right = not self._turn_right   # alternate next time
        self._row_index += 1
        self._drop_row_start_node()
        self._enter_follow_row()

    # ------------------------------------------------------------------
    # Main tick — drives the whole state machine at monitor_rate_hz.
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        if self._state == _State.IDLE:
            return

        if self._state == _State.FOLLOW_ROW:
            self._tick_follow_row()
        elif self._state == _State.HEADLAND_CLEARANCE:
            self._tick_headland_clearance()
        elif self._state == _State.TURN_1:
            self._tick_turn(next_state=self._enter_lateral_step)
        elif self._state == _State.LATERAL_STEP:
            self._tick_lateral_step()
        elif self._state == _State.TURN_2:
            self._tick_turn(next_state=self._enter_seek_row)
        elif self._state == _State.SEEK_ROW:
            self._tick_seek_row()
        elif self._state == _State.WIGGLE_SEEK:
            self._tick_wiggle_seek()
        # COMPLETE / FAULT: idle, nothing to drive.

    def _tick_follow_row(self) -> None:
        # Neo is enabled and doing its own /cmd_vel — we only watch.
        if (not self._heartbeat_alive
                and self._heartbeat_state_duration() >= self._row_lost_confirm_s):
            # Debounced: heartbeat has been false continuously for the
            # confirm window, not just one bad frame — treat as row end.
            self._drop_row_end_node()
            self._enter_headland_clearance()

    def _tick_headland_clearance(self) -> None:
        pose = self._pose()
        if pose is None:
            self._publish_zero_twist()
            return
        x, y, _yaw = pose
        sx, sy, _ = self._maneuver_start_pose
        travelled = math.hypot(x - sx, y - sy)
        if travelled >= self._headland_clearance:
            self._publish_zero_twist()
            self._enter_turn_1()
            return
        t = Twist()
        t.linear.x = self._creep_vel
        self._cmd_vel_pub.publish(t)

    def _tick_turn(self, next_state) -> None:
        pose = self._pose()
        if pose is None:
            self._publish_zero_twist()
            return
        _x, _y, yaw = pose
        err = _wrap_pi(self._maneuver_target_yaw - yaw)
        if abs(err) <= math.radians(3.0):
            self._publish_zero_twist()
            next_state()
            return
        t = Twist()
        t.angular.z = math.copysign(self._turn_speed, err)
        self._cmd_vel_pub.publish(t)

    def _tick_lateral_step(self) -> None:
        pose = self._pose()
        if pose is None:
            self._publish_zero_twist()
            return
        x, y, _yaw = pose
        sx, sy, _ = self._maneuver_start_pose
        travelled = math.hypot(x - sx, y - sy)
        if travelled >= self._lateral_step:
            self._publish_zero_twist()
            self._enter_turn_2()
            return
        t = Twist()
        t.linear.x = self._creep_vel
        self._cmd_vel_pub.publish(t)

    def _tick_seek_row(self) -> None:
        pose = self._pose()
        if pose is None:
            self._publish_zero_twist()
            return
        x, y, _yaw = pose
        sx, sy, _ = self._seek_start_pose
        travelled = math.hypot(x - sx, y - sy)

        if (self._heartbeat_alive
                and self._heartbeat_state_duration() >= self._row_reacquire_confirm_s):
            self._on_row_reacquired()
            return

        if travelled >= self._max_search_m:
            # Straight creep alone didn't find it — try the fan-scan wiggle
            # before giving up (this used to go straight to COMPLETE).
            self._enter_wiggle_seek()
            return

        t = Twist()
        t.linear.x = self._creep_vel
        self._cmd_vel_pub.publish(t)

    def _tick_wiggle_seek(self) -> None:
        if (self._heartbeat_alive
                and self._heartbeat_state_duration() >= self._row_reacquire_confirm_s):
            self._on_row_reacquired()
            return

        pose = self._pose()
        if pose is None:
            self._publish_zero_twist()
            return
        _x, _y, yaw = pose
        err = _wrap_pi(self._wiggle_target_yaw - yaw)
        if abs(err) <= math.radians(2.0):
            self._publish_zero_twist()
            self._advance_wiggle_target()
            return

        # Single-track-forward kinematics (the other track stationary):
        # v = v_side / 2, w = ∓v_side / track_sep. 'left' track forward
        # pivots right (negative angular.z); 'right' track forward pivots
        # left (positive angular.z) — matches the sign convention used to
        # build the target schedule in _enter_wiggle_seek.
        t = Twist()
        t.linear.x = self._wiggle_speed_mps / 2.0
        w = self._wiggle_speed_mps / self._track_sep
        t.angular.z = -w if self._wiggle_side == 'left' else w
        self._cmd_vel_pub.publish(t)

    # ------------------------------------------------------------------
    # Topo node dropping — same schema as ui_node.py's drop_topo_node, so
    # a plain topological_navigation load of the resulting map behaves
    # identically to one built via the F2C web-UI path. Row edges use
    # _ROW_ACTION ('limbic_row_follow') exactly like a pre-planned map;
    # headland edges (row-exit -> next row-entry) use _NAV_ACTION
    # ('navigate_to_pose') since that's a short, unremarkable point-to-
    # point hop through open headland, same as existing headland edges.
    # ------------------------------------------------------------------

    def _drop_origin_node(self) -> None:
        # Dropped once, at start_discovery, at the robot's actual pose
        # before any motion. connect_to is None here (it's the first node
        # ever dropped this run) so no edge is created yet — the edge FROM
        # this node onward is added automatically when _drop_row_end_node
        # for row 1 is later dropped (edge_action=_ROW_ACTION), same as
        # every other row-to-row link.
        name = f'{self._name_prefix}{self._row_index}_IN'
        self._drop_node(name, row_id=self._row_index, row_role='entry',
                         edge_action=_ROW_ACTION)

    def _drop_row_end_node(self) -> None:
        name = f'{self._name_prefix}{self._row_index}_OUT'
        self._drop_node(name, row_id=self._row_index, row_role='exit',
                         edge_action=_ROW_ACTION)

    def _drop_row_start_node(self) -> None:
        name = f'{self._name_prefix}{self._row_index}_IN'
        # Headland edge connects the PREVIOUS row's exit to this new
        # row's entry — that's the maneuver we just physically drove.
        self._drop_node(name, row_id=self._row_index, row_role='entry',
                         edge_action=_NAV_ACTION)

    def _drop_node(self, name: str, row_id: int, row_role: str,
                    edge_action: str) -> None:
        pose = self._pose()
        if pose is None:
            self.get_logger().error(
                f'row_discovery: no TF — cannot drop node {name}')
            return
        x, y, _yaw = pose
        connect_to = self._last_node_name
        timestamp = datetime.now(timezone.utc).strftime('%d-%m-%Y_%H-%M-%S')
        xy_tol, yaw_tol, vert_r = (
            (0.1, 0.05, 0.5) if edge_action == _ROW_ACTION else (0.3, 0.1, 1.0))

        node_meta = {'map': self._map_name, 'node': name,
                     'pointset': self._map_name, 'dropped_by': 'row_discovery',
                     'timestamp': timestamp, 'row_id': row_id,
                     'row_role': row_role}
        node_props = {'xy_goal_tolerance': xy_tol, 'yaw_goal_tolerance': yaw_tol,
                       'dropped_by': 'row_discovery', 'timestamp': timestamp,
                       'row_id': row_id, 'row_role': row_role}

        node_dict = {
            'meta': node_meta,
            'node': {
                'edges': ([{'action': edge_action,
                            'edge_id': f'{name}_{connect_to}',
                            'node': connect_to}] if connect_to else []),
                'name': name,
                'nav_frame': self._map_frame,
                'pose': {'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
                         'position': {'x': round(x, 3), 'y': round(y, 3), 'z': 0.0}},
                'properties': node_props,
                'verts': [{'x': -vert_r, 'y': -vert_r}, {'x': vert_r, 'y': -vert_r},
                          {'x': vert_r, 'y': vert_r}, {'x': -vert_r, 'y': vert_r}],
            },
        }

        self._append_and_write(node_dict, connect_to, name, edge_action)
        self._last_node_name = name
        self._set_status(f'dropped {name}' + (f' <- {connect_to}' if connect_to else ''))

    # ------------------------------------------------------------------
    # File I/O — same load/append/write/switch pattern as ui_node.py's
    # _publish_and_persist, kept synchronous here (called from the ROS
    # timer thread, off the hot control loop, so a slow disk write just
    # delays the next tick slightly rather than corrupting state).
    # ------------------------------------------------------------------

    def _load_or_init_doc(self) -> dict:
        map_file = os.path.join(self._maps_dir, self._map_name)
        if os.path.exists(map_file):
            with open(map_file, encoding='utf-8') as f:
                return yaml.safe_load(f) or {}
        # Fresh map: minimal header matching the schema topological_
        # navigation expects (see maize_map.yaml for the reference shape).
        return {
            'meta': {'last_updated': ''},
            'name': self._map_name,
            'metric_map': self._map_name,
            'pointset': self._map_name,
            'transformation': {
                'topo_frame_id': self._map_frame, 'child': 'topo_map',
                'parent': self._map_frame,
                'rotation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
                'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            },
            'nodes': [],
        }

    def _append_and_write(self, node_dict: dict, connect_to: str | None,
                           name: str, edge_action: str) -> None:
        map_file = os.path.join(self._maps_dir, self._map_name)
        try:
            os.makedirs(self._maps_dir, exist_ok=True)
            doc = self._topo_doc or self._load_or_init_doc()
            existing = {e.get('node', {}).get('name')
                        for e in doc.get('nodes', [])}
            if name in existing:
                self.get_logger().warn(f'{name} already in map — skipping')
                return

            doc.setdefault('meta', {})['last_updated'] = (
                datetime.now(timezone.utc).strftime('%d-%m-%Y_%H-%M-%S'))
            doc.setdefault('nodes', []).append(copy.deepcopy(node_dict))

            if connect_to:
                for entry in doc.get('nodes', []):
                    n = entry.get('node', {})
                    if n.get('name') == connect_to:
                        n.setdefault('edges', []).append(
                            {'action': edge_action,
                             'edge_id': f'{connect_to}_{name}', 'node': name})
                        break

            with open(map_file, 'w', encoding='utf-8') as f:
                yaml.dump(doc, f, default_flow_style=False,
                          allow_unicode=True, sort_keys=False)
            self._topo_doc = doc
            self._reload_live_map(map_file)
        except OSError as e:
            self.get_logger().error(f'row_discovery: failed writing map: {e}')

    def _reload_live_map(self, map_file: str) -> None:
        if not _TOPO_SRV_OK or self._switch_map_cli is None:
            return
        if not self._switch_map_cli.wait_for_service(timeout_sec=1.0):
            return
        req = WriteTopologicalMap.Request()
        req.filename = map_file
        req.no_alias = True

        ev = threading.Event()

        def _cb(fut):
            ev.set()

        self._switch_map_cli.call_async(req).add_done_callback(_cb)
        ev.wait(timeout=3.0)

    def _write_map(self, final: bool = False) -> None:
        # Final write is really just "make sure last_updated reflects
        # completion" — the incremental writes in _append_and_write already
        # persist every node as it's dropped, so discovery is safe to stop
        # (or crash) at any point without losing progress.
        if not self._topo_doc:
            return
        map_file = os.path.join(self._maps_dir, self._map_name)
        self._topo_doc.setdefault('meta', {})['last_updated'] = (
            datetime.now(timezone.utc).strftime('%d-%m-%Y_%H-%M-%S'))
        with open(map_file, 'w', encoding='utf-8') as f:
            yaml.dump(self._topo_doc, f, default_flow_style=False,
                      allow_unicode=True, sort_keys=False)


def main(args=None):
    rclpy.init(args=args)
    node = RowDiscoveryNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
