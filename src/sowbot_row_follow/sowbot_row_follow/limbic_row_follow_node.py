#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2026 Agroecology Lab Ltd
# BSD-2-Clause License

# limbic_row_follow_node.py
# =========================
# ROS 2 action server on Limbic that orchestrates visual crop-row following.
#
# Accepts:  nav2_msgs/NavigateToPose
# Requires: /row_follow/enable  (std_srvs/SetBool)  — Neo's crop_row_node
#           /aoc/heartbeat/neo_vision (std_msgs/Bool) — Neo perception health
#           /odometry/global  (nav_msgs/Odometry)     — robot pose
#           /navigate_to_pose (nav2_msgs/NavigateToPose action) — nav2 for Phase 2
#
# Phase 1: Enable Neo, visual servo drives /cmd_vel until within
#          row_follow_handover_distance of goal pose.
# Phase 2: Disable Neo (acknowledged), then hand off to nav2 navigate_to_pose
#          for precise final approach and heading alignment.
#
# Phase 0 (align, runs before Phase 1): topological_navigation's preceding
# edge only guarantees loose xy/yaw tolerance at the row endpoint (fine for
# headland travel, not tight enough for TSM line-fitting to find the row on
# its first frame). Before enabling Neo, rotate in place to face goal_pose
# (the opposite row endpoint) within row_entry_align_tolerance_deg. Goal_pose
# is always "the other end of this row" regardless of which direction the
# edge is traversed, so this is symmetric for both IN->OUT and OUT->IN without
# needing to know row_id/role — it only uses the goal already passed in.
#
# Cancel / abort: Neo is always disabled before the cancel is accepted.
# Heartbeat loss (>heartbeat_timeout_s): abort with Neo disabled.
# enable() service timeout: treated as hard fault — abort immediately.

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _distance_2d(pose_a: PoseStamped, pose_b: PoseStamped) -> float:
    """Euclidean XY distance between two PoseStamped messages."""
    dx = pose_a.pose.position.x - pose_b.pose.position.x
    dy = pose_a.pose.position.y - pose_b.pose.position.y
    return math.sqrt(dx * dx + dy * dy)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class LimbicRowFollow(Node):

    def __init__(self):
        super().__init__("limbic_row_follow")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("row_follow_handover_distance", 1.0)
        self.declare_parameter("heartbeat_timeout_s", 3.0)
        self.declare_parameter("enable_service_timeout_s", 2.0)
        self.declare_parameter("monitor_rate_hz", 10.0)
        # Phase 0 (align) — see docstring at top and _align_to_goal below.
        self.declare_parameter("row_entry_align_tolerance_deg", 8.0)
        self.declare_parameter("row_entry_align_timeout_s", 15.0)
        self.declare_parameter("row_entry_align_angular_speed", 0.3)

        self._handover_dist: float = (
            self.get_parameter("row_follow_handover_distance").value)
        self._hb_timeout: float = (
            self.get_parameter("heartbeat_timeout_s").value)
        self._enable_timeout: float = (
            self.get_parameter("enable_service_timeout_s").value)
        monitor_rate: float = self.get_parameter("monitor_rate_hz").value

        # ReentrantCallbackGroup allows the monitor timer to fire while the
        # action callback is awaiting a service or sub-action future.
        self._cbg = ReentrantCallbackGroup()

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._current_odom: Odometry | None = None
        self._last_heartbeat_time: rclpy.time.Time | None = None
        self._heartbeat_alive: bool = False

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(
            Odometry, "/odometry/global", self._on_odom, 10,
            callback_group=self._cbg)
        self.create_subscription(
            Bool, "/aoc/heartbeat/neo_vision", self._on_heartbeat, 10,
            callback_group=self._cbg)

        # ------------------------------------------------------------------
        # Service client — Neo enable/disable
        # ------------------------------------------------------------------
        self._enable_client = self.create_client(
            SetBool, "/row_follow/enable", callback_group=self._cbg)

        # Align-phase (Phase 0) drives directly — safe because Neo/visual
        # servo is not enabled yet at that point, so there's no conflicting
        # publisher on this topic during that window.
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ------------------------------------------------------------------
        # Nav2 action client — Phase 2 final approach
        # ------------------------------------------------------------------
        self._nav2_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose",
            callback_group=self._cbg)

        # ------------------------------------------------------------------
        # Action server
        # ------------------------------------------------------------------
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            "/limbic_row_follow",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._cbg,
        )

        # ------------------------------------------------------------------
        # Heartbeat staleness monitor timer
        # ------------------------------------------------------------------
        self._monitor_timer = self.create_timer(
            1.0 / monitor_rate, self._check_heartbeat_staleness,
            callback_group=self._cbg)

        self.get_logger().info("limbic_row_follow action server ready")

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        self._current_odom = msg

    def _on_heartbeat(self, msg: Bool) -> None:
        self._heartbeat_alive = msg.data
        if msg.data:
            self._last_heartbeat_time = self.get_clock().now()

    def _check_heartbeat_staleness(self) -> None:
        """Mark heartbeat dead if no True message received within timeout."""
        if self._last_heartbeat_time is None:
            return
        elapsed = (self.get_clock().now() - self._last_heartbeat_time).nanoseconds * 1e-9
        if elapsed > self._hb_timeout:
            self._heartbeat_alive = False

    # ------------------------------------------------------------------
    # Action server callbacks
    # ------------------------------------------------------------------

    def _on_goal(self, goal_handle) -> GoalResponse:
        self.get_logger().info("limbic_row_follow: goal received")
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info("limbic_row_follow: cancel requested")
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    # Execute
    # ------------------------------------------------------------------

    def _execute(self, goal_handle) -> NavigateToPose.Result:
        goal_pose: PoseStamped = goal_handle.request.pose
        result = NavigateToPose.Result()

        self.get_logger().info("limbic_row_follow: Phase 0 — aligning to row heading")
        if not self._align_to_goal(goal_handle, goal_pose):
            # Helper already called abort()/canceled() and zeroed cmd_vel.
            return result

        self.get_logger().info("limbic_row_follow: starting Phase 1 — visual servo")

        # ------------------------------------------------------------------
        # Phase 1: enable Neo, monitor until within handover distance
        # ------------------------------------------------------------------
        if not self._call_enable(True):
            self.get_logger().error(
                "limbic_row_follow: enable service call failed — aborting")
            goal_handle.abort()
            return result

        rate = self.create_rate(self.get_parameter("monitor_rate_hz").value)

        while rclpy.ok():
            # Cancel requested
            if goal_handle.is_cancel_requested:
                self.get_logger().info(
                    "limbic_row_follow: cancel received during Phase 1")
                self._safe_disable()
                goal_handle.canceled()
                return result

            # Heartbeat lost
            if not self._heartbeat_alive:
                self.get_logger().error(
                    "limbic_row_follow: Neo heartbeat lost — aborting")
                self._safe_disable()
                goal_handle.abort()
                return result

            # Check distance to goal
            if self._current_odom is not None:
                robot_pose = PoseStamped()
                robot_pose.pose = self._current_odom.pose.pose
                dist = _distance_2d(robot_pose, goal_pose)

                goal_handle.publish_feedback(
                    NavigateToPose.Feedback(current_pose=robot_pose,
                                            distance_remaining=dist))

                if dist <= self._handover_dist:
                    self.get_logger().info(
                        "limbic_row_follow: within %.2fm — handing over to nav2"
                        % dist)
                    break

            rate.sleep()

        # ------------------------------------------------------------------
        # Phase 2: disable Neo (acknowledged), then nav2 final approach
        # ------------------------------------------------------------------
        self.get_logger().info("limbic_row_follow: Phase 2 — nav2 final approach")

        if not self._safe_disable():
            # Neo unresponsive — escalate
            self.get_logger().error(
                "limbic_row_follow: disable timed out — hard fault, aborting")
            goal_handle.abort()
            return result

        # Check for cancel that arrived during the disable call
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return result

        nav2_result = self._call_nav2(goal_handle, goal_pose)

        if nav2_result:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _current_yaw(self) -> float | None:
        """Robot yaw (rad) from /odometry/global, or None if no odom yet."""
        if self._current_odom is None:
            return None
        q = self._current_odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _yaw_error(target: float, current: float) -> float:
        """Signed shortest angular distance target-current, wrapped to [-pi, pi]."""
        e = target - current
        return math.atan2(math.sin(e), math.cos(e))

    def _publish_zero_twist(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _align_to_goal(self, goal_handle, goal_pose: PoseStamped) -> bool:
        """
        Phase 0: rotate in place to face goal_pose before enabling Neo.

        Safe to drive /cmd_vel directly here — Neo/visual servo has not
        been enabled yet, so nothing else is publishing on this topic for
        this goal. Returns True once aligned within tolerance. On
        cancel/timeout/no-odom it calls the appropriate terminal
        goal_handle method itself (mirroring the Phase 1/2 pattern) and
        returns False — callers should just `return result` in that case.
        """
        tol = math.radians(
            self.get_parameter("row_entry_align_tolerance_deg").value)
        timeout_s = self.get_parameter("row_entry_align_timeout_s").value
        ang_speed = self.get_parameter("row_entry_align_angular_speed").value

        # Wait briefly for odom if the action started before any arrived.
        wait_start = self.get_clock().now()
        wait_timeout = Duration(seconds=5.0)
        while self._current_odom is None and rclpy.ok():
            if self.get_clock().now() - wait_start > wait_timeout:
                self.get_logger().error(
                    "limbic_row_follow: no odom before align phase — aborting")
                goal_handle.abort()
                return False
            time.sleep(0.05)

        cur = self._current_odom.pose.pose.position
        dx = goal_pose.pose.position.x - cur.x
        dy = goal_pose.pose.position.y - cur.y
        if math.hypot(dx, dy) < 1e-3:
            # Already essentially at the goal position — nothing meaningful
            # to align to (shouldn't normally happen for a row edge).
            return True
        target_yaw = math.atan2(dy, dx)

        rate = self.create_rate(self.get_parameter("monitor_rate_hz").value)
        align_start = self.get_clock().now()
        align_timeout = Duration(seconds=timeout_s)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info(
                    "limbic_row_follow: cancel received during Phase 0")
                self._publish_zero_twist()
                goal_handle.canceled()
                return False

            yaw = self._current_yaw()
            if yaw is None:
                self._publish_zero_twist()
                goal_handle.abort()
                return False

            err = self._yaw_error(target_yaw, yaw)
            if abs(err) <= tol:
                self._publish_zero_twist()
                self.get_logger().info(
                    "limbic_row_follow: aligned (err=%.1f deg)"
                    % math.degrees(err))
                return True

            if self.get_clock().now() - align_start > align_timeout:
                self.get_logger().error(
                    "limbic_row_follow: align phase timed out "
                    "(err=%.1f deg) — aborting" % math.degrees(err))
                self._publish_zero_twist()
                goal_handle.abort()
                return False

            twist = Twist()
            twist.angular.z = math.copysign(ang_speed, err)
            self._cmd_vel_pub.publish(twist)
            rate.sleep()

        self._publish_zero_twist()
        return False

    def _call_enable(self, enable: bool) -> bool:
        """
        Call /row_follow/enable synchronously.
        Returns True on success, False on timeout or service error.
        """
        if not self._enable_client.wait_for_service(
                timeout_sec=self._enable_timeout):
            self.get_logger().error(
                "limbic_row_follow: /row_follow/enable service not available")
            return False

        req = SetBool.Request()
        req.data = enable
        future = self._enable_client.call_async(req)

        # Spin until done or timeout
        start = self.get_clock().now()
        timeout = Duration(seconds=self._enable_timeout)
        while not future.done():
            if self.get_clock().now() - start > timeout:
                self.get_logger().error(
                    "limbic_row_follow: /row_follow/enable timed out")
                return False
            time.sleep(0.05)

        response: SetBool.Response = future.result()
        if not response.success:
            self.get_logger().error(
                "limbic_row_follow: enable service returned failure: %s"
                % response.message)
            return False

        return True

    def _safe_disable(self) -> bool:
        """
        Disable Neo. Always called before cancel or abort.
        Returns True if acknowledged, False if timed out (hard fault).
        """
        return self._call_enable(False)

    def _call_nav2(self, goal_handle, goal_pose: PoseStamped) -> bool:
        """
        Send goal to nav2 /navigate_to_pose and block until complete.
        Returns True on success, False on failure or cancel.
        """
        if not self._nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "limbic_row_follow: nav2 /navigate_to_pose not available")
            return False

        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = goal_pose

        send_future = self._nav2_client.send_goal_async(nav2_goal)

        # Wait for goal acceptance (executor is already spinning this node
        # on other threads via the ReentrantCallbackGroup, so just poll)
        start = self.get_clock().now()
        accept_timeout = Duration(seconds=5.0)
        while not send_future.done():
            if self.get_clock().now() - start > accept_timeout:
                self.get_logger().error(
                    "limbic_row_follow: nav2 goal acceptance timed out")
                return False
            time.sleep(0.05)
        nav2_handle = send_future.result()

        if not nav2_handle or not nav2_handle.accepted:
            self.get_logger().error(
                "limbic_row_follow: nav2 rejected goal")
            return False

        result_future = nav2_handle.get_result_async()

        # Poll until nav2 finishes, forwarding cancel if requested
        rate = self.create_rate(self.get_parameter("monitor_rate_hz").value)
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().info(
                    "limbic_row_follow: cancel during Phase 2 — cancelling nav2")
                nav2_handle.cancel_goal_async()
                goal_handle.canceled()
                return False
            rate.sleep()

        status = result_future.result().status
        return status == GoalStatus.STATUS_SUCCEEDED


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = LimbicRowFollow()
    # Explicit thread count: the sleep-poll loops in _call_enable/_call_nav2
    # occupy one executor thread each while waiting, so a too-small default
    # (e.g. num_threads=os.cpu_count() on a low-core SBC) can starve the
    # thread needed to service the pending future's callback and hang forever.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
