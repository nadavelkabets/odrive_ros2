from contextlib import contextmanager
from typing import Optional

from odrive.enums import AxisState, ControlMode, InputMode
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from motor_controller_utils import rpm_to_rps
from odrive_can import srv
from odrive_can.msg import ControllerStatus, ControlMessage, ODriveStatus

DEFAULT_QOS = 10
MAGNETIC_BRAKE_PIN = 22  # TS


# TODO: implement a keepalive like system using status messages
class OdriveController:
    NOMINAL_ODRIVE_STATES = [AxisState.IDLE, AxisState.CLOSED_LOOP_CONTROL]

    def __init__(self, node: Node):
        self._node = node
        self._last_odrive_status: Optional[ODriveStatus] = None
        self._last_controller_status: Optional[ControllerStatus] = None
        self._last_brake_state: Optional[bool] = None

        self._odrive_status_subscriber = self._node.create_subscription(
            ODriveStatus,
            "odrive_status",
            self._handle_odrive_status,
            DEFAULT_QOS
        )
        self._controller_status_subscriber = self._node.create_subscription(
            ControllerStatus,
            "controller_status",
            self._handle_controller_status,
            DEFAULT_QOS
        )
        self._control_message_publisher = self._node.create_publisher(ControlMessage, "control_message", DEFAULT_QOS)
        self._axis_state_service_client = self._node.create_client(srv.AxisState, "request_axis_state")
        self._set_brake_state_client = self._node.create_client(SetBool, "/brake/set_state")
        self._brake_state_subscriber = self._node.create_subscription(
            Bool,
            "/brake/state",
            self._handle_brake_state,
            DEFAULT_QOS
        )

    async def roll_position(self, requested_position_cycles: float):
        # TODO: idea - define a minimum average travel velocity and check if since start_time the actual average velocity is larger
        async with self._closed_loop_control:
            async with self._open_brake:
                self._publish_position_command(requested_position_cycles=requested_position_cycles)
                await self._wait_until_reached_position(requested_position_cycles)

    # TODO - idea: estimate the time needed to reach the requested position and add a timeout
    async def _wait_until_reached_position(self, requested_position_cycles):
        # TODO: move final position margin of error to external variable
        while abs(self._last_controller_status.pos_estimate - requested_position_cycles) > 1:
            await self._async_ros_sleep(0.1)

    async def roll_velocity(self, requested_velocity_rpm: float):
        await self._set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
        await self.set_brake_state(is_brake_locked=False)
        self._publish_velocity_command(requested_velocity_rpm=requested_velocity_rpm)

    async def calibrate(self):
        # TODO: check for errors
        await self._set_axis_state(AxisState.FULL_CALIBRATION_SEQUENCE)
        await self._wait_for_axis_state(requested_state=AxisState.IDLE)

    async def _wait_for_axis_state(self, requested_state):
        while self._last_controller_status.axis_state != requested_state:
            await self._async_ros_sleep(0.1)

    async def abort(self):
        if self._last_controller_status.axis_state == AxisState.IDLE:
            return

        # during calibration, we are comfortable setting idle directly
        if self._last_controller_status.axis_state == AxisState.CLOSED_LOOP_CONTROL:
            await self._abort()

        await self._set_axis_state(AxisState.IDLE)

    async def _abort(self):
        # TODO: add a slow_to_stop_timeout that sends the command again if there is no deceleration or not reaching stop in allowed time
        # TODO: move the minimum brake speed to an external variable
        self._publish_velocity_command(requested_velocity_rpm=0)
        await self._wait_for_velocity(requested_velocity=0)
        await self.set_brake_state(is_brake_locked=True)

    async def _wait_for_velocity(self, requested_velocity: float):
        # TODO: timeout
        while abs(self._last_controller_status.vel_estimate - requested_velocity) < 5:
            await self._async_ros_sleep(0.1)

    async def set_brake_state(self, is_brake_locked: bool):
        set_brake_state_request = SetBool.Request(data=is_brake_locked)
        await self._set_brake_state_client.call_async(set_brake_state_request)

    @contextmanager
    async def _closed_loop_control(self):
        await self._set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
        try:
            yield
        finally:
            await self._set_axis_state(AxisState.IDLE)

    @contextmanager
    async def _open_brake(self):
        # WARNING! YOU MUST USE CLOSED LOOP CONTROL BEFORE BRAKE!
        if self._last_brake_state:
            await self.set_brake_state(is_brake_locked=False)
        try:
            yield
        finally:
            await self.set_brake_state(is_brake_locked=True)

    def _async_ros_sleep(self, time_seconds: float) -> Future:
        # TODO: creating and destroying timers at this rate is very cpu intensive
        self._sleep_future = Future()
        self._end_sleep_timer = self._node.create_timer(time_seconds, self._end_sleep_timer_callback)
        return self._sleep_future

    def _end_sleep_timer_callback(self):
        self._sleep_future.done()
        self._end_sleep_timer.cancel()

    async def _set_axis_state(self, requested_state: AxisState):
        # TODO check for returned axis errors
        # TODO!!! CHECK WHEN THE SERVICE RETURNS??? DOES IT WAIT FOR THE STATE CHANGE TO COMPLETE???
        if self._last_controller_status.axis_state != requested_state:
            await self._request_axis_state(requested_state)
            await self._wait_for_axis_state(requested_state)

    async def _request_axis_state(self, requested_state: AxisState):
        axis_state_service_request = srv.AxisState.Request(axis_requested_state=AxisState.CLOSED_LOOP_CONTROL)
        # TODO: add timeout
        await self._axis_state_service_client.call_async(axis_state_service_request)

    def _publish_velocity_command(self, requested_velocity_rpm: float):
        velocity_command = ControlMessage(control_mode=ControlMode.VELOCITY_CONTROL, input_mode=InputMode.VEL_RAMP)
        velocity_command.input_vel = rpm_to_rps(requested_velocity_rpm)
        self._control_message_publisher.publish(velocity_command)

    def _publish_position_command(self, requested_position_cycles: float):
        position_command = ControlMessage(control_mode=ControlMode.POSITION_CONTROL, input_mode=InputMode.TRAP_TRAJ)
        position_command.input_pos = requested_position_cycles
        self._control_message_publisher.publish(position_command)

    def _handle_odrive_status(self, msg: ODriveStatus):
        self._last_odrive_status = msg

    def _handle_controller_status(self, msg: ControllerStatus):
        self._last_controller_status = msg

    def _handle_brake_state(self, msg: Bool):
        self._last_brake_state = msg.data
