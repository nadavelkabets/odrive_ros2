from motor_controller_interface import MotorControllerInterface
from odrive_can.msg import ControllerStatus, ControlMessage, ODriveStatus
from odrive_can.srv import AxisState as AxisStateService
from odrive.enums import AxisState, ControlMode, InputMode
from dataclasses import dataclass
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from motor_controller_utils import rps_to_rpm, rpm_to_rps

DEFAULT_QOS = 10

@dataclass
class OdriveTelemetry:
    def __init__(self):
        self.position_cycles = 0.0
        self.velocity_rpm = 0.0
        self.torque_target = 0.0
        self.torque_estimate = 0.0
        self.iq_setpoint = 0.0
        self.iq_measured = 0.0
        self.active_errors = 0
        self.axis_state = 0
        self.procedure_result = 0
        self.trajectory_done_flag = False
        self.bus_voltage = 0.0
        self.bus_current = 0.0
        self.fet_temperature = 0.0
        self.motor_temperature = 0.0
        self.active_errors = 0
        self.disarm_reason = 0


class OdriveController(MotorControllerInterface):
    def __init__(self, node: Node):
        self.node = node
        self.telemetry = OdriveTelemetry()
        
        self._odrive_status_subscriber = self.node.create_subscription(ODriveStatus, "odrive_status", self._odrive_status_subscriber_callback, DEFAULT_QOS, callback_group=ReentrantCallbackGroup())
        self._controller_status_subscriber = self.node.create_subscription(ControllerStatus, "controller_status", self._controller_status_subscriber_callback, DEFAULT_QOS, callback_group=ReentrantCallbackGroup())
        self._control_message_publisher = self.node.create_publisher(ControlMessage, "control_message", DEFAULT_QOS)
        self._axis_state_service_client = self.node.create_client(AxisStateService, "request_axis_state", callback_group=ReentrantCallbackGroup())

    async def roll_position(self, position_cycles: float):
        # set odrive to closed loop control
        axis_state_request_result = await self._request_axis_state(AxisState.CLOSED_LOOP_CONTROL)
        if axis_state_service_response.

    def roll_velocity(self, velocity_rpm: float):
        pass

    def roll_duty(self, duty_cycle: float):
        pass

    async def calibrate(self):
        await self._request_axis_state(AxisState.IDLE)
        # return if successful or not

    async def abort(self):
        # TODO: if currently idle, ignore. if in action - send abort
        await self._reach_complete_stop()

    
    def reset_cycles(position_cycles: float):
        pass

    def set_brake_state(is_brake_locked: bool):
        pass

    async def _reach_complete_stop(self):
        self._publish_velocity_command(velocity_rpm=0)
        #TODO: add a slow to stop timeout that sends the command again if there is no deceleration or not reaching stop in allowed time
        while True:
            await self._async_ros_sleep(0.1)
            if abs(self.telemetry.velocity_rpm) < 10: # or (time.time() - start_time) < operation timeout
                break
        self.set_brake_state(is_brake_locked=True)
        await self._request_axis_state(AxisState.IDLE)

    def _async_ros_sleep(self, time_seconds: float) -> Future:
        self._sleep_future = Future()
        self._end_sleep_timer = self.node.create_timer(time_seconds, self._end_sleep_timer_callback)
        return self._sleep_future

    def _end_ros_sleep_timer_callback(self):
        self._sleep_future.done()
        self._end_sleep_timer.cancel()

    async def _request_axis_state(self, state: AxisState) -> AxisStateService.Response:
        # TODO: add timeout
        axis_state_service_request = AxisStateService.Request(axis_requested_state=AxisState.CLOSED_LOOP_CONTROL)
        return await self._axis_state_service_client.call_async(axis_state_service_request)
    
    def _publish_velocity_command(self, velocity_rpm: float):
        velocity_command = ControlMessage(control_mode = ControlMode.VELOCITY_CONTROL, input_mode = InputMode.VEL_RAMP)
        velocity_command.input_vel = velocity_rpm
        self._control_message_publisher.publish(velocity_command)

    def _odrive_status_subscriber_callback(self, msg: ODriveStatus):
        self.telemetry.bus_voltage = msg.bus_voltage
        self.telemetry.bus_current = msg.bus_current
        self.telemetry.fet_temperature = msg.fet_temperature
        self.telemetry.motor_temperature = msg.motor_temperature
        self.telemetry.active_errors = msg.active_errors
        self.telemetry.disarm_reason = msg.disarm_reason

    def _controller_status_subscriber_callback(self, msg: ControllerStatus):
        self.telemetry.position_cycles = msg.pos_estimate
        self.telemetry.velocity_rpm = rps_to_rpm(msg.vel_estimate)
        self.telemetry.torque_target = msg.torque_target
        self.telemetry.torque_estimate = msg.torque_estimate
        self.telemetry.iq_setpoint = msg.iq_setpoint
        self.telemetry.iq_measured = msg.iq_measured
        self.telemetry.active_errors = msg.active_errors
        self.telemetry.axis_state = msg.axis_state
        self.telemetry.procedure_result = msg.procedure_result
        self.telemetry.trajectory_done_flag = msg.trajectory_done_flag

