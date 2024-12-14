from zipfile import error

from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
import lgpio

MAGNETIC_BRAKE_PIN = 22
BRAKE_STATE_PUBLISH_INTERVAL_SECONDS = 1
GPIO_CONTROLLER_ID = 0
DEFAULT_QOS_PROFILE = 10


class MagneticBrakeHandler:
    def __init__(self, node: Node):
        self._gpio_controller = lgpio.gpiochip_open(GPIO_CONTROLLER_ID)
        self._magnetic_brake_gpio = lgpio.gpio_claim_output(self._gpio_controller, MAGNETIC_BRAKE_PIN)
        self._update_brake_state()

        node.create_service(SetBool, "/brake/set_state", self._set_brake_state_callback)
        self._brake_state_publisher = node.create_publisher(Bool, "/brake/state", DEFAULT_QOS_PROFILE)
        node.create_timer(BRAKE_STATE_PUBLISH_INTERVAL_SECONDS, self._publish_brake_state)

    def _publish_brake_state(self):
        self._update_brake_state()
        self._brake_state_publisher.publish(Bool(data=self._is_brake_released))

    def _set_brake_state_callback(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self._update_brake_state()
        if self._is_brake_released != req.data :
            error_code = self._set_brake_state(req.data)
            if not error_code:
                res.success = True
                res.message = f"Changed brake state to {'released' if self._is_brake_released else 'locked'}"
            else:
                res.success = False
                res.message = f"Failed to set brake state to {'released' if self._is_brake_released else 'locked'}: {str(error_code)}"

        return res

    def _update_brake_state(self):
        self._is_brake_released = bool(lgpio.gpio_read(self._gpio_controller, self._magnetic_brake_gpio))

    def _set_brake_state(self, state: bool):
        error_code = lgpio.gpio_write(self._gpio_controller, self._magnetic_brake_gpio, int(state))
        self._publish_brake_state()
        return error_code