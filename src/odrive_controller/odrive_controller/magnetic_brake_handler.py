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
        self._magnetic_brake_gpio_pin = lgpio.gpio_claim_output(self._gpio_controller, MAGNETIC_BRAKE_PIN)
        self._is_brake_released = lgpio.gpio_read(self._gpio_controller, self._magnetic_brake_gpio_pin)

        node.create_service(SetBool, "/brake/set_state", self._set_brake_state)
        self._brake_state_publisher = node.create_publisher(Bool, "/brake/state", DEFAULT_QOS_PROFILE)
        node.create_timer(BRAKE_STATE_PUBLISH_INTERVAL_SECONDS, self._publish_brake_state)


    def _publish_brake_state(self):
        self._brake_state_publisher.publish(Bool(data=self._is_brake_released))
        

    def _set_brake_state(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        if self._is_brake_released != req.data:
            error_code = lgpio.gpio_write(self._gpio_controller, self._magnetic_brake_gpio_pin, int(req.data))
            if not error_code:
                self._is_brake_released = req.data
                self._publish_brake_state()
                     
        res.data = self._is_brake_released
        return res
            