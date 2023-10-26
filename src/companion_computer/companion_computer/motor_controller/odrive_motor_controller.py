from motor_controller_interface import MotorControllerInterface

class OdriveController(MotorControllerInterface):
    async def roll_position(position_cycles: float):
        pass

    def roll_velocity(velocity_rpm: float):
        pass

    def roll_duty(duty_cycle: float):
        pass

    def calibrate():
        pass

    def abort():
        pass
    
    def reset_cycles(position_cycles: float):
        pass

    def set_brake_state(is_brake_locked: bool):
        pass