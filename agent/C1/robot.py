from croblink import *
from pid_controller import PIDController
import time

TIME_STEP = 0.001
MAX_VEL = 1000.1 #lPow rPow max velocity value

class Robot(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.pid_controller = PIDController(kp=0.2, ki=0.0, kd=0.0, time_step=TIME_STEP, max_output=MAX_VEL)
        self.setpoint = 0 

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        while True:
            self.readSensors()

            # Get the IR sensor values (left and right)
            left_sensor = self.measures.irSensor[1]  # Left sensor
            right_sensor = self.measures.irSensor[2]  # Right sensor

            # Calculate the error as the difference between the left and right sensors
            error = left_sensor - right_sensor  # Positive error means closer to the right, negative closer to the left

            # Use the PID controller to compute the control signal
            control_signal = self.pid_controller.compute(error, self.setpoint)

            # Adjust the motors based on the control signal
            self.adjust_motors(control_signal)

            self.printObstacleSensors()
            print("\n----------------------------------------\n")

            time.sleep(TIME_STEP)


    def adjust_motors(self, control_signal):
        # Control the motors based on PID output
        base_speed = 0.1  # Maximum forward speed for both motors
        left_motor_power = round(max(0.0, min(base_speed, base_speed - control_signal)), 2)  # Reduce power to left motor for right turn
        right_motor_power = round(max(0.0, min(base_speed, base_speed + control_signal)), 2)  # Reduce power to right motor for left turn
        
        print(control_signal)
        print(left_motor_power, right_motor_power)
        self.driveMotors(left_motor_power, right_motor_power)

    def printObstacleSensors(self):
        """Prints the values from the obstacle sensors."""
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        print(f"Center IR Sensor: {self.measures.irSensor[center_id]}") 
        print(f"Left IR Sensor: {self.measures.irSensor[left_id]}")  
        print(f"Right IR Sensor: {self.measures.irSensor[right_id]}")  
        print(f"Back IR Sensor: {self.measures.irSensor[back_id]}\n")  
