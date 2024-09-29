from croblink import *
from pid_controller import PIDController
import time

TIME_STEP = 0.001 # Sampling time --> TODO
MAX_POW = 1000.15 #lPow rPow max velocity value 
MIN_POW = -0.15 #lPow rPow min velocity value 

# Speed PID Controller values
KP = 0.008 # TODO 0.01
KI = 0 # TODO
KD = 0 # TODO

# Steering PID Controller Values
KPs = 0.09 # 0.2 
KIs = 0 # TODO
KDs = 0 # TODO

class Robot(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.speed_pid_controller = PIDController(kp=KP, ki=KI, kd=KD, time_step=TIME_STEP, max_output=MAX_POW)
        self.steering_pid_controller = PIDController(kp=KPs, ki=KIs, kd=KDs, time_step=TIME_STEP, max_output=MAX_POW)
        self.speed_setpoint = 0.7 # Slow down as the center sensor increases. 
        self.steering_setpoint = 0


    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        while True:
            self.readSensors()

            # Get the IR sensor values (left and right)
            center_sensor = self.measures.irSensor[0] # Center sensor
            left_sensor = self.measures.irSensor[1]  # Left sensor
            right_sensor = self.measures.irSensor[2]  # Right sensor

            # Calculate the error as the difference between the left and right sensors
            error = left_sensor - right_sensor  # Positive error means closer to the right, negative closer to the left

            # Compute control signal if error is significant
            speed_control_signal = self.speed_pid_controller.compute(center_sensor, self.speed_setpoint)
            steering_control_signal = self.steering_pid_controller.compute(error, self.steering_setpoint)
            self.adjust_motors(speed_control_signal, steering_control_signal)

            self.printObstacleSensors()
            print("\n----------------------------------------\n")
    
            time.sleep(TIME_STEP)


    def adjust_motors(self, speed_control_signal, steering_control_signal):
        # Control the motors based on PID output
        base_speed = min(0.15, 0.15 + speed_control_signal)
        
        left_motor_power = max(MIN_POW, min(base_speed, base_speed - steering_control_signal))  # Reduce power to left motor for right turn
        right_motor_power = max(MIN_POW, min(base_speed, base_speed + steering_control_signal))  # Reduce power to right motor for left turn

        print(f"Speed Control Signal: {speed_control_signal}")
        print(f"Steering Control Signal: {steering_control_signal}")
        print(f"lPow rPow: ({round(left_motor_power, 2)}, {round(right_motor_power, 2)})")
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
