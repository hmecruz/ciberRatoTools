from croblink import *
from pid_controller import PIDController
import time
from noise_filter import NoiseFilter

# MAX / MIN Velocity values 
MAX_POW = 0.15 #lPow rPow max velocity value 
MIN_POW = -0.15 #lPow rPow min velocity value 

TIME_STEP = 0.005 # Sampling time 

# Throttle PID Controller values
KP = 0.002 # 0.002
KI = 0 
KD = 0

# Steering PID Controller Values
KPs = 0.092
KIs = 0
KDs = 0.00085 # 0.0008

class Robot(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

        # PIDController 
        self.speed_pid_controller = PIDController(kp=KP, ki=KI, kd=KD, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PIDController Throttle
        self.steering_pid_controller = PIDController(kp=KPs, ki=KIs, kd=KDs, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PIDController Steering
        
        self.speed_setpoint = 0.8
        self.steering_setpoint = 0

        self.error_threshold = 0.2 # Ignore errors --> Errors can be cause by noise and or noise filter
        
        # Noise Filter
        self.filter_left = NoiseFilter(window_size=2, noise_threshold=0.05)
        self.filter_right = NoiseFilter(window_size=2, noise_threshold=0.05)
        self.filter_center = NoiseFilter(window_size=1, noise_threshold=0.05) # Raw Value

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        while True:

            # Read IR obstacle sensor values 
            self.readSensors()
            center_sensor = self.filter_center.update(self.measures.irSensor[0])
            left_sensor = self.filter_left.update(self.measures.irSensor[1])
            right_sensor = self.filter_right.update(self.measures.irSensor[2])

            # Detect intersection
            #if self.is_intersection(center_sensor, left_sensor, right_sensor):
                #print("Intersection")
                #self.print_obstacle_sensors(center_sensor, left_sensor, right_sensor)

            # Calculate the error as the difference between the left and right sensors
            if abs(left_sensor - right_sensor) <= self.error_threshold and center_sensor <= self.speed_setpoint or self.is_intersection(center_sensor, left_sensor, right_sensor):
                steering_error = 0
            else:
                steering_error = left_sensor - right_sensor # Positive error means closer to the right, negative closer to the left
        
            # Compute control signal if error is significant
            speed_control_signal = self.speed_pid_controller.compute(center_sensor, self.speed_setpoint)
            steering_control_signal = self.steering_pid_controller.compute(steering_error, self.steering_setpoint)
            self.adjust_motors(speed_control_signal, steering_control_signal)
           
    
            self.print_obstacle_sensors(center_sensor, left_sensor, right_sensor)

            print("\n----------------------------------------\n")
            time.sleep(TIME_STEP) # Sleep 


    def adjust_motors(self, speed_control_signal, steering_control_signal):
        # Control the motors based on PID output
        base_speed = min(MAX_POW, max(MIN_POW, MAX_POW + speed_control_signal))
        
        left_motor_power = max(MIN_POW, min(base_speed, base_speed - steering_control_signal))  # Reduce power to left motor for right turn
        right_motor_power = max(MIN_POW, min(base_speed, base_speed + steering_control_signal))  # Reduce power to right motor for left turn
        
        print(f"Speed Control Signal: {speed_control_signal}")
        print(f"Steering Control Signal: {steering_control_signal}")
        print(f"lPow rPow: ({round(left_motor_power, 2)}, {round(right_motor_power, 2)})")
        self.driveMotors(left_motor_power, right_motor_power)

    def print_obstacle_sensors(self, center_sensor, left_sensor, right_sensor):
        """Prints the values from the obstacle sensors."""
        print(f"Filter Center IR Sensor: {center_sensor}") 
        print(f"Filter Left IR Sensor: {left_sensor}")  
        print(f"Filter Right IR Sensor: {right_sensor}")  

    def print_line_sensors(self):
        print("".join(str(m) for m in self.measures.lineSensor))

    def is_intersection(self, center_sensor, left_sensor, right_sensor):
        """Detects an intersection based on sensor values."""
        return center_sensor <= 0.8 and left_sensor <= 1.1 and right_sensor <= 1.1
    
    def is_dead_end(self):
        # Do not detect dead end
        # If the center sensor is too high, center_sensor >= 2.0
        # Rotate until is less than 1.0
        # Move
        # This counters dead ends 
        # and possibly other situations
        pass