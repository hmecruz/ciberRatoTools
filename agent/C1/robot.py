from croblink import *
from pid_controller import PIDController
from noise_filter import NoiseFilter

# MAX / MIN Velocity values 
MAX_POW = 0.15 #lPow rPow max velocity value 
MIN_POW = -0.15 #lPow rPow min velocity value 

TIME_STEP = 0.005 # Sampling time 50 ms --> 0.005 

# Throttle PD Controller values
KP = 0.002 
KD = 0

# Steering PD Controller Values
KPS = 0.098
KDS = 0.00075 # 0.0008


class Robot(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

        # PIDController 
        self.speed_pid_controller = PIDController(kp=KP, kd=KD, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PIDController Throttle
        self.steering_pid_controller = PIDController(kp=KPS, kd=KDS, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PIDController Steering
        
        self.speed_setpoint = 0.8
        self.steering_setpoint = 0

        self.error_threshold = 0.2 # Ignore small errors --> Errors can be cause by noise
        
        # Noise Filter used to detect intersections
        self.filter_left = NoiseFilter(window_size=4)
        self.filter_right = NoiseFilter(window_size=4)
        self.filter_center = NoiseFilter(window_size=4) 

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        while True:

            # Read IR obstacle sensors values 
            self.readSensors() # Read sensor is a blocking action every 50ms --> See Labs/rmi-2425/C1-config.xml
    
            center_sensor = self.measures.irSensor[0]
            left_sensor = self.measures.irSensor[1]
            right_sensor = self.measures.irSensor[2]

            #self.print_obstacle_sensors(center_sensor, left_sensor, right_sensor)

            # Calculate the error as the difference between the left and right sensors
            if self.is_intersection(center_sensor, left_sensor, right_sensor): print("Intersection")
            if abs(left_sensor - right_sensor) <= self.error_threshold and center_sensor <= self.speed_setpoint or self.is_intersection(center_sensor, left_sensor, right_sensor):
                steering_error = 0
            else:
                steering_error = left_sensor - right_sensor # Positive error means closer to the right, negative closer to the left
        
            # Compute control signal if error is significant
            speed_control_signal = self.speed_pid_controller.compute(center_sensor, self.speed_setpoint)
            steering_control_signal = self.steering_pid_controller.compute(steering_error, self.steering_setpoint)
            self.adjust_motors(speed_control_signal, steering_control_signal)
           
            #print("\n----------------------------------------\n")


    def adjust_motors(self, speed_control_signal, steering_control_signal):
        # Control the motors based on PID output
        base_speed = min(MAX_POW, max(MIN_POW, MAX_POW + speed_control_signal))
        
        left_motor_power = max(MIN_POW, min(base_speed, base_speed - steering_control_signal))  # Reduce power to left motor for right turn
        right_motor_power = max(MIN_POW, min(base_speed, base_speed + steering_control_signal))  # Reduce power to right motor for left turn
        
        #print(f"Speed Control Signal: {speed_control_signal}")
        #print(f"Steering Control Signal: {steering_control_signal}")
        #print(f"lPow rPow: ({round(left_motor_power, 2)}, {round(right_motor_power, 2)})")
        self.driveMotors(left_motor_power, right_motor_power)

    
    def is_intersection(self, center_sensor, left_sensor, right_sensor):
        """Detects an intersection based on sensor values."""
        center_sensor = self.filter_center.update(center_sensor)
        left_sensor = self.filter_left.update(left_sensor)
        right_sensor = self.filter_right.update(right_sensor)

        return center_sensor <= 1 and left_sensor <= 1.1 and right_sensor <= 1.1
    
    def print_obstacle_sensors(self, center_sensor, left_sensor, right_sensor):
        """Prints the values from the obstacle sensors."""
        print(f"Center IR Sensor: {center_sensor}") 
        print(f"Left IR Sensor: {left_sensor}")  
        print(f"Right IR Sensor: {right_sensor}")  

    