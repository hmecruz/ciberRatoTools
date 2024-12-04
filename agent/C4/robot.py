import sys

from croblink import *
from robot_state import RobotState
from utils.maze_map import MazeMap, Cell
from utils.pd_controller import PDController
from utils.bfs import bfs, shortest_path_bfs, shortest_unvisited_path_bfs
from constants import *
from utils.sensor_reliability import *
from utils.MultiColumnData import *


import signal
import sys

DATA_X = MultiColumnData(2,['X (GPS)','X (MM)'])
DATA_Y = MultiColumnData(2,['Y (GPS)','Y (MM)'])
DATA_COMPASS = MultiColumnData(3,['θ (Noise)','θ (MM)','θ (KF)'])

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    DATA_X.plot_all_columns()
    DATA_Y.plot_all_columns()
    DATA_COMPASS.plot_all_columns()
    plt.show()
    
    
    # Perform any cleanup here
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#############################



class Robot(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, outfile):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angs=angles, host=host)

        self.outfile = outfile
        
        self.robot = RobotState()  # Encapsulates robot state
        self.maze = MazeMap(rows=CELLROWS, cols=CELLCOLS)

        self.speed_pd_controller = PDController(kp=KP, kd=KD, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PDController Throttle
        self.steering_pd_controller = PDController(kp=KPS, kd=KDS, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PDController Steering
        self.recalibration_pd_controller = PDController(kp=KPR, kd=KDR, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PDController Steering

        self.sensor_reliability  = SensorReliabilty(window_size=5)


    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        self.robot.initialize(self)

        # TODO: remove this
        self.realGPS = (self.measures.x,self.measures.y)

        while True:
            # TODO: add a false so the loop doesnt continue
            if  not self.robot.read_sensors_update_measures(self): continue # Update sensor readings and position
            print("----------------------------------------")

            # TODO: remove this
            DATA_X.add_row([self.realGPS[0] + self.robot.current_position[0], self.measures.x])
            DATA_Y.add_row([self.realGPS[1] + self.robot.current_position[1], self.measures.y])
            DATA_COMPASS.add_row([self.measures.compass, self.robot.movement_model.compass_movement_model,self.robot.current_direction])

            
            if self.robot.steering_mode == True and self.robot.direction_setpoint is not None:
                if self.steering():
                    continue
                self.robot.switch_to_moving()
                if self.robot.recalibration_mode == True: continue 
            if self.robot.moving_mode == True and self.robot.position_setpoint is not None:
                if self.move_forward(): 
                    continue
                self.robot.switch_to_steering()
                if self.steering(): continue # After finish moving forward adjust direction
            if self.robot.recalibration_mode == True:
                if self.robot.recalibration_has_ended:
                    if self.recalibration():
                        print("Recalibrar")
                        continue

                self.robot.recalibration_has_ended = False
                
                is_reliable,distance = self.sensor_reliability.update(self.robot.ir_sensors["center"])
                if not is_reliable:
                    continue

                self.robot.recalibration_has_ended = True

                # TODO: change current position
                dir = closest_direction(self.robot.current_direction) 

                print(f"Cycle:  {self.measures.time}")
                print(f"Distance: {1/distance}")


                normalized_angle =  self.robot.current_direction
                if normalized_angle < 0:
                    normalized_angle = 360 + normalized_angle   

                diff_angle = abs(dir - normalized_angle)


                # TODO: Doing wrong this calculation, GPS and MM should be equal
                # WALLS ARE 0.2 THICK
                distance = 1/distance
                distance = (0.5-0.1) - abs(math.cos(diff_angle) * distance)
                

                print(f"DistanceT: {distance}")
                print(f"Dir: {dir}")
                print(f"CD: {self.robot.current_direction}")
                print(f"DiffDir: {diff_angle}")
                print(f"SP: {self.robot.position_setpoint}")
                print(f"CPb: {self.robot.current_position}")

                if dir ==  EAST:
                    self.robot.current_position = (round(self.robot.position_setpoint[0]+distance,2), self.robot.current_position[1])
                    print("X")
                elif dir == WEST:
                    self.robot.current_position = (round(self.robot.position_setpoint[0]-distance,2), self.robot.current_position[1])
                    print("X")
                elif dir == NORTH:
                    self.robot.current_position = (self.robot.current_position[0], round(self.robot.position_setpoint[1]+distance,2))
                    print("Y")
                else:
                    print("Y")
                    self.robot.current_position = (self.robot.current_position[0], round(self.robot.position_setpoint[1]- distance,2))
                
                
                print(f"CPa: {self.robot.current_position}")
                print(f"M: {self.measures.x - self.realGPS[0]} {self.measures.y- self.realGPS[1]}")
                
                # TODO: might need to reset either MM or KF (i think it is KF)

                print("Terminei a recalibração")
                #self.robot.switch_to_moving()
                self.robot.switch_to_steering()
                self.robot.recalibration_complete = True
                continue

            # Robot reach new position
            self.robot.cell = self.robot.cell_setpoint # Update robot cell after new position is reached 
            sensor_map = self.robot.cell.mark_walls(self.robot.ir_sensors, closest_direction(self.robot.current_direction))
            
            # Activate recalibration if wall in front
            if self.robot.recalibration_complete == False: # If recalibration has not been done
                front_wall = sensor_map.get("center")
                if getattr(self.robot.cell, front_wall) == True: # If wall in front 
                    self.robot.switch_to_recalibration() 
                    continue

            self.robot.recalibration_complete = False # Reset recalibration for next move
            self.sensor_reliability.clear_window()
            #self.robot.movement_model.angle_kalman_filter.reset()


            # Recalibrate 


            if self.robot.first_target_cell is None and self.measures.ground == 1: 
                self.robot.first_target_cell = self.robot.cell
            if self.robot.second_target_cell is None and self.measures.ground == 2:
                self.robot.second_target_cell = self.robot.cell

            if self.robot.first_target_cell is not None and self.robot.second_target_cell is not None:
                if self.compute_target_cell_path():
                    sys.exit(0)


            # Compute next position
            if self.robot.pathfinding_path:
                self.follow_path()
            else: 
                if not self.get_next_move() : # Compute the next move
                    if bfs(self) == False: break # Map exploration complete
          
        self.compute_target_cell_path()
    
    def get_next_move(self):
        for sensor_name, sensor_value in self.robot.ir_sensors.items():
            if sensor_value <= SENSOR_THRESHOLD: # If no wall
                move_vector = self.robot.sensor_vector_map(sensor_name)
                #next_position = (self.robot.current_position[0] + move_vector[0], self.robot.current_position[1] + move_vector[1])
                cell_middle_position = self.robot.cell.get_middle_position()
                next_position = (
                    cell_middle_position[0] + move_vector[0],
                    cell_middle_position[1] + move_vector[1]
                )

                if not self.maze.is_cell_visited(next_position): # If cell not visited
                    # Update direction and position setpoint
                    self.robot.position_setpoint = next_position 
                    self.robot.direction_setpoint = vector_to_direction(move_vector)
                    
                    # Cell, creation and add to map and visited
                    self.robot.cell_index = (self.robot.cell_index[0] + move_vector[0], self.robot.cell_index[1] + move_vector[1])
                    next_cell = self.create_cell_from_vector(move_vector)
                    self.robot.cell_setpoint = next_cell
                    self.maze.add_cell_map(next_cell, self.robot.cell_index)
                    return True # Success 
                
        return False # Not able to compute next move


    def follow_path(self):
        self.robot.cell_setpoint = self.robot.pathfinding_path.pop(0) 
        
        current_cell_middle_position = self.robot.cell.get_middle_position()
        cell_setpoint_middle_position = self.robot.cell_setpoint.get_middle_position()

        move_vector = (
            cell_setpoint_middle_position[0] - current_cell_middle_position[0],
            cell_setpoint_middle_position[1] - current_cell_middle_position[1]
        )

        self.robot.cell_index = (
            self.robot.cell_index[0] + move_vector[0], 
            self.robot.cell_index[1] + move_vector[1]
        )

        next_position = (
            current_cell_middle_position[0] + move_vector[0], 
            current_cell_middle_position[1] + move_vector[1]
        )

        self.robot.position_setpoint = next_position 
        self.robot.direction_setpoint = vector_to_direction(move_vector)
                

    def compute_target_cell_path(self):    
        initial_cell = self.maze.get_cell(self.robot.initial_position)
    
        path1 = shortest_path_bfs(initial_cell, self.robot.first_target_cell, self.maze)
        path1_unvisited = shortest_unvisited_path_bfs(initial_cell, self.robot.first_target_cell, self.maze)
        print(len(path1))
        print(len(path1_unvisited))
        if len(path1) != len(path1_unvisited): return False

        path2 = shortest_path_bfs(self.robot.first_target_cell, self.robot.second_target_cell, self.maze)
        path2_unvisited = shortest_unvisited_path_bfs(self.robot.first_target_cell, self.robot.second_target_cell, self.maze)
        print(len(path2))
        print(len(path2_unvisited))
        if len(path2) != len(path2_unvisited): return False

        path3 = shortest_path_bfs(self.robot.second_target_cell, initial_cell, self.maze)
        path3_unvisited = shortest_unvisited_path_bfs(self.robot.second_target_cell, initial_cell, self.maze)
        print(len(path3))
        print(len(path3_unvisited))
        if len(path3) != len(path3_unvisited): return False

        self.robot.target_cell_path.extend(path1)
        self.robot.target_cell_path.extend(path2[1:-1])
        self.robot.target_cell_path.extend(path3)

        for cell in self.robot.target_cell_path:
            x, y = self.maze.get_cell_index(cell)
            
            # Write the final map to a text file
            with open(self.outfile, "w") as file:
                for cell in self.robot.target_cell_path:
                    x, y = self.maze.get_cell_index(cell)
                    x = int(x)
                    y = int(y)
                    file.write(f"{x} {y}\n")
                
        return True
            
    
    def steering(self):
        if self.robot.previous_direction == self.robot.current_direction == self.robot.direction_setpoint or \
            (
                abs(self.robot.previous_direction - self.robot.current_direction) <= 3 and \
                abs(self.robot.direction_setpoint - self.robot.current_direction) <= 1
            ):

            self.robot.movement_model.input_signal_left = 0
            self.robot.movement_model.input_signal_right  = 0
             
            self.driveMotors(0, 0) # Stop motors
            return False

        steering_correction = self.steering_pd_controller.compute_angle(self.robot.current_direction, self.robot.direction_setpoint)
        self.robot.movement_model.input_signal_left = -steering_correction
        self.robot.movement_model.input_signal_right = steering_correction
        self.driveMotors(-steering_correction, steering_correction)
        #print(f"Steering Power: ({-steering_correction}, {steering_correction})")
        return True
    
    
    def move_forward(self):
        print(f"Previous Position: {self.robot.previous_position}")
        print(f"Current Position: {self.robot.current_position}")
        print(f"Position Setpoint: {self.robot.position_setpoint}")
        
        if self.robot.previous_position == self.robot.current_position == self.robot.position_setpoint or \
        (
            abs(self.robot.previous_position[0] - self.robot.current_position[0]) < 0.1 and \
            abs(self.robot.previous_position[1] - self.robot.current_position[1]) < 0.1 and \
            (
                (closest_direction(self.robot.current_direction) in (NORTH, SOUTH) and \
                abs(self.robot.current_position[1] - self.robot.position_setpoint[1]) < 0.1) or \
                (closest_direction(self.robot.current_direction) in (WEST, EAST) and \
                abs(self.robot.current_position[0] - self.robot.position_setpoint[0]) < 0.1)
            )
        ):

            self.robot.movement_model.input_signal_left = 0
            self.robot.movement_model.input_signal_right = 0
            self.driveMotors(0, 0) # Stop Motors
            return False
                
        invert_power = self.robot.direction_setpoint in [SOUTH, WEST]
        if self.robot.direction_setpoint in (WEST, EAST):
            self.move_to_position(self.robot.current_position[0], self.robot.position_setpoint[0], invert_power) # x coordinate
        elif self.robot.direction_setpoint in (NORTH, SOUTH):
            self.move_to_position(self.robot.current_position[1], self.robot.position_setpoint[1], invert_power) # y coordinate
        
        return True
    

    def move_to_position(self, current_val, target_val, invert_power):
        motor_power = self.speed_pd_controller.compute(current_val, target_val)

        if invert_power:
            motor_power = -motor_power # Reverse motor power if robot is facing SOUTH or WEST

        # current_angle = self.robot.current_direction
        # if current_angle < 0:
        #     current_angle = 360 + current_angle
        
        # diff_angle = abs(current_angle - self.robot.direction_setpoint)
        
        # if abs(diff_angle) > 3:
        #     angle_correction = self.angle_pd_controller.compute(current_angle, self.robot.direction_setpoint)
        #     self.robot.movement_model.input_signal_left = motor_power - angle_correction
        #     self.robot.movement_model.input_signal_right = motor_power + angle_correction
        # else:
        #     self.robot.movement_model.input_signal_left = motor_power
        #     self.robot.movement_model.input_signal_right = motor_power

        self.robot.movement_model.input_signal_left = motor_power
        self.robot.movement_model.input_signal_right = motor_power

        self.driveMotors(motor_power, motor_power)    
        #print(f"Throttle Power: ({motor_power}, {motor_power})")
    
    def recalibration(self):
        if self.robot.previous_position == self.robot.current_position and self.robot.ir_sensors["center"] == CENTER_SENSOR_SETPOINT or \
            (
                abs(self.robot.previous_position[0] - self.robot.current_position[0]) < 0.1 and \
                abs(self.robot.previous_position[1] - self.robot.current_position[1]) < 0.1 and \
                abs(self.robot.ir_sensors["center"] - CENTER_SENSOR_SETPOINT) < 0.1 # TODO: adjust value
            ):

            self.robot.movement_model.input_signal_left = 0
            self.robot.movement_model.input_signal_right = 0
            self.driveMotors(0, 0)
            return False

        
        motor_power = self.recalibration_pd_controller.compute(self.robot.ir_sensors["center"], CENTER_SENSOR_SETPOINT)

        self.robot.movement_model.input_signal_left = motor_power
        self.robot.movement_model.input_signal_right = motor_power
        self.driveMotors(motor_power, motor_power)  

        return True
    

    def create_cell_from_vector(self, vector):
        """Create a cell based on the robot's next position vector and current cell."""
        (bl_x, bl_y), (tr_x, tr_y) = self.robot.cell.coordinates

        if vector == MOVE_NORTH: 
            bottom_left = (bl_x, tr_y)
        elif vector == MOVE_WEST: 
            bottom_left = (bl_x - 2, bl_y)
        elif vector == MOVE_EAST: 
            bottom_left = (tr_x, bl_y)
        elif vector == MOVE_SOUTH: 
            bottom_left = (bl_x, bl_y - 2)
            
        return Cell(bottom_left)
        