import sys

from croblink import *
from robot_state import RobotState
from utils.maze_map import MazeMap, Cell
from utils.pd_controller import PDController
from utils.bfs import bfs, shortest_path_bfs, shortest_unvisited_path_bfs
from constants import *
from utils.sensor_reliability import *
from utils.MultiColumnData import *
import itertools





#############################

DATA_X = MultiColumnData(2,['X (GPS)','X (MM)'])
DATA_Y = MultiColumnData(2,['Y (GPS)','Y (MM)'])
DATA_COMPASS = MultiColumnData(3,['θ (Noise)','θ (MM)','θ (KF)'])

import signal
import sys

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    if DEBUG:
        DATA_X.plot_all_columns()
        plt.savefig("./plot/x_plot.png", format='png', dpi=300)
        DATA_Y.plot_all_columns()
        plt.savefig("./plot/y_plot.png", format='png', dpi=300)
        DATA_COMPASS.plot_all_columns()
        plt.savefig("./plot/compass_plot.png", format='png', dpi=300)
        plt.show()
    
    # Perform any cleanup here
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#############################

DEBUG = False


class Robot(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, outfile):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angs=angles, host=host)

        self.outfile = outfile
        
        self.robot = RobotState()  # Encapsulates robot state
        self.maze = MazeMap(rows=CELLROWS, cols=CELLCOLS)

        self.speed_pd_controller = PDController(kp=KP, kd=KD, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PDController Throttle
        self.speed_steering_pd_controller = PDController(kp=KPSS, kd=KDSS, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PDController Throttle Steering
        self.steering_pd_controller = PDController(kp=KPS, kd=KDS, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PDController Steering
        self.recalibration_pd_controller = PDController(kp=KPR, kd=KDR, time_step=TIME_STEP, min_output=MIN_POW, max_output=MAX_POW) # PDController Steering

        self.sensor_reliability  = SensorReliabilty(window_size=5)
        self.ground_reliability = SensorReliabilty(window_size=5)

        self.is_next_cell = False

        self.endLap = False


    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        self.robot.initialize(self)

        if DEBUG:
            self.realGPS = (self.measures.x,self.measures.y)

        while True:
            if  not self.robot.read_sensors_update_measures(self): continue # Update sensor readings and position

            if DEBUG:
                print(f"[{self.measures.time}]-------------------------------")
                DATA_X.add_row([self.measures.x,self.realGPS[0] + self.robot.current_position[0]])
                DATA_Y.add_row([self.measures.y, self.realGPS[1] + self.robot.current_position[1]])
                DATA_COMPASS.add_row([self.measures.compass, self.robot.movement_model.compass_movement_model,self.robot.current_direction])
                print(f"Target Cell: {self.robot.target_cells}")


            # Target Cells
            has_target_cell, target_id = self.ground_reliability.update(self.measures.ground)

            if has_target_cell and self.is_next_cell and target_id > 0:
                self.robot.target_cells[target_id] = self.robot.cell
                self.is_next_cell = False

            # Steering Mode
            if self.robot.steering_mode == True and self.robot.direction_setpoint is not None:
                if self.steering():
                    continue
                self.robot.switch_to_moving()
                if self.robot.recalibration_mode == True: continue 
            
            # Throttle Mode
            if self.robot.moving_mode == True and self.robot.position_setpoint is not None:
                if self.move_forward(): 
                    continue
                self.robot.switch_to_steering()
                if self.steering(): continue # After finish moving forward adjust direction
            
            # Recalibration Mode
            if self.robot.recalibration_mode == True:
                if self.robot.recalibration_phase == 0:
                    if self.recalibration():
                        continue
                    else: self.robot.recalibration_phase = 1 # Next recalibration phase
                
                is_reliable, distance = self.sensor_reliability.update(self.robot.ir_sensors["center"])
                if is_reliable: 
                    self.recalibrate_position(distance)
                continue


            # Robot reach a new position
            self.robot.cell = self.robot.cell_setpoint # Update robot cell after new position is reached 
            self.is_next_cell = True
            sensor_map = self.robot.cell.mark_walls(self.robot.ir_sensors, closest_direction(self.robot.current_direction))
            

            # Recalibration
            is_x_recalibration_due = closest_direction(self.robot.current_direction) in [WEST, EAST] and self.robot.recalibration_counter_x >= RECALIBRATION_PERIOD_X
            
            is_y_recalibration_due = closest_direction(self.robot.current_direction) in [NORTH, SOUTH] and self.robot.recalibration_counter_y >= RECALIBRATION_PERIOD_Y
            
            is_wall_in_front = getattr(self.robot.cell, sensor_map.get("center")) == True

            if not self.robot.recalibration_complete and (is_x_recalibration_due or is_y_recalibration_due) and is_wall_in_front:
                self.robot.switch_to_recalibration()
                if closest_direction(self.robot.current_direction) in [WEST, EAST]: self.robot.recalibration_counter_x = 0
                elif closest_direction(self.robot.current_direction) in [NORTH, SOUTH]: self.robot.recalibration_counter_y = 0
                continue
                

            if len(self.robot.target_cells) >= int(self.nBeacons) - 1:
                if len(self.robot.pathfinding_path) <= 0 and self.robot.cell == self.maze.get_cell(self.robot.initial_position): 
                    if DEBUG:
                        DATA_X.plot_all_columns()
                        plt.savefig("./plot/x_plot.png", format='png', dpi=300)
                        DATA_Y.plot_all_columns()
                        plt.savefig("./plot/y_plot.png", format='png', dpi=300)
                        DATA_COMPASS.plot_all_columns()
                        plt.savefig("./plot/compass_plot.png", format='png', dpi=300)
                        plt.show()
                    
                    print("End of Lap.")
                    self.driveMotors(0, 0)
                    sys.exit(0)
                
                if len(self.robot.pathfinding_path) <= 0: 
                    self.compute_target_cell_path()
                    


            # Compute next position
            if self.robot.pathfinding_path:
                self.follow_path()
            else: 
                if not self.get_next_move() : # Compute the next move
                    if bfs(self) == False: break # Map exploration complete
                    else: continue
            
            self.robot.recalibration_complete = False
            self.robot.recalibration_counter_x += 1
            self.robot.recalibration_counter_y += 1
        
        #self.compute_target_cell_path()
    

    def get_next_move(self):
        for sensor_name, sensor_value in self.robot.ir_sensors.items():
            if sensor_value <= SENSOR_THRESHOLD: # If no wall
                move_vector = self.robot.sensor_vector_map(sensor_name)
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

        path_combinations = list(itertools.permutations(self.robot.target_cells.keys(), len(self.robot.target_cells)))
        
        self.robot.target_cell_path = None

        for combination in path_combinations:
            path1 = shortest_path_bfs(initial_cell, self.robot.target_cells[combination[0]], self.maze)
            path1_unvisited = shortest_unvisited_path_bfs(initial_cell, self.robot.target_cells[combination[0]], self.maze)
            if len(path1) != len(path1_unvisited): return False

            # print all cell from path1 using get_middle_position

            for i in range(1, len(combination)):
                path2 = shortest_path_bfs(self.robot.target_cells[combination[i-1]], self.robot.target_cells[combination[i]], self.maze)
                path2_unvisited = shortest_unvisited_path_bfs(self.robot.target_cells[combination[i-1]], self.robot.target_cells[combination[i]], self.maze)
                if len(path2) != len(path2_unvisited): return False

                path1.extend(path2)
            
            path3 = shortest_path_bfs(self.robot.target_cells[combination[-1]], initial_cell, self.maze)
            path3_unvisited = shortest_unvisited_path_bfs(self.robot.target_cells[combination[-1]], initial_cell, self.maze)
            if len(path3) != len(path3_unvisited): return False
            path1.extend(path3)

            if not self.robot.target_cell_path:
                self.robot.target_cell_path = path1

            if len(self.robot.target_cell_path) > len(path1):
                self.robot.target_cell_path = path1


        goToStartPath = shortest_path_bfs(self.robot.cell, initial_cell, self.maze)

        finalPath = []
        finalPath.extend(goToStartPath)
        finalPath.extend(self.robot.target_cell_path)
        
        self.robot.pathfinding_path = finalPath

        if DEBUG:
            print(f"SP:\t{len(self.robot.target_cell_path)}")

            p = []
            for cell in self.robot.pathfinding_path:
                p.append(cell.get_middle_position())
            print(p)


        for cell in self.robot.target_cell_path:
            x, y = self.maze.get_cell_index(cell)
            
            # Write the final map to a text file
            with open(self.outfile, "w") as file:
                file.write("0 0 #0\n")
                for cell in self.robot.target_cell_path:
                    x, y = self.maze.get_cell_index(cell)
                    x = int(x)
                    y = int(y)

                    for key,value in self.robot.target_cells.items():
                        if value == cell:
                            file.write(f"{x} {y} #{int(key)}\n")
                            break

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


        if DEBUG:
            print(f"Steering Power: ({-steering_correction}, {steering_correction})")

        return True
    
    
    def move_forward(self):
        if DEBUG:
            print("MOVING FORWARD")
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
                
        invert_power_speed = self.robot.direction_setpoint in [SOUTH, WEST]
        if self.robot.direction_setpoint in (WEST, EAST):
            self.move_to_position(self.robot.current_position[0], self.robot.position_setpoint[0], self.robot.current_position[1], self.robot.position_setpoint[1], invert_power_speed) # x coordinate
        elif self.robot.direction_setpoint in (NORTH, SOUTH):
            self.move_to_position(self.robot.current_position[1], self.robot.position_setpoint[1], self.robot.current_position[0], self.robot.position_setpoint[0], invert_power_speed) # y coordinate
        
        return True
    

    def move_to_position(self, current_val_speed, target_val_speed, current_val_speed_steering, targer_val_speed_steering, invert_power_speed):
        motor_power = self.speed_pd_controller.compute(current_val_speed, target_val_speed)
        steering_power = self.speed_steering_pd_controller.compute(current_val_speed_steering, targer_val_speed_steering)
        
        if invert_power_speed:
            motor_power = -motor_power # Reverse motor power if robot is facing SOUTH or WEST

        base_speed = motor_power
        
        if self.robot.direction_setpoint in [EAST, SOUTH]:
            left_motor_power, right_motor_power = self.calculate_motor_power(base_speed, steering_power)
        elif self.robot.direction_setpoint in [WEST, NORTH]:
            right_motor_power, left_motor_power = self.calculate_motor_power(base_speed, steering_power)

    
        self.robot.movement_model.input_signal_left = left_motor_power
        self.robot.movement_model.input_signal_right = right_motor_power

        self.driveMotors(left_motor_power, right_motor_power)    



        if DEBUG:
            if left_motor_power < right_motor_power:
               print("Esquerda")
            elif left_motor_power > right_motor_power:
               print("Direita")
            else: print("Frente")
            print(f"Base Speed: {base_speed}")
            print(f"Steering Speed: {steering_power}")
            print(f"lPow rPow: ({round(left_motor_power, 2)}, {round(right_motor_power, 2)})")
            print(f"Throttle Power: ({left_motor_power}, {right_motor_power})")
    

    def calculate_motor_power(self, base_speed, steering_power):
        if steering_power >= 0:
            left_motor = max(MIN_POW, min(base_speed, base_speed - steering_power))
            right_motor = base_speed
        else:
            left_motor = base_speed
            right_motor = max(MIN_POW, min(base_speed, base_speed + steering_power))
        return left_motor, right_motor


    def recalibration(self):
        if self.robot.previous_position == self.robot.current_position and self.robot.ir_sensors["center"] == CENTER_SENSOR_SETPOINT or \
            (
                abs(self.robot.previous_position[0] - self.robot.current_position[0]) < 0.1 and \
                abs(self.robot.previous_position[1] - self.robot.current_position[1]) < 0.1 and \
                abs(self.robot.ir_sensors["center"] - CENTER_SENSOR_SETPOINT) < 0.1
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
    
    
    def recalibrate_position(self, distance):
        dir = closest_direction(self.robot.current_direction) 
        
        # Center of cell - Wall thickness - distance from the wall
        distance = (0.5 - 0.1) - (1 / distance)
        

        if dir ==  EAST:
            self.robot.current_position = (round(self.robot.position_setpoint[0]+distance,2), self.robot.current_position[1]) 
        elif dir == WEST:
            self.robot.current_position = (round(self.robot.position_setpoint[0]-distance,2), self.robot.current_position[1])
        elif dir == NORTH:
            self.robot.current_position = (self.robot.current_position[0], round(self.robot.position_setpoint[1]+distance,2))
        else: # South
            self.robot.current_position = (self.robot.current_position[0], round(self.robot.position_setpoint[1]- distance,2))
        
        if DEBUG:
            print("Robot Recalibrated.")
            print(f"Distance: {distance}")
            print(f"Direction: {dir}")
            print(f"Current Direction: {self.robot.current_direction}")
            print(f"Current Position: {self.robot.current_position}")
            print(f"Position Setpoint: {self.robot.position_setpoint}")
            print(f"Recalibrated Current Position: {self.robot.current_position}")
        
        self.robot.switch_to_steering()
        self.robot.recalibration_complete = True
        self.robot.recalibration_phase = 0
        self.sensor_reliability.clear_window()
                

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
        
