from croblink import *
import time

class Robot(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'
        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True);
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()
        

            self.printObstacleSensors()
            self.printLineSensors()
        
            print("\n----------------------------------------\n")
            time.sleep(0.1)

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if    self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id]   > 5.0\
           or self.measures.irSensor[right_id]  > 5.0\
           or self.measures.irSensor[back_id]   > 5.0:
            print('Rotate left')
            self.driveMotors(-0.05,+0.05)
        elif self.measures.irSensor[left_id]> 2.7:
            print('Rotate slowly right')
            self.driveMotors(0.05,0.0)
        elif self.measures.irSensor[right_id]> 2.7:
            print('Rotate slowly left')
            self.driveMotors(0.0,0.05)
        else:
            print('Go')
            self.driveMotors(0.05,0.05)

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

    def printLineSensors(self):
        """Prints the values from the line sensors."""
        for i, lineSensor in enumerate(self.measures.lineSensor):
            print(f"Line Sensor {i}: {lineSensor}")

    def printGroundSensor(self):
        """Prints the value from the ground sensor."""
        print(f"Ground Sensor Value: {self.measures.ground}")

    def printCollisionSensor(self):
        """Prints the value from the collision sensor."""
        print(f"Collision Sensor Ready: {self.measures.collisionReady}")

