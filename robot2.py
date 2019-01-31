#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""
import oi
from oi import read_all_controller_inputs
import wpilib
from wpilib.drive import MecanumDrive
import time
from state import state

class MyRobot(wpilib.IterativeRobot):
    
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """

        #self.encoder = wpilib.Encoder(0, 6)
        self.ir = wpilib.DigitalInput(1)
        self.ir2 = wpilib.DigitalInput(2)
        self.ir3 = wpilib.DigitalInput(3)
        self.ir4 = wpilib.DigitalInput(4)
        self.ir5 = wpilib.DigitalInput(5)
        self.ir6 = wpilib.DigitalInput(6)


        self.timer = wpilib.Timer()
        self.front_left_motor = wpilib.Talon(0)
        self.rear_left_motor = wpilib.Talon(1)
        self.front_right_motor = wpilib.Talon(2)
        self.rear_right_motor = wpilib.Talon(3)

        self.drive = MecanumDrive(
            self.front_left_motor,
            self.rear_left_motor,
            self.front_right_motor,
            self.rear_right_motor)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):

        if self.ir3.get() or self.ir4.get():
            if self.timer.get() < 3:
                self.timer.reset()
                self.timer.start()
                self.drive.driveCartesian(1,0,0,0)

        elif self.ir.get() or self.ir2.get():
            self.drive.driveCartesian(0,1,0,0)


        elif self.ir5.get() or self.ir6.get():
            self.drive.driveCartesian(0,-1,0,0)
           

        else:
            self.drive.driveCartesian(0,-1,0,0)



        """This function is called periodically during autonomous."""
        
        #Rocket right side
        """if self.timer.get() < 7:
            self.drive.driveCartesian(1,0,0,0)

        elif self.timer.get() > 7 and self.timer.get() < 9:
            self.drive.driveCartesian(0,0,1,0)

        elif self.timer.get() > 9 and self.timer.get() < 14:
            self.drive.driveCartesian(1,0,0,0)

        elif self.timer.get() > 14 and self.timer.get() < 17:
            self.drive.driveCartesian(0,0,1,0)
        """
        #Cargo ship
        """if self.timer.get() < 9:
            self.drive.driveCartesian(1,0,0,0)
        elif self.timer.get() > 9 and self.timer.get() < 11:
            self.drive.driveCartesian(0,0,-1,0)"""                 
        #
        """if self.timer.get() < 5:
            self.drive.driveCartesian(1,0,0,0)
        elif self.timer.get() > 5 and self.timer.get() < 10:
            self.drive.driveCartesian(0,0,1,0)
        elif self.timer.get() > 10 and self.timer.get() < 15:
            self.drive.driveCartesian(1,0,0,0)
        elif self.timer.get() > 15 and self.timer.get( ) < 20:
            self.drive.driveCartesian(0,0,-1,0)"""






    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        oi.read_all_controller_inputs()
        x = state["mov_x"] * .7
        y = state["mov_y"] * .7
        z = state["mov_z"] * .5

        powerX = 0 if x < 0.15 and x > -0.15 else x
        powerY = 0 if y < 0.15 and y > -0.15 else y
        powerZ = 0 if z < 0.15 and z > -0.15 else z

        self.drive.driveCartesian(powerX, -powerY, powerZ, 0)
        #print(self.encoder.get())

        if state["ir"] == True:
            if self.ir5.get() or self.ir6.get():
                self.drive.driveCartesian(0,-1,0,0)

            elif self.ir.get() or self.ir2.get():
                self.drive.driveCartesian(0,1,0,0)

            elif self.ir3.get() or self.ir4.get():
                self.drive.driveCartesian(1,0,0,0)
                self.timer.start()

                if self.timer.get() <= 3:
                    """self.timer.start()"""
                    self.drive.driveCartesian(1,0,0,0)
           
            else:
                self.drive.driveCartesian(0,0,0,0)
        # else:
        #     self.drive.driveCartesian(0,0,0,0)


if __name__ == "__main__":
    wpilib.run(MyRobot)