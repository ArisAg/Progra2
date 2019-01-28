
import wpilib
import threading
from wpilib.drive import MecanumDrive
from state import state
import oi
import time
import infrared
import math 
import data
import wpilib
import wpilib.encoder
from pidcontroller import PIDController
import pidcontroller 

#import wpilibcontroller
#import PID
#from encoder import Encoder
from wpilib import Encoder, IterativeRobot
#import pygame


class MyRobot(wpilib.TimedRobot):


	def robotInit(self):
		
		#motores

		self.mutex = threading.RLock(1)
		self.frontLeftMotor = wpilib.Talon(0)
		self.rearLeftMotor = wpilib.Talon(1)
		self.frontRightMotor = wpilib.Talon(2)
		self.rearRightMotor = wpilib.Talon(3)
		self.output = 0
		self.lift_motor = wpilib.Talon(4)
		self.cargo_motor = wpilib.Talon(5)
		self.result = 0
		#self.rcw = pidcontroller.rcw
		#self.
		#sensores
		#self.encoder_left = Encoder(self.pi, settings.PINS['encoder']['left'])
		#self.encoder_right = Encoder(self.pi, settings.PINS['encoder']['right'])
		self.encoder = wpilib.Encoder(0, 7)

		self.sensor_izquierdo = wpilib.DigitalInput(1)
		self.sensor_principal = wpilib.DigitalInput(2)
		self.sensor_derecho = wpilib.DigitalInput(3)
		self.ir = wpilib.AnalogInput(1)
		self.ir2 = wpilib.DigitalInput(4)
		#invertidores de motores
		#self.pid = wpilib.PIDController(P, I, D, self.TwoEncoders, self.Drive)
		self.frontLeftMotor.setInverted(True)
		self.rearLeftMotor.setInverted(True)

		self.timer = wpilib.Timer()

		#Unión de los motores para su funcionamiento
		# en conjunto de mecaunm

		self.drive = MecanumDrive(
			self.frontLeftMotor,
			self.rearLeftMotor,
			self.frontRightMotor,
			self.rearRightMotor)

		self.setpoint = 1
		#self.PIDController = PIDController() 

	def autonomousInit(self):
		"""This function is run once each time the robot enters autonomous mode."""
		self.timer.reset()
		self.timer.start()

	def autonomousPeriodic(self):
		"""This function is called periodically during autonomous."""

		# Avanzar 2.5s girar 1s avanzar 1s girar 1s avanzar 3s girar 1s avanzar 2s
		if self.timer.get() < 2.5:
			self.drive.driveCartesian(1,0,0,0)

		elif self.timer.get() > 2.5 and self.timer.get() < 3.5:
			self.drive.driveCartesian(0,0,1,0)

		elif self.timer.get() > 3.5 and self.timer.get() < 4.5:
			self.drive.driveCartesian(1,0,0,0)

		elif self.timer.get() > 4.5 and self.timer.get() < 5.5:
			self.drive.driveCartesian(0,0,1,0)

		elif self.timer.get() > 5.5 and self.timer.get() < 6.5:
			self.cargo_motor.set(1)

		elif self.timer.get() > 6.5 and self.timer.get() < 7.5:
			self.drive.driveCartesian(1,0,0,0)
			self.cargo_motor.set(-1)

		elif self.timer.get() > 8.5 and self.timer.get() < 9.5:
			self.drive.driveCartesian(1,0,0,0)
			self.cargo_motor.set(0)

		elif self.timer.get() > 10.5 and self.timer.get() < 11.5:
			self.drive.driveCartesian(0,0,1,0)

		elif self.timer.get() > 11.5 and self.timer.get() < 12.5:
			self.drive.driveCartesian(1,0,0,0)
			self.cargo_motor.set(1)
		elif self.timer.get() > 12.5 and self.timer.get() < 13.5:
			self.cargo_motor.set(-1)
		elif self.timer.get() > 13.5:
			self.cargo_motor.set(0)

		# elif self.timer.get() > 26.5 and self.timer.get() < 29.5:
		# 	self.drive.driveCartesian(1,0,0,0)
		# elif self.timer.get() > 29.5 and self.timer.get() < 31.5:
		# 	self.drive.driveCartesian(0,0,-1,0)
		else:
			self.drive.driveCartesian(0,0,0,0)
	
	def teleopPeriodic(self):

		print(PIDController.get(self))
		# print(self.setpoint)
		#print(self.rcw)

		
		#self.encoder.reset()
		Encoder.EncodingType(1)
		#se leen constantemente los botones y joysticks
		print(self.encoder.get())
		oi.read_all_controller_inputs()

		#código para el funcionamiento del movimiento
		# de las mecanum a través del control de xbox
	
		v = max(self.ir.getVoltage(), 0.00001)
		d = 62.28 * math.pow(v, -1.092)

		#print(self.ir2.get())
		# Constrain output
		#print(max(min(d, 145.0), 22.5))

		x = state["mov_x"] * .7
		y = state["mov_y"] * .7
		z = state["mov_z"] * .7

		powerX = 0 if x < 0.15 and x > -0.15 else x
		powerY = 0 if y < 0.15 and y > -0.15 else y
		powerZ = 0 if z < 0.15 and z > -0.15 else z
	

		if state["button_x_active"]:
			if self.sensor_principal.get():
				drive_for()
				self.drive.driveCartesian(0, 0, 0, 0)
			elif self.sensor_izquierdo.get():
				self.drive.driveCartesian(0.4, 0, 0, 0)
			elif self.sensor_derecho.get():
				self.drive.driveCartesian(-0.4, 0 ,0, 0)
			else:
				self.drive.driveCartesian(0, -0.5, 0, 0)

		else:
			self.drive.driveCartesian(powerX, powerY, powerZ, 0)
			
		#código para el funcionamiento del elevador y la garra
        
		if state["activating_lift"]:
			state["timer_lift"] += 1
			if state["timer_lift"] <= 100:
				self.lift_motor.set(1)
			elif state["timer_lift"] <= 200:
				self.lift_motor.set(0)
				self.cargo_motor.set(-1)
			elif state["timer_lift"] <= 300:
				self.lift_motor.set(-1)
				self.cargo_motor.set(0)

			else:
				self.lift_motor.set(0)
				self.cargo_motor.set(0)
		else:
			state["timer_lift"] = 0
			self.lift_motor.set(0)
			self.cargo_motor.set(0)

#funcion para correr el código del robot utlizando
# este archivo como el principal

if __name__ == '__main__':
	wpilib.run(MyRobot)
