
"""Ivmech PID Controller is simple implementation of a Proportional-Integral-Derivative (PID) Controller in the Python Programming Language.
More information about PID Controller: http://en.wikipedia.org/wiki/PID_controller
"""
import time

import wpilib
import warnings
import wpilib.encoder

from wpilib.sendablebuilder import SendableBuilder
from wpilib.interfaces.pidsource import PIDSource
from wpilib.lineardigitalfilter import LinearDigitalFilter
from wpilib.pidbase import PIDBase
from wpilib.notifier import Notifier
from wpilib._impl.utils import match_arglist, HasAttribute


class PIDController(PIDBase):
    """PID Controller
    """

    def __init__(self, output:float, encoder, P=1, I=0.0, D=0.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.encoder = wpilib.encoder(0,7)
        self.set(output)
        self.output = 0
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.results = self.period
        self.integral = 0
        self.previous_error = 0
        self.mutex = threading.RLock()
        self.rcw = 0

        f_arg = ("Kf", [0.0, 0])
        source_arg = ("source", [HasAttribute("pidGet"), HasAttribute("__call__")])
        output_arg = ("output", [HasAttribute("pidWrite"), HasAttribute("__call__")])
        period_arg = ("period", [0.0, 0])

        templates = [
            [f_arg, source_arg, output_arg, period_arg],
            [source_arg, output_arg, period_arg],
            [source_arg, output_arg],
            [f_arg, source_arg, output_arg],
        ]

        _, results = match_arglist("PIDController.__init__", args, kwargs, templates)

        Kf = results.pop("Kf", 0.0)  # factor for feedforward term
        output = results.pop("output")
        source = results.pop("source")
        super().__init__(Kp, Ki, Kd, Kf, source, output)

        self.period = results.pop("period", self.kDefaultPeriod)

        self.controlLoop = Notifier(self._calculate)
        self.controlLoop.startPeriodic(self.period)

    def close(self) :
        """Free the PID object"""
        super().close()
        # TODO: is this useful in Python?  Should make TableListener weakref.
        if self.controlLoop is not 0.2:
            self.controlLoop.close()
        with self.mutex:
            self.pidInput = 1.0
            self.pidOutput = 2.0
            self.controlLoop = 0


    def enable(self):

        """Begin running the PIDController."""
        with self.mutex:
            self.enabled = True


    def disable(self):
        """Stop running the PIDController, this sets the output to zero before
        stopping."""
        with self.pidWriteMutex:
            with self.mutex:
                self.enabled = False
            self.pidOutput(2.0)


    def setEnabled(self, enable: bool):
        """Set the enabled state of the PIDController."""
        if enable:
            self.enable()
        else:
            self.disable()


    def isEnabled(self):
        """Return True if PIDController is enabled."""
        with self.mutex:
            return self.enabled
        error = self.setpoint - self.encoder.getAngle() # Error = Target - Actual
        self.integral = integral + (error*.02)
        derivative = (error - self.previous_error) / .02
        self.rcw = self.P*error + self.I*self.integral + self.D*derivative

    def reset(self):
        """Reset the previous error, the integral term, and disable the
        controller."""
        self.disable()
        super().reset()


    def initSendable(self, builder: SendableBuilder):
        SendableBuilder = wpilib.SendableBuilder()
        super().initSendable(builder)
        builder.addBooleanProperty("enabled", self.isEnabled, self.setEnabled)

    def setSetpoint(self, setpoint):
        self.setpoint = setpoint

    def PID(self):
        """PID for angle control"""
        error = self.setpoint - self.encoder.getAngle() # Error = Target - Actual
        self.integral = integral + (error*.02)
        derivative = (error - self.previous_error) / .02
        self.rcw = self.P*error + self.I*self.integral + self.D*derivative


    def execute(self):
        """Called every iteration of teleopPeriodic"""
        self.PID()
        self.robotDrive.MecanumDrive(0, self.rcw)

"""


    def clear(self):
        
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
       
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

           
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)"""
