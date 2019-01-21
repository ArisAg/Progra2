import wpilib
import math


class SharpIR2Y0A02:
    """
        Sharp GP2Y0A02YK0F is an analog IR sensor capable of measuring
        distances from 20cm to 150cm. Output distance is measured in
        centimeters.
        
        Distance is calculated using the following equation derived from
        the graph provided in the datasheet::
        
            62.28*x ^ -1.092
            
        .. warning:: FRC Teams: the case on these sensors is conductive and
                     grounded, and should not be mounted on a metallic
                     surface!
    """

    def __init__(self, port):
        """:param port: Analog port number"""
        self.distance = wpilib.AnalogInput(1)

    def getDistance(self):
        """
            :returns: distance in centimeters. The output is constrained to
                      be between 22.5 and 145
        """

        # Don't allow zero/negative values
        v = max(self.distance.getVoltage(), 0.00001)
        d = 62.28 * math.pow(v, -1.092)

        # Constrain output
        return max(min(d, 145.0), 22.5)
