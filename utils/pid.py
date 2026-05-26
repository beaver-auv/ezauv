import time

# A PID controller is a piece of code which tries to bring a chaotic system to a specific
# value; eg. bring a sub to a specific depth or angle, maneuvering against the chaotic water.
# it is based on three parts; one Proportional, Integral, and Derivative (P-I-D). The final
# output signal is a weighted sum of the three parts. To find the proportional piece, you just
# take the error between the current value and the wanted value. The integral piece is the sum
# of all the previous errors. The derivative piece is more complicated, being the rate of change
# between the current and previous errors; so, (error - prev_error) / (time - prev_time).
# once you have these three pieces, the output is just a weighted sum of them, using
# Kp, Ki, and Kd respectively

# as of the 2023 competition, the parameters were
# Kp = 0.6
# Ki = 0.0
# Kd = 0.1
# these will probably (almost definitely) need to be retuned in the future


class PID:
    def __init__(self, proportional: float, integral: float, derivative: float, wanted: float, interval: float = None):
        """
        The first three parameters represent the weight of their respect part of the PID;
        Kp, Ki, and Kd. Wanted is the value the PID will attempt to reach, this cannot be
        changed without creating a new PID object. Interval is optional, if unset the PID
        will use real time, which is good in case of potential lag. If set, it will assume
        every call is `interval` seconds apart. It's good for testing, but in a real
        scenario use real time.
        """
        # the first three params represent the weight of their respective part of the PID
        # interval is optional, if unset the PID time will be based on actual time, which is 
        # good in case of potential lag. if set, it will assume every call is `interval`
        # seconds apart. it's good for testing, but in a real scenario use timestamp

        self.Kp = proportional
        self.Ki = integral
        self.Kd = derivative

        self.wanted = wanted

        self.interval = interval

        self.integral = 0
        # integral represents the total error

        self.prev = 0
        # prev represents the previous error

        self.prevTime = time.time()
        # only used if interval is unset

    def signal(self, current: float) -> float:
        """
        Get the control signal given the `current` value
        """
        # return the control signal, given a current value
        # this will update the saved data such as total error and previous error

        time_diff = self.interval if self.interval is not None else time.time() - self.prevTime
        error = self.wanted - current
        self.integral += error
        derivative = (error - self.prev) / time_diff

        self.prev = error
        signal = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        return signal