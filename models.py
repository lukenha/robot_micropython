class PID_parameters:
    def __init__(self, proportional_gain, integrative_gain, derivative_gain, reference, last_error=0):
        self.proportional_gain = proportional_gain
        self.integrative_gain = integrative_gain
        self.derivative_gain = derivative_gain

        self.reference = reference

        self.last_error = last_error 
