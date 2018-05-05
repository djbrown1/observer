class controller():    
    """PID controller with a Luenberger observer."""
    
    def __init__(self,Kp, Ki, Kd, MV_bar=0, beta=1, gamma=0):
        from numpy import matrix
        
        """ Initialize the PID aspect of the controller """
        # Define the proportional, integral, 
        # and derivative coefficients
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        # Define the reference value of MV
        self.MV_bar = MV_bar
        
        # Define the coefficients for setpoint weighting
        self.beta = beta
        self.gamma = gamma
        
        self.eD_prev = 0
        self.t_prev = 0
        
        # Initialize the PID variables
        self.P = 0
        self.I = 0
        self.D = 0
        
        """ Initialize the Luenberger observer 
        aspect of the controller """
        # set the model parameters for the observer
        self.A = matrix([[-0.01546814,0.00639784],
                         [0.03924884,-0.03924884]])
        self.B = matrix([[5.71428571429e-3],[0]])
        self.C = matrix([[0,1]])
        self.L = matrix([[1],[1]])
        
        # Use the observer by default
        self.use_observer = True
        
        # Initialize the observer variable
        self.x = matrix([[0],[0]])
        
    def control(self, t, PV, SP, Q1):
        """PID control function."""

        # Apply observer
        PV = self.observer_func(PV, Q1, t)
        
        # Tracking
        self.I = self.MV - self.MV_bar - self.P - self.D
        
        # Setpoint Weighting
        eD = self.gamma*SP - PV
        eP = self.beta*SP - PV
        
        # PID Calculations
        self.P = self.Kp*eP
        self.I = self.I + self.Ki*(SP - PV)*(t - self.t_prev)
        self.D = self.Kd*(eD - self.eD_prev)/(t - self.t_prev)
        
        MV = self.MV_bar + self.P + self.I + self.D
        
        # Set values for next iteration
        self.eD_prev = eD
        self.t_prev = t
        
        # anti-reset windup
        self.MV = 0 if MV < 0 else 100 if MV > 100 else MV
        
        # return control value for manipulated variable
        return self.MV
        
    def observer_func(self, y, u, t):
        """Apply observer for the given model."""
        from numpy import matmul
        
        term1 = matmul(self.A,self.x)
        term2 = matmul(self.L,(y - matmul(self.C,self.x)))
        term3 = self.B*u
        
        d_est = term1 + term2 + term3
        
        self.x = self.x + (d_est*(t - self.t_prev))
        
        PV = matmul(self.C,self.x)[0,0]
        return PV
    

class observer():
    """Luenberger state observer"""
    def __init__(self, x_0, t_0):
        """ Initialize the coefficients
        and parameters for the observer """
        # set the model parameters for the observer
        self.A = matrix([[-0.01546814,0.00639784],
                         [0.03924884,-0.03924884]])
        self.B = matrix([[5.71428571429e-3],[0]])
        self.C = matrix([[0,1]])
        self.L = matrix([[1],[0.2]])

        self.x = x_0
        self.t_prev = t_0

    def observer_func(self, y, u, t):
        """Apply observer for the given model."""
        from numpy import matmul

        term1 = matmul(self.A,self.x)
        term2 = matmul(self.L,(y - matmul(self.C,self.x)))
        term3 = self.B*u

        d_est = term1 + term2 + term3

        self.x = self.x + (d_est*(t - self.t_prev))
        
        # Returns the current state of the TCLab
        return self.x