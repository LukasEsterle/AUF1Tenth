from os import access
import pickle
import math
from xml.sax.handler import DTDHandler
import numpy as np
from scipy.integrate import odeint

"""
Uncented Kalman Filter FMU 
The code is form Luminita C. Totu's lectures at AiRLab Skejby.
Modified for use in this project.
"""

class Model:
    def __init__(self) -> None:

        #constants
        self.g = 9.82                #gravity in m/s²
        self.wheelbase = 0.3302
        self.friction_coeff = 0.523
        self.h_cg = 0.074 
        self.l_f = 0.15875 
        self.l_r = 0.17145 
        self.cs_f = 4.718
        self.cs_r = 5.4562
        self.mass = 3.47
        self.I_z = 0.04712
        self.max_steering_angle = 0.4189
        self.max_speed = 7.

        # measured states
        self.x_s = 0.
        self.y_s = 0.
        self.steer_angle_s = 0.
        self.velocity_s = 0.
        self.theta_s = 0.

        # inputs
        self.acceleration = 0.
        self.steer_angel_velocity = 0.

        # output predicted states
        self.x_p = 0.
        self.y_p = 0.
        self.steer_angle_p = 0.
        self.velocity_p = 0.
        self.theta_p = 0.

        #states
        #x0 = x-position in a global coordinate system
        #x1 = y-position in a global coordinate system
        #x2 = steering angle of front wheels
        #x3 = velocity in x-direction
        #x4 = yaw angle
        #x5 = yaw rate
        #x6 = slip angle at vehicle center

        #inputs
        #u0 = steering angle velocity of front wheels
        #u1 = longitudinal acceleration

        """ Continous state dynamics; dot(x) =  f(x,u) """
        self.f = lambda x, t, u: np.array([x[3]*math.cos(x[6] + x[4]), 
            x[3]*math.sin(x[6] + x[4]), 
            u[0], 
            u[1], 
            x[5], 
            0, 
            0])
        """
        f = lambda x, t, u: np.array([x[3]*math.cos(x[6] + x[4]), 
            x[3]*math.sin(x[6] + x[4]), 
            u[0], 
            u[1], 
            x[5], 
            (self.friction_coeff * self.mass / (self.I_z * self.wheelbase)) * \
            (self.l_f * self.cs_f * x[2] * (self.g * self.l_r - u[1] * self.h_cg) + x[6] 
            * (self.l_r * self.cs_r * (self.g * self.l_f + u[1] * self.h_cg) - self.l_f * self.cs_f * (self.g * self.l_r - u[1] * self.h_cg)) 
            - (x[5] / x[3]) * (self.l_f**2 * self.cs_f * (self.g * self.l_r - u[1] * self.h_cg) + self.l_r**2 * self.cs_r * (self.g * self.l_f + u[1] * self.h_cg))), 
            (self.friction_coeff / (x[3] * (self.l_r + self.l_f))) * (self.cs_f * x[2] * (self.g * self.l_r - u[1] * self.h_cg) - x[6] * 
            (self.cs_r * (self.g * self.l_f + u[1] * self.h_cg) + self.cs_f * (self.g * self.l_r - u[1] * self.h_cg)) +  (x[5] / x[3]) * (self.cs_r * self.l_r * 
            (self.g * self.l_f + u[1] * self.h_cg) - self.cs_f * self.l_f * (self.g * self.l_r - u[1] * self.h_cg))) - x[5]])
        """


        self.h = lambda x: x[0:5]

        self.n = 7
        self.G = np.eye(self.n)
        self.Q = np.diag([0.1*0.1, 0.1*0.1, 0.01*0.01, 0.1*0.1, 0.1*0.1, 0.01*0.01, 0.1*0.1])
        self.R = np.diag([0.04*0.04, 0.04*0.04, 0.04*0.04, 0.04*0.04, 0.04*0.04])
        self.x0_est = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
        self.P0 = np.diag([0.1*0.1, 0.1*0.1, 0.01*0.01, 0.1*0.1, 0.1*0.1, 0.01*0.01, 0.1*0.1])
        self.Q = self.Q + np.diag([10**-4, 10**-4, 10**-9, 10**-9, 10**-9, 10**-9, 10**-9])

        # Sigma Point Kalman Filterinput[1][0]
        self.alpha = 0.0001
        self.kappa = 0.0
        self.beta = 2.0
        self.sut = SUT(self.alpha, self.beta, self.kappa, self.n)
        self.filter = SPKF(self.f,self.h,self.G,self.Q,self.R,self.x0_est,self.P0,self.sut,variant=0)  # 0 - normal UKF, 1 - IUKF, 2 - UKFz
        self.meas = [0, 0, 0, 0, 0]
        self.u = [0, 0]

        self.reference_to_attribute = {
            0: "x_s",
            1: "y_s",
            2: "steer_angle_s",
            3: "velocity_s",
            4: "theta_s",            
            5: "acceleration",
            6: "steer_angel_velocity",
            7: "x_p",
            8: "y_p",
            9: "steer_angle_p",
            10: "velocity_p",
            11: "theta_p"
        }

        #references to the inputs and outputs 
        self.references_input = [0, 1, 2, 3, 4, 5, 6]
        self.references_output = [7, 8]

        #getting the values 
        input = self.fmi2GetReal(self.references_input)

        self.x_s = input[1][0]
        self.y_s = input[1][1]
        self.steer_angle_s = input[1][2]
        self.velocity_s = input[1][3]
        self.theta_s = input[1][4]
        self.acceleration = input[1][5]
        self.steer_angel_velocity = input[1][6]
        
    
    #------PROCESS IN TIME------ 
    def fmi2DoStep(self, current_time, step_size, no_step_prior):
        self.meas = [self.x_s, self.y_s, self.steer_angle_s, self.velocity_s, self.theta_s]
        self.u = [self.steer_angel_velocity, self.acceleration]
        self.filter.predict(self.u, step_size,1)
        self.filter.update(self.meas,0)

        self.x_p = self.filter.x[0]
        self.y_p = self.filter.x[1]
        self.steer_angle_p = self.filter.x[2] 
        self.velocity_p = self.filter.x[3]
        self.theta_p = self.filter.x[4]
        
        #updating the output values
        self.fmi2SetReal(self.references_output, (self.x_p, self.y_p, self.steer_angle_p, self.velocity_p, self.theta_p))

        return Fmi2Status.ok


    #------INITIALIZATION------ 
    def fmi2EnterInitializationMode(self):
        return Fmi2Status.ok

    def fmi2ExitInitializationMode(self):
        return Fmi2Status.ok


    #------SETUP EXPERIMENT------ 
    def fmi2SetupExperiment(self, start_time, stop_time, tolerance):
        return Fmi2Status.ok
    
    def fmi2Reset(self):
        return Fmi2Status.ok

    def fmi2Terminate(self):
        return Fmi2Status.ok


    #------SETTERS------ 
    def fmi2SetReal(self, references, values):
        return self._set_value(references, values)

    def fmi2SetInteger(self, references, values):
        return self._set_value(references, values)

    def fmi2SetBoolean(self, references, values):
        return self._set_value(references, values)

    def fmi2SetString(self, references, values):
        return self._set_value(references, values)
    
    def _set_value(self, references, values):

        for r, v in zip(references, values):
            setattr(self, self.reference_to_attribute[r], v)

        return Fmi2Status.ok
    

    #------GETTERS------ 
    def fmi2GetReal(self, references):
        return self._get_value(references)

    def fmi2GetInteger(self, references):
        return self._get_value(references)

    def fmi2GetBoolean(self, references):
        return self._get_value(references)

    def fmi2GetString(self, references):
        return self._get_value(references)
    
    def _get_value(self, references):

        values = []

        for r in references:
            values.append(getattr(self, self.reference_to_attribute[r]))

        return Fmi2Status.ok, values


  
class SUT: 
    """ Scaled Unscented Transform """

    def __init__(self, alpha, beta, kappa, n):
        
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.n = n

        self.l = (self.alpha**2)*(self.n+self.kappa) - self.n # lambda parameter 

        # Init weights (constants) 
        self.W0m = self.l/(self.n + self.l)
        self.W0c = self.W0m + 1 - self.alpha**2 + self.beta
        self.Wi = 0.5/(self.n + self.l)
        """ sigma points weights """
    
    def create_points(self, x, P):
        sr_P = np.linalg.cholesky((self.n + self.l)*P)
        S = [x]
        for i in range(self.n):
            S.append(x+sr_P[:,i])
            S.append(x-sr_P[:,i])
        return S
##########################################################

#class CDT:       
#  """ Central Difference Transform """
##########################################################

class SPKF:
    """ Sigma Point Kalman Filter """

    def __init__(self,f,h,G,Q,R,x0,P0,spt,variant=0):

        self.f = f
        """ Continous state dynamics; dot(x) =  f(x,u) """

        self.h = h
        """ Measurement function y = h(x) """

        self.G = G
        """ Noise input matrix """

        self.x = x0
        """ Initial State """

        self.P = P0
        """ Initial Covariance """

        self.Q = Q
        """ Propagation noise matrix """

        self.R = R
        """ Measurement noise matrix """

        self.spt = spt
        """ Sigma point transform and parameters """

        self.n = x0.size
        """ State dimensionality """

        self.variant = variant # 0 - normal UKF, 1 - IUKF, 2 - UKFz

    def predict(self,u,dt,simple = 1):

        S = self.spt.create_points(self.x, self.P)
  
        # propagate the points 
        Sp = [ ]
        for i in range(len(S)):
            if (simple):
                # Euler faster
                Sp.append(S[i]+self.f(S[i],0,u)*dt)
            else:
                # ODE Int integration to make the state tranzition
                Y = odeint(self.f,S[i],np.array([0, dt]),args=(u,))
                Sp.append(Y[1]) # Y[0]=x(t=t0)
     
        # calculate the mean, covariance and cross-covariance of the set
        Xm = self.spt.W0m*Sp[0]
        for i in range(2*self.n):
            Xm = Xm + self.spt.Wi*Sp[i+1]

        Cx = self.spt.W0c*np.outer(Sp[0]-Xm,Sp[0]-Xm)+dt*self.Q
        for i in range(2*self.n):
            Cx = Cx + self.spt.Wi*np.outer(Sp[i+1]-Xm,Sp[i+1]-Xm)

        self.x = Xm
        self.P = Cx

    def update(self,meas,var=0):

        X = self.spt.create_points(self.x, self.P)
        
        Y = [ ]
        for i in range(2*self.n+1):
            Y.append(self.h(X[i]))

        # calculate the mean and covariance of the set
        if (self.variant == 2): # UKFz
            Ym = self.h(self.x)
        else:  
            Ym = self.spt.W0m*Y[0]
            for i in range(2*self.n):
                Ym = Ym + self.spt.Wi*Y[i+1]
        
        Cy = self.spt.W0c*np.outer(Y[0]-Ym,Y[0]-Ym)
        Cxy = self.spt.W0c*np.outer(X[0]-self.x,Y[0]-Ym)
        for i in range(2*self.n):
            Cy = Cy + self.spt.Wi*np.outer(Y[i+1]-Ym,Y[i+1]-Ym)
            Cxy = Cxy + self.spt.Wi*np.outer(X[i+1]-self.x,Y[i+1]-Ym)

        # Kalman Gain
        K = Cxy@np.linalg.inv(Cy + self.R)

        # Update mean and covarince
        if (self.variant == 1): # IUKF
            inn = meas - self.h(self.x)
        else:
            inn = meas - Ym

        self.x = self.x + K@inn
        
        if (var == 0):
            # Simple Covariance Update
            self.P = self.P - K@(Cy+self.R)@np.transpose(K)
        else:
            # -> Joseph Form Covariance Update
            # H = stochastic linearization derived from the fact that Cxy = PH' in the linear case [Skoglund, Gustafsson, Hendeby - 2019]
            self.H = Cxy.transpose()@np.linalg.inv(self.P)
            IKH =  np.eye(self.n) - K@self.H
            self.P = IKH@self.P@IKH.transpose()+K@self.R@K.transpose() # Joseph Form
##########################################################

#------Represents the status of the FMU or the results of function calls------
class Fmi2Status:
    """
    Values:
        * ok: all well
        * warning: an issue has arisen, but the computation can continue.
        * discard: an operation has resulted in invalid output, which must be discarded
        * error: an error has ocurred for this specific FMU instance.
        * fatal: an fatal error has ocurred which has corrupted ALL FMU instances.
        * pending: indicates that the FMu is doing work asynchronously, which can be retrived later.
    Notes:
        FMI section 2.1.3
    """

    ok = 0
    warning = 1
    discard = 2
    error = 3
    fatal = 4
    pending = 5


