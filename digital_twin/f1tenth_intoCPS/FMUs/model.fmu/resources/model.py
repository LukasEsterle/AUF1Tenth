from os import access
import pickle
import math
from xml.sax.handler import DTDHandler

#Reference: https://github.com/f1tenth/f1tenth_simulator/blob/master/src/st_kinematics.cpp

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

        #parameters
        self.threshold = 0.5        #cut off to avoid singular behaviour 
        self.error = 0.03           #deadband to avoid flip-flop 

        #inputs
        self.acceleration = 0.0 
        self.steer_angle_vel = 0.0  

        #car state variables
        self.x = 0.0                #coordinates x and y is defined as output in order to track the position 
        self.y = 0.0
        self.theta = 0.0
        self.angular_velocity = 0.0
        self.slip_angle = 0.0
        self.st_dyn = False
        self.velocity = 0.0         #output
        self.steer_angle = 0.0      #output


        self.reference_to_attribute = {
            0: "threshold",
            1: "error", 
            2: "acceleration",           
            3: "steer_angle_vel",
            4: "velocity",
            5: "steer_angle",
            6: "x",
            7: "y",
            8: "theta",
        }

        #references to the inputs and outputs 
        self.references_input = [2, 3]
        self.references_output = [4, 5, 8]

        #getting the values 
        input = self.fmi2GetReal(self.references_input)

        self.acceleration = input[1][0]
        self.steer_angle_vel = input[1][1]

    
    #------PROCESS IN TIME------ 
    def fmi2DoStep(self, current_time, step_size, no_step_prior):
        #if velocity is low or negative, use normal kinematic single track dynamics 
        if self.velocity < self.threshold:
            self.update_k(step_size)
        
        else:
            self.update(step_size)

        self.steer_angle = min(max(self.steer_angle, -self.max_steering_angle), self.max_steering_angle)
        self.velocity = min(max(self.velocity, -self.max_speed),self.max_speed)

        #updating the output values
        self.fmi2SetReal(self.references_output, (self.velocity, self.steer_angle, self.x, self.y, self.theta))

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


    
    #------UPDATING THE STATE------
    def update(self, step_size):

        #computing first derivatives of the state
        x_dot = self.velocity * math.cos(self.theta + self.slip_angle)
        y_dot = self.velocity * math.sin(self.theta + self.slip_angle)
        velocity_dot = self.acceleration
        steer_angle_dot = self.steer_angle_vel
        theta_dot = self.angular_velocity

        #for eases of next two calculations
        rear_val = self.g * self.l_r - self.acceleration * self.h_cg
        front_val = self.g * self.l_f + self.acceleration * self.h_cg


        vel_ratio = 0
        first_term = 0

        #if velocity is zero
        if self.velocity == 0:
            vel_ratio = 0
            first_term = 0
        
        else:
            vel_ratio = self.angular_velocity / self.velocity
            first_term = self.friction_coeff / (self.velocity * (self.l_r + self.l_f))


        theta_double_dot = (self.friction_coeff * self.mass / (self.I_z * self.wheelbase)) * \
            (self.l_f * self.cs_f * self.steer_angle * (rear_val) + self.slip_angle 
            * (self.l_r * self.cs_r * (front_val) - self.l_f * self.cs_f * (rear_val)) 
            - vel_ratio * (self.l_f**2 * self.cs_f * (rear_val) + self.l_r**2 * self.cs_r * (front_val)))

        
        slip_angle_dot = (first_term) * (self.cs_f * self.steer_angle * (rear_val) - self.slip_angle * 
        (self.cs_r * (front_val) + self.cs_f * (rear_val)) +  vel_ratio * (self.cs_r * self.l_r * 
        (front_val) - self.cs_f * self.l_f * (rear_val))) - self.angular_velocity


        #updating the state
        self.x = self.x + x_dot * step_size
        self.y = self.y + y_dot * step_size
        self.theta = self.theta + theta_dot * step_size
        self.velocity = self.velocity + velocity_dot * step_size
        self.steer_angle = self.steer_angle + steer_angle_dot * step_size
        self.angular_velocity = self.angular_velocity + theta_double_dot * step_size
        self.slip_angle = self.slip_angle + slip_angle_dot * step_size
        self.st_dyn = True

    
    #------normal kinematic------
    def update_k(self, step_size):
        #computing first derivative of state
        x_dot = self.velocity * math.cos(self.theta)
        y_dot = self.velocity * math.sin(self.theta)
        velocity_dot = self.acceleration
        slip_angle_dot = 0
        steer_angle_dot = self.steer_angle_vel
        theta_dot = self.velocity / self.wheelbase * math.tan(self.steer_angle)
        theta_double_dot = self.acceleration / self.wheelbase * math.tan(self.steer_angle) + \
            self.velocity * self.steer_angle_vel / (self.wheelbase * math.cos(self.steer_angle)**2)

    
        #updating the state
        self.x = self.x + x_dot * step_size
        self.y = self.y + y_dot * step_size
        self.theta = self.theta + theta_dot * step_size
        self.velocity = self.velocity + velocity_dot * step_size
        self.steer_angle = self.steer_angle + steer_angle_dot * step_size
        self.angular_velocity = 0      #start.angular_velocity + theta_double_dot * dt;
        self.slip_angle = 0            #start.slip_angle + slip_angle_dot * dt;
        self.st_dyn = False


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



#------TEST CASES------
"""
if __name__ == "__main__":
    m = Model()
    m.acceleration = 3.0
    m.steer_angle_vel = 0.0
    m.fmi2DoStep(0.0, 4.0, False)
    print(m.x)
    print(m.y)
    print(m.theta)
    print(m.angular_velocity)
    print(m.slip_angle)
    print(m.velocity)
    print(m.steer_angle)
"""