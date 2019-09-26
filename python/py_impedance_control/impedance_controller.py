##################################################################################################################
## This is the implementation for impedance controller between any two frames of the robot. 
## This code works with just pybullet and does not depend on dynamic graph. 
## Primarly designed for designing and debuggin controllers
#################################################################################################################
## Author: Avadesh Meduri
## Date: 20/09/2019 
#################################################################################################################


import numpy as np
import pinocchio as pin
from pinocchio.utils import zero


class impedance_controller():
    
    def __init__(self, name, pin_robot, frame_root_name, frame_end_name, start_column):
        
        '''
        Input : 
            name : Name of the impdeance controller (Ex. Front left leg impedance)
            pifrom pinocchio
n_robot : pinocchio wrapper instance.
            frame_root_name : The root frame name where the spring starts(Ex. Hip)
            frame_end_name : the second frame name where the spring ends(Ex. end effector)
            start_column : the column from where 3 columns from the jacobian are selected
        '''
        
        self.name = name  
        self.pin_robot = pin_robot  
        self.frame_root_name = frame_root_name 
        self.frame_end_name = frame_end_name 
        self.frame_root_idx = self.pin_robot.model.getFrameId(self.frame_root_name) 
        self.frame_end_idx = self.pin_robot.model.getFrameId(self.frame_end_name) 
        self.start_column = start_column
        
    def compute_forward_kinematics(self,q):
        '''
            Computes forward kinematics of all the frames and stores in data
        '''
        pin.framesForwardKinematics(self.pin_robot.model, self.pin_robot.data, q)
        
        
        
    def compute_distance_between_frames(self,q):
        '''
            Computes the distance between the two frames or computes the location 
            of frame_end with respect to frame_root
        '''
        self.compute_forward_kinematics(q)
        
        return (pin.SE3(self.pin_robot.data.oMf[self.frame_root_idx].inverse()) * \
                                                    (pin.SE3(self.pin_robot.data.oMf[self.frame_end_idx]))).translation
        # return pin.SE3(self.pin_robot.data.oMf[self.frame_root_idx]).translation - pin.SE3(self.pin_robot.data.oMf[self.frame_end_idx]).translation
                                                    
    def compute_relative_velocity_between_frames(self,q,dq):
        '''
            computes the velocity of the end_frame with respect to the root frame
        '''
        self.compute_forward_kinematics(q)

        T_hip_foot = pin.SE3(self.pin_robot.data.oMf[self.frame_root_idx].inverse()) * \
                                                    (pin.SE3(self.pin_robot.data.oMf[self.frame_end_idx]))
        
        jac = T_hip_foot.action * pin.frameJacobian(self.pin_robot.model, self.pin_robot.data, q, self.frame_end_idx)
        
        return np.matmul(jac,dq)[0:3]
        
    def compute_jacobian(self,q):
        '''
            computes the jacobian with respect to the root_frame for the end_effector frame
            Math : J = Adj_(HFE, Foot) * J_(Foot frame)
            Selection of the required portion of the jacobian 
        '''
        
        T_hip_foot = pin.SE3(self.pin_robot.data.oMf[self.frame_root_idx]).inverse() * \
                                                    pin.SE3(self.pin_robot.data.oMf[self.frame_end_idx])
        
        ### SE3 matrix that maps point in the end_frame to the root_frame
        
        return np.matmul((T_hip_foot.action), pin.frameJacobian(self.pin_robot.model, self.pin_robot.data, q, self.frame_end_idx))
     
     
    def jacobian_selector(self, jac):
        '''
            select the part of the jacobian neccessary to compute the force 
        '''    
        
        return jac[:,self.start_column:self.start_column+3][0:3]
        
        
    def compute_impedance_torques(self, q, dq, kp, kd, x_des, xd_des, f):
        '''
            Computes the desired joint torques tau = Jt * (F + kp(x-x_des) + kd(xd-xd_des))
            Inputs:
                q = joint angles
                dq = joint velocites
                Kp = proportional gain
                Kd = derivative gain
                x_des = desired [x,y,z] at time t (in the root joint frame)
                xd_des = desired velocity of end effector at time t (in the root joint frame)
                
        '''

        ### Write the assert statements here 


        #### Reshaping values to desired shapes
        
        x_des = np.matrix(x_des).T
        xd_des = np.matrix(xd_des).T
        f = np.matrix(f).T
        kp = np.matrix([[kp[0],0,0],
                        [0,kp[1],0],
                        [0,0,kp[2]]])
        
        kd = np.matrix([[kd[0],0,0],
                        [0,kd[1],0],
                        [0,0,kd[2]]])
        
        #######################################

        self.compute_forward_kinematics(q)
        x = self.compute_distance_between_frames(q)
        xd = self.compute_relative_velocity_between_frames(q,dq)
        # print(xd[2])  

                                                    
        jac = self.compute_jacobian(q)
        jac_sel = self.jacobian_selector(jac)
        # tau = jac_sel.T * (f + kp*(x - x_des))
        tau = np.matmul(jac_sel.T ,(kp*(x - x_des)))

        return  tau
    
        

        
        
        
        