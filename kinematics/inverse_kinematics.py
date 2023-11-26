'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
from keyframes import hello
from enum import Enum
import numpy as np

class Rotation(Enum):
    YAW = 1
    ROLL = 2
    PITCH = 3

class InverseKinematicsAgent(ForwardKinematicsAgent):
    implemented_effector_names = ['LLeg', 'RLeg']

    def jacobian_yaw_pitch_column(self, x, y, z):
        return (self.jacobian_yaw_column(x, y, z) + self.jacobian_pitch_column(x, y, z)) / 2

    def jacobian_roll_column(self, x, y, z):
        # [[x_1,
        #   y_1,
        #   z_1,
        #   theta_x,
        #   theta_y,
        #   theta_z,]]
        return np.array([[0],
                [-z],
                [y],
                [1],
                [0],
                [0]])
    
    def jacobian_pitch_column(self, x, y, z):
        return np.array([[z],
                [0],
                [-x],
                [0],
                [1],
                [0]])  
    
    def jacobian_yaw_column(self, x, y, z):
        return np.array([[-y],
                [x],
                [0],
                [0],
                [0],
                [1]])
    
    def create_rotation_matrix_roll(self, theta):
        return np.matrix([[1, 0, 0],
                        [0, np.cos(theta), -np.sin(theta)],
                        [0, np.sin(theta), np.cos(theta)]])
    
    def create_rotation_matrix_pitch(self, theta):
        return np.matrix([[np.cos(theta), 0, np.sin(theta)],
                        [0, 1, 0],
                        [-np.sin(theta), 0, np.cos(theta)]])
    
    def create_rotation_matrix_yaw(self, theta):
        return np.matrix([[np.cos(theta), -np.sin(theta), 0],
                        [np.sin(theta), np.cos(theta), 0],
                        [0, 0, 1]])

    def encode_information_into_transform(self, x, y, z, theta_x, theta_y, theta_z):
        # Create rotation matrix
        rot_matrix = self.create_rotation_matrix_yaw(theta_z) *\
                        self.create_rotation_matrix_pitch(theta_y) *\
                        self.create_rotation_matrix_roll(theta_x)

        # Create a 4x4 identity matrix
        transform_matrix = np.eye(4)

        # Replace the top-left 3x3 portion with the rotation matrix
        transform_matrix[:3, :3] = rot_matrix

        # Set the translation
        transform_matrix[:3, 3] = [x, y, z]

        return transform_matrix
    
    def extract_theta_x(self, transform):
        return np.arctan2(transform[2, 1], transform[2, 2])
    
    def extract_theta_y(self, transform):
        return np.arctan2(-transform[2, 0], np.sqrt(transform[2, 1]**2 + transform[2, 2]**2))
    
    def extract_theta_z(self, transform):
        return np.arctan2(transform[1, 0], transform[0, 0])
    
    def extract_x_y_z_thetas(self, transform):
        x = transform[0, 3]
        y = transform[1, 3]
        z = transform[2, 3]
        theta_x = self.extract_theta_x(transform)
        theta_y = self.extract_theta_y(transform)
        theta_z = self.extract_theta_z(transform)
        return x, y, z, theta_x, theta_y, theta_z

    max_step = 0.1
    def inverse_kinematics(self, effector_name, transform, delta_target):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        # YOUR CODE HERE
        print("transform: ", transform)
        x, y, z, theta_x, theta_y, theta_z = self.extract_x_y_z_thetas(transform)
        print("x: ", x, " y: ", y, " z: ", z, " theta_x: ", theta_x, " theta_y: ", theta_y, " theta_z: ", theta_z)

        if (effector_name not in self.implemented_effector_names):
            raise NotImplementedError("Effector %s is not implemented" % effector_name)
        
        # http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       # http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
        # based on http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain LLeg and RLeg
        jacobian = np.concatenate((self.jacobian_yaw_pitch_column(x, y, z),
            self.jacobian_roll_column(x, y, z),
            self.jacobian_pitch_column(x, y, z),
            self.jacobian_pitch_column(x, y, z),
            self.jacobian_pitch_column(x, y, z),
            self.jacobian_roll_column(x, y, z)), axis=1)
        print("jacobin: ", jacobian)
        jacobian_inv = np.linalg.pinv(jacobian)
        print("jacobina_inv: ", jacobian_inv)
        print("jacobian_dim: ", jacobian.shape, " vs. ", jacobian_inv.shape)

        print("input_vector: ", delta_target)
        
        joint_angles = jacobian_inv * delta_target
        print("joint_angles: ", joint_angles)
        return joint_angles
    
    def my_keyframes(self, effector_name, anglesDiff):
        chain_joints = self.chains[effector_name]
        
        # remove first joint
        # chain_joints = chain_joints[1:]
        # create dictionary with joint names as keys and angles as values
        joint_angles = dict()
        for i in range(len(chain_joints)):
            joint_angles[chain_joints[i]] = anglesDiff[i, 0]

        print("joint_angles: ", joint_angles)

        keyframes = ([], [], [])

        names = list()
        times = list()
        keys = list()

        for joint_name, angle in joint_angles.items():
            names.append(joint_name)
            times.append([1.0, 2.0])
            keys.append([[0, [3, 0.0, 0.0], [3, 1.0, 0.0]], [angle, [3, 0.0, 0.0], [3, 1.0, 0.0]]])

        names.append("HeadPitch")
        times.append([0.80000, 1.56000, 2.24000, 2.80000, 3.48000, 4.60000])
        keys.append([[0.29602], [-0.17032], [-0.34059], [-0.05987], [-0.19333], [-0.01078]])

        keyframes = (names, times, keys)

        return keyframes
    
    def angle_to_percentages(self, angle):
        return angle / 2*np.pi 
    
    def percentages_to_angle(self, percentage):
        return percentage * 2*np.pi

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        effector_name = "LLeg"
        # encode current position into transform
        transform = self.encode_information_into_transform(0.0, 0.0, -1.0, 0, 0.0, 0)
        delta_target = np.matrix([[3],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0]])
        anglesDiff = self.inverse_kinematics(effector_name, transform, delta_target)
        percentages = self.angle_to_percentages(anglesDiff)
        self.keyframes = self.my_keyframes(effector_name, percentages)
        print("percentages: ", percentages)
        
        # print("anglesDiff: ", anglesDiff)

        # self.keyframes = hello()  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
