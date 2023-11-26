'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from jsonrpclib.SimpleJSONRPCServer import SimpleJSONRPCServer
import threading

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    
    def __init__(self):
        # Initialize the base InverseKinematicsAgent
        super(ServerAgent, self).__init__()
        # Set up the RPC server
        self.server = SimpleJSONRPCServer(('localhost', 8000))
        self.register_functions()
        print("works1")
        
        # Start the RPC server in a separate thread
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.start()
        print("works2")

    def register_functions(self):
        '''Register functions to be exposed through RPC'''
        self.server.register_function(self.get_angle, 'get_angle')
        self.server.register_function(self.set_angle, 'set_angle')
        self.server.register_function(self.get_posture, 'get_posture')
        self.server.register_function(self.execute_keyframes, 'execute_keyframes')
        self.server.register_function(self.get_transform, 'get_transform')
        self.server.register_function(self.set_transform, 'set_transform')
        print("everything registered")
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.posture[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        print("execute_keyframes: ")
        self.keyframes = keyframes

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        joint_name = name
        return self.transforms[joint_name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transforms(effector_name, transform)

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()