import weakref
import threading
from jsonrpclib import Server
import os
import sys

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from keyframes import hello

class ClientAgent(object):
    '''ClientAgent requests RPC services from a remote server'''

    def __init__(self, server_url):
        # Initialize the JSON-RPC server connection
        self.rpc_server = Server(server_url)
        # Create a PostHandler instance for non-blocking calls
        self.post = PostHandler(self)
    
    # Method to get the angle of a joint
    def get_angle(self, joint_name):
        # Remote procedure call to get joint angle
        return self.rpc_server.get_angle(joint_name)
    
    # Method to set the angle of a joint
    def set_angle(self, joint_name, angle):
        # Remote procedure call to set joint angle
        self.rpc_server.set_angle(joint_name, angle)

    # Method to get the current posture of the robot
    def get_posture(self):
        # Remote procedure call to get the robot's posture
        return self.rpc_server.get_posture()

    # Method to execute keyframes (blocking call)
    def execute_keyframes(self, keyframes):
        # Remote procedure call to execute keyframes
        self.rpc_server.execute_keyframes(keyframes)

    # Method to get a transform with a given name
    def get_transform(self, name):
        # Remote procedure call to get a transform
        return self.rpc_server.get_transform(name)

    # Method to set a transform for an effector
    def set_transform(self, effector_name, transform):
        # Remote procedure call to set a transform
        self.rpc_server.set_transform(effector_name, transform)

class PostHandler(object):
    '''PostHandler provides non-blocking function calls'''

    def __init__(self, client_agent):
        # Store a weak reference to the client agent to avoid circular references
        self.proxy = weakref.proxy(client_agent)

    # Non-blocking method to execute keyframes
    def execute_keyframes(self, keyframes):
        # Start a new thread for the blocking execute_keyframes call
        threading.Thread(target=self.proxy.execute_keyframes, args=(keyframes,)).start()

    # Non-blocking method to set a transform
    def set_transform(self, effector_name, transform):
        # Start a new thread for the blocking set_transform call
        threading.Thread(target=self.proxy.set_transform, args=(effector_name, transform)).start()

if __name__ == '__main__':
    print("works1")
    server_url = 'http://localhost:8000'  # Replace with your server URL
    print("works2")
    agent = ClientAgent(server_url)
    print("works3")

    # Example usage:
    keyframes = hello()
    print("works4", keyframes)
    agent.post.execute_keyframes(keyframes)
    print("works5")
    # agent.post.set_transform(effector_name, transform)