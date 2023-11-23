'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle

PATH = 'robot_pose.pkl'

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = None  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def perception_get_prediction_input(self, perception):
        percieved_values = []
        participating_joints = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
        # XAngle
        # YAngle
        joint = perception.joint

        for key in participating_joints:
            val = joint[key]
            percieved_values.append(val)
        
        percieved_values.append(perception.gyr[0])
        percieved_values.append(perception.gyr[1])

        return percieved_values
    
    def num_to_posture(self, num):
        postures = ['Back', 'Belly', 'Crouch', 'Frog', 'HeadBack', 'Knee', 'Left', 'Right', 'Sit', 'Stand', 'StandInit']
        return postures[num]


    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE^
        
        with open(PATH, 'rb') as file:
            self.posture_classifier = pickle.load(file)
        
        percieved_values = [self.perception_get_prediction_input(perception)]
        prediction = self.posture_classifier.predict(percieved_values)[0]
        posture = self.num_to_posture(prediction)

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
