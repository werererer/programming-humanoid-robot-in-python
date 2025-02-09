'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from scipy.interpolate import CubicSpline

class AngleInterpolationAgent(PIDAgent):
    
    time_offset = float('nan')

    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        # YOUR CODE HERE
        names, times, keys = keyframes
        # print("names: ", names)
        # print("times: ", times)
        # print("keys: ", keys)
        target_joints = {}
        
        import math
        if math.isnan(self.time_offset):
            self.time_offset = perception.time
        
        time = max(perception.time - self.time_offset, 0)

        # Gehe jedes Gelenk durch
        for joint_index, joint_name in enumerate(names):
            # Extrahiere die Zeitpunkte und Winkel für das aktuelle Gelenk
            joint_times = times[joint_index]
            joint_keys = [key[0] for key in keys[joint_index]]

            # Erstelle einen kubischen Spline für das aktuelle Gelenk
            spline = CubicSpline(joint_times, joint_keys, bc_type='natural')
            # they should end with y' = 0 and y'' = 0

            # Berechne den aktuellen Winkel für dieses Gelenk
            # print("perception.time: ", time, " current_time: ", min(max(time, joint_times[0]), joint_times[-1]))
            current_time = min(max(time, joint_times[0]), joint_times[-1])
            if time > 12:
                self.time_offset = perception.time
            target_angle = spline(current_time)
            target_joints[joint_name] = target_angle

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
