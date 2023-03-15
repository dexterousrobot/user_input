import numpy as np

try:
    import pyspacemouse
except:
    print('To use SpaceMouse pyspacemouse needs installing')


LEFT, RIGHT, FORE, BACK, SHIFT, CTRL, QUIT \
    = 65295, 65296, 65297, 65298, 65306, 65307, ord('Q')


class ManualControl:
    def __init__(self, embodiment):    
        self.embodiment = embodiment
        try:
            pyspacemouse.open()
        except:
            print('no spacemouse')

    def keyboard(self,
        pose_init = [0.0, 0, 0, 0, 0, 0]
    ):
        pose = np.array([0, 0, 0, 0, 0, 0]) + pose_init # stop keeping state

        keys = self.embodiment.controller._client._sim_env._pb.getKeyboardEvents()
        if CTRL in keys:
            if FORE in keys:  pose -= [0, 0, 0, 0, 1, 0]
            if BACK in keys:  pose += [0, 0, 0, 0, 1, 0]
            if RIGHT in keys: pose -= [0, 0, 0, 1, 0, 0]
            if LEFT in keys:  pose += [0, 0, 0, 1, 0, 0]
        elif SHIFT in keys:
            if FORE in keys:  pose -= [0, 0, 1, 0, 0, 0]
            if BACK in keys:  pose += [0, 0, 1, 0, 0, 0]
            if RIGHT in keys: pose -= [0, 0, 0, 0, 0, 2.5]
            if LEFT in keys:  pose += [0, 0, 0, 0, 0, 2.5]
        else:
            if FORE in keys:  pose -= [1, 0, 0, 0, 0, 0]
            if BACK in keys:  pose += [1, 0, 0, 0, 0, 0]
            if RIGHT in keys: pose -= [0, 1, 0, 0, 0, 0]
            if LEFT in keys:  pose += [0, 1, 0, 0, 0, 0]
        if QUIT in keys:  pose = None

        return np.array(pose)

    def spacemouse(self,
        pose_init = [0, 0, 0, 0, 0, 0], 
        gain = 2, 
        sign = [-1, -1, -1, -1, 1, -1]
    ):
        state = pyspacemouse.read()
        pose_state = np.array([state[i]*sign[i-1] for i in [2, 1, 3, 4, 5, 6]])
        pose = pose_init + gain * pose_state

        return pose
