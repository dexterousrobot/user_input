import tkinter as tk
POSE_LABEL_NAMES = ["x", "y", "z", "Rx", "Ry", "Rz"]


class Slider:
    def __init__(self,
        init_pose=[0, 0, 0, 0, 0, 0],
        pose_llims=[-5, -5, 0, -15, -15, -180],
        pose_ulims=[ 5,  5, 5,  15,  15,  180]
    ):    
        self.pose_ids = []
        self.tk = tk.Tk()
        self.tk.geometry("300x500+200+0")
        for i, label_name in enumerate(POSE_LABEL_NAMES):
            self.pose_ids.append(
                tk.Scale(self.tk, from_=pose_llims[i], to=pose_ulims[i],
                    label=label_name, length=400, orient=tk.HORIZONTAL, 
                    tickinterval=(pose_ulims[i]-pose_llims[i])/4, resolution=0.1
                )
            )
            self.pose_ids[i].pack()
            self.pose_ids[i].set(init_pose[i])

    def read(self, pose):
        self.tk.update_idletasks()
        self.tk.update()
        for i in range(len(pose)):
            pose[i] = self.pose_ids[i].get()
        return pose

# within tactile gym
class Slider_sim:
    def __init__(self,
        embodiment, 
        init_pose=[0, 0, 0, 0, 0, 0],
        pose_llims=[-5, -5, 0, -15, -15, -180],
        pose_ulims=[ 5,  5, 5,  15,  15,  180]
    ):    
        self.embodiment = embodiment
        self.pose_ids = []
        for i, label_name in enumerate(POSE_LABEL_NAMES):
            self.pose_ids.append(
                embodiment.controller._client._sim_env._pb.addUserDebugParameter(
                    label_name, pose_llims[i], pose_ulims[i], init_pose[i]
                )
            )

    def read(self, pose):
        for i in range(len(pose)):
            pose[i] = self.embodiment.controller._client._sim_env._pb.readUserDebugParameter(
                self.pose_ids[i]
            ) 
        return pose
