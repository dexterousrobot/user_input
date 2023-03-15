import tkinter as tk

class Slider:
    def __init__(self,
        init=[0, 0, 0, 0, 0, 0],
        label_names = ["x", "y", "z", "Rx", "Ry", "Rz"],
        llims=[-5, -5, 0, -15, -15, -180],
        ulims=[ 5,  5, 5,  15,  15,  180]
    ):    
        self.ids = []
        self.tk = tk.Tk()
        self.tk.geometry("300x500+200+0")
        for i, label_name in enumerate(label_names):
            self.ids.append(
                tk.Scale(self.tk, from_=llims[i], to=ulims[i],
                    label=label_name, length=400, orient=tk.HORIZONTAL, 
                    tickinterval=(ulims[i]-llims[i])/4, resolution=0.1
                )
            )
            self.ids[i].pack()
            self.ids[i].set(init[i])

    def read(self):
        self.tk.update_idletasks()
        self.tk.update()
        values = []
        for id in self.ids:
            values.append(id.get())
        return values

# within tactile gym
class Slider_sim:
    def __init__(self,
        embodiment, 
        init=[0, 0, 0, 0, 0, 0],
        label_names = ["x", "y", "z", "Rx", "Ry", "Rz"],
        llims=[-5, -5, 0, -15, -15, -180],
        ulims=[ 5,  5, 5,  15,  15,  180]
    ):    
        self._pb = embodiment.controller._client._sim_env._pb
        self.ids = []
        for i, label_name in enumerate(label_names):
            self.pose_ids.append(
                self._pb.addUserDebugParameter(
                    label_name, llims[i], ulims[i], init[i]
                )
            )

    def read(self, values):
        values = []
        for id in self.ids:
            values[i] = self._pb.readUserDebugParameter(id) 
        return values
