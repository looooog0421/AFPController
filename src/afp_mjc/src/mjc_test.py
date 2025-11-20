import mujoco_py
import numpy as np
import time


if __name__ == "__main__":
    model = mujoco_py.load_model_from_xml("path_to_your_model.xml")
    sim = mujoco_py.MjSim(model)
    viewer = mujoco_py.MjViewer(sim)

    for _ in range(1000):
        sim.step()
        viewer.render()
        time.sleep(0.01)