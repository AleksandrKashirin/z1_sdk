import sys
sys.path.append("/home/aleksandr/.local/share/ov/pkg/isaac_sim-2022.2.1/\
exts/omni.isaac.examples/omni/isaac/examples/user_examples/\
z1_manipulation/z1_sdk/lib")
import unitree_arm_interface
import time
import numpy as np
import json
import pathlib


# Collect data to analyze
pathlib.Path("/home/aleksandr/.local/share/ov/pkg/\
isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/\
isaac/examples/user_examples/z1_manipulation/\
data").mkdir(parents=True, exist_ok=True)
data = {"q": [], "qd": [], "tau": [], "dt": []}

print("Press ctrl+\ to quit process.")

np.set_printoptions(precision=3, suppress=True)
arm = unitree_arm_interface.ArmInterface(hasGripper=True)
armModel = arm._ctrlComp.armModel
arm.setFsmLowcmd()

duration = 1000
lastPos = arm.lowstate.getQ()

targetPos = np.array([0.0, 1.5, -1.0, -0.54, 0.0, 0.0])  # forward

for i in range(0, duration):
    q = lastPos*(1-i/duration) + targetPos*(i/duration)
    qd = (targetPos-lastPos)/(duration*0.002)
    (q, qd) = armModel.jointProtect(q, qd)
    tau = armModel.inverseDynamics(q, qd, np.zeros(6), np.zeros(6))  # set torque
    arm.q = q
    arm.qd = qd
    arm.tau = tau
    arm.gripperQ = -1*(i/duration)

    # Add data to array
    data["q"].append(q)  # Просто прочитать arm.q нельзя, он ничего не выдает. Видимо нужно через геттер это делать или промежуточную переменную.
    data["qd"].append(qd)
    data["tau"].append(tau)
    data["dt"].append(arm._ctrlComp.dt)

    arm.setArmCmd(arm.q, arm.qd, arm.tau)
    arm.setGripperCmd(arm.gripperQ, arm.gripperQd, arm.gripperTau)
    arm.sendRecv()  # udp connection
    # print(arm.lowstate.getQ())

    time.sleep(arm._ctrlComp.dt)

for key, value in data.items():
    data[key] = np.vstack(value).tolist()

# Dump collected data into json file
with open('/home/aleksandr/.local/share/ov/pkg/\
isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/\
isaac/examples/user_examples/z1_manipulation/\
data/data.json', 'w') as outfile:
    json.dump(data, outfile, indent=4)
