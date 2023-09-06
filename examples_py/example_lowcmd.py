import sys
sys.path.append("/home/aleksandr/.local/share/ov/pkg/isaac_sim-2022.2.1/\
exts/omni.isaac.examples/omni/isaac/examples/user_examples/\
z1_manipulation/z1_sdk/lib")
import unitree_arm_interface
import time
import numpy as np

print("Press ctrl+\ to quit process.")

np.set_printoptions(precision=3, suppress=True)
arm = unitree_arm_interface.ArmInterface(hasGripper=True)
armModel = arm._ctrlComp.armModel

# print("JointQMax:", armModel.getJointQMax())
# print("JointQMin:", armModel.getJointQMin())
# print("JointSpeedMax:", armModel.getJointSpeedMax())

arm.setFsmLowcmd()

duration = 1000
lastPos = arm.lowstate.getQ()
targetPos = np.array([0.0, 1.5, -1.0, -0.54, 0.0, 0.0]) #forward

for i in range(0, duration):
    arm.q = lastPos*(1-i/duration) + targetPos*(i/duration)# set position
    arm.qd = (targetPos-lastPos)/(duration*0.002) # set velocity
    arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6)) # set torque
    arm.gripperQ = -1*(i/duration)

    arm.setArmCmd(arm.q, arm.qd, arm.tau)
    arm.setGripperCmd(arm.gripperQ, arm.gripperQd, arm.gripperTau)
    arm.sendRecv()# udp connection
    # print(arm.lowstate.getQ())
    time.sleep(arm._ctrlComp.dt)

arm.loopOn()
arm.backToStart()
arm.loopOff()
