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

# JointQMax: [2.6179938779914944, 2.8797932657906435, 0.0, 1.5184364492350666, 1.3439035240356338, 2.792526803190927]
# JointQMin: [-2.6179938779914944, 0.0, -2.885592653589793, -1.5184364492350666, -1.3439035240356338, -2.792526803190927]
# JointSpeedMax: [3.141592653589793, 3.141592653589793, 3.141592653589793, 3.141592653589793, 3.141592653589793, 3.141592653589793]

arm.setFsmLowcmd()

duration = 1000
lastPos = arm.lowstate.getQ()
targetPos = np.array([0.0, 0.0, -0.005, -0.074, 0.0, 0.0])  # forward
for i in range(0, duration):
    q = lastPos*(1-i/duration) + targetPos*(i/duration)  # set position
    qd = (targetPos-lastPos)/(duration*0.002)  # set velocity
    (arm.q, arm.qd) = armModel.jointProtect(q, qd)
    print("q", abs(q-arm.q))
    print("qd", abs(qd-arm.qd))
    arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6))  # set torque
    arm.gripperQ = -1*(i/duration)

    arm.setArmCmd(arm.q, arm.qd, arm.tau)
    # arm.setGripperCmd(arm.gripperQ, arm.gripperQd, arm.gripperTau)
    arm.sendRecv()# udp connection
    # print(arm.lowstate.getQ())
    time.sleep(arm._ctrlComp.dt)

duration = 1000
lastPos = arm.lowstate.getQ()
targetPos = np.array([0.0, 1.5, -1.0, -0.54, 0.0, 0.0])  # forward
for i in range(0, duration):
    q = lastPos*(1-i/duration) + targetPos*(i/duration)  # set position
    qd = (targetPos-lastPos)/(duration*0.002)  # set velocity
    (arm.q, arm.qd) = armModel.jointProtect(q, qd)
    print("q", abs(q-arm.q))
    print("qd", abs(qd-arm.qd))
    arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6))  # set torque
    arm.gripperQ = -1*(i/duration)

    arm.setArmCmd(arm.q, arm.qd, arm.tau)
    # arm.setGripperCmd(arm.gripperQ, arm.gripperQd, arm.gripperTau)
    arm.sendRecv()# udp connection
    # print(arm.lowstate.getQ())
    time.sleep(arm._ctrlComp.dt)

while True:
    try:
        arm.sendRecv()
    except KeyboardInterrupt:
        print("Finished!")
    except Exception as e:
        print(e)
    finally:
        pass
        # arm.loopOn()
        # arm.backToStart()
        # arm.loopOff()
