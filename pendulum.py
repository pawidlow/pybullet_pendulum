import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt


dt = 1/240 # pybullet simulation step
q0 = 3  # starting position (radian)
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)
# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()
# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
angle = [q0]
counter = 1
while True:
    currentangle = []
    currentangle.append(p.getJointState(bodyUniqueId=boxId, jointIndex=1))
    angle.append(currentangle[0][0])
    p.stepSimulation()
    time.sleep(dt)
    if counter == 1:
        counter += 1
    else:
        if abs(angle[counter-1] - angle[counter]) > (10 ** (-7)):
            counter += 1
        else:
            break
p.disconnect()
time = [j/240 for j in range(counter+1)]
plt.plot(time, angle)
plt.show()


