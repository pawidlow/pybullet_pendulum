import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

def euler_method(theta_0, omega_0, t, L, g, dt):
    theta_values = [theta_0]
    omega = omega_0
    for i in range(1, len(t)):
        omega -= (g / L) * np.sin(theta_values[i - 1]) * dt
        theta = theta_values[i - 1] + omega * dt
        theta_values.append(theta)
    return theta_values

dt = 1/240 # pybullet simulation step
q0 = 0.3  # starting position (radian)
g, L = 9.8, 0.8
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-g)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)
# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0, jointDamping = 0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0, jointDamping = 0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()
# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

# Построение графика угла отклонения маятника
angle = []
t_max = 4
t = np.arange(0, t_max, dt)
for i in range(len(t)):
    angle.append(p.getJointState(bodyUniqueId=boxId, jointIndex=1)[0])
    p.stepSimulation()
    time.sleep(dt)
    i += 1

plt.figure(figsize = (13, 5.5))
plt.subplot(1, 3, 1)
plt.plot(t, angle)
plt.xlabel('t, сек'), plt.ylabel('θ, рад')
plt.title('Динамика угла отклонения маятника')

# Численное решение ДУ
def model(y, t, g, L):
    theta, omega = y
    dtheta_dt = omega
    domega_dt = -(g / L) * np.sin(theta)
    return [dtheta_dt, domega_dt]

sol = odeint(model, [q0, 0], t, args=(g, L))
theta = sol[:, 0]

plt.subplot(1, 3, 2)
plt.plot(t, theta, color='orange')
plt.xlabel('t, сек'), plt.ylabel('θ, рад')
plt.title('Численное решение ДУ')

theta = euler_method(q0, 0, t, L, g, dt)
plt.subplot(1, 3, 3)
plt.plot(t, theta, color='g')
plt.xlabel('t, сек'), plt.ylabel('θ, рад')
plt.title('Решение, полученное методом Эйлера')
plt.tight_layout()
plt.show()
