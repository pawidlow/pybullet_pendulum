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
q0 = 1.5  # starting position (radian)
g, L, m = 9.8, 0.8, 1
physicsClient = p.connect(p.DIRECT) # or p.DIRECT for non-graphical version
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
t_max = 3
t = np.arange(0, t_max, dt)
for i in range(len(t)):
    angle.append(p.getJointState(bodyUniqueId=boxId, jointIndex=1)[0])
    p.stepSimulation()
    #time.sleep(dt)
    i += 1
plt.plot(t, angle, label = 'симулятор')

# Численное решение ДУ
def model(y, t, g, L):
    theta, omega = y
    dtheta_dt = omega
    domega_dt = -(g / L) * np.sin(theta)
    return [dtheta_dt, domega_dt]

sol = odeint(model, [q0, 0], t, args=(g, L))
theta1 = sol[:, 0]

plt.plot(t, theta1, color='orange', label = 'odeint')

# Неявный метод Эйлера (симплектический)
theta2 = euler_method(q0, 0, t, L, g, dt)
plt.plot(t, theta2, color='g', label = 'неявный метод Эйлера')
plt.xlabel('t, сек'), plt.ylabel('θ, рад')
plt.title('Изменение угла отклонения маятника')
plt.legend(loc='lower left')
plt.show()

# Сравнение траекторий
diff1 = np.array(angle) - np.array(theta1)
diff2 = np.array(angle) - np.array(theta2)
norm1 = np.linalg.norm(diff1)
norm2 = np.linalg.norm(diff2)
linf_norm1 = np.linalg.norm(diff1, ord=np.inf)
linf_norm2 = np.linalg.norm(diff2, ord=np.inf)
print('Евклидова норма для симулятора-odeint =', norm1)
print('Евклидова норма для симулятора-эйлера =', norm2)
print('Linf-норма для симулятора-odeint =', linf_norm1)
print('Linf-норма для симулятора-эйлера =', linf_norm2)


# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()
# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)


#P controller
desired_angle = -0.5
kp = 5
error = 0.1
angle = []
for i in range(len(t)):
    current_angle = p.getJointState(bodyUniqueId=boxId, jointIndex=1)[0]
    angle.append(current_angle)
    error = desired_angle - current_angle
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=kp*error, controlMode=p.VELOCITY_CONTROL)
    p.stepSimulation()
    #time.sleep(dt)

plt.plot(t, angle, label='P controller')

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)


#PID controller kd = 6
kp = 20
ki = 10
kd = 6
error = 0.1
angle = []

prev_error = 0
integral_error = 0

for i in range(len(t)):
    current_angle = p.getJointState(bodyUniqueId=boxId, jointIndex=1)[0]
    angle.append(current_angle)
    error = desired_angle - current_angle
    integral_error += error * dt
    derivative_error = (error - prev_error) / dt
    control_torque = kp * error + ki * integral_error + kd * derivative_error
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, controlMode=p.TORQUE_CONTROL, force=control_torque)
    prev_error = error
    p.stepSimulation()
    #time.sleep(dt)

plt.plot(t, angle, label='PID kd = 6', color='g')
plt.grid()


p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)


#PID controller kd = 11
kp = 20
ki = 10
kd = 11
error = 0.1
angle = []

prev_error = 0
integral_error = 0

for i in range(len(t)):
    current_angle = p.getJointState(bodyUniqueId=boxId, jointIndex=1)[0]
    angle.append(current_angle)
    error = desired_angle - current_angle
    integral_error += error * dt
    derivative_error = (error - prev_error) / dt
    control_torque = kp * error + ki * integral_error + kd * derivative_error
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, controlMode=p.TORQUE_CONTROL, force=control_torque)
    prev_error = error
    p.stepSimulation()
    #time.sleep(dt)

plt.plot(t, angle, label='PID kd = 11', color='orange')
plt.grid(True)
plt.xlabel('time, сек'), plt.ylabel('angle, рад')
plt.title('Изменение угла отклонения маятника')


p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)


#Feedback Linearization
k1 = 150
k2 = 25
damping = 1
angle = []
for i in range(len(t)):
    state = p.getJointState(bodyUniqueId=boxId, jointIndex=1)
    angle.append(state[0])
    x1 = state[0] - desired_angle
    x2 = state[1]
    u = (g/L * np.sin(state[0]) - k1 * x1 - k2 * x2 + damping*x2/(m*L*L))*(m*L*L)
    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, controlMode=p.TORQUE_CONTROL, force=u)
    p.stepSimulation()
    #time.sleep(dt)

plt.plot(t, angle, color='k', label = 'Feedback Linearization')
plt.axhline(y=desired_angle, color='r', label='desired angle', linestyle='--')
plt.legend(loc='upper right')
plt.show()
