
import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

startPosition = [0,0,1.0]
euler_angles = [0,0,0]
startOrientation = p.getQuaternionFromEuler(euler_angles)

robotId = p.loadURDF("robot.urdf",startPosition,startOrientation)

# Parámetros de control de fricción y torque
joint_friction = p.addUserDebugParameter("jointFriction",0,0.1,0.01)
joint_torque = p.addUserDebugParameter("jointTorque",0,0.1,0.01)

# Aplicar freno inicial a los joints
for j in range(p.getNumJoints(robotId)):
    p.setJointMotorControl2(robotId,j,p.VELOCITY_CONTROL,targetVelocity=0,force=10)

# Bucle de simulación
for i in range(10000): 
    jointFriction = p.readUserDebugParameter(joint_friction)
    jointTorque = p.readUserDebugParameter(joint_torque)

    p.setJointMotorControlArray(robotId,[0,1],p.VELOCITY_CONTROL,targetVelocities=[jointFriction,jointFriction])
    p.setJointMotorControl2(robotId,1,p.TORQUE_CONTROL,force=jointTorque)

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()