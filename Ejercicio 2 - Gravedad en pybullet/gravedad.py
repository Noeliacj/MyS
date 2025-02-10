import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

planeId = p.loadURDF("plane.urdf")
sphereId = p.loadURDF("sphere2.urdf", [0, 0, 3])

# Obtener radio esfera
visual_shape_data = p.getVisualShapeData(sphereId)
radio = visual_shape_data[0][3][0] 

# Parámetros
g = -9.81
y = 3
v = 0
dt = 1/240
restitucion = 0.8

while True:
    # Ecuaciones MRUA
    v += g * dt
    y += v * dt
         
    # Detectar colisión con el suelo (radio esfera) y rebotar
    if y <= radio:
        y = radio 
        v = -v * restitucion
        
    p.resetBasePositionAndOrientation(sphereId, [0, 0, y], [0, 0, 0, 1])
    
    p.stepSimulation()
    time.sleep(dt)

p.disconnect()
