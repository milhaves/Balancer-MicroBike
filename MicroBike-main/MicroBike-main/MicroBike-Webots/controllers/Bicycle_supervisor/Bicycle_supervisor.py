from controller import Supervisor

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# [CODE PLACEHOLDER 1]
mc_node = robot.getFromDef('MOTORCYCLE')


i = 0
while robot.step(TIME_STEP) != -1:
  if (i <50 ):
    mc_node.setVelocity([1.75,0,0,0,0,0])
    # mc_node.setVelocity([0,0,0,0,0,0])
  i += 1
