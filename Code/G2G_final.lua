function sysCall_init()
    -- Setting up the main parameters
    -- Get object handles
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    target = sim.getObjectHandle("Goal")
    robot_pose = sim.getObjectHandle("Robot")


    pose_R = r_pose()
    pose_G = g_pose()
    v_R = 1 -- speed of the robot
    v_G = 0.3 -- speed of the goal/target
    R = 0.195/2 -- radius of the robot
    l = 3 -- length of the robot
    wheel_radius=0.195/2
    b=0.1655
    vref=0.15
    -- e=0.24
    k=0.05
    kv=0.1

end
-- This is a very simple EXAMPLE navigation program, which avoids obstacles using the Braitenberg algorithm

function sysCall_actuation()
    local p_ref,p_off,wR,wL
    p_ref = g_pose()
    r_ref = r_pose()
    lambda,ed = angleCoord(r_ref,p_ref)
    p_off = getOffCenterPoint(p_ref, ed)
    wL,wR = kinematicControl(p_off,pose_R,k,ed)
    print(ed)
    sim.setJointTargetVelocity(motorLeft,wL)
    sim.setJointTargetVelocity(motorRight,wR)
    if ed <1e-2 then
      sim.setJointTargetVelocity(motorLeft,0)
      sim.setJointTargetVelocity(motorRight,0)
    end
end

function sysCall_sensing()
    pose_R = r_pose()
    pose_G = g_pose()

end

function sysCall_cleanup()

end


function r_pose()
    local pose
    position = sim.getObjectPosition(robot_pose,-1)
    orientation = sim.getObjectOrientation(robot_pose,-1)
    pose = {position[1],position[2],orientation[3]}
    return pose
end

function g_pose()
    local pose
    position = sim.getObjectPosition(target,-1)
    orientation = sim.getObjectOrientation(target,-1)
    pose = {position[1],position[2],orientation[3]}
    return pose
end

function getOffCenterPoint(p_ref,e)
  local xc,yc,vxc,vyc
  xc = p_ref[1] + e*math.cos(p_ref[3])
  yc = p_ref[2] + e*math.sin(p_ref[3])
  pos = {xc,yc}
  return pos
end

function kinematicControl(p_ref,pose,k,e)
  local wL,wR,ex,ey
  ex = k*(p_ref[1] - (pose[1]+e*math.cos(pose[3])))
  ey = k*(p_ref[2] - (pose[2]+e*math.sin(pose[3])))
  -- wL=(1/(e*R))*((e*math.cos(pose[3])+b*math.sin(pose[3]))*ex+(e*math.sin(pose[3])-b*math.cos(pose[3]))*ey)
  -- wR=(1/(e*R))*((e*math.cos(pose[3])-b*math.sin(pose[3]))*ex+(e*math.sin(pose[3])+b*math.cos(pose[3]))*ey)
  wL=(1/(R))*((math.cos(pose[3])+b*math.sin(pose[3]))*ex+(math.sin(pose[3])-b*math.cos(pose[3]))*ey)
  wR=(1/(R))*((math.cos(pose[3])-b*math.sin(pose[3]))*ex+(math.sin(pose[3])+b*math.cos(pose[3]))*ey)

  return wL,wR
end


function angleCoord(pose_R,pose_G)
    local y_d, x_d, Lambda_GR, r_GR
    y_d = pose_G[2] - pose_R[2]
    x_d = pose_G[1] - pose_R[1]
    Lambda_GR = math.atan(y_d/x_d)
    r_GR = kv*math.sqrt(y_d^2 + x_d^2)
    return Lambda_GR,r_GR
end
