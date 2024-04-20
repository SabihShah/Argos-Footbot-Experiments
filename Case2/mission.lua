is_cambot = false                         -- Cambot drops on Blue
is_groundbot = false                      -- Groundbot drops on Yellow

IS_GROUND_YELLOW = false
IS_GROUND_BLUE = false
IS_OBJECT_GRIPPED = false
IS_OBJECT_SENSED = false


dropping_timer = 0
DROPPING_TIMEOUT = 50
WHEEL_SPEED = 10
angl = 0


function DRIVE(forwardSpeed, angularSpeed)
   -- robot.leds.set_all_colors("blue")
   -- We have an equal component, and an opposed one
   local leftSpeed  = forwardSpeed - angularSpeed
   local rightSpeed = forwardSpeed + angularSpeed

   robot.wheels.set_velocity(leftSpeed, rightSpeed)
end

function OBSTACLE()
   local leftObstacleDistance = 0
   for i = 1, 6 do
      leftObstacleDistance = leftObstacleDistance + robot.proximity[i].value
   end
   local rightObstacleDistance = 0
   for i = 19, 24 do
      rightObstacleDistance = rightObstacleDistance + robot.proximity[i].value
   end
   -- local leftObstacleDistance = robot.proximity[3].value + robot.proximity[4].value + robot.proximity[5].value +
   --     robot.proximity[6].value
   -- local rightObstacleDistance = robot.proximity[22].value + robot.proximity[21].value + robot.proximity[20].value +
   --     robot.proximity[19].value
   if (leftObstacleDistance ~= 0) then
      DRIVE(5, -4) -- right turn
      -- log("Avoiding obstacle on left")
   elseif (rightObstacleDistance ~= 0) then
      DRIVE(5, 4)
      -- log("Avoiding obstacle on right")
   end
end

function table.copy(t)
   local t2 = {}
   for k, v in pairs(t) do
      t2[k] = v
   end
   return t2
end

function Processobject() -- ! OBJECT SENSE WORKING FINE
   -- IS_OBJECT_SENSED = false
   sort_blob = table.copy(robot.colored_blob_omnidirectional_camera)
   log(#robot.colored_blob_omnidirectional_camera)
   if (#robot.colored_blob_omnidirectional_camera > 0) then
      table.sort(sort_blob, function(a, b) return a.distance < b.distance end)
      for i = 1, #robot.colored_blob_omnidirectional_camera do
         if sort_blob[i].color.green == 0 and sort_blob[i].color.blue == 0 and sort_blob[i].color.red == 255 then
            IS_OBJECT_SENSED = true
            log("object sensed")
            -- log (sort_blob[i].distance)
            -- log("angle: " .. sort_blob[i].angle)
            angl = sort_blob[i].angle
            dis1 = sort_blob[i].distance
         end
         break
      end
   end
   return angl, dis1
end

function Gripper_Lock()
   -- IS_OBJECT_GRIPPED = false
   angle, dis1 = Processobject()
   robot.wheels.set_velocity(0, 0)
   robot.turret.set_position_control_mode()
   robot.turret.set_rotation(angle)
   robot.gripper.lock_positive()
   log("object gripped")
   IS_OBJECT_GRIPPED = true
   -- robot.leds.set_all_colors("green")
   -- robot.turret.set_passive_mode()
   -- DRIVE(robot.random.uniform(10,20), robot.random.uniform(-10,10))
end


function init_drop_procedure()
   -- set timer for dropping
   dropping_timer = DROPPING_TIMEOUT
   -- find the closest LED, this is (most likely) the LED of the object being carried
   closest_distance = 100
   closest_angle = 0
   for x = 1, #robot.colored_blob_omnidirectional_camera
   do
      if robot.colored_blob_omnidirectional_camera[x].distance < closest_distance
      then
         closest_distance = robot.colored_blob_omnidirectional_camera[x].distance
         closest_angle = robot.colored_blob_omnidirectional_camera[x].angle
         if flag == 1 then
            flag = 0
            closest_angle = closest_angle
         else
            flag = 1
            closest_angle = -closest_angle
         end
      end
   end
   dropping_angle = closest_angle
   return dropping_angle
end

function drop_object()
   -- this is a multi-step approach that rotates the gripper into position
   dropping_angle = init_drop_procedure()
   if dropping_timer == DROPPING_TIMEOUT then
      robot.turret.set_position_control_mode()
      robot.turret.set_rotation(dropping_angle)

      -- robot.wheels.set_velocity(0, 0)
      robot.gripper.unlock()
      IS_OBJECT_GRIPPED = false
   elseif dropping_timer == 0 then
      robot.gripper.unlock()
      IS_OBJECT_GRIPPED = false
      -- TODO: change to the next state
   end
   dropping_timer = dropping_timer - 1
end


function ComputeSpeedFromAngle(angle) --[[this finction returen the speed wheel given to the robot speed[1],speed[2]]
                                                                                                                      --
   if angle == nil then
      log(" error angle = nil ")
   end
   --else
   dotProduct = 0.0;
   KProp = 20;
   wheelsDistance = 0.14;
   -- if the target angle is behind the robot, we just rotate, no forward motion
   if angle > math.pi / 2 or angle < -math.pi / 2 then
      dotProduct = 0.0;
   else
      -- else, we compute the projection of the forward motion vector with the desired angle
      forwardVector = { math.cos(0), math.sin(0) }
      targetVector = { math.cos(angle), math.sin(angle) }
      dotProduct = forwardVector[1] * targetVector[1] + forwardVector[2] * targetVector[2]
   end
   -- the angular velocity component is the desired angle scaled linearly
   angularVelocity = KProp * angle;
   -- the final wheel speeds are compute combining the forward and angular velocities, with different signs for the left and right wheel.
   speeds = { dotProduct * WHEEL_SPEED - angularVelocity * wheelsDistance,
      dotProduct * WHEEL_SPEED + angularVelocity * wheelsDistance }
   --[[the function returns an array where speeds[1] contains the velocity for the left wheel, and speeds[2] contains the velocity for the right wheel]]
                                                                                                                                                         --
   -- log("Speed Left: " .. speeds[1])
   -- log("Speed Right: " .. speeds[2])
   -- robot.wheels.set_velocity(speeds[1],speeds[2])
   return speeds
   --end
end

function ground()
   -- is_ground_yellow = false
   sort_ground = table.copy(robot.motor_ground)
   table.sort(sort_ground, function(a, b) return a.value < b.value end)

   if sort_ground[1].value < 0.8 and sort_ground[1].value > 0.6 then -- ! Check COLORS
      IS_GROUND_YELLOW = true
      IS_GROUND_BLUE = false
     
   elseif sort_ground[1].value < 0.6  and sort_ground[1].value > 0.4 then
      IS_GROUND_BLUE = true
      IS_GROUND_YELLOW = false

   else
      IS_GROUND_YELLOW = false
      IS_GROUND_BLUE = false
   end
end

function Light_Direction() -- Moves towards the yellow light after gripping
   MAIN_LIGHT_DETECTED = false
   dist = 0
   dl = 0
   dl_x = 0
   dl_y = 0
   sort_light = table.copy(robot.light)
   table.sort(sort_light, function(a, b) return a.value > b.value end)
   light_val = sort_light[1].value
   dl_x = dl_x + math.cos(sort_light[1].angle)
   dl_y = dl_y + math.sin(sort_light[1].angle)
   dl = math.atan(dl_y, dl_x)
   dist = sort_light[1].value
   log("dist: " .. dist)

   return dl, dist
end

function check_robot_type()
    -- special logic to enable multi-robot opeation - do not alter this part!
   if string.find(robot.id, "cam") ~= nil then
        is_cambot    = true
        is_groundbot = false
        is_lightbot  = false
        led_color = "blue"
   end

   if string.find(robot.id, "gnd") ~= nil then
      is_cambot    = false
      is_groundbot = true
      is_lightbot  = false
      led_color = "yellow"
   end

    -- this line is to help you with visualizing the roles in the swarm
    -- comment or remove this line before performing your experiments
    robot.leds.set_all_colors(led_color)
   
end
function senseWhite()
   color = false
   for i = 1, 4 do
      -- log(robot.motor_ground[i].value)
      -- log(robot.motor_ground[i].offset.x .. " " .. robot.motor_ground[i].offset.y)

      if robot.motor_ground[i].value > 0.8 then
         color = true
      end
      break
   end
   if (color) then
      DRIVE(5, 4)
      -- DRIVE(robot.random.uniform(10,20), robot.random.uniform(-10,10))
   end
   
end

function senseYellow()
   colorYellow = false
   for i = 1, 4 do
      -- log(robot.motor_ground[i].value)
      -- log(robot.motor_ground[i].offset.x .. " " .. robot.motor_ground[i].offset.y)

      if robot.motor_ground[i].value < 0.8 and robot.motor_ground[i].value > 0.6 then
         colorYellow = true
      end
      break
   end
   if (colorYellow) then
      DRIVE(5, 4)
      -- DRIVE(robot.random.uniform(10,20), robot.random.uniform(-10,10))
   end
   
end

function groundbot()

   DRIVE(robot.random.uniform(10, 20), robot.random.uniform(-10, 10))
   OBSTACLE()
   ground()
   senseWhite()

   if IS_GROUND_BLUE then
      angle2, dis2 = Processobject()
      speedss = ComputeSpeedFromAngle(angle2)
   end

   if IS_OBJECT_SENSED and IS_GROUND_BLUE then
      robot.wheels.set_velocity(speedss[1], speedss[2])
      if dis2 < 19 then
         Gripper_Lock()
      end
   end

   if IS_OBJECT_GRIPPED then
      dl, dist = Light_Direction()
      speeds = ComputeSpeedFromAngle(dl)
      robot.wheels.set_velocity(speeds[1], speeds[2])
      while (IS_GROUND_YELLOW and dist > 0.67) do
         robot.wheels.set_velocity(0,0)
         drop_object()
         
         IS_OBJECT_SENSED = false
         if not IS_OBJECT_GRIPPED then
            break
         end
      end
   end
end


function cambot()

   DRIVE(robot.random.uniform(10, 20), robot.random.uniform(-10, 10))
   OBSTACLE()
   ground()
   senseYellow()
   -- DETECT_TRACK()
   if not IS_GROUND_YELLOW and not IS_GROUND_BLUE then
      angle2, dis2 = Processobject()
      speedss = ComputeSpeedFromAngle(angle2)
   end

   if IS_OBJECT_SENSED and not IS_GROUND_BLUE and not IS_GROUND_YELLOW then
      robot.wheels.set_velocity(speedss[1], speedss[2])
      if dis2 < 19 then
         Gripper_Lock()
      end
   end

   if IS_OBJECT_GRIPPED then
      dl, dist = Light_Direction()
      speeds = ComputeSpeedFromAngle(dl)
      robot.wheels.set_velocity(speeds[1], speeds[2])
      while (IS_GROUND_BLUE and dist > 0.6) do
         robot.wheels.set_velocity(0,0)
         drop_object()
         -- IS_OBJECT_GRIPPED = false
         IS_OBJECT_SENSED = false
         if not IS_OBJECT_GRIPPED then
            break
         end
      end
   end
end


function init()
   check_robot_type()
   robot.colored_blob_omnidirectional_camera.enable()
end


function step()
   if is_cambot then
      cambot()
   end

   if is_groundbot then
      groundbot()
   end
end

function reset()
   check_robot_type()
end

function destroy()
end