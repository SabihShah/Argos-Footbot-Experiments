-- Range and Bearing Actuator for Sending Signal
-- Rnage and Bearing Sensor for Receiving Signal


is_cambot = false 
is_groundbot = false

IS_GROUND_WHITE = false
IS_GROUND_YELLOW = false
IS_GROUND_BLUE = false
IS_OBJECT_GRIPPED = false
IS_OBJECT_SENSED = false
FOOD_PILE = false
send_message = false
receive_message = false
receiving = false
WHITE_ROBOT = false
BLACK_ROBOT = false

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
   IS_OBJECT_SENSED = false
   WHITE_ROBOT = false
   BLACK_ROBOT = false
   sort_blob = table.copy(robot.colored_blob_omnidirectional_camera)
   log(#robot.colored_blob_omnidirectional_camera)
   if (#robot.colored_blob_omnidirectional_camera > 0) then
      table.sort(sort_blob, function(a, b) return a.distance < b.distance end)
      for i = 1, #robot.colored_blob_omnidirectional_camera do
         if (sort_blob[i].color.green == 0 and sort_blob[i].color.blue == 0 and sort_blob[i].color.red == 255) then
            IS_OBJECT_SENSED = true
            log("object sensed")
            angl = sort_blob[i].angle
            dis1 = sort_blob[i].distance
         end

         break
      end
   end
   return angl, dis1
end

function Processobject_ground() 
   FOOD_PILE = false
   robot.leds.set_all_colors("yellow")
   sort_blob = table.copy(robot.colored_blob_omnidirectional_camera)
   objects_detected = 0
   if (#robot.colored_blob_omnidirectional_camera > 0) then
      table.sort(sort_blob, function(a, b) return a.distance > b.distance end)
      for i =1, #robot.colored_blob_omnidirectional_camera do
         if sort_blob[i].color.green == 0 and sort_blob[i].color.blue == 0 and sort_blob[i].color.red == 255 then
            objects_detected = objects_detected + 1
            FOOD_PILE = true
            angleGround = sort_blob[i].angle
            DistGround = sort_blob[i].distance
         end
      end
      log("Pile Detected")
      log("objects_detected: " ..objects_detected)
   end
   return angleGround, DistGround, objects_detected
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
      IS_GROUND_WHITE = false
     
   elseif sort_ground[1].value < 0.6 and sort_ground[1].value > 0.4 then
      IS_GROUND_BLUE = true
      IS_GROUND_YELLOW = false
      IS_GROUND_WHITE = false

   elseif sort_ground[1].value > 0.85 then
      IS_GROUND_WHITE = true
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


-- function Send()
--    log("Sending Signal")
--    -- robot.range_and_bearing.set_data({0,1,2,3,4,5,6,7,8,9})
--    robot.range_and_bearing.set_data(1,1)
-- end

function receive()
   receiving = false

   sort_signal = table.copy(robot.range_and_bearing)
   table.sort(sort_signal, function(a, b) return a.range > b.range end)
   log(#robot.range_and_bearing)
   if #robot.range_and_bearing > 0 then
      for i=1, #robot.range_and_bearing do
         for j=1, 10 do
            data = sort_signal[i].data[j]
            -- log(data)
            if data == 1 then
               speeds = ComputeSpeedFromAngle(sort_signal[i].horizontal_bearing)
               robot.wheels.set_velocity(speeds[1], speeds[2])

               if sort_signal[i].range < 40 then
                  robot.wheels.set_velocity(speeds[1], speeds[2] + 10)
               end
               log(sort_signal[i].range)
               receiving = true
               break
            end
         end
      end
   end
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

function check_carry_object()
   object_dis = 200
   IS_GRABING_OBJECT=false
   if (#robot.colored_blob_omnidirectional_camera > 0) then
       sort_blob = table.copy(robot.colored_blob_omnidirectional_camera)
       table.sort(sort_blob, function(a,b) return a.distance<b.distance end)
       object_dis = sort_blob[1].distance
       log("object dis = " ..object_dis)
      if sort_blob[1].distance < 30 and sort_blob[1].color.green ==0 and sort_blob[1].color.blue ==0 and sort_blob[1].color.red ==255 then
         IS_GRABING_OBJECT=true
      end
   end 
   return object_dis
end

function groundbot()
   distance = check_carry_object()
   if distance < 16 then
      DRIVE(5,4)
   end
   robot.range_and_bearing.set_data(1,0)
   DRIVE(robot.random.uniform(10, 20), robot.random.uniform(-10, 10))
   OBSTACLE()
   ground()
   angleGround, DistGround, objects_detected = Processobject_ground()

   if FOOD_PILE and not IS_GROUND_YELLOW then
      speeds = ComputeSpeedFromAngle(angleGround)
      robot.wheels.set_velocity(speeds[1], speeds[2])

      if DistGround < 30 then
         robot.wheels.set_velocity(0,0)
         robot.leds.set_all_colors("white")
         robot.range_and_bearing.set_data(1,1)
      end
   end
end

function cambot()
   distance = check_carry_object()
   if distance < 30 then
      DRIVE(5,4)
   end

   ground()
   OBSTACLE()

   if not IS_OBJECT_GRIPPED then
      receive()
   end

   if not receiving then
      DRIVE(robot.random.uniform(10, 20), robot.random.uniform(-10, 10))
      senseWhite()
      -- robot.wheels.set_velocity(0,0)
   end

   if receiving then
      angle, distance = Processobject()
      
      -- if WHITE_ROBOT and distance <  then
      --    robot.wheels.set_velocity(8, 15)
      -- end

      if IS_OBJECT_SENSED and IS_GROUND_WHITE then
         speeds = ComputeSpeedFromAngle(angle)
         robot.wheels.set_velocity(speeds[1], speeds[2])
         if distance < 19 then
            Gripper_Lock()
         end
      end

      if IS_OBJECT_GRIPPED then
         robot.leds.set_all_colors("black")
         dl, dist = Light_Direction()
         speedss = ComputeSpeedFromAngle(dl)
         robot.wheels.set_velocity(speedss[1], speedss[2])
         while (IS_GROUND_YELLOW and dist > 0.75) do
            robot.wheels.set_velocity(0,0)
            drop_object()
            IS_OBJECT_SENSED = false
            if not IS_OBJECT_GRIPPED then
               robot.leds.set_all_colors("blue")
               break
            end
         end
      end
   end
end


function init()
   robot.colored_blob_omnidirectional_camera.enable()
   check_robot_type()
end

function step()

   if is_groundbot then
      groundbot()
   end

   if is_cambot then
      cambot()
   end
end

function reset()
   check_robot_type()
end

function destroy()
end