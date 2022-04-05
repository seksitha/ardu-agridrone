-- This script is a test for AP_Mission bindings
-- local last_mission_index = mission:num_commands-1
local state = 0
local current_mission_index = 0
local set_yaw = 0;
local home = nil;
local rc7 = 0;
local spray = SRV_Channels:find_channel(22) --pump
local spinner = SRV_Channels:find_channel(23) --spinner
local spraySpeed = 1200;
local spinnerSpeed = 1400;
local current_x = 0;
local current_y = 0;
local pos_to_continue;
local pos_waypoint_break;
local monitor_state = 0
local pre_mission_length = 0;
local pre_mission_waypoint ;
local loop_increment_count = 0 ;
local loop_max_increment = 4;
local gpio1_average = 0 ;
local gpio1 = 0 ;
local output_pwm_srv_spray = 1000;
local loop_mission_increment_count = 0;
local loop_rc_increment_count = 0;

-- function set_spray_speed_by_rc() -- showing the percentage on QGC
-- 	-- gcs:send_text(6,string.format("chan 6: %d and chan 8: %d",rc:get_radio_in(5),rc:get_radio_in(7)));
-- 	if rc:get_radio_in(5) < 1050 and rc:get_radio_in(7) < 1050 then
-- 		-- spraySpeed = 1250;
-- 		-- spinnerSpeed = 1400;
-- 	else 
-- 		-- if it is not equal we can set the value
-- 		if spraySpeed ~= rc:get_radio_in(5) or spinnerSpeed ~= rc:get_radio_in(7) then
-- 			spraySpeed = rc:get_radio_in(5)  ;
-- 			spinnerSpeed = rc:get_radio_in(7) ;
-- 			loop_rc_increment_count = loop_rc_increment_count + 1;
-- 		end 
-- 		-- this is for delay the show (throttle in react) value on QGC
-- 		if loop_rc_increment_count > 0 and loop_rc_increment_count < 3 then 
-- 			loop_rc_increment_count = loop_rc_increment_count + 1;
-- 		end 
-- 		if loop_rc_increment_count == 3 then
-- 			gcs:send_text(6,( string.format("spray: %.2f",(spraySpeed-1000)/10) .. "% " .. string.format(" and spinner: %.2f",(spinnerSpeed-1000)/10) .. "%" ));
-- 			loop_rc_increment_count = 0
-- 		end
-- 	end
-- end
function testFlowSensor()
	local gpio2 = wpnav:readFlowSensor(60);
	gcs:send_text(6,string.format("sensor val %f", gpio2))
end
function monitor_flow_sensor ()
  gpio1 = gpio1 + wpnav:readFlowSensor(60);
	-- gpio1 = gpio1 + 12000
	loop_increment_count = loop_increment_count + 1;
	--seguence 1
	if loop_increment_count == loop_max_increment then
		gpio1_average = gpio1/loop_max_increment;
		gpio1 = 0;
	end
  	--sequence 2
	if loop_mission_increment_count > 10 and output_pwm_srv_spray == spraySpeed and loop_increment_count == loop_max_increment and (current_mission_index - 1) % 2 == 0 then
		-- gcs:send_text(6,string.format("sensor val %f", gpio1_average))
		if gpio1_average > 21000 then
			vehicle:set_mode(5)
		end
	end
  -- seguence 3
	if loop_increment_count == loop_max_increment then
		-- cannot set it to zero before checking the value gpio1_average for RTL
		loop_increment_count = 0
	end
end

function update() -- this is the loop which periodically runs
	local mode = vehicle:get_mode()
	local mission_state = mission:state()
	-- check when user start mission
	
	if mission_state == 1 then
		-- when mission start current_index will go to 1 not 0, the 1 is takeoff
		-- but the mission index is started from 0
		-- last mission is current_index - 1 and num commands includes home so - 1 here
			current_mission_index = mission:get_current_nav_index()
			
			-- if current_mission_index > 1 then
			-- 	local origin = Location()
			-- 	origin:lat((mission:get_item(current_mission_index-1):x()))
			-- 	origin:lng((mission:get_item(current_mission_index-1):y()))
			-- 	local disination = Location()
			-- 	disination:lat((mission:get_item(current_mission_index):x()))
			-- 	disination:lng((mission:get_item(current_mission_index):y()))
			-- 	local distance = origin:get_distance(disination)
			-- 	gcs:send_text(6,string.format("distance_m %f", distance))
			-- end

			set_spray_speed_by_rc() -- TODO this only work at new waypoint because the misison loop is run only once
		-- if mission:get_current_nav_index() <  mission:num_commands() - 1 and current_mission_index ~= mission:get_current_nav_index() then
			-- this condition we only limit it to run once during the whole waypoint 
			-- so don't increament anything here
			-- spray on index even 3,5,7
			if current_mission_index > 1 and (current_mission_index - 1) % 2 == 0 then
				output_pwm_srv_spray = spraySpeed
				-- TODO set spray speed only work at new waypoint because the misison loop is run only once
				SRV_Channels:set_output_pwm_chan(spray,spraySpeed)
				SRV_Channels:set_output_pwm_chan(spinner,spinnerSpeed)
			else
				output_pwm_srv_spray = 1000;
				loop_mission_increment_count = 0
				SRV_Channels:set_output_pwm_chan(spray,1000)
				SRV_Channels:set_output_pwm_chan(spinner,1000)
			end       
			-- gcs:send_text(6,string.format("====== spray num %d", output_pwm_srv_spray))
		-- end

		if (current_mission_index - 1) % 2 == 0 then
			loop_mission_increment_count = loop_mission_increment_count + 1;
		else 
			loop_mission_increment_count = 0
		end

		-- if current_mission_index == 2 then
		--   vehicle:set_yaw(120,150,1,false)
		-- end
		if rc:get_radio_in(6) > 1050 then 
			monitor_flow_sensor()
		end
		-- get current loaction in case break_point is triggered 
		local location = ahrs:get_position()
		current_x = location:lat()
		current_y = location:lng()
		pre_mission_length = mission:num_commands();
		pre_mission_waypoint = mission:get_item(mission:num_commands()-1)
		-- if the pilot break the mission on the same waypoint 2 time and click on resume it will
		-- reset the the waypoint #2 to early val by GCS
		pos_waypoint_break = mission:get_item(current_mission_index-1)

		if current_mission_index > 1 then 
			state = 1;
		end
		return update, 500 -- reschedules the loop stop the code here
	end
	-- testFlowSensor()

	-- if  vehicle:get_mode() == 17 then 
	-- 	local cur_point = mission:get_item(current_mission_index);
	-- end

	set_spray_speed_by_rc()
	-- when mission complete do following cmd
	if mission_state == 2 and state == 1 then
		vehicle:set_mode(5)
		output_pwm_srv_spray = 1000;
		SRV_Channels:set_output_pwm_chan(spray,1000)
		SRV_Channels:set_output_pwm_chan(spinner,1000)
		state = 0;
		monitor_state = 0
		loop_mission_increment_count = 0
		-- gcs:send_text(6,string.format("====== state num %d", state))
	end

	-- when mission stop by user or flow sensor
	if mission_state == 0 and state == 1 then
		output_pwm_srv_spray = 1000;
		if(current_mission_index > 2)then
		pos_to_continue = mission:get_item(current_mission_index-1)
		pos_to_continue:x(current_x)
		pos_to_continue:y(current_y)
		end
		state = 3;
		monitor_state = 0
		loop_mission_increment_count = 0
		-- gcs:send_text(6,string.format("====== state num %d", state))
	end

	-- check the breakpoint if we should set break point at waypoint #2 on resume
	if pre_mission_length - (current_mission_index-3) == (mission:num_commands()) and 
		pre_mission_waypoint:x() == mission:get_item(mission:num_commands()-1):x() and 
		mission_state == 0 and state == 3 and 
		mission:get_item(2):x() ~= pos_to_continue:x() and 
		mission:get_item(2):y() ~= pos_to_continue:y()  then
		-- we can not set state to 0; because the resume command is happen after
		-- this loop then it will override the waypoint #2 again. 
		-- but this also mean it will override the freshly upload mission :(
		-- only reset the FC distroy state == 3. 
		-- Oh, we can check num_commands and last_waypont
		mission:set_item(2, pos_to_continue);
		
	end
	if rc:get_radio_in(9) > 1600 and mission_state == 0 then
		testFlowSensor()
	end
  -- the rc chenel is index from 0 while in the ardupilot index from 1. strange!
	rc7 = rc:get_radio_in(6)
	if rc7 > 1550 then
		-- gcs:send_text(6,string.format("spray#: %d and spin#: %d",spray,spinner));
		SRV_Channels:set_output_pwm_chan(spray,spraySpeed)
		SRV_Channels:set_output_pwm_chan(spinner,spinnerSpeed)
	end
	if rc7 <= 1500 then
		SRV_Channels:set_output_pwm_chan(spray,1000)
		SRV_Channels:set_output_pwm_chan(spinner,1000)
	end
 
  	return update, 500 -- reschedules the loop
end

return update, 500 -- run immediately before starting to reschedule
