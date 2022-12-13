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
local prevouis_mission_length = 0;
local prevouis_mission_waypoint_endpoint ;
local loop_increment_count = 0 ;
local loop_max_increment = 4;
local gpio1_average = 0 ;
local gpio1 = 0 ;
local output_pwm_srv_spray = 1000;
local loop_mission_increment_count = 0;
local loop_rc_increment_count = 0;


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

		-- FOR TEST FLOW SS
		if rc:get_radio_in(6) > 1050 then 
			monitor_flow_sensor()
		end

		-- FOR RESUME SPRAY get current loaction in case break_point is triggered 
		
		return update, 500 -- reschedules the loop stop the code here
	end

	-- if  vehicle:get_mode() == 17 then 
	-- 	local location = ahrs:get_position()
	-- 	current_x = location:lat()
	-- 	current_y = location:lng()
	-- 	prevouis_mission_length = mission:num_commands();
	-- 	prevouis_mission_waypoint_endpoint = mission:get_item(mission:num_commands()-1)
	-- 	-- if the pilot break the mission on the same waypoint 2 time and click on resume it will
	-- 	-- reset the the waypoint #2 to early val by GCS
	-- 	pos_waypoint_break = mission:get_item(current_mission_index-1)
	-- end

	-- when mission complete do following cmd
	if mission_state == 2 and state == 1 then
		vehicle:set_mode(5)
		-- output_pwm_srv_spray = 1000;
		-- SRV_Channels:set_output_pwm_chan(spray,1000)
		-- SRV_Channels:set_output_pwm_chan(spinner,1000)
		-- state = 0;
		-- monitor_state = 0
		loop_mission_increment_count = 0
		-- gcs:send_text(6,string.format("====== state num %d", state))
	end

	-- when mission stop by user or flow sensor
	-- if mission_state == 0 and state == 1 then
	-- 	output_pwm_srv_spray = 1000;
	-- 	if(current_mission_index > 2)then
	-- 	pos_to_continue = mission:get_item(current_mission_index-1)
	-- 	pos_to_continue:x(current_x)
	-- 	pos_to_continue:y(current_y)
	-- 	end
	-- 	state = 3;
	-- 	monitor_state = 0
		loop_mission_increment_count = 0
	-- 	-- gcs:send_text(6,string.format("====== state num %d", state))
	-- end

	-- check the breakpoint if we should set break point at waypoint #2 on resume
		-- 1. check if the resume length and remain mission length is the same
	-- if prevouis_mission_length - (current_mission_index-3) == (mission:num_commands()) and 
	-- 	-- 2. check if the last wp the same the last resum waypoint	
	-- 	prevouis_mission_waypoint_endpoint:x() == mission:get_item(mission:num_commands()-1):x() and 
	-- 	mission_state == 0 and state == 3 and 
	-- 	-- check if the we already set the resume point we don't have to do it again.
	-- 	mission:get_item(2):x() ~= pos_to_continue:x() and 
	-- 	mission:get_item(2):y() ~= pos_to_continue:y()  then
	-- 	-- we can not set state to 0; because the resume command is happen after
	-- 	-- this loop then it will override the waypoint #2 again. 
	-- 	-- but this also mean it will override the freshly upload mission :(
	-- 	-- only reset the FC distroy state == 3. 
	-- 	-- Oh, we can check num_commands and last_waypont
	-- 	mission:set_item(2, pos_to_continue);
	-- end

	if rc:get_radio_in(9) > 1600 and mission_state == 0 then
		testFlowSensor()
	end
  -- the rc chenel is index from 0 while in the ardupilot index from 1. strange!
	-- rc7 = rc:get_radio_in(6)
	-- if rc7 > 1550 then
	-- 	-- gcs:send_text(6,string.format("spray#: %d and spin#: %d",spray,spinner));
	-- 	SRV_Channels:set_output_pwm_chan(spray,spraySpeed)
	-- 	SRV_Channels:set_output_pwm_chan(spinner,spinnerSpeed)
	-- end
	-- if rc7 <= 1500 then
	-- 	SRV_Channels:set_output_pwm_chan(spray,1000)
	-- 	SRV_Channels:set_output_pwm_chan(spinner,1000)
	-- end
 
  	return update, 500 -- reschedules the loop
end

return update, 500 -- run immediately before starting to reschedule
