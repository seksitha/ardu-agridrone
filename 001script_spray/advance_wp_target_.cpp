// when pilot switch to auto => 
// mission.start_or_resume => auto.start_command => auto.do_nav_wp => auto.wp_start => auto.wp_run





bool ModeAuto::init(bool ignore_checks)
{
    if (mission.num_commands() > 1 || ignore_checks) {
        _mode = Auto_Loiter;

        // reject switching to auto mode if landed with motors armed but first command is not a takeoff (reduce chance of flips)
        if (motors->armed() && copter.ap.land_complete && !mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto: Missing Takeoff Cmd");
            return false;
        }

        // stop ROI from carrying over from previous runs of the mission
        // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
        if (auto_yaw.mode() == AUTO_YAW_ROI) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }

        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();

        // clear guided limits
        copter.mode_guided.limit_clear();

        // start/resume the mission (based on MIS_RESTART parameter)
        mission.start_or_resume();
        return true;
    } else {
        return false;
    }
}
void ModeAuto::run() // command 16 will call wp_run()

void ModeAuto::wp_run() // this is a low frequency loop
{
    
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        wp_nav->wp_and_spline_init();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}

bool AC_WPNav::update_wpnav()
{
    bool ret = true;

    // get dt from pos controller
    float dt = _pos_control.get_dt();

    // allow the accel and speed values to be set without changing
    // out of auto mode. This makes it easier to tune auto flight
    _pos_control.set_max_accel_xy(_wp_accel_cmss);
    _pos_control.set_max_accel_z(_wp_accel_z_cmss);

    // wp_speed_update - update _pos_control.set_max_speed_xy if speed change has been requested
    wp_speed_update(dt);

    // advance the target if necessary
    if (!advance_wp_target_along_track(dt)) {
        // To-Do: handle inability to advance along track (probably because of missing terrain data)
        ret = false;
    }

    // freeze feedforwards during known discontinuities
    if (_flags.new_wp_destination) {
        _flags.new_wp_destination = false;
        _pos_control.freeze_ff_z();
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "sitha: =>fast %i",  _flags.fast_waypoint);
    _pos_control.update_xy_controller();
    check_wp_leash_length();

    _wp_last_update = AP_HAL::millis();
    

    return ret;
}

bool AC_WPNav::advance_wp_target_along_track(float dt)
{
    float track_covered;        // distance (in cm) along the track that the vehicle has traveled.  Measured by drawing a perpendicular line from the track to the vehicle.
    Vector3f track_error;       // distance error (in cm) from the track_covered position (i.e. closest point on the line to the vehicle) and the vehicle
    float track_desired_max;    // the farthest distance (in cm) along the track that the leash will allow
    float track_leash_slack;    // additional distance (in cm) along the track from our track_covered position that our leash will allow
    bool reached_leash_limit = false;   // true when track has reached leash limit and we need to slow down the target point

    // 2. calculate terrain adjustments
    float terr_offset = 0.0f;
    if (_terrain_alt && !get_terrain_offset(terr_offset)) {
        return false;
    }

    /* BREAKPOINT resuming when empty tank*/
    if(copter.mode_auto.mission.state() != 0){ // we don't want code to run at RTL because it use wpnav controller too
        if(copter.mode_auto.mission.get_current_nav_index() == 1){
            copter.mode_auto.mission.get_item(copter.mode_auto.mission.num_commands() - 1,copter.current_mission_waypoint_finish_point);
        }
        copter.ahrs.get_position(copter.mission_breakpoint); // assigning current post to mission_breakpoint
        copter.current_mission_length = copter.mode_auto.mission.num_commands(); // total command + 1
        copter.current_mission_index = copter.mode_auto.mission.get_current_nav_index();
        
    }

    /* PUMPSPINNER speed change detector: pump and spinner only at spray time or will spray all the time */
    if (copter.rc6_pwm != RC_Channels::get_radio_in(5) or copter.rc8_pwm != RC_Channels::get_radio_in(7) ){
        if (copter.mission_16_index % 2 == 0 && copter.mode_auto.cmd_16_index > 1) copter.set_pump_spinner_pwm(true);
    }
    
    /* ALT: Altitude*/
    int32_t throttle_val = copter.channel_throttle->get_radio_in();
    // positive throttle
    if (throttle_val > 1550 && _flags_change_alt_by_pilot ){
        // test with SITL carefull with throttle not come back to 1500
        // limit height 20m up only Test with and next waypoint clime rate is set to 0 and if throttle not 1500 it will keep going up
        _pilot_clime_cm < 2000 ? _pilot_clime_cm = (_pilot_clime_cm + ((float)throttle_val/5000)) : _pilot_clime_cm;
    }
    // negative throttle
    else if (throttle_val < 1450 && _flags_change_alt_by_pilot  && throttle_val > 1000 /* SITL start at rc 3 1000*/ ){
        // limit height -10monly
        // test with SITL carefull with throttle not come back to 1500 and next waypoint clime rate is set to 0 and if throttle not 1500 it will keep going down
        _pilot_clime_cm = (_pilot_clime_cm - 0.3);
    }
    // mid stick 
    else {
        // if we set like this, it is going to be reverted to original alt which copter stay at mission alt
        // so this should comment out. 
        // _pilot_clime_cm = 0.0f;
    }   
    // gcs().send_text(MAV_SEVERITY_INFO, "sitha: =>flags 2 %f",  _pilot_clime_cm);
    // 3.calculate 3d vector from segment's origin
    // 1. get current location
    const Vector3f &curr_pos = _inav.get_position();
    // _origin.x = _origin.x+200;
    Vector3f curr_delta = (curr_pos - Vector3f(0,0,terr_offset)) - _origin;
    if (_flags_change_alt_by_pilot){
        curr_delta.z -= _pilot_clime_cm;
        // gcs().send_text(MAV_SEVERITY_INFO, "sitha: =>hit %f", _pos_delta_unit.z);
    }
    
    // 4.calculate how far along the track we are in cm?
    track_covered = curr_delta.x * _pos_delta_unit.x + (curr_delta.y) * _pos_delta_unit.y + curr_delta.z * _pos_delta_unit.z;
    // 5.calculate the point closest to the vehicle on the segment from origin to destination
    Vector3f track_covered_pos = _pos_delta_unit * track_covered;
    // 6.calculate the distance vector from the vehicle to the closest point on the segment from origin to destination
    // curr_delta is length has travel
    track_error = curr_delta - track_covered_pos;
    // 7.calculate the horizontal error
    _track_error_xy = norm(track_error.x, track_error.y);
    // 8. calculate the vertical error
    float track_error_z = fabsf(track_error.z);
    // 9. get up leash if we are moving up, down leash if we are moving down
    // leash_up / down return distan cm of the vehicle
    float leash_z = track_error.z >= 0 ? _pos_control.get_leash_up_z() : _pos_control.get_leash_down_z();
    // 10. use pythagoras's theorem calculate how far along the track we could move the intermediate target before reaching the end of the leash
    //   track_desired_max is the distance from the vehicle to our target point along the track.  It is the "hypotenuse" which we want to be no longer than our leash (aka _track_leash_length)
    //   track_error is the line from the vehicle to the closest point on the track.  It is the "opposite" side
    //   track_leash_slack is the line from the closest point on the track to the target point.  It is the "adjacent" side.  We adjust this so the track_desired_max is no longer than the leash
    float track_leash_length_abs = fabsf(_track_leash_length);
    float track_error_max_abs = MAX(_track_leash_length*track_error_z/leash_z, _track_leash_length*_track_error_xy/_pos_control.get_leash_xy());
    track_leash_slack = (track_leash_length_abs > track_error_max_abs) ? safe_sqrt(sq(_track_leash_length) - sq(track_error_max_abs)) : 0;
    track_desired_max = track_covered + track_leash_slack;
    
    // 11.check if target is already beyond the leash
    if (_track_desired > track_desired_max) {
        reached_leash_limit = true;
    }

    // 12.get current velocity
    const Vector3f &curr_vel = _inav.get_velocity();
    // get speed along track
    float speed_along_track = curr_vel.x * _pos_delta_unit.x + curr_vel.y * _pos_delta_unit.y + curr_vel.z * _pos_delta_unit.z;

    // 13.calculate point at which velocity switches from linear to sqrt
    float linear_velocity = _pos_control.get_max_speed_xy();
    float kP = _pos_control.get_pos_xy_p().kP();
    if (is_positive(kP)) {   // avoid divide by zero
        linear_velocity = _track_accel/kP;
    }

    // 14.let the limited_speed_xy_cms be some range above or below current velocity along track
    if (speed_along_track < -linear_velocity) {
        // we are traveling fast in the opposite direction of travel to the waypoint so do not move the intermediate point
        _limited_speed_xy_cms = 0;
    }else{
        // increase intermediate target point's velocity if not yet at the leash limit
        if(dt > 0 && !reached_leash_limit) {
            _limited_speed_xy_cms += 2.0f * _track_accel * dt;
        }
        // do not allow speed to be below zero or over top speed
        _limited_speed_xy_cms = constrain_float(_limited_speed_xy_cms, 0.0f, _track_speed);

        // check if we should begin slowing down
        // track_length is set in set_wp_origin_and destination
        if (!_flags.fast_waypoint) {
            float dist_to_dest = _track_length - _track_desired;
            if (!_flags.slowing_down && dist_to_dest <= _slow_down_dist) {
                _flags.slowing_down = true;
            }
            // if target is slowing down, limit the speed
            if (_flags.slowing_down) {
                _limited_speed_xy_cms = MIN(_limited_speed_xy_cms, get_slow_down_speed(dist_to_dest, _track_accel));
            }
        }

        // if our current velocity is within the linear velocity range limit the intermediate point's velocity to be no more than the linear_velocity above or below our current velocity
        if (fabsf(speed_along_track) < linear_velocity) {
            _limited_speed_xy_cms = constrain_float(_limited_speed_xy_cms,speed_along_track-linear_velocity,speed_along_track+linear_velocity);
        }
    }
    // 15.advance the current target
    if (!reached_leash_limit) {
    	_track_desired += _limited_speed_xy_cms * dt;

    	// reduce speed if we reach end of leash
        if (_track_desired > track_desired_max) {
        	_track_desired = track_desired_max;
        	_limited_speed_xy_cms -= 2.0f * _track_accel * dt;
        	if (_limited_speed_xy_cms < 0.0f) {
        	    _limited_speed_xy_cms = 0.0f;
        	}
    	}
    }

    // 16.do not let desired point go past the end of the track unless it's a fast waypoint
    if (_flags.fast_waypoint) {
        _track_desired = constrain_float(_track_desired, 0, _track_length + 10.0f);
    } else {
        _track_desired = constrain_float(_track_desired, 0, _track_length);
    }

    // 17.recalculate the desired position
    // _pos_delta_unit is a fix percentage per wp (how is this defined)
    // sitha: origin is the pos start takeoff 0.587cm, reach tf high 400cm, reach wp2 500cm, reach wp3 700cm
    // sitha: take off 4m if I set the _origin.z = 500 then the take off is 9m 
    // _origin.z=200;

    Vector3f final_target = _origin + _pos_delta_unit * _track_desired; // only calculate xy
    // convert final_target.z to altitude above the ekf origin
    final_target.z += terr_offset;
    if (_flags_change_alt_by_pilot){
        final_target.z += _pilot_clime_cm;
    }
    _pos_control.set_pos_target(final_target);

   

    // 18.check if we've reached the waypoint
    if( !_flags.reached_destination ) {
        // gcs().send_text(MAV_SEVERITY_INFO, "__________tracklenght %f %f", _track_desired, _track_length);
        if( _track_desired >= _track_length ) { // hit only once
            // "fast" waypoints are complete once the intermediate point reaches the destination
            // gcs().send_text(MAV_SEVERITY_INFO, "_____________hit yyy");
            if (_flags.fast_waypoint) {
                // Vector3f dist_to_dest = (curr_pos - Vector3f(0,0,terr_offset)) - _destination;
                // if (_flags_change_alt_by_pilot){
                //     dist_to_dest.z -= _pilot_clime_cm;
                // }
                _flags.reached_destination = true;
                // gcs().send_text(MAV_SEVERITY_INFO, "____¿se_________hit flags");
                
                _flags_change_alt_by_pilot = true;
            }else{
                // regular waypoints also require the copter to be within the waypoint radius
                // gcs().send_text(MAV_SEVERITY_INFO, "_____________hit no");                                                                                                                                  
                Vector3f dist_to_dest = (curr_pos - Vector3f(0,0,terr_offset)) - _destination;
                if (_flags_change_alt_by_pilot){
                    dist_to_dest.z -= _pilot_clime_cm;
                }
                if( dist_to_dest.length() <= _wp_radius_cm ) {
                    _flags_change_alt_by_pilot = true;
                    _flags.reached_destination = true;
                    // allow change altitude after the takeoff 
                }
            }
        }
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "sitha: => leash4 %i", _flags.reached_destination);
    // 19.update the target yaw if origin and destination are at least 2m apart horizontally
    if (_track_length_xy >= WPNAV_YAW_DIST_MIN) {
        if (_pos_control.get_leash_xy() < WPNAV_YAW_DIST_MIN) {
            // if the leash is short (i.e. moving slowly) and destination is at least 2m horizontally, point along the segment from origin to destination
            set_yaw_cd(get_bearing_cd(_origin, _destination));
        } else {
            Vector3f horiz_leash_xy = final_target - curr_pos;
            horiz_leash_xy.z = 0;
            if (horiz_leash_xy.length() > MIN(WPNAV_YAW_DIST_MIN, _pos_control.get_leash_xy()*WPNAV_YAW_LEASH_PCT_MIN)) {
                set_yaw_cd(RadiansToCentiDegrees(atan2f(horiz_leash_xy.y,horiz_leash_xy.x)));
            }
        }
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "__________tracklenght %f %f", _track_desired, _track_length);
    // 20.successfully advanced along track
    return true;
}
