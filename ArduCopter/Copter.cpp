/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  ArduCopter (also known as APM, APM:Copter or just Copter)
 *  Wiki:           copter.ardupilot.org
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen,
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to:	Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel	        :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  Sebastian Quilter   :SmartRTL
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/ArduPilot/ardupilot/graphs/contributors
 *  Wiki: http://copter.ardupilot.org/
 *
 */

#include "Copter.h"
#include <math.h>
#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Copter, &copter, func, rate_hz, max_time_micros)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    SCHED_TASK(rc_loop,              100,    130),
    SCHED_TASK(throttle_loop,         50,     75),
    SCHED_TASK(update_GPS,            50,    200),
#if OPTFLOW == ENABLED
    SCHED_TASK_CLASS(OpticalFlow,          &copter.optflow,             update,         200, 160),
#endif
    SCHED_TASK(update_batt_compass,   10,    120),
    SCHED_TASK_CLASS(RC_Channels,          (RC_Channels*)&copter.g2.rc_channels,      read_aux_all,    10,     50),
    SCHED_TASK(arm_motors_check,      10,     50),
#if TOY_MODE_ENABLED == ENABLED
    SCHED_TASK_CLASS(ToyMode,              &copter.g2.toy_mode,         update,          10,  50),
#endif
    SCHED_TASK(auto_disarm_check,     10,     50),
    SCHED_TASK(auto_trim,             10,     75),
#if RANGEFINDER_ENABLED == ENABLED
    SCHED_TASK(read_rangefinder,      20,    100),
#endif
#if PROXIMITY_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Proximity,         &copter.g2.proximity,        update,         200,  50),
#endif
#if BEACON_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Beacon,            &copter.g2.beacon,           update,         400,  50),
#endif
#if VISUAL_ODOMETRY_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_VisualOdom,       &copter.g2.visual_odom,        update,         400,  50),
#endif
    SCHED_TASK(update_altitude,       10,    100), 
    SCHED_TASK(run_nav_updates,       50,    100), // mission.update() & mission.verify_cmd
    SCHED_TASK(update_throttle_hover,100,     90),
#if MODE_SMARTRTL_ENABLED == ENABLED
    SCHED_TASK_CLASS(ModeSmartRTL, &copter.mode_smartrtl,       save_position,    3, 100),
#endif
#if SPRAYER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AC_Sprayer,           &copter.sprayer,             update,           3,  90),
#endif
    SCHED_TASK(three_hz_loop,          3,     75),
    SCHED_TASK_CLASS(AP_ServoRelayEvents,  &copter.ServoRelayEvents,      update_events, 50,     75),
    SCHED_TASK_CLASS(AP_Baro,              &copter.barometer,           accumulate,      50,  90),
#if AC_FENCE == ENABLED
    SCHED_TASK_CLASS(AC_Fence,             &copter.fence,               update,          10, 100),
#endif
#if PRECISION_LANDING == ENABLED
    SCHED_TASK(update_precland,      400,     50),
#endif
#if FRAME_CONFIG == HELI_FRAME
    SCHED_TASK(check_dynamic_flight,  50,     75),
#endif
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(fourhundred_hz_logging,400,    50),
#endif
    SCHED_TASK_CLASS(AP_Notify,            &copter.notify,              update,          50,  90),
    SCHED_TASK(one_hz_loop,            1,    100),
    SCHED_TASK(ekf_check,             10,     75),
    SCHED_TASK(check_vibration,       10,     50),
    SCHED_TASK(gpsglitch_check,       10,     50),
    SCHED_TASK(landinggear_update,    10,     75),
    SCHED_TASK(standby_update,        100,    75),
    SCHED_TASK(lost_vehicle_check,    10,     50),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_receive, 400, 180),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_send,    400, 550),
#if MOUNT == ENABLED
    SCHED_TASK_CLASS(AP_Mount,             &copter.camera_mount,        update,          50,  75),
#endif
#if CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Camera,            &copter.camera,              update_trigger,  50,  75),
#endif
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350),
    SCHED_TASK(twentyfive_hz_logging, 25,    110),
    SCHED_TASK_CLASS(AP_Logger,      &copter.logger,           periodic_tasks, 400, 300),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,    &copter.ins,                 periodic,       400,  50),
    SCHED_TASK_CLASS(AP_Scheduler,         &copter.scheduler,           update_logging, 0.1,  75),
#if RPM_ENABLED == ENABLED
    SCHED_TASK(rpm_update,            40,    200),
#endif
    SCHED_TASK(compass_cal_update,   100,    100),
    SCHED_TASK(accel_cal_update,      10,    100),
    SCHED_TASK_CLASS(AP_TempCalibration,   &copter.g2.temp_calibration, update,          10, 100),
#if ADSB_ENABLED == ENABLED
    SCHED_TASK(avoidance_adsb_update, 10,    100),
#endif
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,          10,    100),
#endif
#if AC_TERRAIN == ENABLED
    SCHED_TASK(terrain_update,        10,    100),
#endif
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Gripper,           &copter.g2.gripper,          update,          10,  75),
#endif
#if WINCH_ENABLED == ENABLED
    SCHED_TASK(winch_update,          10,     50),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,   75),
#endif
#if BUTTON_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Button,            &copter.g2.button,           update,           5, 100),
#endif
#if STATS_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Stats,             &copter.g2.stats,            update,           1, 100),
#endif
#if OSD_ENABLED == ENABLED
    SCHED_TASK(publish_osd_info, 1, 10),
#endif
};

constexpr int8_t Copter::_failsafe_priorities[7];

void Copter::setup()
{
    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    // setup storage layout for copter
    StorageManager::set_layout_copter();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), MASK_LOG_PM);
    hal.gpio->pinMode(60,0);
}

void Copter::loop()
{
    scheduler.loop();
    G_Dt = scheduler.get_last_loop_time_s();
}


// Main loop - 400hz
void Copter::fast_loop()
{
    // update INS immediately to get current gyro data populated
    ins.update();

    // run low level rate controllers that only require IMU data
    attitude_control->rate_controller_run();

    // send outputs to the motors library immediately
    motors_output();

    // run EKF state estimator (expensive)
    // --------------------
    read_AHRS();

    #if FRAME_CONFIG == HELI_FRAME
        update_heli_control_dynamics();
        #if MODE_AUTOROTATE_ENABLED == ENABLED
            heli_update_autorotation();
        #endif
    #endif //HELI_FRAME

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading or position
    check_ekf_reset();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've landed or crashed
    update_land_and_crash_detectors();

    #if MOUNT == ENABLED
        // camera mount's fast update
        camera_mount.update_fast();
    #endif

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }
}

// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    rc().read_mode_switch();
    
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Copter::throttle_loop()
{
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_mix();

    // check auto_armed status
    update_auto_armed();

    #if FRAME_CONFIG == HELI_FRAME
        // update rotor speed
        heli_update_rotor_speed_targets();

        // update trad heli swash plate movement
        heli_update_landing_swash();
    #endif

    // compensate for ground effect (if enabled)
    update_ground_effect_detector();

    update_dynamic_notch();
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Copter::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if(AP::compass().enabled()) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors->get_throttle());
        compass.set_voltage(battery.voltage());
        compass.read();
    }
}

// Full rate logging of attitude, rate and pid loops
// should be run at 400hz
void Copter::fourhundred_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        Log_Write_Attitude();
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Copter::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        Log_Write_Attitude();
    }
    // log EKF attitude data
    if (should_log(MASK_LOG_ATTITUDE_MED) || should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
        if (rssi.enabled()) {
            logger.Write_RSSI();
        }
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS())) {
        pos_control->write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        logger.Write_Vibration();
    }
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control->control_monitor_log();
    #if PROXIMITY_ENABLED == ENABLED
            logger.Write_Proximity(g2.proximity);  // Write proximity sensor distances
    #endif
    #if BEACON_ENABLED == ENABLED
            logger.Write_Beacon(g2.beacon);
    #endif
        }
    #if FRAME_CONFIG == HELI_FRAME
        Log_Write_Heli();
    #endif
    // read value of level sensor
    if (RC_Channels::get_radio_in(9) > 1500){
        //uint16_t flow_val = wp_nav->readFlowSensor(60);
        uint16_t flow_val = hal.gpio->read(wp_nav->_sensor_pin); // nano  v5
        // uint16_t flow_val = hal.gpio->read(54); //       v5+

        if (sensor_loop_index >= 25){
            gcs().send_text(MAV_SEVERITY_INFO, "sensor val %i", flow_val);
            sensor_loop_index = 0;
        }
        sensor_loop_index = sensor_loop_index + 1;
    }

    /*FLOWSENSOR */
    if(get_mode()==3 && copter.mode_auto.cmd_16_index > 1){
        // not to trigger the flow sensor at the beginning of the mission.
        uint8_t delay_monitor_flow = 40;
        if (copter.mode_auto.cmd_16_index % 2 != 0) {
            mission_timer_not_to_monitor_flow_at_start_waypoint = 0;
            return;
        }
        if( copter.mode_auto.cmd_16_index % 2 == 0  && mission_timer_not_to_monitor_flow_at_start_waypoint < delay_monitor_flow) {
            mission_timer_not_to_monitor_flow_at_start_waypoint = mission_timer_not_to_monitor_flow_at_start_waypoint + 1;
            return;
        }
        
        // if empty tank stop copter
        if (mission_timer_not_to_monitor_flow_at_start_waypoint >= delay_monitor_flow && RC_Channels::get_radio_in(6) > 1400 ){
            uint16_t flow_val = hal.gpio->read(wp_nav->_sensor_pin); // nano  v5
            // uint16_t flow_val = hal.gpio->read(54); //       v5+
            flow_value = flow_value + flow_val ;
            // flow_value =  flow_value + flow_val;
            uint8_t loop_counter_max = 4;
            if ( flow_index >= loop_counter_max) {
                if ( RC_Channels::get_radio_in(9) > 1400){
                    gcs().send_text(MAV_SEVERITY_INFO, "flow val %i %i", flow_value/5, flow_val);
                }
                if ((flow_value/(loop_counter_max+1)) == 1 && !alert_empty_tank ){
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Water Tank Empty");
                    copter.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND);
                    alert_empty_tank = true;
                }
                flow_value = 0;
            }

            flow_index = flow_index >= loop_counter_max ? 0 : flow_index+1;
            
        }
    }
}

// twentyfive_hz_logging - should be run at 25hz
void Copter::twentyfive_hz_logging()
{
    #if HIL_MODE != HIL_MODE_DISABLED
        // HIL for a copter needs very fast update of the servo values
        gcs().send_message(MSG_SERVO_OUTPUT_RAW);
    #endif

    #if HIL_MODE == HIL_MODE_DISABLED
        if (should_log(MASK_LOG_ATTITUDE_FAST)) {
            Log_Write_EKF_POS();
        }

        if (should_log(MASK_LOG_IMU)) {
            logger.Write_IMU();
        }
    #endif

    #if PRECISION_LANDING == ENABLED
        // log output
        Log_Write_Precland();
    #endif

    #if MODE_AUTOROTATE_ENABLED == ENABLED
        if (should_log(MASK_LOG_ATTITUDE_MED) || should_log(MASK_LOG_ATTITUDE_FAST)) {
            //update autorotation log
            g2.arot.Log_Write_Autorotation();
        }
    #endif
    // Sitha set yaw on turn
    if(copter.mode_auto.mission.get_current_nav_index() > 2 and (copter.mode_auto.mission.get_current_nav_index())%2==0 && copter.mode_auto.mission.get_current_nav_index() < copter.mode_auto.mission.num_commands() && copter.get_mode()==3 && wp_nav->_has_oaradar){
        AP_Mission::Mission_Command temp_cmd;
        mode_auto.mission.get_next_nav_cmd(mode_auto.mission.get_current_nav_index()+1,temp_cmd);
        Location destination = mode_auto.loc_from_cmd(temp_cmd);
        int32_t bearingMe = wp_nav->get_wp_bearing_to_target(destination);
        // gcs().send_text(MAV_SEVERITY_INFO,"ind %i",bearingMe);
        copter.flightmode->auto_yaw.set_fixed_yaw(bearingMe * 0.01f, 0.0f, 1, false);
    }
    // Sitha set yaw on first waypoint and keep the yaw at waypoint
    // At RTL we set yaw at mode_rtl.cpp
    if((copter.mode_auto.mission.get_current_nav_index() == 2 || (copter.mode_auto.mission.get_current_nav_index())%2==1) && copter.get_mode()==3 && wp_nav->_has_oaradar){
        int32_t bearingMe = wp_nav->get_wp_bearing_to_destination();
        copter.flightmode->auto_yaw.set_fixed_yaw(bearingMe * 0.01f, wp_nav->_yaw_oa_rate, 1, false);
    }
    
}

// three_hz_loop - 3.3hz loop
void Copter::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();
    // check if we've lost terrain data
    failsafe_terrain_check();

    #if AC_FENCE == ENABLED
        // check if we have breached a fence
        fence_check();
    #endif // AC_FENCE_ENABLED
    // update ch6 in flight tuning
    tuning();
    
}

void Copter::set_pump_spinner_pwm(bool spray_state){
    if( spray_state == false) {
        SRV_Channels::set_output_pwm_chan( chan_pump , 1000);
        SRV_Channels::set_output_pwm_chan( chan_spinner , 1000);
        //gcs().send_text(MAV_SEVERITY_INFO, "spray off");
    }
    if(spray_state == true){
       if(wp_nav->_radio_type == 12){
            if(RC_Channels::get_radio_in(5) > 1600){
                rc6_pwm =  wp_nav->_pwm_pump < 60 ? (wp_nav->_pwm_pump + 30) * 10 + 1000 : 2000;
            }
            else if(RC_Channels::get_radio_in(5) > 1150 && RC_Channels::get_radio_in(5) < 1550 ){
                rc6_pwm = (wp_nav->_pwm_pump + 15) * 10 + 1000;
            }
            else if (RC_Channels::get_radio_in(5) < 1150){
                 rc6_pwm = 1000;
            } 
            SRV_Channels::set_output_pwm_chan( chan_pump , rc6_pwm);
            SRV_Channels::set_output_pwm_chan( chan_spinner , rc8_pwm = RC_Channels::get_radio_in(7) > 1080 ? wp_nav->_pwm_nozzle < 100 ? wp_nav->_pwm_nozzle *10+1000: 1950 : 1000 );
        
        }else{
            if (rc6_pwm != RC_Channels::get_radio_in(5) or rc8_pwm != RC_Channels::get_radio_in(7) ){
                rc6_pwm = RC_Channels::get_radio_in(5);
                rc8_pwm = RC_Channels::get_radio_in(7) > wp_nav->_pwm_nozzle*10+1000 ? wp_nav->_pwm_nozzle*10+1000 : RC_Channels::get_radio_in(7);
            }
            SRV_Channels::set_output_pwm_chan( chan_pump , rc6_pwm);
            SRV_Channels::set_output_pwm_chan( chan_spinner , rc8_pwm);    
        }
        
        //gcs().send_text(MAV_SEVERITY_INFO, "spray on");
    }
}
// one_hz_loop - runs at 1Hz
void Copter::one_hz_loop()
{
    // SPRAY speed update
    if(!chan_pump){
        SRV_Channels::find_channel(SRV_Channel::k_sprayer_pump,chan_pump);
    }
    if(!chan_spinner){
        SRV_Channels::find_channel(SRV_Channel::k_sprayer_spinner,chan_spinner);
        // gcs().send_text(MAV_SEVERITY_INFO, "sitha: =>chanel_pum %i", chan_spinner);
    }
    // PUMP switch on off
    if(RC_Channels::get_radio_in(6) /*chanel 7 switch*/ < 1600 && !pump_off_on_boot){
        pump_off_on_boot = true;
    }
    // switch the pump on by RC
    if (copter.get_mode()!=3 /*not auto*/ && chan_pump && chan_spinner && pump_off_on_boot){
        if (RC_Channels::get_radio_in(6) > 1500){
            uint16_t flow_val = hal.gpio->read(wp_nav->_sensor_pin); // nano  v5 pin 60
            // uint16_t flow_val = hal.gpio->read(54); // v5+ set brd_pwm_count = 4 //connect aux5 // we set brd_pwm to 4 because it is count only servo pin not m1-8 pin
            if(flow_val == 0){
                set_pump_spinner_pwm(true);
            }else{
                set_pump_spinner_pwm(false); 
            }
        } else {
            set_pump_spinner_pwm(false);         
        }
    }
    // gcs().send_text(MAV_SEVERITY_INFO, "_______missionState %i ",mode_auto.mission.state());
    /*(Done) misison complete loiter and stop spray*/ 
    if(mode_auto.mission.state() == 2 && wp_nav->loiter_state_after_mission_completed == false){
        //copter.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND);
        set_pump_spinner_pwm(false);   
        wp_nav->loiter_state_after_mission_completed = true;
        gcs().send_text(MAV_SEVERITY_INFO, "sitha: => ___________finished");
    }
    // stop spray on RTL when has water
    if(copter.get_mode()==6 && motors->armed()){
        set_pump_spinner_pwm(false);   
    }
    // 
    // MISSIONBREAKPOINT code start here.​ // this never get false until it is in auto.
    // so if mission finished success state is alway true.
    if( copter.get_mode() == 3 && mode_auto.mission.mission_uploaded_success_state ){
        mode_auto.mission.mission_uploaded_success_state = false;
        // gcs().send_text(MAV_SEVERITY_INFO, "sitha: => ___________breakpoint hit1");
    }

    /* BREAKPOINT resuming when empty tank*/
    // TEST case A, upload mission -> stop -> continue, stop @ wp 2, 3, 5, 7 then land 
    // hit resume, then fly auto -> hit loiter immediately -> hit auto see where it go.
    // wait till wp 5 -> land -> resume -> download plan
    // TEST case B: fly auto , stop at wp 4 and start again will spray at that point?
    if( !motors->armed() && mode_auto.mission.num_commands() &&  mode_auto.mission.mission_uploaded_success_state == true)
    {   
        // get new mission finish location after resume command hit
        mavlink_mission_item_int_t new_mission_finish_point ;
        mode_auto.mission.get_item(mode_auto.mission.num_commands()-1, new_mission_finish_point);
        // get new mission wapypoint #2 location after resume command hit
        mavlink_mission_item_int_t new_mission_waypoint_2 ;
        mode_auto.mission.get_item(2, new_mission_waypoint_2);
        // gcs().send_text(MAV_SEVERITY_INFO, "sitha: => new %i", new_mission_waypoint_2.x);
        // gcs().send_text(MAV_SEVERITY_INFO, "sitha: => old %i", mission_breakpoint.lat);
        
        /* WE SET breakpoint only if the end point is the same with new upload plan 
        and the length is sorter than the old one otherwise we can not upload new mission when 
        user make mistake*/
        if( current_mission_waypoint_finish_point.x == new_mission_finish_point.x && 
            current_mission_waypoint_finish_point.y == new_mission_finish_point.y &&
            mode_auto.mission.num_commands() < current_mission_length )
        {
            // gcs().send_text(MAV_SEVERITY_INFO, "sitha=> resume success %i",mode_auto.cmd_16_index );

            // TODO: this algorithm problem at the near of the end wp breakpoint
            // float PI = 3.14159265f;
            // float R = 6378137.0f; // Sitha: Earth radius in meter
            // float half_radian = 180.0f;
            
            // // devide by 10+e7 won't work well but multiply by 0.00000001 work well
            // float lat = (float)(wp_nav->origin_for_breakpoint.lat * 0.0000001f) * PI / half_radian; //convert degree to radian
            // float lon = (float)(wp_nav->origin_for_breakpoint.lng * 0.0000001f) * PI / half_radian;
            
            // float brng = (float)(wp_nav->wp_bearing)*0.01f * PI / half_radian;
            // float d = /*wp_nav->traveled_distance<400 && wp_nav->_fast_turn ? 0.0f :*/  (wp_nav->traveled_distance-70)*0.01f;
            // float newLat = asinf(sinf(lat) * cosf(d / R) + cosf(lat) * sinf(d / R) * cosf(brng));
            // float newLon = lon + atan2f(sinf(brng) * sinf(d / R) * cosf(lat), cosf(d / R) - sinf(lat) * sinf(newLat));
            // float latDegree = newLat * half_radian / PI;
            // float lonDegree = newLon * half_radian / PI; // result as radian so convert back to degree
            // new_mission_waypoint_2.x = (int32_t)(latDegree*10000000);
            // new_mission_waypoint_2.y = (int32_t)(lonDegree*10000000);    
            // gcs().send_text(MAV_SEVERITY_INFO, "sitha=> resume lat %f, %f brng %f", (float)(wp_nav->origin_for_breakpoint.lat*.0000001f), (float)(wp_nav->origin_for_breakpoint.lng*0.0000001f),(float)wp_nav->wp_bearing*0.01f);
            
            // TODO: calculate offset lat x metters and offset lng x meters // work well, better than above dont know why.
            float R = 6378137.00000000f;
            float dlat = wp_nav->_corect_coordinate_ns/R;
            float dlon = wp_nav->_corect_coordinate_we/(R*cosf(3.14150000f*(float)mission_breakpoint.lat/10000000/180.00000000f));
            float correct_breakpoint_lat = ((float)mission_breakpoint.lat/10000000-(dlat*180/3.14150000f));
            float correct_breakpoint_lng = ((float)mission_breakpoint.lng/10000000)-((dlon*180/3.14150000f));
            new_mission_waypoint_2.x = int32_t(correct_breakpoint_lat*10000000);
            new_mission_waypoint_2.y = (int32_t)(correct_breakpoint_lng*10000000);
            
            // in case pilot stop at the side turn don't set break point QGC will take out this wp
            if(mode_auto.cmd_16_index % 2 == 0 || wp_nav->_spray_all==1){
                mode_auto.mission.set_item(2, new_mission_waypoint_2 ); 
            } 
            // we set this to false so that it is not doing it again and again 
            // and when we upload it is not reset out mission
            mode_auto.mission.mission_uploaded_success_state = false;
            mode_auto.cmd_16_index= 0; // don't change this will effect resume waypoint at side turn
            current_mission_index = 0;
        }
    }
    
    // MISSION break by user and resume / this prevent user fly to somewhere and decide to resume so it resume to breakpoint
    if(mode_auto.mission.get_current_nav_index() > 1 && copter.get_mode()!=3 && motors->armed()){
        wp_nav->break_auto_by_user_state = true;
    }
    
    // if (motors->armed() && copter.get_mode()!=3 /*not equal auto*/
    //     && mode_auto.mission.state() == 0 
    //     && current_mission_index >= 3 && wp_nav->break_auto_by_user_state == true)
    // {
        
        // Stop implement go to breakpoint when user stop or sensor problem. 
        // TODO: should be an option if user wanted to incase sensor problem
        // gcs().send_text(MAV_SEVERITY_INFO, "sitha: => _________breakpoint success");
        
        // mavlink_mission_item_int_t current_waypoint ;
        // mode_auto.mission.get_item(current_mission_index-1, current_waypoint);
        // current_waypoint.x = mission_breakpoint.lat;
        // current_waypoint.y = mission_breakpoint.lng;
        // mode_auto.mission.set_item(current_mission_index-1, current_waypoint);
        // mode_auto.mission.set_current_cmd(current_mission_index-1);
    //     wp_nav->break_auto_by_user_state = false; // help not to set cmd_16_index + 1 so continue spray
    // }
    
    if(copter.get_mode()!=3 /*not = auto*/ ){
        // mode_auto.cmd_16_index = 0; // set this will make break by user no spray untill land
        if(!motors->armed()) { // prevent on take off set current waypoint the old user break point
            wp_nav->break_auto_by_user_state = false;
            wp_nav->reset_param_on_start_mission();
            wp_nav->loiter_state_after_mission_completed = false;
        }
    }
    // MISSIONBREAKPOINT code end here.
    // 
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    arming.update();

    if (!motors->armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.update_orientation();

        update_using_interlock();

        // check the user hasn't updated the frame class or type
        motors->set_frame_class_and_type((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());

        #if FRAME_CONFIG != HELI_FRAME
                // set all throttle channel settings
                motors->set_throttle_range(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
        #endif
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    // log terrain data
    terrain_logging();

    #if ADSB_ENABLED == ENABLED
        adsb.set_is_flying(!ap.land_complete);
    #endif

    AP_Notify::flags.flying = !ap.land_complete;
}

// called at 50hz
void Copter::update_GPS(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    bool gps_updated = false;

    gps.update();

    // log after every gps message
    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);
            gps_updated = true;
            break;
        }
    }

    if (gps_updated) {
    #if CAMERA == ENABLED
            camera.update();
    #endif
    }
}

void Copter::init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (ap.simple_mode == 0 || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    if (ap.simple_mode == 1) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = channel_roll->get_control_in()*simple_cos_yaw - channel_pitch->get_control_in()*simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*simple_sin_yaw + channel_pitch->get_control_in()*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = channel_roll->get_control_in()*super_simple_cos_yaw - channel_pitch->get_control_in()*super_simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*super_simple_sin_yaw + channel_pitch->get_control_in()*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    channel_roll->set_control_in(rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw());
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void Copter::update_super_simple_bearing(bool force_update)
{
    if (!force_update) {
        if (ap.simple_mode != 2) {
            return;
        }
        if (home_distance() < SUPER_SIMPLE_RADIUS) {
            return;
        }
    }

    const int32_t bearing = home_bearing();

    // check the bearing to home has changed by at least 5 degrees
    if (labs(super_simple_last_bearing - bearing) < 500) {
        return;
    }

    super_simple_last_bearing = bearing;
    const float angle_rad = radians((super_simple_last_bearing+18000)/100);
    super_simple_cos_yaw = cosf(angle_rad);
    super_simple_sin_yaw = sinf(angle_rad);
}

void Copter::read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
    #if HIL_MODE != HIL_MODE_DISABLED
        // update hil before ahrs update
        gcs().update();
    #endif

    // we tell AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
}

// read baro and log control tuning
void Copter::update_altitude()
{
    // read in baro altitude
    read_barometer();

    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}

#if OSD_ENABLED == ENABLED
void Copter::publish_osd_info()
{
    AP_OSD::NavInfo nav_info;
    nav_info.wp_distance = flightmode->wp_distance() * 1.0e-2f;
    nav_info.wp_bearing = flightmode->wp_bearing();
    nav_info.wp_xtrack_error = flightmode->crosstrack_error() * 1.0e-2f;
    nav_info.wp_number = mode_auto.mission.get_current_nav_index();
    osd.set_nav_info(nav_info);
}
#endif

/*
  constructor for main Copter class
 */
Copter::Copter(void)
    : logger(g.log_bitmask),
    flight_modes(&g.flight_mode1),
    control_mode(Mode::Number::STABILIZE),
    simple_cos_yaw(1.0f),
    super_simple_cos_yaw(1.0),
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    rc_throttle_control_in_filter(1.0f),
    inertial_nav(ahrs),
    param_loader(var_info),
    flightmode(&mode_stabilize)

{
    // init sensor error logging flags
    sensor_health.baro = true;
    sensor_health.compass = true;
}

Copter copter;

AP_HAL_MAIN_CALLBACKS(&copter);
