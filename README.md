# quadcoper
Ap_arming.cpp
#include "Copter.h"

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
 #include <AP_ToshibaCAN/AP_ToshibaCAN.h>
#endif

// performs pre-arm checks. expects to be called at 1hz.
void AP_Arming_Copter::update(void)
{
    // perform pre-arm checks & display failures every 30 seconds
    static uint8_t pre_arm_display_counter = PREARM_DISPLAY_PERIOD/2;
    pre_arm_display_counter++;
    bool display_fail = false;
    if ((_arming_options & uint32_t(AP_Arming::ArmingOptions::DISABLE_PREARM_DISPLAY)) == 0 &&
        pre_arm_display_counter >= PREARM_DISPLAY_PERIOD) {
        display_fail = true;
        pre_arm_display_counter = 0;
    }

    pre_arm_checks(display_fail);
}

bool AP_Arming_Copter::pre_arm_checks(bool display_failure)
{
    const bool passed = run_pre_arm_checks(display_failure);
    set_pre_arm_check(passed);
    return passed;
}

// perform pre-arm checks
//  return true if the checks pass successfully
bool AP_Arming_Copter::run_pre_arm_checks(bool display_failure)
{
    // exit immediately if already armed
    if (copter.motors->armed()) {
        return true;
    }

    // check if motor interlock and Emergency Stop aux switches are used
    // at the same time.  This cannot be allowed.
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK) &&
        rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_ESTOP)){
        check_failed(display_failure, "Interlock/E-Stop Conflict");
        return false;
    }

    // check if motor interlock aux switch is in use
    // if it is, switch needs to be in disabled position to arm
    // otherwise exit immediately.
    if (copter.ap.using_interlock && copter.ap.motor_interlock_switch) {
        check_failed(display_failure, "Motor Interlock Enabled");
        return false;
    }

    // if we are using motor Estop switch, it must not be in Estop position
    if (SRV_Channels::get_emergency_stop()){
        check_failed(display_failure, "Motor Emergency Stopped");
        return false;
    }

    if (!disarm_switch_checks(display_failure)) {
        return false;
    }

    // if pre arm checks are disabled run only the mandatory checks
    if (checks_to_perform == 0) {
        return mandatory_checks(display_failure);
    }

    return parameter_checks(display_failure)
        & motor_checks(display_failure)
        & oa_checks(display_failure)
        & gcs_failsafe_check(display_failure)
        & winch_checks(display_failure)
        & alt_checks(display_failure)
#if AP_AIRSPEED_ENABLED
        & AP_Arming::airspeed_checks(display_failure)
#endif
        & AP_Arming::pre_arm_checks(display_failure);
}

bool AP_Arming_Copter::barometer_checks(bool display_failure)
{
    if (!AP_Arming::barometer_checks(display_failure)) {
        return false;
    }

    bool ret = true;
    // check Baro
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_BARO)) {
        // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
        // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
        // that may differ from the baro height due to baro drift.
        nav_filter_status filt_status = copter.inertial_nav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref) {
            if (fabsf(copter.inertial_nav.get_position_z_up_cm() - copter.baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
                check_failed(ARMING_CHECK_BARO, display_failure, "Altitude disparity");
                ret = false;
            }
        }
    }
    return ret;
}

bool AP_Arming_Copter::ins_checks(bool display_failure)
{
    bool ret = AP_Arming::ins_checks(display_failure);

    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS)) {

        // get ekf attitude (if bad, it's usually the gyro biases)
        if (!pre_arm_ekf_attitude_check()) {
            check_failed(ARMING_CHECK_INS, display_failure, "EKF attitude is bad");
            ret = false;
        }
    }

    return ret;
}

bool AP_Arming_Copter::board_voltage_checks(bool display_failure)
{
    if (!AP_Arming::board_voltage_checks(display_failure)) {
        return false;
    }

    // check battery voltage
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_VOLTAGE)) {
        if (copter.battery.has_failsafed()) {
            check_failed(ARMING_CHECK_VOLTAGE, display_failure, "Battery failsafe");
            return false;
        }
    }

    return true;
}

bool AP_Arming_Copter::parameter_checks(bool display_failure)
{
    // check various parameter values
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_PARAMETERS)) {

        // failsafe parameter checks
        if (copter.g.failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (copter.channel_throttle->get_radio_min() <= copter.g.failsafe_throttle_value+10 || copter.g.failsafe_throttle_value < 910) {
                check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check FS_THR_VALUE");
                return false;
            }
        }
        if (copter.g.failsafe_gcs == FS_GCS_ENABLED_CONTINUE_MISSION) {
            // FS_GCS_ENABLE == 2 has been removed
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "FS_GCS_ENABLE=2 removed, see FS_OPTIONS");
        }

        // lean angle parameter check
        if (copter.aparm.angle_max < 1000 || copter.aparm.angle_max > 8000) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check ANGLE_MAX");
            return false;
        }

        // acro balance parameter check
#if MODE_ACRO_ENABLED == ENABLED || MODE_SPORT_ENABLED == ENABLED
        if ((copter.g.acro_balance_roll > copter.attitude_control->get_angle_roll_p().kP()) || (copter.g.acro_balance_pitch > copter.attitude_control->get_angle_pitch_p().kP())) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check ACRO_BAL_ROLL/PITCH");
            return false;
        }
#endif

        // pilot-speed-up parameter check
        if (copter.g.pilot_speed_up <= 0) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check PILOT_SPEED_UP");
            return false;
        }

        #if FRAME_CONFIG == HELI_FRAME
        if (copter.g2.frame_class.get() != AP_Motors::MOTOR_FRAME_HELI_QUAD &&
            copter.g2.frame_class.get() != AP_Motors::MOTOR_FRAME_HELI_DUAL &&
            copter.g2.frame_class.get() != AP_Motors::MOTOR_FRAME_HELI) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Invalid Heli FRAME_CLASS");
            return false;
        }

        // check helicopter parameters
        if (!copter.motors->parameter_check(display_failure)) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Heli motors checks failed");
            return false;
        }

        char fail_msg[50];
        // check input mangager parameters
        if (!copter.input_manager.parameter_check(fail_msg, ARRAY_SIZE(fail_msg))) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "%s", fail_msg);
            return false;
        }

        // Inverted flight feature disabled for Heli Single and Dual frames
        if (copter.g2.frame_class.get() != AP_Motors::MOTOR_FRAME_HELI_QUAD &&
            rc().find_channel_for_option(RC_Channel::aux_func_t::INVERTED) != nullptr) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Inverted flight option not supported");
            return false;
        }
        // Ensure an Aux Channel is configured for motor interlock
        if (rc().find_channel_for_option(RC_Channel::aux_func_t::MOTOR_INTERLOCK) == nullptr) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Motor Interlock not configured");
            return false;
        }

        #else
        if (copter.g2.frame_class.get() == AP_Motors::MOTOR_FRAME_HELI_QUAD ||
            copter.g2.frame_class.get() == AP_Motors::MOTOR_FRAME_HELI_DUAL ||
            copter.g2.frame_class.get() == AP_Motors::MOTOR_FRAME_HELI) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Invalid MultiCopter FRAME_CLASS");
            return false;
        }

        // checks MOT_PWM_MIN/MAX for acceptable values
        if (!copter.motors->check_mot_pwm_params()) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Check MOT_PWM_MIN/MAX");
            return false;
        }
        #endif // HELI_FRAME

        // checks when using range finder for RTL
#if MODE_RTL_ENABLED == ENABLED
        if (copter.mode_rtl.get_alt_type() == ModeRTL::RTLAltType::RTL_ALTTYPE_TERRAIN) {
            // get terrain source from wpnav
            const char *failure_template = "RTL_ALT_TYPE is above-terrain but %s";
            switch (copter.wp_nav->get_terrain_source()) {
            case AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE:
                check_failed(ARMING_CHECK_PARAMETERS, display_failure, failure_template, "no terrain data");
                return false;
                break;
            case AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER:
                if (!copter.rangefinder_state.enabled || !copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
                    check_failed(ARMING_CHECK_PARAMETERS, display_failure, failure_template, "no rangefinder");
                    return false;
                }
                // check if RTL_ALT is higher than rangefinder's max range
                if (copter.g.rtl_altitude > copter.rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)) {
                    check_failed(ARMING_CHECK_PARAMETERS, display_failure, failure_template, "RTL_ALT>RNGFND_MAX_CM");
                    return false;
                }
                break;
            case AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
#if AP_TERRAIN_AVAILABLE
                if (!copter.terrain.enabled()) {
                    check_failed(ARMING_CHECK_PARAMETERS, display_failure, failure_template, "terrain disabled");
                    return false;
                }
                // check terrain data is loaded
                uint16_t terr_pending, terr_loaded;
                copter.terrain.get_statistics(terr_pending, terr_loaded);
                if (terr_pending != 0) {
                    check_failed(ARMING_CHECK_PARAMETERS, display_failure, failure_template, "waiting for terrain data");
                    return false;
                }
#else
                check_failed(ARMING_CHECK_PARAMETERS, display_failure, failure_template, "terrain disabled");
                return false;
#endif
                break;
            }
        }
#endif

        // check adsb avoidance failsafe
#if HAL_ADSB_ENABLED
        if (copter.failsafe.adsb) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "ADSB threat detected");
            return false;
        }
#endif

        // ensure controllers are OK with us arming:
        char failure_msg[50];
        if (!copter.pos_control->pre_arm_checks("PSC", failure_msg, ARRAY_SIZE(failure_msg))) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
            return false;
        }
        if (!copter.attitude_control->pre_arm_checks("ATC", failure_msg, ARRAY_SIZE(failure_msg))) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Bad parameter: %s", failure_msg);
            return false;
        }
    }

    return true;
}

// check motor setup was successful
bool AP_Arming_Copter::motor_checks(bool display_failure)
{
    // check motors initialised  correctly
    if (!copter.motors->initialised_ok()) {
        check_failed(display_failure, "Check firmware or FRAME_CLASS");
        return false;
    }

	// servo_test check
#if FRAME_CONFIG == HELI_FRAME
    if (copter.motors->servo_test_running()) {
        check_failed(display_failure, "Servo Test is still running");
        return false;
    }
#endif
    // further checks enabled with parameters
    if (!check_enabled(ARMING_CHECK_PARAMETERS)) {
        return true;
    }

    // if this is a multicopter using ToshibaCAN ESCs ensure MOT_PMW_MIN = 1000, MOT_PWM_MAX = 2000
#if HAL_MAX_CAN_PROTOCOL_DRIVERS && (FRAME_CONFIG != HELI_FRAME)
    bool tcan_active = false;
    uint8_t tcan_index = 0;
    const uint8_t num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < num_drivers; i++) {
        if (AP::can().get_driver_type(i) == AP_CANManager::Driver_Type_ToshibaCAN) {
            tcan_active = true;
            tcan_index = i;
        }
    }
    if (tcan_active) {
        // check motor range parameters
        if (copter.motors->get_pwm_output_min() != 1000) {
            check_failed(display_failure, "TCAN ESCs require MOT_PWM_MIN=1000");
            return false;
        }
        if (copter.motors->get_pwm_output_max() != 2000) {
            check_failed(display_failure, "TCAN ESCs require MOT_PWM_MAX=2000");
            return false;
        }

        // check we have an ESC present for every SERVOx_FUNCTION = motorx
        // find and report first missing ESC, extra ESCs are OK
        AP_ToshibaCAN *tcan = AP_ToshibaCAN::get_tcan(tcan_index);
        const uint16_t motors_mask = copter.motors->get_motor_mask();
        const uint16_t esc_mask = tcan->get_present_mask();
        uint8_t escs_missing = 0;
        uint8_t first_missing = 0;
        for (uint8_t i = 0; i < 16; i++) {
            uint32_t bit = 1UL << i;
            if (((motors_mask & bit) > 0) && ((esc_mask & bit) == 0)) {
                escs_missing++;
                if (first_missing == 0) {
                    first_missing = i+1;
                }
            }
        }
        if (escs_missing > 0) {
            check_failed(display_failure, "TCAN missing %d escs, check #%d", (int)escs_missing, (int)first_missing);
            return false;
        }
    }
#endif

    return true;
}

bool AP_Arming_Copter::oa_checks(bool display_failure)
{
#if AC_OAPATHPLANNER_ENABLED == ENABLED
    char failure_msg[50];
    if (copter.g2.oa.pre_arm_check(failure_msg, ARRAY_SIZE(failure_msg))) {
        return true;
    }
    // display failure
    if (strlen(failure_msg) == 0) {
        check_failed(display_failure, "%s", "Check Object Avoidance");
    } else {
        check_failed(display_failure, "%s", failure_msg);
    }
    return false;
#else
    return true;
#endif
}

bool AP_Arming_Copter::rc_calibration_checks(bool display_failure)
{
    const RC_Channel *channels[] = {
        copter.channel_roll,
        copter.channel_pitch,
        copter.channel_throttle,
        copter.channel_yaw
    };

    copter.ap.pre_arm_rc_check = rc_checks_copter_sub(display_failure, channels)
        & AP_Arming::rc_calibration_checks(display_failure);

    return copter.ap.pre_arm_rc_check;
}

// performs pre_arm gps related checks and returns true if passed
bool AP_Arming_Copter::gps_checks(bool display_failure)
{
    // check if fence requires GPS
    bool fence_requires_gps = false;
    #if AC_FENCE == ENABLED
    // if circular or polygon fence is enabled we need GPS
    fence_requires_gps = (copter.fence.get_enabled_fences() & (AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON)) > 0;
    #endif

    // check if flight mode requires GPS
    bool mode_requires_gps = copter.flightmode->requires_GPS() || fence_requires_gps || (copter.simple_mode == Copter::SimpleMode::SUPERSIMPLE);

    // call parent gps checks
    if (mode_requires_gps) {
        if (!AP_Arming::gps_checks(display_failure)) {
            AP_Notify::flags.pre_arm_gps_check = false;
            return false;
        }
    }

    // run mandatory gps checks first
    if (!mandatory_gps_checks(display_failure)) {
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // return true if GPS is not required
    if (!mode_requires_gps) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // return true immediately if gps check is disabled
    if (!(checks_to_perform == ARMING_CHECK_ALL || checks_to_perform & ARMING_CHECK_GPS)) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
    if (copter.gps.get_hdop() > copter.g.gps_hdop_good) {
        check_failed(ARMING_CHECK_GPS, display_failure, "High GPS HDOP");
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // if we got here all must be ok
    AP_Notify::flags.pre_arm_gps_check = true;
    return true;
}

// check ekf attitude is acceptable
bool AP_Arming_Copter::pre_arm_ekf_attitude_check()
{
    // get ekf filter status
    nav_filter_status filt_status = copter.inertial_nav.get_filter_status();

    return filt_status.flags.attitude;
}

// check nothing is too close to vehicle
bool AP_Arming_Copter::proximity_checks(bool display_failure) const
{
#if HAL_PROXIMITY_ENABLED

    if (!AP_Arming::proximity_checks(display_failure)) {
        return false;
    }

    if (!((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_PARAMETERS))) {
        // check is disabled
        return true;
    }

    // get closest object if we might use it for avoidance
#if AC_AVOID_ENABLED == ENABLED
    float angle_deg, distance;
    if (copter.avoid.proximity_avoidance_enabled() && copter.g2.proximity.get_closest_object(angle_deg, distance)) {
        // display error if something is within 60cm
        const float tolerance = 0.6f;
        if (distance <= tolerance) {
            check_failed(ARMING_CHECK_PARAMETERS, display_failure, "Proximity %d deg, %4.2fm (want > %0.1fm)", (int)angle_deg, (double)distance, (double)tolerance);
            return false;
        }
    }
#endif

#endif
    return true;
}

// performs mandatory gps checks.  returns true if passed
bool AP_Arming_Copter::mandatory_gps_checks(bool display_failure)
{
    // check if flight mode requires GPS
    bool mode_requires_gps = copter.flightmode->requires_GPS();

    // always check if inertial nav has started and is ready
    const auto &ahrs = AP::ahrs();
    char failure_msg[50] = {};
    if (!ahrs.pre_arm_check(mode_requires_gps, failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "AHRS: %s", failure_msg);
        return false;
    }

    // check if fence requires GPS
    bool fence_requires_gps = false;
    #if AC_FENCE == ENABLED
    // if circular or polygon fence is enabled we need GPS
    fence_requires_gps = (copter.fence.get_enabled_fences() & (AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON)) > 0;
    #endif

    if (mode_requires_gps) {
        if (!copter.position_ok()) {
            // vehicle level position estimate checks
            check_failed(display_failure, "Need Position Estimate");
            return false;
        }
    } else {
        if (fence_requires_gps) {
            if (!copter.position_ok()) {
                // clarify to user why they need GPS in non-GPS flight mode
                check_failed(display_failure, "Fence enabled, need position estimate");
                return false;
            }
        } else {
            // return true if GPS is not required
            return true;
        }
    }

    // check for GPS glitch (as reported by EKF)
    nav_filter_status filt_status;
    if (ahrs.get_filter_status(filt_status)) {
        if (filt_status.flags.gps_glitching) {
            check_failed(display_failure, "GPS glitching");
            return false;
        }
    }

    // check EKF's compass, position and velocity variances are below failsafe threshold
    if (copter.g.fs_ekf_thresh > 0.0f) {
        float vel_variance, pos_variance, hgt_variance, tas_variance;
        Vector3f mag_variance;
        ahrs.get_variances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance);
        if (mag_variance.length() >= copter.g.fs_ekf_thresh) {
            check_failed(display_failure, "EKF compass variance");
            return false;
        }
        if (pos_variance >= copter.g.fs_ekf_thresh) {
            check_failed(display_failure, "EKF position variance");
            return false;
        }
        if (vel_variance >= copter.g.fs_ekf_thresh) {
            check_failed(display_failure, "EKF velocity variance");
            return false;
        }
    }

    // check if home is too far from EKF origin
    if (copter.far_from_EKF_origin(ahrs.get_home())) {
        check_failed(display_failure, "Home too far from EKF origin");
        return false;
    }

    // check if vehicle is too far from EKF origin
    if (copter.far_from_EKF_origin(copter.current_loc)) {
        check_failed(display_failure, "Vehicle too far from EKF origin");
        return false;
    }

    // if we got here all must be ok
    return true;
}

// Check GCS failsafe
bool AP_Arming_Copter::gcs_failsafe_check(bool display_failure)
{
    if (copter.failsafe.gcs) {
        check_failed(display_failure, "GCS failsafe on");
        return false;
    }
    return true;
}

// check winch
bool AP_Arming_Copter::winch_checks(bool display_failure) const
{
#if WINCH_ENABLED == ENABLED
    // pass if parameter or all arming checks disabled
    if (((checks_to_perform & ARMING_CHECK_ALL) == 0) && ((checks_to_perform & ARMING_CHECK_PARAMETERS) == 0)) {
        return true;
    }

    const AP_Winch *winch = AP::winch();
    if (winch == nullptr) {
        return true;
    }
    char failure_msg[50] = {};
    if (!winch->pre_arm_check(failure_msg, sizeof(failure_msg))) {
        check_failed(display_failure, "%s", failure_msg);
        return false;
    }
#endif
    return true;
}

// performs altitude checks.  returns true if passed
bool AP_Arming_Copter::alt_checks(bool display_failure)
{
    // always EKF altitude estimate
    if (!copter.flightmode->has_manual_throttle() && !copter.ekf_alt_ok()) {
        check_failed(display_failure, "Need Alt Estimate");
        return false;
    }

    return true;
}

// arm_checks - perform final checks before arming
//  always called just before arming.  Return true if ok to arm
//  has side-effect that logging is started
bool AP_Arming_Copter::arm_checks(AP_Arming::Method method)
{
    const auto &ahrs = AP::ahrs();

    // always check if inertial nav has started and is ready
    if (!ahrs.healthy()) {
        check_failed(true, "AHRS not healthy");
        return false;
    }

#ifndef ALLOW_ARM_NO_COMPASS
    // if non-compass is source of heading we can skip compass health check
    if (!ahrs.using_noncompass_for_yaw()) {
        const Compass &_compass = AP::compass();
        // check compass health
        if (!_compass.healthy()) {
            check_failed(true, "Compass not healthy");
            return false;
        }
    }
#endif

    // always check if the current mode allows arming
    if (!copter.flightmode->allows_arming(method)) {
        check_failed(true, "Mode not armable");
        return false;
    }

    // always check motors
    if (!motor_checks(true)) {
        return false;
    }

    // succeed if arming checks are disabled
    if (checks_to_perform == 0) {
        return true;
    }

    // check lean angle
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_INS)) {
        if (degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()))*100.0f > copter.aparm.angle_max) {
            check_failed(ARMING_CHECK_INS, true, "Leaning");
            return false;
        }
    }

    // check adsb
#if HAL_ADSB_ENABLED
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_PARAMETERS)) {
        if (copter.failsafe.adsb) {
            check_failed(ARMING_CHECK_PARAMETERS, true, "ADSB threat detected");
            return false;
        }
    }
#endif

    // check throttle
    if ((checks_to_perform == ARMING_CHECK_ALL) || (checks_to_perform & ARMING_CHECK_RC)) {
         #if FRAME_CONFIG == HELI_FRAME
        const char *rc_item = "Collective";
        #else
        const char *rc_item = "Throttle";
        #endif
        // check throttle is not too low - must be above failsafe throttle
        if (copter.g.failsafe_throttle != FS_THR_DISABLED && copter.channel_throttle->get_radio_in() < copter.g.failsafe_throttle_value) {
            check_failed(ARMING_CHECK_RC, true, "%s below failsafe", rc_item);
            return false;
        }

        // check throttle is not too high - skips checks if arming from GCS in Guided
        if (!(method == AP_Arming::Method::MAVLINK && (copter.flightmode->mode_number() == Mode::Number::GUIDED || copter.flightmode->mode_number() == Mode::Number::GUIDED_NOGPS))) {
            // above top of deadband is too always high
            if (copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in()) > 0.0f) {
                check_failed(ARMING_CHECK_RC, true, "%s too high", rc_item);
                return false;
            }
            // in manual modes throttle must be at zero
            #if FRAME_CONFIG != HELI_FRAME
            if ((copter.flightmode->has_manual_throttle() || copter.flightmode->mode_number() == Mode::Number::DRIFT) && copter.channel_throttle->get_control_in() > 0) {
                check_failed(ARMING_CHECK_RC, true, "%s too high", rc_item);
                return false;
            }
            #endif
        }
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        check_failed(true, "Safety Switch");
        return false;
    }

    // superclass method should always be the last thing called; it
    // has side-effects which would need to be cleaned up if one of
    // our arm checks failed
    return AP_Arming::arm_checks(method);
}

// mandatory checks that will be run if ARMING_CHECK is zero or arming forced
bool AP_Arming_Copter::mandatory_checks(bool display_failure)
{
    // call mandatory gps checks and update notify status because regular gps checks will not run
    bool result = mandatory_gps_checks(display_failure);
    AP_Notify::flags.pre_arm_gps_check = result;

    // call mandatory alt check
    if (!alt_checks(display_failure)) {
        result = false;
    }

    return result & AP_Arming::mandatory_checks(display_failure);
}

void AP_Arming_Copter::set_pre_arm_check(bool b)
{
    copter.ap.pre_arm_check = b;
    AP_Notify::flags.pre_arm_check = b;
}

bool AP_Arming_Copter::arm(const AP_Arming::Method method, const bool do_arming_checks)
{
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // return true if already armed
    if (copter.motors->armed()) {
        in_arm_motors = false;
        return true;
    }

    if (!AP_Arming::arm(method, do_arming_checks)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

    // let logger know that we're armed (it may open logs e.g.)
    AP::logger().set_vehicle_armed(true);

    // disable cpu failsafe because initialising everything takes a while
    copter.failsafe_disable();

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call notify update a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        AP::notify().update();
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Arming motors");
#endif

    // Remember Orientation
    // --------------------
    copter.init_simple_bearing();

    auto &ahrs = AP::ahrs();

    copter.initial_armed_bearing = ahrs.yaw_sensor;

    if (!ahrs.home_is_set()) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)
        ahrs.resetHeightDatum();
        AP::logger().Write_Event(LogEvent::EKF_ALT_RESET);

        // we have reset height, so arming height is zero
        copter.arming_altitude_m = 0;
    } else if (!ahrs.home_is_locked()) {
        // Reset home position if it has already been set before (but not locked)
        if (!copter.set_home_to_current_location(false)) {
            // ignore failure
        }

        // remember the height when we armed
        copter.arming_altitude_m = copter.inertial_nav.get_position_z_up_cm() * 0.01;
    }
    copter.update_super_simple_bearing(false);

    // Reset SmartRTL return location. If activated, SmartRTL will ultimately try to land at this point
#if MODE_SMARTRTL_ENABLED == ENABLED
    copter.g2.smart_rtl.set_home(copter.position_ok());
#endif

    hal.util->set_soft_armed(true);

#if SPRAYER_ENABLED == ENABLED
    // turn off sprayer's test if on
    copter.sprayer.test_pump(false);
#endif

    // enable output to motors
    copter.enable_motor_output();

    // finally actually arm the motors
    copter.motors->armed(true);

    // log flight mode in case it was changed while vehicle was disarmed
    AP::logger().Write_Mode((uint8_t)copter.flightmode->mode_number(), copter.control_mode_reason);

    // re-enable failsafe
    copter.failsafe_enable();

    // perf monitor ignores delay due to arming
    AP::scheduler().perf_info.ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // Log time stamp of arming event
    copter.arm_time_ms = millis();

    // Start the arming delay
    copter.ap.in_arming_delay = true;

    // assumed armed without a arming, switch. Overridden in switches.cpp
    copter.ap.armed_with_airmode_switch = false;

    // return success
    return true;
}

// arming.disarm - disarm motors
bool AP_Arming_Copter::disarm(const AP_Arming::Method method, bool do_disarm_checks)
{
    // return immediately if we are already disarmed
    if (!copter.motors->armed()) {
        return true;
    }

    // do not allow disarm via mavlink if we think we are flying:
    if (do_disarm_checks &&
        method == AP_Arming::Method::MAVLINK &&
        !copter.ap.land_complete) {
        return false;
    }

    if (!AP_Arming::disarm(method, do_disarm_checks)) {
        return false;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Disarming motors");
#endif

    auto &ahrs = AP::ahrs();

    // save compass offsets learned by the EKF if enabled
    Compass &compass = AP::compass();
    if (ahrs.use_compass() && compass.get_learn_type() == Compass::LEARN_EKF) {
        for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            Vector3f magOffsets;
            if (ahrs.getMagOffsets(i, magOffsets)) {
                compass.set_and_save_offsets(i, magOffsets);
            }
        }
    }

#if AUTOTUNE_ENABLED == ENABLED
    // save auto tuned parameters
    if (copter.flightmode == &copter.mode_autotune) {
        copter.mode_autotune.save_tuning_gains();
    } else {
        copter.mode_autotune.reset();
    }
#endif

    // we are not in the air
    copter.set_land_complete(true);
    copter.set_land_complete_maybe(true);

    // send disarm command to motors
    copter.motors->armed(false);

#if MODE_AUTO_ENABLED == ENABLED
    // reset the mission
    copter.mode_auto.mission.reset();
#endif

    AP::logger().set_vehicle_armed(false);

    hal.util->set_soft_armed(false);

    copter.ap.in_arming_delay = false;

    return true;
}
Ap_rally.cpp
#include <AP_Common/Location.h>

#include "Copter.h"

#include "AP_Rally.h"

bool AP_Rally_Copter::is_valid(const Location &rally_point) const
{
#if AC_FENCE == ENABLED
    if (!copter.fence.check_destination_within_fence(rally_point)) {
        return false;
    }
#endif
    return true;
}
Ap_state.cpp
#include "Copter.h"

// ---------------------------------------------
void Copter::set_auto_armed(bool b)
{
    // if no change, exit immediately
    if( ap.auto_armed == b )
        return;

    ap.auto_armed = b;
    if(b){
        AP::logger().Write_Event(LogEvent::AUTO_ARMED);
    }
}

// ---------------------------------------------
/**
 * Set Simple mode
 *
 * @param [in] b 0:false or disabled, 1:true or SIMPLE, 2:SUPERSIMPLE
 */
void Copter::set_simple_mode(SimpleMode b)
{
    if (simple_mode != b) {
        switch (b) {
            case SimpleMode::NONE:
                AP::logger().Write_Event(LogEvent::SET_SIMPLE_OFF);
                gcs().send_text(MAV_SEVERITY_INFO, "SIMPLE mode off");
                break;
            case SimpleMode::SIMPLE:
                AP::logger().Write_Event(LogEvent::SET_SIMPLE_ON);
                gcs().send_text(MAV_SEVERITY_INFO, "SIMPLE mode on");
                break;
            case SimpleMode::SUPERSIMPLE:
                // initialise super simple heading
                update_super_simple_bearing(true);
                AP::logger().Write_Event(LogEvent::SET_SUPERSIMPLE_ON);
                gcs().send_text(MAV_SEVERITY_INFO, "SUPERSIMPLE mode on");
                break;
        }
        simple_mode = b;
    }
}

// ---------------------------------------------
void Copter::set_failsafe_radio(bool b)
{
    // only act on changes
    // -------------------
    if(failsafe.radio != b) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        failsafe.radio = b;

        if (failsafe.radio == false) {
            // We've regained radio contact
            // ----------------------------
            failsafe_radio_off_event();
        }else{
            // We've lost radio contact
            // ------------------------
            failsafe_radio_on_event();
        }

        // update AP_Notify
        AP_Notify::flags.failsafe_radio = b;
    }
}


// ---------------------------------------------
void Copter::set_failsafe_gcs(bool b)
{
    failsafe.gcs = b;

    // update AP_Notify
    AP_Notify::flags.failsafe_gcs = b;
}

// ---------------------------------------------

void Copter::update_using_interlock()
{
#if FRAME_CONFIG == HELI_FRAME
    // helicopters are always using motor interlock
    ap.using_interlock = true;
#else
    // check if we are using motor interlock control on an aux switch or are in throw mode
    // which uses the interlock to stop motors while the copter is being thrown
    ap.using_interlock = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK) != nullptr;
#endif
}
Attitude.cpp
#include "Copter.h"

/*************************************************************
 *  throttle control
 ****************************************************************/

// update estimated throttle required to hover (if necessary)
//  called at 100hz
void Copter::update_throttle_hover()
{
    // if not armed or landed or on standby then exit
    if (!motors->armed() || ap.land_complete || standby_active) {
        return;
    }

    // do not update in manual throttle modes or Drift
    if (flightmode->has_manual_throttle() || (copter.flightmode->mode_number() == Mode::Number::DRIFT)) {
        return;
    }

    // do not update while climbing or descending
    if (!is_zero(pos_control->get_vel_desired_cms().z)) {
        return;
    }

    // get throttle output
    float throttle = motors->get_throttle();

    // calc average throttle if we are in a level hover.  accounts for heli hover roll trim
    if (throttle > 0.0f && fabsf(inertial_nav.get_velocity_z_up_cms()) < 60 &&
        fabsf(ahrs.roll_sensor-attitude_control->get_roll_trim_cd()) < 500 && labs(ahrs.pitch_sensor) < 500) {
        // Can we set the time constant automatically
        motors->update_throttle_hover(0.01f);
#if HAL_GYROFFT_ENABLED
        gyro_fft.update_freq_hover(0.01f, motors->get_throttle_out());
#endif
    }
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to climb rate in cm/s
// without any deadzone at the bottom
float Copter::get_pilot_desired_climb_rate(float throttle_control)
{
    // throttle failsafe check
    if (failsafe.radio || !ap.rc_receiver_present) {
        return 0.0f;
    }

#if TOY_MODE_ENABLED == ENABLED
    if (g2.toy_mode.enabled()) {
        // allow throttle to be reduced after throttle arming and for
        // slower descent close to the ground
        g2.toy_mode.throttle_adjust(throttle_control);
    }
#endif

    // ensure a reasonable throttle value
    throttle_control = constrain_float(throttle_control,0.0f,1000.0f);

    // ensure a reasonable deadzone
    g.throttle_deadzone = constrain_int16(g.throttle_deadzone, 0, 400);

    float desired_rate = 0.0f;
    const float mid_stick = get_throttle_mid();
    const float deadband_top = mid_stick + g.throttle_deadzone;
    const float deadband_bottom = mid_stick - g.throttle_deadzone;

    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        desired_rate = get_pilot_speed_dn() * (throttle_control-deadband_bottom) / deadband_bottom;
    } else if (throttle_control > deadband_top) {
        // above the deadband
        desired_rate = g.pilot_speed_up * (throttle_control-deadband_top) / (1000.0f-deadband_top);
    } else {
        // must be in the deadband
        desired_rate = 0.0f;
    }

    return desired_rate;
}

// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
float Copter::get_non_takeoff_throttle()
{
    return MAX(0,motors->get_throttle_hover()/2.0f);
}

// set_accel_throttle_I_from_pilot_throttle - smoothes transition from pilot controlled throttle to autopilot throttle
void Copter::set_accel_throttle_I_from_pilot_throttle()
{
    // get last throttle input sent to attitude controller
    float pilot_throttle = constrain_float(attitude_control->get_throttle_in(), 0.0f, 1.0f);
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    pos_control->get_accel_z_pid().set_integrator((pilot_throttle-motors->get_throttle_hover()) * 1000.0f);
}

// rotate vector from vehicle's perspective to North-East frame
void Copter::rotate_body_frame_to_NE(float &x, float &y)
{
    float ne_x = x*ahrs.cos_yaw() - y*ahrs.sin_yaw();
    float ne_y = x*ahrs.sin_yaw() + y*ahrs.cos_yaw();
    x = ne_x;
    y = ne_y;
}

// It will return the PILOT_SPEED_DN value if non zero, otherwise if zero it returns the PILOT_SPEED_UP value.
uint16_t Copter::get_pilot_speed_dn() const
{
    if (g2.pilot_speed_dn == 0) {
        return abs(g.pilot_speed_up);
    } else {
        return abs(g2.pilot_speed_dn);
    }
}
Copter.cpp

#include "Copter.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, _interval_ticks, _max_time_micros, _prio) SCHED_TASK_CLASS(Copter, &copter, func, _interval_ticks, _max_time_micros, _prio)
#define FAST_TASK(func) FAST_TASK_CLASS(Copter, &copter, func)

const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    // update INS immediately to get current gyro data populated
    FAST_TASK_CLASS(AP_InertialSensor, &copter.ins, update),
    // run low level rate controllers that only require IMU data
    FAST_TASK(run_rate_controller),
    // send outputs to the motors library immediately
    FAST_TASK(motors_output),
     // run EKF state estimator (expensive)
    FAST_TASK(read_AHRS),
#if FRAME_CONFIG == HELI_FRAME
    FAST_TASK(update_heli_control_dynamics),
    #if MODE_AUTOROTATE_ENABLED == ENABLED
    FAST_TASK(heli_update_autorotation),
    #endif
#endif //HELI_FRAME
    // Inertial Nav
    FAST_TASK(read_inertia),
    // check if ekf has reset target heading or position
    FAST_TASK(check_ekf_reset),
    // run the attitude controllers
    FAST_TASK(update_flight_mode),
    // update home from EKF if necessary
    FAST_TASK(update_home_from_EKF),
    // check if we've landed or crashed
    FAST_TASK(update_land_and_crash_detectors),
#if HAL_MOUNT_ENABLED
    // camera mount's fast update
    FAST_TASK_CLASS(AP_Mount, &copter.camera_mount, update_fast),
#endif
    // log sensor health
    FAST_TASK(Log_Sensor_Health),
    FAST_TASK(Log_Video_Stabilisation),

    SCHED_TASK(rc_loop,              100,    130,  3),
    SCHED_TASK(throttle_loop,         50,     75,  6),
    SCHED_TASK_CLASS(AP_GPS,               &copter.gps,                 update,          50, 200,   9),
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(OpticalFlow,          &copter.optflow,             update,         200, 160,  12),
#endif
    SCHED_TASK(update_batt_compass,   10,    120, 15),
    SCHED_TASK_CLASS(RC_Channels, (RC_Channels*)&copter.g2.rc_channels, read_aux_all,    10,  50,  18),
    SCHED_TASK(arm_motors_check,      10,     50, 21),
#if TOY_MODE_ENABLED == ENABLED
    SCHED_TASK_CLASS(ToyMode,              &copter.g2.toy_mode,         update,          10,  50,  24),
#endif
    SCHED_TASK(auto_disarm_check,     10,     50,  27),
    SCHED_TASK(auto_trim,             10,     75,  30),
#if RANGEFINDER_ENABLED == ENABLED
    SCHED_TASK(read_rangefinder,      20,    100,  33),
#endif
#if HAL_PROXIMITY_ENABLED
    SCHED_TASK_CLASS(AP_Proximity,         &copter.g2.proximity,        update,         200,  50,  36),
#endif
#if BEACON_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Beacon,            &copter.g2.beacon,           update,         400,  50,  39),
#endif
    SCHED_TASK(update_altitude,       10,    100,  42),
    SCHED_TASK(run_nav_updates,       50,    100,  45),
    SCHED_TASK(update_throttle_hover,100,     90,  48),
#if MODE_SMARTRTL_ENABLED == ENABLED
    SCHED_TASK_CLASS(ModeSmartRTL,         &copter.mode_smartrtl,       save_position,    3, 100,  51),
#endif
#if SPRAYER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AC_Sprayer,           &copter.sprayer,               update,         3,  90,  54),
#endif
    SCHED_TASK(three_hz_loop,          3,     75, 57),
    SCHED_TASK_CLASS(AP_ServoRelayEvents,  &copter.ServoRelayEvents,      update_events, 50,  75,  60),
    SCHED_TASK_CLASS(AP_Baro,              &copter.barometer,             accumulate,    50,  90,  63),
#if AC_FENCE == ENABLED
    SCHED_TASK_CLASS(AC_Fence,             &copter.fence,                 update,        10, 100,  66),
#endif
#if PRECISION_LANDING == ENABLED
    SCHED_TASK(update_precland,      400,     50,  69),
#endif
#if FRAME_CONFIG == HELI_FRAME
    SCHED_TASK(check_dynamic_flight,  50,     75,  72),
#endif
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(fourhundred_hz_logging,400,    50,  75),
#endif
    SCHED_TASK_CLASS(AP_Notify,            &copter.notify,              update,          50,  90,  78),
    SCHED_TASK(one_hz_loop,            1,    100,  81),
    SCHED_TASK(ekf_check,             10,     75,  84),
    SCHED_TASK(check_vibration,       10,     50,  87),
    SCHED_TASK(gpsglitch_check,       10,     50,  90),
#if LANDING_GEAR_ENABLED == ENABLED
    SCHED_TASK(landinggear_update,    10,     75,  93),
#endif
    SCHED_TASK(standby_update,        100,    75,  96),
    SCHED_TASK(lost_vehicle_check,    10,     50,  99),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_receive, 400, 180, 102),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_send,    400, 550, 105),
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount,             &copter.camera_mount,        update,          50,  75, 108),
#endif
#if CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Camera,            &copter.camera,              update,          50,  75, 111),
#endif
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350, 114),
    SCHED_TASK(twentyfive_hz_logging, 25,    110, 117),
    SCHED_TASK_CLASS(AP_Logger,            &copter.logger,              periodic_tasks, 400, 300, 120),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,    &copter.ins,                 periodic,       400,  50, 123),

    SCHED_TASK_CLASS(AP_Scheduler,         &copter.scheduler,           update_logging, 0.1,  75, 126),
#if RPM_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_RPM,               &copter.rpm_sensor,          update,          40, 200, 129),
#endif
    SCHED_TASK_CLASS(Compass, &copter.compass, cal_update, 100, 100, 132),
    SCHED_TASK_CLASS(AP_TempCalibration,   &copter.g2.temp_calibration, update,          10, 100, 135),
#if HAL_ADSB_ENABLED
    SCHED_TASK(avoidance_adsb_update, 10,    100, 138),
#endif
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,          10,    100, 141),
#endif
#if AP_TERRAIN_AVAILABLE
    SCHED_TASK(terrain_update,        10,    100, 144),
#endif
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Gripper,           &copter.g2.gripper,          update,          10,  75, 147),
#endif
#if WINCH_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Winch,             &copter.g2.winch,            update,          50,  50, 150),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75, 153),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75, 156),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75, 159),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,      3.3,   75, 162),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,     75, 165),
#endif
#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button,            &copter.button,              update,           5, 100, 168),
#endif
#if STATS_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Stats,             &copter.g2.stats,            update,           1, 100, 171),
#endif
};

void Copter::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Copter::_failsafe_priorities[7];

#if AP_SCRIPTING_ENABLED
// start takeoff to given altitude (for use by scripting)
bool Copter::start_takeoff(float alt)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    if (mode_guided.do_user_takeoff_start(alt * 100.0f)) {
        copter.set_auto_armed(true);
        return true;
    }
    return false;
}

// set target location (for use by scripting)
bool Copter::set_target_location(const Location& target_loc)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    return mode_guided.set_destination(target_loc);
}

// set target position (for use by scripting)
bool Copter::set_target_pos_NED(const Vector3f& target_pos, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative, bool terrain_alt)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);

    return mode_guided.set_destination(pos_neu_cm, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, yaw_relative, terrain_alt);
}

// set target position and velocity (for use by scripting)
bool Copter::set_target_posvel_NED(const Vector3f& target_pos, const Vector3f& target_vel)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);

    return mode_guided.set_destination_posvelaccel(pos_neu_cm, vel_neu_cms, Vector3f());
}

// set target position, velocity and acceleration (for use by scripting)
bool Copter::set_target_posvelaccel_NED(const Vector3f& target_pos, const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);
    const Vector3f accel_neu_cms(target_accel.x * 100.0f, target_accel.y * 100.0f, -target_accel.z * 100.0f);

    return mode_guided.set_destination_posvelaccel(pos_neu_cm, vel_neu_cms, accel_neu_cms, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, yaw_relative);
}

bool Copter::set_target_velocity_NED(const Vector3f& vel_ned)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // convert vector to neu in cm
    const Vector3f vel_neu_cms(vel_ned.x * 100.0f, vel_ned.y * 100.0f, -vel_ned.z * 100.0f);
    mode_guided.set_velocity(vel_neu_cms);
    return true;
}

// set target velocity and acceleration (for use by scripting)
bool Copter::set_target_velaccel_NED(const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool relative_yaw)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // convert vector to neu in cm
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);
    const Vector3f accel_neu_cms(target_accel.x * 100.0f, target_accel.y * 100.0f, -target_accel.z * 100.0f);

    mode_guided.set_velaccel(vel_neu_cms, accel_neu_cms, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, relative_yaw);
    return true;
}

bool Copter::set_target_angle_and_climbrate(float roll_deg, float pitch_deg, float yaw_deg, float climb_rate_ms, bool use_yaw_rate, float yaw_rate_degs)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    Quaternion q;
    q.from_euler(radians(roll_deg),radians(pitch_deg),radians(yaw_deg));

    mode_guided.set_angle(q, Vector3f{}, climb_rate_ms*100, false);
    return true;
}

// circle mode controls
bool Copter::get_circle_radius(float &radius_m)
{
    radius_m = circle_nav->get_radius() * 0.01f;
    return true;
}

bool Copter::set_circle_rate(float rate_dps)
{
    circle_nav->set_rate(rate_dps);
    return true;
}

// returns true if mode supports NAV_SCRIPT_TIME mission commands
bool Copter::nav_scripting_enable(uint8_t mode)
{
    return mode == (uint8_t)mode_auto.mode_number();
}

// lua scripts use this to retrieve the contents of the active command
bool Copter::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2)
{
    if (flightmode != &mode_auto) {
        return false;
    }

    return mode_auto.nav_script_time(id, cmd, arg1, arg2);
}

// lua scripts use this to indicate when they have complete the command
void Copter::nav_script_time_done(uint16_t id)
{
    if (flightmode != &mode_auto) {
        return;
    }

    return mode_auto.nav_script_time_done(id);
}

// returns true if the EKF failsafe has triggered.  Only used by Lua scripts
bool Copter::has_ekf_failsafed() const
{
    return failsafe.ekf;
}

#endif // AP_SCRIPTING_ENABLED


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
    update_ekf_terrain_height_stable();
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Copter::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if(AP::compass().available()) {
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
        motors->Log_Write();
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
    if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS() || !flightmode->has_manual_throttle())) {
        pos_control->write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control->control_monitor_log();
#if HAL_PROXIMITY_ENABLED
        g2.proximity.log();  // Write proximity sensor distances
#endif
#if BEACON_ENABLED == ENABLED
        g2.beacon.log();
#endif
    }
#if FRAME_CONFIG == HELI_FRAME
    Log_Write_Heli();
#endif
#if WINCH_ENABLED == ENABLED
    if (should_log(MASK_LOG_ANY)) {
        g2.winch.write_log();
    }
#endif
}

// twentyfive_hz_logging - should be run at 25hz
void Copter::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }

    if (should_log(MASK_LOG_IMU)) {
        AP::ins().Write_IMU();
    }

#if MODE_AUTOROTATE_ENABLED == ENABLED
    if (should_log(MASK_LOG_ATTITUDE_MED) || should_log(MASK_LOG_ATTITUDE_FAST)) {
        //update autorotation log
        g2.arot.Log_Write_Autorotation();
    }
#endif
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

    // check if avoidance should be enabled based on alt
    low_alt_avoidance();
}

// one_hz_loop - runs at 1Hz
void Copter::one_hz_loop()
{
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap.value);
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
        motors->update_throttle_range();
#endif
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    // log terrain data
    terrain_logging();

#if HAL_ADSB_ENABLED
    adsb.set_is_flying(!ap.land_complete);
#endif

    AP_Notify::flags.flying = !ap.land_complete;
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
        Log_Write_Data(LogDataID::INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (simple_mode == SimpleMode::NONE || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    if (simple_mode == SimpleMode::SIMPLE) {
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
        if (simple_mode != SimpleMode::SUPERSIMPLE) {
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
        AP::ins().write_notch_log_messages();
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#endif
    }
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    distance = flightmode->wp_distance() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    bearing = flightmode->wp_bearing() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    xtrack_error = flightmode->crosstrack_error() * 0.01;
    return true;
}

/*
  constructor for main Copter class
 */
Copter::Copter(void)
    : logger(g.log_bitmask),
    flight_modes(&g.flight_mode1),
    simple_cos_yaw(1.0f),
    super_simple_cos_yaw(1.0),
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    rc_throttle_control_in_filter(1.0f),
    inertial_nav(ahrs),
    param_loader(var_info),
    flightmode(&mode_stabilize)
{
    // init sensor error logging flags
    sensor_health.compass = true;
}

Copter copter;
AP_Vehicle& vehicle = copter;

AP_HAL_MAIN_CALLBACKS(&copter);
GCS_Copter.cpp
#include "GCS_Copter.h"

#include "Copter.h"

uint8_t GCS_Copter::sysid_this_mav() const
{
    return copter.g.sysid_this_mav;
}

const char* GCS_Copter::frame_string() const
{
    if (copter.motors == nullptr) {
        return "motors not allocated";
    }
    return copter.motors->get_frame_string();
}

bool GCS_Copter::simple_input_active() const
{
    return copter.simple_mode == Copter::SimpleMode::SIMPLE;
}

bool GCS_Copter::supersimple_input_active() const
{
    return copter.simple_mode == Copter::SimpleMode::SUPERSIMPLE;
}

void GCS_Copter::update_vehicle_sensor_status_flags(void)
{
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_enabled |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_health |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    const Copter::ap_t &ap = copter.ap;

    if (ap.rc_receiver_present) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }
    if (ap.rc_receiver_present && !copter.failsafe.radio) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    }

    // update flightmode-specific flags:
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;

    switch (copter.flightmode->mode_number()) {
    case Mode::Number::AUTO:
    case Mode::Number::AUTO_RTL:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::GUIDED:
    case Mode::Number::LOITER:
    case Mode::Number::RTL:
    case Mode::Number::CIRCLE:
    case Mode::Number::LAND:
    case Mode::Number::POSHOLD:
    case Mode::Number::BRAKE:
    case Mode::Number::THROW:
    case Mode::Number::SMART_RTL:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
        break;
    case Mode::Number::ALT_HOLD:
    case Mode::Number::GUIDED_NOGPS:
    case Mode::Number::SPORT:
    case Mode::Number::AUTOTUNE:
    case Mode::Number::FLOWHOLD:
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
        break;
    default:
        // stabilize, acro, drift, and flip have no automatic x,y or z control (i.e. all manual)
        break;
    }

    // optional sensors, some of which are essentially always
    // available in the firmware:
#if HAL_PROXIMITY_ENABLED
    if (copter.g2.proximity.sensor_present()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
    if (copter.g2.proximity.sensor_enabled()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
    if (!copter.g2.proximity.sensor_failed()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_PROXIMITY;
    }
#endif

#if RANGEFINDER_ENABLED == ENABLED
    const RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder && rangefinder->has_orientation(ROTATION_PITCH_270)) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    }
    if (copter.rangefinder_state.enabled) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        if (rangefinder && rangefinder->has_data_orient(ROTATION_PITCH_270)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
        }
    }
#endif

#if AP_OPTICALFLOW_ENABLED
    const OpticalFlow *optflow = AP::opticalflow();
    if (optflow && optflow->enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
    if (optflow && optflow->healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    }
#endif

#if PRECISION_LANDING == ENABLED
    if (copter.precland.enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
    if (copter.precland.enabled() && copter.precland.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    }
#endif

#if AP_TERRAIN_AVAILABLE
    switch (copter.terrain.status()) {
    case AP_Terrain::TerrainStatusDisabled:
        break;
    case AP_Terrain::TerrainStatusUnhealthy:
        // To-Do: restore unhealthy terrain status reporting once terrain is used in copter
        //control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        //control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        //break;
    case AP_Terrain::TerrainStatusOK:
        control_sensors_present |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_enabled |= MAV_SYS_STATUS_TERRAIN;
        control_sensors_health  |= MAV_SYS_STATUS_TERRAIN;
        break;
    }
#endif

    control_sensors_present |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    // only mark propulsion healthy if all of the motors are producing
    // nominal thrust
    if (!copter.motors->get_thrust_boost()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_PROPULSION;
    }
}
GCS_Mavlink.cpp
#include "Copter.h"

#include "GCS_Mavlink.h"

MAV_TYPE GCS_Copter::frame_type() const
{
    if (copter.motors == nullptr) {
        return MAV_TYPE_GENERIC;
    }
    return copter.motors->get_frame_mav_type();
}

MAV_MODE GCS_MAVLINK_Copter::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (copter.flightmode->mode_number()) {
    case Mode::Number::AUTO:
    case Mode::Number::AUTO_RTL:
    case Mode::Number::RTL:
    case Mode::Number::LOITER:
    case Mode::Number::AVOID_ADSB:
    case Mode::Number::FOLLOW:
    case Mode::Number::GUIDED:
    case Mode::Number::CIRCLE:
    case Mode::Number::POSHOLD:
    case Mode::Number::BRAKE:
    case Mode::Number::SMART_RTL:
        _base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    default:
        break;
    }

    // all modes except INITIALISING have some form of manual
    // override if stick mixing is enabled
    _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    // we are armed if we are not initialising
    if (copter.motors != nullptr && copter.motors->armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    // indicate we have set a custom mode
    _base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return (MAV_MODE)_base_mode;
}

uint32_t GCS_Copter::custom_mode() const
{
    return (uint32_t)copter.flightmode->mode_number();
}

MAV_STATE GCS_MAVLINK_Copter::vehicle_system_status() const
{
    // set system as critical if any failsafe have triggered
    if (copter.any_failsafe_triggered())  {
        return MAV_STATE_CRITICAL;
    }

    if (copter.ap.land_complete) {
        return MAV_STATE_STANDBY;
    }

    return MAV_STATE_ACTIVE;
}


void GCS_MAVLINK_Copter::send_attitude_target()
{
    const Quaternion quat  = copter.attitude_control->get_attitude_target_quat();
    const Vector3f ang_vel = copter.attitude_control->get_attitude_target_ang_vel();
    const float thrust = copter.attitude_control->get_throttle_in();

    const float quat_out[4] {quat.q1, quat.q2, quat.q3, quat.q4};

    // Note: When sending out the attitude_target info. we send out all of info. no matter the mavlink typemask
    // This way we send out the maximum information that can be used by the sending control systems to adapt their generated trajectories
    const uint16_t typemask = 0;    // Ignore nothing

    mavlink_msg_attitude_target_send(
        chan,
        AP_HAL::millis(),       // time since boot (ms)
        typemask,               // Bitmask that tells the system what control dimensions should be ignored by the vehicle
        quat_out,               // Attitude quaternion [w, x, y, z] order, zero-rotation is [1, 0, 0, 0], unit-length
        ang_vel.x,              // roll rate (rad/s)
        ang_vel.y,              // pitch rate (rad/s)
        ang_vel.z,              // yaw rate (rad/s)
        thrust);                // Collective thrust, normalized to 0 .. 1
}

void GCS_MAVLINK_Copter::send_position_target_global_int()
{
    Location target;
    if (!copter.flightmode->get_wp(target)) {
        return;
    }

    // convert altitude frame to AMSL (this may use the terrain database)
    if (!target.change_alt_frame(Location::AltFrame::ABSOLUTE)) {
        return;
    }
    static constexpr uint16_t POSITION_TARGET_TYPEMASK_LAST_BYTE = 0xF000;
    static constexpr uint16_t TYPE_MASK = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                                          POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE | POSITION_TARGET_TYPEMASK_LAST_BYTE;
    mavlink_msg_position_target_global_int_send(
        chan,
        AP_HAL::millis(), // time_boot_ms
        MAV_FRAME_GLOBAL, // targets are always global altitude
        TYPE_MASK, // ignore everything except the x/y/z components
        target.lat, // latitude as 1e7
        target.lng, // longitude as 1e7
        target.alt * 0.01f, // altitude is sent as a float
        0.0f, // vx
        0.0f, // vy
        0.0f, // vz
        0.0f, // afx
        0.0f, // afy
        0.0f, // afz
        0.0f, // yaw
        0.0f); // yaw_rate
}

void GCS_MAVLINK_Copter::send_position_target_local_ned()
{
#if MODE_GUIDED_ENABLED == ENABLED
    if (!copter.flightmode->in_guided_mode()) {
        return;
    }

    const ModeGuided::SubMode guided_mode = copter.mode_guided.submode();
    Vector3f target_pos;
    Vector3f target_vel;
    Vector3f target_accel;
    uint16_t type_mask = 0;

    switch (guided_mode) {
    case ModeGuided::SubMode::Angle:
        // we don't have a local target when in angle mode
        return;
    case ModeGuided::SubMode::TakeOff:
    case ModeGuided::SubMode::WP:
    case ModeGuided::SubMode::Pos:
        type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except position
        target_pos = copter.mode_guided.get_target_pos().tofloat() * 0.01; // convert to metres
        break;
    case ModeGuided::SubMode::PosVelAccel:
        type_mask = POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except position, velocity & acceleration
        target_pos = copter.mode_guided.get_target_pos().tofloat() * 0.01; // convert to metres
        target_vel = copter.mode_guided.get_target_vel() * 0.01f; // convert to metres/s
        target_accel = copter.mode_guided.get_target_accel() * 0.01f; // convert to metres/s/s
        break;
    case ModeGuided::SubMode::VelAccel:
        type_mask = POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except velocity & acceleration
        target_vel = copter.mode_guided.get_target_vel() * 0.01f; // convert to metres/s
        target_accel = copter.mode_guided.get_target_accel() * 0.01f; // convert to metres/s/s
        break;
    case ModeGuided::SubMode::Accel:
        type_mask = POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
                    POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE| POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // ignore everything except velocity & acceleration
        target_accel = copter.mode_guided.get_target_accel() * 0.01f; // convert to metres/s/s
        break;
    }

    mavlink_msg_position_target_local_ned_send(
        chan,
        AP_HAL::millis(), // time boot ms
        MAV_FRAME_LOCAL_NED, 
        type_mask,
        target_pos.x,   // x in metres
        target_pos.y,   // y in metres
        -target_pos.z,  // z in metres NED frame
        target_vel.x,   // vx in m/s
        target_vel.y,   // vy in m/s
        -target_vel.z,  // vz in m/s NED frame
        target_accel.x, // afx in m/s/s
        target_accel.y, // afy in m/s/s
        -target_accel.z,// afz in m/s/s NED frame
        0.0f, // yaw
        0.0f); // yaw_rate
#endif
}

void GCS_MAVLINK_Copter::send_nav_controller_output() const
{
    if (!copter.ap.initialised) {
        return;
    }
    const Vector3f &targets = copter.attitude_control->get_att_target_euler_cd();
    const Mode *flightmode = copter.flightmode;
    mavlink_msg_nav_controller_output_send(
        chan,
        targets.x * 1.0e-2f,
        targets.y * 1.0e-2f,
        targets.z * 1.0e-2f,
        flightmode->wp_bearing() * 1.0e-2f,
        MIN(flightmode->wp_distance() * 1.0e-2f, UINT16_MAX),
        copter.pos_control->get_pos_error_z_cm() * 1.0e-2f,
        0,
        flightmode->crosstrack_error() * 1.0e-2f);
}

float GCS_MAVLINK_Copter::vfr_hud_airspeed() const
{
#if AP_AIRSPEED_ENABLED
    // airspeed sensors are best. While the AHRS airspeed_estimate
    // will use an airspeed sensor, that value is constrained by the
    // ground speed. When reporting we should send the true airspeed
    // value if possible:
    if (copter.airspeed.enabled() && copter.airspeed.healthy()) {
        return copter.airspeed.get_airspeed();
    }
#endif
    
    Vector3f airspeed_vec_bf;
    if (AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // we are running the EKF3 wind estimation code which can give
        // us an airspeed estimate
        return airspeed_vec_bf.length();
    }
    return AP::gps().ground_speed();
}

int16_t GCS_MAVLINK_Copter::vfr_hud_throttle() const
{
    if (copter.motors == nullptr) {
        return 0;
    }
    return (int16_t)(copter.motors->get_throttle() * 100);
}

/*
  send PID tuning message
 */
void GCS_MAVLINK_Copter::send_pid_tuning()
{
    static const PID_TUNING_AXIS axes[] = {
        PID_TUNING_ROLL,
        PID_TUNING_PITCH,
        PID_TUNING_YAW,
        PID_TUNING_ACCZ
    };
    for (uint8_t i=0; i<ARRAY_SIZE(axes); i++) {
        if (!(copter.g.gcs_pid_mask & (1<<(axes[i]-1)))) {
            continue;
        }
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
        const AP_PIDInfo *pid_info = nullptr;
        switch (axes[i]) {
        case PID_TUNING_ROLL:
            pid_info = &copter.attitude_control->get_rate_roll_pid().get_pid_info();
            break;
        case PID_TUNING_PITCH:
            pid_info = &copter.attitude_control->get_rate_pitch_pid().get_pid_info();
            break;
        case PID_TUNING_YAW:
            pid_info = &copter.attitude_control->get_rate_yaw_pid().get_pid_info();
            break;
        case PID_TUNING_ACCZ:
            pid_info = &copter.pos_control->get_accel_z_pid().get_pid_info();
            break;
        default:
            continue;
        }
        if (pid_info != nullptr) {
            mavlink_msg_pid_tuning_send(chan,
                                        axes[i],
                                        pid_info->target,
                                        pid_info->actual,
                                        pid_info->FF,
                                        pid_info->P,
                                        pid_info->I,
                                        pid_info->D,
                                        pid_info->slew_rate,
                                        pid_info->Dmod);
        }
    }
}

// send winch status message
void GCS_MAVLINK_Copter::send_winch_status() const
{
#if WINCH_ENABLED == ENABLED
    AP_Winch *winch = AP::winch();
    if (winch == nullptr) {
        return;
    }
    winch->send_status(*this);
#endif
}

uint8_t GCS_MAVLINK_Copter::sysid_my_gcs() const
{
    return copter.g.sysid_my_gcs;
}
bool GCS_MAVLINK_Copter::sysid_enforce() const
{
    return copter.g2.sysid_enforce;
}

uint32_t GCS_MAVLINK_Copter::telem_delay() const
{
    return (uint32_t)(copter.g.telem_delay);
}

bool GCS_Copter::vehicle_initialised() const {
    return copter.ap.initialised;
}

// try to send a message, return false if it wasn't sent
bool GCS_MAVLINK_Copter::try_send_message(enum ap_message id)
{
    switch(id) {

    case MSG_TERRAIN:
#if AP_TERRAIN_AVAILABLE
        CHECK_PAYLOAD_SIZE(TERRAIN_REQUEST);
        copter.terrain.send_request(chan);
#endif
        break;

    case MSG_WIND:
        CHECK_PAYLOAD_SIZE(WIND);
        send_wind();
        break;

    case MSG_SERVO_OUT:
    case MSG_AOA_SSA:
    case MSG_LANDING:
        // unused
        break;

    case MSG_ADSB_VEHICLE: {
#if HAL_ADSB_ENABLED
        CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
        copter.adsb.send_adsb_vehicle(chan);
#endif
#if AC_OAPATHPLANNER_ENABLED == ENABLED
        AP_OADatabase *oadb = AP_OADatabase::get_singleton();
        if (oadb != nullptr) {
            CHECK_PAYLOAD_SIZE(ADSB_VEHICLE);
            uint16_t interval_ms = 0;
            if (get_ap_message_interval(id, interval_ms)) {
                oadb->send_adsb_vehicle(chan, interval_ms);
            }
        }
#endif
        break;
    }

    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}


const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    // @Param: RAW_SENS
    // @DisplayName: Raw sensor stream rate
    // @Description: Stream rate of RAW_IMU, SCALED_IMU2, SCALED_IMU3, SCALED_PRESSURE, SCALED_PRESSURE2, SCALED_PRESSURE3 and SENSOR_OFFSETS to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("RAW_SENS", 0, GCS_MAVLINK_Parameters, streamRates[0],  0),

    // @Param: EXT_STAT
    // @DisplayName: Extended status stream rate to ground station
    // @Description: Stream rate of SYS_STATUS, POWER_STATUS, MCU_STATUS, MEMINFO, CURRENT_WAYPOINT, GPS_RAW_INT, GPS_RTK (if available), GPS2_RAW (if available), GPS2_RTK (if available), NAV_CONTROLLER_OUTPUT, and FENCE_STATUS to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXT_STAT", 1, GCS_MAVLINK_Parameters, streamRates[1],  0),

    // @Param: RC_CHAN
    // @DisplayName: RC Channel stream rate to ground station
    // @Description: Stream rate of SERVO_OUTPUT_RAW and RC_CHANNELS to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("RC_CHAN",  2, GCS_MAVLINK_Parameters, streamRates[2],  0),

    // @Param: RAW_CTRL
    // @DisplayName: Unused
    // @Description: Unused
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("RAW_CTRL", 3, GCS_MAVLINK_Parameters, streamRates[3],  0),

    // @Param: POSITION
    // @DisplayName: Position stream rate to ground station
    // @Description: Stream rate of GLOBAL_POSITION_INT and LOCAL_POSITION_NED to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("POSITION", 4, GCS_MAVLINK_Parameters, streamRates[4],  0),

    // @Param: EXTRA1
    // @DisplayName: Extra data type 1 stream rate to ground station
    // @Description: Stream rate of ATTITUDE, SIMSTATE (SIM only), AHRS2 and PID_TUNING to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXTRA1",   5, GCS_MAVLINK_Parameters, streamRates[5],  0),

    // @Param: EXTRA2
    // @DisplayName: Extra data type 2 stream rate to ground station
    // @Description: Stream rate of VFR_HUD to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXTRA2",   6, GCS_MAVLINK_Parameters, streamRates[6],  0),

    // @Param: EXTRA3
    // @DisplayName: Extra data type 3 stream rate to ground station
    // @Description: Stream rate of AHRS, HWSTATUS, SYSTEM_TIME, RANGEFINDER, DISTANCE_SENSOR, TERRAIN_REQUEST, BATTERY2, MOUNT_STATUS, OPTICAL_FLOW, GIMBAL_REPORT, MAG_CAL_REPORT, MAG_CAL_PROGRESS, EKF_STATUS_REPORT, VIBRATION and RPM to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EXTRA3",   7, GCS_MAVLINK_Parameters, streamRates[7],  0),

    // @Param: PARAMS
    // @DisplayName: Parameter stream rate to ground station
    // @Description: Stream rate of PARAM_VALUE to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("PARAMS",   8, GCS_MAVLINK_Parameters, streamRates[8],  0),

    // @Param: ADSB
    // @DisplayName: ADSB stream rate to ground station
    // @Description: ADSB stream rate to ground station
    // @Units: Hz
    // @Range: 0 50
    // @Increment: 1
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("ADSB",   9, GCS_MAVLINK_Parameters, streamRates[9],  0),
AP_GROUPEND
};

static const ap_message STREAM_RAW_SENSORS_msgs[] = {
    MSG_RAW_IMU,
    MSG_SCALED_IMU2,
    MSG_SCALED_IMU3,
    MSG_SCALED_PRESSURE,
    MSG_SCALED_PRESSURE2,
    MSG_SCALED_PRESSURE3,
};
static const ap_message STREAM_EXTENDED_STATUS_msgs[] = {
    MSG_SYS_STATUS,
    MSG_POWER_STATUS,
    MSG_MCU_STATUS,
    MSG_MEMINFO,
    MSG_CURRENT_WAYPOINT, // MISSION_CURRENT
    MSG_GPS_RAW,
    MSG_GPS_RTK,
    MSG_GPS2_RAW,
    MSG_GPS2_RTK,
    MSG_NAV_CONTROLLER_OUTPUT,
    MSG_FENCE_STATUS,
    MSG_POSITION_TARGET_GLOBAL_INT,
};
static const ap_message STREAM_POSITION_msgs[] = {
    MSG_LOCATION,
    MSG_LOCAL_POSITION
};
static const ap_message STREAM_RC_CHANNELS_msgs[] = {
    MSG_SERVO_OUTPUT_RAW,
    MSG_RC_CHANNELS,
    MSG_RC_CHANNELS_RAW, // only sent on a mavlink1 connection
};
static const ap_message STREAM_EXTRA1_msgs[] = {
    MSG_ATTITUDE,
    MSG_SIMSTATE,
    MSG_AHRS2,
    MSG_PID_TUNING // Up to four PID_TUNING messages are sent, depending on GCS_PID_MASK parameter
};
static const ap_message STREAM_EXTRA2_msgs[] = {
    MSG_VFR_HUD
};
static const ap_message STREAM_EXTRA3_msgs[] = {
    MSG_AHRS,
    MSG_HWSTATUS,
    MSG_SYSTEM_TIME,
    MSG_WIND,
    MSG_RANGEFINDER,
    MSG_DISTANCE_SENSOR,
#if AP_TERRAIN_AVAILABLE
    MSG_TERRAIN,
#endif
    MSG_BATTERY2,
    MSG_BATTERY_STATUS,
    MSG_MOUNT_STATUS,
    MSG_OPTICAL_FLOW,
    MSG_GIMBAL_REPORT,
    MSG_MAG_CAL_REPORT,
    MSG_MAG_CAL_PROGRESS,
    MSG_EKF_STATUS_REPORT,
    MSG_VIBRATION,
    MSG_RPM,
    MSG_ESC_TELEMETRY,
    MSG_GENERATOR_STATUS,
    MSG_WINCH_STATUS,
};
static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM
};
static const ap_message STREAM_ADSB_msgs[] = {
    MSG_ADSB_VEHICLE
};

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_RAW_SENSORS),
    MAV_STREAM_ENTRY(STREAM_EXTENDED_STATUS),
    MAV_STREAM_ENTRY(STREAM_POSITION),
    MAV_STREAM_ENTRY(STREAM_RC_CHANNELS),
    MAV_STREAM_ENTRY(STREAM_EXTRA1),
    MAV_STREAM_ENTRY(STREAM_EXTRA2),
    MAV_STREAM_ENTRY(STREAM_EXTRA3),
    MAV_STREAM_ENTRY(STREAM_ADSB),
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_TERMINATOR // must have this at end of stream_entries
};

bool GCS_MAVLINK_Copter::handle_guided_request(AP_Mission::Mission_Command &cmd)
{
#if MODE_AUTO_ENABLED == ENABLED
    return copter.mode_auto.do_guided(cmd);
#else
    return false;
#endif
}

void GCS_MAVLINK_Copter::packetReceived(const mavlink_status_t &status,
                                        const mavlink_message_t &msg)
{
#if HAL_ADSB_ENABLED
    if (copter.g2.dev_options.get() & DevOptionADSBMAVLink) {
        // optional handling of GLOBAL_POSITION_INT as a MAVLink based avoidance source
        copter.avoidance_adsb.handle_msg(msg);
    }
#endif
#if MODE_FOLLOW_ENABLED == ENABLED
    // pass message to follow library
    copter.g2.follow.handle_msg(msg);
#endif
    GCS_MAVLINK::packetReceived(status, msg);
}

bool GCS_MAVLINK_Copter::params_ready() const
{
    if (AP_BoardConfig::in_config_error()) {
        // we may never have parameters "initialised" in this case
        return true;
    }
    // if we have not yet initialised (including allocating the motors
    // object) we drop this request. That prevents the GCS from getting
    // a confusing parameter count during bootup
    return copter.ap.initialised_params;
}

void GCS_MAVLINK_Copter::send_banner()
{
    GCS_MAVLINK::send_banner();
    if (copter.motors == nullptr) {
        send_text(MAV_SEVERITY_INFO, "motors not allocated");
        return;
    }
    char frame_and_type_string[30];
    copter.motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    send_text(MAV_SEVERITY_INFO, "%s", frame_and_type_string);
}

void GCS_MAVLINK_Copter::handle_command_ack(const mavlink_message_t &msg)
{
    copter.command_ack_counter++;
    GCS_MAVLINK::handle_command_ack(msg);
}

/*
  handle a LANDING_TARGET command. The timestamp has been jitter corrected
*/
void GCS_MAVLINK_Copter::handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
#if PRECISION_LANDING == ENABLED
    copter.precland.handle_msg(packet, timestamp_ms);
#endif
}

MAV_RESULT GCS_MAVLINK_Copter::_handle_command_preflight_calibration(const mavlink_command_long_t &packet)
{
    if (is_equal(packet.param6,1.0f)) {
        // compassmot calibration
        return copter.mavlink_compassmot(*this);
    }

    return GCS_MAVLINK::_handle_command_preflight_calibration(packet);
}


MAV_RESULT GCS_MAVLINK_Copter::handle_command_do_set_roi(const Location &roi_loc)
{
    if (!roi_loc.check_latlng()) {
        return MAV_RESULT_FAILED;
    }
    copter.flightmode->auto_yaw.set_roi(roi_loc);
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Copter::handle_preflight_reboot(const mavlink_command_long_t &packet)
{
    // reject reboot if user has also specified they want the "Auto" ESC calibration on next reboot
    if (copter.g.esc_calibrate == (uint8_t)Copter::ESCCalibrationModes::ESCCAL_AUTO) {
        send_text(MAV_SEVERITY_CRITICAL, "Reboot rejected, ESC cal on reboot");
        return MAV_RESULT_FAILED;
    }

    // call parent
    return GCS_MAVLINK::handle_preflight_reboot(packet);
}

bool GCS_MAVLINK_Copter::set_home_to_current_location(bool _lock) {
    return copter.set_home_to_current_location(_lock);
}
bool GCS_MAVLINK_Copter::set_home(const Location& loc, bool _lock) {
    return copter.set_home(loc, _lock);
}

MAV_RESULT GCS_MAVLINK_Copter::handle_command_int_do_reposition(const mavlink_command_int_t &packet)
{
    const bool change_modes = ((int32_t)packet.param2 & MAV_DO_REPOSITION_FLAGS_CHANGE_MODE) == MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;
    if (!copter.flightmode->in_guided_mode() && !change_modes) {
        return MAV_RESULT_DENIED;
    }

    // sanity check location
    if (!check_latlng(packet.x, packet.y)) {
        return MAV_RESULT_DENIED;
    }

    Location request_location;
    if (!location_from_command_t(packet, request_location)) {
        return MAV_RESULT_DENIED;
    }

    if (request_location.sanitize(copter.current_loc)) {
        // if the location wasn't already sane don't load it
        return MAV_RESULT_DENIED; // failed as the location is not valid
    }

    // we need to do this first, as we don't want to change the flight mode unless we can also set the target
    if (!copter.mode_guided.set_destination(request_location, false, 0, false, 0)) {
        return MAV_RESULT_FAILED;
    }

    if (!copter.flightmode->in_guided_mode()) {
        if (!copter.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        // the position won't have been loaded if we had to change the flight mode, so load it again
        if (!copter.mode_guided.set_destination(request_location, false, 0, false, 0)) {
            return MAV_RESULT_FAILED;
        }
    }

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK_Copter::handle_command_int_packet(const mavlink_command_int_t &packet)
{
    switch(packet.command) {
    case MAV_CMD_DO_FOLLOW:
#if MODE_FOLLOW_ENABLED == ENABLED
        // param1: sysid of target to follow
        if ((packet.param1 > 0) && (packet.param1 <= 255)) {
            copter.g2.follow.set_target_sysid((uint8_t)packet.param1);
            return MAV_RESULT_ACCEPTED;
        }
#endif
        return MAV_RESULT_UNSUPPORTED;

    case MAV_CMD_DO_REPOSITION:
        return handle_command_int_do_reposition(packet);

    // pause or resume an auto mission
    case MAV_CMD_DO_PAUSE_CONTINUE:
        return handle_command_pause_continue(packet);

    default:
        return GCS_MAVLINK::handle_command_int_packet(packet);
    }
}

MAV_RESULT GCS_MAVLINK_Copter::handle_command_mount(const mavlink_command_long_t &packet)
{
    switch (packet.command) {
#if HAL_MOUNT_ENABLED
    case MAV_CMD_DO_MOUNT_CONTROL:
        // if vehicle has a camera mount but it doesn't do pan control then yaw the entire vehicle instead
        if ((copter.camera_mount.get_mount_type() != copter.camera_mount.MountType::Mount_Type_None) &&
            !copter.camera_mount.has_pan_control()) {
            copter.flightmode->auto_yaw.set_yaw_angle_rate(
                (float)packet.param3 * 0.01f,
                0.0f);
        }
        break;
#endif
    default:
        break;
    }
    return GCS_MAVLINK::handle_command_mount(packet);
}

MAV_RESULT GCS_MAVLINK_Copter::handle_command_long_packet(const mavlink_command_long_t &packet)
{
    switch(packet.command) {

    case MAV_CMD_NAV_TAKEOFF: {
        // param3 : horizontal navigation by pilot acceptable
        // param4 : yaw angle   (not supported)
        // param5 : latitude    (not supported)
        // param6 : longitude   (not supported)
        // param7 : altitude [metres]

        float takeoff_alt = packet.param7 * 100;      // Convert m to cm

        if (!copter.flightmode->do_user_takeoff(takeoff_alt, is_zero(packet.param3))) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
    }

#if MODE_AUTO_ENABLED == ENABLED
    case MAV_CMD_DO_LAND_START:
        if (copter.mode_auto.jump_to_landing_sequence_auto_RTL(ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
#endif

    case MAV_CMD_NAV_LOITER_UNLIM:
        if (!copter.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        if (!copter.set_mode(Mode::Number::RTL, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

    case MAV_CMD_NAV_LAND:
        if (!copter.set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

#if MODE_FOLLOW_ENABLED == ENABLED
    case MAV_CMD_DO_FOLLOW:
        // param1: sysid of target to follow
        if ((packet.param1 > 0) && (packet.param1 <= 255)) {
            copter.g2.follow.set_target_sysid((uint8_t)packet.param1);
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
#endif

    case MAV_CMD_CONDITION_YAW:
        // param1 : target angle [0-360]
        // param2 : speed during change [deg per second]
        // param3 : direction (-1:ccw, +1:cw)
        // param4 : relative offset (1) or absolute angle (0)
        if ((packet.param1 >= 0.0f)   &&
            (packet.param1 <= 360.0f) &&
            (is_zero(packet.param4) || is_equal(packet.param4,1.0f))) {
            copter.flightmode->auto_yaw.set_fixed_yaw(
                packet.param1,
                packet.param2,
                (int8_t)packet.param3,
                is_positive(packet.param4));
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

    case MAV_CMD_DO_CHANGE_SPEED:
        // param1 : Speed type (0 or 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
        // param2 : new speed in m/s
        // param3 : unused
        // param4 : unused
        if (packet.param2 > 0.0f) {
            if (packet.param1 > 2.9f) { // 3 = speed down
                copter.wp_nav->set_speed_down(packet.param2 * 100.0f);
            } else if (packet.param1 > 1.9f) { // 2 = speed up
                copter.wp_nav->set_speed_up(packet.param2 * 100.0f);
            } else {
                copter.wp_nav->set_speed_xy(packet.param2 * 100.0f);
            }
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;

#if MODE_AUTO_ENABLED == ENABLED
    case MAV_CMD_MISSION_START:
        if (copter.set_mode(Mode::Number::AUTO, ModeReason::GCS_COMMAND)) {
            copter.set_auto_armed(true);
            if (copter.mode_auto.mission.state() != AP_Mission::MISSION_RUNNING) {
                copter.mode_auto.mission.start_or_resume();
            }
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
#endif

#if PARACHUTE == ENABLED
    case MAV_CMD_DO_PARACHUTE:
        // configure or release parachute
        switch ((uint16_t)packet.param1) {
        case PARACHUTE_DISABLE:
            copter.parachute.enabled(false);
            AP::logger().Write_Event(LogEvent::PARACHUTE_DISABLED);
            return MAV_RESULT_ACCEPTED;
        case PARACHUTE_ENABLE:
            copter.parachute.enabled(true);
            AP::logger().Write_Event(LogEvent::PARACHUTE_ENABLED);
            return MAV_RESULT_ACCEPTED;
        case PARACHUTE_RELEASE:
            // treat as a manual release which performs some additional check of altitude
            copter.parachute_manual_release();
            return MAV_RESULT_ACCEPTED;
        }
        return MAV_RESULT_FAILED;
#endif

    case MAV_CMD_DO_MOTOR_TEST:
        // param1 : motor sequence number (a number from 1 to max number of motors on the vehicle)
        // param2 : throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)
        // param3 : throttle (range depends upon param2)
        // param4 : timeout (in seconds)
        // param5 : num_motors (in sequence)
        // param6 : motor test order
        return copter.mavlink_motor_test_start(*this,
                                               (uint8_t)packet.param1,
                                               (uint8_t)packet.param2,
                                               packet.param3,
                                               packet.param4,
                                               (uint8_t)packet.param5);

#if WINCH_ENABLED == ENABLED
    case MAV_CMD_DO_WINCH:
        // param1 : winch number (ignored)
        // param2 : action (0=relax, 1=relative length control, 2=rate control). See WINCH_ACTIONS enum.
        if (!copter.g2.winch.enabled()) {
            return MAV_RESULT_FAILED;
        }
        switch ((uint8_t)packet.param2) {
        case WINCH_RELAXED:
            copter.g2.winch.relax();
            return MAV_RESULT_ACCEPTED;
        case WINCH_RELATIVE_LENGTH_CONTROL: {
            copter.g2.winch.release_length(packet.param3);
            return MAV_RESULT_ACCEPTED;
        }
        case WINCH_RATE_CONTROL:
            copter.g2.winch.set_desired_rate(packet.param4);
            return MAV_RESULT_ACCEPTED;
        default:
            break;
        }
        return MAV_RESULT_FAILED;
#endif

#if LANDING_GEAR_ENABLED == ENABLED
        case MAV_CMD_AIRFRAME_CONFIGURATION: {
            // Param 1: Select which gear, not used in ArduPilot
            // Param 2: 0 = Deploy, 1 = Retract
            // For safety, anything other than 1 will deploy
            switch ((uint8_t)packet.param2) {
                case 1:
                    copter.landinggear.set_position(AP_LandingGear::LandingGear_Retract);
                    return MAV_RESULT_ACCEPTED;
                default:
                    copter.landinggear.set_position(AP_LandingGear::LandingGear_Deploy);
                    return MAV_RESULT_ACCEPTED;
            }
            return MAV_RESULT_FAILED;
        }
#endif

        /* Solo user presses Fly button */
    case MAV_CMD_SOLO_BTN_FLY_CLICK: {
        if (copter.failsafe.radio) {
            return MAV_RESULT_ACCEPTED;
        }

        // set mode to Loiter or fall back to AltHold
        if (!copter.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND)) {
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND);
        }
        return MAV_RESULT_ACCEPTED;
    }

        /* Solo user holds down Fly button for a couple of seconds */
    case MAV_CMD_SOLO_BTN_FLY_HOLD: {
        if (copter.failsafe.radio) {
            return MAV_RESULT_ACCEPTED;
        }

        if (!copter.motors->armed()) {
            // if disarmed, arm motors
            copter.arming.arm(AP_Arming::Method::MAVLINK);
        } else if (copter.ap.land_complete) {
            // if armed and landed, takeoff
            if (copter.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND)) {
                copter.flightmode->do_user_takeoff(packet.param1*100, true);
            }
        } else {
            // if flying, land
            copter.set_mode(Mode::Number::LAND, ModeReason::GCS_COMMAND);
        }
        return MAV_RESULT_ACCEPTED;
    }

        /* Solo user presses pause button */
    case MAV_CMD_SOLO_BTN_PAUSE_CLICK: {
        if (copter.failsafe.radio) {
            return MAV_RESULT_ACCEPTED;
        }

        if (copter.motors->armed()) {
            if (copter.ap.land_complete) {
                // if landed, disarm motors
                copter.arming.disarm(AP_Arming::Method::SOLOPAUSEWHENLANDED);
            } else {
                // assume that shots modes are all done in guided.
                // NOTE: this may need to change if we add a non-guided shot mode
                bool shot_mode = (!is_zero(packet.param1) && (copter.flightmode->mode_number() == Mode::Number::GUIDED || copter.flightmode->mode_number() == Mode::Number::GUIDED_NOGPS));

                if (!shot_mode) {
#if MODE_BRAKE_ENABLED == ENABLED
                    if (copter.set_mode(Mode::Number::BRAKE, ModeReason::GCS_COMMAND)) {
                        copter.mode_brake.timeout_to_loiter_ms(2500);
                    } else {
                        copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND);
                    }
#else
                    copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::GCS_COMMAND);
#endif
                } else {
                    // SoloLink is expected to handle pause in shots
                }
            }
        }
        return MAV_RESULT_ACCEPTED;
    }

    // pause or resume an auto mission
    case MAV_CMD_DO_PAUSE_CONTINUE: {
        mavlink_command_int_t packet_int;
        GCS_MAVLINK_Copter::convert_COMMAND_LONG_to_COMMAND_INT(packet, packet_int);
        return handle_command_pause_continue(packet_int);
    }
    default:
        return GCS_MAVLINK::handle_command_long_packet(packet);
    }
}

MAV_RESULT GCS_MAVLINK_Copter::handle_command_pause_continue(const mavlink_command_int_t &packet)
{
    // requested pause
    if ((uint8_t) packet.param1 == 0) {
        if (copter.flightmode->pause()) {
            return MAV_RESULT_ACCEPTED;
        }
        send_text(MAV_SEVERITY_INFO, "Failed to pause");
        return MAV_RESULT_FAILED;
    }

    // requested resume
    if ((uint8_t) packet.param1 == 1) {
        if (copter.flightmode->resume()) {
            return MAV_RESULT_ACCEPTED;
        }
        send_text(MAV_SEVERITY_INFO, "Failed to resume");
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_DENIED;
}

void GCS_MAVLINK_Copter::handle_mount_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
#if HAL_MOUNT_ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONTROL:
        // if vehicle has a camera mount but it doesn't do pan control then yaw the entire vehicle instead
        if ((copter.camera_mount.get_mount_type() != copter.camera_mount.MountType::Mount_Type_None) &&
            !copter.camera_mount.has_pan_control()) {
            copter.flightmode->auto_yaw.set_yaw_angle_rate(
                mavlink_msg_mount_control_get_input_c(&msg) * 0.01f,
                0.0f);

            break;
        }
#endif
    }
    GCS_MAVLINK::handle_mount_message(msg);
}

void GCS_MAVLINK_Copter::handleMessage(const mavlink_message_t &msg)
{
    // for mavlink SET_POSITION_TARGET messages
    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE =
        POSITION_TARGET_TYPEMASK_X_IGNORE |
        POSITION_TARGET_TYPEMASK_Y_IGNORE |
        POSITION_TARGET_TYPEMASK_Z_IGNORE;

    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE =
        POSITION_TARGET_TYPEMASK_VX_IGNORE |
        POSITION_TARGET_TYPEMASK_VY_IGNORE |
        POSITION_TARGET_TYPEMASK_VZ_IGNORE;

    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE =
        POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE;

    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE =
        POSITION_TARGET_TYPEMASK_YAW_IGNORE;
    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE =
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    constexpr uint32_t MAVLINK_SET_POS_TYPE_MASK_FORCE_SET =
        POSITION_TARGET_TYPEMASK_FORCE_SET;

    switch (msg.msgid) {

    case MAVLINK_MSG_ID_MANUAL_CONTROL:
    {
        if (msg.sysid != copter.g.sysid_my_gcs) {
            break; // only accept control from our gcs
        }

        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(&msg, &packet);

        if (packet.target != copter.g.sysid_this_mav) {
            break; // only accept control aimed at us
        }

        if (packet.z < 0) { // Copter doesn't do negative thrust
            break;
        }

        uint32_t tnow = AP_HAL::millis();

        manual_override(copter.channel_roll, packet.y, 1000, 2000, tnow);
        manual_override(copter.channel_pitch, packet.x, 1000, 2000, tnow, true);
        manual_override(copter.channel_throttle, packet.z, 0, 1000, tnow);
        manual_override(copter.channel_yaw, packet.r, 1000, 2000, tnow);

        // a manual control message is considered to be a 'heartbeat'
        // from the ground station for failsafe purposes
        gcs().sysid_myggcs_seen(tnow);
        break;
    }

#if MODE_GUIDED_ENABLED == ENABLED
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:   // MAV ID: 82
    {
        // decode packet
        mavlink_set_attitude_target_t packet;
        mavlink_msg_set_attitude_target_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if (!copter.flightmode->in_guided_mode()) {
            break;
        }

        const bool roll_rate_ignore   = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE;
        const bool pitch_rate_ignore  = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE;
        const bool yaw_rate_ignore    = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE;
        const bool throttle_ignore    = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE;
        const bool attitude_ignore    = packet.type_mask & ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE;

        // ensure thrust field is not ignored
        if (throttle_ignore) {
            break;
        }

        Quaternion attitude_quat;
        if (attitude_ignore) {
            attitude_quat.zero();
        } else {
            attitude_quat = Quaternion(packet.q[0],packet.q[1],packet.q[2],packet.q[3]);

            // Do not accept the attitude_quaternion
            // if its magnitude is not close to unit length +/- 1E-3
            // this limit is somewhat greater than sqrt(FLT_EPSL)
            if (!attitude_quat.is_unit_length()) {
                // The attitude quaternion is ill-defined
                break;
            }
        }

        // check if the message's thrust field should be interpreted as a climb rate or as thrust
        const bool use_thrust = copter.mode_guided.set_attitude_target_provides_thrust();

        float climb_rate_or_thrust;
        if (use_thrust) {
            // interpret thrust as thrust
            climb_rate_or_thrust = constrain_float(packet.thrust, -1.0f, 1.0f);
        } else {
            // convert thrust to climb rate
            packet.thrust = constrain_float(packet.thrust, 0.0f, 1.0f);
            if (is_equal(packet.thrust, 0.5f)) {
                climb_rate_or_thrust = 0.0f;
            } else if (packet.thrust > 0.5f) {
                // climb at up to WPNAV_SPEED_UP
                climb_rate_or_thrust = (packet.thrust - 0.5f) * 2.0f * copter.wp_nav->get_default_speed_up();
            } else {
                // descend at up to WPNAV_SPEED_DN
                climb_rate_or_thrust = (0.5f - packet.thrust) * 2.0f * -copter.wp_nav->get_default_speed_down();
            }
        }

        Vector3f ang_vel;
        if (!roll_rate_ignore) {
            ang_vel.x = packet.body_roll_rate;
        }
        if (!pitch_rate_ignore) {
            ang_vel.y = packet.body_pitch_rate;
        }
        if (!yaw_rate_ignore) {
            ang_vel.z = packet.body_yaw_rate;
        }

        copter.mode_guided.set_angle(attitude_quat, ang_vel,
                climb_rate_or_thrust, use_thrust);

        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:     // MAV ID: 84
    {
        // decode packet
        mavlink_set_position_target_local_ned_t packet;
        mavlink_msg_set_position_target_local_ned_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if (!copter.flightmode->in_guided_mode()) {
            break;
        }

        // check for supported coordinate frames
        if (packet.coordinate_frame != MAV_FRAME_LOCAL_NED &&
            packet.coordinate_frame != MAV_FRAME_LOCAL_OFFSET_NED &&
            packet.coordinate_frame != MAV_FRAME_BODY_NED &&
            packet.coordinate_frame != MAV_FRAME_BODY_OFFSET_NED) {
            // input is not valid so stop
            copter.mode_guided.init(true);
            break;
        }

        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
        bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
        bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
        bool force_set       = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE_SET;

        // Force inputs are not supported
        // Do not accept command if force_set is true and acc_ignore is false
        if (force_set && !acc_ignore) {
            break;
        }

        // prepare position
        Vector3f pos_vector;
        if (!pos_ignore) {
            // convert to cm
            pos_vector = Vector3f(packet.x * 100.0f, packet.y * 100.0f, -packet.z * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                copter.rotate_body_frame_to_NE(pos_vector.x, pos_vector.y);
            }
            // add body offset if necessary
            if (packet.coordinate_frame == MAV_FRAME_LOCAL_OFFSET_NED ||
                packet.coordinate_frame == MAV_FRAME_BODY_NED ||
                packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                pos_vector += copter.inertial_nav.get_position_neu_cm();
            }
        }

        // prepare velocity
        Vector3f vel_vector;
        if (!vel_ignore) {
            // convert to cm
            vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                copter.rotate_body_frame_to_NE(vel_vector.x, vel_vector.y);
            }
        }

        // prepare acceleration
        Vector3f accel_vector;
        if (!acc_ignore) {
            // convert to cm
            accel_vector = Vector3f(packet.afx * 100.0f, packet.afy * 100.0f, -packet.afz * 100.0f);
            // rotate to body-frame if necessary
            if (packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED) {
                copter.rotate_body_frame_to_NE(accel_vector.x, accel_vector.y);
            }
        }

        // prepare yaw
        float yaw_cd = 0.0f;
        bool yaw_relative = false;
        float yaw_rate_cds = 0.0f;
        if (!yaw_ignore) {
            yaw_cd = ToDeg(packet.yaw) * 100.0f;
            yaw_relative = packet.coordinate_frame == MAV_FRAME_BODY_NED || packet.coordinate_frame == MAV_FRAME_BODY_OFFSET_NED;
        }
        if (!yaw_rate_ignore) {
            yaw_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
        }

        // send request
        if (!pos_ignore && !vel_ignore) {
            copter.mode_guided.set_destination_posvelaccel(pos_vector, vel_vector, accel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (pos_ignore && !vel_ignore) {
            copter.mode_guided.set_velaccel(vel_vector, accel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (pos_ignore && vel_ignore && !acc_ignore) {
            copter.mode_guided.set_accel(accel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative);
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            copter.mode_guided.set_destination(pos_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds, yaw_relative, false);
        } else {
            // input is not valid so stop
            copter.mode_guided.init(true);
        }

        break;
    }

    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:    // MAV ID: 86
    {
        // decode packet
        mavlink_set_position_target_global_int_t packet;
        mavlink_msg_set_position_target_global_int_decode(&msg, &packet);

        // exit if vehicle is not in Guided mode or Auto-Guided mode
        if (!copter.flightmode->in_guided_mode()) {
            break;
        }

        // todo: do we need to check for supported coordinate frames

        bool pos_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE;
        bool vel_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE;
        bool acc_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE;
        bool yaw_ignore      = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE;
        bool yaw_rate_ignore = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE;
        bool force_set       = packet.type_mask & MAVLINK_SET_POS_TYPE_MASK_FORCE_SET;

        // Force inputs are not supported
        // Do not accept command if force_set is true and acc_ignore is false
        if (force_set && !acc_ignore) {
            break;
        }

        // extract location from message
        Location loc;
        if (!pos_ignore) {
            // sanity check location
            if (!check_latlng(packet.lat_int, packet.lon_int)) {
                // input is not valid so stop
                copter.mode_guided.init(true);
                break;
            }
            Location::AltFrame frame;
            if (!mavlink_coordinate_frame_to_location_alt_frame((MAV_FRAME)packet.coordinate_frame, frame)) {
                // unknown coordinate frame
                // input is not valid so stop
                copter.mode_guided.init(true);
                break;
            }
            loc = {packet.lat_int, packet.lon_int, int32_t(packet.alt*100), frame};
        }

        // prepare velocity
        Vector3f vel_vector;
        if (!vel_ignore) {
            // convert to cm
            vel_vector = Vector3f(packet.vx * 100.0f, packet.vy * 100.0f, -packet.vz * 100.0f);
        }

        // prepare acceleration
        Vector3f accel_vector;
        if (!acc_ignore) {
            // convert to cm
            accel_vector = Vector3f(packet.afx * 100.0f, packet.afy * 100.0f, -packet.afz * 100.0f);
        }

        // prepare yaw
        float yaw_cd = 0.0f;
        float yaw_rate_cds = 0.0f;
        if (!yaw_ignore) {
            yaw_cd = ToDeg(packet.yaw) * 100.0f;
        }
        if (!yaw_rate_ignore) {
            yaw_rate_cds = ToDeg(packet.yaw_rate) * 100.0f;
        }

        // send targets to the appropriate guided mode controller
        if (!pos_ignore && !vel_ignore) {
            // convert Location to vector from ekf origin for posvel controller
            if (loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) {
                // posvel controller does not support alt-above-terrain
                // input is not valid so stop
                copter.mode_guided.init(true);
                break;
            }
            Vector3f pos_neu_cm;
            if (!loc.get_vector_from_origin_NEU(pos_neu_cm)) {
                // input is not valid so stop
                copter.mode_guided.init(true);
                break;
            }
            copter.mode_guided.set_destination_posvel(pos_neu_cm, vel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds);
        } else if (pos_ignore && !vel_ignore) {
            copter.mode_guided.set_velaccel(vel_vector, accel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds);
        } else if (pos_ignore && vel_ignore && !acc_ignore) {
            copter.mode_guided.set_accel(accel_vector, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds);
        } else if (!pos_ignore && vel_ignore && acc_ignore) {
            copter.mode_guided.set_destination(loc, !yaw_ignore, yaw_cd, !yaw_rate_ignore, yaw_rate_cds);
        } else {
            // input is not valid so stop
            copter.mode_guided.init(true);
        }

        break;
    }
#endif

    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS:       // MAV ID: 109
    {
        handle_radio_status(msg, copter.should_log(MASK_LOG_PM));
        break;
    }

    case MAVLINK_MSG_ID_TERRAIN_DATA:
    case MAVLINK_MSG_ID_TERRAIN_CHECK:
#if AP_TERRAIN_AVAILABLE
        copter.terrain.handle_data(chan, msg);
#endif
        break;

    case MAVLINK_MSG_ID_SET_HOME_POSITION:
    {
        send_received_message_deprecation_warning(STR_VALUE(MAVLINK_MSG_ID_SET_HOME_POSITION));

        mavlink_set_home_position_t packet;
        mavlink_msg_set_home_position_decode(&msg, &packet);
        if ((packet.latitude == 0) && (packet.longitude == 0) && (packet.altitude == 0)) {
            IGNORE_RETURN(copter.set_home_to_current_location(true));
        } else {
            Location new_home_loc;
            new_home_loc.lat = packet.latitude;
            new_home_loc.lng = packet.longitude;
            new_home_loc.alt = packet.altitude / 10;
            IGNORE_RETURN(copter.set_home(new_home_loc, true));
        }
        break;
    }

#if TOY_MODE_ENABLED == ENABLED
    case MAVLINK_MSG_ID_NAMED_VALUE_INT:
        copter.g2.toy_mode.handle_message(msg);
        break;
#endif
        
    default:
        handle_common_message(msg);
        break;
    }     // end switch
} // end handle mavlink


MAV_RESULT GCS_MAVLINK_Copter::handle_flight_termination(const mavlink_command_long_t &packet) {
#if ADVANCED_FAILSAFE == ENABLED
    if (GCS_MAVLINK::handle_flight_termination(packet) == MAV_RESULT_ACCEPTED) {
        return MAV_RESULT_ACCEPTED;
    }
#endif
    if (packet.param1 > 0.5f) {
        copter.arming.disarm(AP_Arming::Method::TERMINATION);
        return MAV_RESULT_ACCEPTED;
    }

    return MAV_RESULT_FAILED;
}

float GCS_MAVLINK_Copter::vfr_hud_alt() const
{
    if (copter.g2.dev_options.get() & DevOptionVFR_HUDRelativeAlt) {
        // compatibility option for older mavlink-aware devices that
        // assume Copter returns a relative altitude in VFR_HUD.alt
        return copter.current_loc.alt * 0.01f;
    }
    return GCS_MAVLINK::vfr_hud_alt();
}

uint64_t GCS_MAVLINK_Copter::capabilities() const
{
    return (MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT |
            MAV_PROTOCOL_CAPABILITY_MISSION_INT |
            MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED |
            MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION |
            MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET |
#if AP_TERRAIN_AVAILABLE
            (copter.terrain.enabled() ? MAV_PROTOCOL_CAPABILITY_TERRAIN : 0) |
#endif
            GCS_MAVLINK::capabilities());
}

MAV_LANDED_STATE GCS_MAVLINK_Copter::landed_state() const
{
    if (copter.ap.land_complete) {
        return MAV_LANDED_STATE_ON_GROUND;
    }
    if (copter.flightmode->is_landing()) {
        return MAV_LANDED_STATE_LANDING;
    }
    if (copter.flightmode->is_taking_off()) {
        return MAV_LANDED_STATE_TAKEOFF;
    }
    return MAV_LANDED_STATE_IN_AIR;
}

void GCS_MAVLINK_Copter::send_wind() const
{
    Vector3f airspeed_vec_bf;
    if (!AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        // if we don't have an airspeed estimate then we don't have a
        // valid wind estimate on copters
        return;
    }
    const Vector3f wind = AP::ahrs().wind_estimate();
    mavlink_msg_wind_send(
        chan,
        degrees(atan2f(-wind.y, -wind.x)),
        wind.length(),
        wind.z);
}

#if HAL_HIGH_LATENCY2_ENABLED
int16_t GCS_MAVLINK_Copter::high_latency_target_altitude() const
{
    AP_AHRS &ahrs = AP::ahrs();
    struct Location global_position_current;
    UNUSED_RESULT(ahrs.get_location(global_position_current));

    //return units are m
    if (copter.ap.initialised) {
        return 0.01 * (global_position_current.alt + copter.pos_control->get_pos_error_z_cm());
    }
    return 0;
    
}

uint8_t GCS_MAVLINK_Copter::high_latency_tgt_heading() const
{
    if (copter.ap.initialised) {
        // return units are deg/2
        const Mode *flightmode = copter.flightmode;
        // need to convert -18000->18000 to 0->360/2
        return wrap_360_cd(flightmode->wp_bearing()) / 200;
    }
    return 0;     
}
    
uint16_t GCS_MAVLINK_Copter::high_latency_tgt_dist() const
{
    if (copter.ap.initialised) {
        // return units are dm
        const Mode *flightmode = copter.flightmode;
        return MIN(flightmode->wp_distance() * 1.0e-2, UINT16_MAX) / 10;
    }
    return 0;
}

uint8_t GCS_MAVLINK_Copter::high_latency_tgt_airspeed() const
{
    if (copter.ap.initialised) {
        // return units are m/s*5
        return MIN(copter.pos_control->get_vel_target_cms().length() * 5.0e-2, UINT8_MAX);
    }
    return 0;  
}

uint8_t GCS_MAVLINK_Copter::high_latency_wind_speed() const
{
    Vector3f airspeed_vec_bf;
    Vector3f wind;
    // return units are m/s*5
    if (AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        wind = AP::ahrs().wind_estimate();
        return wind.length() * 5;
    }
    return 0; 
}

uint8_t GCS_MAVLINK_Copter::high_latency_wind_direction() const
{
    Vector3f airspeed_vec_bf;
    Vector3f wind;
    // return units are deg/2
    if (AP::ahrs().airspeed_vector_true(airspeed_vec_bf)) {
        wind = AP::ahrs().wind_estimate();
        // need to convert -180->180 to 0->360/2
        return wrap_360(degrees(atan2f(-wind.y, -wind.x))) / 2;
    }
    return 0;
}
#endif // HAL_HIGH_LATENCY2_ENABLED
Log.cpp
#include "Copter.h"

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from AP_Logger log memory
// Code to interact with the user to dump or erase logs

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    throttle_in;
    float    angle_boost;
    float    throttle_out;
    float    throttle_hover;
    float    desired_alt;
    float    inav_alt;
    int32_t  baro_alt;
    float    desired_rangefinder_alt;
    float    rangefinder_alt;
    float    terr_alt;
    int16_t  target_climb_rate;
    int16_t  climb_rate;
};

// Write a control tuning packet
void Copter::Log_Write_Control_Tuning()
{
    // get terrain altitude
    float terr_alt = 0.0f;
#if AP_TERRAIN_AVAILABLE
    if (!terrain.height_above_terrain(terr_alt, true)) {
        terr_alt = logger.quiet_nan();
    }
#endif
    float des_alt_m = 0.0f;
    int16_t target_climb_rate_cms = 0;
    if (!flightmode->has_manual_throttle()) {
        des_alt_m = pos_control->get_pos_target_z_cm() * 0.01f;
        target_climb_rate_cms = pos_control->get_vel_target_z_cms();
    }

    // get surface tracking alts
    float desired_rangefinder_alt;
    if (!surface_tracking.get_target_dist_for_logging(desired_rangefinder_alt)) {
        desired_rangefinder_alt = AP::logger().quiet_nan();
    }

    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_in         : attitude_control->get_throttle_in(),
        angle_boost         : attitude_control->angle_boost(),
        throttle_out        : motors->get_throttle(),
        throttle_hover      : motors->get_throttle_hover(),
        desired_alt         : des_alt_m,
        inav_alt            : inertial_nav.get_position_z_up_cm() * 0.01f,
        baro_alt            : baro_alt,
        desired_rangefinder_alt : desired_rangefinder_alt,
        rangefinder_alt     : surface_tracking.get_dist_for_logging(),
        terr_alt            : terr_alt,
        target_climb_rate   : target_climb_rate_cms,
        climb_rate          : int16_t(inertial_nav.get_velocity_z_up_cms()) // float -> int16_t
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void Copter::Log_Write_Attitude()
{
    Vector3f targets = attitude_control->get_att_target_euler_cd();
    targets.z = wrap_360_cd(targets.z);
    ahrs.Write_Attitude(targets);
    ahrs_view->Write_Rate(*motors, *attitude_control, *pos_control);
    if (should_log(MASK_LOG_PID)) {
        logger.Write_PID(LOG_PIDR_MSG, attitude_control->get_rate_roll_pid().get_pid_info());
        logger.Write_PID(LOG_PIDP_MSG, attitude_control->get_rate_pitch_pid().get_pid_info());
        logger.Write_PID(LOG_PIDY_MSG, attitude_control->get_rate_yaw_pid().get_pid_info());
        logger.Write_PID(LOG_PIDA_MSG, pos_control->get_accel_z_pid().get_pid_info() );
        if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS())) {
            logger.Write_PID(LOG_PIDN_MSG, pos_control->get_vel_xy_pid().get_pid_info_x());
            logger.Write_PID(LOG_PIDE_MSG, pos_control->get_vel_xy_pid().get_pid_info_y());
        }
    }
}

// Write an EKF and POS packet
void Copter::Log_Write_EKF_POS()
{
    AP::ahrs().Log_Write();
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
UNUSED_FUNCTION
void Copter::Log_Write_Data(LogDataID id, int16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
UNUSED_FUNCTION 
void Copter::Log_Write_Data(LogDataID id, uint16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
void Copter::Log_Write_Data(LogDataID id, int32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            time_us  : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
void Copter::Log_Write_Data(LogDataID id, uint32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    float data_value;
};

// Write a float data packet
UNUSED_FUNCTION
void Copter::Log_Write_Data(LogDataID id, float value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_ParameterTuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  parameter;     // parameter we are tuning, e.g. 39 is CH6_CIRCLE_RATE
    float    tuning_value;  // normalized value used inside tuning() function
    float    tuning_min;    // tuning minimum value
    float    tuning_max;    // tuning maximum value
};

void Copter::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max)
{
    struct log_ParameterTuning pkt_tune = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMTUNE_MSG),
        time_us        : AP_HAL::micros64(),
        parameter      : param,
        tuning_value   : tuning_val,
        tuning_min     : tune_min,
        tuning_max     : tune_max
    };

    logger.WriteBlock(&pkt_tune, sizeof(pkt_tune));
}

// logs when compass becomes unhealthy
void Copter::Log_Sensor_Health()
{
    if (!should_log(MASK_LOG_ANY)) {
        return;
    }

    // check compass
    if (sensor_health.compass != compass.healthy()) {
        sensor_health.compass = compass.healthy();
        AP::logger().Write_Error(LogErrorSubsystem::COMPASS, (sensor_health.compass ? LogErrorCode::ERROR_RESOLVED : LogErrorCode::UNHEALTHY));
    }
}

void Copter::Log_Video_Stabilisation()
{
    if (!should_log(MASK_LOG_VIDEO_STABILISATION)) {
        return;
    }
    ahrs.write_video_stabilisation();
}

struct PACKED log_SysIdD {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    waveform_time;
    float    waveform_sample;
    float    waveform_freq;
    float    angle_x;
    float    angle_y;
    float    angle_z;
    float    accel_x;
    float    accel_y;
    float    accel_z;
};

// Write an rate packet
void Copter::Log_Write_SysID_Data(float waveform_time, float waveform_sample, float waveform_freq, float angle_x, float angle_y, float angle_z, float accel_x, float accel_y, float accel_z)
{
#if MODE_SYSTEMID_ENABLED == ENABLED
    struct log_SysIdD pkt_sidd = {
        LOG_PACKET_HEADER_INIT(LOG_SYSIDD_MSG),
        time_us         : AP_HAL::micros64(),
        waveform_time   : waveform_time,
        waveform_sample : waveform_sample,
        waveform_freq   : waveform_freq,
        angle_x         : angle_x,
        angle_y         : angle_y,
        angle_z         : angle_z,
        accel_x         : accel_x,
        accel_y         : accel_y,
        accel_z         : accel_z
    };
    logger.WriteBlock(&pkt_sidd, sizeof(pkt_sidd));
#endif
}

struct PACKED log_SysIdS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  systemID_axis;
    float    waveform_magnitude;
    float    frequency_start;
    float    frequency_stop;
    float    time_fade_in;
    float    time_const_freq;
    float    time_record;
    float    time_fade_out;
};

// Write an rate packet
void Copter::Log_Write_SysID_Setup(uint8_t systemID_axis, float waveform_magnitude, float frequency_start, float frequency_stop, float time_fade_in, float time_const_freq, float time_record, float time_fade_out)
{
#if MODE_SYSTEMID_ENABLED == ENABLED
    struct log_SysIdS pkt_sids = {
        LOG_PACKET_HEADER_INIT(LOG_SYSIDS_MSG),
        time_us             : AP_HAL::micros64(),
        systemID_axis       : systemID_axis,
        waveform_magnitude  : waveform_magnitude,
        frequency_start     : frequency_start,
        frequency_stop      : frequency_stop,
        time_fade_in        : time_fade_in,
        time_const_freq     : time_const_freq,
        time_record         : time_record,
        time_fade_out       : time_fade_out
    };
    logger.WriteBlock(&pkt_sids, sizeof(pkt_sids));
#endif
}

#if FRAME_CONFIG == HELI_FRAME
struct PACKED log_Heli {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    desired_rotor_speed;
    float    main_rotor_speed;
    float    governor_output;
    float    control_output;
};

// Write an helicopter packet
void Copter::Log_Write_Heli()
{
    struct log_Heli pkt_heli = {
        LOG_PACKET_HEADER_INIT(LOG_HELI_MSG),
        time_us                 : AP_HAL::micros64(),
        desired_rotor_speed     : motors->get_desired_rotor_speed(),
        main_rotor_speed        : motors->get_main_rotor_speed(),
        governor_output         : motors->get_governor_output(),
        control_output          : motors->get_control_output(),
    };
    logger.WriteBlock(&pkt_heli, sizeof(pkt_heli));
}
#endif

// guided position target logging
struct PACKED log_Guided_Position_Target {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float pos_target_x;
    float pos_target_y;
    float pos_target_z;
    uint8_t terrain;
    float vel_target_x;
    float vel_target_y;
    float vel_target_z;
    float accel_target_x;
    float accel_target_y;
    float accel_target_z;
};

// guided attitude target logging
struct PACKED log_Guided_Attitude_Target {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float roll;
    float pitch;
    float yaw;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float thrust;
    float climb_rate;
};

// Write a Guided mode position target
// pos_target is lat, lon, alt OR offset from ekf origin in cm
// terrain should be 0 if pos_target.z is alt-above-ekf-origin, 1 if alt-above-terrain
// vel_target is cm/s
void Copter::Log_Write_Guided_Position_Target(ModeGuided::SubMode target_type, const Vector3f& pos_target, bool terrain_alt, const Vector3f& vel_target, const Vector3f& accel_target)
{
    const log_Guided_Position_Target pkt {
        LOG_PACKET_HEADER_INIT(LOG_GUIDED_POSITION_TARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : (uint8_t)target_type,
        pos_target_x    : pos_target.x,
        pos_target_y    : pos_target.y,
        pos_target_z    : pos_target.z,
        terrain         : terrain_alt,
        vel_target_x    : vel_target.x,
        vel_target_y    : vel_target.y,
        vel_target_z    : vel_target.z,
        accel_target_x  : accel_target.x,
        accel_target_y  : accel_target.y,
        accel_target_z  : accel_target.z
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// Write a Guided mode attitude target
// roll, pitch and yaw are in radians
// ang_vel: angular velocity, [roll rate, pitch_rate, yaw_rate] in radians/sec
// thrust is between 0 to 1
// climb_rate is in (m/s)
void Copter::Log_Write_Guided_Attitude_Target(ModeGuided::SubMode target_type, float roll, float pitch, float yaw, const Vector3f &ang_vel, float thrust, float climb_rate)
{
    const log_Guided_Attitude_Target pkt {
        LOG_PACKET_HEADER_INIT(LOG_GUIDED_ATTITUDE_TARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : (uint8_t)target_type,
        roll            : degrees(roll),       // rad to deg
        pitch           : degrees(pitch),      // rad to deg
        yaw             : degrees(yaw),        // rad to deg
        roll_rate       : degrees(ang_vel.x),  // rad/s to deg/s
        pitch_rate      : degrees(ang_vel.y),  // rad/s to deg/s
        yaw_rate        : degrees(ang_vel.z),  // rad/s to deg/s
        thrust          : thrust,
        climb_rate      : climb_rate
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Copter::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    
// @LoggerMessage: PTUN
// @Description: Parameter Tuning information
// @URL: https://ardupilot.org/copter/docs/tuning.html#in-flight-tuning
// @Field: TimeUS: Time since system startup
// @Field: Param: Parameter being tuned
// @Field: TunVal: Normalized value used inside tuning() function
// @Field: TunMin: Tuning minimum limit
// @Field: TunMax: Tuning maximum limit

    { LOG_PARAMTUNE_MSG, sizeof(log_ParameterTuning),
      "PTUN", "QBfff",         "TimeUS,Param,TunVal,TunMin,TunMax", "s----", "F----" },

// @LoggerMessage: CTUN
// @Description: Control Tuning information
// @Field: TimeUS: Time since system startup
// @Field: ThI: throttle input
// @Field: ABst: angle boost
// @Field: ThO: throttle output
// @Field: ThH: calculated hover throttle
// @Field: DAlt: desired altitude
// @Field: Alt: achieved altitude
// @Field: BAlt: barometric altitude
// @Field: DSAlt: desired rangefinder altitude
// @Field: SAlt: achieved rangefinder altitude
// @Field: TAlt: terrain altitude
// @Field: DCRt: desired climb rate
// @Field: CRt: climb rate

// @LoggerMessage: D16
// @Description: Generic 16-bit-signed-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DU16
// @Description: Generic 16-bit-unsigned-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: D32
// @Description: Generic 32-bit-signed-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DFLT
// @Description: Generic float storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DU32
// @Description: Generic 32-bit-unsigned-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qffffffefffhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt", "s----mmmmmmnn", "F----00B000BB" , true },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "QBh",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "QBH",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "QBi",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "QBI",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "QBf",         "TimeUS,Id,Value", "s--", "F--" },
    
// @LoggerMessage: HELI
// @Description: Helicopter related messages 
// @Field: TimeUS: Time since system startup
// @Field: DRRPM: Desired rotor speed
// @Field: ERRPM: Estimated rotor speed
// @Field: Gov: Governor Output
// @Field: Throt: Throttle output
#if FRAME_CONFIG == HELI_FRAME
    { LOG_HELI_MSG, sizeof(log_Heli),
      "HELI",  "Qffff",        "TimeUS,DRRPM,ERRPM,Gov,Throt", "s----", "F----" , true },
#endif

// @LoggerMessage: SIDD
// @Description: System ID data
// @Field: TimeUS: Time since system startup
// @Field: Time: Time reference for waveform
// @Field: Targ: Current waveform sample
// @Field: F: Instantaneous waveform frequency
// @Field: Gx: Delta angle, X-Axis
// @Field: Gy: Delta angle, Y-Axis
// @Field: Gz: Delta angle, Z-Axis
// @Field: Ax: Delta velocity, X-Axis
// @Field: Ay: Delta velocity, Y-Axis
// @Field: Az: Delta velocity, Z-Axis

    { LOG_SYSIDD_MSG, sizeof(log_SysIdD),
      "SIDD", "Qfffffffff",  "TimeUS,Time,Targ,F,Gx,Gy,Gz,Ax,Ay,Az", "ss-zkkkooo", "F---------" , true },

// @LoggerMessage: SIDS
// @Description: System ID settings
// @Field: TimeUS: Time since system startup
// @Field: Ax: The axis which is being excited
// @Field: Mag: Magnitude of the chirp waveform
// @Field: FSt: Frequency at the start of chirp
// @Field: FSp: Frequency at the end of chirp
// @Field: TFin: Time to reach maximum amplitude of chirp
// @Field: TC: Time at constant frequency before chirp starts
// @Field: TR: Time taken to complete chirp waveform
// @Field: TFout: Time to reach zero amplitude after chirp finishes

    { LOG_SYSIDS_MSG, sizeof(log_SysIdS),
      "SIDS", "QBfffffff",  "TimeUS,Ax,Mag,FSt,FSp,TFin,TC,TR,TFout", "s--ssssss", "F--------" , true },

// @LoggerMessage: GUIP
// @Description: Guided mode position target information
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of guided mode
// @Field: pX: Target position, X-Axis
// @Field: pY: Target position, Y-Axis
// @Field: pZ: Target position, Z-Axis
// @Field: Terrain: Target position, Z-Axis is alt above terrain
// @Field: vX: Target velocity, X-Axis
// @Field: vY: Target velocity, Y-Axis
// @Field: vZ: Target velocity, Z-Axis
// @Field: aX: Target acceleration, X-Axis
// @Field: aY: Target acceleration, Y-Axis
// @Field: aZ: Target acceleration, Z-Axis

    { LOG_GUIDED_POSITION_TARGET_MSG, sizeof(log_Guided_Position_Target),
      "GUIP",  "QBfffbffffff",    "TimeUS,Type,pX,pY,pZ,Terrain,vX,vY,vZ,aX,aY,aZ", "s-mmm-nnnooo", "F-BBB-BBBBBB" , true },

// @LoggerMessage: GUIA
// @Description: Guided mode attitude target information
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of guided mode
// @Field: Roll: Target attitude, Roll
// @Field: Pitch: Target attitude, Pitch
// @Field: Yaw: Target attitude, Yaw
// @Field: RollRt: Roll rate
// @Field: PitchRt: Pitch rate
// @Field: YawRt: Yaw rate
// @Field: Thrust: Thrust 
// @Field: ClimbRt: Climb rate

    { LOG_GUIDED_ATTITUDE_TARGET_MSG, sizeof(log_Guided_Attitude_Target),
      "GUIA",  "QBffffffff",    "TimeUS,Type,Roll,Pitch,Yaw,RollRt,PitchRt,YawRt,Thrust,ClimbRt", "s-dddkkk-n", "F-000000-0" , true },
};

void Copter::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    char frame_and_type_string[30];
    copter.motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    logger.Write_MessageF("%s", frame_and_type_string);
    logger.Write_Mode((uint8_t)flightmode->mode_number(), control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

void Copter::log_init(void)
{
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
}

#else // LOGGING_ENABLED

void Copter::Log_Write_Control_Tuning() {}
void Copter::Log_Write_Attitude(void) {}
void Copter::Log_Write_EKF_POS() {}
void Copter::Log_Write_Data(LogDataID id, int32_t value) {}
void Copter::Log_Write_Data(LogDataID id, uint32_t value) {}
void Copter::Log_Write_Data(LogDataID id, int16_t value) {}
void Copter::Log_Write_Data(LogDataID id, uint16_t value) {}
void Copter::Log_Write_Data(LogDataID id, float value) {}
void Copter::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max) {}
void Copter::Log_Sensor_Health() {}
void Copter::Log_Write_Guided_Position_Target(ModeGuided::SubMode target_type, const Vector3f& pos_target, bool terrain_alt, const Vector3f& vel_target, const Vector3f& accel_target) {}
void Copter::Log_Write_Guided_Attitude_Target(ModeGuided::SubMode target_type, float roll, float pitch, float yaw, const Vector3f &ang_vel, float thrust, float climb_rate) {}
void Copter::Log_Write_SysID_Setup(uint8_t systemID_axis, float waveform_magnitude, float frequency_start, float frequency_stop, float time_fade_in, float time_const_freq, float time_record, float time_fade_out) {}
void Copter::Log_Write_SysID_Data(float waveform_time, float waveform_sample, float waveform_freq, float angle_x, float angle_y, float angle_z, float accel_x, float accel_y, float accel_z) {}
void Copter::Log_Write_Vehicle_Startup_Messages() {}

#if FRAME_CONFIG == HELI_FRAME
void Copter::Log_Write_Heli() {}
#endif

void Copter::log_init(void) {}

#endif // LOGGING_ENABLED
Parameters.cpp
#include "Copter.h"

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
 *  ArduCopter parameter definitions
 *
 */

#define GSCALAR(v, name, def) { copter.g.v.vtype, name, Parameters::k_param_ ## v, &copter.g.v, {def_value : def} }
#define ASCALAR(v, name, def) { copter.aparm.v.vtype, name, Parameters::k_param_ ## v, (const void *)&copter.aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &copter.g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&copter.v, {group_info : class::var_info} }
#define GOBJECTPTR(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&copter.v, {group_info : class::var_info}, AP_PARAM_FLAG_POINTER }
#define GOBJECTVARPTR(v, name, var_info_ptr) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&copter.v, {group_info_ptr : var_info_ptr}, AP_PARAM_FLAG_POINTER | AP_PARAM_FLAG_INFO_POINTER }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, (const void *)&copter.v, {group_info : class::var_info} }

#if FRAME_CONFIG == HELI_FRAME
// 6 here is AP_Motors::MOTOR_FRAME_HELI
#define DEFAULT_FRAME_CLASS 6
#else
#define DEFAULT_FRAME_CLASS 0
#endif

const AP_Param::Info Copter::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    // @ReadOnly: True
    GSCALAR(format_version, "FORMAT_VERSION",   0),

    // @Param: SYSID_THISMAV
    // @DisplayName: MAVLink system ID of this vehicle
    // @Description: Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_this_mav, "SYSID_THISMAV",   MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: My ground station number
    // @Description: Allows restricting radio overrides to only come from my ground station
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_my_gcs,   "SYSID_MYGCS",     255),

    // @Param: PILOT_THR_FILT
    // @DisplayName: Throttle filter cutoff
    // @Description: Throttle filter cutoff (Hz) - active whenever altitude control is inactive - 0 to disable
    // @User: Advanced
    // @Units: Hz
    // @Range: 0 10
    // @Increment: .5
    GSCALAR(throttle_filt,  "PILOT_THR_FILT",     0),

    // @Param: PILOT_TKOFF_ALT
    // @DisplayName: Pilot takeoff altitude
    // @Description: Altitude that altitude control modes will climb to when a takeoff is triggered with the throttle stick.
    // @User: Standard
    // @Units: cm
    // @Range: 0.0 1000.0
    // @Increment: 10
    GSCALAR(pilot_takeoff_alt,  "PILOT_TKOFF_ALT",  PILOT_TKOFF_ALT_DEFAULT),

    // @Param: PILOT_THR_BHV
    // @DisplayName: Throttle stick behavior
    // @Description: Bitmask containing various throttle stick options. TX with sprung throttle can set PILOT_THR_BHV to "1" so motor feedback when landed starts from mid-stick instead of bottom of stick.
    // @User: Standard
    // @Values: 0:None,1:Feedback from mid stick,2:High throttle cancels landing,4:Disarm on land detection
    // @Bitmask: 0:Feedback from mid stick,1:High throttle cancels landing,2:Disarm on land detection
    GSCALAR(throttle_behavior, "PILOT_THR_BHV", 0),

    // @Group: SERIAL
    // @Path: ../libraries/AP_SerialManager/AP_SerialManager.cpp
    GOBJECT(serial_manager, "SERIAL",   AP_SerialManager),

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Advanced
    // @Units: s
    // @Range: 0 30
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: GCS_PID_MASK
    // @DisplayName: GCS PID tuning mask
    // @Description: bitmask of PIDs to send MAVLink PID_TUNING messages for
    // @User: Advanced
    // @Values: 0:None,1:Roll,2:Pitch,4:Yaw,8:AccelZ
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw,3:AccelZ
    GSCALAR(gcs_pid_mask,           "GCS_PID_MASK",     0),

#if MODE_RTL_ENABLED == ENABLED
    // @Param: RTL_ALT
    // @DisplayName: RTL Altitude
    // @Description: The minimum alt above home the vehicle will climb to before returning.  If the vehicle is flying higher than this value it will return at its current altitude.
    // @Units: cm
    // @Range: 200 300000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_altitude,   "RTL_ALT",     RTL_ALT),

    // @Param: RTL_CONE_SLOPE
    // @DisplayName: RTL cone slope
    // @Description: Defines a cone above home which determines maximum climb
    // @Range: 0.5 10.0
    // @Increment: .1
    // @Values: 0:Disabled,1:Shallow,3:Steep
    // @User: Standard
    GSCALAR(rtl_cone_slope,   "RTL_CONE_SLOPE",     RTL_CONE_SLOPE_DEFAULT),

    // @Param: RTL_SPEED
    // @DisplayName: RTL speed
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally while flying home. If this is set to zero, WPNAV_SPEED will be used instead.
    // @Units: cm/s
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    GSCALAR(rtl_speed_cms,   "RTL_SPEED",     0),

    // @Param: RTL_ALT_FINAL
    // @DisplayName: RTL Final Altitude
    // @Description: This is the altitude the vehicle will move to as the final stage of Returning to Launch or after completing a mission.  Set to zero to land.
    // @Units: cm
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_alt_final,  "RTL_ALT_FINAL", RTL_ALT_FINAL),

    // @Param: RTL_CLIMB_MIN
    // @DisplayName: RTL minimum climb
    // @Description: The vehicle will climb this many cm during the initial climb portion of the RTL
    // @Units: cm
    // @Range: 0 3000
    // @Increment: 10
    // @User: Standard
    GSCALAR(rtl_climb_min,  "RTL_CLIMB_MIN",    RTL_CLIMB_MIN_DEFAULT),

    // @Param: RTL_LOIT_TIME
    // @DisplayName: RTL loiter time
    // @Description: Time (in milliseconds) to loiter above home before beginning final descent
    // @Units: ms
    // @Range: 0 60000
    // @Increment: 1000
    // @User: Standard
    GSCALAR(rtl_loiter_time,      "RTL_LOIT_TIME",    RTL_LOITER_TIME),

    // @Param: RTL_ALT_TYPE
    // @DisplayName: RTL mode altitude type
    // @Description: RTL altitude type.  Set to 1 for Terrain following during RTL and then set WPNAV_RFND_USE=1 to use rangefinder or WPNAV_RFND_USE=0 to use Terrain database
    // @Values: 0:Relative to Home, 1:Terrain
    // @User: Standard
    GSCALAR(rtl_alt_type, "RTL_ALT_TYPE", 0),
#endif

    // @Param: FS_GCS_ENABLE
    // @DisplayName: Ground Station Failsafe Enable
    // @Description: Controls whether failsafe will be invoked (and what action to take) when connection with Ground station is lost for at least 5 seconds. See FS_OPTIONS param for additional actions, or for cases allowing Mission continuation, when GCS failsafe is enabled.
    // @Values: 0:Disabled/NoAction,1:RTL,2:RTL or Continue with Mission in Auto Mode (Removed in 4.0+-see FS_OPTIONS),3:SmartRTL or RTL,4:SmartRTL or Land,5:Land,6:Auto DO_LAND_START or RTL
    // @User: Standard
    GSCALAR(failsafe_gcs, "FS_GCS_ENABLE", FS_GCS_DISABLED),

    // @Param: GPS_HDOP_GOOD
    // @DisplayName: GPS Hdop Good
    // @Description: GPS Hdop value at or below this value represent a good position.  Used for pre-arm checks
    // @Range: 100 900
    // @User: Advanced
    GSCALAR(gps_hdop_good, "GPS_HDOP_GOOD", GPS_HDOP_GOOD_DEFAULT),

    // @Param: SUPER_SIMPLE
    // @DisplayName: Super Simple Mode
    // @Description: Bitmask to enable Super Simple mode for some flight modes. Setting this to Disabled(0) will disable Super Simple Mode. The bitmask is for flight mode switch positions
    // @Bitmask: 0:SwitchPos1, 1:SwitchPos2, 2:SwitchPos3, 3:SwitchPos4, 4:SwitchPos5, 5:SwitchPos6
    // @User: Standard
    GSCALAR(super_simple,   "SUPER_SIMPLE",     0),

    // @Param: WP_YAW_BEHAVIOR
    // @DisplayName: Yaw behaviour during missions
    // @Description: Determines how the autopilot controls the yaw during missions and RTL
    // @Values: 0:Never change yaw, 1:Face next waypoint, 2:Face next waypoint except RTL, 3:Face along GPS course
    // @User: Standard
    GSCALAR(wp_yaw_behavior,  "WP_YAW_BEHAVIOR",    WP_YAW_BEHAVIOR_DEFAULT),

    // @Param: LAND_SPEED
    // @DisplayName: Land speed
    // @Description: The descent speed for the final stage of landing in cm/s
    // @Units: cm/s
    // @Range: 30 200
    // @Increment: 10
    // @User: Standard
    GSCALAR(land_speed,             "LAND_SPEED",   LAND_SPEED),

    // @Param: LAND_SPEED_HIGH
    // @DisplayName: Land speed high
    // @Description: The descent speed for the first stage of landing in cm/s. If this is zero then WPNAV_SPEED_DN is used
    // @Units: cm/s
    // @Range: 0 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(land_speed_high,        "LAND_SPEED_HIGH",   0),
    
    // @Param: PILOT_SPEED_UP
    // @DisplayName: Pilot maximum vertical speed ascending
    // @Description: The maximum vertical ascending velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_speed_up,     "PILOT_SPEED_UP",   PILOT_VELZ_MAX),

    // @Param: PILOT_ACCEL_Z
    // @DisplayName: Pilot vertical acceleration
    // @Description: The vertical acceleration used when pilot is controlling the altitude
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    GSCALAR(pilot_accel_z,  "PILOT_ACCEL_Z",    PILOT_ACCEL_Z_DEFAULT),

    // @Param: FS_THR_ENABLE
    // @DisplayName: Throttle Failsafe Enable
    // @Description: The throttle failsafe allows you to configure a software failsafe activated by a setting on the throttle input channel
    // @Values:  0:Disabled,1:Enabled always RTL,2:Enabled Continue with Mission in Auto Mode (Removed in 4.0+),3:Enabled always Land,4:Enabled always SmartRTL or RTL,5:Enabled always SmartRTL or Land,6:Enabled Auto DO_LAND_START or RTL
    // @User: Standard
    GSCALAR(failsafe_throttle,  "FS_THR_ENABLE",   FS_THR_ENABLED_ALWAYS_RTL),

    // @Param: FS_THR_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level in microseconds on channel 3 below which throttle failsafe triggers
    // @Range: 910 1100
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    GSCALAR(failsafe_throttle_value, "FS_THR_VALUE",      FS_THR_VALUE_DEFAULT),

    // @Param: THR_DZ
    // @DisplayName: Throttle deadzone
    // @Description: The deadzone above and below mid throttle in PWM microseconds. Used in AltHold, Loiter, PosHold flight modes
    // @User: Standard
    // @Range: 0 300
    // @Units: PWM
    // @Increment: 1
    GSCALAR(throttle_deadzone,  "THR_DZ",    THR_DZ_DEFAULT),

    // @Param: FLTMODE1
    // @DisplayName: Flight Mode 1
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is <= 1230
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:Smart_RTL,22:FlowHold,23:Follow,24:ZigZag,25:SystemID,26:Heli_Autorotate,27:Auto RTL
    // @User: Standard
    GSCALAR(flight_mode1, "FLTMODE1",               (uint8_t)FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: Flight Mode 2
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1230, <= 1360
    // @CopyValuesFrom: FLTMODE1
    // @User: Standard
    GSCALAR(flight_mode2, "FLTMODE2",               (uint8_t)FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: Flight Mode 3
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1360, <= 1490
    // @CopyValuesFrom: FLTMODE1
    // @User: Standard
    GSCALAR(flight_mode3, "FLTMODE3",               (uint8_t)FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: Flight Mode 4
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1490, <= 1620
    // @CopyValuesFrom: FLTMODE1
    // @User: Standard
    GSCALAR(flight_mode4, "FLTMODE4",               (uint8_t)FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: Flight Mode 5
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >1620, <= 1749
    // @CopyValuesFrom: FLTMODE1
    // @User: Standard
    GSCALAR(flight_mode5, "FLTMODE5",               (uint8_t)FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: Flight Mode 6
    // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is >=1750
    // @CopyValuesFrom: FLTMODE1
    // @User: Standard
    GSCALAR(flight_mode6, "FLTMODE6",               (uint8_t)FLIGHT_MODE_6),

    // @Param: FLTMODE_CH
    // @DisplayName: Flightmode channel
    // @Description: RC Channel to use for flight mode control
    // @Values: 0:Disabled,5:Channel5,6:Channel6,7:Channel7,8:Channel8,9:Channel9,10:Channel 10,11:Channel 11,12:Channel 12,13:Channel 13,14:Channel 14,15:Channel 15
    // @User: Advanced
    GSCALAR(flight_mode_chan, "FLTMODE_CH",         CH_MODE_DEFAULT),

    // @Param: INITIAL_MODE
    // @DisplayName: Initial flight mode
    // @Description: This selects the mode to start in on boot. This is useful for when you want to start in AUTO mode on boot without a receiver.
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:Smart_RTL,22:FlowHold,23:Follow,24:ZigZag,25:SystemID,26:Heli_Autorotate
    // @User: Advanced
    GSCALAR(initial_mode,        "INITIAL_MODE",     (uint8_t)Mode::Number::STABILIZE),

    // @Param: SIMPLE
    // @DisplayName: Simple mode bitmask
    // @Description: Bitmask which holds which flight modes use simple heading mode (eg bit 0 = 1 means Flight Mode 0 uses simple mode). The bitmask is for flightmode switch positions.
    // @Bitmask: 0:SwitchPos1, 1:SwitchPos2, 2:SwitchPos3, 3:SwitchPos4, 4:SwitchPos5, 5:SwitchPos6
    // @User: Advanced
    GSCALAR(simple_modes, "SIMPLE",                 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: 4 byte bitmap of log types to enable
    // @Bitmask: 0:ATTITUDE_FAST,1:ATTITUDE_MED,2:GPS,3:PM,4:CTUN,5:NTUN,6:RCIN,7:IMU,8:CMD,9:CURRENT,10:RCOUT,11:OPTFLOW,12:PID,13:COMPASS,14:INAV,15:CAMERA,17:MOTBATT,18:IMU_FAST,19:IMU_RAW,20:VideoStabilization
    // @User: Standard
    GSCALAR(log_bitmask,    "LOG_BITMASK",          DEFAULT_LOG_BITMASK),

    // @Param: ESC_CALIBRATION
    // @DisplayName: ESC Calibration
    // @Description: Controls whether ArduCopter will enter ESC calibration on the next restart.  Do not adjust this parameter manually.
    // @User: Advanced
    // @Values: 0:Normal Start-up, 1:Start-up in ESC Calibration mode if throttle high, 2:Start-up in ESC Calibration mode regardless of throttle, 3:Start-up and automatically calibrate ESCs, 9:Disabled
    GSCALAR(esc_calibrate, "ESC_CALIBRATION",       0),

    // @Param: TUNE
    // @DisplayName: Channel 6 Tuning
    // @Description: Controls which parameters (normally PID gains) are being tuned with transmitter's channel 6 knob
    // @User: Standard
    // @Values: 0:None,1:Stab Roll/Pitch kP,4:Rate Roll/Pitch kP,5:Rate Roll/Pitch kI,21:Rate Roll/Pitch kD,3:Stab Yaw kP,6:Rate Yaw kP,26:Rate Yaw kD,56:Rate Yaw Filter,55:Motor Yaw Headroom,14:AltHold kP,7:Throttle Rate kP,34:Throttle Accel kP,35:Throttle Accel kI,36:Throttle Accel kD,12:Loiter Pos kP,22:Velocity XY kP,28:Velocity XY kI,10:WP Speed,25:Acro Roll/Pitch deg/s,40:Acro Yaw deg/s,45:RC Feel,13:Heli Ext Gyro,38:Declination,39:Circle Rate,46:Rate Pitch kP,47:Rate Pitch kI,48:Rate Pitch kD,49:Rate Roll kP,50:Rate Roll kI,51:Rate Roll kD,52:Rate Pitch FF,53:Rate Roll FF,54:Rate Yaw FF,58:SysID Magnitude
    GSCALAR(radio_tuning, "TUNE",                   0),

    // @Param: FRAME_TYPE
    // @DisplayName: Frame Type (+, X, V, etc)
    // @Description: Controls motor mixing for multicopters.  Not used for Tri or Traditional Helicopters.
    // @Values: 0:Plus, 1:X, 2:V, 3:H, 4:V-Tail, 5:A-Tail, 10:Y6B, 11:Y6F, 12:BetaFlightX, 13:DJIX, 14:ClockwiseX, 15: I, 18: BetaFlightXReversed, 19:Y4
    // @User: Standard
    // @RebootRequired: True
    GSCALAR(frame_type, "FRAME_TYPE", HAL_FRAME_TYPE_DEFAULT),

    // @Group: ARMING_
    // @Path: ../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming,                 "ARMING_", AP_Arming_Copter),

    // @Param: DISARM_DELAY
    // @DisplayName: Disarm delay
    // @Description: Delay before automatic disarm in seconds. A value of zero disables auto disarm.
    // @Units: s
    // @Range: 0 127
    // @User: Advanced
    GSCALAR(disarm_delay, "DISARM_DELAY",           AUTO_DISARMING_DELAY),
    
    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all flight modes
    // @Units: cdeg
    // @Increment: 10
    // @Range: 1000 8000
    // @User: Advanced
    ASCALAR(angle_max, "ANGLE_MAX",                 DEFAULT_ANGLE_MAX),

#if MODE_POSHOLD_ENABLED == ENABLED
    // @Param: PHLD_BRAKE_RATE
    // @DisplayName: PosHold braking rate
    // @Description: PosHold flight mode's rotation rate during braking in deg/sec
    // @Units: deg/s
    // @Range: 4 12
    // @User: Advanced
    GSCALAR(poshold_brake_rate, "PHLD_BRAKE_RATE",  POSHOLD_BRAKE_RATE_DEFAULT),

    // @Param: PHLD_BRAKE_ANGLE
    // @DisplayName: PosHold braking angle max
    // @Description: PosHold flight mode's max lean angle during braking in centi-degrees
    // @Units: cdeg
    // @Increment: 10
    // @Range: 2000 4500
    // @User: Advanced
    GSCALAR(poshold_brake_angle_max, "PHLD_BRAKE_ANGLE",  POSHOLD_BRAKE_ANGLE_DEFAULT),
#endif

    // @Param: LAND_REPOSITION
    // @DisplayName: Land repositioning
    // @Description: Enables user input during LAND mode, the landing phase of RTL, and auto mode landings.
    // @Values: 0:No repositioning, 1:Repositioning
    // @User: Advanced
    GSCALAR(land_repositioning, "LAND_REPOSITION",     LAND_REPOSITION_DEFAULT),

    // @Param: FS_EKF_ACTION
    // @DisplayName: EKF Failsafe Action
    // @Description: Controls the action that will be taken when an EKF failsafe is invoked
    // @Values: 1:Land, 2:AltHold, 3:Land even in Stabilize
    // @User: Advanced
    GSCALAR(fs_ekf_action, "FS_EKF_ACTION",    FS_EKF_ACTION_DEFAULT),

    // @Param: FS_EKF_THRESH
    // @DisplayName: EKF failsafe variance threshold
    // @Description: Allows setting the maximum acceptable compass and velocity variance
    // @Values: 0.6:Strict, 0.8:Default, 1.0:Relaxed
    // @User: Advanced
    GSCALAR(fs_ekf_thresh, "FS_EKF_THRESH",    FS_EKF_THRESHOLD_DEFAULT),

    // @Param: FS_CRASH_CHECK
    // @DisplayName: Crash check enable
    // @Description: This enables automatic crash checking. When enabled the motors will disarm if a crash is detected.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    GSCALAR(fs_crash_check, "FS_CRASH_CHECK",    1),

    // @Param: RC_SPEED
    // @DisplayName: ESC Update Speed
    // @Description: This is the speed in Hertz that your ESCs will receive updates
    // @Units: Hz
    // @Range: 50 490
    // @Increment: 1
    // @User: Advanced
    GSCALAR(rc_speed, "RC_SPEED",              RC_FAST_SPEED),

    // @Param: ACRO_RP_P
    // @DisplayName: Acro Roll and Pitch P gain
    // @Description: Converts pilot roll and pitch into a desired rate of rotation in ACRO and SPORT mode.  Higher values mean faster rate of rotation.
    // @Range: 1 10
    // @User: Standard

    // @Param: ACRO_YAW_P
    // @DisplayName: Acro Yaw P gain
    // @Description: Converts pilot yaw input into a desired rate of rotation.  Higher values mean faster rate of rotation.
    // @Range: 1 10
    // @User: Standard

#if MODE_ACRO_ENABLED == ENABLED || MODE_SPORT_ENABLED == ENABLED
    // @Param: ACRO_BAL_ROLL
    // @DisplayName: Acro Balance Roll
    // @Description: rate at which roll angle returns to level in acro and sport mode.  A higher value causes the vehicle to return to level faster. For helicopter sets the decay rate of the virtual flybar in the roll axis. A higher value causes faster decay of desired to actual attitude.
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(acro_balance_roll,      "ACRO_BAL_ROLL",    ACRO_BALANCE_ROLL),

    // @Param: ACRO_BAL_PITCH
    // @DisplayName: Acro Balance Pitch
    // @Description: rate at which pitch angle returns to level in acro and sport mode.  A higher value causes the vehicle to return to level faster. For helicopter sets the decay rate of the virtual flybar in the pitch axis. A higher value causes faster decay of desired to actual attitude.
    // @Range: 0 3
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(acro_balance_pitch,     "ACRO_BAL_PITCH",   ACRO_BALANCE_PITCH),
#endif

#if MODE_ACRO_ENABLED == ENABLED
    // @Param: ACRO_TRAINER
    // @DisplayName: Acro Trainer
    // @Description: Type of trainer used in acro mode
    // @Values: 0:Disabled,1:Leveling,2:Leveling and Limited
    // @User: Advanced
    GSCALAR(acro_trainer,   "ACRO_TRAINER",     (uint8_t)ModeAcro::Trainer::LIMITED),

    // @Param: ACRO_RP_EXPO
    // @DisplayName: Acro Roll/Pitch Expo
    // @Description: Acro roll/pitch Expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -0.5 0.95
    // @User: Advanced
    GSCALAR(acro_rp_expo,  "ACRO_RP_EXPO",    ACRO_RP_EXPO_DEFAULT),
#endif

    // variables not in the g class which contain EEPROM saved variables

#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera,           "CAM_", AP_Camera),
#endif

    // @Group: RELAY_
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY_", AP_Relay),

#if PARACHUTE == ENABLED
    // @Group: CHUTE_
    // @Path: ../libraries/AP_Parachute/AP_Parachute.cpp
    GOBJECT(parachute, "CHUTE_", AP_Parachute),
#endif

#if LANDING_GEAR_ENABLED == ENABLED
    // @Group: LGR_
    // @Path: ../libraries/AP_LandingGear/AP_LandingGear.cpp
    GOBJECT(landinggear,    "LGR_", AP_LandingGear),
#endif

#if FRAME_CONFIG == HELI_FRAME
    // @Group: IM_
    // @Path: ../libraries/AC_InputManager/AC_InputManager_Heli.cpp
    GOBJECT(input_manager, "IM_", AC_InputManager_Heli),
#endif

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,        "COMPASS_", Compass),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,            "INS_", AP_InertialSensor),

    // @Group: WPNAV_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    GOBJECTPTR(wp_nav, "WPNAV_",       AC_WPNav),

    // @Group: LOIT_
    // @Path: ../libraries/AC_WPNav/AC_Loiter.cpp
    GOBJECTPTR(loiter_nav, "LOIT_", AC_Loiter),

#if MODE_CIRCLE_ENABLED == ENABLED
    // @Group: CIRCLE_
    // @Path: ../libraries/AC_WPNav/AC_Circle.cpp
    GOBJECTPTR(circle_nav, "CIRCLE_",  AC_Circle),
#endif

    // @Group: ATC_
    // @Path: ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp,../libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp,../libraries/AC_AttitudeControl/AC_AttitudeControl_Heli.cpp
#if FRAME_CONFIG == HELI_FRAME
    GOBJECTPTR(attitude_control, "ATC_", AC_AttitudeControl_Heli),
#else
    GOBJECTPTR(attitude_control, "ATC_", AC_AttitudeControl_Multi),
#endif

    // @Group: PSC
    // @Path: ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    GOBJECTPTR(pos_control, "PSC", AC_PosControl),

    // @Group: SR0_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[0],  gcs0,       "SR0_",     GCS_MAVLINK_Parameters),

#if MAVLINK_COMM_NUM_BUFFERS >= 2
    // @Group: SR1_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[1],  gcs1,       "SR1_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 3
    // @Group: SR2_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[2],  gcs2,       "SR2_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 4
    // @Group: SR3_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[3],  gcs3,       "SR3_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 5
    // @Group: SR4_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[4],  gcs4,       "SR4_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 6
    // @Group: SR5_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[5],  gcs5,       "SR5_",     GCS_MAVLINK_Parameters),
#endif

#if MAVLINK_COMM_NUM_BUFFERS >= 7
    // @Group: SR6_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs.chan_parameters[6],  gcs6,       "SR6_",     GCS_MAVLINK_Parameters),
#endif

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if HAL_MOUNT_ENABLED
    // @Group: MNT
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT",  AP_Mount),
#endif

    // @Group: LOG
    // @Path: ../libraries/AP_Logger/AP_Logger.cpp
    GOBJECT(logger,           "LOG",  AP_Logger),

    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT",         AP_BattMonitor),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    // @Group: CAN_
    // @Path: ../libraries/AP_CANManager/AP_CANManager.cpp
    GOBJECT(can_mgr,        "CAN_",       AP_CANManager),
#endif

#if SPRAYER_ENABLED == ENABLED
    // @Group: SPRAY_
    // @Path: ../libraries/AC_Sprayer/AC_Sprayer.cpp
    GOBJECT(sprayer,                "SPRAY_",       AC_Sprayer),
#endif

#if AP_SIM_ENABLED
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL::SIM),
#endif

    // @Group: BARO
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "BARO", AP_Baro),

    // GPS driver
    // @Group: GPS
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS", AP_GPS),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

#if AC_FENCE == ENABLED
    // @Group: FENCE_
    // @Path: ../libraries/AC_Fence/AC_Fence.cpp
    GOBJECT(fence,      "FENCE_",   AC_Fence),
#endif

    // @Group: AVOID_
    // @Path: ../libraries/AC_Avoidance/AC_Avoid.cpp
#if AC_AVOID_ENABLED == ENABLED
    GOBJECT(avoid,      "AVOID_",   AC_Avoid),
#endif

#if AC_RALLY == ENABLED
    // @Group: RALLY_
    // @Path: AP_Rally.cpp,../libraries/AP_Rally/AP_Rally.cpp
    GOBJECT(rally,      "RALLY_",   AP_Rally_Copter),
#endif

#if FRAME_CONFIG == HELI_FRAME
    // @Group: H_
    // @Path: ../libraries/AP_Motors/AP_MotorsHeli_Single.cpp,../libraries/AP_Motors/AP_MotorsHeli_Dual.cpp,../libraries/AP_Motors/AP_MotorsHeli.cpp
    GOBJECTVARPTR(motors, "H_",        &copter.motors_var_info),
#else
    // @Group: MOT_
    // @Path: ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    GOBJECTVARPTR(motors, "MOT_",      &copter.motors_var_info),
#endif

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap, "RCMAP_",        RCMapper),

#if HAL_NAVEKF2_AVAILABLE
    // @Group: EK2_
    // @Path: ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    GOBJECTN(ahrs.EKF2, NavEKF2, "EK2_", NavEKF2),
#endif

#if HAL_NAVEKF3_AVAILABLE
    // @Group: EK3_
    // @Path: ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
    GOBJECTN(ahrs.EKF3, NavEKF3, "EK3_", NavEKF3),
#endif

#if MODE_AUTO_ENABLED == ENABLED
    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECTN(mode_auto.mission, mission, "MIS_", AP_Mission),
#endif

    // @Group: RSSI_
    // @Path: ../libraries/AP_RSSI/AP_RSSI.cpp
    GOBJECT(rssi, "RSSI_",  AP_RSSI),
    
#if RANGEFINDER_ENABLED == ENABLED
    // @Group: RNGFND
    // @Path: ../libraries/AP_RangeFinder/AP_RangeFinder.cpp
    GOBJECT(rangefinder,   "RNGFND", RangeFinder),
#endif

#if AP_TERRAIN_AVAILABLE
    // @Group: TERRAIN_
    // @Path: ../libraries/AP_Terrain/AP_Terrain.cpp
    GOBJECT(terrain,                "TERRAIN_", AP_Terrain),
#endif

#if AP_OPTICALFLOW_ENABLED
    // @Group: FLOW
    // @Path: ../libraries/AP_OpticalFlow/AP_OpticalFlow.cpp
    GOBJECT(optflow,   "FLOW", OpticalFlow),
#endif

#if PRECISION_LANDING == ENABLED
    // @Group: PLND_
    // @Path: ../libraries/AC_PrecLand/AC_PrecLand.cpp
    GOBJECT(precland, "PLND_", AC_PrecLand),
#endif

#if RPM_ENABLED == ENABLED
    // @Group: RPM
    // @Path: ../libraries/AP_RPM/AP_RPM.cpp
    GOBJECT(rpm_sensor, "RPM", AP_RPM),
#endif

#if HAL_ADSB_ENABLED
    // @Group: ADSB_
    // @Path: ../libraries/AP_ADSB/AP_ADSB.cpp
    GOBJECT(adsb,                "ADSB_", AP_ADSB),

    // @Group: AVD_
    // @Path: ../libraries/AP_Avoidance/AP_Avoidance.cpp
    GOBJECT(avoidance_adsb, "AVD_", AP_Avoidance_Copter),
#endif

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),

#if MODE_THROW_ENABLED == ENABLED
    // @Param: THROW_MOT_START
    // @DisplayName: Start motors before throwing is detected
    // @Description: Used by Throw mode. Controls whether motors will run at the speed set by MOT_SPIN_MIN or will be stopped when armed and waiting for the throw.
    // @Values: 0:Stopped,1:Running
    // @User: Standard
    GSCALAR(throw_motor_start, "THROW_MOT_START", (float)ModeThrow::PreThrowMotorState::STOPPED),
#endif

#if OSD_ENABLED || OSD_PARAM_ENABLED
    // @Group: OSD
    // @Path: ../libraries/AP_OSD/AP_OSD.cpp
    GOBJECT(osd, "OSD", AP_OSD),
#endif

    // @Group:
    // @Path: Parameters.cpp
    GOBJECT(g2, "",  ParametersG2),

    // @Group:
    // @Path: ../libraries/AP_Vehicle/AP_Vehicle.cpp
    { AP_PARAM_GROUP, "", Parameters::k_param_vehicle, (const void *)&copter, {group_info : AP_Vehicle::var_info} },

    AP_VAREND
};

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {

    // @Param: WP_NAVALT_MIN
    // @DisplayName: Minimum navigation altitude
    // @Description: This is the altitude in meters above which for navigation can begin. This applies in auto takeoff and auto landing.
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("WP_NAVALT_MIN", 1, ParametersG2, wp_navalt_min, 0),

#if HAL_BUTTON_ENABLED
    // @Group: BTN_
    // @Path: ../libraries/AP_Button/AP_Button.cpp
    AP_SUBGROUPPTR(button_ptr, "BTN_", 2, ParametersG2, AP_Button),
#endif

#if MODE_THROW_ENABLED == ENABLED
    // @Param: THROW_NEXTMODE
    // @DisplayName: Throw mode's follow up mode
    // @Description: Vehicle will switch to this mode after the throw is successfully completed.  Default is to stay in throw mode (18)
    // @Values: 3:Auto,4:Guided,5:LOITER,6:RTL,9:Land,17:Brake,18:Throw
    // @User: Standard
    AP_GROUPINFO("THROW_NEXTMODE", 3, ParametersG2, throw_nextmode, 18),

    // @Param: THROW_TYPE
    // @DisplayName: Type of Type
    // @Description: Used by Throw mode. Specifies whether Copter is thrown upward or dropped.
    // @Values: 0:Upward Throw,1:Drop
    // @User: Standard
    AP_GROUPINFO("THROW_TYPE", 4, ParametersG2, throw_type, (float)ModeThrow::ThrowType::Upward),
#endif

    // @Param: GND_EFFECT_COMP
    // @DisplayName: Ground Effect Compensation Enable/Disable
    // @Description: Ground Effect Compensation Enable/Disable
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("GND_EFFECT_COMP", 5, ParametersG2, gndeffect_comp_enabled, 1),

#if ADVANCED_FAILSAFE == ENABLED
    // @Group: AFS_
    // @Path: ../libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp
    AP_SUBGROUPINFO(afs, "AFS_", 6, ParametersG2, AP_AdvancedFailsafe),
#endif

    // @Param: DEV_OPTIONS
    // @DisplayName: Development options
    // @Description: Bitmask of developer options. The meanings of the bit fields in this parameter may vary at any time. Developers should check the source code for current meaning
    // @Bitmask: 0:ADSBMavlinkProcessing,1:DevOptionVFR_HUDRelativeAlt
    // @User: Advanced
    AP_GROUPINFO("DEV_OPTIONS", 7, ParametersG2, dev_options, 0),

#if BEACON_ENABLED == ENABLED
    // @Group: BCN
    // @Path: ../libraries/AP_Beacon/AP_Beacon.cpp
    AP_SUBGROUPINFO(beacon, "BCN", 14, ParametersG2, AP_Beacon),
#endif

#if HAL_PROXIMITY_ENABLED
    // @Group: PRX
    // @Path: ../libraries/AP_Proximity/AP_Proximity.cpp
    AP_SUBGROUPINFO(proximity, "PRX", 8, ParametersG2, AP_Proximity),
#endif

    // @Param: ACRO_Y_EXPO
    // @DisplayName: Acro Yaw Expo
    // @Description: Acro yaw expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -1.0 0.95
    // @User: Advanced
    AP_GROUPINFO("ACRO_Y_EXPO", 9, ParametersG2, acro_y_expo, ACRO_Y_EXPO_DEFAULT),

#if MODE_ACRO_ENABLED == ENABLED
    // @Param: ACRO_THR_MID
    // @DisplayName: Acro Thr Mid
    // @Description: Acro Throttle Mid
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("ACRO_THR_MID", 10, ParametersG2, acro_thr_mid, ACRO_THR_MID_DEFAULT),
#endif

    // @Param: SYSID_ENFORCE
    // @DisplayName: GCS sysid enforcement
    // @Description: This controls whether packets from other than the expected GCS system ID will be accepted
    // @Values: 0:NotEnforced,1:Enforced
    // @User: Advanced
    AP_GROUPINFO("SYSID_ENFORCE", 11, ParametersG2, sysid_enforce, 0),

#if STATS_ENABLED == ENABLED
    // @Group: STAT
    // @Path: ../libraries/AP_Stats/AP_Stats.cpp
    AP_SUBGROUPINFO(stats, "STAT", 12, ParametersG2, AP_Stats),
#endif

#if GRIPPER_ENABLED == ENABLED
    // @Group: GRIP_
    // @Path: ../libraries/AP_Gripper/AP_Gripper.cpp
    AP_SUBGROUPINFO(gripper, "GRIP_", 13, ParametersG2, AP_Gripper),
#endif

    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Controls major frame class for multicopter component
    // @Values: 0:Undefined, 1:Quad, 2:Hexa, 3:Octa, 4:OctaQuad, 5:Y6, 6:Heli, 7:Tri, 8:SingleCopter, 9:CoaxCopter, 10:BiCopter, 11:Heli_Dual, 12:DodecaHexa, 13:HeliQuad, 14:Deca, 15:Scripting Matrix, 16:6DoF Scripting, 17:Dynamic Scripting Matrix
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("FRAME_CLASS", 15, ParametersG2, frame_class, DEFAULT_FRAME_CLASS),

    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    AP_SUBGROUPINFO(servo_channels, "SERVO", 16, ParametersG2, SRV_Channels),

    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    AP_SUBGROUPINFO(rc_channels, "RC", 17, ParametersG2, RC_Channels_Copter),

    // 18 was used by AP_VisualOdom

    // @Group: TCAL
    // @Path: ../libraries/AP_TempCalibration/AP_TempCalibration.cpp
    AP_SUBGROUPINFO(temp_calibration, "TCAL", 19, ParametersG2, AP_TempCalibration),

#if TOY_MODE_ENABLED == ENABLED
    // @Group: TMODE
    // @Path: toy_mode.cpp
    AP_SUBGROUPINFO(toy_mode, "TMODE", 20, ParametersG2, ToyMode),
#endif

#if MODE_SMARTRTL_ENABLED == ENABLED
    // @Group: SRTL_
    // @Path: ../libraries/AP_SmartRTL/AP_SmartRTL.cpp
    AP_SUBGROUPINFO(smart_rtl, "SRTL_", 21, ParametersG2, AP_SmartRTL),
#endif

#if WINCH_ENABLED == ENABLED
    // 22 was AP_WheelEncoder

    // @Group: WINCH
    // @Path: ../libraries/AP_Winch/AP_Winch.cpp
    AP_SUBGROUPINFO(winch, "WINCH", 23, ParametersG2, AP_Winch),
#endif

    // @Param: PILOT_SPEED_DN
    // @DisplayName: Pilot maximum vertical speed descending
    // @Description: The maximum vertical descending velocity the pilot may request in cm/s.  If 0 PILOT_SPEED_UP value is used.
    // @Units: cm/s
    // @Range: 0 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("PILOT_SPEED_DN", 24, ParametersG2, pilot_speed_dn, 0),

    // @Param: LAND_ALT_LOW
    // @DisplayName: Land alt low
    // @Description: Altitude during Landing at which vehicle slows to LAND_SPEED
    // @Units: cm
    // @Range: 100 10000
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("LAND_ALT_LOW", 25, ParametersG2, land_alt_low, 1000),

#if !HAL_MINIMIZE_FEATURES && AP_OPTICALFLOW_ENABLED
    // @Group: FHLD
    // @Path: mode_flowhold.cpp
    AP_SUBGROUPPTR(mode_flowhold_ptr, "FHLD", 26, ParametersG2, ModeFlowHold),
#endif

#if MODE_FOLLOW_ENABLED == ENABLED
    // @Group: FOLL
    // @Path: ../libraries/AP_Follow/AP_Follow.cpp
    AP_SUBGROUPINFO(follow, "FOLL", 27, ParametersG2, AP_Follow),
#endif

#ifdef USER_PARAMS_ENABLED
    AP_SUBGROUPINFO(user_parameters, "USR", 28, ParametersG2, UserParameters),
#endif

#if AUTOTUNE_ENABLED == ENABLED
    // @Group: AUTOTUNE_
    // @Path: ../libraries/AC_AutoTune/AC_AutoTune_Multi.cpp,../libraries/AC_AutoTune/AC_AutoTune_Heli.cpp
    AP_SUBGROUPPTR(autotune_ptr, "AUTOTUNE_",  29, ParametersG2, AutoTune),
#endif

#if AP_SCRIPTING_ENABLED
    // @Group: SCR_
    // @Path: ../libraries/AP_Scripting/AP_Scripting.cpp
    AP_SUBGROUPINFO(scripting, "SCR_", 30, ParametersG2, AP_Scripting),
#endif

    // @Param: TUNE_MIN
    // @DisplayName: Tuning minimum
    // @Description: Minimum value that the parameter currently being tuned with the transmitter's channel 6 knob will be set to
    // @User: Standard
    AP_GROUPINFO("TUNE_MIN", 31, ParametersG2, tuning_min, 0),

    // @Param: TUNE_MAX
    // @DisplayName: Tuning maximum
    // @Description: Maximum value that the parameter currently being tuned with the transmitter's channel 6 knob will be set to
    // @User: Standard
    AP_GROUPINFO("TUNE_MAX", 32, ParametersG2, tuning_max, 0),

#if AC_OAPATHPLANNER_ENABLED == ENABLED
    // @Group: OA_
    // @Path: ../libraries/AC_Avoidance/AP_OAPathPlanner.cpp
    AP_SUBGROUPINFO(oa, "OA_", 33, ParametersG2, AP_OAPathPlanner),
#endif

#if MODE_SYSTEMID_ENABLED == ENABLED
    // @Group: SID
    // @Path: mode_systemid.cpp
    AP_SUBGROUPPTR(mode_systemid_ptr, "SID", 34, ParametersG2, ModeSystemId),
#endif

    // @Param: FS_VIBE_ENABLE
    // @DisplayName: Vibration Failsafe enable
    // @Description: This enables the vibration failsafe which will use modified altitude estimation and control during high vibrations
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("FS_VIBE_ENABLE", 35, ParametersG2, fs_vibe_enabled, 1),

    // @Param: FS_OPTIONS
    // @DisplayName: Failsafe options bitmask
    // @Description: Bitmask of additional options for battery, radio, & GCS failsafes. 0 (default) disables all options.
    // @Values: 0:Disabled, 1:Continue if in Auto on RC failsafe only, 2:Continue if in Auto on GCS failsafe only, 3:Continue if in Auto on RC and/or GCS failsafe, 4:Continue if in Guided on RC failsafe only, 8:Continue if landing on any failsafe, 16:Continue if in pilot controlled modes on GCS failsafe, 19:Continue if in Auto on RC and/or GCS failsafe and continue if in pilot controlled modes on GCS failsafe
    // @Bitmask: 0:Continue if in Auto on RC failsafe, 1:Continue if in Auto on GCS failsafe, 2:Continue if in Guided on RC failsafe, 3:Continue if landing on any failsafe, 4:Continue if in pilot controlled modes on GCS failsafe, 5:Release Gripper
    // @User: Advanced
    AP_GROUPINFO("FS_OPTIONS", 36, ParametersG2, fs_options, (float)Copter::FailsafeOption::GCS_CONTINUE_IF_PILOT_CONTROL),

#if MODE_AUTOROTATE_ENABLED == ENABLED
    // @Group: AROT_
    // @Path: ../libraries/AC_Autorotation/AC_Autorotation.cpp
    AP_SUBGROUPINFO(arot, "AROT_", 37, ParametersG2, AC_Autorotation),
#endif

#if MODE_ZIGZAG_ENABLED == ENABLED
    // @Group: ZIGZ_
    // @Path: mode_zigzag.cpp
    AP_SUBGROUPPTR(mode_zigzag_ptr, "ZIGZ_", 38, ParametersG2, ModeZigZag),
#endif

#if MODE_ACRO_ENABLED == ENABLED
    // @Param: ACRO_OPTIONS
    // @DisplayName: Acro mode options
    // @Description: A range of options that can be applied to change acro mode behaviour. Air-mode enables ATC_THR_MIX_MAN at all times (air-mode has no effect on helicopters). Rate Loop Only disables the use of angle stabilization and uses angular rate stabilization only.
    // @Bitmask: 0:Air-mode,1:Rate Loop Only
    // @User: Advanced
    AP_GROUPINFO("ACRO_OPTIONS", 39, ParametersG2, acro_options, 0),
#endif

#if MODE_AUTO_ENABLED == ENABLED
    // @Param: AUTO_OPTIONS
    // @DisplayName: Auto mode options
    // @Description: A range of options that can be applied to change auto mode behaviour. Allow Arming allows the copter to be armed in Auto. Allow Takeoff Without Raising Throttle allows takeoff without the pilot having to raise the throttle. Ignore pilot yaw overrides the pilot's yaw stick being used while in auto.
    // @Bitmask: 0:Allow Arming,1:Allow Takeoff Without Raising Throttle,2:Ignore pilot yaw
    // @User: Advanced
    AP_GROUPINFO("AUTO_OPTIONS", 40, ParametersG2, auto_options, 0),
#endif

#if MODE_GUIDED_ENABLED == ENABLED
    // @Param: GUID_OPTIONS
    // @DisplayName: Guided mode options
    // @Description: Options that can be applied to change guided mode behaviour
    // @Bitmask: 0:Allow Arming from Transmitter,2:Ignore pilot yaw,3:SetAttitudeTarget interprets Thrust As Thrust,4:Do not stabilize PositionXY,5:Do not stabilize VelocityXY,6:Waypoint navigation used for position targets
    // @User: Advanced
    AP_GROUPINFO("GUID_OPTIONS", 41, ParametersG2, guided_options, 0),
#endif

    // @Param: FS_GCS_TIMEOUT
    // @DisplayName: GCS failsafe timeout
    // @Description: Timeout before triggering the GCS failsafe
    // @Units: s
    // @Range: 2 120
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FS_GCS_TIMEOUT", 42, ParametersG2, fs_gcs_timeout, 5),

#if MODE_RTL_ENABLED == ENABLED
    // @Param: RTL_OPTIONS
    // @DisplayName: RTL mode options
    // @Description: Options that can be applied to change RTL mode behaviour
    // @Bitmask: 2:Ignore pilot yaw
    // @User: Advanced
    AP_GROUPINFO("RTL_OPTIONS", 43, ParametersG2, rtl_options, 0),
#endif

    // @Param: FLIGHT_OPTIONS
    // @DisplayName: Flight mode options
    // @Description: Flight mode specific options
    // @Bitmask: 0:Disable thrust loss check, 1:Disable yaw imbalance warning, 2:Release gripper on thrust loss
    // @User: Advanced
    AP_GROUPINFO("FLIGHT_OPTIONS", 44, ParametersG2, flight_options, 0),

#if RANGEFINDER_ENABLED == ENABLED
    // @Param: RNGFND_FILT
    // @DisplayName: Rangefinder filter
    // @Description: Rangefinder filter to smooth distance.  Set to zero to disable filtering
    // @Units: Hz
    // @Range: 0 5
    // @Increment: 0.05
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("RNGFND_FILT", 45, ParametersG2, rangefinder_filt, RANGEFINDER_FILT_DEFAULT),
#endif

#if MODE_GUIDED_ENABLED == ENABLED
    // @Param: GUID_TIMEOUT
    // @DisplayName: Guided mode timeout
    // @Description: Guided mode timeout after which vehicle will stop or return to level if no updates are received from caller. Only applicable during any combination of velocity, acceleration, angle control, and/or angular rate control
    // @Units: s
    // @Range: 0.1 5
    // @User: Advanced
    AP_GROUPINFO("GUID_TIMEOUT", 46, ParametersG2, guided_timeout, 3.0),
#endif

#if MODE_ACRO_ENABLED == ENABLED || MODE_SPORT_ENABLED == ENABLED
    // @Param: ACRO_RP_RATE
    // @DisplayName: Acro Roll and Pitch Rate
    // @Description: Acro mode maximum roll and pitch rate.  Higher values mean faster rate of rotation
    // @Units: deg/s
    // @Range: 1 1080
    // @User: Standard
    AP_GROUPINFO("ACRO_RP_RATE", 47, ParametersG2, acro_rp_rate, ACRO_RP_RATE_DEFAULT),
#endif

#if MODE_ACRO_ENABLED == ENABLED || MODE_DRIFT_ENABLED == ENABLED
    // @Param: ACRO_Y_RATE
    // @DisplayName: Acro Yaw Rate
    // @Description: Acro mode maximum yaw rate.  Higher value means faster rate of rotation
    // @Units: deg/s
    // @Range: 1 360
    // @User: Standard
    AP_GROUPINFO("ACRO_Y_RATE", 48, ParametersG2, acro_y_rate, ACRO_Y_RATE_DEFAULT),
#endif

    // @Param: PILOT_Y_RATE
    // @DisplayName: Pilot controlled yaw rate
    // @Description: Pilot controlled yaw rate max.  Used in all pilot controlled modes except Acro
    // @Units: deg/s
    // @Range: 1 360
    // @User: Standard
    AP_GROUPINFO("PILOT_Y_RATE", 49, ParametersG2, pilot_y_rate, PILOT_Y_RATE_DEFAULT),

    // @Param: PILOT_Y_EXPO
    // @DisplayName: Pilot controlled yaw expo
    // @Description: Pilot controlled yaw expo to allow faster rotation when stick at edges
    // @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
    // @Range: -0.5 1.0
    // @User: Advanced
    AP_GROUPINFO("PILOT_Y_EXPO", 50, ParametersG2, pilot_y_expo, PILOT_Y_EXPO_DEFAULT),

    // @Param: SURFTRAK_MODE
    // @DisplayName: Surface Tracking Mode
    // @Description: set which surface to track in surface tracking
    // @Values: 0:Do not track, 1:Ground, 2:Ceiling
    // @User: Advanced
    AP_GROUPINFO("SURFTRAK_MODE", 51, ParametersG2, surftrak_mode, (uint8_t)Copter::SurfaceTracking::Surface::GROUND),

    AP_GROUPEND
};

/*
  constructor for g2 object
 */
ParametersG2::ParametersG2(void)
    : temp_calibration() // this doesn't actually need constructing, but removing it here is problematic syntax-wise
#if BEACON_ENABLED == ENABLED
    , beacon(copter.serial_manager)
#endif
#if HAL_PROXIMITY_ENABLED
    , proximity()
#endif
#if ADVANCED_FAILSAFE == ENABLED
    ,afs()
#endif
#if MODE_SMARTRTL_ENABLED == ENABLED
    ,smart_rtl()
#endif
#if !HAL_MINIMIZE_FEATURES && AP_OPTICALFLOW_ENABLED
    ,mode_flowhold_ptr(&copter.mode_flowhold)
#endif
#if MODE_FOLLOW_ENABLED == ENABLED
    ,follow()
#endif
#ifdef USER_PARAMS_ENABLED
    ,user_parameters()
#endif
#if AUTOTUNE_ENABLED == ENABLED
    ,autotune_ptr(&copter.mode_autotune.autotune)
#endif
#if MODE_SYSTEMID_ENABLED == ENABLED
    ,mode_systemid_ptr(&copter.mode_systemid)
#endif
#if MODE_AUTOROTATE_ENABLED == ENABLED
    ,arot()
#endif
#if HAL_BUTTON_ENABLED
    ,button_ptr(&copter.button)
#endif
#if MODE_ZIGZAG_ENABLED == ENABLED
    ,mode_zigzag_ptr(&copter.mode_zigzag)
#endif
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  This is a conversion table from old parameter values to new
  parameter names. The startup code looks for saved values of the old
  parameters and will copy them across to the new parameters if the
  new parameter does not yet have a saved value. It then saves the new
  value.

  Note that this works even if the old parameter has been removed. It
  relies on the old k_param index not being removed

  The second column below is the index in the var_info[] table for the
  old object. This should be zero for top level parameters.
 */
const AP_Param::ConversionInfo conversion_table[] = {
    // PARAMETER_CONVERSION - Added: Oct-2014
    { Parameters::k_param_log_bitmask_old,    0,      AP_PARAM_INT16, "LOG_BITMASK" },
    // PARAMETER_CONVERSION - Added: Jan-2015
    { Parameters::k_param_serial0_baud,       0,      AP_PARAM_INT16, "SERIAL0_BAUD" },
    { Parameters::k_param_serial1_baud,       0,      AP_PARAM_INT16, "SERIAL1_BAUD" },
    { Parameters::k_param_serial2_baud,       0,      AP_PARAM_INT16, "SERIAL2_BAUD" },
    // PARAMETER_CONVERSION - Added: Jan-2017
    { Parameters::k_param_arming_check_old,   0,      AP_PARAM_INT8,  "ARMING_CHECK" },
    // battery
    // PARAMETER_CONVERSION - Added: Mar-2018
    { Parameters::k_param_fs_batt_voltage,    0,      AP_PARAM_FLOAT,  "BATT_LOW_VOLT" },
    { Parameters::k_param_fs_batt_mah,        0,      AP_PARAM_FLOAT,  "BATT_LOW_MAH" },
    { Parameters::k_param_failsafe_battery_enabled,0, AP_PARAM_INT8,   "BATT_FS_LOW_ACT" },

    // PARAMETER_CONVERSION - Added: Aug-2018
    { Parameters::Parameters::k_param_ch7_option_old,   0,      AP_PARAM_INT8,  "RC7_OPTION" },
    { Parameters::Parameters::k_param_ch8_option_old,   0,      AP_PARAM_INT8,  "RC8_OPTION" },
    { Parameters::Parameters::k_param_ch9_option_old,   0,      AP_PARAM_INT8,  "RC9_OPTION" },
    { Parameters::Parameters::k_param_ch10_option_old,   0,      AP_PARAM_INT8,  "RC10_OPTION" },
    { Parameters::Parameters::k_param_ch11_option_old,   0,      AP_PARAM_INT8,  "RC11_OPTION" },
    { Parameters::Parameters::k_param_ch12_option_old,   0,      AP_PARAM_INT8,  "RC12_OPTION" },
    // PARAMETER_CONVERSION - Added: Apr-2019
    { Parameters::k_param_compass_enabled_deprecated,    0,      AP_PARAM_INT8, "COMPASS_ENABLE" },
    // PARAMETER_CONVERSION - Added: Jul-2019
    { Parameters::k_param_arming,             2,     AP_PARAM_INT16,  "ARMING_CHECK" },
};

void Copter::load_parameters(void)
{
    if (!AP_Param::check_var_info()) {
        hal.console->printf("Bad var table\n");
        AP_HAL::panic("Bad var table");
    }

    hal.util->set_soft_armed(false);

    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        hal.console->printf("Firmware change: erasing EEPROM...\n");
        StorageManager::erase();
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        hal.console->printf("done.\n");
    }

    uint32_t before = micros();
    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
    AP_Param::convert_old_parameters(&conversion_table[0], ARRAY_SIZE(conversion_table));

#if LANDING_GEAR_ENABLED == ENABLED
    // convert landing gear parameters
    // PARAMETER_CONVERSION - Added: Nov-2018
    convert_lgr_parameters();
#endif

    // convert fs_options parameters
    // PARAMETER_CONVERSION - Added: Nov-2019
    convert_fs_options_params();

#if MODE_RTL_ENABLED == ENABLED
    // PARAMETER_CONVERSION - Added: Sep-2021
    g.rtl_altitude.convert_parameter_width(AP_PARAM_INT16);
#endif

    hal.console->printf("load_all took %uus\n", (unsigned)(micros() - before));

    // setup AP_Param frame type flags
    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_COPTER);

}

// handle conversion of PID gains
void Copter::convert_pid_parameters(void)
{
    // conversion info
    const AP_Param::ConversionInfo pid_conversion_info[] = {
        // PARAMETER_CONVERSION - Added: Apr-2016
        { Parameters::k_param_pid_rate_roll, 0, AP_PARAM_FLOAT, "ATC_RAT_RLL_P" },
        { Parameters::k_param_pid_rate_roll, 1, AP_PARAM_FLOAT, "ATC_RAT_RLL_I" },
        { Parameters::k_param_pid_rate_roll, 2, AP_PARAM_FLOAT, "ATC_RAT_RLL_D" },
        { Parameters::k_param_pid_rate_pitch, 0, AP_PARAM_FLOAT, "ATC_RAT_PIT_P" },
        { Parameters::k_param_pid_rate_pitch, 1, AP_PARAM_FLOAT, "ATC_RAT_PIT_I" },
        { Parameters::k_param_pid_rate_pitch, 2, AP_PARAM_FLOAT, "ATC_RAT_PIT_D" },
        { Parameters::k_param_pid_rate_yaw, 0, AP_PARAM_FLOAT, "ATC_RAT_YAW_P" },
        { Parameters::k_param_pid_rate_yaw, 1, AP_PARAM_FLOAT, "ATC_RAT_YAW_I" },
        { Parameters::k_param_pid_rate_yaw, 2, AP_PARAM_FLOAT, "ATC_RAT_YAW_D" },
#if FRAME_CONFIG == HELI_FRAME
        // PARAMETER_CONVERSION - Added: May-2016
        { Parameters::k_param_pid_rate_roll,  4, AP_PARAM_FLOAT, "ATC_RAT_RLL_VFF" },
        { Parameters::k_param_pid_rate_pitch, 4, AP_PARAM_FLOAT, "ATC_RAT_PIT_VFF" },
        { Parameters::k_param_pid_rate_yaw  , 4, AP_PARAM_FLOAT, "ATC_RAT_YAW_VFF" },
#endif
    };
    const AP_Param::ConversionInfo imax_conversion_info[] = {
        // PARAMETER_CONVERSION - Added: Apr-2016
        { Parameters::k_param_pid_rate_roll,  5, AP_PARAM_FLOAT, "ATC_RAT_RLL_IMAX" },
        { Parameters::k_param_pid_rate_pitch, 5, AP_PARAM_FLOAT, "ATC_RAT_PIT_IMAX" },
        { Parameters::k_param_pid_rate_yaw,   5, AP_PARAM_FLOAT, "ATC_RAT_YAW_IMAX" },
#if FRAME_CONFIG == HELI_FRAME
        // PARAMETER_CONVERSION - Added: May-2016
        { Parameters::k_param_pid_rate_roll,  7, AP_PARAM_FLOAT, "ATC_RAT_RLL_ILMI" },
        { Parameters::k_param_pid_rate_pitch, 7, AP_PARAM_FLOAT, "ATC_RAT_PIT_ILMI" },
        { Parameters::k_param_pid_rate_yaw,   7, AP_PARAM_FLOAT, "ATC_RAT_YAW_ILMI" },
#endif
    };
    // conversion from Copter-3.3 to Copter-3.4
    const AP_Param::ConversionInfo angle_and_filt_conversion_info[] = {
        // PARAMETER_CONVERSION - Added: May-2016
        { Parameters::k_param_p_stabilize_roll, 0, AP_PARAM_FLOAT, "ATC_ANG_RLL_P" },
        { Parameters::k_param_p_stabilize_pitch, 0, AP_PARAM_FLOAT, "ATC_ANG_PIT_P" },
        { Parameters::k_param_p_stabilize_yaw, 0, AP_PARAM_FLOAT, "ATC_ANG_YAW_P" },
        // PARAMETER_CONVERSION - Added: Apr-2016
        { Parameters::k_param_pid_rate_roll, 6, AP_PARAM_FLOAT, "ATC_RAT_RLL_FILT" },
        { Parameters::k_param_pid_rate_pitch, 6, AP_PARAM_FLOAT, "ATC_RAT_PIT_FILT" },
        // PARAMETER_CONVERSION - Added: Jan-2018
        { Parameters::k_param_pid_rate_yaw, 6, AP_PARAM_FLOAT, "ATC_RAT_YAW_FILT" },
        { Parameters::k_param_pi_vel_xy, 0, AP_PARAM_FLOAT, "PSC_VELXY_P" },
        { Parameters::k_param_pi_vel_xy, 1, AP_PARAM_FLOAT, "PSC_VELXY_I" },
        { Parameters::k_param_pi_vel_xy, 2, AP_PARAM_FLOAT, "PSC_VELXY_IMAX" },
        // PARAMETER_CONVERSION - Added: Aug-2021
        { Parameters::k_param_pi_vel_xy, 3, AP_PARAM_FLOAT, "PSC_VELXY_FLTE" },
        // PARAMETER_CONVERSION - Added: Jan-2018
        { Parameters::k_param_p_vel_z, 0, AP_PARAM_FLOAT, "PSC_VELZ_P" },
        { Parameters::k_param_pid_accel_z, 0, AP_PARAM_FLOAT, "PSC_ACCZ_P" },
        { Parameters::k_param_pid_accel_z, 1, AP_PARAM_FLOAT, "PSC_ACCZ_I" },
        { Parameters::k_param_pid_accel_z, 2, AP_PARAM_FLOAT, "PSC_ACCZ_D" },
        { Parameters::k_param_pid_accel_z, 5, AP_PARAM_FLOAT, "PSC_ACCZ_IMAX" },
        // PARAMETER_CONVERSION - Added: Oct-2019
        { Parameters::k_param_pid_accel_z, 6, AP_PARAM_FLOAT, "PSC_ACCZ_FLTE" },
        // PARAMETER_CONVERSION - Added: Jan-2018
        { Parameters::k_param_p_alt_hold, 0, AP_PARAM_FLOAT, "PSC_POSZ_P" },
        { Parameters::k_param_p_pos_xy, 0, AP_PARAM_FLOAT, "PSC_POSXY_P" },
    };
    const AP_Param::ConversionInfo throttle_conversion_info[] = {
        // PARAMETER_CONVERSION - Added: Jun-2016
        { Parameters::k_param_throttle_min, 0, AP_PARAM_FLOAT, "MOT_SPIN_MIN" },
        { Parameters::k_param_throttle_mid, 0, AP_PARAM_FLOAT, "MOT_THST_HOVER" }
    };
    const AP_Param::ConversionInfo loiter_conversion_info[] = {
        // PARAMETER_CONVERSION - Added: Apr-2018
        { Parameters::k_param_wp_nav, 4, AP_PARAM_FLOAT, "LOIT_SPEED" },
        { Parameters::k_param_wp_nav, 7, AP_PARAM_FLOAT, "LOIT_BRK_JERK" },
        { Parameters::k_param_wp_nav, 8, AP_PARAM_FLOAT, "LOIT_ACC_MAX" },
        { Parameters::k_param_wp_nav, 9, AP_PARAM_FLOAT, "LOIT_BRK_ACCEL" }
    };

    // PARAMETER_CONVERSION - Added: Apr-2016
    // gains increase by 27% due to attitude controller's switch to use radians instead of centi-degrees
    // and motor libraries switch to accept inputs in -1 to +1 range instead of -4500 ~ +4500
    float pid_scaler = 1.27f;

#if FRAME_CONFIG != HELI_FRAME
    // Multicopter x-frame gains are 40% lower because -1 or +1 input to motors now results in maximum rotation
    if (g.frame_type == AP_Motors::MOTOR_FRAME_TYPE_X || g.frame_type == AP_Motors::MOTOR_FRAME_TYPE_V || g.frame_type == AP_Motors::MOTOR_FRAME_TYPE_H) {
        pid_scaler = 0.9f;
    }
#endif

    // scale PID gains
    for (const auto &info : pid_conversion_info) {
        AP_Param::convert_old_parameter(&info, pid_scaler);
    }
    // reduce IMAX into -1 ~ +1 range
    for (const auto &info : imax_conversion_info) {
        AP_Param::convert_old_parameter(&info, 1.0f/4500.0f);
    }
    // convert angle controller gain and filter without scaling
    for (const auto &info : angle_and_filt_conversion_info) {
        AP_Param::convert_old_parameter(&info, 1.0f);
    }
    // convert throttle parameters (multicopter only)
    for (const auto &info : throttle_conversion_info) {
        AP_Param::convert_old_parameter(&info, 0.001f);
    }
    // convert RC_FEEL_RP to ATC_INPUT_TC
    // PARAMETER_CONVERSION - Added: Mar-2018
    const AP_Param::ConversionInfo rc_feel_rp_conversion_info = { Parameters::k_param_rc_feel_rp, 0, AP_PARAM_INT8, "ATC_INPUT_TC" };
    AP_Int8 rc_feel_rp_old;
    if (AP_Param::find_old_parameter(&rc_feel_rp_conversion_info, &rc_feel_rp_old)) {
        AP_Param::set_default_by_name(rc_feel_rp_conversion_info.new_name, (1.0f / (2.0f + rc_feel_rp_old.get() * 0.1f)));
    }
    // convert loiter parameters
    for (const auto &info : loiter_conversion_info) {
        AP_Param::convert_old_parameter(&info, 1.0f);
    }

    // TradHeli default parameters
#if FRAME_CONFIG == HELI_FRAME
    static const struct AP_Param::defaults_table_struct heli_defaults_table[] = {
        // PARAMETER_CONVERSION - Added: Nov-2018
        { "LOIT_ACC_MAX", 500.0f },
        { "LOIT_BRK_ACCEL", 125.0f },
        { "LOIT_BRK_DELAY", 1.0f },
        { "LOIT_BRK_JERK", 250.0f },
        { "LOIT_SPEED", 3000.0f },
        { "PHLD_BRAKE_ANGLE", 800.0f },
        { "PHLD_BRAKE_RATE", 4.0f },
        { "PSC_ACCZ_P", 0.28f },
        { "PSC_VELXY_D", 0.0f },
        { "PSC_VELXY_I", 0.5f },
        { "PSC_VELXY_P", 1.0f },
        // PARAMETER_CONVERSION - Added: Jan-2019
        { "RC8_OPTION", 32 },
        // PARAMETER_CONVERSION - Added: Aug-2018
        { "RC_OPTIONS", 0 },
        // PARAMETER_CONVERSION - Added: Feb-2022
        { "ATC_RAT_RLL_ILMI", 0.05},
        { "ATC_RAT_PIT_ILMI", 0.05},
    };
    AP_Param::set_defaults_from_table(heli_defaults_table, ARRAY_SIZE(heli_defaults_table));
#endif

    // attitude and position control filter parameter changes (from _FILT to FLTD, FLTE, FLTT) for Copter-4.0
    // magic numbers shown below are discovered by setting AP_PARAM_KEY_DUMP = 1
    const AP_Param::ConversionInfo ff_and_filt_conversion_info[] = {
#if FRAME_CONFIG == HELI_FRAME
        // tradheli moves ATC_RAT_RLL/PIT_FILT to FLTE, ATC_RAT_YAW_FILT to FLTE
        // PARAMETER_CONVERSION - Added: Jul-2019
        { Parameters::k_param_attitude_control, 386, AP_PARAM_FLOAT, "ATC_RAT_RLL_FLTE" },
        { Parameters::k_param_attitude_control, 387, AP_PARAM_FLOAT, "ATC_RAT_PIT_FLTE" },
        { Parameters::k_param_attitude_control, 388, AP_PARAM_FLOAT, "ATC_RAT_YAW_FLTE" },
#else
        // multicopters move ATC_RAT_RLL/PIT_FILT to FLTD & FLTT, ATC_RAT_YAW_FILT to FLTE
        { Parameters::k_param_attitude_control, 385, AP_PARAM_FLOAT, "ATC_RAT_RLL_FLTD" },
        // PARAMETER_CONVERSION - Added: Oct-2019
        { Parameters::k_param_attitude_control, 385, AP_PARAM_FLOAT, "ATC_RAT_RLL_FLTT" },
        // PARAMETER_CONVERSION - Added: Jul-2019
        { Parameters::k_param_attitude_control, 386, AP_PARAM_FLOAT, "ATC_RAT_PIT_FLTD" },
        // PARAMETER_CONVERSION - Added: Oct-2019
        { Parameters::k_param_attitude_control, 386, AP_PARAM_FLOAT, "ATC_RAT_PIT_FLTT" },
        // PARAMETER_CONVERSION - Added: Jul-2019
        { Parameters::k_param_attitude_control, 387, AP_PARAM_FLOAT, "ATC_RAT_YAW_FLTE" },
        { Parameters::k_param_attitude_control, 449, AP_PARAM_FLOAT, "ATC_RAT_RLL_FF" },
        { Parameters::k_param_attitude_control, 450, AP_PARAM_FLOAT, "ATC_RAT_PIT_FF" },
        { Parameters::k_param_attitude_control, 451, AP_PARAM_FLOAT, "ATC_RAT_YAW_FF" },
#endif
        // PARAMETER_CONVERSION - Added: Oct-2019
        { Parameters::k_param_pos_control, 388, AP_PARAM_FLOAT, "PSC_ACCZ_FLTE" },
    };
    uint8_t filt_table_size = ARRAY_SIZE(ff_and_filt_conversion_info);
    for (uint8_t i=0; i<filt_table_size; i++) {
        AP_Param::convert_old_parameters(&ff_and_filt_conversion_info[i], 1.0f);
    }

#if HAL_INS_NUM_HARMONIC_NOTCH_FILTERS > 1
    if (!ins.harmonic_notches[1].params.enabled()) {
        // notch filter parameter conversions (moved to INS_HNTC2) for 4.2.x, converted from fixed notch
        const AP_Param::ConversionInfo notchfilt_conversion_info[] {
            // PARAMETER_CONVERSION - Added: Apr 2022
            { Parameters::k_param_ins, 101, AP_PARAM_INT8,  "INS_HNTC2_ENABLE" },
            { Parameters::k_param_ins, 293, AP_PARAM_FLOAT, "INS_HNTC2_ATT" },
            { Parameters::k_param_ins, 357, AP_PARAM_FLOAT, "INS_HNTC2_FREQ" },
            { Parameters::k_param_ins, 421, AP_PARAM_FLOAT, "INS_HNTC2_BW" },
        };
        uint8_t notchfilt_table_size = ARRAY_SIZE(notchfilt_conversion_info);
        for (uint8_t i=0; i<notchfilt_table_size; i++) {
            AP_Param::convert_old_parameters(&notchfilt_conversion_info[i], 1.0f);
        }
        AP_Param::set_default_by_name("INS_HNTC2_MODE", 0);
        AP_Param::set_default_by_name("INS_HNTC2_HMNCS", 1);
    }
#endif

    // ACRO_RP_P and ACRO_Y_P replaced with ACRO_RP_RATE and ACRO_Y_RATE for Copter-4.2
    // PARAMETER_CONVERSION - Added: Sep-2021
    const AP_Param::ConversionInfo acro_rpy_conversion_info[] = {
        { Parameters::k_param_acro_rp_p, 0, AP_PARAM_FLOAT, "ACRO_RP_RATE" },
        { Parameters::k_param_acro_yaw_p,  0, AP_PARAM_FLOAT, "ACRO_Y_RATE" }
    };
    for (const auto &info : acro_rpy_conversion_info) {
        AP_Param::convert_old_parameter(&info, 45.0);
    }

    // make any SRV_Channel upgrades needed
    SRV_Channels::upgrade_parameters();
}

#if LANDING_GEAR_ENABLED == ENABLED
/*
  convert landing gear parameters
 */
void Copter::convert_lgr_parameters(void)
{
    // PARAMETER_CONVERSION - Added: Nov-2018

    // convert landing gear PWM values
    uint8_t chan;
    if (!SRV_Channels::find_channel(SRV_Channel::k_landing_gear_control, chan)) {
        return;
    }
    // parameter names are 1 based
    chan += 1;

    char pname[17];
    AP_Int16 *servo_min, *servo_max, *servo_trim;
    AP_Int16 *servo_reversed;

    enum ap_var_type ptype;
    // get pointers to the servo min, max and trim parameters
    snprintf(pname, sizeof(pname), "SERVO%u_MIN", chan);
    servo_min = (AP_Int16 *)AP_Param::find(pname, &ptype);

    snprintf(pname, sizeof(pname), "SERVO%u_MAX", chan);
    servo_max = (AP_Int16 *)AP_Param::find(pname, &ptype);

    snprintf(pname, sizeof(pname), "SERVO%u_TRIM", chan);
    servo_trim = (AP_Int16 *)AP_Param::find(pname, &ptype);

    snprintf(pname, sizeof(pname), "SERVO%u_REVERSED", chan & 0x3F); // Only use the 6 LSBs, avoids a cpp warning
    servo_reversed = (AP_Int16 *)AP_Param::find(pname, &ptype);

    if (!servo_min || !servo_max || !servo_trim || !servo_reversed) {
        // this shouldn't happen
        return;
    }
    if (servo_min->configured_in_storage() ||
        servo_max->configured_in_storage() ||
        servo_trim->configured_in_storage() ||
        servo_reversed->configured_in_storage()) {
        // has been previously saved, don't upgrade
        return;
    }

    // get the old PWM values
    AP_Int16 old_pwm;
    uint16_t old_retract=0, old_deploy=0;
    const AP_Param::ConversionInfo cinfo_ret { Parameters::k_param_landinggear, 0, AP_PARAM_INT16, nullptr };
    const AP_Param::ConversionInfo cinfo_dep { Parameters::k_param_landinggear, 1, AP_PARAM_INT16, nullptr };
    if (AP_Param::find_old_parameter(&cinfo_ret, &old_pwm)) {
        old_retract = (uint16_t)old_pwm.get();
    }
    if (AP_Param::find_old_parameter(&cinfo_dep, &old_pwm)) {
        old_deploy = (uint16_t)old_pwm.get();
    }

    if (old_retract == 0 && old_deploy == 0) {
        // old parameters were never set
        return;
    }

    // use old defaults
    if (old_retract == 0) {
        old_retract = 1250;
    }
    if (old_deploy == 0) {
        old_deploy = 1750;
    }

    // set and save correct values on the servo
    if (old_retract <= old_deploy) {
        servo_max->set_and_save(old_deploy);
        servo_min->set_and_save(old_retract);
        servo_trim->set_and_save(old_retract);
        servo_reversed->set_and_save_ifchanged(0);
    } else {
        servo_max->set_and_save(old_retract);
        servo_min->set_and_save(old_deploy);
        servo_trim->set_and_save(old_deploy);
        servo_reversed->set_and_save_ifchanged(1);
    }
}
#endif

#if FRAME_CONFIG == HELI_FRAME
// handle conversion of tradheli parameters from Copter-3.6 to Copter-3.7
void Copter::convert_tradheli_parameters(void) const
{
        // PARAMETER_CONVERSION - Added: Mar-2019
    if (g2.frame_class.get() == AP_Motors::MOTOR_FRAME_HELI) {
        // single heli conversion info
        const AP_Param::ConversionInfo singleheli_conversion_info[] = {
            { Parameters::k_param_motors, 1, AP_PARAM_INT16, "H_SW_H3_SV1_POS" },
            { Parameters::k_param_motors, 2, AP_PARAM_INT16, "H_SW_H3_SV2_POS" },
            { Parameters::k_param_motors, 3, AP_PARAM_INT16, "H_SW_H3_SV3_POS" },
            { Parameters::k_param_motors, 7, AP_PARAM_INT16, "H_SW_H3_PHANG" },
            { Parameters::k_param_motors, 19, AP_PARAM_INT8, "H_SW_COL_DIR" },
        };

        // convert single heli parameters without scaling
        uint8_t table_size = ARRAY_SIZE(singleheli_conversion_info);
        for (uint8_t i=0; i<table_size; i++) {
            AP_Param::convert_old_parameter(&singleheli_conversion_info[i], 1.0f);
        }

        // convert to known swash type for setups that match
        // PARAMETER_CONVERSION - Added: Sep-2019
        AP_Int16 swash_pos_1, swash_pos_2, swash_pos_3, swash_phang; 
        AP_Int8  swash_type;
        bool swash_pos1_exist = AP_Param::find_old_parameter(&singleheli_conversion_info[0], &swash_pos_1);
        bool swash_pos2_exist = AP_Param::find_old_parameter(&singleheli_conversion_info[1], &swash_pos_2);
        bool swash_pos3_exist = AP_Param::find_old_parameter(&singleheli_conversion_info[2], &swash_pos_3);
        bool swash_phang_exist = AP_Param::find_old_parameter(&singleheli_conversion_info[3], &swash_phang);
        const AP_Param::ConversionInfo swash_type_info { Parameters::k_param_motors, 5, AP_PARAM_INT8, "H_SW_TYPE" };
        bool swash_type_exists = AP_Param::find_old_parameter(&swash_type_info, &swash_type);

        if (swash_type_exists) {
            // convert swash type to new parameter
            AP_Param::convert_old_parameter(&swash_type_info, 1.0f);
        } else {
        // old swash type is not in eeprom and thus type is default value of generic swash
            if (swash_pos1_exist || swash_pos2_exist || swash_pos3_exist || swash_phang_exist) {
                // if any params exist with the generic swash then the upgraded swash type must be generic
                // find the new variable in the variable structures
                enum ap_var_type ptype;
                AP_Param *ap2;
                ap2 = AP_Param::find("H_SW_TYPE", &ptype);
                // make sure the pointer is valid
                if (ap2 != nullptr) {
                    // see if we can load it from EEPROM
                    if (!ap2->configured_in_storage()) {
                        // the new parameter is not in storage so set generic swash
                        AP_Param::set_and_save_by_name("H_SW_TYPE", SwashPlateType::SWASHPLATE_TYPE_H3);            
                    }
                }
            }
        }
    } else if (g2.frame_class.get() == AP_Motors::MOTOR_FRAME_HELI_DUAL) {
        // dual heli conversion info
        const AP_Param::ConversionInfo dualheli_conversion_info[] = {
            { Parameters::k_param_motors, 1, AP_PARAM_INT16, "H_SW_H3_SV1_POS" },
            { Parameters::k_param_motors, 2, AP_PARAM_INT16, "H_SW_H3_SV2_POS" },
            { Parameters::k_param_motors, 3, AP_PARAM_INT16, "H_SW_H3_SV3_POS" },
        // PARAMETER_CONVERSION - Added: Mar-2019
            { Parameters::k_param_motors, 4, AP_PARAM_INT16, "H_SW2_H3_SV1_POS" },
            { Parameters::k_param_motors, 5, AP_PARAM_INT16, "H_SW2_H3_SV2_POS" },
            { Parameters::k_param_motors, 6, AP_PARAM_INT16, "H_SW2_H3_SV3_POS" },
        // PARAMETER_CONVERSION - Added: Sep-2019
            { Parameters::k_param_motors, 7, AP_PARAM_INT16, "H_SW_H3_PHANG" },
        // PARAMETER_CONVERSION - Added: Mar-2019
            { Parameters::k_param_motors, 8, AP_PARAM_INT16, "H_SW2_H3_PHANG" },
        // PARAMETER_CONVERSION - Added: Sep-2019
            { Parameters::k_param_motors, 19, AP_PARAM_INT8, "H_SW_COL_DIR" },
        // PARAMETER_CONVERSION - Added: Mar-2019
            { Parameters::k_param_motors, 19, AP_PARAM_INT8, "H_SW2_COL_DIR" },
        };

        // convert dual heli parameters without scaling
        uint8_t table_size = ARRAY_SIZE(dualheli_conversion_info);
        for (uint8_t i=0; i<table_size; i++) {
            AP_Param::convert_old_parameter(&dualheli_conversion_info[i], 1.0f);
        }


        // PARAMETER_CONVERSION - Added: Sep-2019

        // convert to known swash type for setups that match
        AP_Int16 swash1_pos_1, swash1_pos_2, swash1_pos_3, swash1_phang, swash2_pos_1, swash2_pos_2, swash2_pos_3, swash2_phang; 
        bool swash1_pos1_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[0], &swash1_pos_1);
        bool swash1_pos2_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[1], &swash1_pos_2);
        bool swash1_pos3_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[2], &swash1_pos_3);
        bool swash1_phang_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[6], &swash1_phang);
        bool swash2_pos1_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[3], &swash2_pos_1);
        bool swash2_pos2_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[4], &swash2_pos_2);
        bool swash2_pos3_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[5], &swash2_pos_3);
        bool swash2_phang_exist = AP_Param::find_old_parameter(&dualheli_conversion_info[7], &swash2_phang);

        // SWASH 1
        // old swash type is not in eeprom and thus type is default value of generic swash
        if (swash1_pos1_exist || swash1_pos2_exist || swash1_pos3_exist || swash1_phang_exist) {
            // if any params exist with the generic swash then the upgraded swash type must be generic
            // find the new variable in the variable structures
            enum ap_var_type ptype;
            AP_Param *ap2;
            ap2 = AP_Param::find("H_SW_TYPE", &ptype);
            // make sure the pointer is valid
            if (ap2 != nullptr) {
                // see if we can load it from EEPROM
                if (!ap2->configured_in_storage()) {
                    // the new parameter is not in storage so set generic swash
                    AP_Param::set_and_save_by_name("H_SW_TYPE", SwashPlateType::SWASHPLATE_TYPE_H3);            
                }
            }
        }
        //SWASH 2
        // old swash type is not in eeprom and thus type is default value of generic swash
        if (swash2_pos1_exist || swash2_pos2_exist || swash2_pos3_exist || swash2_phang_exist) {
            // if any params exist with the generic swash then the upgraded swash type must be generic
            // find the new variable in the variable structures
            enum ap_var_type ptype;
            AP_Param *ap2;
            ap2 = AP_Param::find("H_SW2_TYPE", &ptype);
            // make sure the pointer is valid
            if (ap2 != nullptr) {
                // see if we can load it from EEPROM
                if (!ap2->configured_in_storage()) {
                    // the new parameter is not in storage so set generic swash
                    AP_Param::set_and_save_by_name("H_SW2_TYPE", SwashPlateType::SWASHPLATE_TYPE_H3);            
                }
            }
        }
    }

    // table of rsc parameters to be converted with scaling
    const AP_Param::ConversionInfo rschelipct_conversion_info[] = {
        { Parameters::k_param_motors, 1280, AP_PARAM_INT16, "H_RSC_THRCRV_0" },
        { Parameters::k_param_motors, 1344, AP_PARAM_INT16, "H_RSC_THRCRV_25" },
        { Parameters::k_param_motors, 1408, AP_PARAM_INT16, "H_RSC_THRCRV_50" },
        { Parameters::k_param_motors, 1472, AP_PARAM_INT16, "H_RSC_THRCRV_75" },
        { Parameters::k_param_motors, 1536, AP_PARAM_INT16, "H_RSC_THRCRV_100" },
        { Parameters::k_param_motors, 448, AP_PARAM_INT16, "H_RSC_SETPOINT" },
        { Parameters::k_param_motors, 768, AP_PARAM_INT16, "H_RSC_CRITICAL" },
        { Parameters::k_param_motors, 832, AP_PARAM_INT16, "H_RSC_IDLE" },
    };
    // convert heli rsc parameters with scaling
    uint8_t table_size = ARRAY_SIZE(rschelipct_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&rschelipct_conversion_info[i], 0.1f);
    }

    // table of rsc parameters to be converted without scaling
    const AP_Param::ConversionInfo rscheli_conversion_info[] = {
        { Parameters::k_param_motors, 512, AP_PARAM_INT8,  "H_RSC_MODE" },
        { Parameters::k_param_motors, 640, AP_PARAM_INT8,  "H_RSC_RAMP_TIME" },
        { Parameters::k_param_motors, 704, AP_PARAM_INT8,  "H_RSC_RUNUP_TIME" },
        { Parameters::k_param_motors, 1216, AP_PARAM_INT16,"H_RSC_SLEWRATE" },
    };
    // convert heli rsc parameters without scaling
    table_size = ARRAY_SIZE(rscheli_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&rscheli_conversion_info[i], 1.0f);
    }

    // update tail speed parameter with scaling
    AP_Int16 *tailspeed;
    enum ap_var_type ptype;
    tailspeed = (AP_Int16 *)AP_Param::find("H_TAIL_SPEED", &ptype);
    if (tailspeed != nullptr && tailspeed->get() > 100 ) {
        uint16_t tailspeed_pct = (uint16_t)(0.1f * tailspeed->get());
        AP_Param::set_and_save_by_name("H_TAIL_SPEED", tailspeed_pct );
    }

    // PARAMETER_CONVERSION - Added: Dec-2019
    // table of stabilize collective parameters to be converted with scaling
    const AP_Param::ConversionInfo collhelipct_conversion_info[] = {
        { Parameters::k_param_input_manager, 1, AP_PARAM_INT16,  "IM_STB_COL_1" },
        { Parameters::k_param_input_manager, 2, AP_PARAM_INT16,  "IM_STB_COL_2" },
        { Parameters::k_param_input_manager, 3, AP_PARAM_INT16,  "IM_STB_COL_3" },
        { Parameters::k_param_input_manager, 4, AP_PARAM_INT16,  "IM_STB_COL_4" },
    };

    // convert stabilize collective parameters with scaling
    table_size = ARRAY_SIZE(collhelipct_conversion_info);
    for (uint8_t i=0; i<table_size; i++) {
        AP_Param::convert_old_parameter(&collhelipct_conversion_info[i], 0.1f);
    }

}
#endif

void Copter::convert_fs_options_params(void) const
{
    // PARAMETER_CONVERSION - Added: Nov-2019

    // If FS_OPTIONS has already been configured and we don't change it.
    enum ap_var_type ptype;
    AP_Int32 *fs_opt = (AP_Int32 *)AP_Param::find("FS_OPTIONS", &ptype);

    if (fs_opt == nullptr || fs_opt->configured_in_storage() || ptype != AP_PARAM_INT32) {
        return;
    }

    // Variable to capture the new FS_OPTIONS setting
    int32_t fs_options_converted = (int32_t)FailsafeOption::GCS_CONTINUE_IF_PILOT_CONTROL;

    // If FS_THR_ENABLED is 2 (continue mission), change to RTL and add continue mission to the new FS_OPTIONS parameter
    if (g.failsafe_throttle == FS_THR_ENABLED_CONTINUE_MISSION) {
        fs_options_converted |= int32_t(FailsafeOption::RC_CONTINUE_IF_AUTO);
        AP_Param::set_and_save_by_name("FS_THR_ENABLE", FS_THR_ENABLED_ALWAYS_RTL);
    }

    // If FS_GCS_ENABLED is 2 (continue mission), change to RTL and add continue mission to the new FS_OPTIONS parameter
    if (g.failsafe_gcs == FS_GCS_ENABLED_CONTINUE_MISSION) {
        fs_options_converted |= int32_t(FailsafeOption::GCS_CONTINUE_IF_AUTO);
        AP_Param::set_and_save_by_name("FS_GCS_ENABLE", FS_GCS_ENABLED_ALWAYS_RTL);
    }

    // Write the new value to FS_OPTIONS
    // AP_Param::set_and_save_by_name("FS_OPTIONS", fs_options_converted);
    fs_opt->set_and_save(fs_options_converted);
}
RC_Channel.cpp 
#include "Copter.h"

#include "RC_Channel.h"


// defining these two macros and including the RC_Channels_VarInfo header defines the parameter information common to all vehicle types
#define RC_CHANNELS_SUBCLASS RC_Channels_Copter
#define RC_CHANNEL_SUBCLASS RC_Channel_Copter

#include <RC_Channel/RC_Channels_VarInfo.h>

int8_t RC_Channels_Copter::flight_mode_channel_number() const
{
    return copter.g.flight_mode_chan.get();
}

void RC_Channel_Copter::mode_switch_changed(modeswitch_pos_t new_pos)
{
    if (new_pos < 0 || (uint8_t)new_pos > copter.num_flight_modes) {
        // should not have been called
        return;
    }

    if (!copter.set_mode((Mode::Number)copter.flight_modes[new_pos].get(), ModeReason::RC_COMMAND)) {
        return;
    }

    if (!rc().find_channel_for_option(AUX_FUNC::SIMPLE_MODE) &&
        !rc().find_channel_for_option(AUX_FUNC::SUPERSIMPLE_MODE)) {
        // if none of the Aux Switches are set to Simple or Super Simple Mode then
        // set Simple Mode using stored parameters from EEPROM
        if (BIT_IS_SET(copter.g.super_simple, new_pos)) {
            copter.set_simple_mode(Copter::SimpleMode::SUPERSIMPLE);
        } else {
            copter.set_simple_mode(BIT_IS_SET(copter.g.simple_modes, new_pos) ? Copter::SimpleMode::SIMPLE : Copter::SimpleMode::NONE);
        }
    }
}

bool RC_Channels_Copter::has_valid_input() const
{
    if (copter.failsafe.radio) {
        return false;
    }
    if (copter.failsafe.radio_counter != 0) {
        return false;
    }
    return true;
}

// returns true if throttle arming checks should be run
bool RC_Channels_Copter::arming_check_throttle() const {
    if ((copter.g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0) {
        // center sprung throttle configured, dont run AP_Arming check
        // Copter already checks this case in its own arming checks
        return false;
    }
    return RC_Channels::arming_check_throttle();
}

RC_Channel * RC_Channels_Copter::get_arming_channel(void) const
{
    return copter.channel_yaw;
}

// init_aux_switch_function - initialize aux functions
void RC_Channel_Copter::init_aux_function(const aux_func_t ch_option, const AuxSwitchPos ch_flag)
{
    // init channel options
    switch(ch_option) {
    // the following functions do not need to be initialised:
    case AUX_FUNC::ALTHOLD:
    case AUX_FUNC::AUTO:
    case AUX_FUNC::AUTOTUNE:
    case AUX_FUNC::BRAKE:
    case AUX_FUNC::CIRCLE:
    case AUX_FUNC::DRIFT:
    case AUX_FUNC::FLIP:
    case AUX_FUNC::FLOWHOLD:
    case AUX_FUNC::FOLLOW:
    case AUX_FUNC::GUIDED:
    case AUX_FUNC::LAND:
    case AUX_FUNC::LOITER:
    case AUX_FUNC::PARACHUTE_RELEASE:
    case AUX_FUNC::POSHOLD:
    case AUX_FUNC::RESETTOARMEDYAW:
    case AUX_FUNC::RTL:
    case AUX_FUNC::SAVE_TRIM:
    case AUX_FUNC::SAVE_WP:
    case AUX_FUNC::SMART_RTL:
    case AUX_FUNC::STABILIZE:
    case AUX_FUNC::THROW:
    case AUX_FUNC::USER_FUNC1:
    case AUX_FUNC::USER_FUNC2:
    case AUX_FUNC::USER_FUNC3:
    case AUX_FUNC::WINCH_CONTROL:
    case AUX_FUNC::ZIGZAG:
    case AUX_FUNC::ZIGZAG_Auto:
    case AUX_FUNC::ZIGZAG_SaveWP:
    case AUX_FUNC::ACRO:
    case AUX_FUNC::AUTO_RTL:
    case AUX_FUNC::TURTLE:
    case AUX_FUNC::SIMPLE_HEADING_RESET:
    case AUX_FUNC::ARMDISARM_AIRMODE:
    case AUX_FUNC::TURBINE_START:
        break;
    case AUX_FUNC::ACRO_TRAINER:
    case AUX_FUNC::ATTCON_ACCEL_LIM:
    case AUX_FUNC::ATTCON_FEEDFWD:
    case AUX_FUNC::INVERTED:
    case AUX_FUNC::MOTOR_INTERLOCK:
    case AUX_FUNC::PARACHUTE_3POS:      // we trust the vehicle will be disarmed so even if switch is in release position the chute will not release
    case AUX_FUNC::PARACHUTE_ENABLE:
    case AUX_FUNC::PRECISION_LOITER:
    case AUX_FUNC::RANGEFINDER:
    case AUX_FUNC::SIMPLE_MODE:
    case AUX_FUNC::STANDBY:
    case AUX_FUNC::SUPERSIMPLE_MODE:
    case AUX_FUNC::SURFACE_TRACKING:
    case AUX_FUNC::WINCH_ENABLE:
    case AUX_FUNC::AIRMODE:
    case AUX_FUNC::FORCEFLYING:
        run_aux_function(ch_option, ch_flag, AuxFuncTriggerSource::INIT);
        break;
    default:
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
    }
}

// do_aux_function_change_mode - change mode based on an aux switch
// being moved
void RC_Channel_Copter::do_aux_function_change_mode(const Mode::Number mode,
                                                    const AuxSwitchPos ch_flag)
{
    switch(ch_flag) {
    case AuxSwitchPos::HIGH: {
        // engage mode (if not possible we remain in current flight mode)
        copter.set_mode(mode, ModeReason::RC_COMMAND);
        break;
    }
    default:
        // return to flight mode switch's flight mode if we are currently
        // in this mode
        if (copter.flightmode->mode_number() == mode) {
            rc().reset_mode_switch();
        }
    }
}

// do_aux_function - implement the function invoked by auxiliary switches
bool RC_Channel_Copter::do_aux_function(const aux_func_t ch_option, const AuxSwitchPos ch_flag)
{
    switch(ch_option) {
        case AUX_FUNC::FLIP:
            // flip if switch is on, positive throttle and we're actually flying
            if (ch_flag == AuxSwitchPos::HIGH) {
                copter.set_mode(Mode::Number::FLIP, ModeReason::RC_COMMAND);
            }
            break;

        case AUX_FUNC::SIMPLE_MODE:
            // low = simple mode off, middle or high position turns simple mode on
            copter.set_simple_mode((ch_flag == AuxSwitchPos::LOW) ? Copter::SimpleMode::NONE : Copter::SimpleMode::SIMPLE);
            break;

        case AUX_FUNC::SUPERSIMPLE_MODE: {
            Copter::SimpleMode newmode = Copter::SimpleMode::NONE;
            switch (ch_flag) {
            case AuxSwitchPos::LOW:
                break;
            case AuxSwitchPos::MIDDLE:
                newmode = Copter::SimpleMode::SIMPLE;
                break;
            case AuxSwitchPos::HIGH:
                newmode = Copter::SimpleMode::SUPERSIMPLE;
                break;
            }
            copter.set_simple_mode(newmode);
            break;
        }

        case AUX_FUNC::RTL:
#if MODE_RTL_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::RTL, ch_flag);
#endif
            break;

        case AUX_FUNC::SAVE_TRIM:
            if ((ch_flag == AuxSwitchPos::HIGH) &&
                (copter.flightmode->allows_save_trim()) &&
                (copter.channel_throttle->get_control_in() == 0)) {
                copter.save_trim();
            }
            break;

        case AUX_FUNC::SAVE_WP:
#if MODE_AUTO_ENABLED == ENABLED
            // save waypoint when switch is brought high
            if (ch_flag == RC_Channel::AuxSwitchPos::HIGH) {

                // do not allow saving new waypoints while we're in auto or disarmed
                if (copter.flightmode == &copter.mode_auto || !copter.motors->armed()) {
                    break;
                }

                // do not allow saving the first waypoint with zero throttle
                if ((copter.mode_auto.mission.num_commands() == 0) && (copter.channel_throttle->get_control_in() == 0)) {
                    break;
                }

                // create new mission command
                AP_Mission::Mission_Command cmd  = {};

                // if the mission is empty save a takeoff command
                if (copter.mode_auto.mission.num_commands() == 0) {
                    // set our location ID to 16, MAV_CMD_NAV_WAYPOINT
                    cmd.id = MAV_CMD_NAV_TAKEOFF;
                    cmd.content.location.alt = MAX(copter.current_loc.alt,100);

                    // use the current altitude for the target alt for takeoff.
                    // only altitude will matter to the AP mission script for takeoff.
                    if (copter.mode_auto.mission.add_cmd(cmd)) {
                        // log event
                        AP::logger().Write_Event(LogEvent::SAVEWP_ADD_WP);
                    }
                }

                // set new waypoint to current location
                cmd.content.location = copter.current_loc;

                // if throttle is above zero, create waypoint command
                if (copter.channel_throttle->get_control_in() > 0) {
                    cmd.id = MAV_CMD_NAV_WAYPOINT;
                } else {
                    // with zero throttle, create LAND command
                    cmd.id = MAV_CMD_NAV_LAND;
                }

                // save command
                if (copter.mode_auto.mission.add_cmd(cmd)) {
                    // log event
                    AP::logger().Write_Event(LogEvent::SAVEWP_ADD_WP);
                }
            }
#endif
            break;

        case AUX_FUNC::AUTO:
#if MODE_AUTO_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::AUTO, ch_flag);
#endif
            break;

        case AUX_FUNC::RANGEFINDER:
            // enable or disable the rangefinder
#if RANGEFINDER_ENABLED == ENABLED
            if ((ch_flag == AuxSwitchPos::HIGH) &&
                copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
                copter.rangefinder_state.enabled = true;
            } else {
                copter.rangefinder_state.enabled = false;
            }
#endif
            break;

        case AUX_FUNC::ACRO_TRAINER:
#if MODE_ACRO_ENABLED == ENABLED
            switch(ch_flag) {
                case AuxSwitchPos::LOW:
                    copter.g.acro_trainer = (uint8_t)ModeAcro::Trainer::OFF;
                    AP::logger().Write_Event(LogEvent::ACRO_TRAINER_OFF);
                    break;
                case AuxSwitchPos::MIDDLE:
                    copter.g.acro_trainer = (uint8_t)ModeAcro::Trainer::LEVELING;
                    AP::logger().Write_Event(LogEvent::ACRO_TRAINER_LEVELING);
                    break;
                case AuxSwitchPos::HIGH:
                    copter.g.acro_trainer = (uint8_t)ModeAcro::Trainer::LIMITED;
                    AP::logger().Write_Event(LogEvent::ACRO_TRAINER_LIMITED);
                    break;
            }
#endif
            break;

        case AUX_FUNC::AUTOTUNE:
#if AUTOTUNE_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::AUTOTUNE, ch_flag);
#endif
            break;

        case AUX_FUNC::LAND:
            do_aux_function_change_mode(Mode::Number::LAND, ch_flag);
            break;

        case AUX_FUNC::GUIDED:
            do_aux_function_change_mode(Mode::Number::GUIDED, ch_flag);
            break;

        case AUX_FUNC::LOITER:
            do_aux_function_change_mode(Mode::Number::LOITER, ch_flag);
            break;

        case AUX_FUNC::FOLLOW:
            do_aux_function_change_mode(Mode::Number::FOLLOW, ch_flag);
            break;

        case AUX_FUNC::PARACHUTE_ENABLE:
#if PARACHUTE == ENABLED
            // Parachute enable/disable
            copter.parachute.enabled(ch_flag == AuxSwitchPos::HIGH);
#endif
            break;

        case AUX_FUNC::PARACHUTE_RELEASE:
#if PARACHUTE == ENABLED
            if (ch_flag == AuxSwitchPos::HIGH) {
                copter.parachute_manual_release();
            }
#endif
            break;

        case AUX_FUNC::PARACHUTE_3POS:
#if PARACHUTE == ENABLED
            // Parachute disable, enable, release with 3 position switch
            switch (ch_flag) {
                case AuxSwitchPos::LOW:
                    copter.parachute.enabled(false);
                    AP::logger().Write_Event(LogEvent::PARACHUTE_DISABLED);
                    break;
                case AuxSwitchPos::MIDDLE:
                    copter.parachute.enabled(true);
                    AP::logger().Write_Event(LogEvent::PARACHUTE_ENABLED);
                    break;
                case AuxSwitchPos::HIGH:
                    copter.parachute.enabled(true);
                    copter.parachute_manual_release();
                    break;
            }
#endif
            break;

        case AUX_FUNC::ATTCON_FEEDFWD:
            // enable or disable feed forward
            copter.attitude_control->bf_feedforward(ch_flag == AuxSwitchPos::HIGH);
            break;

        case AUX_FUNC::ATTCON_ACCEL_LIM:
            // enable or disable accel limiting by restoring defaults
            copter.attitude_control->accel_limiting(ch_flag == AuxSwitchPos::HIGH);
            break;

        case AUX_FUNC::MOTOR_INTERLOCK:
#if FRAME_CONFIG == HELI_FRAME
            // The interlock logic for ROTOR_CONTROL_MODE_PASSTHROUGH is handled 
            // in heli_update_rotor_speed_targets.  Otherwise turn on when above low.
            if (copter.motors->get_rsc_mode() != ROTOR_CONTROL_MODE_PASSTHROUGH) {
                copter.ap.motor_interlock_switch = (ch_flag == AuxSwitchPos::HIGH || ch_flag == AuxSwitchPos::MIDDLE);
            }
#else
            copter.ap.motor_interlock_switch = (ch_flag == AuxSwitchPos::HIGH || ch_flag == AuxSwitchPos::MIDDLE);
#endif
            break;
			
        case AUX_FUNC::TURBINE_START:
#if FRAME_CONFIG == HELI_FRAME     
           switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    copter.motors->set_turb_start(true);
                    break;
                case AuxSwitchPos::MIDDLE:
                    // nothing
                    break;
                case AuxSwitchPos::LOW:
                    copter.motors->set_turb_start(false);
                    break;
           }
#endif
           break;
		 
        case AUX_FUNC::BRAKE:
#if MODE_BRAKE_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::BRAKE, ch_flag);
#endif
            break;

        case AUX_FUNC::THROW:
#if MODE_THROW_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::THROW, ch_flag);
#endif
            break;

        case AUX_FUNC::PRECISION_LOITER:
#if PRECISION_LANDING == ENABLED && MODE_LOITER_ENABLED == ENABLED
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    copter.mode_loiter.set_precision_loiter_enabled(true);
                    break;
                case AuxSwitchPos::MIDDLE:
                    // nothing
                    break;
                case AuxSwitchPos::LOW:
                    copter.mode_loiter.set_precision_loiter_enabled(false);
                    break;
            }
#endif
            break;

        case AUX_FUNC::SMART_RTL:
#if MODE_SMARTRTL_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::SMART_RTL, ch_flag);
#endif
            break;

        case AUX_FUNC::INVERTED:
#if FRAME_CONFIG == HELI_FRAME
            switch (ch_flag) {
            case AuxSwitchPos::HIGH:
                copter.motors->set_inverted_flight(true);
                copter.attitude_control->set_inverted_flight(true);
                copter.heli_flags.inverted_flight = true;
                break;
            case AuxSwitchPos::MIDDLE:
                // nothing
                break;
            case AuxSwitchPos::LOW:
                copter.motors->set_inverted_flight(false);
                copter.attitude_control->set_inverted_flight(false);
                copter.heli_flags.inverted_flight = false;
                break;
            }
#endif
            break;

        case AUX_FUNC::WINCH_ENABLE:
#if WINCH_ENABLED == ENABLED
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    // high switch position stops winch using rate control
                    copter.g2.winch.set_desired_rate(0.0f);
                    break;
                case AuxSwitchPos::MIDDLE:
                case AuxSwitchPos::LOW:
                    // all other position relax winch
                    copter.g2.winch.relax();
                    break;
                }
#endif
            break;

        case AUX_FUNC::WINCH_CONTROL:
            // do nothing, used to control the rate of the winch and is processed within AP_Winch
            break;

#ifdef USERHOOK_AUXSWITCH
        case AUX_FUNC::USER_FUNC1:
            copter.userhook_auxSwitch1(ch_flag);
            break;

        case AUX_FUNC::USER_FUNC2:
            copter.userhook_auxSwitch2(ch_flag);
            break;

        case AUX_FUNC::USER_FUNC3:
            copter.userhook_auxSwitch3(ch_flag);
            break;
#endif

        case AUX_FUNC::ZIGZAG:
#if MODE_ZIGZAG_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::ZIGZAG, ch_flag);
#endif
            break;

        case AUX_FUNC::ZIGZAG_SaveWP:
#if MODE_ZIGZAG_ENABLED == ENABLED
            if (copter.flightmode == &copter.mode_zigzag) {
                // initialize zigzag auto
                copter.mode_zigzag.init_auto();
                switch (ch_flag) {
                    case AuxSwitchPos::LOW:
                        copter.mode_zigzag.save_or_move_to_destination(ModeZigZag::Destination::A);
                        break;
                    case AuxSwitchPos::MIDDLE:
                        copter.mode_zigzag.return_to_manual_control(false);
                        break;
                    case AuxSwitchPos::HIGH:
                        copter.mode_zigzag.save_or_move_to_destination(ModeZigZag::Destination::B);
                        break;
                }
            }
#endif
            break;

        case AUX_FUNC::STABILIZE:
            do_aux_function_change_mode(Mode::Number::STABILIZE, ch_flag);
            break;

        case AUX_FUNC::POSHOLD:
#if MODE_POSHOLD_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::POSHOLD, ch_flag);
#endif
            break;

        case AUX_FUNC::ALTHOLD:
            do_aux_function_change_mode(Mode::Number::ALT_HOLD, ch_flag);
            break;


        case AUX_FUNC::ACRO:
#if MODE_ACRO_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::ACRO, ch_flag);
#endif
            break;

        case AUX_FUNC::FLOWHOLD:
#if AP_OPTICALFLOW_ENABLED
            do_aux_function_change_mode(Mode::Number::FLOWHOLD, ch_flag);
#endif
            break;

        case AUX_FUNC::CIRCLE:
#if MODE_CIRCLE_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::CIRCLE, ch_flag);
#endif
            break;

        case AUX_FUNC::DRIFT:
#if MODE_DRIFT_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::DRIFT, ch_flag);
#endif
            break;

        case AUX_FUNC::STANDBY: {
            switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    copter.standby_active = true;
                    AP::logger().Write_Event(LogEvent::STANDBY_ENABLE);
                    gcs().send_text(MAV_SEVERITY_INFO, "Stand By Enabled");
                    break;
                default:
                    copter.standby_active = false;
                    AP::logger().Write_Event(LogEvent::STANDBY_DISABLE);
                    gcs().send_text(MAV_SEVERITY_INFO, "Stand By Disabled");
                    break;
                }
            break;
        }

        case AUX_FUNC::SURFACE_TRACKING:
            switch (ch_flag) {
            case AuxSwitchPos::LOW:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::GROUND);
                break;
            case AuxSwitchPos::MIDDLE:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::NONE);
                break;
            case AuxSwitchPos::HIGH:
                copter.surface_tracking.set_surface(Copter::SurfaceTracking::Surface::CEILING);
                break;
            }
            break;

        case AUX_FUNC::ZIGZAG_Auto:
#if MODE_ZIGZAG_ENABLED == ENABLED
            if (copter.flightmode == &copter.mode_zigzag) {
                switch (ch_flag) {
                case AuxSwitchPos::HIGH:
                    copter.mode_zigzag.run_auto();
                    break;
                default:
                    copter.mode_zigzag.suspend_auto();
                    break;
                }
            }
#endif
            break;

        case AUX_FUNC::AIRMODE:
            do_aux_function_change_air_mode(ch_flag);
#if MODE_ACRO_ENABLED == ENABLED && FRAME_CONFIG != HELI_FRAME
            copter.mode_acro.air_mode_aux_changed();
#endif
            break;

        case AUX_FUNC::FORCEFLYING:
            do_aux_function_change_force_flying(ch_flag);
            break;

        case AUX_FUNC::AUTO_RTL:
#if MODE_AUTO_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::AUTO_RTL, ch_flag);
#endif
            break;

        case AUX_FUNC::TURTLE:
#if MODE_TURTLE_ENABLED == ENABLED
            do_aux_function_change_mode(Mode::Number::TURTLE, ch_flag);
#endif
            break;

        case AUX_FUNC::SIMPLE_HEADING_RESET:
            if (ch_flag == AuxSwitchPos::HIGH) {
                copter.init_simple_bearing();
                gcs().send_text(MAV_SEVERITY_INFO, "Simple heading reset");
            }
            break;

        case AUX_FUNC::ARMDISARM_AIRMODE:
            RC_Channel::do_aux_function_armdisarm(ch_flag);
            if (copter.arming.is_armed()) {
                copter.ap.armed_with_airmode_switch = true;
            }
            break;

    default:
        return RC_Channel::do_aux_function(ch_option, ch_flag);
    }
    return true;
}

// change air-mode status
void RC_Channel_Copter::do_aux_function_change_air_mode(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        copter.air_mode = AirMode::AIRMODE_ENABLED;
        break;
    case AuxSwitchPos::MIDDLE:
        break;
    case AuxSwitchPos::LOW:
        copter.air_mode = AirMode::AIRMODE_DISABLED;
        break;
    }
}

// change force flying status
void RC_Channel_Copter::do_aux_function_change_force_flying(const AuxSwitchPos ch_flag)
{
    switch (ch_flag) {
    case AuxSwitchPos::HIGH:
        copter.force_flying = true;
        break;
    case AuxSwitchPos::MIDDLE:
        break;
    case AuxSwitchPos::LOW:
        copter.force_flying = false;
        break;
    }
}

// save_trim - adds roll and pitch trims from the radio to ahrs
void Copter::save_trim()
{
    // save roll and pitch trim
    float roll_trim = ToRad((float)channel_roll->get_control_in()/100.0f);
    float pitch_trim = ToRad((float)channel_pitch->get_control_in()/100.0f);
    ahrs.add_trim(roll_trim, pitch_trim);
    AP::logger().Write_Event(LogEvent::SAVE_TRIM);
    gcs().send_text(MAV_SEVERITY_INFO, "Trim saved");
}

// auto_trim - slightly adjusts the ahrs.roll_trim and ahrs.pitch_trim towards the current stick positions
// meant to be called continuously while the pilot attempts to keep the copter level
void Copter::auto_trim_cancel()
{
    auto_trim_counter = 0;
    AP_Notify::flags.save_trim = false;
    gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim cancelled");
}

void Copter::auto_trim()
{
    if (auto_trim_counter > 0) {
        if (copter.flightmode != &copter.mode_stabilize ||
            !copter.motors->armed()) {
            auto_trim_cancel();
            return;
        }

        // flash the leds
        AP_Notify::flags.save_trim = true;

        if (!auto_trim_started) {
            if (ap.land_complete) {
                // haven't taken off yet
                return;
            }
            auto_trim_started = true;
        }

        if (ap.land_complete) {
            // landed again.
            auto_trim_cancel();
            return;
        }

        auto_trim_counter--;

        // calculate roll trim adjustment
        float roll_trim_adjustment = ToRad((float)channel_roll->get_control_in() / 4000.0f);

        // calculate pitch trim adjustment
        float pitch_trim_adjustment = ToRad((float)channel_pitch->get_control_in() / 4000.0f);

        // add trim to ahrs object
        // save to eeprom on last iteration
        ahrs.add_trim(roll_trim_adjustment, pitch_trim_adjustment, (auto_trim_counter == 0));

        // on last iteration restore leds and accel gains to normal
        if (auto_trim_counter == 0) {
            AP_Notify::flags.save_trim = false;
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim: Trims saved");
        }
    }
}
UserCode.cpp 
#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
UserParameters.cpp 
#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars
    AP_GROUPINFO("_INT8", 0, UserParameters, _int8, 0),
    AP_GROUPINFO("_INT16", 1, UserParameters, _int16, 0),
    AP_GROUPINFO("_FLOAT", 2, UserParameters, _float, 0),
    
    AP_GROUPEND
};
afs_copter.cpp
/*
  copter specific AP_AdvancedFailsafe class
 */

#include "Copter.h"

#if ADVANCED_FAILSAFE == ENABLED

/*
  setup radio_out values for all channels to termination values
 */
void AP_AdvancedFailsafe_Copter::terminate_vehicle(void)
{
    if (_terminate_action == TERMINATE_ACTION_LAND) {
        copter.set_mode(Mode::Number::LAND, ModeReason::TERMINATE);
    } else {
        // stop motors
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        copter.motors->output();

        // disarm as well
        copter.arming.disarm(AP_Arming::Method::AFS);
    
        // and set all aux channels
        SRV_Channels::set_output_limit(SRV_Channel::k_heli_rsc, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_output_limit(SRV_Channel::k_heli_tail_rsc, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_output_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_output_limit(SRV_Channel::k_ignition, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_output_limit(SRV_Channel::k_none, SRV_Channel::Limit::TRIM);
        SRV_Channels::set_output_limit(SRV_Channel::k_manual, SRV_Channel::Limit::TRIM);
    }

    SRV_Channels::output_ch_all();
}

void AP_AdvancedFailsafe_Copter::setup_IO_failsafe(void)
{
    // setup failsafe for all aux channels
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_heli_rsc, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_heli_tail_rsc, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_engine_run_enable, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_ignition, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_none, SRV_Channel::Limit::TRIM);
    SRV_Channels::set_failsafe_limit(SRV_Channel::k_manual, SRV_Channel::Limit::TRIM);

#if FRAME_CONFIG != HELI_FRAME
    // setup AP_Motors outputs for failsafe
    uint16_t mask = copter.motors->get_motor_mask();
    hal.rcout->set_failsafe_pwm(mask, copter.motors->get_pwm_output_min());
#endif
}

/*
  return an AFS_MODE for current control mode
 */
AP_AdvancedFailsafe::control_mode AP_AdvancedFailsafe_Copter::afs_mode(void)
{
    switch (copter.flightmode->mode_number()) {
    case Mode::Number::AUTO:
    case Mode::Number::AUTO_RTL:
    case Mode::Number::GUIDED:
    case Mode::Number::RTL:
    case Mode::Number::LAND:
        return AP_AdvancedFailsafe::AFS_AUTO;
    default:
        break;
    }
    return AP_AdvancedFailsafe::AFS_STABILIZED;
}

#endif // ADVANCED_FAILSAFE
Autoyaw.cpp
#include "Copter.h"

Mode::AutoYaw Mode::auto_yaw;

// roi_yaw - returns heading towards location held in roi
float Mode::AutoYaw::roi_yaw() const
{
    return get_bearing_cd(copter.inertial_nav.get_position_xy_cm(), roi.xy());
}

float Mode::AutoYaw::look_ahead_yaw()
{
    const Vector3f& vel = copter.inertial_nav.get_velocity_neu_cms();
    const float speed_sq = vel.xy().length_squared();
    // Commanded Yaw to automatically look ahead.
    if (copter.position_ok() && (speed_sq > (YAW_LOOK_AHEAD_MIN_SPEED * YAW_LOOK_AHEAD_MIN_SPEED))) {
        _look_ahead_yaw = degrees(atan2f(vel.y,vel.x))*100.0f;
    }
    return _look_ahead_yaw;
}

void Mode::AutoYaw::set_mode_to_default(bool rtl)
{
    set_mode(default_mode(rtl));
}

// default_mode - returns auto_yaw.mode() based on WP_YAW_BEHAVIOR parameter
// set rtl parameter to true if this is during an RTL
autopilot_yaw_mode Mode::AutoYaw::default_mode(bool rtl) const
{
    switch (copter.g.wp_yaw_behavior) {

    case WP_YAW_BEHAVIOR_NONE:
        return AUTO_YAW_HOLD;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL:
        if (rtl) {
            return AUTO_YAW_HOLD;
        } else {
            return AUTO_YAW_LOOK_AT_NEXT_WP;
        }

    case WP_YAW_BEHAVIOR_LOOK_AHEAD:
        return AUTO_YAW_LOOK_AHEAD;

    case WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP:
    default:
        return AUTO_YAW_LOOK_AT_NEXT_WP;
    }
}

// set_mode - sets the yaw mode for auto
void Mode::AutoYaw::set_mode(autopilot_yaw_mode yaw_mode)
{
    // return immediately if no change
    if (_mode == yaw_mode) {
        return;
    }
    _mode = yaw_mode;

    // perform initialisation
    switch (_mode) {

    case AUTO_YAW_HOLD:
        break;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
        // wpnav will initialise heading when wpnav's set_destination method is called
        break;

    case AUTO_YAW_ROI:
        // look ahead until we know otherwise
        break;

    case AUTO_YAW_FIXED:
        // keep heading pointing in the direction held in fixed_yaw
        // caller should set the fixed_yaw
        break;

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        _look_ahead_yaw = copter.ahrs.yaw_sensor;
        break;

    case AUTO_YAW_RESETTOARMEDYAW:
        // initial_armed_bearing will be set during arming so no init required
        break;

    case AUTO_YAW_ANGLE_RATE:
        break;

    case AUTO_YAW_RATE:
        // initialise target yaw rate to zero
        _yaw_rate_cds = 0.0f;
        break;

    case AUTO_YAW_CIRCLE:
        // no initialisation required
        break;
    }
}

// set_fixed_yaw - sets the yaw look at heading for auto mode
void Mode::AutoYaw::set_fixed_yaw(float angle_deg, float turn_rate_ds, int8_t direction, bool relative_angle)
{
    _last_update_ms = millis();

    _yaw_angle_cd = copter.attitude_control->get_att_target_euler_cd().z;
    _yaw_rate_cds = 0.0;

    // calculate final angle as relative to vehicle heading or absolute
    if (relative_angle) {
        _fixed_yaw_offset_cd = angle_deg * 100.0 * (direction >= 0 ? 1.0 : -1.0);
    } else {
        // absolute angle
        _fixed_yaw_offset_cd = wrap_180_cd(angle_deg * 100.0 - _yaw_angle_cd);
        if ( direction < 0 && is_positive(_fixed_yaw_offset_cd) ) {
            _fixed_yaw_offset_cd -= 36000.0;
        } else if ( direction > 0 && is_negative(_fixed_yaw_offset_cd) ) {
            _fixed_yaw_offset_cd += 36000.0;
        }
    }

    // get turn speed
    if (!is_positive(turn_rate_ds)) {
        // default to default slew rate
        _fixed_yaw_slewrate_cds = copter.attitude_control->get_slew_yaw_max_degs() * 100.0;
    } else {
        _fixed_yaw_slewrate_cds = MIN(copter.attitude_control->get_slew_yaw_max_degs(), turn_rate_ds) * 100.0;
    }

    // set yaw mode
    set_mode(AUTO_YAW_FIXED);
}

// set_fixed_yaw - sets the yaw look at heading for auto mode
void Mode::AutoYaw::set_yaw_angle_rate(float yaw_angle_d, float yaw_rate_ds)
{
    _last_update_ms = millis();

    _yaw_angle_cd = yaw_angle_d * 100.0;
    _yaw_rate_cds = yaw_rate_ds * 100.0;

    // set yaw mode
    set_mode(AUTO_YAW_ANGLE_RATE);
}

// set_roi - sets the yaw to look at roi for auto mode
void Mode::AutoYaw::set_roi(const Location &roi_location)
{
    // if location is zero lat, lon and altitude turn off ROI
    if (roi_location.alt == 0 && roi_location.lat == 0 && roi_location.lng == 0) {
        // set auto yaw mode back to default assuming the active command is a waypoint command.  A more sophisticated method is required to ensure we return to the proper yaw control for the active command
        auto_yaw.set_mode_to_default(false);
#if HAL_MOUNT_ENABLED
        // switch off the camera tracking if enabled
        if (copter.camera_mount.get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
            copter.camera_mount.set_mode_to_default();
        }
#endif  // HAL_MOUNT_ENABLED
    } else {
#if HAL_MOUNT_ENABLED
        // check if mount type requires us to rotate the quad
        if (!copter.camera_mount.has_pan_control()) {
            if (roi_location.get_vector_from_origin_NEU(roi)) {
                auto_yaw.set_mode(AUTO_YAW_ROI);
            }
        }
        // send the command to the camera mount
        copter.camera_mount.set_roi_target(roi_location);

        // TO-DO: expand handling of the do_nav_roi to support all modes of the MAVLink.  Currently we only handle mode 4 (see below)
        //      0: do nothing
        //      1: point at next waypoint
        //      2: point at a waypoint taken from WP# parameter (2nd parameter?)
        //      3: point at a location given by alt, lon, lat parameters
        //      4: point at a target given a target id (can't be implemented)
#else
        // if we have no camera mount aim the quad at the location
        if (roi_location.get_vector_from_origin_NEU(roi)) {
            auto_yaw.set_mode(AUTO_YAW_ROI);
        }
#endif  // HAL_MOUNT_ENABLED
    }
}

// set auto yaw rate in centi-degrees per second
void Mode::AutoYaw::set_rate(float turn_rate_cds)
{
    set_mode(AUTO_YAW_RATE);
    _yaw_rate_cds = turn_rate_cds;
}

// yaw - returns target heading depending upon auto_yaw.mode()
float Mode::AutoYaw::yaw()
{
    switch (_mode) {

    case AUTO_YAW_ROI:
        // point towards a location held in roi
        return roi_yaw();

    case AUTO_YAW_FIXED: {
        // keep heading pointing in the direction held in fixed_yaw
        // with no pilot input allowed
        const uint32_t now_ms = millis();
        float dt = (now_ms - _last_update_ms) * 0.001;
        _last_update_ms = now_ms;
        float yaw_angle_step = constrain_float(_fixed_yaw_offset_cd, - dt * _fixed_yaw_slewrate_cds, dt * _fixed_yaw_slewrate_cds);
        _fixed_yaw_offset_cd -= yaw_angle_step;
        _yaw_angle_cd += yaw_angle_step;
        return _yaw_angle_cd;
    }

    case AUTO_YAW_LOOK_AHEAD:
        // Commanded Yaw to automatically look ahead.
        return look_ahead_yaw();

    case AUTO_YAW_RESETTOARMEDYAW:
        // changes yaw to be same as when quad was armed
        return copter.initial_armed_bearing;

    case AUTO_YAW_CIRCLE:
#if MODE_CIRCLE_ENABLED
        if (copter.circle_nav->is_active()) {
            return copter.circle_nav->get_yaw();
        }
#endif
        // return the current attitude target
        return wrap_360_cd(copter.attitude_control->get_att_target_euler_cd().z);

    case AUTO_YAW_ANGLE_RATE:{
        const uint32_t now_ms = millis();
        float dt = (now_ms - _last_update_ms) * 0.001;
        _last_update_ms = now_ms;
        _yaw_angle_cd += _yaw_rate_cds * dt;
        return _yaw_angle_cd;
    }

    case AUTO_YAW_LOOK_AT_NEXT_WP:
    default:
        // point towards next waypoint.
        // we don't use wp_bearing because we don't want the copter to turn too much during flight
        return copter.pos_control->get_yaw_cd();
    }
}

// returns yaw rate normally set by SET_POSITION_TARGET mavlink
// messages (positive is clockwise, negative is counter clockwise)
float Mode::AutoYaw::rate_cds() const
{
    switch (_mode) {

    case AUTO_YAW_HOLD:
    case AUTO_YAW_ROI:
    case AUTO_YAW_FIXED:
    case AUTO_YAW_LOOK_AHEAD:
    case AUTO_YAW_RESETTOARMEDYAW:
    case AUTO_YAW_CIRCLE:
        return 0.0f;

    case AUTO_YAW_ANGLE_RATE:
    case AUTO_YAW_RATE:
        return _yaw_rate_cds;

    case AUTO_YAW_LOOK_AT_NEXT_WP:
        return copter.pos_control->get_yaw_rate_cds();
    }

    // return zero turn rate (this should never happen)
    return 0.0f;
}
Avoidance.cpp
#include "Copter.h"

// check if proximity type Simple Avoidance should be enabled based on alt
void Copter::low_alt_avoidance()
{
#if AC_AVOID_ENABLED == ENABLED
    int32_t alt_cm;
    if (!get_rangefinder_height_interpolated_cm(alt_cm)) {
        // enable avoidance if we don't have a valid rangefinder reading
        avoid.proximity_alt_avoidance_enable(true);
        return;
    }

    bool enable_avoidance = true;
    if (alt_cm < avoid.get_min_alt() * 100.0f) {
        enable_avoidance = false;
    }
    avoid.proximity_alt_avoidance_enable(enable_avoidance);
#endif
}
Avoidance_adsb.cpp
#include "Copter.h"
#include <AP_Notify/AP_Notify.h>

#if HAL_ADSB_ENABLED
void Copter::avoidance_adsb_update(void)
{
    adsb.update();
    avoidance_adsb.update();
}

#include <stdio.h>

MAV_COLLISION_ACTION AP_Avoidance_Copter::handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action)
{
    MAV_COLLISION_ACTION actual_action = requested_action;
    bool failsafe_state_change = false;

    // check for changes in failsafe
    if (!copter.failsafe.adsb) {
        copter.failsafe.adsb = true;
        failsafe_state_change = true;
        // record flight mode in case it's required for the recovery
        prev_control_mode = copter.flightmode->mode_number();
    }

    // take no action in some flight modes
    if (copter.flightmode->mode_number() == Mode::Number::LAND ||
#if MODE_THROW_ENABLED == ENABLED
        copter.flightmode->mode_number() == Mode::Number::THROW ||
#endif
        copter.flightmode->mode_number() == Mode::Number::FLIP) {
        actual_action = MAV_COLLISION_ACTION_NONE;
    }

    // if landed and we will take some kind of action, just disarm
    if ((actual_action > MAV_COLLISION_ACTION_REPORT) && copter.should_disarm_on_failsafe()) {
        copter.arming.disarm(AP_Arming::Method::ADSBCOLLISIONACTION);
        actual_action = MAV_COLLISION_ACTION_NONE;
    } else {

        // take action based on requested action
        switch (actual_action) {

            case MAV_COLLISION_ACTION_RTL:
                // attempt to switch to RTL, if this fails (i.e. flying in manual mode with bad position) do nothing
                if (failsafe_state_change) {
                    if (!copter.set_mode(Mode::Number::RTL, ModeReason::AVOIDANCE)) {
                        actual_action = MAV_COLLISION_ACTION_NONE;
                    }
                }
                break;

            case MAV_COLLISION_ACTION_HOVER:
                // attempt to switch to Loiter, if this fails (i.e. flying in manual mode with bad position) do nothing
                if (failsafe_state_change) {
                    if (!copter.set_mode(Mode::Number::LOITER, ModeReason::AVOIDANCE)) {
                        actual_action = MAV_COLLISION_ACTION_NONE;
                    }
                }
                break;

            case MAV_COLLISION_ACTION_ASCEND_OR_DESCEND:
                // climb or descend to avoid obstacle
                if (!handle_avoidance_vertical(obstacle, failsafe_state_change)) {
                    actual_action = MAV_COLLISION_ACTION_NONE;
                }
                break;

            case MAV_COLLISION_ACTION_MOVE_HORIZONTALLY:
                // move horizontally to avoid obstacle
                if (!handle_avoidance_horizontal(obstacle, failsafe_state_change)) {
                    actual_action = MAV_COLLISION_ACTION_NONE;
                }
                break;

            case MAV_COLLISION_ACTION_MOVE_PERPENDICULAR:
                if (!handle_avoidance_perpendicular(obstacle, failsafe_state_change)) {
                    actual_action = MAV_COLLISION_ACTION_NONE;
                }
                break;

            // unsupported actions and those that require no response
            case MAV_COLLISION_ACTION_NONE:
                return actual_action;
            case MAV_COLLISION_ACTION_REPORT:
            default:
                break;
        }
    }

    if (failsafe_state_change) {
        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_ADSB,
                                 LogErrorCode(actual_action));
    }

    // return with action taken
    return actual_action;
}

void AP_Avoidance_Copter::handle_recovery(RecoveryAction recovery_action)
{
    // check we are coming out of failsafe
    if (copter.failsafe.adsb) {
        copter.failsafe.adsb = false;
        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_ADSB,
                                 LogErrorCode::ERROR_RESOLVED);

        // restore flight mode if requested and user has not changed mode since
        if (copter.control_mode_reason == ModeReason::AVOIDANCE) {
            switch (recovery_action) {

            case RecoveryAction::REMAIN_IN_AVOID_ADSB:
                // do nothing, we'll stay in the AVOID_ADSB mode which is guided which will loiter forever
                break;

            case RecoveryAction::RESUME_PREVIOUS_FLIGHTMODE:
                set_mode_else_try_RTL_else_LAND(prev_control_mode);
                break;

            case RecoveryAction::RTL:
                set_mode_else_try_RTL_else_LAND(Mode::Number::RTL);
                break;

            case RecoveryAction::RESUME_IF_AUTO_ELSE_LOITER:
                if (prev_control_mode == Mode::Number::AUTO) {
                    set_mode_else_try_RTL_else_LAND(Mode::Number::AUTO);
                }
                break;

            default:
                break;
            } // switch
        }
    }
}

void AP_Avoidance_Copter::set_mode_else_try_RTL_else_LAND(Mode::Number mode)
{
    if (!copter.set_mode(mode, ModeReason::AVOIDANCE_RECOVERY)) {
        // on failure RTL or LAND
        if (!copter.set_mode(Mode::Number::RTL, ModeReason::AVOIDANCE_RECOVERY)) {
            copter.set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE_RECOVERY);
        }
    }
}

int32_t AP_Avoidance_Copter::get_altitude_minimum() const
{
#if MODE_RTL_ENABLED == ENABLED
    // do not descend if below RTL alt
    return copter.g.rtl_altitude;
#else
    return 0;
#endif
}

// check flight mode is avoid_adsb
bool AP_Avoidance_Copter::check_flightmode(bool allow_mode_change)
{
    // ensure copter is in avoid_adsb mode
    if (allow_mode_change && copter.flightmode->mode_number() != Mode::Number::AVOID_ADSB) {
        if (!copter.set_mode(Mode::Number::AVOID_ADSB, ModeReason::AVOIDANCE)) {
            // failed to set mode so exit immediately
            return false;
        }
    }

    // check flight mode
    return (copter.flightmode->mode_number() == Mode::Number::AVOID_ADSB);
}

bool AP_Avoidance_Copter::handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // ensure copter is in avoid_adsb mode
    if (!check_flightmode(allow_mode_change)) {
        return false;
    }

    // decide on whether we should climb or descend
    bool should_climb = false;
    Location my_loc;
    if (AP::ahrs().get_location(my_loc)) {
        should_climb = my_loc.alt > obstacle->_location.alt;
    }

    // get best vector away from obstacle
    Vector3f velocity_neu;
    if (should_climb) {
        velocity_neu.z = copter.wp_nav->get_default_speed_up();
    } else {
        velocity_neu.z = -copter.wp_nav->get_default_speed_down();
        // do not descend if below minimum altitude
        if (copter.current_loc.alt < get_altitude_minimum()) {
            velocity_neu.z = 0.0f;
        }
    }

    // send target velocity
    copter.mode_avoid_adsb.set_velocity(velocity_neu);
    return true;
}

bool AP_Avoidance_Copter::handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // ensure copter is in avoid_adsb mode
    if (!check_flightmode(allow_mode_change)) {
        return false;
    }

    // get best vector away from obstacle
    Vector3f velocity_neu;
    if (get_vector_perpendicular(obstacle, velocity_neu)) {
        // remove vertical component
        velocity_neu.z = 0.0f;
        // check for divide by zero
        if (is_zero(velocity_neu.x) && is_zero(velocity_neu.y)) {
            return false;
        }
        // re-normalise
        velocity_neu.normalize();
        // convert horizontal components to velocities
        velocity_neu.x *= copter.wp_nav->get_default_speed_xy();
        velocity_neu.y *= copter.wp_nav->get_default_speed_xy();
        // send target velocity
        copter.mode_avoid_adsb.set_velocity(velocity_neu);
        return true;
    }

    // if we got this far we failed to set the new target
    return false;
}

bool AP_Avoidance_Copter::handle_avoidance_perpendicular(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // ensure copter is in avoid_adsb mode
    if (!check_flightmode(allow_mode_change)) {
        return false;
    }

    // get best vector away from obstacle
    Vector3f velocity_neu;
    if (get_vector_perpendicular(obstacle, velocity_neu)) {
        // convert horizontal components to velocities
        velocity_neu.x *= copter.wp_nav->get_default_speed_xy();
        velocity_neu.y *= copter.wp_nav->get_default_speed_xy();
        // use up and down waypoint speeds
        if (velocity_neu.z > 0.0f) {
            velocity_neu.z *= copter.wp_nav->get_default_speed_up();
        } else {
            velocity_neu.z *= copter.wp_nav->get_default_speed_down();
            // do not descend if below minimum altitude
            if (copter.current_loc.alt < get_altitude_minimum()) {
                velocity_neu.z = 0.0f;
            }
        }
        // send target velocity
        copter.mode_avoid_adsb.set_velocity(velocity_neu);
        return true;
    }

    // if we got this far we failed to set the new target
    return false;
}
#endif
Baro_ground_effect.cpp
#include "Copter.h"

void Copter::update_ground_effect_detector(void)
{
    if(!g2.gndeffect_comp_enabled || !motors->armed()) {
        // disarmed - disable ground effect and return
        gndeffect_state.takeoff_expected = false;
        gndeffect_state.touchdown_expected = false;
        ahrs.set_takeoff_expected(gndeffect_state.takeoff_expected);
        ahrs.set_touchdown_expected(gndeffect_state.touchdown_expected);
        return;
    }

    // variable initialization
    uint32_t tnow_ms = millis();
    float xy_des_speed_cms = 0.0f;
    float xy_speed_cms = 0.0f;
    float des_climb_rate_cms = pos_control->get_vel_desired_cms().z;

    if (pos_control->is_active_xy()) {
        Vector3f vel_target = pos_control->get_vel_target_cms();
        vel_target.z = 0.0f;
        xy_des_speed_cms = vel_target.length();
    }

    if (position_ok() || ekf_has_relative_position()) {
        Vector3f vel = inertial_nav.get_velocity_neu_cms();
        vel.z = 0.0f;
        xy_speed_cms = vel.length();
    }

    // takeoff logic

    if (flightmode->mode_number() == Mode::Number::THROW) {
        // throw mode never wants the takeoff expected EKF code
        gndeffect_state.takeoff_expected = false;
    } else if (motors->armed() && ap.land_complete) {
        // if we are armed and haven't yet taken off then we expect an imminent takeoff
        gndeffect_state.takeoff_expected = true;
    }

    // if we aren't taking off yet, reset the takeoff timer, altitude and complete flag
    const bool throttle_up = flightmode->has_manual_throttle() && channel_throttle->get_control_in() > 0;
    if (!throttle_up && ap.land_complete) {
        gndeffect_state.takeoff_time_ms = tnow_ms;
        gndeffect_state.takeoff_alt_cm = inertial_nav.get_position_z_up_cm();
    }

    // if we are in takeoff_expected and we meet the conditions for having taken off
    // end the takeoff_expected state
    if (gndeffect_state.takeoff_expected && (tnow_ms-gndeffect_state.takeoff_time_ms > 5000 || inertial_nav.get_position_z_up_cm()-gndeffect_state.takeoff_alt_cm > 50.0f)) {
        gndeffect_state.takeoff_expected = false;
    }

    // landing logic
    Vector3f angle_target_rad = attitude_control->get_att_target_euler_cd() * radians(0.01f);
    bool small_angle_request = cosf(angle_target_rad.x)*cosf(angle_target_rad.y) > cosf(radians(7.5f));
    bool xy_speed_low = (position_ok() || ekf_has_relative_position()) && xy_speed_cms <= 125.0f;
    bool xy_speed_demand_low = pos_control->is_active_xy() && xy_des_speed_cms <= 125.0f;
    bool slow_horizontal = xy_speed_demand_low || (xy_speed_low && !pos_control->is_active_xy()) || (flightmode->mode_number() == Mode::Number::ALT_HOLD && small_angle_request);

    bool descent_demanded = pos_control->is_active_z() && des_climb_rate_cms < 0.0f;
    bool slow_descent_demanded = descent_demanded && des_climb_rate_cms >= -100.0f;
    bool z_speed_low = fabsf(inertial_nav.get_velocity_z_up_cms()) <= 60.0f;
    bool slow_descent = (slow_descent_demanded || (z_speed_low && descent_demanded));

    gndeffect_state.touchdown_expected = slow_horizontal && slow_descent;

    // Prepare the EKF for ground effect if either takeoff or touchdown is expected.
    ahrs.set_takeoff_expected(gndeffect_state.takeoff_expected);
    ahrs.set_touchdown_expected(gndeffect_state.touchdown_expected);
}

// update ekf terrain height stable setting
// when set to true, this allows the EKF to stabilize the normally barometer based altitude using a rangefinder
// this is not related to terrain following
void Copter::update_ekf_terrain_height_stable()
{
    // set to false if no position estimate
    if (!position_ok() && !ekf_has_relative_position()) {
        ahrs.set_terrain_hgt_stable(false);
        return;
    }

    // consider terrain height stable if vehicle is taking off or landing
    ahrs.set_terrain_hgt_stable(flightmode->is_taking_off() || flightmode->is_landing());
}
Commands.cpp
#include "Copter.h"

// checks if we should update ahrs/RTL home position from the EKF
void Copter::update_home_from_EKF()
{
    // exit immediately if home already set
    if (ahrs.home_is_set()) {
        return;
    }

    // special logic if home is set in-flight
    if (motors->armed()) {
        set_home_to_current_location_inflight();
    } else {
        // move home to current ekf location (this will set home_state to HOME_SET)
        if (!set_home_to_current_location(false)) {
            // ignore failure
        }
    }
}

// set_home_to_current_location_inflight - set home to current GPS location (horizontally) and EKF origin vertically
void Copter::set_home_to_current_location_inflight() {
    // get current location from EKF
    Location temp_loc;
    Location ekf_origin;
    if (ahrs.get_location(temp_loc) && ahrs.get_origin(ekf_origin)) {
        temp_loc.alt = ekf_origin.alt;
        if (!set_home(temp_loc, false)) {
            return;
        }
        // we have successfully set AHRS home, set it for SmartRTL
#if MODE_SMARTRTL_ENABLED == ENABLED
        g2.smart_rtl.set_home(true);
#endif
    }
}

// set_home_to_current_location - set home to current GPS location
bool Copter::set_home_to_current_location(bool lock) {
    // get current location from EKF
    Location temp_loc;
    if (ahrs.get_location(temp_loc)) {
        if (!set_home(temp_loc, lock)) {
            return false;
        }
        // we have successfully set AHRS home, set it for SmartRTL
#if MODE_SMARTRTL_ENABLED == ENABLED
        g2.smart_rtl.set_home(true);
#endif
        return true;
    }
    return false;
}

// set_home - sets ahrs home (used for RTL) to specified location
//  initialises inertial nav and compass on first call
//  returns true if home location set successfully
bool Copter::set_home(const Location& loc, bool lock)
{
    // check EKF origin has been set
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }

    // check home is close to EKF origin
    if (far_from_EKF_origin(loc)) {
        return false;
    }

    const bool home_was_set = ahrs.home_is_set();

    // set ahrs home (used for RTL)
    if (!ahrs.set_home(loc)) {
        return false;
    }

    // init inav and compass declination
    if (!home_was_set) {
#if MODE_AUTO_ENABLED == ENABLED
        // log new home position which mission library will pull from ahrs
        if (should_log(MASK_LOG_CMD)) {
            AP_Mission::Mission_Command temp_cmd;
            if (mode_auto.mission.read_cmd_from_storage(0, temp_cmd)) {
                logger.Write_Mission_Cmd(mode_auto.mission, temp_cmd);
            }
        }
#endif
    }

    // lock home position
    if (lock) {
        ahrs.lock_home();
    }

    // return success
    return true;
}

// far_from_EKF_origin - checks if a location is too far from the EKF origin
//  returns true if too far
bool Copter::far_from_EKF_origin(const Location& loc)
{
    // check distance to EKF origin
    Location ekf_origin;
    if (ahrs.get_origin(ekf_origin)) {
        if (labs(ekf_origin.alt - loc.alt)*0.01 > EKF_ORIGIN_MAX_ALT_KM*1000.0) {
            return true;
        }
    }

    // close enough to origin
    return false;
}
Compassmot.cpp
#include "Copter.h"

/*
  compass/motor interference calibration
 */

// setup_compassmot - sets compass's motor interference parameters
MAV_RESULT Copter::mavlink_compassmot(const GCS_MAVLINK &gcs_chan)
{
#if FRAME_CONFIG == HELI_FRAME
    // compassmot not implemented for tradheli
    return MAV_RESULT_UNSUPPORTED;
#else
    int8_t   comp_type;                 // throttle or current based compensation
    Vector3f compass_base[COMPASS_MAX_INSTANCES];           // compass vector when throttle is zero
    Vector3f motor_impact[COMPASS_MAX_INSTANCES];           // impact of motors on compass vector
    Vector3f motor_impact_scaled[COMPASS_MAX_INSTANCES];    // impact of motors on compass vector scaled with throttle
    Vector3f motor_compensation[COMPASS_MAX_INSTANCES];     // final compensation to be stored to eeprom
    float    throttle_pct;              // throttle as a percentage 0.0 ~ 1.0
    float    throttle_pct_max = 0.0f;   // maximum throttle reached (as a percentage 0~1.0)
    float    current_amps_max = 0.0f;   // maximum current reached
    float    interference_pct[COMPASS_MAX_INSTANCES]{};       // interference as a percentage of total mag field (for reporting purposes only)
    uint32_t last_run_time;
    uint32_t last_send_time;
    bool     updated = false;           // have we updated the compensation vector at least once
    uint8_t  command_ack_start = command_ack_counter;

    // exit immediately if we are already in compassmot
    if (ap.compass_mot) {
        // ignore restart messages
        return MAV_RESULT_TEMPORARILY_REJECTED;
    } else {
        ap.compass_mot = true;
    }

    // check compass is enabled
    if (!AP::compass().available()) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Compass disabled");
        ap.compass_mot = false;
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // check compass health
    compass.read();
    for (uint8_t i=0; i<compass.get_count(); i++) {
        if (!compass.healthy(i)) {
            gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Check compass");
            ap.compass_mot = false;
            return MAV_RESULT_TEMPORARILY_REJECTED;
        }
    }

    // check if radio is calibrated
    if (!arming.rc_calibration_checks(true)) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "RC not calibrated");
        ap.compass_mot = false;
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // check throttle is at zero
    read_radio();
    if (channel_throttle->get_control_in() != 0) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Throttle not zero");
        ap.compass_mot = false;
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // check we are landed
    if (!ap.land_complete) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL, "Not landed");
        ap.compass_mot = false;
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // disable cpu failsafe
    failsafe_disable();

    float current;

    // default compensation type to use current if possible
    if (battery.current_amps(current)) {
        comp_type = AP_COMPASS_MOT_COMP_CURRENT;
    } else {
        comp_type = AP_COMPASS_MOT_COMP_THROTTLE;
    }

    // send back initial ACK
    mavlink_msg_command_ack_send(gcs_chan.get_chan(), MAV_CMD_PREFLIGHT_CALIBRATION,0,
                                 0, 0, 0, 0);

    // flash leds
    AP_Notify::flags.esc_calibration = true;

    // warn user we are starting calibration
    gcs_chan.send_text(MAV_SEVERITY_INFO, "Starting calibration");

    // inform what type of compensation we are attempting
    if (comp_type == AP_COMPASS_MOT_COMP_CURRENT) {
        gcs_chan.send_text(MAV_SEVERITY_INFO, "Current");
    } else {
        gcs_chan.send_text(MAV_SEVERITY_INFO, "Throttle");
    }

    // disable throttle failsafe
    g.failsafe_throttle = FS_THR_DISABLED;

    // disable motor compensation
    compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    for (uint8_t i=0; i<compass.get_count(); i++) {
        compass.set_motor_compensation(i, Vector3f(0,0,0));
    }

    // get initial compass readings
    compass.read();

    // store initial x,y,z compass values
    // initialise interference percentage
    for (uint8_t i=0; i<compass.get_count(); i++) {
        compass_base[i] = compass.get_field(i);
        interference_pct[i] = 0.0f;
    }

    EXPECT_DELAY_MS(5000);

    // enable motors and pass through throttle
    enable_motor_output();
    motors->armed(true);
    hal.util->set_soft_armed(true);

    // initialise run time
    last_run_time = millis();
    last_send_time = millis();

    // main run while there is no user input and the compass is healthy
    while (command_ack_start == command_ack_counter && compass.healthy() && motors->armed()) {
        EXPECT_DELAY_MS(5000);

        // 50hz loop
        if (millis() - last_run_time < 20) {
            hal.scheduler->delay(5);
            continue;
        }
        last_run_time = millis();

        // read radio input
        read_radio();

        // pass through throttle to motors
        SRV_Channels::cork();
        motors->set_throttle_passthrough_for_esc_calibration(channel_throttle->get_control_in() * 0.001f);
        SRV_Channels::push();

        // read some compass values
        compass.read();

        // read current
        battery.read();

        // calculate scaling for throttle
        throttle_pct = (float)channel_throttle->get_control_in() * 0.001f;
        throttle_pct = constrain_float(throttle_pct,0.0f,1.0f);

        // record maximum throttle
        throttle_pct_max = MAX(throttle_pct_max, throttle_pct);

        if (!battery.current_amps(current)) {
            current = 0;
        }
        current_amps_max = MAX(current_amps_max, current);

        // if throttle is near zero, update base x,y,z values
        if (!is_positive(throttle_pct)) {
            for (uint8_t i=0; i<compass.get_count(); i++) {
                compass_base[i] = compass_base[i] * 0.99f + compass.get_field(i) * 0.01f;
            }
        } else {

            // calculate diff from compass base and scale with throttle
            for (uint8_t i=0; i<compass.get_count(); i++) {
                motor_impact[i] = compass.get_field(i) - compass_base[i];
            }

            // throttle based compensation
            if (comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
                // for each compass
                for (uint8_t i=0; i<compass.get_count(); i++) {
                    // scale by throttle
                    motor_impact_scaled[i] = motor_impact[i] / throttle_pct;
                    // adjust the motor compensation to negate the impact
                    motor_compensation[i] = motor_compensation[i] * 0.99f - motor_impact_scaled[i] * 0.01f;
                }
                updated = true;
            } else {
                // for each compass
                for (uint8_t i=0; i<compass.get_count(); i++) {
                    // adjust the motor compensation to negate the impact if drawing over 3amps
                    if (current >= 3.0f) {
                        motor_impact_scaled[i] = motor_impact[i] / current;
                        motor_compensation[i] = motor_compensation[i] * 0.99f - motor_impact_scaled[i] * 0.01f;
                        updated = true;
                    }
                }
            }

            // calculate interference percentage at full throttle as % of total mag field
            if (comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
                for (uint8_t i=0; i<compass.get_count(); i++) {
                    // interference is impact@fullthrottle / mag field * 100
                    interference_pct[i] = motor_compensation[i].length() / (float)arming.compass_magfield_expected() * 100.0f;
                }
            } else {
                for (uint8_t i=0; i<compass.get_count(); i++) {
                    // interference is impact/amp * (max current seen / max throttle seen) / mag field * 100
                    interference_pct[i] = motor_compensation[i].length() * (current_amps_max/throttle_pct_max) / (float)arming.compass_magfield_expected() * 100.0f;
                }
            }
        }

        if (AP_HAL::millis() - last_send_time > 500) {
            last_send_time = AP_HAL::millis();
            mavlink_msg_compassmot_status_send(gcs_chan.get_chan(),
                                               channel_throttle->get_control_in(),
                                               current,
                                               interference_pct[0],
                                               motor_compensation[0].x,
                                               motor_compensation[0].y,
                                               motor_compensation[0].z);
#if HAL_WITH_ESC_TELEM
            // send ESC telemetry to monitor ESC and motor temperatures
            AP::esc_telem().send_esc_telemetry_mavlink(gcs_chan.get_chan());
#endif
        }
    }

    // stop motors
    motors->output_min();
    motors->armed(false);
    hal.util->set_soft_armed(false);

    // set and save motor compensation
    if (updated) {
        compass.motor_compensation_type(comp_type);
        for (uint8_t i=0; i<compass.get_count(); i++) {
            compass.set_motor_compensation(i, motor_compensation[i]);
        }
        compass.save_motor_compensation();
        // display success message
        gcs_chan.send_text(MAV_SEVERITY_INFO, "Calibration successful");
    } else {
        // compensation vector never updated, report failure
        gcs_chan.send_text(MAV_SEVERITY_NOTICE, "Failed");
        compass.motor_compensation_type(AP_COMPASS_MOT_COMP_DISABLED);
    }

    // turn off notify leds
    AP_Notify::flags.esc_calibration = false;

    // re-enable cpu failsafe
    failsafe_enable();

    // re-enable failsafes
    g.failsafe_throttle.load();

    // flag we have completed
    ap.compass_mot = false;

    return MAV_RESULT_ACCEPTED;
#endif  // FRAME_CONFIG != HELI_FRAME
}
Crash_ check.cpp
#include "Copter.h"

// Code to detect a crash main ArduCopter code
#define CRASH_CHECK_TRIGGER_SEC         2       // 2 seconds inverted indicates a crash
#define CRASH_CHECK_ANGLE_DEVIATION_DEG 30.0f   // 30 degrees beyond target angle is signal we are out of control
#define CRASH_CHECK_ANGLE_MIN_DEG       15.0f   // vehicle must be leaning at least 15deg to trigger crash check
#define CRASH_CHECK_SPEED_MAX           10.0f   // vehicle must be moving at less than 10m/s to trigger crash check
#define CRASH_CHECK_ACCEL_MAX           3.0f    // vehicle must be accelerating less than 3m/s/s to be considered crashed

// Code to detect a thrust loss main ArduCopter code
#define THRUST_LOSS_CHECK_TRIGGER_SEC         1     // 1 second descent while level and high throttle indicates thrust loss
#define THRUST_LOSS_CHECK_ANGLE_DEVIATION_CD  1500  // we can't expect to maintain altitude beyond 15 degrees on all aircraft
#define THRUST_LOSS_CHECK_MINIMUM_THROTTLE    0.9f  // we can expect to maintain altitude above 90 % throttle

// Yaw imbalance check
#define YAW_IMBALANCE_IMAX_THRESHOLD 0.75f
#define YAW_IMBALANCE_WARN_MS 10000

// crash_check - disarms motors if a crash has been detected
// crashes are detected by the vehicle being more than 20 degrees beyond it's angle limits continuously for more than 1 second
// called at MAIN_LOOP_RATE
void Copter::crash_check()
{
    static uint16_t crash_counter;  // number of iterations vehicle may have been crashed

    // return immediately if disarmed, or crash checking disabled
    if (!motors->armed() || ap.land_complete || g.fs_crash_check == 0) {
        crash_counter = 0;
        return;
    }

    // exit immediately if in standby
    if (standby_active) {
        crash_counter = 0;
        return;
    }

    // exit immediately if in force flying
    if (force_flying && !flightmode->is_landing()) {
        crash_counter = 0;
        return;
    }

    // return immediately if we are not in an angle stabilize flight mode or we are flipping
    if (flightmode->mode_number() == Mode::Number::ACRO || flightmode->mode_number() == Mode::Number::FLIP) {
        crash_counter = 0;
        return;
    }

#if MODE_AUTOROTATE_ENABLED == ENABLED
    //return immediately if in autorotation mode
    if (flightmode->mode_number() == Mode::Number::AUTOROTATE) {
        crash_counter = 0;
        return;
    }
#endif

    // vehicle not crashed if 1hz filtered acceleration is more than 3m/s (1G on Z-axis has been subtracted)
    const float filtered_acc = land_accel_ef_filter.get().length();
    if (filtered_acc >= CRASH_CHECK_ACCEL_MAX) {
        crash_counter = 0;
        return;
    }

    // check for lean angle over 15 degrees
    const float lean_angle_deg = degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()));
    if (lean_angle_deg <= CRASH_CHECK_ANGLE_MIN_DEG) {
        crash_counter = 0;
        return;
    }

    // check for angle error over 30 degrees
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error <= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        crash_counter = 0;
        return;
    }

    // check for speed under 10m/s (if available)
    Vector3f vel_ned;
    if (ahrs.get_velocity_NED(vel_ned) && (vel_ned.length() >= CRASH_CHECK_SPEED_MAX)) {
        crash_counter = 0;
        return;
    }

    // we may be crashing
    crash_counter++;

    // check if crashing for 2 seconds
    if (crash_counter >= (CRASH_CHECK_TRIGGER_SEC * scheduler.get_loop_rate_hz())) {
        AP::logger().Write_Error(LogErrorSubsystem::CRASH_CHECK, LogErrorCode::CRASH_CHECK_CRASH);
        // send message to gcs
        gcs().send_text(MAV_SEVERITY_EMERGENCY,"Crash: Disarming: AngErr=%.0f>%.0f, Accel=%.1f<%.1f", angle_error, CRASH_CHECK_ANGLE_DEVIATION_DEG, filtered_acc, CRASH_CHECK_ACCEL_MAX);
        // disarm motors
        copter.arming.disarm(AP_Arming::Method::CRASH);
    }
}

// check for loss of thrust and trigger thrust boost in motors library
void Copter::thrust_loss_check()
{
    static uint16_t thrust_loss_counter;  // number of iterations vehicle may have been crashed

    // no-op if suppresed by flight options param
    if ((copter.g2.flight_options & uint32_t(FlightOptions::DISABLE_THRUST_LOSS_CHECK)) != 0) {
        return;
    }

    // exit immediately if thrust boost is already engaged
    if (motors->get_thrust_boost()) {
        return;
    }

    // return immediately if disarmed
    if (!motors->armed() || ap.land_complete) {
        thrust_loss_counter = 0;
        return;
    }

    // exit immediately if in standby
    if (standby_active) {
        return;
    }

    // check for desired angle over 15 degrees
    // todo: add thrust angle to AC_AttitudeControl
    const Vector3f angle_target = attitude_control->get_att_target_euler_cd();
    if (sq(angle_target.x) + sq(angle_target.y) > sq(THRUST_LOSS_CHECK_ANGLE_DEVIATION_CD)) {
        thrust_loss_counter = 0;
        return;
    }

    // check for throttle over 90% or throttle saturation
    if ((attitude_control->get_throttle_in() < THRUST_LOSS_CHECK_MINIMUM_THROTTLE) && (!motors->limit.throttle_upper)) {
        thrust_loss_counter = 0;
        return;
    }

    // check throttle is over 25% to prevent checks triggering from thrust limitations caused by low commanded throttle
    if ((attitude_control->get_throttle_in() < 0.25f)) {
        thrust_loss_counter = 0;
        return;
    }

    // check for descent
    if (!is_negative(inertial_nav.get_velocity_z_up_cms())) {
        thrust_loss_counter = 0;
        return;
    }

    // check for angle error over 30 degrees to ensure the aircraft has attitude control
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error >= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        thrust_loss_counter = 0;
        return;
    }

    // the aircraft is descending with low requested roll and pitch, at full available throttle, with attitude control
    // we may have lost thrust
    thrust_loss_counter++;

    // check if thrust loss for 1 second
    if (thrust_loss_counter >= (THRUST_LOSS_CHECK_TRIGGER_SEC * scheduler.get_loop_rate_hz())) {
        // reset counter
        thrust_loss_counter = 0;
        AP::logger().Write_Error(LogErrorSubsystem::THRUST_LOSS_CHECK, LogErrorCode::FAILSAFE_OCCURRED);
        // send message to gcs
        gcs().send_text(MAV_SEVERITY_EMERGENCY, "Potential Thrust Loss (%d)", (int)motors->get_lost_motor() + 1);
        // enable thrust loss handling
        motors->set_thrust_boost(true);
        // the motors library disables this when it is no longer needed to achieve the commanded output

#if GRIPPER_ENABLED == ENABLED
        if ((copter.g2.flight_options & uint32_t(FlightOptions::RELEASE_GRIPPER_ON_THRUST_LOSS)) != 0) {
            copter.g2.gripper.release();
        }
#endif
    }
}

// check for a large yaw imbalance, could be due to badly calibrated ESC or misaligned motors
void Copter::yaw_imbalance_check()
{
    // no-op if suppresed by flight options param
    if ((copter.g2.flight_options & uint32_t(FlightOptions::DISABLE_YAW_IMBALANCE_WARNING)) != 0) {
        return;
    }

    // If I is disabled it is unlikely that the issue is not obvious
    if (!is_positive(attitude_control->get_rate_yaw_pid().kI())) {
        return;
    }

    // thrust loss is trigerred, yaw issues are expected
    if (motors->get_thrust_boost()) {
        yaw_I_filt.reset(0.0f);
        return;
    }

    // return immediately if disarmed
    if (!motors->armed() || ap.land_complete) {
        yaw_I_filt.reset(0.0f);
        return;
    }

    // exit immediately if in standby
    if (standby_active) {
        yaw_I_filt.reset(0.0f);
        return;
    }

    // magnitude of low pass filtered I term
    const float I_term = attitude_control->get_rate_yaw_pid().get_pid_info().I;
    const float I = fabsf(yaw_I_filt.apply(attitude_control->get_rate_yaw_pid().get_pid_info().I,G_Dt));
    if (I > fabsf(I_term)) {
        // never allow to be larger than I
        yaw_I_filt.reset(I_term);
    }

    const float I_max = attitude_control->get_rate_yaw_pid().imax();
    if ((is_positive(I_max) && ((I > YAW_IMBALANCE_IMAX_THRESHOLD * I_max) || (is_equal(I_term,I_max))))) {
        // filtered using over precentage of I max or unfiltered = I max
        // I makes up more than precentage of total available control power
        const uint32_t now = millis();
        if (now - last_yaw_warn_ms > YAW_IMBALANCE_WARN_MS) {
            last_yaw_warn_ms = now;
            gcs().send_text(MAV_SEVERITY_EMERGENCY, "Yaw Imbalance %0.0f%%", I *100);
        }
    }
}

#if PARACHUTE == ENABLED

// Code to detect a crash main ArduCopter code
#define PARACHUTE_CHECK_TRIGGER_SEC         1       // 1 second of loss of control triggers the parachute
#define PARACHUTE_CHECK_ANGLE_DEVIATION_DEG 30.0f   // 30 degrees off from target indicates a loss of control

// parachute_check - disarms motors and triggers the parachute if serious loss of control has been detected
// vehicle is considered to have a "serious loss of control" by the vehicle being more than 30 degrees off from the target roll and pitch angles continuously for 1 second
// called at MAIN_LOOP_RATE
void Copter::parachute_check()
{
    static uint16_t control_loss_count; // number of iterations we have been out of control
    static int32_t baro_alt_start;

    // exit immediately if parachute is not enabled
    if (!parachute.enabled()) {
        return;
    }

    // pass is_flying to parachute library
    parachute.set_is_flying(!ap.land_complete);

    // pass sink rate to parachute library
    parachute.set_sink_rate(-inertial_nav.get_velocity_z_up_cms() * 0.01f);

    // exit immediately if in standby
    if (standby_active) {
        return;
    }

    // call update to give parachute a chance to move servo or relay back to off position
    parachute.update();

    // return immediately if motors are not armed or pilot's throttle is above zero
    if (!motors->armed()) {
        control_loss_count = 0;
        return;
    }

    if (parachute.release_initiated()) {
        copter.arming.disarm(AP_Arming::Method::PARACHUTE_RELEASE);
        return;
    }

    // return immediately if we are not in an angle stabilize flight mode or we are flipping
    if (flightmode->mode_number() == Mode::Number::ACRO || flightmode->mode_number() == Mode::Number::FLIP) {
        control_loss_count = 0;
        return;
    }

    // ensure we are flying
    if (ap.land_complete) {
        control_loss_count = 0;
        return;
    }

    // ensure the first control_loss event is from above the min altitude
    if (control_loss_count == 0 && parachute.alt_min() != 0 && (current_loc.alt < (int32_t)parachute.alt_min() * 100)) {
        return;
    }

    // trigger parachute release based on sink rate
    parachute.check_sink_rate();

    // check for angle error over 30 degrees
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error <= PARACHUTE_CHECK_ANGLE_DEVIATION_DEG) {
        if (control_loss_count > 0) {
            control_loss_count--;
        }
        return;
    }

    // increment counter
    if (control_loss_count < (PARACHUTE_CHECK_TRIGGER_SEC*scheduler.get_loop_rate_hz())) {
        control_loss_count++;
    }

    // record baro alt if we have just started losing control
    if (control_loss_count == 1) {
        baro_alt_start = baro_alt;

    // exit if baro altitude change indicates we are not falling
    } else if (baro_alt >= baro_alt_start) {
        control_loss_count = 0;
        return;

    // To-Do: add check that the vehicle is actually falling

    // check if loss of control for at least 1 second
    } else if (control_loss_count >= (PARACHUTE_CHECK_TRIGGER_SEC*scheduler.get_loop_rate_hz())) {
        // reset control loss counter
        control_loss_count = 0;
        AP::logger().Write_Error(LogErrorSubsystem::CRASH_CHECK, LogErrorCode::CRASH_CHECK_LOSS_OF_CONTROL);
        // release parachute
        parachute_release();
    }
}

// parachute_release - trigger the release of the parachute, disarm the motors and notify the user
void Copter::parachute_release()
{
    // disarm motors
    arming.disarm(AP_Arming::Method::PARACHUTE_RELEASE);

    // release parachute
    parachute.release();

#if LANDING_GEAR_ENABLED == ENABLED
    // deploy landing gear
    landinggear.set_position(AP_LandingGear::LandingGear_Deploy);
#endif
}

// parachute_manual_release - trigger the release of the parachute, after performing some checks for pilot error
//   checks if the vehicle is landed 
void Copter::parachute_manual_release()
{
    // exit immediately if parachute is not enabled
    if (!parachute.enabled()) {
        return;
    }

    // do not release if vehicle is landed
    // do not release if we are landed or below the minimum altitude above home
    if (ap.land_complete) {
        // warn user of reason for failure
        gcs().send_text(MAV_SEVERITY_INFO,"Parachute: Landed");
        AP::logger().Write_Error(LogErrorSubsystem::PARACHUTES, LogErrorCode::PARACHUTE_LANDED);
        return;
    }

    // do not release if we are landed or below the minimum altitude above home
    if ((parachute.alt_min() != 0 && (current_loc.alt < (int32_t)parachute.alt_min() * 100))) {
        // warn user of reason for failure
        gcs().send_text(MAV_SEVERITY_ALERT,"Parachute: Too low");
        AP::logger().Write_Error(LogErrorSubsystem::PARACHUTES, LogErrorCode::PARACHUTE_TOO_LOW);
        return;
    }

    // if we get this far release parachute
    parachute_release();
}

#endif // PARACHUTE == ENABLED
Ekf_check.cpp
#include "Copter.h"

/**
 *
 * Detects failures of the ekf or inertial nav system triggers an alert
 * to the pilot and helps take countermeasures
 *
 */

#ifndef EKF_CHECK_ITERATIONS_MAX
 # define EKF_CHECK_ITERATIONS_MAX          10      // 1 second (ie. 10 iterations at 10hz) of bad variances signals a failure
#endif

#ifndef EKF_CHECK_WARNING_TIME
 # define EKF_CHECK_WARNING_TIME            (30*1000)   // warning text messages are sent to ground no more than every 30 seconds
#endif

////////////////////////////////////////////////////////////////////////////////
// EKF_check structure
////////////////////////////////////////////////////////////////////////////////
static struct {
    uint8_t fail_count;         // number of iterations ekf or dcm have been out of tolerances
    uint8_t bad_variance : 1;   // true if ekf should be considered untrusted (fail_count has exceeded EKF_CHECK_ITERATIONS_MAX)
    bool has_ever_passed;       // true if the ekf checks have ever passed
    uint32_t last_warn_time;    // system time of last warning in milliseconds.  Used to throttle text warnings sent to GCS
} ekf_check_state;

// ekf_check - detects if ekf variance are out of tolerance and triggers failsafe
// should be called at 10hz
void Copter::ekf_check()
{
    // ensure EKF_CHECK_ITERATIONS_MAX is at least 7
    static_assert(EKF_CHECK_ITERATIONS_MAX >= 7, "EKF_CHECK_ITERATIONS_MAX must be at least 7");

    // exit immediately if ekf has no origin yet - this assumes the origin can never become unset
    Location temp_loc;
    if (!ahrs.get_origin(temp_loc)) {
        return;
    }

    // return immediately if ekf check is disabled
    if (g.fs_ekf_thresh <= 0.0f) {
        ekf_check_state.fail_count = 0;
        ekf_check_state.bad_variance = false;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
        failsafe_ekf_off_event();   // clear failsafe
        return;
    }

    // compare compass and velocity variance vs threshold and also check
    // if we has a position estimate
    const bool over_threshold = ekf_over_threshold();
    const bool has_position = ekf_has_relative_position() || ekf_has_absolute_position();
    const bool checks_passed = !over_threshold && has_position;

    // return if ekf checks have never passed
    ekf_check_state.has_ever_passed |= checks_passed;
    if (!ekf_check_state.has_ever_passed) {
        return;
    }

    // increment or decrement counters and take action
    if (!checks_passed) {
        // if compass is not yet flagged as bad
        if (!ekf_check_state.bad_variance) {
            // increase counter
            ekf_check_state.fail_count++;
            if (ekf_check_state.fail_count == (EKF_CHECK_ITERATIONS_MAX-2) && over_threshold) {
                // we are two iterations away from declaring an EKF failsafe, ask the EKF if we can reset
                // yaw to resolve the issue
                ahrs.request_yaw_reset();
            }
            if (ekf_check_state.fail_count == (EKF_CHECK_ITERATIONS_MAX-1)) {
                // we are just about to declare a EKF failsafe, ask the EKF if we can
                // change lanes to resolve the issue
                ahrs.check_lane_switch();
            }
            // if counter above max then trigger failsafe
            if (ekf_check_state.fail_count >= EKF_CHECK_ITERATIONS_MAX) {
                // limit count from climbing too high
                ekf_check_state.fail_count = EKF_CHECK_ITERATIONS_MAX;
                ekf_check_state.bad_variance = true;
                AP::logger().Write_Error(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_BAD_VARIANCE);
                // send message to gcs
                if ((AP_HAL::millis() - ekf_check_state.last_warn_time) > EKF_CHECK_WARNING_TIME) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"EKF variance");
                    ekf_check_state.last_warn_time = AP_HAL::millis();
                }
                failsafe_ekf_event();
            }
        }
    } else {
        // reduce counter
        if (ekf_check_state.fail_count > 0) {
            ekf_check_state.fail_count--;

            // if compass is flagged as bad and the counter reaches zero then clear flag
            if (ekf_check_state.bad_variance && ekf_check_state.fail_count == 0) {
                ekf_check_state.bad_variance = false;
                AP::logger().Write_Error(LogErrorSubsystem::EKFCHECK, LogErrorCode::EKFCHECK_VARIANCE_CLEARED);
                // clear failsafe
                failsafe_ekf_off_event();
            }
        }
    }

    // set AP_Notify flags
    AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;

    // To-Do: add ekf variances to extended status
}

// ekf_over_threshold - returns true if the ekf's variance are over the tolerance
bool Copter::ekf_over_threshold()
{
    // return false immediately if disabled
    if (g.fs_ekf_thresh <= 0.0f) {
        return false;
    }

    // use EKF to get variance
    float position_variance, vel_variance, height_variance, tas_variance;
    Vector3f mag_variance;
    ahrs.get_variances(vel_variance, position_variance, height_variance, mag_variance, tas_variance);

    const float mag_max = fmaxf(fmaxf(mag_variance.x,mag_variance.y),mag_variance.z);

    // return true if two of compass, velocity and position variances are over the threshold OR velocity variance is twice the threshold
    uint8_t over_thresh_count = 0;
    if (mag_max >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }

    bool optflow_healthy = false;
#if AP_OPTICALFLOW_ENABLED
    optflow_healthy = optflow.healthy();
#endif
    if (!optflow_healthy && (vel_variance >= (2.0f * g.fs_ekf_thresh))) {
        over_thresh_count += 2;
    } else if (vel_variance >= g.fs_ekf_thresh) {
        over_thresh_count++;
    }

    if ((position_variance >= g.fs_ekf_thresh && over_thresh_count >= 1) || over_thresh_count >= 2) {
        return true;
    }

    return false;
}


// failsafe_ekf_event - perform ekf failsafe
void Copter::failsafe_ekf_event()
{
    // return immediately if ekf failsafe already triggered
    if (failsafe.ekf) {
        return;
    }

    // EKF failsafe event has occurred
    failsafe.ekf = true;
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_OCCURRED);

    // if disarmed take no action
    if (!motors->armed()) {
        return;
    }

    // sometimes LAND *does* require GPS so ensure we are in non-GPS land
    if (flightmode->mode_number() == Mode::Number::LAND && landing_with_GPS()) {
        mode_land.do_not_use_GPS();
        return;
    }

    // does this mode require position?
    if (!copter.flightmode->requires_GPS() && (g.fs_ekf_action != FS_EKF_ACTION_LAND_EVEN_STABILIZE)) {
        return;
    }

    // take action based on fs_ekf_action parameter
    switch (g.fs_ekf_action) {
        case FS_EKF_ACTION_ALTHOLD:
            // AltHold
            if (failsafe.radio || !set_mode(Mode::Number::ALT_HOLD, ModeReason::EKF_FAILSAFE)) {
                set_mode_land_with_pause(ModeReason::EKF_FAILSAFE);
            }
            break;
        case FS_EKF_ACTION_LAND:
        case FS_EKF_ACTION_LAND_EVEN_STABILIZE:
        default:
            set_mode_land_with_pause(ModeReason::EKF_FAILSAFE);
            break;
    }

    // set true if ekf action is triggered
    AP_Notify::flags.failsafe_ekf = true;
    gcs().send_text(MAV_SEVERITY_CRITICAL, "EKF Failsafe: changed to %s Mode", flightmode->name());
}

// failsafe_ekf_off_event - actions to take when EKF failsafe is cleared
void Copter::failsafe_ekf_off_event(void)
{
    // return immediately if not in ekf failsafe
    if (!failsafe.ekf) {
        return;
    }

    failsafe.ekf = false;
    if (AP_Notify::flags.failsafe_ekf) {
        AP_Notify::flags.failsafe_ekf = false;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "EKF Failsafe Cleared");
    }
    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_EKFINAV, LogErrorCode::FAILSAFE_RESOLVED);
}

// check for ekf yaw reset and adjust target heading, also log position reset
void Copter::check_ekf_reset()
{
    // check for yaw reset
    float yaw_angle_change_rad;
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        attitude_control->inertial_frame_reset();
        ekfYawReset_ms = new_ekfYawReset_ms;
        AP::logger().Write_Event(LogEvent::EKF_YAW_RESET);
    }

    // check for change in primary EKF, reset attitude target and log.  AC_PosControl handles position target adjustment
    if ((ahrs.get_primary_core_index() != ekf_primary_core) && (ahrs.get_primary_core_index() != -1)) {
        attitude_control->inertial_frame_reset();
        ekf_primary_core = ahrs.get_primary_core_index();
        AP::logger().Write_Error(LogErrorSubsystem::EKF_PRIMARY, LogErrorCode(ekf_primary_core));
        gcs().send_text(MAV_SEVERITY_WARNING, "EKF primary changed:%d", (unsigned)ekf_primary_core);
    }
}

// check for high vibrations affecting altitude control
void Copter::check_vibration()
{
    uint32_t now = AP_HAL::millis();

    // assume checks will succeed
    bool innovation_checks_valid = true;

    // check if vertical velocity and position innovations are positive (NKF3.IVD & NKF3.IPD are both positive)
    Vector3f vel_innovation;
    Vector3f pos_innovation;
    Vector3f mag_innovation;
    float tas_innovation;
    float yaw_innovation;
    if (!ahrs.get_innovations(vel_innovation, pos_innovation, mag_innovation, tas_innovation, yaw_innovation)) {
        innovation_checks_valid = false;
    }
    const bool innov_velD_posD_positive = is_positive(vel_innovation.z) && is_positive(pos_innovation.z);

    // check if vertical velocity variance is at least 1 (NK4.SV >= 1.0)
    float position_variance, vel_variance, height_variance, tas_variance;
    Vector3f mag_variance;
    if (!ahrs.get_variances(vel_variance, position_variance, height_variance, mag_variance, tas_variance)) {
        innovation_checks_valid = false;
    }

    const bool is_vibration_affected = ahrs.is_vibration_affected();
    const bool bad_vibe_detected = (innovation_checks_valid && innov_velD_posD_positive && (vel_variance > 1.0f)) || is_vibration_affected;
    const bool do_bad_vibe_actions = (g2.fs_vibe_enabled == 1) && bad_vibe_detected && motors->armed();

    if (!vibration_check.high_vibes) {
        // initialise timers
        if (!do_bad_vibe_actions) {
            vibration_check.start_ms = now;
        }
        // check if failure has persisted for at least 1 second
        if (now - vibration_check.start_ms > 1000) {
            // switch ekf to use resistant gains
            vibration_check.clear_ms = 0;
            vibration_check.high_vibes = true;
            pos_control->set_vibe_comp(true);
            AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_VIBE, LogErrorCode::FAILSAFE_OCCURRED);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Vibration compensation ON");
        }
    } else {
        // initialise timer
        if (do_bad_vibe_actions) {
            vibration_check.clear_ms = now;
        }
        // turn off vibration compensation after 15 seconds
        if (now - vibration_check.clear_ms > 15000) {
            // restore ekf gains, reset timers and update user
            vibration_check.start_ms = 0;
            vibration_check.high_vibes = false;
            pos_control->set_vibe_comp(false);
            vibration_check.clear_ms = 0;
            AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_VIBE, LogErrorCode::FAILSAFE_RESOLVED);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Vibration compensation OFF");
        }
    }

    return;
}
Esc_calibration.cpp
#include "Copter.h"

/*****************************************************************************
* Functions to check and perform ESC calibration
*****************************************************************************/

#define ESC_CALIBRATION_HIGH_THROTTLE   950

// check if we should enter esc calibration mode
void Copter::esc_calibration_startup_check()
{
    if (motors->is_brushed_pwm_type()) {
        // ESC cal not valid for brushed motors
        return;
    }

#if FRAME_CONFIG != HELI_FRAME
    // delay up to 2 second for first radio input
    uint8_t i = 0;
    while ((i++ < 100) && (last_radio_update_ms == 0)) {
        hal.scheduler->delay(20);
        read_radio();
    }

    // exit immediately if pre-arm rc checks fail
    if (!arming.rc_calibration_checks(true)) {
        // clear esc flag for next time
        if ((g.esc_calibrate != ESCCalibrationModes::ESCCAL_NONE) && (g.esc_calibrate != ESCCalibrationModes::ESCCAL_DISABLED)) {
            g.esc_calibrate.set_and_save(ESCCalibrationModes::ESCCAL_NONE);
        }
        return;
    }

    // check ESC parameter
    switch (g.esc_calibrate) {
        case ESCCalibrationModes::ESCCAL_NONE:
            // check if throttle is high
            if (channel_throttle->get_control_in() >= ESC_CALIBRATION_HIGH_THROTTLE) {
                // we will enter esc_calibrate mode on next reboot
                g.esc_calibrate.set_and_save(ESCCalibrationModes::ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH);
                // send message to gcs
                gcs().send_text(MAV_SEVERITY_CRITICAL,"ESC calibration: Restart board");
                // turn on esc calibration notification
                AP_Notify::flags.esc_calibration = true;
                // block until we restart
                while(1) { hal.scheduler->delay(5); }
            }
            break;
        case ESCCalibrationModes::ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH:
            // check if throttle is high
            if (channel_throttle->get_control_in() >= ESC_CALIBRATION_HIGH_THROTTLE) {
                // pass through pilot throttle to escs
                esc_calibration_passthrough();
            }
            break;
        case ESCCalibrationModes::ESCCAL_PASSTHROUGH_ALWAYS:
            // pass through pilot throttle to escs
            esc_calibration_passthrough();
            break;
        case ESCCalibrationModes::ESCCAL_AUTO:
            // perform automatic ESC calibration
            esc_calibration_auto();
            break;
        case ESCCalibrationModes::ESCCAL_DISABLED:
        default:
            // do nothing
            break;
    }

    // clear esc flag for next time
    if (g.esc_calibrate != ESCCalibrationModes::ESCCAL_DISABLED) {
        g.esc_calibrate.set_and_save(ESCCalibrationModes::ESCCAL_NONE);
    }
#endif  // FRAME_CONFIG != HELI_FRAME
}

// esc_calibration_passthrough - pass through pilot throttle to escs
void Copter::esc_calibration_passthrough()
{
#if FRAME_CONFIG != HELI_FRAME
    // send message to GCS
    gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Passing pilot throttle to ESCs");

    esc_calibration_setup();

    while(1) {
        // flash LEDs
        esc_calibration_notify();

        // read pilot input
        read_radio();

        // we run at high rate to make oneshot ESCs happy. Normal ESCs
        // will only see pulses at the RC_SPEED
        hal.scheduler->delay(3);

        // pass through to motors
        SRV_Channels::cork();
        motors->set_throttle_passthrough_for_esc_calibration(channel_throttle->get_control_in() * 0.001f);
        SRV_Channels::push();
    }
#endif  // FRAME_CONFIG != HELI_FRAME
}

// esc_calibration_auto - calibrate the ESCs automatically using a timer and no pilot input
void Copter::esc_calibration_auto()
{
#if FRAME_CONFIG != HELI_FRAME
    // send message to GCS
    gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Auto calibration");

    esc_calibration_setup();

    // raise throttle to maximum
    SRV_Channels::cork();
    motors->set_throttle_passthrough_for_esc_calibration(1.0f);
    SRV_Channels::push();

    // delay for 5 seconds while outputting pulses
    uint32_t tstart = millis();
    while (millis() - tstart < 5000) {
        SRV_Channels::cork();
        motors->set_throttle_passthrough_for_esc_calibration(1.0f);
        SRV_Channels::push();
        esc_calibration_notify();
        hal.scheduler->delay(3);
    }

    // block until we restart
    while(1) {
        SRV_Channels::cork();
        motors->set_throttle_passthrough_for_esc_calibration(0.0f);
        SRV_Channels::push();
        esc_calibration_notify();
        hal.scheduler->delay(3);
    }
#endif // FRAME_CONFIG != HELI_FRAME
}

// flash LEDs to notify the user that ESC calibration is happening
void Copter::esc_calibration_notify()
{
    AP_Notify::flags.esc_calibration = true;
    uint32_t now = AP_HAL::millis();
    if (now - esc_calibration_notify_update_ms > 20) {
        esc_calibration_notify_update_ms = now;
        notify.update();
    }
}

void Copter::esc_calibration_setup()
{
    // clear esc flag for next time
    g.esc_calibrate.set_and_save(ESCCAL_NONE);

    if (motors->is_normal_pwm_type()) {
        // run at full speed for oneshot ESCs (actually done on push)
        motors->set_update_rate(g.rc_speed);
    } else {
        // reduce update rate to motors to 50Hz
        motors->set_update_rate(50);
    }

    // disable safety if requested
    BoardConfig.init_safety();

    // wait for safety switch to be pressed
    uint32_t tstart = 0;
    while (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        const uint32_t tnow = AP_HAL::millis();
        if (tnow - tstart >= 5000) {
            gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Push safety switch");
            tstart = tnow;
        }
        esc_calibration_notify();
        hal.scheduler->delay(3);
    }

    // arm and enable motors
    motors->armed(true);
    SRV_Channels::enable_by_mask(motors->get_motor_mask());
    hal.util->set_soft_armed(true);
}
Events.cpp
#include "Copter.h"

/*****************************************************************************
* Functions to check and perform ESC calibration
*****************************************************************************/

#define ESC_CALIBRATION_HIGH_THROTTLE   950

// check if we should enter esc calibration mode
void Copter::esc_calibration_startup_check()
{
    if (motors->is_brushed_pwm_type()) {
        // ESC cal not valid for brushed motors
        return;
    }

#if FRAME_CONFIG != HELI_FRAME
    // delay up to 2 second for first radio input
    uint8_t i = 0;
    while ((i++ < 100) && (last_radio_update_ms == 0)) {
        hal.scheduler->delay(20);
        read_radio();
    }

    // exit immediately if pre-arm rc checks fail
    if (!arming.rc_calibration_checks(true)) {
        // clear esc flag for next time
        if ((g.esc_calibrate != ESCCalibrationModes::ESCCAL_NONE) && (g.esc_calibrate != ESCCalibrationModes::ESCCAL_DISABLED)) {
            g.esc_calibrate.set_and_save(ESCCalibrationModes::ESCCAL_NONE);
        }
        return;
    }

    // check ESC parameter
    switch (g.esc_calibrate) {
        case ESCCalibrationModes::ESCCAL_NONE:
            // check if throttle is high
            if (channel_throttle->get_control_in() >= ESC_CALIBRATION_HIGH_THROTTLE) {
                // we will enter esc_calibrate mode on next reboot
                g.esc_calibrate.set_and_save(ESCCalibrationModes::ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH);
                // send message to gcs
                gcs().send_text(MAV_SEVERITY_CRITICAL,"ESC calibration: Restart board");
                // turn on esc calibration notification
                AP_Notify::flags.esc_calibration = true;
                // block until we restart
                while(1) { hal.scheduler->delay(5); }
            }
            break;
        case ESCCalibrationModes::ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH:
            // check if throttle is high
            if (channel_throttle->get_control_in() >= ESC_CALIBRATION_HIGH_THROTTLE) {
                // pass through pilot throttle to escs
                esc_calibration_passthrough();
            }
            break;
        case ESCCalibrationModes::ESCCAL_PASSTHROUGH_ALWAYS:
            // pass through pilot throttle to escs
            esc_calibration_passthrough();
            break;
        case ESCCalibrationModes::ESCCAL_AUTO:
            // perform automatic ESC calibration
            esc_calibration_auto();
            break;
        case ESCCalibrationModes::ESCCAL_DISABLED:
        default:
            // do nothing
            break;
    }

    // clear esc flag for next time
    if (g.esc_calibrate != ESCCalibrationModes::ESCCAL_DISABLED) {
        g.esc_calibrate.set_and_save(ESCCalibrationModes::ESCCAL_NONE);
    }
#endif  // FRAME_CONFIG != HELI_FRAME
}

// esc_calibration_passthrough - pass through pilot throttle to escs
void Copter::esc_calibration_passthrough()
{
#if FRAME_CONFIG != HELI_FRAME
    // send message to GCS
    gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Passing pilot throttle to ESCs");

    esc_calibration_setup();

    while(1) {
        // flash LEDs
        esc_calibration_notify();

        // read pilot input
        read_radio();

        // we run at high rate to make oneshot ESCs happy. Normal ESCs
        // will only see pulses at the RC_SPEED
        hal.scheduler->delay(3);

        // pass through to motors
        SRV_Channels::cork();
        motors->set_throttle_passthrough_for_esc_calibration(channel_throttle->get_control_in() * 0.001f);
        SRV_Channels::push();
    }
#endif  // FRAME_CONFIG != HELI_FRAME
}

// esc_calibration_auto - calibrate the ESCs automatically using a timer and no pilot input
void Copter::esc_calibration_auto()
{
#if FRAME_CONFIG != HELI_FRAME
    // send message to GCS
    gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Auto calibration");

    esc_calibration_setup();

    // raise throttle to maximum
    SRV_Channels::cork();
    motors->set_throttle_passthrough_for_esc_calibration(1.0f);
    SRV_Channels::push();

    // delay for 5 seconds while outputting pulses
    uint32_t tstart = millis();
    while (millis() - tstart < 5000) {
        SRV_Channels::cork();
        motors->set_throttle_passthrough_for_esc_calibration(1.0f);
        SRV_Channels::push();
        esc_calibration_notify();
        hal.scheduler->delay(3);
    }

    // block until we restart
    while(1) {
        SRV_Channels::cork();
        motors->set_throttle_passthrough_for_esc_calibration(0.0f);
        SRV_Channels::push();
        esc_calibration_notify();
        hal.scheduler->delay(3);
    }
#endif // FRAME_CONFIG != HELI_FRAME
}

// flash LEDs to notify the user that ESC calibration is happening
void Copter::esc_calibration_notify()
{
    AP_Notify::flags.esc_calibration = true;
    uint32_t now = AP_HAL::millis();
    if (now - esc_calibration_notify_update_ms > 20) {
        esc_calibration_notify_update_ms = now;
        notify.update();
    }
}

void Copter::esc_calibration_setup()
{
    // clear esc flag for next time
    g.esc_calibrate.set_and_save(ESCCAL_NONE);

    if (motors->is_normal_pwm_type()) {
        // run at full speed for oneshot ESCs (actually done on push)
        motors->set_update_rate(g.rc_speed);
    } else {
        // reduce update rate to motors to 50Hz
        motors->set_update_rate(50);
    }

    // disable safety if requested
    BoardConfig.init_safety();

    // wait for safety switch to be pressed
    uint32_t tstart = 0;
    while (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        const uint32_t tnow = AP_HAL::millis();
        if (tnow - tstart >= 5000) {
            gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Push safety switch");
            tstart = tnow;
        }
        esc_calibration_notify();
        hal.scheduler->delay(3);
    }

    // arm and enable motors
    motors->armed(true);
    SRV_Channels::enable_by_mask(motors->get_motor_mask());
    hal.util->set_soft_armed(true);
}
Failsafe.cpp
#include "Copter.h"

//
//  failsafe support
//  Andrew Tridgell, December 2011
//
//  our failsafe strategy is to detect main loop lockup and disarm the motors
//

static bool failsafe_enabled = false;
static uint16_t failsafe_last_ticks;
static uint32_t failsafe_last_timestamp;
static bool in_failsafe;

//
// failsafe_enable - enable failsafe
//
void Copter::failsafe_enable()
{
    failsafe_enabled = true;
    failsafe_last_timestamp = micros();
}

//
// failsafe_disable - used when we know we are going to delay the mainloop significantly
//
void Copter::failsafe_disable()
{
    failsafe_enabled = false;
}

//
//  failsafe_check - this function is called from the core timer interrupt at 1kHz.
//
void Copter::failsafe_check()
{
    uint32_t tnow = AP_HAL::micros();

    const uint16_t ticks = scheduler.ticks();
    if (ticks != failsafe_last_ticks) {
        // the main loop is running, all is OK
        failsafe_last_ticks = ticks;
        failsafe_last_timestamp = tnow;
        if (in_failsafe) {
            in_failsafe = false;
            AP::logger().Write_Error(LogErrorSubsystem::CPU, LogErrorCode::FAILSAFE_RESOLVED);
        }
        return;
    }

    if (!in_failsafe && failsafe_enabled && tnow - failsafe_last_timestamp > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors->
        in_failsafe = true;
        // reduce motors to minimum (we do not immediately disarm because we want to log the failure)
        if (motors->armed()) {
            motors->output_min();
        }

        AP::logger().Write_Error(LogErrorSubsystem::CPU, LogErrorCode::FAILSAFE_OCCURRED);
    }

    if (failsafe_enabled && in_failsafe && tnow - failsafe_last_timestamp > 1000000) {
        // disarm motors every second
        failsafe_last_timestamp = tnow;
        if(motors->armed()) {
            motors->armed(false);
            motors->output();
        }
    }
}


#if ADVANCED_FAILSAFE == ENABLED
/*
  check for AFS failsafe check
*/
void Copter::afs_fs_check(void)
{
    // perform AFS failsafe checks
#if AC_FENCE
    const bool fence_breached = fence.get_breaches() != 0;
#else
    const bool fence_breached = false;
#endif
    g2.afs.check(fence_breached, last_radio_update_ms);
}
#endif
Fence.cpp
#include "Copter.h"

// Code to integrate AC_Fence library with main ArduCopter code

#if AC_FENCE == ENABLED

// fence_check - ask fence library to check for breaches and initiate the response
// called at 1hz
void Copter::fence_check()
{
    const uint8_t orig_breaches = fence.get_breaches();

    // check for new breaches; new_breaches is bitmask of fence types breached
    const uint8_t new_breaches = fence.check();

    // we still don't do anything when disarmed, but we do check for fence breaches.
    // fence pre-arm check actually checks if any fence has been breached 
    // that's not ever going to be true if we don't call check on AP_Fence while disarmed.
    if (!motors->armed()) {
        return;
    }

    // if there is a new breach take action
    if (new_breaches) {

        if (!copter.ap.land_complete) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Fence Breached");
        }

        // if the user wants some kind of response and motors are armed
        uint8_t fence_act = fence.get_action();
        if (fence_act != AC_FENCE_ACTION_REPORT_ONLY ) {

            // disarm immediately if we think we are on the ground or in a manual flight mode with zero throttle
            // don't disarm if the high-altitude fence has been broken because it's likely the user has pulled their throttle to zero to bring it down
            if (ap.land_complete || (flightmode->has_manual_throttle() && ap.throttle_zero && !failsafe.radio && ((fence.get_breaches() & AC_FENCE_TYPE_ALT_MAX)== 0))){
                arming.disarm(AP_Arming::Method::FENCEBREACH);

            } else {

                // if more than 100m outside the fence just force a land
                if (fence.get_breach_distance(new_breaches) > AC_FENCE_GIVE_UP_DISTANCE) {
                    set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                } else {
                    switch (fence_act) {
                    case AC_FENCE_ACTION_RTL_AND_LAND:
                    default:
                        // switch to RTL, if that fails then Land
                        if (!set_mode(Mode::Number::RTL, ModeReason::FENCE_BREACHED)) {
                            set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        }
                        break;
                    case AC_FENCE_ACTION_ALWAYS_LAND:
                        // if always land option mode is specified, land
                        set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        break;
                    case AC_FENCE_ACTION_SMART_RTL:
                        // Try SmartRTL, if that fails, RTL, if that fails Land
                        if (!set_mode(Mode::Number::SMART_RTL, ModeReason::FENCE_BREACHED)) {
                            if (!set_mode(Mode::Number::RTL, ModeReason::FENCE_BREACHED)) {
                                set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                            }
                        }
                        break;
                    case AC_FENCE_ACTION_BRAKE:
                        // Try Brake, if that fails Land
                        if (!set_mode(Mode::Number::BRAKE, ModeReason::FENCE_BREACHED)) {
                            set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        }
                        break;
                    case AC_FENCE_ACTION_SMART_RTL_OR_LAND:
                        // Try SmartRTL, if that fails, Land
                        if (!set_mode(Mode::Number::SMART_RTL, ModeReason::FENCE_BREACHED)) {
                            set_mode(Mode::Number::LAND, ModeReason::FENCE_BREACHED);
                        }
                        break;
                    }
                }
            }
        }

        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode(new_breaches));

    } else if (orig_breaches) {
        // record clearing of breach
        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_FENCE, LogErrorCode::ERROR_RESOLVED);
    }
}

#endif
Heli.cpp
#include "Copter.h"

// Traditional helicopter variables and functions

#if FRAME_CONFIG == HELI_FRAME

#ifndef HELI_DYNAMIC_FLIGHT_SPEED_MIN
 #define HELI_DYNAMIC_FLIGHT_SPEED_MIN      250     // we are in "dynamic flight" when the speed is over 2.5m/s for 2 seconds
#endif

// counter to control dynamic flight profile
static int8_t heli_dynamic_flight_counter;

// heli_init - perform any special initialisation required for the tradheli
void Copter::heli_init()
{
    // pre-load stab col values as mode is initialized as Stabilize, but stabilize_init() function is not run on start-up.
    input_manager.set_use_stab_col(true);
    input_manager.set_stab_col_ramp(1.0);
}

// heli_check_dynamic_flight - updates the dynamic_flight flag based on our horizontal velocity
// should be called at 50hz
void Copter::check_dynamic_flight(void)
{
    if (motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED ||
        flightmode->is_landing()) {
        heli_dynamic_flight_counter = 0;
        heli_flags.dynamic_flight = false;
        return;
    }

    bool moving = false;

    // with GPS lock use inertial nav to determine if we are moving
    if (position_ok()) {
        // get horizontal speed
        const float speed = inertial_nav.get_speed_xy_cms();
        moving = (speed >= HELI_DYNAMIC_FLIGHT_SPEED_MIN);
    }else{
        // with no GPS lock base it on throttle and forward lean angle
        moving = (motors->get_throttle() > 0.8f || ahrs.pitch_sensor < -1500);
    }

    if (!moving && rangefinder_state.enabled && rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) {
        // when we are more than 2m from the ground with good
        // rangefinder lock consider it to be dynamic flight
        moving = (rangefinder.distance_cm_orient(ROTATION_PITCH_270) > 200);
    }
    
    if (moving) {
        // if moving for 2 seconds, set the dynamic flight flag
        if (!heli_flags.dynamic_flight) {
            heli_dynamic_flight_counter++;
            if (heli_dynamic_flight_counter >= 100) {
                heli_flags.dynamic_flight = true;
                heli_dynamic_flight_counter = 100;
            }
        }
    }else{
        // if not moving for 2 seconds, clear the dynamic flight flag
        if (heli_flags.dynamic_flight) {
            if (heli_dynamic_flight_counter > 0) {
                heli_dynamic_flight_counter--;
            }else{
                heli_flags.dynamic_flight = false;
            }
        }
    }
}

// update_heli_control_dynamics - pushes several important factors up into AP_MotorsHeli.
// should be run between the rate controller and the servo updates.
void Copter::update_heli_control_dynamics(void)
{

    if (!motors->using_leaky_integrator()) {
        //turn off leaky_I
        attitude_control->use_leaky_i(false);
        if (ap.land_complete || ap.land_complete_maybe) {
            motors->set_land_complete(true);
        } else {
            motors->set_land_complete(false);
        }
    } else {
        // Use Leaky_I if we are not moving fast
        attitude_control->use_leaky_i(!heli_flags.dynamic_flight);
        motors->set_land_complete(false);
    }

    if (ap.land_complete || (is_zero(motors->get_desired_rotor_speed()))){
        // if we are landed or there is no rotor power demanded, decrement slew scalar
        hover_roll_trim_scalar_slew--;        
    } else {
        // if we are not landed and motor power is demanded, increment slew scalar
        hover_roll_trim_scalar_slew++;
    }
    hover_roll_trim_scalar_slew = constrain_int16(hover_roll_trim_scalar_slew, 0, scheduler.get_loop_rate_hz());

    // set hover roll trim scalar, will ramp from 0 to 1 over 1 second after we think helicopter has taken off
    attitude_control->set_hover_roll_trim_scalar((float) hover_roll_trim_scalar_slew/(float) scheduler.get_loop_rate_hz());
}

bool Copter::should_use_landing_swash() const
{
    if (flightmode->has_manual_throttle() ||
        flightmode->mode_number() == Mode::Number::DRIFT) {
        // manual modes always uses full swash range
        return false;
    }
    if (flightmode->is_landing()) {
        // landing with non-manual throttle mode always uses limit swash range
        return true;
    }
    if (ap.land_complete) {
        // when landed in non-manual throttle mode limit swash range
        return true;
    }
    if (!ap.auto_armed) {
        // when waiting to takeoff in non-manual throttle mode limit swash range
        return true;
    }
    if (!heli_flags.dynamic_flight) {
        // Just in case we are unsure of being in non-manual throttle
        // mode, limit swash range in low speed and hovering flight.
        // This will catch any non-manual throttle mode attempting a
        // landing and driving the collective too low before the land
        // complete flag is set.
        return true;
    }
    return false;
}

// heli_update_landing_swash - sets swash plate flag so higher minimum is used when landed or landing
// should be called soon after update_land_detector in main code
void Copter::heli_update_landing_swash()
{
    motors->set_collective_for_landing(should_use_landing_swash());
    update_collective_low_flag(channel_throttle->get_control_in());
}

// convert motor interlock switch's position to desired rotor speed expressed as a value from 0 to 1
// returns zero if motor interlock auxiliary switch hasn't been defined
float Copter::get_pilot_desired_rotor_speed() const
{
    RC_Channel *rc_ptr = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK);
    if (rc_ptr != nullptr) {
        rc_ptr->set_range(1000);
        return (float)rc_ptr->get_control_in() * 0.001f;
    }
    return 0.0f;
}

// heli_update_rotor_speed_targets - reads pilot input and passes new rotor speed targets to heli motors object
void Copter::heli_update_rotor_speed_targets()
{

    static bool rotor_runup_complete_last = false;

    // get rotor control method
    uint8_t rsc_control_mode = motors->get_rsc_mode();

    switch (rsc_control_mode) {
        case ROTOR_CONTROL_MODE_PASSTHROUGH:
            // pass through pilot desired rotor speed from the RC
            if (get_pilot_desired_rotor_speed() > 0.01) {
                ap.motor_interlock_switch = true;
                motors->set_desired_rotor_speed(get_pilot_desired_rotor_speed());
            } else {
                ap.motor_interlock_switch = false;
                motors->set_desired_rotor_speed(0.0f);
            }
            break;
        case ROTOR_CONTROL_MODE_SETPOINT:
        case ROTOR_CONTROL_MODE_THROTTLECURVE:
        case ROTOR_CONTROL_MODE_AUTOTHROTTLE:
            if (motors->get_interlock()) {
                motors->set_desired_rotor_speed(motors->get_rsc_setpoint());
            }else{
                motors->set_desired_rotor_speed(0.0f);
            }
            break;
    }

    // when rotor_runup_complete changes to true, log event
    if (!rotor_runup_complete_last && motors->rotor_runup_complete()){
        AP::logger().Write_Event(LogEvent::ROTOR_RUNUP_COMPLETE);
    } else if (rotor_runup_complete_last && !motors->rotor_runup_complete()){
        AP::logger().Write_Event(LogEvent::ROTOR_SPEED_BELOW_CRITICAL);
    }
    rotor_runup_complete_last = motors->rotor_runup_complete();
}


// heli_update_autorotation - determines if aircraft is in autorotation and sets motors flag and switches
// to autorotation flight mode if manual collective is not being used.
void Copter::heli_update_autorotation()
{
#if MODE_AUTOROTATE_ENABLED == ENABLED
    // check if flying and interlock disengaged
    if (!ap.land_complete && !motors->get_interlock() && g2.arot.is_enable()) {
        if (!flightmode->has_manual_throttle()) {
            // set autonomous autorotation flight mode
            set_mode(Mode::Number::AUTOROTATE, ModeReason::AUTOROTATION_START);
        }
        // set flag to facilitate both auto and manual autorotations
        heli_flags.in_autorotation = true;
    } else {
        heli_flags.in_autorotation = false;
    }

    // sets autorotation flags through out libraries
    heli_set_autorotation(heli_flags.in_autorotation);
    if (!ap.land_complete && g2.arot.is_enable()) {
        motors->set_enable_bailout(true);
    } else {
        motors->set_enable_bailout(false);
    }
#else
    heli_flags.in_autorotation = false;
    motors->set_enable_bailout(false);
#endif
}

#if MODE_AUTOROTATE_ENABLED == ENABLED
// heli_set_autorotation - set the autorotation flag throughout libraries
void Copter::heli_set_autorotation(bool autorotation)
{
    motors->set_in_autorotation(autorotation);
}
#endif

// update collective low flag.  Use a debounce time of 400 milliseconds.
void Copter::update_collective_low_flag(int16_t throttle_control)
{
    static uint32_t last_nonzero_collective_ms = 0;
    uint32_t tnow_ms = millis();

    if (throttle_control > 0) {
        last_nonzero_collective_ms = tnow_ms;
        heli_flags.coll_stk_low = false;
    } else if (tnow_ms - last_nonzero_collective_ms > 400) {
        heli_flags.coll_stk_low = true;
    }
}

#endif  // FRAME_CONFIG == HELI_FRAME
Inertia.cpp
#include "Copter.h"

// read_inertia - read inertia in from accelerometers
void Copter::read_inertia()
{
    // inertial altitude estimates. Use barometer climb rate during high vibrations
    inertial_nav.update(vibration_check.high_vibes);

    // pull position from ahrs
    Location loc;
    ahrs.get_location(loc);
    current_loc.lat = loc.lat;
    current_loc.lng = loc.lng;

    // exit immediately if we do not have an altitude estimate
    if (!inertial_nav.get_filter_status().flags.vert_pos) {
        return;
    }

    // current_loc.alt is alt-above-home, converted from inertial nav's alt-above-ekf-origin
    const int32_t alt_above_origin_cm = inertial_nav.get_position_z_up_cm();
    current_loc.set_alt_cm(alt_above_origin_cm, Location::AltFrame::ABOVE_ORIGIN);
    if (!ahrs.home_is_set() || !current_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
        // if home has not been set yet we treat alt-above-origin as alt-above-home
        current_loc.set_alt_cm(alt_above_origin_cm, Location::AltFrame::ABOVE_HOME);
    }
}
Land_detector.cpp
#include "Copter.h"

// Code to detect a crash main ArduCopter code
#define LAND_CHECK_ANGLE_ERROR_DEG  30.0f       // maximum angle error to be considered landing
#define LAND_CHECK_LARGE_ANGLE_CD   1500.0f     // maximum angle target to be considered landing
#define LAND_CHECK_ACCEL_MOVING     3.0f        // maximum acceleration after subtracting gravity


// counter to verify landings
static uint32_t land_detector_count = 0;

// run land and crash detectors
// called at MAIN_LOOP_RATE
void Copter::update_land_and_crash_detectors()
{
    // update 1hz filtered acceleration
    Vector3f accel_ef = ahrs.get_accel_ef_blended();
    accel_ef.z += GRAVITY_MSS;
    land_accel_ef_filter.apply(accel_ef, scheduler.get_loop_period_s());

    update_land_detector();

#if PARACHUTE == ENABLED
    // check parachute
    parachute_check();
#endif

    crash_check();
    thrust_loss_check();
    yaw_imbalance_check();
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// called at MAIN_LOOP_RATE
void Copter::update_land_detector()
{
    // land detector can not use the following sensors because they are unreliable during landing
    // barometer altitude :                 ground effect can cause errors larger than 4m
    // EKF vertical velocity or altitude :  poor barometer and large acceleration from ground impact
    // earth frame angle or angle error :   landing on an uneven surface will force the airframe to match the ground angle
    // gyro output :                        on uneven surface the airframe may rock back an forth after landing
    // range finder :                       tend to be problematic at very short distances
    // input throttle :                     in slow land the input throttle may be only slightly less than hover

    if (!motors->armed()) {
        // if disarmed, always landed.
        set_land_complete(true);
    } else if (ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME
        // if rotor speed and collective pitch are high then clear landing flag
        if (motors->get_takeoff_collective() && motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
#else
        // if throttle output is high then clear landing flag
        if (motors->get_throttle() > get_non_takeoff_throttle()) {
#endif
            set_land_complete(false);
        }
    } else if (standby_active) {
        // land detector will not run in standby mode
        land_detector_count = 0;
    } else {

#if FRAME_CONFIG == HELI_FRAME
        // check for both manual collective modes and modes that use altitude hold. For manual collective (called throttle
        // because multi's use throttle), check that collective pitch is below land min collective position or throttle stick is zero.
        // Including the throttle zero check will ensure the conditions where stabilize stick zero position was not below collective min. For modes
        // that use altitude hold, check that the pilot is commanding a descent and collective is at min allowed for altitude hold modes.

        // check if landing
        const bool landing = flightmode->is_landing();
        bool motor_at_lower_limit = (flightmode->has_manual_throttle() && (motors->get_below_land_min_coll() || heli_flags.coll_stk_low) && fabsf(ahrs.get_roll()) < M_PI/2.0f)
                                    || ((!force_flying || landing) && motors->limit.throttle_lower && pos_control->get_vel_desired_cms().z < 0.0f);
#else
        // check that the average throttle output is near minimum (less than 12.5% hover throttle)
        bool motor_at_lower_limit = motors->limit.throttle_lower && attitude_control->is_throttle_mix_min();
#endif

        uint8_t land_detector_scalar = 1;
#if LANDING_GEAR_ENABLED == ENABLED
        if (landinggear.get_wow_state() != AP_LandingGear::LG_WOW_UNKNOWN) {
            // we have a WoW sensor so lets loosen the strictness of the landing detector
            land_detector_scalar = 2;
        }
#endif

        // check that the airframe is not accelerating (not falling or braking after fast forward flight)
        bool accel_stationary = (land_accel_ef_filter.get().length() <= LAND_DETECTOR_ACCEL_MAX * land_detector_scalar);

        // check that vertical speed is within 1m/s of zero
        bool descent_rate_low = fabsf(inertial_nav.get_velocity_z_up_cms()) < 100 * land_detector_scalar;

        // if we have a healthy rangefinder only allow landing detection below 2 meters
        bool rangefinder_check = (!rangefinder_alt_ok() || rangefinder_state.alt_cm_filt.get() < LAND_RANGEFINDER_MIN_ALT_CM);

        // if we have weight on wheels (WoW) or ambiguous unknown. never no WoW
#if LANDING_GEAR_ENABLED == ENABLED
        const bool WoW_check = (landinggear.get_wow_state() == AP_LandingGear::LG_WOW || landinggear.get_wow_state() == AP_LandingGear::LG_WOW_UNKNOWN);
#else
        const bool WoW_check = true;
#endif

        if (motor_at_lower_limit && accel_stationary && descent_rate_low && rangefinder_check && WoW_check) {
            // landed criteria met - increment the counter and check if we've triggered
            if( land_detector_count < ((float)LAND_DETECTOR_TRIGGER_SEC)*scheduler.get_loop_rate_hz()) {
                land_detector_count++;
            } else {
                set_land_complete(true);
            }
        } else {
            // we've sensed movement up or down so reset land_detector
            land_detector_count = 0;
        }
    }

    set_land_complete_maybe(ap.land_complete || (land_detector_count >= LAND_DETECTOR_MAYBE_TRIGGER_SEC*scheduler.get_loop_rate_hz()));
}

// set land_complete flag and disarm motors if disarm-on-land is configured
void Copter::set_land_complete(bool b)
{
    // if no change, exit immediately
    if( ap.land_complete == b )
        return;

    land_detector_count = 0;

    if(b){
        AP::logger().Write_Event(LogEvent::LAND_COMPLETE);
    } else {
        AP::logger().Write_Event(LogEvent::NOT_LANDED);
    }
    ap.land_complete = b;

#if STATS_ENABLED == ENABLED
    g2.stats.set_flying(!b);
#endif

    // tell AHRS flying state
    set_likely_flying(!b);
    
    // trigger disarm-on-land if configured
    bool disarm_on_land_configured = (g.throttle_behavior & THR_BEHAVE_DISARM_ON_LAND_DETECT) != 0;
    const bool mode_disarms_on_land = flightmode->allows_arming(AP_Arming::Method::LANDING) && !flightmode->has_manual_throttle();

    if (ap.land_complete && motors->armed() && disarm_on_land_configured && mode_disarms_on_land) {
        arming.disarm(AP_Arming::Method::LANDED);
    }
}

// set land complete maybe flag
void Copter::set_land_complete_maybe(bool b)
{
    // if no change, exit immediately
    if (ap.land_complete_maybe == b)
        return;

    if (b) {
        AP::logger().Write_Event(LogEvent::LAND_COMPLETE_MAYBE);
    }
    ap.land_complete_maybe = b;
}

// sets motors throttle_low_comp value depending upon vehicle state
//  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
//  has no effect when throttle is above hover throttle
void Copter::update_throttle_mix()
{
#if FRAME_CONFIG != HELI_FRAME
    // if disarmed or landed prioritise throttle
    if (!motors->armed() || ap.land_complete) {
        attitude_control->set_throttle_mix_min();
        return;
    }

    if (flightmode->has_manual_throttle()) {
        // manual throttle
        if (channel_throttle->get_control_in() <= 0 || air_mode == AirMode::AIRMODE_DISABLED) {
            attitude_control->set_throttle_mix_min();
        } else {
            attitude_control->set_throttle_mix_man();
        }
    } else {
        // autopilot controlled throttle

        // check for aggressive flight requests - requested roll or pitch angle below 15 degrees
        const Vector3f angle_target = attitude_control->get_att_target_euler_cd();
        bool large_angle_request = angle_target.xy().length() > LAND_CHECK_LARGE_ANGLE_CD;

        // check for large external disturbance - angle error over 30 degrees
        const float angle_error = attitude_control->get_att_error_angle_deg();
        bool large_angle_error = (angle_error > LAND_CHECK_ANGLE_ERROR_DEG);

        // check for large acceleration - falling or high turbulence
        const bool accel_moving = (land_accel_ef_filter.get().length() > LAND_CHECK_ACCEL_MOVING);

        // check for requested descent
        bool descent_not_demanded = pos_control->get_vel_desired_cms().z >= 0.0f;

        // check if landing
        const bool landing = flightmode->is_landing();

        if (((large_angle_request || force_flying) && !landing) || large_angle_error || accel_moving || descent_not_demanded) {
            attitude_control->set_throttle_mix_max(pos_control->get_vel_z_control_ratio());
        } else {
            attitude_control->set_throttle_mix_min();
        }
    }
#endif
}
Landing_gear.cpp
#include "Copter.h"

#if LANDING_GEAR_ENABLED == ENABLED

// Run landing gear controller at 10Hz
void Copter::landinggear_update()
{
    // exit immediately if no landing gear output has been enabled
    if (!SRV_Channels::function_assigned(SRV_Channel::k_landing_gear_control)) {
        return;
    }

    // support height based triggering using rangefinder or altitude above ground
    int32_t height_cm = flightmode->get_alt_above_ground_cm();

    // use rangefinder if available
    switch (rangefinder.status_orient(ROTATION_PITCH_270)) {
    case RangeFinder::Status::NotConnected:
    case RangeFinder::Status::NoData:
        // use altitude above home for non-functioning rangefinder
        break;

    case RangeFinder::Status::OutOfRangeLow:
        // altitude is close to zero (gear should deploy)
        height_cm = 0;
        break;

    case RangeFinder::Status::OutOfRangeHigh:
    case RangeFinder::Status::Good:
        // use last good reading
        height_cm = rangefinder_state.alt_cm_filt.get();
        break;
    }

    landinggear.update(height_cm * 0.01f); // convert cm->m for update call
}

#endif // LANDING_GEAR_ENABLED
Mode.cpp
#include "Copter.h"

/*
 * High level calls to set and update flight modes logic for individual
 * flight modes is in control_acro.cpp, control_stabilize.cpp, etc
 */

/*
  constructor for Mode object
 */
Mode::Mode(void) :
    g(copter.g),
    g2(copter.g2),
    wp_nav(copter.wp_nav),
    loiter_nav(copter.loiter_nav),
    pos_control(copter.pos_control),
    inertial_nav(copter.inertial_nav),
    ahrs(copter.ahrs),
    attitude_control(copter.attitude_control),
    motors(copter.motors),
    channel_roll(copter.channel_roll),
    channel_pitch(copter.channel_pitch),
    channel_throttle(copter.channel_throttle),
    channel_yaw(copter.channel_yaw),
    G_Dt(copter.G_Dt)
{ };

// return the static controller object corresponding to supplied mode
Mode *Copter::mode_from_mode_num(const Mode::Number mode)
{
    Mode *ret = nullptr;

    switch (mode) {
#if MODE_ACRO_ENABLED == ENABLED
        case Mode::Number::ACRO:
            ret = &mode_acro;
            break;
#endif

        case Mode::Number::STABILIZE:
            ret = &mode_stabilize;
            break;

        case Mode::Number::ALT_HOLD:
            ret = &mode_althold;
            break;

#if MODE_AUTO_ENABLED == ENABLED
        case Mode::Number::AUTO:
            ret = &mode_auto;
            break;
#endif

#if MODE_CIRCLE_ENABLED == ENABLED
        case Mode::Number::CIRCLE:
            ret = &mode_circle;
            break;
#endif

#if MODE_LOITER_ENABLED == ENABLED
        case Mode::Number::LOITER:
            ret = &mode_loiter;
            break;
#endif

#if MODE_GUIDED_ENABLED == ENABLED
        case Mode::Number::GUIDED:
            ret = &mode_guided;
            break;
#endif

        case Mode::Number::LAND:
            ret = &mode_land;
            break;

#if MODE_RTL_ENABLED == ENABLED
        case Mode::Number::RTL:
            ret = &mode_rtl;
            break;
#endif

#if MODE_DRIFT_ENABLED == ENABLED
        case Mode::Number::DRIFT:
            ret = &mode_drift;
            break;
#endif

#if MODE_SPORT_ENABLED == ENABLED
        case Mode::Number::SPORT:
            ret = &mode_sport;
            break;
#endif

#if MODE_FLIP_ENABLED == ENABLED
        case Mode::Number::FLIP:
            ret = &mode_flip;
            break;
#endif

#if AUTOTUNE_ENABLED == ENABLED
        case Mode::Number::AUTOTUNE:
            ret = &mode_autotune;
            break;
#endif

#if MODE_POSHOLD_ENABLED == ENABLED
        case Mode::Number::POSHOLD:
            ret = &mode_poshold;
            break;
#endif

#if MODE_BRAKE_ENABLED == ENABLED
        case Mode::Number::BRAKE:
            ret = &mode_brake;
            break;
#endif

#if MODE_THROW_ENABLED == ENABLED
        case Mode::Number::THROW:
            ret = &mode_throw;
            break;
#endif

#if HAL_ADSB_ENABLED
        case Mode::Number::AVOID_ADSB:
            ret = &mode_avoid_adsb;
            break;
#endif

#if MODE_GUIDED_NOGPS_ENABLED == ENABLED
        case Mode::Number::GUIDED_NOGPS:
            ret = &mode_guided_nogps;
            break;
#endif

#if MODE_SMARTRTL_ENABLED == ENABLED
        case Mode::Number::SMART_RTL:
            ret = &mode_smartrtl;
            break;
#endif

#if AP_OPTICALFLOW_ENABLED
        case Mode::Number::FLOWHOLD:
            ret = (Mode *)g2.mode_flowhold_ptr;
            break;
#endif

#if MODE_FOLLOW_ENABLED == ENABLED
        case Mode::Number::FOLLOW:
            ret = &mode_follow;
            break;
#endif

#if MODE_ZIGZAG_ENABLED == ENABLED
        case Mode::Number::ZIGZAG:
            ret = &mode_zigzag;
            break;
#endif

#if MODE_SYSTEMID_ENABLED == ENABLED
        case Mode::Number::SYSTEMID:
            ret = (Mode *)g2.mode_systemid_ptr;
            break;
#endif

#if MODE_AUTOROTATE_ENABLED == ENABLED
        case Mode::Number::AUTOROTATE:
            ret = &mode_autorotate;
            break;
#endif

#if MODE_TURTLE_ENABLED == ENABLED
        case Mode::Number::TURTLE:
            ret = &mode_turtle;
            break;
#endif

        default:
            break;
    }

    return ret;
}


// called when an attempt to change into a mode is unsuccessful:
void Copter::mode_change_failed(const Mode *mode, const char *reason)
{
    gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to %s failed: %s", mode->name(), reason);
    AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode->mode_number()));
    // make sad noise
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change_failed = 1;
    }
}

// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was successfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
bool Copter::set_mode(Mode::Number mode, ModeReason reason)
{
    // update last reason
    const ModeReason last_reason = _last_reason;
    _last_reason = reason;

    // return immediately if we are already in the desired mode
    if (mode == flightmode->mode_number()) {
        control_mode_reason = reason;
        // make happy noise
        if (copter.ap.initialised && (reason != last_reason)) {
            AP_Notify::events.user_mode_change = 1;
        }
        return true;
    }

#if MODE_AUTO_ENABLED == ENABLED
    if (mode == Mode::Number::AUTO_RTL) {
        // Special case for AUTO RTL, not a true mode, just AUTO in disguise
        return mode_auto.jump_to_landing_sequence_auto_RTL(reason);
    }
#endif

    Mode *new_flightmode = mode_from_mode_num(mode);
    if (new_flightmode == nullptr) {
        notify_no_such_mode((uint8_t)mode);
        return false;
    }

    bool ignore_checks = !motors->armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter a non-manual throttle mode if the
    // rotor runup is not complete
    if (!ignore_checks && !new_flightmode->has_manual_throttle() &&
        (motors->get_spool_state() == AP_Motors::SpoolState::SPOOLING_UP || motors->get_spool_state() == AP_Motors::SpoolState::SPOOLING_DOWN)) {
        #if MODE_AUTOROTATE_ENABLED == ENABLED
            //if the mode being exited is the autorotation mode allow mode change despite rotor not being at
            //full speed.  This will reduce altitude loss on bail-outs back to non-manual throttle modes
            bool in_autorotation_check = (flightmode != &mode_autorotate || new_flightmode != &mode_autorotate);
        #else
            bool in_autorotation_check = false;
        #endif

        if (!in_autorotation_check) {
            mode_change_failed(new_flightmode, "runup not complete");
            return false;
        }
    }
#endif

#if FRAME_CONFIG != HELI_FRAME
    // ensure vehicle doesn't leap off the ground if a user switches
    // into a manual throttle mode from a non-manual-throttle mode
    // (e.g. user arms in guided, raises throttle to 1300 (not enough to
    // trigger auto takeoff), then switches into manual):
    bool user_throttle = new_flightmode->has_manual_throttle();
#if MODE_DRIFT_ENABLED == ENABLED
    if (new_flightmode == &mode_drift) {
        user_throttle = true;
    }
#endif
    if (!ignore_checks &&
        ap.land_complete &&
        user_throttle &&
        !copter.flightmode->has_manual_throttle() &&
        new_flightmode->get_pilot_desired_throttle() > copter.get_non_takeoff_throttle()) {
        mode_change_failed(new_flightmode, "throttle too high");
        return false;
    }
#endif

    if (!ignore_checks &&
        new_flightmode->requires_GPS() &&
        !copter.position_ok()) {
        mode_change_failed(new_flightmode, "requires position");
        return false;
    }

    // check for valid altitude if old mode did not require it but new one does
    // we only want to stop changing modes if it could make things worse
    if (!ignore_checks &&
        !copter.ekf_alt_ok() &&
        flightmode->has_manual_throttle() &&
        !new_flightmode->has_manual_throttle()) {
        mode_change_failed(new_flightmode, "need alt estimate");
        return false;
    }

    if (!new_flightmode->init(ignore_checks)) {
        mode_change_failed(new_flightmode, "initialisation failed");
        return false;
    }

    // perform any cleanup required by previous flight mode
    exit_mode(flightmode, new_flightmode);

    // store previous flight mode (only used by tradeheli's autorotation)
    prev_control_mode = flightmode->mode_number();

    // update flight mode
    flightmode = new_flightmode;
    control_mode_reason = reason;
    logger.Write_Mode((uint8_t)flightmode->mode_number(), reason);
    gcs().send_message(MSG_HEARTBEAT);

#if HAL_ADSB_ENABLED
    adsb.set_is_auto_mode((mode == Mode::Number::AUTO) || (mode == Mode::Number::RTL) || (mode == Mode::Number::GUIDED));
#endif

#if AC_FENCE == ENABLED
    // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
    // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
    // but it should be harmless to disable the fence temporarily in these situations as well
    fence.manual_recovery_start();
#endif

#if CAMERA == ENABLED
    camera.set_is_auto_mode(flightmode->mode_number() == Mode::Number::AUTO);
#endif

    // update notify object
    notify_flight_mode();

    // make happy noise
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change = 1;
    }

    // return success
    return true;
}

bool Copter::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(Mode::Number) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
#ifdef DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE
    if (reason == ModeReason::GCS_COMMAND && copter.failsafe.radio) {
        // don't allow mode changes while in radio failsafe
        return false;
    }
#endif
    return copter.set_mode(static_cast<Mode::Number>(new_mode), reason);
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Copter::update_flight_mode()
{
    surface_tracking.invalidate_for_logging();  // invalidate surface tracking alt, flight mode will set to true if used

    flightmode->run();
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Copter::exit_mode(Mode *&old_flightmode,
                       Mode *&new_flightmode)
{
    // smooth throttle transition when switching from manual to automatic flight modes
    if (old_flightmode->has_manual_throttle() && !new_flightmode->has_manual_throttle() && motors->armed() && !ap.land_complete) {
        // this assumes all manual flight modes use get_pilot_desired_throttle to translate pilot input to output throttle
        set_accel_throttle_I_from_pilot_throttle();
    }

    // cancel any takeoffs in progress
    old_flightmode->takeoff_stop();

    // perform cleanup required for each flight mode
    old_flightmode->exit();

#if FRAME_CONFIG == HELI_FRAME
    // firmly reset the flybar passthrough to false when exiting acro mode.
    if (old_flightmode == &mode_acro) {
        attitude_control->use_flybar_passthrough(false, false);
        motors->set_acro_tail(false);
    }

    // if we are changing from a mode that did not use manual throttle,
    // stab col ramp value should be pre-loaded to the correct value to avoid a twitch
    // heli_stab_col_ramp should really only be active switching between Stabilize and Acro modes
    if (!old_flightmode->has_manual_throttle()){
        if (new_flightmode == &mode_stabilize){
            input_manager.set_stab_col_ramp(1.0);
        } else if (new_flightmode == &mode_acro){
            input_manager.set_stab_col_ramp(0.0);
        }
    }
#endif //HELI_FRAME
}

// notify_flight_mode - sets notify object based on current flight mode.  Only used for OreoLED notify device
void Copter::notify_flight_mode() {
    AP_Notify::flags.autopilot_mode = flightmode->is_autopilot();
    AP_Notify::flags.flight_mode = (uint8_t)flightmode->mode_number();
    notify.set_flight_mode_str(flightmode->name4());
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void Mode::get_pilot_desired_lean_angles(float &roll_out_cd, float &pitch_out_cd, float angle_max_cd, float angle_limit_cd) const
{
    // throttle failsafe check
    if (copter.failsafe.radio || !copter.ap.rc_receiver_present) {
        roll_out_cd = 0.0;
        pitch_out_cd = 0.0;
        return;
    }
    // fetch roll and pitch stick positions
    float thrust_angle_x_cd = - channel_pitch->get_control_in();
    float thrust_angle_y_cd = channel_roll->get_control_in();

    // limit max lean angle
    angle_limit_cd = constrain_float(angle_limit_cd, 1000.0f, angle_max_cd);

    // scale roll and pitch inputs to +- angle_max
    float scaler = angle_max_cd/(float)ROLL_PITCH_YAW_INPUT_MAX;
    thrust_angle_x_cd *= scaler;
    thrust_angle_y_cd *= scaler;

    // convert square mapping to circular mapping with maximum magnitude of angle_limit
    float total_in = norm(thrust_angle_x_cd, thrust_angle_y_cd);
    if (total_in > angle_limit_cd) {
        float ratio = angle_limit_cd / total_in;
        thrust_angle_x_cd *= ratio;
        thrust_angle_y_cd *= ratio;
    }

    // thrust_angle_x and thrust_angle_y represents a level body frame thrust vector in the
    // direction of [thrust_angle_x, thrust_angle_y] and a magnitude
    // tan(mag([thrust_angle_x, thrust_angle_y])) * 9.81 * aircraft mass.

    // Conversion from angular thrust vector to euler angles.
    roll_out_cd = (18000/M_PI) * atanf(cosf(thrust_angle_x_cd*(M_PI/18000))*tanf(thrust_angle_y_cd*(M_PI/18000)));
    pitch_out_cd = - thrust_angle_x_cd;
}

// transform pilot's roll or pitch input into a desired velocity
Vector2f Mode::get_pilot_desired_velocity(float vel_max) const
{
    Vector2f vel;

    // throttle failsafe check
    if (copter.failsafe.radio || !copter.ap.rc_receiver_present) {
        return vel;
    }
    // fetch roll and pitch inputs
    float roll_out = channel_roll->get_control_in();
    float pitch_out = channel_pitch->get_control_in();

    // convert roll and pitch inputs to -1 to +1 range
    float scaler = 1.0 / (float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_out *= scaler;
    pitch_out *= scaler;

    // convert roll and pitch inputs into velocity in NE frame
    vel = Vector2f(-pitch_out, roll_out);
    if (vel.is_zero()) {
        return vel;
    }
    copter.rotate_body_frame_to_NE(vel.x, vel.y);

    // Transform square input range to circular output
    // vel_scaler is the vector to the edge of the +- 1.0 square in the direction of the current input
    Vector2f vel_scaler = vel / MAX(fabsf(vel.x), fabsf(vel.y));
    // We scale the output by the ratio of the distance to the square to the unit circle and multiply by vel_max
    vel *= vel_max / vel_scaler.length();
    return vel;
}

bool Mode::_TakeOff::triggered(const float target_climb_rate) const
{
    if (!copter.ap.land_complete) {
        // can't take off if we're already flying
        return false;
    }
    if (target_climb_rate <= 0.0f) {
        // can't takeoff unless we want to go up...
        return false;
    }

    if (copter.motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        // hold aircraft on the ground until rotor speed runup has finished
        return false;
    }

    return true;
}

bool Mode::is_disarmed_or_landed() const
{
    if (!motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        return true;
    }
    return false;
}

void Mode::zero_throttle_and_relax_ac(bool spool_up)
{
    if (spool_up) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
    attitude_control->set_throttle_out(0.0f, false, copter.g.throttle_filt);
}

void Mode::zero_throttle_and_hold_attitude()
{
    // run attitude controller
    attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, 0.0f, 0.0f);
    attitude_control->set_throttle_out(0.0f, false, copter.g.throttle_filt);
}

// handle situations where the vehicle is on the ground waiting for takeoff
// force_throttle_unlimited should be true in cases where we want to keep the motors spooled up
// (instead of spooling down to ground idle).  This is required for tradheli's in Guided and Auto
// where we always want the motor spooled up in Guided or Auto mode.  Tradheli's main rotor stops 
// when spooled down to ground idle.
// ultimately it forces the motor interlock to be obeyed in auto and guided modes when on the ground.
void Mode::make_safe_ground_handling(bool force_throttle_unlimited)
{
    if (force_throttle_unlimited) {
        // keep rotors turning 
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } else {
        // spool down to ground idle
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }

    // aircraft is landed, integrator terms must be reset regardless of spool state
    attitude_control->reset_rate_controller_I_terms_smoothly();
 
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
    case AP_Motors::SpoolState::GROUND_IDLE:
        // reset yaw targets and rates during idle states
        attitude_control->reset_yaw_target_and_rate();
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // while transitioning though active states continue to operate normally
        break;
    }

    pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
    pos_control->update_z_controller();
    // we may need to move this out
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
}

/*
  get a height above ground estimate for landing
 */
int32_t Mode::get_alt_above_ground_cm(void)
{
    int32_t alt_above_ground_cm;
    if (copter.get_rangefinder_height_interpolated_cm(alt_above_ground_cm)) {
        return alt_above_ground_cm;
    }
    if (!pos_control->is_active_xy()) {
        return copter.current_loc.alt;
    }
    if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, alt_above_ground_cm)) {
        return alt_above_ground_cm;
    }

    // Assume the Earth is flat:
    return copter.current_loc.alt;
}

void Mode::land_run_vertical_control(bool pause_descent)
{
    float cmb_rate = 0;
    bool ignore_descent_limit = false;
    if (!pause_descent) {

        // do not ignore limits until we have slowed down for landing
        ignore_descent_limit = (MAX(g2.land_alt_low,100) > get_alt_above_ground_cm()) || copter.ap.land_complete_maybe;

        float max_land_descent_velocity;
        if (g.land_speed_high > 0) {
            max_land_descent_velocity = -g.land_speed_high;
        } else {
            max_land_descent_velocity = pos_control->get_max_speed_down_cms();
        }

        // Don't speed up for landing.
        max_land_descent_velocity = MIN(max_land_descent_velocity, -abs(g.land_speed));

        // Compute a vertical velocity demand such that the vehicle approaches g2.land_alt_low. Without the below constraint, this would cause the vehicle to hover at g2.land_alt_low.
        cmb_rate = sqrt_controller(MAX(g2.land_alt_low,100)-get_alt_above_ground_cm(), pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), G_Dt);

        // Constrain the demanded vertical velocity so that it is between the configured maximum descent speed and the configured minimum descent speed.
        cmb_rate = constrain_float(cmb_rate, max_land_descent_velocity, -abs(g.land_speed));

#if PRECISION_LANDING == ENABLED
        const bool navigating = pos_control->is_active_xy();
        bool doing_precision_landing = !copter.ap.land_repo_active && copter.precland.target_acquired() && navigating;

        if (doing_precision_landing) {
            // prec landing is active
            Vector2f target_pos;
            float target_error_cm = 0.0f;
            if (copter.precland.get_target_position_cm(target_pos)) {
                const Vector2f current_pos = inertial_nav.get_position_xy_cm();
                // target is this many cm away from the vehicle
                target_error_cm = (target_pos - current_pos).length();
            }
            // check if we should descend or not
            const float max_horiz_pos_error_cm = copter.precland.get_max_xy_error_before_descending_cm();
            if (target_error_cm > max_horiz_pos_error_cm && !is_zero(max_horiz_pos_error_cm)) {
                // doing precland but too far away from the obstacle
                // do not descend
                cmb_rate = 0.0f;
            } else if (copter.rangefinder_alt_ok() && copter.rangefinder_state.alt_cm > 35.0f && copter.rangefinder_state.alt_cm < 200.0f) {
                // very close to the ground and doing prec land, lets slow down to make sure we land on target
                // compute desired descent velocity
                const float precland_acceptable_error_cm = 15.0f;
                const float precland_min_descent_speed_cms = 10.0f;
                const float max_descent_speed_cms = abs(g.land_speed)*0.5f;
                const float land_slowdown = MAX(0.0f, target_error_cm*(max_descent_speed_cms/precland_acceptable_error_cm));
                cmb_rate = MIN(-precland_min_descent_speed_cms, -max_descent_speed_cms+land_slowdown);
            }
        }
#endif
    }

    // update altitude target and call position controller
    pos_control->land_at_climb_rate_cm(cmb_rate, ignore_descent_limit);
    pos_control->update_z_controller();
}

void Mode::land_run_horizontal_control()
{
    Vector2f vel_correction;
    float target_yaw_rate = 0;

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        pos_control->soften_for_landing_xy();
    }

    // process pilot inputs
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            AP::logger().Write_Event(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // convert pilot input to reposition velocity
            // use half maximum acceleration as the maximum velocity to ensure aircraft will
            // stop from full reposition speed in less than 1 second.
            const float max_pilot_vel = wp_nav->get_wp_acceleration() * 0.5;
            vel_correction = get_pilot_desired_velocity(max_pilot_vel);

            // record if pilot has overridden roll or pitch
            if (!vel_correction.is_zero()) {
                if (!copter.ap.land_repo_active) {
                    AP::logger().Write_Event(LogEvent::LAND_REPO_ACTIVE);
                }
                copter.ap.land_repo_active = true;
            }
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // this variable will be updated if prec land target is in sight and pilot isn't trying to reposition the vehicle
    copter.ap.prec_land_active = false;
#if PRECISION_LANDING == ENABLED
    copter.ap.prec_land_active = !copter.ap.land_repo_active && copter.precland.target_acquired();
    // run precision landing
    if (copter.ap.prec_land_active) {
        Vector2f target_pos, target_vel;
        if (!copter.precland.get_target_position_cm(target_pos)) {
            target_pos = inertial_nav.get_position_xy_cm();
        }
         // get the velocity of the target
        copter.precland.get_target_velocity_cms(inertial_nav.get_velocity_xy_cms(), target_vel);

        Vector2f zero;
        Vector2p landing_pos = target_pos.topostype();
        // target vel will remain zero if landing target is stationary
        pos_control->input_pos_vel_accel_xy(landing_pos, target_vel, zero);
    }
#endif

    if (!copter.ap.prec_land_active) {
        Vector2f accel;
        pos_control->input_vel_accel_xy(vel_correction, accel);
    }

    // run pos controller
    pos_control->update_xy_controller();
    Vector3f thrust_vector = pos_control->get_thrust_vector();

    if (g2.wp_navalt_min > 0) {
        // user has requested an altitude below which navigation
        // attitude is limited. This is used to prevent commanded roll
        // over on landing, which particularly affects helicopters if
        // there is any position estimate drift after touchdown. We
        // limit attitude to 7 degrees below this limit and linearly
        // interpolate for 1m above that
        const float attitude_limit_cd = linear_interpolate(700, copter.aparm.angle_max, get_alt_above_ground_cm(),
                                                     g2.wp_navalt_min*100U, (g2.wp_navalt_min+1)*100U);
        const float thrust_vector_max = sinf(radians(attitude_limit_cd * 0.01f)) * GRAVITY_MSS * 100.0f;
        const float thrust_vector_mag = thrust_vector.xy().length();
        if (thrust_vector_mag > thrust_vector_max) {
            float ratio = thrust_vector_max / thrust_vector_mag;
            thrust_vector.x *= ratio;
            thrust_vector.y *= ratio;

            // tell position controller we are applying an external limit
            pos_control->set_externally_limited_xy();
        }
    }

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(thrust_vector, target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_thrust_vector_heading(thrust_vector, auto_yaw.yaw());
    }
}

// run normal or precision landing (if enabled)
// pause_descent is true if vehicle should not descend
void Mode::land_run_normal_or_precland(bool pause_descent)
{
#if PRECISION_LANDING == ENABLED
    if (pause_descent || !copter.precland.enabled()) {
        // we don't want to start descending immediately or prec land is disabled
        // in both cases just run simple land controllers
        land_run_horiz_and_vert_control(pause_descent);
    } else {
        // prec land is enabled and we have not paused descent
        // the state machine takes care of the entire prec landing procedure
        precland_run();
    }
#else
    land_run_horiz_and_vert_control(pause_descent);
#endif
}

#if PRECISION_LANDING == ENABLED
// Go towards a position commanded by prec land state machine in order to retry landing
// The passed in location is expected to be NED and in m
void Mode::precland_retry_position(const Vector3f &retry_pos)
{
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            AP::logger().Write_Event(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        // allow user to take control during repositioning. Note: copied from land_run_horizontal_control()
        // To-Do: this code exists at several different places in slightly diffrent forms and that should be fixed
        if (g.land_repositioning) {
            float target_roll = 0.0f;
            float target_pitch = 0.0f;
            // convert pilot input to lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

            // record if pilot has overridden roll or pitch
            if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                if (!copter.ap.land_repo_active) {
                    AP::logger().Write_Event(LogEvent::LAND_REPO_ACTIVE);
                }
                // this flag will be checked by prec land state machine later and any further landing retires will be cancelled
                copter.ap.land_repo_active = true;
            }
        }
    }

    Vector3p retry_pos_NEU{retry_pos.x, retry_pos.y, retry_pos.z * -1.0f};
    //pos contoller expects input in NEU cm's
    retry_pos_NEU = retry_pos_NEU * 100.0f;
    pos_control->input_pos_xyz(retry_pos_NEU, 0.0f, 1000.0f);

    // run position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    const Vector3f thrust_vector{pos_control->get_thrust_vector()};

    // roll, pitch from position controller, yaw heading from auto_heading()
    attitude_control->input_thrust_vector_heading(thrust_vector, auto_yaw.yaw());
}

// Run precland statemachine. This function should be called from any mode that wants to do precision landing.
// This handles everything from prec landing, to prec landing failures, to retries and failsafe measures
void Mode::precland_run()
{
    // if user is taking control, we will not run the statemachine, and simply land (may or may not be on target)
    if (!copter.ap.land_repo_active) {
        // This will get updated later to a retry pos if needed
        Vector3f retry_pos;

        switch (copter.precland_statemachine.update(retry_pos)) {
        case AC_PrecLand_StateMachine::Status::RETRYING:
            // we want to retry landing by going to another position
            precland_retry_position(retry_pos);
            break;

        case AC_PrecLand_StateMachine::Status::FAILSAFE: {
            // we have hit a failsafe. Failsafe can only mean two things, we either want to stop permanently till user takes over or land
            switch (copter.precland_statemachine.get_failsafe_actions()) {
            case AC_PrecLand_StateMachine::FailSafeAction::DESCEND:
                // descend normally, prec land target is definitely not in sight
                land_run_horiz_and_vert_control();
                break;
            case AC_PrecLand_StateMachine::FailSafeAction::HOLD_POS:
                // sending "true" in this argument will stop the descend
                land_run_horiz_and_vert_control(true);
                break;
            }
            break;
        }
        case AC_PrecLand_StateMachine::Status::ERROR:
            // should never happen, is certainly a bug. Report then descend
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            FALLTHROUGH;
        case AC_PrecLand_StateMachine::Status::DESCEND:
            // run land controller. This will descend towards the target if prec land target is in sight
            // else it will just descend vertically
            land_run_horiz_and_vert_control();
            break;
        }
    } else {
        // just land, since user has taken over controls, it does not make sense to run any retries or failsafe measures
        land_run_horiz_and_vert_control();
    }
}
#endif

float Mode::throttle_hover() const
{
    return motors->get_throttle_hover();
}

// transform pilot's manual throttle input to make hover throttle mid stick
// used only for manual throttle modes
// thr_mid should be in the range 0 to 1
// returns throttle output 0 to 1
float Mode::get_pilot_desired_throttle() const
{
    const float thr_mid = throttle_hover();
    int16_t throttle_control = channel_throttle->get_control_in();

    int16_t mid_stick = copter.get_throttle_mid();
    // protect against unlikely divide by zero
    if (mid_stick <= 0) {
        mid_stick = 500;
    }

    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);

    // calculate normalised throttle input
    float throttle_in;
    if (throttle_control < mid_stick) {
        throttle_in = ((float)throttle_control)*0.5f/(float)mid_stick;
    } else {
        throttle_in = 0.5f + ((float)(throttle_control-mid_stick)) * 0.5f / (float)(1000-mid_stick);
    }

    const float expo = constrain_float(-(thr_mid-0.5f)/0.375f, -0.5f, 1.0f);
    // calculate the output throttle using the given expo function
    float throttle_out = throttle_in*(1.0f-expo) + expo*throttle_in*throttle_in*throttle_in;
    return throttle_out;
}

float Mode::get_avoidance_adjusted_climbrate(float target_rate)
{
#if AC_AVOID_ENABLED == ENABLED
    AP::ac_avoid()->adjust_velocity_z(pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), target_rate, G_Dt);
    return target_rate;
#else
    return target_rate;
#endif
}

// send output to the motors, can be overridden by subclasses
void Mode::output_to_motors()
{
    motors->output();
}

Mode::AltHoldModeState Mode::get_alt_hold_state(float target_climb_rate_cms)
{
    // Alt Hold State Machine Determination
    if (!motors->armed()) {
        // the aircraft should moved to a shut down state
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);

        // transition through states as aircraft spools down
        switch (motors->get_spool_state()) {

        case AP_Motors::SpoolState::SHUT_DOWN:
            return AltHold_MotorStopped;

        case AP_Motors::SpoolState::GROUND_IDLE:
            return AltHold_Landed_Ground_Idle;

        default:
            return AltHold_Landed_Pre_Takeoff;
        }

    } else if (takeoff.running() || takeoff.triggered(target_climb_rate_cms)) {
        // the aircraft is currently landed or taking off, asking for a positive climb rate and in THROTTLE_UNLIMITED
        // the aircraft should progress through the take off procedure
        return AltHold_Takeoff;

    } else if (!copter.ap.auto_armed || copter.ap.land_complete) {
        // the aircraft is armed and landed
        if (target_climb_rate_cms < 0.0f && !copter.ap.using_interlock) {
            // the aircraft should move to a ground idle state
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);

        } else {
            // the aircraft should prepare for imminent take off
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }

        if (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
            // the aircraft is waiting in ground idle
            return AltHold_Landed_Ground_Idle;

        } else {
            // the aircraft can leave the ground at any time
            return AltHold_Landed_Pre_Takeoff;
        }

    } else {
        // the aircraft is in a flying state
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        return AltHold_Flying;
    }
}

// transform pilot's yaw input into a desired yaw rate
// returns desired yaw rate in centi-degrees per second
float Mode::get_pilot_desired_yaw_rate(float yaw_in)
{
    // throttle failsafe check
    if (copter.failsafe.radio || !copter.ap.rc_receiver_present) {
        return 0.0f;
    }

    // convert pilot input to the desired yaw rate
    return g2.pilot_y_rate * 100.0 * input_expo(yaw_in, g2.pilot_y_expo);
}

// pass-through functions to reduce code churn on conversion;
// these are candidates for moving into the Mode base
// class.
float Mode::get_pilot_desired_climb_rate(float throttle_control)
{
    return copter.get_pilot_desired_climb_rate(throttle_control);
}

float Mode::get_non_takeoff_throttle()
{
    return copter.get_non_takeoff_throttle();
}

void Mode::update_simple_mode(void) {
    copter.update_simple_mode();
}

bool Mode::set_mode(Mode::Number mode, ModeReason reason)
{
    return copter.set_mode(mode, reason);
}

void Mode::set_land_complete(bool b)
{
    return copter.set_land_complete(b);
}

GCS_Copter &Mode::gcs()
{
    return copter.gcs();
}

// set_throttle_takeoff - allows modes to tell throttle controller we
// are taking off so I terms can be cleared
void Mode::set_throttle_takeoff()
{
    // initialise the vertical position controller
    pos_control->init_z_controller();
}

uint16_t Mode::get_pilot_speed_dn()
{
    return copter.get_pilot_speed_dn();
}
Mode_acro.cpp
#include "Copter.h"

#include "mode.h"

#if MODE_ACRO_ENABLED == ENABLED

/*
 * Init and run calls for acro flight mode
 */

void ModeAcro::run()
{
    // convert the input to the desired body frame rate
    float target_roll, target_pitch, target_yaw;
    get_pilot_desired_angle_rates(channel_roll->norm_input_dz(), channel_pitch->norm_input_dz(), channel_yaw->norm_input_dz(), target_roll, target_pitch, target_yaw);

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero && copter.air_mode != AirMode::AIRMODE_ENABLED) {
        // Attempting to Land, if airmode is enabled only an actual landing will spool down the motors
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_target_and_rate(true);
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // run attitude controller
    if (g2.acro_options.get() & uint8_t(AcroOptions::RATE_LOOP_ONLY)) {
        attitude_control->input_rate_bf_roll_pitch_yaw_2(target_roll, target_pitch, target_yaw);
    } else {
        attitude_control->input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);
    }

    // output pilot's throttle without angle boost
    attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                       false,
                                       copter.g.throttle_filt);
}

bool ModeAcro::init(bool ignore_checks)
{
    if (g2.acro_options.get() & uint8_t(AcroOptions::AIR_MODE)) {
        disable_air_mode_reset = false;
        copter.air_mode = AirMode::AIRMODE_ENABLED;
    }

    return true;
}

void ModeAcro::exit()
{
    if (!disable_air_mode_reset && (g2.acro_options.get() & uint8_t(AcroOptions::AIR_MODE))) {
        copter.air_mode = AirMode::AIRMODE_DISABLED;
    }
    disable_air_mode_reset = false;
}

void ModeAcro::air_mode_aux_changed()
{
    disable_air_mode_reset = true;
}

float ModeAcro::throttle_hover() const
{
    if (g2.acro_thr_mid > 0) {
        return g2.acro_thr_mid;
    }
    return Mode::throttle_hover();
}

// get_pilot_desired_angle_rates - transform pilot's normalised roll pitch and yaw input into a desired lean angle rates
// inputs are -1 to 1 and the function returns desired angle rates in centi-degrees-per-second
void ModeAcro::get_pilot_desired_angle_rates(float roll_in, float pitch_in, float yaw_in, float &roll_out, float &pitch_out, float &yaw_out)
{
    float rate_limit;
    Vector3f rate_ef_level_cd, rate_bf_level_cd, rate_bf_request_cd;

    // apply circular limit to pitch and roll inputs
    float total_in = norm(pitch_in, roll_in);

    if (total_in > 1.0) {
        float ratio = 1.0 / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // calculate roll, pitch rate requests
    
    // roll expo
    rate_bf_request_cd.x = g2.acro_rp_rate * 100.0 * input_expo(roll_in, g.acro_rp_expo);

    // pitch expo
    rate_bf_request_cd.y = g2.acro_rp_rate * 100.0 * input_expo(pitch_in, g.acro_rp_expo);

    // yaw expo
    rate_bf_request_cd.z = g2.acro_y_rate * 100.0 * input_expo(yaw_in, g2.acro_y_expo);

    // calculate earth frame rate corrections to pull the copter back to level while in ACRO mode

    if (g.acro_trainer != (uint8_t)Trainer::OFF) {

        // get attitude targets
        const Vector3f att_target = attitude_control->get_att_target_euler_cd();

        // Calculate trainer mode earth frame rate command for roll
        int32_t roll_angle = wrap_180_cd(att_target.x);
        rate_ef_level_cd.x = -constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_roll;

        // Calculate trainer mode earth frame rate command for pitch
        int32_t pitch_angle = wrap_180_cd(att_target.y);
        rate_ef_level_cd.y = -constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_pitch;

        // Calculate trainer mode earth frame rate command for yaw
        rate_ef_level_cd.z = 0;

        // Calculate angle limiting earth frame rate commands
        if (g.acro_trainer == (uint8_t)Trainer::LIMITED) {
            const float angle_max = copter.aparm.angle_max;
            if (roll_angle > angle_max){
                rate_ef_level_cd.x += sqrt_controller(angle_max - roll_angle, g2.acro_rp_rate * 100.0 / ACRO_LEVEL_MAX_OVERSHOOT, attitude_control->get_accel_roll_max_cdss(), G_Dt);
            }else if (roll_angle < -angle_max) {
                rate_ef_level_cd.x += sqrt_controller(-angle_max - roll_angle, g2.acro_rp_rate * 100.0 / ACRO_LEVEL_MAX_OVERSHOOT, attitude_control->get_accel_roll_max_cdss(), G_Dt);
            }

            if (pitch_angle > angle_max){
                rate_ef_level_cd.y += sqrt_controller(angle_max - pitch_angle, g2.acro_rp_rate * 100.0 / ACRO_LEVEL_MAX_OVERSHOOT, attitude_control->get_accel_pitch_max_cdss(), G_Dt);
            }else if (pitch_angle < -angle_max) {
                rate_ef_level_cd.y += sqrt_controller(-angle_max - pitch_angle, g2.acro_rp_rate * 100.0 / ACRO_LEVEL_MAX_OVERSHOOT, attitude_control->get_accel_pitch_max_cdss(), G_Dt);
            }
        }

        // convert earth-frame level rates to body-frame level rates
        attitude_control->euler_rate_to_ang_vel(attitude_control->get_att_target_euler_cd() * radians(0.01f), rate_ef_level_cd, rate_bf_level_cd);

        // combine earth frame rate corrections with rate requests
        if (g.acro_trainer == (uint8_t)Trainer::LIMITED) {
            rate_bf_request_cd.x += rate_bf_level_cd.x;
            rate_bf_request_cd.y += rate_bf_level_cd.y;
            rate_bf_request_cd.z += rate_bf_level_cd.z;
        }else{
            float acro_level_mix = constrain_float(1-float(MAX(MAX(abs(roll_in), abs(pitch_in)), abs(yaw_in))/4500.0), 0, 1)*ahrs.cos_pitch();

            // Scale levelling rates by stick input
            rate_bf_level_cd = rate_bf_level_cd * acro_level_mix;

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request_cd.x)-fabsf(rate_bf_level_cd.x));
            rate_bf_request_cd.x += rate_bf_level_cd.x;
            rate_bf_request_cd.x = constrain_float(rate_bf_request_cd.x, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request_cd.y)-fabsf(rate_bf_level_cd.y));
            rate_bf_request_cd.y += rate_bf_level_cd.y;
            rate_bf_request_cd.y = constrain_float(rate_bf_request_cd.y, -rate_limit, rate_limit);

            // Calculate rate limit to prevent change of rate through inverted
            rate_limit = fabsf(fabsf(rate_bf_request_cd.z)-fabsf(rate_bf_level_cd.z));
            rate_bf_request_cd.z += rate_bf_level_cd.z;
            rate_bf_request_cd.z = constrain_float(rate_bf_request_cd.z, -rate_limit, rate_limit);
        }
    }

    // hand back rate request
    roll_out = rate_bf_request_cd.x;
    pitch_out = rate_bf_request_cd.y;
    yaw_out = rate_bf_request_cd.z;
}
#endif
Mode_acro_heli.cpp
#include "Copter.h"

#if MODE_ACRO_ENABLED == ENABLED

#if FRAME_CONFIG == HELI_FRAME
/*
 * Init and run calls for acro flight mode for trad heli
 */

// heli_acro_init - initialise acro controller
bool ModeAcro_Heli::init(bool ignore_checks)
{
    // if heli is equipped with a flybar, then tell the attitude controller to pass through controls directly to servos
    attitude_control->use_flybar_passthrough(motors->has_flybar(), motors->supports_yaw_passthrough());

    motors->set_acro_tail(true);
    
    // set stab collective false to use full collective pitch range
    copter.input_manager.set_use_stab_col(false);

    // always successfully enter acro
    return true;
}

// heli_acro_run - runs the acro controller
// should be called at 100hz or more
void ModeAcro_Heli::run()
{
    float target_roll, target_pitch, target_yaw;
    float pilot_throttle_scaled;

    // Tradheli should not reset roll, pitch, yaw targets when motors are not runup while flying, because
    // we may be in autorotation flight.  This is so that the servos move in a realistic fashion while disarmed
    // for operational checks. Also, unlike multicopters we do not set throttle (i.e. collective pitch) to zero
    // so the swash servos move.

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else {
        // heli will not let the spool state progress to THROTTLE_UNLIMITED until motor interlock is enabled
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
        break;
    case AP_Motors::SpoolState::GROUND_IDLE:
        // If aircraft is landed, set target heading to current and reset the integrator
        // Otherwise motors could be at ground idle for practice autorotation
        if ((motors->init_targets_on_arming() && motors->using_leaky_integrator()) || (copter.ap.land_complete && !motors->using_leaky_integrator())) {
            attitude_control->reset_target_and_rate(false);
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        if (copter.ap.land_complete && !motors->using_leaky_integrator()) {
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    if (!motors->has_flybar()){
        // convert the input to the desired body frame rate
        get_pilot_desired_angle_rates(channel_roll->norm_input_dz(), channel_pitch->norm_input_dz(), channel_yaw->norm_input_dz(), target_roll, target_pitch, target_yaw);
        // only mimic flybar response when trainer mode is disabled
        if ((Trainer)g.acro_trainer.get() == Trainer::OFF) {
            // while landed always leak off target attitude to current attitude
            if (copter.ap.land_complete) {
                virtual_flybar(target_roll, target_pitch, target_yaw, 3.0f, 3.0f);
            // while flying use acro balance parameters for leak rate
            } else {
                virtual_flybar(target_roll, target_pitch, target_yaw, g.acro_balance_pitch, g.acro_balance_roll);
            }
        }
        if (motors->supports_yaw_passthrough()) {
            // if the tail on a flybar heli has an external gyro then
            // also use no deadzone for the yaw control and
            // pass-through the input direct to output.
            target_yaw = channel_yaw->get_control_in_zero_dz();
        }

        // run attitude controller
        if (g2.acro_options.get() & uint8_t(AcroOptions::RATE_LOOP_ONLY)) {
            attitude_control->input_rate_bf_roll_pitch_yaw_2(target_roll, target_pitch, target_yaw);
        } else {
            attitude_control->input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);
        }
    }else{
        /*
          for fly-bar passthrough use control_in values with no
          deadzone. This gives true pass-through.
         */
        float roll_in = channel_roll->get_control_in_zero_dz();
        float pitch_in = channel_pitch->get_control_in_zero_dz();
        float yaw_in;
        
        if (motors->supports_yaw_passthrough()) {
            // if the tail on a flybar heli has an external gyro then
            // also use no deadzone for the yaw control and
            // pass-through the input direct to output.
            yaw_in = channel_yaw->get_control_in_zero_dz();
        } else {
            // if there is no external gyro then run the usual
            // ACRO_YAW_P gain on the input control, including
            // deadzone
            yaw_in = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        }

        // run attitude controller
        attitude_control->passthrough_bf_roll_pitch_rate_yaw(roll_in, pitch_in, yaw_in);
    }

    // get pilot's desired throttle
    pilot_throttle_scaled = copter.input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());

    // output pilot's throttle without angle boost
    attitude_control->set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
}


// virtual_flybar - acts like a flybar by leaking target atttitude back to current attitude
void ModeAcro_Heli::virtual_flybar( float &roll_out, float &pitch_out, float &yaw_out, float pitch_leak, float roll_leak)
{
    Vector3f rate_ef_level, rate_bf_level;

    // get attitude targets
    const Vector3f att_target = attitude_control->get_att_target_euler_cd();

    // Calculate earth frame rate command for roll leak to current attitude
    rate_ef_level.x = -wrap_180_cd(att_target.x - ahrs.roll_sensor) * roll_leak;

    // Calculate earth frame rate command for pitch leak to current attitude
    rate_ef_level.y = -wrap_180_cd(att_target.y - ahrs.pitch_sensor) * pitch_leak;

    // Calculate earth frame rate command for yaw
    rate_ef_level.z = 0;

    // convert earth-frame leak rates to body-frame leak rates
    attitude_control->euler_rate_to_ang_vel(attitude_control->get_att_target_euler_cd()*radians(0.01f), rate_ef_level, rate_bf_level);

    // combine earth frame rate corrections with rate requests
    roll_out += rate_bf_level.x;
    pitch_out += rate_bf_level.y;
    yaw_out += rate_bf_level.z;

}
#endif  //HELI_FRAME
#endif  //MODE_ACRO_ENABLED == ENABLED
Mode_althold.cpp
#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAltHold::init(bool ignore_checks)
{

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAltHold::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}
Mode_auto.cpp
#include "Copter.h"

#if MODE_AUTO_ENABLED == ENABLED

/*
 * Init and run calls for auto flight mode
 *
 * This file contains the implementation for Land, Waypoint navigation and Takeoff from Auto mode
 * Command execution code (i.e. command_logic.pde) should:
 *      a) switch to Auto flight mode with set_mode() function.  This will cause auto_init to be called
 *      b) call one of the three auto initialisation functions: auto_wp_start(), auto_takeoff_start(), auto_land_start()
 *      c) call one of the verify functions auto_wp_verify(), auto_takeoff_verify, auto_land_verify repeated to check if the command has completed
 * The main loop (i.e. fast loop) will call update_flight_modes() which will in turn call auto_run() which, based upon the auto_mode variable will call
 *      correct auto_wp_run, auto_takeoff_run or auto_land_run to actually implement the feature
 */

/*
 *  While in the auto flight mode, navigation or do/now commands can be run.
 *  Code in this file implements the navigation commands
 */

// auto_init - initialise auto controller
bool ModeAuto::init(bool ignore_checks)
{
    auto_RTL = false;
    if (mission.num_commands() > 1 || ignore_checks) {
        // reject switching to auto mode if landed with motors armed but first command is not a takeoff (reduce chance of flips)
        if (motors->armed() && copter.ap.land_complete && !mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto: Missing Takeoff Cmd");
            return false;
        }

        _mode = SubMode::LOITER;

        // stop ROI from carrying over from previous runs of the mission
        // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
        if (auto_yaw.mode() == AUTO_YAW_ROI) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }

        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();

        // set flag to start mission
        waiting_to_start = true;

        // initialise mission change check (ignore results)
        IGNORE_RETURN(mis_change_detector.check_for_mission_change());

        // clear guided limits
        copter.mode_guided.limit_clear();

        // reset flag indicating if pilot has applied roll or pitch inputs during landing
        copter.ap.land_repo_active = false;

#if PRECISION_LANDING == ENABLED
        // initialise precland state machine
        copter.precland_statemachine.init();
#endif

        return true;
    } else {
        return false;
    }
}

// stop mission when we leave auto mode
void ModeAuto::exit()
{
    if (copter.mode_auto.mission.state() == AP_Mission::MISSION_RUNNING) {
        copter.mode_auto.mission.stop();
    }
#if HAL_MOUNT_ENABLED
    copter.camera_mount.set_mode_to_default();
#endif  // HAL_MOUNT_ENABLED

    auto_RTL = false;
}

// auto_run - runs the auto controller
//      should be called at 100hz or more
void ModeAuto::run()
{
    // start or update mission
    if (waiting_to_start) {
        // don't start the mission until we have an origin
        Location loc;
        if (copter.ahrs.get_origin(loc)) {
            // start/resume the mission (based on MIS_RESTART parameter)
            mission.start_or_resume();
            waiting_to_start = false;

            // initialise mission change check (ignore results)
            IGNORE_RETURN(mis_change_detector.check_for_mission_change());
        }
    } else {
        // check for mission changes
        if (mis_change_detector.check_for_mission_change()) {
            // if mission is running restart the current command if it is a waypoint or spline command
            if ((mission.state() == AP_Mission::MISSION_RUNNING) && (_mode == SubMode::WP)) {
                if (mission.restart_current_nav_cmd()) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto mission changed, restarted command");
                } else {
                    // failed to restart mission for some reason
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto mission changed but failed to restart command");
                }
            }
        }

        mission.update();
    }

    // call the correct auto controller
    switch (_mode) {

    case SubMode::TAKEOFF:
        takeoff_run();
        break;

    case SubMode::WP:
    case SubMode::CIRCLE_MOVE_TO_EDGE:
        wp_run();
        break;

    case SubMode::LAND:
        land_run();
        break;

    case SubMode::RTL:
        rtl_run();
        break;

    case SubMode::CIRCLE:
        circle_run();
        break;

    case SubMode::NAVGUIDED:
    case SubMode::NAV_SCRIPT_TIME:
#if NAV_GUIDED == ENABLED || AP_SCRIPTING_ENABLED
        nav_guided_run();
#endif
        break;

    case SubMode::LOITER:
        loiter_run();
        break;

    case SubMode::LOITER_TO_ALT:
        loiter_to_alt_run();
        break;

    case SubMode::NAV_PAYLOAD_PLACE:
        payload_place_run();
        break;
    }

    // only pretend to be in auto RTL so long as mission still thinks its in a landing sequence or the mission has completed
    if (auto_RTL && (!(mission.get_in_landing_sequence_flag() || mission.state() == AP_Mission::mission_state::MISSION_COMPLETE))) {
        auto_RTL = false;
        // log exit from Auto RTL
        copter.logger.Write_Mode((uint8_t)copter.flightmode->mode_number(), ModeReason::AUTO_RTL_EXIT);
    }
}

bool ModeAuto::allows_arming(AP_Arming::Method method) const
{
    return ((copter.g2.auto_options & (uint32_t)Options::AllowArming) != 0) && !auto_RTL;
};

// Go straight to landing sequence via DO_LAND_START, if succeeds pretend to be Auto RTL mode
bool ModeAuto::jump_to_landing_sequence_auto_RTL(ModeReason reason)
{
    if (mission.jump_to_landing_sequence()) {
        mission.set_force_resume(true);
        // if not already in auto switch to auto
        if ((copter.flightmode == &copter.mode_auto) || set_mode(Mode::Number::AUTO, reason)) {
            auto_RTL = true;
            // log entry into AUTO RTL
            copter.logger.Write_Mode((uint8_t)copter.flightmode->mode_number(), reason);

            // make happy noise
            if (copter.ap.initialised) {
                AP_Notify::events.user_mode_change = 1;
            }
            return true;
        }
        // mode change failed, revert force resume flag
        mission.set_force_resume(false);

        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to AUTO RTL failed");
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode change to AUTO RTL failed: No landing sequence found");
    }

    AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(Number::AUTO_RTL));
    // make sad noise
    if (copter.ap.initialised) {
        AP_Notify::events.user_mode_change_failed = 1;
    }
    return false;
}

// lua scripts use this to retrieve the contents of the active command
bool ModeAuto::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2)
{
#if AP_SCRIPTING_ENABLED
    if (_mode == SubMode::NAV_SCRIPT_TIME) {
        id = nav_scripting.id;
        cmd = nav_scripting.command;
        arg1 = nav_scripting.arg1;
        arg2 = nav_scripting.arg2;
        return true;
    }
#endif
    return false;
}

// lua scripts use this to indicate when they have complete the command
void ModeAuto::nav_script_time_done(uint16_t id)
{
#if AP_SCRIPTING_ENABLED
    if ((_mode == SubMode::NAV_SCRIPT_TIME) && (id == nav_scripting.id)) {
        nav_scripting.done = true;
    }
#endif
}

// auto_loiter_start - initialises loitering in auto mode
//  returns success/failure because this can be called by exit_mission
bool ModeAuto::loiter_start()
{
    // return failure if GPS is bad
    if (!copter.position_ok()) {
        return false;
    }
    _mode = SubMode::LOITER;

    // calculate stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // initialise waypoint controller target to stopping point
    wp_nav->set_wp_destination(stopping_point);

    // hold yaw at current heading
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    return true;
}

// auto_rtl_start - initialises RTL in AUTO flight mode
void ModeAuto::rtl_start()
{
    // call regular rtl flight mode initialisation and ask it to ignore checks
    if (copter.mode_rtl.init(true)) {
        _mode = SubMode::RTL;
    } else {
        // this should never happen because RTL never fails init if argument is true
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
}

// auto_takeoff_start - initialises waypoint controller to implement take-off
void ModeAuto::takeoff_start(const Location& dest_loc)
{
    if (!copter.current_loc.initialised()) {
        // this should never happen because mission commands are not executed until
        // the AHRS/EKF origin is set by which time current_loc should also have been set
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }

    _mode = SubMode::TAKEOFF;

    // calculate current and target altitudes
    // by default current_alt_cm and alt_target_cm are alt-above-EKF-origin
    int32_t alt_target_cm;
    bool alt_target_terrain = false;
    float current_alt_cm = inertial_nav.get_position_z_up_cm();
    float terrain_offset;   // terrain's altitude in cm above the ekf origin
    if ((dest_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) && wp_nav->get_terrain_offset(terrain_offset)) {
        // subtract terrain offset to convert vehicle's alt-above-ekf-origin to alt-above-terrain
        current_alt_cm -= terrain_offset;

        // specify alt_target_cm as alt-above-terrain
        alt_target_cm = dest_loc.alt;
        alt_target_terrain = true;
    } else {
        // set horizontal target
        Location dest(dest_loc);
        dest.lat = copter.current_loc.lat;
        dest.lng = copter.current_loc.lng;

        // get altitude target above EKF origin
        if (!dest.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
            // this failure could only happen if take-off alt was specified as an alt-above terrain and we have no terrain data
            AP::logger().Write_Error(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            // fall back to altitude above current altitude
            alt_target_cm = current_alt_cm + dest.alt;
        }
    }

    // sanity check target
    int32_t alt_target_min_cm = current_alt_cm + (copter.ap.land_complete ? 100 : 0);
    alt_target_cm = MAX(alt_target_cm, alt_target_min_cm);

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff_start(alt_target_cm, alt_target_terrain);
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void ModeAuto::wp_start(const Location& dest_loc)
{
    // init wpnav and set origin if transitioning from takeoff
    if (!wp_nav->is_active()) {
        Vector3f stopping_point;
        if (_mode == SubMode::TAKEOFF) {
            Vector3p takeoff_complete_pos;
            if (auto_takeoff_get_position(takeoff_complete_pos)) {
                stopping_point = takeoff_complete_pos.tofloat();
            }
        }
        wp_nav->wp_and_spline_init(0, stopping_point);
    }

    // send target to waypoint controller
    if (!wp_nav->set_wp_destination_loc(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    _mode = SubMode::WP;

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

// auto_land_start - initialises controller to implement a landing
void ModeAuto::land_start()
{
    _mode = SubMode::LAND;

    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialise the vertical position controller
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

#if LANDING_GEAR_ENABLED == ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif

#if AC_FENCE == ENABLED
    // disable the fence on landing
    copter.fence.auto_disable_fence_for_landing();
#endif

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.land_repo_active = false;

    // this will be set true if prec land is later active
    copter.ap.prec_land_active = false;
}

// auto_circle_movetoedge_start - initialise waypoint controller to move to edge of a circle with it's center at the specified location
//  we assume the caller has performed all required GPS_ok checks
void ModeAuto::circle_movetoedge_start(const Location &circle_center, float radius_m)
{
    // set circle center
    copter.circle_nav->set_center(circle_center);

    // set circle radius
    if (!is_zero(radius_m)) {
        copter.circle_nav->set_radius_cm(radius_m * 100.0f);
    }

    // check our distance from edge of circle
    Vector3f circle_edge_neu;
    copter.circle_nav->get_closest_point_on_circle(circle_edge_neu);
    float dist_to_edge = (inertial_nav.get_position_neu_cm() - circle_edge_neu).length();

    // if more than 3m then fly to edge
    if (dist_to_edge > 300.0f) {
        // set the state to move to the edge of the circle
        _mode = SubMode::CIRCLE_MOVE_TO_EDGE;

        // convert circle_edge_neu to Location
        Location circle_edge(circle_edge_neu, Location::AltFrame::ABOVE_ORIGIN);

        // convert altitude to same as command
        circle_edge.set_alt_cm(circle_center.alt, circle_center.get_alt_frame());

        // initialise wpnav to move to edge of circle
        if (!wp_nav->set_wp_destination_loc(circle_edge)) {
            // failure to set destination can only be because of missing terrain data
            copter.failsafe_terrain_on_event();
        }

        // if we are outside the circle, point at the edge, otherwise hold yaw
        const float dist_to_center = get_horizontal_distance_cm(inertial_nav.get_position_xy_cm().topostype(), copter.circle_nav->get_center().xy());
        // initialise yaw
        // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
        if (auto_yaw.mode() != AUTO_YAW_ROI) {
            if (dist_to_center > copter.circle_nav->get_radius() && dist_to_center > 500) {
                auto_yaw.set_mode_to_default(false);
            } else {
                // vehicle is within circle so hold yaw to avoid spinning as we move to edge of circle
                auto_yaw.set_mode(AUTO_YAW_HOLD);
            }
        }
    } else {
        circle_start();
    }
}

// auto_circle_start - initialises controller to fly a circle in AUTO flight mode
//   assumes that circle_nav object has already been initialised with circle center and radius
void ModeAuto::circle_start()
{
    _mode = SubMode::CIRCLE;

    // initialise circle controller
    copter.circle_nav->init(copter.circle_nav->get_center(), copter.circle_nav->center_is_terrain_alt());

    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode(AUTO_YAW_CIRCLE);
    }
}

#if NAV_GUIDED == ENABLED
// auto_nav_guided_start - hand over control to external navigation controller in AUTO mode
void ModeAuto::nav_guided_start()
{
    // call regular guided flight mode initialisation
    if (!copter.mode_guided.init(true)) {
        // this should never happen because guided mode never fails to init
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }

    _mode = SubMode::NAVGUIDED;

    // initialise guided start time and position as reference for limit checking
    copter.mode_guided.limit_init_time_and_pos();
}
#endif //NAV_GUIDED

bool ModeAuto::is_landing() const
{
    switch(_mode) {
    case SubMode::LAND:
        return true;
    case SubMode::RTL:
        return copter.mode_rtl.is_landing();
    default:
        return false;
    }
    return false;
}

bool ModeAuto::is_taking_off() const
{
    return ((_mode == SubMode::TAKEOFF) && !wp_nav->reached_wp_destination());
}

// auto_payload_place_start - initialises controller to implement a placing
void ModeAuto::payload_place_start()
{
    _mode = SubMode::NAV_PAYLOAD_PLACE;
    nav_payload_place.state = PayloadPlaceStateType_Calibrating_Hover_Start;

    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialise the vertical position controller
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool ModeAuto::use_pilot_yaw(void) const
{
    return (copter.g2.auto_options.get() & uint32_t(Options::IgnorePilotYaw)) == 0;
}

// start_command - this function will be called when the ap_mission lib wishes to start a new command
bool ModeAuto::start_command(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: logging when new commands start/end
    if (copter.should_log(MASK_LOG_CMD)) {
        copter.logger.Write_Mission_Cmd(mission, cmd);
    }

    switch(cmd.id) {

    ///
    /// navigation commands
    ///
    case MAV_CMD_NAV_TAKEOFF:                   // 22
        do_takeoff(cmd);
        break;

    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        do_land(cmd);
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // 17 Loiter indefinitely
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 Loiter N Times
        do_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:              // 19
        do_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:
        do_loiter_to_alt(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:             //20
        do_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:           // 82  Navigate to Waypoint using spline
        do_spline_wp(cmd);
        break;

#if NAV_GUIDED == ENABLED
    case MAV_CMD_NAV_GUIDED_ENABLE:             // 92  accept navigation commands from external nav computer
        do_nav_guided_enable(cmd);
        break;
#endif

    case MAV_CMD_NAV_DELAY:                    // 93 Delay the next navigation command
        do_nav_delay(cmd);
        break;

    case MAV_CMD_NAV_PAYLOAD_PLACE:              // 94 place at Waypoint
        do_payload_place(cmd);
        break;

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        do_nav_script_time(cmd);
        break;
#endif

    //
    // conditional commands
    //
    case MAV_CMD_CONDITION_DELAY:             // 112
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:             // 114
        do_within_distance(cmd);
        break;

    case MAV_CMD_CONDITION_YAW:             // 115
        do_yaw(cmd);
        break;

    ///
    /// do commands
    ///
    case MAV_CMD_DO_CHANGE_SPEED:             // 178
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:             // 179
        do_set_home(cmd);
        break;

    case MAV_CMD_DO_SET_ROI:                // 201
        // point the copter and camera at a region of interest (ROI)
        do_roi(cmd);
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:          // 205
        // point the camera to a specified angle
        do_mount_control(cmd);
        break;
    
    case MAV_CMD_DO_FENCE_ENABLE:
#if AC_FENCE == ENABLED
        if (cmd.p1 == 0) { //disable
            copter.fence.enable(false);
            gcs().send_text(MAV_SEVERITY_INFO, "Fence Disabled");
        } else { //enable fence
            copter.fence.enable(true);
            gcs().send_text(MAV_SEVERITY_INFO, "Fence Enabled");
        }
#endif //AC_FENCE == ENABLED
        break;

#if NAV_GUIDED == ENABLED
    case MAV_CMD_DO_GUIDED_LIMITS:                      // 220  accept guided mode limits
        do_guided_limits(cmd);
        break;
#endif

#if WINCH_ENABLED == ENABLED
    case MAV_CMD_DO_WINCH:                             // Mission command to control winch
        do_winch(cmd);
        break;
#endif

    case MAV_CMD_DO_LAND_START:
        break;

    default:
        // unable to use the command, allow the vehicle to try the next command
        return false;
    }

    // always return success
    return true;
}

// exit_mission - function that is called once the mission completes
void ModeAuto::exit_mission()
{
    // play a tone
    AP_Notify::events.mission_complete = 1;
    // if we are not on the ground switch to loiter or land
    if (!copter.ap.land_complete) {
        // try to enter loiter but if that fails land
        if (!loiter_start()) {
            set_mode(Mode::Number::LAND, ModeReason::MISSION_END);
        }
    } else {
        // if we've landed it's safe to disarm
        copter.arming.disarm(AP_Arming::Method::MISSIONEXIT);
    }
}

// do_guided - start guided mode
bool ModeAuto::do_guided(const AP_Mission::Mission_Command& cmd)
{
    // only process guided waypoint if we are in guided mode
    if (copter.flightmode->mode_number() != Mode::Number::GUIDED && !(copter.flightmode->mode_number() == Mode::Number::AUTO && mode() == SubMode::NAVGUIDED)) {
        return false;
    }

    // switch to handle different commands
    switch (cmd.id) {

        case MAV_CMD_NAV_WAYPOINT:
        {
            // set wp_nav's destination
            Location dest(cmd.content.location);
            return copter.mode_guided.set_destination(dest);
        }

        case MAV_CMD_CONDITION_YAW:
            do_yaw(cmd);
            return true;

        default:
            // reject unrecognised command
            return false;
    }

    return true;
}

uint32_t ModeAuto::wp_distance() const
{
    switch (_mode) {
    case SubMode::CIRCLE:
        return copter.circle_nav->get_distance_to_target();
    case SubMode::WP:
    case SubMode::CIRCLE_MOVE_TO_EDGE:
    default:
        return wp_nav->get_wp_distance_to_destination();
    }
}

int32_t ModeAuto::wp_bearing() const
{
    switch (_mode) {
    case SubMode::CIRCLE:
        return copter.circle_nav->get_bearing_to_target();
    case SubMode::WP:
    case SubMode::CIRCLE_MOVE_TO_EDGE:
    default:
        return wp_nav->get_wp_bearing_to_destination();
    }
}

bool ModeAuto::get_wp(Location& destination) const
{
    switch (_mode) {
    case SubMode::NAVGUIDED:
        return copter.mode_guided.get_wp(destination);
    case SubMode::WP:
        return wp_nav->get_oa_wp_destination(destination);
    case SubMode::RTL:
        return copter.mode_rtl.get_wp(destination);
    default:
        return false;
    }
}

/*******************************************************************************
Verify command Handlers

Each type of mission element has a "verify" operation. The verify
operation returns true when the mission element has completed and we
should move onto the next mission element.
Return true if we do not recognize the command so that we move on to the next command
*******************************************************************************/

// verify_command - callback function called from ap-mission at 10hz or higher when a command is being run
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
bool ModeAuto::verify_command(const AP_Mission::Mission_Command& cmd)
{
    if (copter.flightmode != &copter.mode_auto) {
        return false;
    }

    bool cmd_complete = false;

    switch (cmd.id) {
    //
    // navigation commands
    //
    case MAV_CMD_NAV_TAKEOFF:
        cmd_complete = verify_takeoff();
        break;

    case MAV_CMD_NAV_WAYPOINT:
        cmd_complete = verify_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:
        cmd_complete = verify_land();
        break;

    case MAV_CMD_NAV_PAYLOAD_PLACE:
        cmd_complete = verify_payload_place();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:
        cmd_complete = verify_loiter_unlimited();
        break;

    case MAV_CMD_NAV_LOITER_TURNS:
        cmd_complete = verify_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        cmd_complete = verify_loiter_time(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:
        return verify_loiter_to_alt();

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        cmd_complete = verify_RTL();
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:
        cmd_complete = verify_spline_wp(cmd);
        break;

#if NAV_GUIDED == ENABLED
    case MAV_CMD_NAV_GUIDED_ENABLE:
        cmd_complete = verify_nav_guided_enable(cmd);
        break;
#endif

     case MAV_CMD_NAV_DELAY:
        cmd_complete = verify_nav_delay(cmd);
        break;

#if AP_SCRIPTING_ENABLED
    case MAV_CMD_NAV_SCRIPT_TIME:
        cmd_complete = verify_nav_script_time();
        break;
#endif

    ///
    /// conditional commands
    ///
    case MAV_CMD_CONDITION_DELAY:
        cmd_complete = verify_wait_delay();
        break;

    case MAV_CMD_CONDITION_DISTANCE:
        cmd_complete = verify_within_distance();
        break;

    case MAV_CMD_CONDITION_YAW:
        cmd_complete = verify_yaw();
        break;

    // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_MOUNT_CONTROL:
    case MAV_CMD_DO_GUIDED_LIMITS:
    case MAV_CMD_DO_FENCE_ENABLE:
    case MAV_CMD_DO_WINCH:
    case MAV_CMD_DO_LAND_START:
        cmd_complete = true;
        break;

    default:
        // error message
        gcs().send_text(MAV_SEVERITY_WARNING,"Skipping invalid cmd #%i",cmd.id);
        // return true if we do not recognize the command so that we move on to the next command
        cmd_complete = true;
        break;
    }


    // send message to GCS
    if (cmd_complete) {
        gcs().send_mission_item_reached_message(cmd.index);
    }

    return cmd_complete;
}

// takeoff_run - takeoff in auto mode
//      called by auto_run at 100hz or more
void ModeAuto::takeoff_run()
{
    // if the user doesn't want to raise the throttle we can set it automatically
    // note that this can defeat the disarm check on takeoff
    if ((copter.g2.auto_options & (int32_t)Options::AllowTakeOffWithoutRaisingThrottle) != 0) {
        copter.set_auto_armed(true);
    }
    auto_takeoff_run();
}

// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void ModeAuto::wp_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        wp_nav->wp_and_spline_init();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_thrust_vector_heading(wp_nav->get_thrust_vector(), auto_yaw.yaw(), auto_yaw.rate_cds());
    }
}

// auto_land_run - lands in auto mode
//      called by auto_run at 100hz or more
void ModeAuto::land_run()
{

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run normal landing or precision landing (if enabled)
    land_run_normal_or_precland();
}

// auto_rtl_run - rtl in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::rtl_run()
{
    // call regular rtl flight mode run function
    copter.mode_rtl.run(false);
}

// auto_circle_run - circle in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::circle_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // call circle controller
    copter.failsafe_terrain_set_status(copter.circle_nav->update());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(copter.circle_nav->get_thrust_vector(), target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_thrust_vector_heading(copter.circle_nav->get_thrust_vector(), auto_yaw.yaw());
    }
}

#if NAV_GUIDED == ENABLED || AP_SCRIPTING_ENABLED
// auto_nav_guided_run - allows control by external navigation controller
//      called by auto_run at 100hz or more
void ModeAuto::nav_guided_run()
{
    // call regular guided flight mode run function
    copter.mode_guided.run();
}
#endif  // NAV_GUIDED || AP_SCRIPTING_ENABLED

// auto_loiter_run - loiter in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::loiter_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        wp_nav->wp_and_spline_init();
        return;
    }

    // accept pilot input of yaw
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint and z-axis position controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), target_yaw_rate);
}

// auto_loiter_run - loiter to altitude in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::loiter_to_alt_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (is_disarmed_or_landed() || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // possibly just run the waypoint controller:
    if (!loiter_to_alt.reached_destination_xy) {
        loiter_to_alt.reached_destination_xy = wp_nav->reached_wp_destination_xy();
        if (!loiter_to_alt.reached_destination_xy) {
            wp_run();
            return;
        }
    }

    if (!loiter_to_alt.loiter_start_done) {
        // set horizontal speed and acceleration limits
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }

        _mode = SubMode::LOITER_TO_ALT;
        loiter_to_alt.loiter_start_done = true;
    }
    const float alt_error_cm = copter.current_loc.alt - loiter_to_alt.alt;
    if (fabsf(alt_error_cm) < 5.0) { // random numbers R US
        loiter_to_alt.reached_alt = true;
    } else if (alt_error_cm * loiter_to_alt.alt_error_cm < 0) {
        // we were above and are now below, or vice-versa
        loiter_to_alt.reached_alt = true;
    }
    loiter_to_alt.alt_error_cm = alt_error_cm;

    // loiter...

    land_run_horizontal_control();

    // Compute a vertical velocity demand such that the vehicle
    // approaches the desired altitude.
    float target_climb_rate = sqrt_controller(
        -alt_error_cm,
        pos_control->get_pos_z_p().kP(),
        pos_control->get_max_accel_z_cmss(),
        G_Dt);
    target_climb_rate = constrain_float(target_climb_rate, pos_control->get_max_speed_down_cms(), pos_control->get_max_speed_up_cms());

    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

    // update the vertical offset based on the surface measurement
    copter.surface_tracking.update_surface_offset();

    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);

    pos_control->update_z_controller();
}

// auto_payload_place_run - places an object in auto mode
//      called by auto_run at 100hz or more
void ModeAuto::payload_place_run()
{
    if (!payload_place_run_should_run()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    switch (nav_payload_place.state) {
    case PayloadPlaceStateType_FlyToLocation:
        return wp_run();
    case PayloadPlaceStateType_Calibrating_Hover_Start:
    case PayloadPlaceStateType_Calibrating_Hover:
        return payload_place_run_loiter();
    case PayloadPlaceStateType_Descending_Start:
    case PayloadPlaceStateType_Descending:
        return payload_place_run_descend();
    case PayloadPlaceStateType_Releasing_Start:
    case PayloadPlaceStateType_Releasing:
    case PayloadPlaceStateType_Released:
    case PayloadPlaceStateType_Ascending_Start:
        return payload_place_run_loiter();
    case PayloadPlaceStateType_Ascending:
    case PayloadPlaceStateType_Done:
        return wp_run();
    }
}

bool ModeAuto::payload_place_run_should_run()
{
    // must be armed
    if (!motors->armed()) {
        return false;
    }
    // must be auto-armed
    if (!copter.ap.auto_armed) {
        return false;
    }
    // must not be landed
    if (copter.ap.land_complete) {
        return false;
    }
    // interlock must be enabled (i.e. unsafe)
    if (!motors->get_interlock()) {
        return false;
    }

    return true;
}

void ModeAuto::payload_place_run_loiter()
{
    // loiter...
    land_run_horizontal_control();

    // call position controller
    pos_control->update_z_controller();
}

void ModeAuto::payload_place_run_descend()
{
    land_run_horizontal_control();
    land_run_vertical_control();
}

// terrain_adjusted_location: returns a Location with lat/lon from cmd
// and altitude from our current altitude adjusted for location
Location ModeAuto::terrain_adjusted_location(const AP_Mission::Mission_Command& cmd) const
{
    // convert to location class
    Location target_loc(cmd.content.location);

    // decide if we will use terrain following
    int32_t curr_terr_alt_cm, target_terr_alt_cm;
    if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, curr_terr_alt_cm) &&
        target_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, target_terr_alt_cm)) {
        curr_terr_alt_cm = MAX(curr_terr_alt_cm,200);
        // if using terrain, set target altitude to current altitude above terrain
        target_loc.set_alt_cm(curr_terr_alt_cm, Location::AltFrame::ABOVE_TERRAIN);
    } else {
        // set target altitude to current altitude above home
        target_loc.set_alt_cm(copter.current_loc.alt, Location::AltFrame::ABOVE_HOME);
    }
    return target_loc;
}

/********************************************************************************/
// Nav (Must) commands
/********************************************************************************/

// do_takeoff - initiate takeoff navigation command
void ModeAuto::do_takeoff(const AP_Mission::Mission_Command& cmd)
{
    // Set wp navigation target to safe altitude above current position
    takeoff_start(cmd.content.location);
}

Location ModeAuto::loc_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc) const
{
    Location ret(cmd.content.location);

    // use current lat, lon if zero
    if (ret.lat == 0 && ret.lng == 0) {
        ret.lat = default_loc.lat;
        ret.lng = default_loc.lng;
    }
    // use default altitude if not provided in cmd
    if (ret.alt == 0) {
        // set to default_loc's altitude but in command's alt frame
        // note that this may use the terrain database
        int32_t default_alt;
        if (default_loc.get_alt_cm(ret.get_alt_frame(), default_alt)) {
            ret.set_alt_cm(default_alt, ret.get_alt_frame());
        } else {
            // default to default_loc's altitude and frame
            ret.set_alt_cm(default_loc.alt, default_loc.get_alt_frame());
        }
    }
    return ret;
}

// do_nav_wp - initiate move to next waypoint
void ModeAuto::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // calculate default location used when lat, lon or alt is zero
    Location default_loc = copter.current_loc;
    if (wp_nav->is_active() && wp_nav->reached_wp_destination()) {
        if (!wp_nav->get_wp_destination_loc(default_loc)) {
            // this should never happen
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }

    // init wpnav and set origin if transitioning from takeoff
    if (!wp_nav->is_active()) {
        Vector3f stopping_point;
        if (_mode == SubMode::TAKEOFF) {
            Vector3p takeoff_complete_pos;
            if (auto_takeoff_get_position(takeoff_complete_pos)) {
                stopping_point = takeoff_complete_pos.tofloat();
            }
        }
        wp_nav->wp_and_spline_init(0, stopping_point);
    }

    // get waypoint's location from command and send to wp_nav
    const Location dest_loc = loc_from_cmd(cmd, default_loc);
    if (!wp_nav->set_wp_destination_loc(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    _mode = SubMode::WP;

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // set next destination if necessary
    if (!set_next_wp(cmd, dest_loc)) {
        // failure to set next destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

// checks the next mission command and adds it as a destination if necessary
// supports both straight line and spline waypoints
// cmd should be the current command
// default_loc should be the destination from the current_cmd but corrected for cases where user set lat, lon or alt to zero
// returns true on success, false on failure which should only happen due to a failure to retrieve terrain data
bool ModeAuto::set_next_wp(const AP_Mission::Mission_Command& current_cmd, const Location &default_loc)
{
    // do not add next wp if current command has a delay meaning the vehicle will stop at the destination
    if (current_cmd.p1 > 0) {
        return true;
    }

    // do not add next wp if there are no more navigation commands
    AP_Mission::Mission_Command next_cmd;
    if (!mission.get_next_nav_cmd(current_cmd.index+1, next_cmd)) {
        return true;
    }

    // whether vehicle should stop at the target position depends upon the next command
    switch (next_cmd.id) {
    case MAV_CMD_NAV_WAYPOINT:
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME: {
        const Location dest_loc = loc_from_cmd(current_cmd, default_loc);
        const Location next_dest_loc = loc_from_cmd(next_cmd, dest_loc);
        return wp_nav->set_wp_destination_next_loc(next_dest_loc);
    }
    case MAV_CMD_NAV_SPLINE_WAYPOINT: {
        // get spline's location and next location from command and send to wp_nav
        Location next_dest_loc, next_next_dest_loc;
        bool next_next_dest_loc_is_spline;
        get_spline_from_cmd(next_cmd, default_loc, next_dest_loc, next_next_dest_loc, next_next_dest_loc_is_spline);
        return wp_nav->set_spline_destination_next_loc(next_dest_loc, next_next_dest_loc, next_next_dest_loc_is_spline);
    }
    case MAV_CMD_NAV_LAND:
        // stop because we may change between rel,abs and terrain alt types
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
    case MAV_CMD_NAV_TAKEOFF:
        // always stop for RTL and takeoff commands
    default:
        // for unsupported commands it is safer to stop
        break;
    }

    return true;
}

// do_land - initiate landing procedure
void ModeAuto::do_land(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: check if we have already landed

    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to fly to location
        state = State::FlyToLocation;

        const Location target_loc = terrain_adjusted_location(cmd);

        wp_start(target_loc);
    } else {
        // set landing state
        state = State::Descending;

        // initialise landing controller
        land_start();
    }
}

// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
void ModeAuto::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    // convert back to location
    Location target_loc(cmd.content.location);

    // use current location if not provided
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        // To-Do: make this simpler
        Vector3f temp_pos;
        copter.wp_nav->get_wp_stopping_point_xy(temp_pos.xy());
        const Location temp_loc(temp_pos, Location::AltFrame::ABOVE_ORIGIN);
        target_loc.lat = temp_loc.lat;
        target_loc.lng = temp_loc.lng;
    }

    // use current altitude if not provided
    // To-Do: use z-axis stopping point instead of current alt
    if (target_loc.alt == 0) {
        // set to current altitude but in command's alt frame
        int32_t curr_alt;
        if (copter.current_loc.get_alt_cm(target_loc.get_alt_frame(),curr_alt)) {
            target_loc.set_alt_cm(curr_alt, target_loc.get_alt_frame());
        } else {
            // default to current altitude as alt-above-home
            target_loc.set_alt_cm(copter.current_loc.alt,
                                  copter.current_loc.get_alt_frame());
        }
    }

    // start way point navigator and provide it the desired location
    wp_start(target_loc);
}

// do_circle - initiate moving in a circle
void ModeAuto::do_circle(const AP_Mission::Mission_Command& cmd)
{
    const Location circle_center = loc_from_cmd(cmd, copter.current_loc);

    // calculate radius
    uint8_t circle_radius_m = HIGHBYTE(cmd.p1); // circle radius held in high byte of p1

    // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
    circle_movetoedge_start(circle_center, circle_radius_m);
}

// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
void ModeAuto::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = cmd.p1;     // units are (seconds)
}

// do_loiter_alt - initiate loitering at a point until a given altitude is reached
// note: caller should set yaw_mode
void ModeAuto::do_loiter_to_alt(const AP_Mission::Mission_Command& cmd)
{
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);
    _mode = SubMode::LOITER_TO_ALT;

    // if we aren't navigating to a location then we have to adjust
    // altitude for current location
    Location target_loc(cmd.content.location);
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        target_loc.lat = copter.current_loc.lat;
        target_loc.lng = copter.current_loc.lng;
    }

    if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, loiter_to_alt.alt)) {
        loiter_to_alt.reached_destination_xy = true;
        loiter_to_alt.reached_alt = true;
        gcs().send_text(MAV_SEVERITY_INFO, "bad do_loiter_to_alt");
        return;
    }
    loiter_to_alt.reached_destination_xy = false;
    loiter_to_alt.loiter_start_done = false;
    loiter_to_alt.reached_alt = false;
    loiter_to_alt.alt_error_cm = 0;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
}

// do_spline_wp - initiate move to next waypoint
void ModeAuto::do_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    // calculate default location used when lat, lon or alt is zero
    Location default_loc = copter.current_loc;
    if (wp_nav->is_active() && wp_nav->reached_wp_destination()) {
        if (!wp_nav->get_wp_destination_loc(default_loc)) {
            // this should never happen
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        }
    }

    // get spline's location and next location from command and send to wp_nav
    Location dest_loc, next_dest_loc;
    bool next_dest_loc_is_spline;
    get_spline_from_cmd(cmd, default_loc, dest_loc, next_dest_loc, next_dest_loc_is_spline);
    if (!wp_nav->set_spline_destination_loc(dest_loc, next_dest_loc, next_dest_loc_is_spline)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    _mode = SubMode::WP;

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // set next destination if necessary
    if (!set_next_wp(cmd, dest_loc)) {
        // failure to set next destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

// calculate locations required to build a spline curve from a mission command
// dest_loc is populated from cmd's location using default_loc in cases where the lat and lon or altitude is zero
// next_dest_loc and nest_dest_loc_is_spline is filled in with the following navigation command's location if it exists.  If it does not exist it is set to the dest_loc and false
void ModeAuto::get_spline_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc, Location& dest_loc, Location& next_dest_loc, bool& next_dest_loc_is_spline)
{
    dest_loc = loc_from_cmd(cmd, default_loc);

    // if there is no delay at the end of this segment get next nav command
    AP_Mission::Mission_Command temp_cmd;
    if (cmd.p1 == 0 && mission.get_next_nav_cmd(cmd.index+1, temp_cmd)) {
        next_dest_loc = loc_from_cmd(temp_cmd, dest_loc);
        next_dest_loc_is_spline = temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT;
    } else {
        next_dest_loc = dest_loc;
        next_dest_loc_is_spline = false;
    }
}

#if NAV_GUIDED == ENABLED
// do_nav_guided_enable - initiate accepting commands from external nav computer
void ModeAuto::do_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 > 0) {
        // start guided within auto
        nav_guided_start();
    }
}

// do_guided_limits - pass guided limits to guided controller
void ModeAuto::do_guided_limits(const AP_Mission::Mission_Command& cmd)
{
    copter.mode_guided.limit_set(
        cmd.p1 * 1000, // convert seconds to ms
        cmd.content.guided_limits.alt_min * 100.0f,    // convert meters to cm
        cmd.content.guided_limits.alt_max * 100.0f,    // convert meters to cm
        cmd.content.guided_limits.horiz_max * 100.0f); // convert meters to cm
}
#endif  // NAV_GUIDED

// do_nav_delay - Delay the next navigation command
void ModeAuto::do_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    nav_delay_time_start_ms = millis();

    if (cmd.content.nav_delay.seconds > 0) {
        // relative delay
        nav_delay_time_max_ms = cmd.content.nav_delay.seconds * 1000; // convert seconds to milliseconds
    } else {
        // absolute delay to utc time
        nav_delay_time_max_ms = AP::rtc().get_time_utc(cmd.content.nav_delay.hour_utc, cmd.content.nav_delay.min_utc, cmd.content.nav_delay.sec_utc, 0);
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Delaying %u sec", (unsigned)(nav_delay_time_max_ms/1000));
}

#if AP_SCRIPTING_ENABLED
// start accepting position, velocity and acceleration targets from lua scripts
void ModeAuto::do_nav_script_time(const AP_Mission::Mission_Command& cmd)
{
    // call regular guided flight mode initialisation
    if (copter.mode_guided.init(true)) {
        _mode = SubMode::NAV_SCRIPT_TIME;
        nav_scripting.done = false;
        nav_scripting.id++;
        nav_scripting.start_ms = millis();
        nav_scripting.command = cmd.content.nav_script_time.command;
        nav_scripting.timeout_s = cmd.content.nav_script_time.timeout_s;
        nav_scripting.arg1 = cmd.content.nav_script_time.arg1;
        nav_scripting.arg2 = cmd.content.nav_script_time.arg2;
    } else {
        // for safety we set nav_scripting to done to protect against the mission getting stuck
        nav_scripting.done = true;
    }
}
#endif


/********************************************************************************/
// Condition (May) commands
/********************************************************************************/

void ModeAuto::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = millis();
    condition_value = cmd.content.delay.seconds * 1000;     // convert seconds to milliseconds
}

void ModeAuto::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters * 100;
}

void ModeAuto::do_yaw(const AP_Mission::Mission_Command& cmd)
{
    auto_yaw.set_fixed_yaw(
        cmd.content.yaw.angle_deg,
        cmd.content.yaw.turn_rate_dps,
        cmd.content.yaw.direction,
        cmd.content.yaw.relative_angle > 0);
}

/********************************************************************************/
// Do (Now) commands
/********************************************************************************/



void ModeAuto::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.speed.target_ms > 0) {
        if (cmd.content.speed.speed_type == 2)  {
            copter.wp_nav->set_speed_up(cmd.content.speed.target_ms * 100.0f);
        } else if (cmd.content.speed.speed_type == 3)  {
            copter.wp_nav->set_speed_down(cmd.content.speed.target_ms * 100.0f);
        } else {
            copter.wp_nav->set_speed_xy(cmd.content.speed.target_ms * 100.0f);
        }
    }
}

void ModeAuto::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 || (cmd.content.location.lat == 0 && cmd.content.location.lng == 0 && cmd.content.location.alt == 0)) {
        if (!copter.set_home_to_current_location(false)) {
            // ignore failure
        }
    } else {
        if (!copter.set_home(cmd.content.location, false)) {
            // ignore failure
        }
    }
}

// do_roi - starts actions required by MAV_CMD_DO_SET_ROI
//          this involves either moving the camera to point at the ROI (region of interest)
//          and possibly rotating the copter to point at the ROI if our mount type does not support a yaw feature
// TO-DO: add support for other features of MAV_CMD_DO_SET_ROI including pointing at a given waypoint
void ModeAuto::do_roi(const AP_Mission::Mission_Command& cmd)
{
    auto_yaw.set_roi(cmd.content.location);
}

// point the camera to a specified angle
void ModeAuto::do_mount_control(const AP_Mission::Mission_Command& cmd)
{
#if HAL_MOUNT_ENABLED
    // if vehicle has a camera mount but it doesn't do pan control then yaw the entire vehicle instead
    if ((copter.camera_mount.get_mount_type() != copter.camera_mount.MountType::Mount_Type_None) &&
        !copter.camera_mount.has_pan_control()) {
        auto_yaw.set_yaw_angle_rate(cmd.content.mount_control.yaw,0.0f);
    }
    // pass the target angles to the camera mount
    copter.camera_mount.set_angle_targets(cmd.content.mount_control.roll, cmd.content.mount_control.pitch, cmd.content.mount_control.yaw);
#endif
}

#if WINCH_ENABLED == ENABLED
// control winch based on mission command
void ModeAuto::do_winch(const AP_Mission::Mission_Command& cmd)
{
    // Note: we ignore the gripper num parameter because we only support one gripper
    switch (cmd.content.winch.action) {
        case WINCH_RELAXED:
            g2.winch.relax();
            break;
        case WINCH_RELATIVE_LENGTH_CONTROL:
            g2.winch.release_length(cmd.content.winch.release_length);
            break;
        case WINCH_RATE_CONTROL:
            g2.winch.set_desired_rate(cmd.content.winch.release_rate);
            break;
        default:
            // do nothing
            break;
    }
}
#endif

// do_payload_place - initiate placing procedure
void ModeAuto::do_payload_place(const AP_Mission::Mission_Command& cmd)
{
    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to fly to location
        nav_payload_place.state = PayloadPlaceStateType_FlyToLocation;

        const Location target_loc = terrain_adjusted_location(cmd);

        wp_start(target_loc);
    } else {
        nav_payload_place.state = PayloadPlaceStateType_Calibrating_Hover_Start;

        // initialise placing controller
        payload_place_start();
    }
    nav_payload_place.descend_max = cmd.p1;
}

// do_RTL - start Return-to-Launch
void ModeAuto::do_RTL(void)
{
    // start rtl in auto flight mode
    rtl_start();
}

/********************************************************************************/
// Verify Nav (Must) commands
/********************************************************************************/

// verify_takeoff - check if we have completed the takeoff
bool ModeAuto::verify_takeoff()
{
#if LANDING_GEAR_ENABLED == ENABLED
    // if we have reached our destination
    if (auto_takeoff_complete) {
        // retract the landing gear
        copter.landinggear.retract_after_takeoff();
    }
#endif

    return auto_takeoff_complete;
}

// verify_land - returns true if landing has been completed
bool ModeAuto::verify_land()
{
    bool retval = false;

    switch (state) {
        case State::FlyToLocation:
            // check if we've reached the location
            if (copter.wp_nav->reached_wp_destination()) {
                // initialise landing controller
                land_start();

                // advance to next state
                state = State::Descending;
            }
            break;

        case State::Descending:
            // rely on THROTTLE_LAND mode to correctly update landing status
            retval = copter.ap.land_complete && (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE);
            if (retval && !mission.continue_after_land_check_for_takeoff() && copter.motors->armed()) {
                /*
                  we want to stop mission processing on land
                  completion. Disarm now, then return false. This
                  leaves mission state machine in the current NAV_LAND
                  mission item. After disarming the mission will reset
                */
                copter.arming.disarm(AP_Arming::Method::LANDED);
                retval = false;
            }
            break;

        default:
            // this should never happen
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            retval = true;
            break;
    }

    // true is returned if we've successfully landed
    return retval;
}

#define NAV_PAYLOAD_PLACE_DEBUGGING 0

#if NAV_PAYLOAD_PLACE_DEBUGGING
#include <stdio.h>
#define debug(fmt, args ...)  do {::fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

// verify_payload_place - returns true if placing has been completed
bool ModeAuto::verify_payload_place()
{
    const uint16_t hover_throttle_calibrate_time = 2000; // milliseconds
    const uint16_t descend_throttle_calibrate_time = 2000; // milliseconds
    const float hover_throttle_placed_fraction = 0.7; // i.e. if throttle is less than 70% of hover we have placed
    const float descent_throttle_placed_fraction = 0.9; // i.e. if throttle is less than 90% of descent throttle we have placed
    const uint16_t placed_time = 500; // how long we have to be below a throttle threshold before considering placed

    const float current_throttle_level = motors->get_throttle();
    const uint32_t now = AP_HAL::millis();

    // if we discover we've landed then immediately release the load:
    if (copter.ap.land_complete) {
        switch (nav_payload_place.state) {
        case PayloadPlaceStateType_FlyToLocation:
        case PayloadPlaceStateType_Calibrating_Hover_Start:
        case PayloadPlaceStateType_Calibrating_Hover:
        case PayloadPlaceStateType_Descending_Start:
        case PayloadPlaceStateType_Descending:
            gcs().send_text(MAV_SEVERITY_INFO, "PayloadPlace: landed");
            nav_payload_place.state = PayloadPlaceStateType_Releasing_Start;
            break;
        case PayloadPlaceStateType_Releasing_Start:
        case PayloadPlaceStateType_Releasing:
        case PayloadPlaceStateType_Released:
        case PayloadPlaceStateType_Ascending_Start:
        case PayloadPlaceStateType_Ascending:
        case PayloadPlaceStateType_Done:
            break;
        }
    }

    switch (nav_payload_place.state) {
    case PayloadPlaceStateType_FlyToLocation:
        if (!copter.wp_nav->reached_wp_destination()) {
            return false;
        }
        payload_place_start();
        return false;
    case PayloadPlaceStateType_Calibrating_Hover_Start:
        // hover for 1 second to get an idea of what our hover
        // throttle looks like
        debug("Calibrate start");
        nav_payload_place.hover_start_timestamp = now;
        nav_payload_place.state = PayloadPlaceStateType_Calibrating_Hover;
        FALLTHROUGH;
    case PayloadPlaceStateType_Calibrating_Hover: {
        if (now - nav_payload_place.hover_start_timestamp < hover_throttle_calibrate_time) {
            // still calibrating...
            debug("Calibrate Timer: %d", now - nav_payload_place.hover_start_timestamp);
            return false;
        }
        // we have a valid calibration.  Hopefully.
        nav_payload_place.hover_throttle_level = current_throttle_level;
        const float hover_throttle_delta = fabsf(nav_payload_place.hover_throttle_level - motors->get_throttle_hover());
        gcs().send_text(MAV_SEVERITY_INFO, "hover throttle delta: %f", static_cast<double>(hover_throttle_delta));
        nav_payload_place.state = PayloadPlaceStateType_Descending_Start;
        }
        FALLTHROUGH;
    case PayloadPlaceStateType_Descending_Start:
        nav_payload_place.descend_start_timestamp = now;
        nav_payload_place.descend_start_altitude = inertial_nav.get_position_z_up_cm();
        nav_payload_place.descend_throttle_level = 0;
        nav_payload_place.state = PayloadPlaceStateType_Descending;
        FALLTHROUGH;
    case PayloadPlaceStateType_Descending:
        // make sure we don't descend too far:
        debug("descended: %f cm (%f cm max)", (nav_payload_place.descend_start_altitude - inertial_nav.get_position_z_up_cm()), nav_payload_place.descend_max);
        if (!is_zero(nav_payload_place.descend_max) &&
            nav_payload_place.descend_start_altitude - inertial_nav.get_position_z_up_cm()  > nav_payload_place.descend_max) {
            nav_payload_place.state = PayloadPlaceStateType_Ascending;
            gcs().send_text(MAV_SEVERITY_WARNING, "Reached maximum descent");
            return false; // we'll do any cleanups required next time through the loop
        }
        // see if we've been descending long enough to calibrate a descend-throttle-level:
        if (is_zero(nav_payload_place.descend_throttle_level) &&
            now - nav_payload_place.descend_start_timestamp > descend_throttle_calibrate_time) {
            nav_payload_place.descend_throttle_level = current_throttle_level;
        }
        // watch the throttle to determine whether the load has been placed
        // debug("hover ratio: %f   descend ratio: %f\n", current_throttle_level/nav_payload_place.hover_throttle_level, ((nav_payload_place.descend_throttle_level == 0) ? -1.0f : current_throttle_level/nav_payload_place.descend_throttle_level));
        if (current_throttle_level/nav_payload_place.hover_throttle_level > hover_throttle_placed_fraction &&
            (is_zero(nav_payload_place.descend_throttle_level) ||
             current_throttle_level/nav_payload_place.descend_throttle_level > descent_throttle_placed_fraction)) {
            // throttle is above both threshold ratios (or above hover threshold ration and descent threshold ratio not yet valid)
            nav_payload_place.place_start_timestamp = 0;
            return false;
        }
        if (nav_payload_place.place_start_timestamp == 0) {
            // we've only just now hit the correct throttle level
            nav_payload_place.place_start_timestamp = now;
            return false;
        } else if (now - nav_payload_place.place_start_timestamp < placed_time) {
            // keep going down....
            debug("Place Timer: %d", now - nav_payload_place.place_start_timestamp);
            return false;
        }
        nav_payload_place.state = PayloadPlaceStateType_Releasing_Start;
        FALLTHROUGH;
    case PayloadPlaceStateType_Releasing_Start:
#if GRIPPER_ENABLED == ENABLED
        if (g2.gripper.valid()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Releasing the gripper");
            g2.gripper.release();
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Gripper not valid");
            nav_payload_place.state = PayloadPlaceStateType_Ascending_Start;
            break;
        }
#else
        gcs().send_text(MAV_SEVERITY_INFO, "Gripper code disabled");
#endif
        nav_payload_place.state = PayloadPlaceStateType_Releasing;
        FALLTHROUGH;
    case PayloadPlaceStateType_Releasing:
#if GRIPPER_ENABLED == ENABLED
        if (g2.gripper.valid() && !g2.gripper.released()) {
            return false;
        }
#endif
        nav_payload_place.state = PayloadPlaceStateType_Released;
        FALLTHROUGH;
    case PayloadPlaceStateType_Released: {
        nav_payload_place.state = PayloadPlaceStateType_Ascending_Start;
        }
        FALLTHROUGH;
    case PayloadPlaceStateType_Ascending_Start: {
        Location target_loc(inertial_nav.get_position_neu_cm(), Location::AltFrame::ABOVE_ORIGIN);
        target_loc.alt = nav_payload_place.descend_start_altitude;
        wp_start(target_loc);
        nav_payload_place.state = PayloadPlaceStateType_Ascending;
        }
        FALLTHROUGH;
    case PayloadPlaceStateType_Ascending:
        if (!copter.wp_nav->reached_wp_destination()) {
            return false;
        }
        nav_payload_place.state = PayloadPlaceStateType_Done;
        FALLTHROUGH;
    case PayloadPlaceStateType_Done:
        return true;
    default:
        // this should never happen
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return true;
    }
    // should never get here
    return true;
}
#undef debug

bool ModeAuto::verify_loiter_unlimited()
{
    return false;
}

// verify_loiter_time - check if we have loitered long enough
bool ModeAuto::verify_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    // return immediately if we haven't reached our destination
    if (!copter.wp_nav->reached_wp_destination()) {
        return false;
    }

    // start our loiter timer
    if ( loiter_time == 0 ) {
        loiter_time = millis();
    }

    // check if loiter timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }

    return false;
}

// verify_loiter_to_alt - check if we have reached both destination
// (roughly) and altitude (precisely)
bool ModeAuto::verify_loiter_to_alt() const
{
    if (loiter_to_alt.reached_destination_xy &&
        loiter_to_alt.reached_alt) {
        return true;
    }
    return false;
}

// verify_RTL - handles any state changes required to implement RTL
// do_RTL should have been called once first to initialise all variables
// returns true with RTL has completed successfully
bool ModeAuto::verify_RTL()
{
    return (copter.mode_rtl.state_complete() && 
            (copter.mode_rtl.state() == ModeRTL::SubMode::FINAL_DESCENT || copter.mode_rtl.state() == ModeRTL::SubMode::LAND) &&
            (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE));
}

/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

bool ModeAuto::verify_wait_delay()
{
    if (millis() - condition_start > (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

bool ModeAuto::verify_within_distance()
{
    if (wp_distance() < (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

// verify_yaw - return true if we have reached the desired heading
bool ModeAuto::verify_yaw()
{
    // make sure still in fixed yaw mode, the waypoint controller often retakes control of yaw as it executes a new waypoint command
    auto_yaw.set_mode(AUTO_YAW_FIXED);

    // check if we are within 2 degrees of the target heading
    return auto_yaw.fixed_yaw_slew_finished() && (fabsf(wrap_180_cd(ahrs.yaw_sensor-auto_yaw.yaw())) <= 200);
}

// verify_nav_wp - check if we have reached the next way point
bool ModeAuto::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if ( !copter.wp_nav->reached_wp_destination() ) {
        return false;
    }

    // start timer if necessary
    if (loiter_time == 0) {
        loiter_time = millis();
        if (loiter_time_max > 0) {
            // play a tone
            AP_Notify::events.waypoint_complete = 1;
        }
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        if (loiter_time_max == 0) {
            // play a tone
            AP_Notify::events.waypoint_complete = 1;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }
    return false;
}

// verify_circle - check if we have circled the point enough
bool ModeAuto::verify_circle(const AP_Mission::Mission_Command& cmd)
{
    // check if we've reached the edge
    if (mode() == SubMode::CIRCLE_MOVE_TO_EDGE) {
        if (copter.wp_nav->reached_wp_destination()) {
            // start circling
            circle_start();
        }
        return false;
    }

    // check if we have completed circling
    return fabsf(copter.circle_nav->get_angle_total()/float(M_2PI)) >= LOWBYTE(cmd.p1);
}

// verify_spline_wp - check if we have reached the next way point using spline
bool ModeAuto::verify_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if ( !copter.wp_nav->reached_wp_destination() ) {
        return false;
    }

    // start timer if necessary
    if (loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }
    return false;
}

#if NAV_GUIDED == ENABLED
// verify_nav_guided - check if we have breached any limits
bool ModeAuto::verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    // if disabling guided mode then immediately return true so we move to next command
    if (cmd.p1 == 0) {
        return true;
    }

    // check time and position limits
    return copter.mode_guided.limit_check();
}
#endif  // NAV_GUIDED

// verify_nav_delay - check if we have waited long enough
bool ModeAuto::verify_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    if (millis() - nav_delay_time_start_ms > nav_delay_time_max_ms) {
        nav_delay_time_max_ms = 0;
        return true;
    }
    return false;
}

#if AP_SCRIPTING_ENABLED
// check if verify_nav_script_time command has completed
bool ModeAuto::verify_nav_script_time()
{
    // if done or timeout then return true
    if (nav_scripting.done ||
        ((nav_scripting.timeout_s > 0) &&
         (AP_HAL::millis() - nav_scripting.start_ms) > (nav_scripting.timeout_s * 1000))) {
        return true;
    }
    return false;
}
#endif

// pause - Prevent aircraft from progressing along the track
bool ModeAuto::pause()
{
    // do not pause if already paused or not in the WP sub mode or already reached to the destination
    if(wp_nav->paused() || _mode != SubMode::WP || wp_nav->reached_wp_destination()) {
        return false;
    }

    wp_nav->set_pause();
    return true;
}

// resume - Allow aircraft to progress along the track
bool ModeAuto::resume()
{
    // do not resume if not paused before
    if(!wp_nav->paused()) {
        return false;
    }

    wp_nav->set_resume();
    return true;
}

#endif
Mode_autorotate.cpp
/* -----------------------------------------------------------------------------------------
    This is currently a SITL only function until the project is complete.
    To trial this in SITL you will need to use Real Flight 8.
    Instructions for how to set this up in SITL can be found here:
    https://discuss.ardupilot.org/t/autonomous-autorotation-gsoc-project-blog/42139
 -----------------------------------------------------------------------------------------*/

#include "Copter.h"
#include <AC_Autorotation/AC_Autorotation.h>
#include "mode.h"

#include <utility>

#if MODE_AUTOROTATE_ENABLED == ENABLED

#define AUTOROTATE_ENTRY_TIME          2.0f    // (s) number of seconds that the entry phase operates for
#define BAILOUT_MOTOR_RAMP_TIME        1.0f    // (s) time set on bailout ramp up timer for motors - See AC_MotorsHeli_Single
#define HEAD_SPEED_TARGET_RATIO        1.0f    // Normalised target main rotor head speed (unit: -)

bool ModeAutorotate::init(bool ignore_checks)
{
#if FRAME_CONFIG != HELI_FRAME
    // Only allow trad heli to use autorotation mode
    return false;
#endif

    // Check that mode is enabled
    if (!g2.arot.is_enable()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Autorot Mode Not Enabled");
        return false;
    }

    // Check that interlock is disengaged
    if (motors->get_interlock()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Autorot Mode Change Fail: Interlock Engaged");
        return false;
    }

    // Initialise controllers
    // This must be done before RPM value is fetched
    g2.arot.init_hs_controller();
    g2.arot.init_fwd_spd_controller();

    // Retrive rpm and start rpm sensor health checks
    _initial_rpm = g2.arot.get_rpm(true);

    // Display message 
    gcs().send_text(MAV_SEVERITY_INFO, "Autorotation initiated");

     // Set all inial flags to on
    _flags.entry_initial = 1;
    _flags.ss_glide_initial = 1;
    _flags.flare_initial = 1;
    _flags.touch_down_initial = 1;
    _flags.level_initial = 1;
    _flags.break_initial = 1;
    _flags.straight_ahead_initial = 1;
    _flags.bail_out_initial = 1;
    _msg_flags.bad_rpm = true;

    // Setting default starting switches
    phase_switch = Autorotation_Phase::ENTRY;

    // Set entry timer
    _entry_time_start_ms = millis();

    // The decay rate to reduce the head speed from the current to the target
    _hs_decay = ((_initial_rpm/g2.arot.get_hs_set_point()) - HEAD_SPEED_TARGET_RATIO) / AUTOROTATE_ENTRY_TIME;

    return true;
}



void ModeAutorotate::run()
{
    // Check if interlock becomes engaged
    if (motors->get_interlock() && !copter.ap.land_complete) {
        phase_switch = Autorotation_Phase::BAIL_OUT;
    } else if (motors->get_interlock() && copter.ap.land_complete) {
        // Aircraft is landed and no need to bail out
        set_mode(copter.prev_control_mode, ModeReason::AUTOROTATION_BAILOUT);
    }

    // Current time
    uint32_t now = millis(); //milliseconds

    // Initialise internal variables
    float curr_vel_z = inertial_nav.get_velocity_z_up_cms();   // Current vertical descent

    //----------------------------------------------------------------
    //                  State machine logic
    //----------------------------------------------------------------

     // Setting default phase switch positions
     nav_pos_switch = Navigation_Decision::USER_CONTROL_STABILISED;

    // Timer from entry phase to progress to glide phase
    if (phase_switch == Autorotation_Phase::ENTRY){

        if ((now - _entry_time_start_ms)/1000.0f > AUTOROTATE_ENTRY_TIME) {
            // Flight phase can be progressed to steady state glide
            phase_switch = Autorotation_Phase::SS_GLIDE;
        }

    }


    //----------------------------------------------------------------
    //                  State machine actions
    //----------------------------------------------------------------
    switch (phase_switch) {

        case Autorotation_Phase::ENTRY:
        {
            // Entry phase functions to be run only once
            if (_flags.entry_initial == 1) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Entry Phase");
                #endif

                // Set following trim low pass cut off frequency
                g2.arot.set_col_cutoff_freq(g2.arot.get_col_entry_freq());

                // Target head speed is set to rpm at initiation to prevent unwanted changes in attitude
                _target_head_speed = _initial_rpm/g2.arot.get_hs_set_point();

                // Set desired forward speed target
                g2.arot.set_desired_fwd_speed();

                // Prevent running the initial entry functions again
                _flags.entry_initial = 0;

            }

            // Slowly change the target head speed until the target head speed matches the parameter defined value
            if (g2.arot.get_rpm() > HEAD_SPEED_TARGET_RATIO*1.005f  ||  g2.arot.get_rpm() < HEAD_SPEED_TARGET_RATIO*0.995f) {
                _target_head_speed -= _hs_decay*G_Dt;
            } else {
                _target_head_speed = HEAD_SPEED_TARGET_RATIO;
            }

            // Set target head speed in head speed controller
            g2.arot.set_target_head_speed(_target_head_speed);

            // Run airspeed/attitude controller
            g2.arot.set_dt(G_Dt);
            g2.arot.update_forward_speed_controller();

            // Retrieve pitch target
            _pitch_target = g2.arot.get_pitch();

            // Update controllers
            _flags.bad_rpm = g2.arot.update_hs_glide_controller(G_Dt); //run head speed/ collective controller

            break;
        }

        case Autorotation_Phase::SS_GLIDE:
        {
            // Steady state glide functions to be run only once
            if (_flags.ss_glide_initial == 1) {

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "SS Glide Phase");
                #endif

                // Set following trim low pass cut off frequency
                g2.arot.set_col_cutoff_freq(g2.arot.get_col_glide_freq());

                // Set desired forward speed target
                g2.arot.set_desired_fwd_speed();

                // Set target head speed in head speed controller
                _target_head_speed = HEAD_SPEED_TARGET_RATIO;  //Ensure target hs is set to glide incase hs hasent reached target for glide
                g2.arot.set_target_head_speed(_target_head_speed);

                // Prevent running the initial glide functions again
                _flags.ss_glide_initial = 0;
            }

            // Run airspeed/attitude controller
            g2.arot.set_dt(G_Dt);
            g2.arot.update_forward_speed_controller();

            // Retrieve pitch target 
            _pitch_target = g2.arot.get_pitch();

            // Update head speed/ collective controller
            _flags.bad_rpm = g2.arot.update_hs_glide_controller(G_Dt); 
            // Attitude controller is updated in navigation switch-case statements

            break;
        }

        case Autorotation_Phase::FLARE:
        case Autorotation_Phase::TOUCH_DOWN:
        {
            break;
        }

        case Autorotation_Phase::BAIL_OUT:
        {
        if (_flags.bail_out_initial == 1) {
                // Functions and settings to be done once are done here.

                #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    gcs().send_text(MAV_SEVERITY_INFO, "Bailing Out of Autorotation");
                #endif

                // Set bail out timer remaining equal to the paramter value, bailout time 
                // cannot be less than the motor spool-up time: BAILOUT_MOTOR_RAMP_TIME.
                _bail_time = MAX(g2.arot.get_bail_time(),BAILOUT_MOTOR_RAMP_TIME+0.1f);

                // Set bail out start time
                _bail_time_start_ms = now;

                // Set initial target vertical speed
                _desired_v_z = curr_vel_z;

                // Initialise position and desired velocity
                if (!pos_control->is_active_z()) {
                    pos_control->relax_z_controller(g2.arot.get_last_collective());
                }

                // Get pilot parameter limits
                const float pilot_spd_dn = -get_pilot_speed_dn();
                const float pilot_spd_up = g.pilot_speed_up;

                float pilot_des_v_z = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
                pilot_des_v_z = constrain_float(pilot_des_v_z, pilot_spd_dn, pilot_spd_up);

                // Calculate target climb rate adjustment to transition from bail out descent speed to requested climb rate on stick.
                _target_climb_rate_adjust = (curr_vel_z - pilot_des_v_z)/(_bail_time - BAILOUT_MOTOR_RAMP_TIME); //accounting for 0.5s motor spool time

                // Calculate pitch target adjustment rate to return to level
                _target_pitch_adjust = _pitch_target/_bail_time;

                // set vertical speed and acceleration limits
                pos_control->set_max_speed_accel_z(curr_vel_z, pilot_spd_up, fabsf(_target_climb_rate_adjust));
                pos_control->set_correction_speed_accel_z(curr_vel_z, pilot_spd_up, fabsf(_target_climb_rate_adjust));

                motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

                _flags.bail_out_initial = 0;
            }

        if ((now - _bail_time_start_ms)/1000.0f >= BAILOUT_MOTOR_RAMP_TIME) {
            // Update desired vertical speed and pitch target after the bailout motor ramp timer has completed
            _desired_v_z -= _target_climb_rate_adjust*G_Dt;
            _pitch_target -= _target_pitch_adjust*G_Dt;
        }
        // Set position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(_desired_v_z);

        // Update controllers
        pos_control->update_z_controller();

        if ((now - _bail_time_start_ms)/1000.0f >= _bail_time) {
            // Bail out timer complete.  Change flight mode. Do not revert back to auto. Prevent aircraft
            // from continuing mission and potentially flying further away after a power failure.
            if (copter.prev_control_mode == Mode::Number::AUTO) {
                set_mode(Mode::Number::ALT_HOLD, ModeReason::AUTOROTATION_BAILOUT);
            } else {
                set_mode(copter.prev_control_mode, ModeReason::AUTOROTATION_BAILOUT);
            }
        }

        break;
        }
    }


    switch (nav_pos_switch) {

        case Navigation_Decision::USER_CONTROL_STABILISED:
        {
            // Operator is in control of roll and yaw.  Controls act as if in stabilise flight mode.  Pitch 
            // is controlled by speed-height controller.
            float pilot_roll, pilot_pitch;
            get_pilot_desired_lean_angles(pilot_roll, pilot_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

            // Get pilot's desired yaw rate
            float pilot_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

            // Pitch target is calculated in autorotation phase switch above
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pilot_roll, _pitch_target, pilot_yaw_rate);
            break;
        }

        case Navigation_Decision::STRAIGHT_AHEAD:
        case Navigation_Decision::INTO_WIND:
        case Navigation_Decision::NEAREST_RALLY:
        {
            break;
        }
    }

    // Output warning messaged if rpm signal is bad
    if (_flags.bad_rpm) {
        warning_message(1);
    }

} // End function run()

void ModeAutorotate::warning_message(uint8_t message_n)
{
    switch (message_n) {
        case 1:
        {
            if (_msg_flags.bad_rpm) {
                // Bad rpm sensor health.
                gcs().send_text(MAV_SEVERITY_INFO, "Warning: Poor RPM Sensor Health");
                gcs().send_text(MAV_SEVERITY_INFO, "Action: Minimum Collective Applied");
                _msg_flags.bad_rpm = false;
            }
            break;
        }
    }
}

#endif
Mode_autotune.cpp
#include "Copter.h"

/*
  autotune mode is a wrapper around the AC_AutoTune library
 */

#if AUTOTUNE_ENABLED == ENABLED

bool AutoTune::init()
{
    // only allow AutoTune from some flight modes, for example Stabilize, AltHold,  PosHold or Loiter modes
    if (!copter.flightmode->allows_autotune()) {
        return false;
    }

    // ensure throttle is above zero
    if (copter.ap.throttle_zero) {
        return false;
    }

    // ensure we are flying
    if (!copter.motors->armed() || !copter.ap.auto_armed || copter.ap.land_complete) {
        return false;
    }

    // use position hold while tuning if we were in QLOITER
    bool position_hold = (copter.flightmode->mode_number() == Mode::Number::LOITER || copter.flightmode->mode_number() == Mode::Number::POSHOLD);

    return init_internals(position_hold,
                          copter.attitude_control,
                          copter.pos_control,
                          copter.ahrs_view,
                          &copter.inertial_nav);
}

void AutoTune::run()
{
    // apply SIMPLE mode transform to pilot inputs
    copter.update_simple_mode();

    // reset target lean angles and heading while landed
    if (copter.ap.land_complete) {
        // we are landed, shut down
        float target_climb_rate = get_pilot_desired_climb_rate_cms();

        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        }
        copter.attitude_control->reset_rate_controller_I_terms_smoothly();
        copter.attitude_control->reset_yaw_target_and_rate();

        float target_roll, target_pitch, target_yaw_rate;
        get_pilot_desired_rp_yrate_cd(target_roll, target_pitch, target_yaw_rate);

        copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        copter.pos_control->relax_z_controller(0.0f);
        copter.pos_control->update_z_controller();
    } else {
        // run autotune mode
        AC_AutoTune::run();
    }
}


/*
  get stick input climb rate
 */
float AutoTune::get_pilot_desired_climb_rate_cms(void) const
{
    float target_climb_rate = copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in());

    // get avoidance adjusted climb rate
    target_climb_rate = copter.mode_autotune.get_avoidance_adjusted_climbrate(target_climb_rate);

    return target_climb_rate;
}

/*
  get stick roll, pitch and yaw rate
 */
void AutoTune::get_pilot_desired_rp_yrate_cd(float &des_roll_cd, float &des_pitch_cd, float &yaw_rate_cds)
{
    copter.mode_autotune.get_pilot_desired_lean_angles(des_roll_cd, des_pitch_cd, copter.aparm.angle_max,
                                                       copter.attitude_control->get_althold_lean_angle_max_cd());
    yaw_rate_cds = copter.mode_autotune.get_pilot_desired_yaw_rate(copter.channel_yaw->norm_input_dz());
}

/*
  setup z controller velocity and accel limits
 */
void AutoTune::init_z_limits()
{
    // set vertical speed and acceleration limits
    copter.pos_control->set_max_speed_accel_z(-copter.get_pilot_speed_dn(), copter.g.pilot_speed_up, copter.g.pilot_accel_z);
    copter.pos_control->set_correction_speed_accel_z(-copter.get_pilot_speed_dn(), copter.g.pilot_speed_up, copter.g.pilot_accel_z);
}

void AutoTune::log_pids()
{
    copter.logger.Write_PID(LOG_PIDR_MSG, copter.attitude_control->get_rate_roll_pid().get_pid_info());
    copter.logger.Write_PID(LOG_PIDP_MSG, copter.attitude_control->get_rate_pitch_pid().get_pid_info());
    copter.logger.Write_PID(LOG_PIDY_MSG, copter.attitude_control->get_rate_yaw_pid().get_pid_info());
}

/*
  check if we have a good position estimate
 */
bool AutoTune::position_ok()
{
    return copter.position_ok();
}

/*
  initialise autotune mode
*/
bool ModeAutoTune::init(bool ignore_checks)
{
    return autotune.init();
}

void ModeAutoTune::run()
{
    autotune.run();
}

void ModeAutoTune::save_tuning_gains()
{
    autotune.save_tuning_gains();
}

void ModeAutoTune::exit()
{
    autotune.stop();
}

void ModeAutoTune::reset()
{
    autotune.reset();
}

#endif  // AUTOTUNE_ENABLED == ENABLED
Mode_avoid_adsb.cpp
#include "Copter.h"

/*
 * control_avoid.cpp - init and run calls for AP_Avoidance's AVOID flight mode
 *
 * This re-uses GUIDED mode functions but does not interfere with the GCS or companion computer's
 * use of guided mode because the velocity requests arrive from different sources (i.e MAVLink messages
 * for GCS and Companion Computers vs the AP_Avoidance_Copter class for adsb avoidance) and inputs from
 * each source are only accepted and processed in the appropriate flight mode.
 */

// initialise avoid_adsb controller
bool ModeAvoidADSB::init(const bool ignore_checks)
{
    // re-use guided mode
    return ModeGuided::init(ignore_checks);
}

bool ModeAvoidADSB::set_velocity(const Vector3f& velocity_neu)
{
    // check flight mode
    if (copter.flightmode->mode_number() != Mode::Number::AVOID_ADSB) {
        return false;
    }

    // re-use guided mode's velocity controller
    ModeGuided::set_velocity(velocity_neu);
    return true;
}

// runs the AVOID_ADSB controller
void ModeAvoidADSB::run()
{
    // re-use guided mode's velocity controller
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode
    ModeGuided::run();
}
Mode_brake.cpp
#include "Copter.h"

#if MODE_BRAKE_ENABLED == ENABLED

/*
 * Init and run calls for brake flight mode
 */

// brake_init - initialise brake controller
bool ModeBrake::init(bool ignore_checks)
{
    // initialise pos controller speed and acceleration
    pos_control->set_max_speed_accel_xy(inertial_nav.get_velocity_neu_cms().length(), BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_xy(inertial_nav.get_velocity_neu_cms().length(), BRAKE_MODE_DECEL_RATE);

    // initialise position controller
    pos_control->init_xy_controller();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    _timeout_ms = 0;

    return true;
}

// brake_run - runs the brake controller
// should be called at 100hz or more
void ModeBrake::run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        pos_control->relax_velocity_controller_xy();
        pos_control->relax_z_controller(0.0f);
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // relax stop target if we might be landed
    if (copter.ap.land_complete_maybe) {
        pos_control->soften_for_landing_xy();
    }

    // use position controller to stop
    Vector2f vel;
    Vector2f accel;
    pos_control->input_vel_accel_xy(vel, accel);
    pos_control->update_xy_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);

    pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
    pos_control->update_z_controller();

    if (_timeout_ms != 0 && millis()-_timeout_start >= _timeout_ms) {
        if (!copter.set_mode(Mode::Number::LOITER, ModeReason::BRAKE_TIMEOUT)) {
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::BRAKE_TIMEOUT);
        }
    }
}

void ModeBrake::timeout_to_loiter_ms(uint32_t timeout_ms)
{
    _timeout_start = millis();
    _timeout_ms = timeout_ms;
}

#endif
Mode_circle.cpp
#include "Copter.h"

#if MODE_CIRCLE_ENABLED == ENABLED

/*
 * Init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool ModeCircle::init(bool ignore_checks)
{
    pilot_yaw_override = false;
    speed_changing = false;

    // set speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    copter.circle_nav->init();

    return true;
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void ModeCircle::run()
{
    // set speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // get pilot's desired yaw rate (or zero if in radio failsafe)
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    if (!is_zero(target_yaw_rate)) {
        pilot_yaw_override = true;
    }

    // Check for any change in params and update in real time
    copter.circle_nav->check_param_change();

    // pilot changes to circle rate and radius
    // skip if in radio failsafe
    if (!copter.failsafe.radio && copter.circle_nav->pilot_control_enabled()) {
        // update the circle controller's radius target based on pilot pitch stick inputs
        const float radius_current = copter.circle_nav->get_radius();           // circle controller's radius target, which begins as the circle_radius parameter
        const float pitch_stick = channel_pitch->norm_input_dz();               // pitch stick normalized -1 to 1
        const float nav_speed = copter.wp_nav->get_default_speed_xy();          // copter WP_NAV parameter speed
        const float radius_pilot_change = (pitch_stick * nav_speed) * G_Dt;     // rate of change (pitch stick up reduces the radius, as in moving forward)
        const float radius_new = MAX(radius_current + radius_pilot_change,0);   // new radius target

        if (!is_equal(radius_current, radius_new)) {
            copter.circle_nav->set_radius_cm(radius_new);
        }

        // update the orbicular rate target based on pilot roll stick inputs
        // skip if using CH6 tuning knob for circle rate
        if (g.radio_tuning != TUNING_CIRCLE_RATE) {
            const float roll_stick = channel_roll->norm_input_dz();         // roll stick normalized -1 to 1

            if (is_zero(roll_stick)) {
                // no speed change, so reset speed changing flag
                speed_changing = false;
            } else {
                const float rate = copter.circle_nav->get_rate();           // circle controller's rate target, which begins as the circle_rate parameter
                const float rate_current = copter.circle_nav->get_rate_current(); // current adjusted rate target, which is probably different from _rate
                const float rate_pilot_change = (roll_stick * G_Dt);        // rate of change from 0 to 1 degrees per second
                float rate_new = rate_current;                              // new rate target
                if (is_positive(rate)) {
                    // currently moving clockwise, constrain 0 to 90
                    rate_new = constrain_float(rate_current + rate_pilot_change, 0, 90);

                } else if (is_negative(rate)) {
                    // currently moving counterclockwise, constrain -90 to 0
                    rate_new = constrain_float(rate_current + rate_pilot_change, -90, 0);

                } else if (is_zero(rate) && !speed_changing) {
                    // Stopped, pilot has released the roll stick, and pilot now wants to begin moving with the roll stick
                    rate_new = rate_pilot_change;
                }

                speed_changing = true;
                copter.circle_nav->set_rate(rate_new);
            }
        }
    }

    // get pilot desired climb rate (or zero if in radio failsafe)
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // update the vertical offset based on the surface measurement
    copter.surface_tracking.update_surface_offset();

    copter.failsafe_terrain_set_status(copter.circle_nav->update(target_climb_rate));

    // call attitude controller
    if (pilot_yaw_override) {
        attitude_control->input_thrust_vector_rate_heading(copter.circle_nav->get_thrust_vector(), target_yaw_rate);
    } else {
        attitude_control->input_thrust_vector_heading(copter.circle_nav->get_thrust_vector(), copter.circle_nav->get_yaw());
    }
    pos_control->update_z_controller();
}

uint32_t ModeCircle::wp_distance() const
{
    return copter.circle_nav->get_distance_to_target();
}

int32_t ModeCircle::wp_bearing() const
{
    return copter.circle_nav->get_bearing_to_target();
}

#endif
Mode_drift.cpp
#include "Copter.h"

#if MODE_DRIFT_ENABLED == ENABLED

/*
 * Init and run calls for drift flight mode
 */

#ifndef DRIFT_SPEEDGAIN
 # define DRIFT_SPEEDGAIN 8.0f
#endif
#ifndef DRIFT_SPEEDLIMIT
 # define DRIFT_SPEEDLIMIT 560.0f
#endif

#ifndef DRIFT_THR_ASSIST_GAIN
 # define DRIFT_THR_ASSIST_GAIN 0.0018f    // gain controlling amount of throttle assistance
#endif

#ifndef DRIFT_THR_ASSIST_MAX
 # define DRIFT_THR_ASSIST_MAX  0.3f    // maximum assistance throttle assist will provide
#endif

#ifndef DRIFT_THR_MIN
 # define DRIFT_THR_MIN         0.213f  // throttle assist will be active when pilot's throttle is above this value
#endif
#ifndef DRIFT_THR_MAX
 # define DRIFT_THR_MAX         0.787f  // throttle assist will be active when pilot's throttle is below this value
#endif

// drift_init - initialise drift controller
bool ModeDrift::init(bool ignore_checks)
{
    return true;
}

// drift_run - runs the drift controller
// should be called at 100hz or more
void ModeDrift::run()
{
    static float braker = 0.0f;
    static float roll_input = 0.0f;

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // Grab inertial velocity
    const Vector3f& vel = inertial_nav.get_velocity_neu_cms();

    // rotate roll, pitch input from north facing to vehicle's perspective
    float roll_vel =  vel.y * ahrs.cos_yaw() - vel.x * ahrs.sin_yaw(); // body roll vel
    float pitch_vel = vel.y * ahrs.sin_yaw() + vel.x * ahrs.cos_yaw(); // body pitch vel

    // gain scheduling for yaw
    float pitch_vel2 = MIN(fabsf(pitch_vel), 2000);
    float target_yaw_rate = target_roll * (1.0f - (pitch_vel2 / 5000.0f)) * g2.acro_y_rate / 45.0;

    roll_vel = constrain_float(roll_vel, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);
    pitch_vel = constrain_float(pitch_vel, -DRIFT_SPEEDLIMIT, DRIFT_SPEEDLIMIT);

    roll_input = roll_input * .96f + (float)channel_yaw->get_control_in() * .04f;

    // convert user input into desired roll velocity
    float roll_vel_error = roll_vel - (roll_input / DRIFT_SPEEDGAIN);

    // roll velocity is feed into roll acceleration to minimize slip
    target_roll = roll_vel_error * -DRIFT_SPEEDGAIN;
    target_roll = constrain_float(target_roll, -4500.0f, 4500.0f);

    // If we let go of sticks, bring us to a stop
    if (is_zero(target_pitch)) {
        // .14/ (.03 * 100) = 4.6 seconds till full braking
        braker += .03f;
        braker = MIN(braker, DRIFT_SPEEDGAIN);
        target_pitch = pitch_vel * braker;
    } else {
        braker = 0.0f;
    }

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle with angle boost
    const float assisted_throttle = get_throttle_assist(vel.z, get_pilot_desired_throttle());
    attitude_control->set_throttle_out(assisted_throttle, true, g.throttle_filt);
}

// get_throttle_assist - return throttle output (range 0 ~ 1) based on pilot input and z-axis velocity
float ModeDrift::get_throttle_assist(float velz, float pilot_throttle_scaled)
{
    // throttle assist - adjusts throttle to slow the vehicle's vertical velocity
    //      Only active when pilot's throttle is between 213 ~ 787
    //      Assistance is strongest when throttle is at mid, drops linearly to no assistance at 213 and 787
    float thr_assist = 0.0f;
    if (pilot_throttle_scaled > DRIFT_THR_MIN && pilot_throttle_scaled < DRIFT_THR_MAX) {
        // calculate throttle assist gain
        thr_assist = 1.2f - ((float)fabsf(pilot_throttle_scaled - 0.5f) / 0.24f);
        thr_assist = constrain_float(thr_assist, 0.0f, 1.0f) * -DRIFT_THR_ASSIST_GAIN * velz;

        // ensure throttle assist never adjusts the throttle by more than 300 pwm
        thr_assist = constrain_float(thr_assist, -DRIFT_THR_ASSIST_MAX, DRIFT_THR_ASSIST_MAX);
    }
    
    return constrain_float(pilot_throttle_scaled + thr_assist, 0.0f, 1.0f);
}
#endif
Mode_flip.cpp
#include "Copter.h"

#if MODE_FLIP_ENABLED == ENABLED

/*
 * Init and run calls for flip flight mode
 *      original implementation in 2010 by Jose Julio
 *      Adapted and updated for AC2 in 2011 by Jason Short
 *
 *      Controls:
 *          RC7_OPTION - RC12_OPTION parameter must be set to "Flip" (AUXSW_FLIP) which is "2"
 *          Pilot switches to Stabilize, Acro or AltHold flight mode and puts ch7/ch8 switch to ON position
 *          Vehicle will Roll right by default but if roll or pitch stick is held slightly left, forward or back it will flip in that direction
 *          Vehicle should complete the roll within 2.5sec and will then return to the original flight mode it was in before flip was triggered
 *          Pilot may manually exit flip by switching off ch7/ch8 or by moving roll stick to >40deg left or right
 *
 *      State machine approach:
 *          FlipState::Start (while copter is leaning <45deg) : roll right at 400deg/sec, increase throttle
 *          FlipState::Roll (while copter is between +45deg ~ -90) : roll right at 400deg/sec, reduce throttle
 *          FlipState::Recover (while copter is between -90deg and original target angle) : use earth frame angle controller to return vehicle to original attitude
 */

#define FLIP_THR_INC        0.20f   // throttle increase during FlipState::Start stage (under 45deg lean angle)
#define FLIP_THR_DEC        0.24f   // throttle decrease during FlipState::Roll stage (between 45deg ~ -90deg roll)
#define FLIP_ROTATION_RATE  40000   // rotation rate request in centi-degrees / sec (i.e. 400 deg/sec)
#define FLIP_TIMEOUT_MS     2500    // timeout after 2.5sec.  Vehicle will switch back to original flight mode
#define FLIP_RECOVERY_ANGLE 500     // consider successful recovery when roll is back within 5 degrees of original

#define FLIP_ROLL_RIGHT      1      // used to set flip_dir
#define FLIP_ROLL_LEFT      -1      // used to set flip_dir

#define FLIP_PITCH_BACK      1      // used to set flip_dir
#define FLIP_PITCH_FORWARD  -1      // used to set flip_dir

// flip_init - initialise flip controller
bool ModeFlip::init(bool ignore_checks)
{
    // only allow flip from some flight modes, for example ACRO, Stabilize, AltHold or FlowHold flight modes
    if (!copter.flightmode->allows_flip()) {
        return false;
    }

    // if in acro or stabilize ensure throttle is above zero
    if (copter.ap.throttle_zero && (copter.flightmode->mode_number() == Mode::Number::ACRO || copter.flightmode->mode_number() == Mode::Number::STABILIZE)) {
        return false;
    }

    // ensure roll input is less than 40deg
    if (abs(channel_roll->get_control_in()) >= 4000) {
        return false;
    }

    // only allow flip when flying
    if (!motors->armed() || copter.ap.land_complete) {
        return false;
    }

    // capture original flight mode so that we can return to it after completion
    orig_control_mode = copter.flightmode->mode_number();

    // initialise state
    _state = FlipState::Start;
    start_time_ms = millis();

    roll_dir = pitch_dir = 0;

    // choose direction based on pilot's roll and pitch sticks
    if (channel_pitch->get_control_in() > 300) {
        pitch_dir = FLIP_PITCH_BACK;
    } else if (channel_pitch->get_control_in() < -300) {
        pitch_dir = FLIP_PITCH_FORWARD;
    } else if (channel_roll->get_control_in() >= 0) {
        roll_dir = FLIP_ROLL_RIGHT;
    } else {
        roll_dir = FLIP_ROLL_LEFT;
    }

    // log start of flip
    AP::logger().Write_Event(LogEvent::FLIP_START);

    // capture current attitude which will be used during the FlipState::Recovery stage
    const float angle_max = copter.aparm.angle_max;
    orig_attitude.x = constrain_float(ahrs.roll_sensor, -angle_max, angle_max);
    orig_attitude.y = constrain_float(ahrs.pitch_sensor, -angle_max, angle_max);
    orig_attitude.z = ahrs.yaw_sensor;

    return true;
}

// run - runs the flip controller
// should be called at 100hz or more
void ModeFlip::run()
{
    // if pilot inputs roll > 40deg or timeout occurs abandon flip
    if (!motors->armed() || (abs(channel_roll->get_control_in()) >= 4000) || (abs(channel_pitch->get_control_in()) >= 4000) || ((millis() - start_time_ms) > FLIP_TIMEOUT_MS)) {
        _state = FlipState::Abandon;
    }

    // get pilot's desired throttle
    float throttle_out = get_pilot_desired_throttle();

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get corrected angle based on direction and axis of rotation
    // we flip the sign of flip_angle to minimize the code repetition
    int32_t flip_angle;

    if (roll_dir != 0) {
        flip_angle = ahrs.roll_sensor * roll_dir;
    } else {
        flip_angle = ahrs.pitch_sensor * pitch_dir;
    }

    // state machine
    switch (_state) {

    case FlipState::Start:
        // under 45 degrees request 400deg/sec roll or pitch
        attitude_control->input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE * roll_dir, FLIP_ROTATION_RATE * pitch_dir, 0.0);

        // increase throttle
        throttle_out += FLIP_THR_INC;

        // beyond 45deg lean angle move to next stage
        if (flip_angle >= 4500) {
            if (roll_dir != 0) {
                // we are rolling
            _state = FlipState::Roll;
            } else {
                // we are pitching
                _state = FlipState::Pitch_A;
        }
        }
        break;

    case FlipState::Roll:
        // between 45deg ~ -90deg request 400deg/sec roll
        attitude_control->input_rate_bf_roll_pitch_yaw(FLIP_ROTATION_RATE * roll_dir, 0.0, 0.0);
        // decrease throttle
        throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

        // beyond -90deg move on to recovery
        if ((flip_angle < 4500) && (flip_angle > -9000)) {
            _state = FlipState::Recover;
        }
        break;

    case FlipState::Pitch_A:
        // between 45deg ~ -90deg request 400deg/sec pitch
        attitude_control->input_rate_bf_roll_pitch_yaw(0.0f, FLIP_ROTATION_RATE * pitch_dir, 0.0);
        // decrease throttle
        throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

        // check roll for inversion
        if ((labs(ahrs.roll_sensor) > 9000) && (flip_angle > 4500)) {
            _state = FlipState::Pitch_B;
        }
        break;

    case FlipState::Pitch_B:
        // between 45deg ~ -90deg request 400deg/sec pitch
        attitude_control->input_rate_bf_roll_pitch_yaw(0.0, FLIP_ROTATION_RATE * pitch_dir, 0.0);
        // decrease throttle
        throttle_out = MAX(throttle_out - FLIP_THR_DEC, 0.0f);

        // check roll for inversion
        if ((labs(ahrs.roll_sensor) < 9000) && (flip_angle > -4500)) {
            _state = FlipState::Recover;
        }
        break;

    case FlipState::Recover: {
        // use originally captured earth-frame angle targets to recover
        attitude_control->input_euler_angle_roll_pitch_yaw(orig_attitude.x, orig_attitude.y, orig_attitude.z, false);

        // increase throttle to gain any lost altitude
        throttle_out += FLIP_THR_INC;

        float recovery_angle;
        if (roll_dir != 0) {
            // we are rolling
            recovery_angle = fabsf(orig_attitude.x - (float)ahrs.roll_sensor);
        } else {
            // we are pitching
            recovery_angle = fabsf(orig_attitude.y - (float)ahrs.pitch_sensor);
        }

        // check for successful recovery
        if (fabsf(recovery_angle) <= FLIP_RECOVERY_ANGLE) {
            // restore original flight mode
            if (!copter.set_mode(orig_control_mode, ModeReason::FLIP_COMPLETE)) {
                // this should never happen but just in case
                copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
            }
            // log successful completion
            AP::logger().Write_Event(LogEvent::FLIP_END);
        }
        break;

    }
    case FlipState::Abandon:
        // restore original flight mode
        if (!copter.set_mode(orig_control_mode, ModeReason::FLIP_COMPLETE)) {
            // this should never happen but just in case
            copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
        }
        // log abandoning flip
        AP::logger().Write_Error(LogErrorSubsystem::FLIP, LogErrorCode::FLIP_ABANDONED);
        break;
    }

    // output pilot's throttle without angle boost
    attitude_control->set_throttle_out(throttle_out, false, g.throttle_filt);
}

#endif
Mode_flowhold.cpp
#include "Copter.h"
#include <utility>

#if !HAL_MINIMIZE_FEATURES && AP_OPTICALFLOW_ENABLED

/*
  implement FLOWHOLD mode, for position hold using optical flow
  without rangefinder
 */

const AP_Param::GroupInfo ModeFlowHold::var_info[] = {
    // @Param: _XY_P
    // @DisplayName: FlowHold P gain
    // @Description: FlowHold (horizontal) P gain.
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _XY_I
    // @DisplayName: FlowHold I gain
    // @Description: FlowHold (horizontal) I gain
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _XY_IMAX
    // @DisplayName: FlowHold Integrator Max
    // @Description: FlowHold (horizontal) integrator maximum
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cdeg
    // @User: Advanced

    // @Param: _XY_FILT_HZ
    // @DisplayName: FlowHold filter on input to control
    // @Description: FlowHold (horizontal) filter on input to control
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(flow_pi_xy, "_XY_",  1, ModeFlowHold, AC_PI_2D),

    // @Param: _FLOW_MAX
    // @DisplayName: FlowHold Flow Rate Max
    // @Description: Controls maximum apparent flow rate in flowhold
    // @Range: 0.1 2.5
    // @User: Standard
    AP_GROUPINFO("_FLOW_MAX", 2, ModeFlowHold, flow_max, 0.6),

    // @Param: _FILT_HZ
    // @DisplayName: FlowHold Filter Frequency
    // @Description: Filter frequency for flow data
    // @Range: 1 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_FILT_HZ", 3, ModeFlowHold, flow_filter_hz, 5),

    // @Param: _QUAL_MIN
    // @DisplayName: FlowHold Flow quality minimum
    // @Description: Minimum flow quality to use flow position hold
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_QUAL_MIN", 4, ModeFlowHold, flow_min_quality, 10),

    // 5 was FLOW_SPEED

    // @Param: _BRAKE_RATE
    // @DisplayName: FlowHold Braking rate
    // @Description: Controls deceleration rate on stick release
    // @Range: 1 30
    // @User: Standard
    // @Units: deg/s
    AP_GROUPINFO("_BRAKE_RATE", 6, ModeFlowHold, brake_rate_dps, 8),

    AP_GROUPEND
};

ModeFlowHold::ModeFlowHold(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#define CONTROL_FLOWHOLD_EARTH_FRAME 0

// flowhold_init - initialise flowhold controller
bool ModeFlowHold::init(bool ignore_checks)
{
    if (!copter.optflow.enabled() || !copter.optflow.healthy()) {
        return false;
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    if (!copter.pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    flow_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), flow_filter_hz.get());

    quality_filtered = 0;
    flow_pi_xy.reset_I();
    limited = false;

    flow_pi_xy.set_dt(1.0/copter.scheduler.get_loop_rate_hz());

    // start with INS height
    last_ins_height = copter.inertial_nav.get_position_z_up_cm() * 0.01;
    height_offset = 0;

    return true;
}

/*
  calculate desired attitude from flow sensor. Called when flow sensor is healthy
 */
void ModeFlowHold::flowhold_flow_to_angle(Vector2f &bf_angles, bool stick_input)
{
    uint32_t now = AP_HAL::millis();

    // get corrected raw flow rate
    Vector2f raw_flow = copter.optflow.flowRate() - copter.optflow.bodyRate();

    // limit sensor flow, this prevents oscillation at low altitudes
    raw_flow.x = constrain_float(raw_flow.x, -flow_max, flow_max);
    raw_flow.y = constrain_float(raw_flow.y, -flow_max, flow_max);

    // filter the flow rate
    Vector2f sensor_flow = flow_filter.apply(raw_flow);

    // scale by height estimate, limiting it to height_min to height_max
    float ins_height = copter.inertial_nav.get_position_z_up_cm() * 0.01;
    float height_estimate = ins_height + height_offset;

    // compensate for height, this converts to (approx) m/s
    sensor_flow *= constrain_float(height_estimate, height_min, height_max);

    // rotate controller input to earth frame
    Vector2f input_ef = copter.ahrs.body_to_earth2D(sensor_flow);

    // run PI controller
    flow_pi_xy.set_input(input_ef);

    // get earth frame controller attitude in centi-degrees
    Vector2f ef_output;

    // get P term
    ef_output = flow_pi_xy.get_p();

    if (stick_input) {
        last_stick_input_ms = now;
        braking = true;
    }
    if (!stick_input && braking) {
        // stop braking if either 3s has passed, or we have slowed below 0.3m/s
        if (now - last_stick_input_ms > 3000 || sensor_flow.length() < 0.3) {
            braking = false;
#if 0
            printf("braking done at %u vel=%f\n", now - last_stick_input_ms,
                   (double)sensor_flow.length());
#endif
        }
    }

    if (!stick_input && !braking) {
        // get I term
        if (limited) {
            // only allow I term to shrink in length
            xy_I = flow_pi_xy.get_i_shrink();
        } else {
            // normal I term operation
            xy_I = flow_pi_xy.get_pi();
        }
    }

    if (!stick_input && braking) {
        // calculate brake angle for each axis separately
        for (uint8_t i=0; i<2; i++) {
            float &velocity = sensor_flow[i];
            float abs_vel_cms = fabsf(velocity)*100;
            const float brake_gain = (15.0f * brake_rate_dps.get() + 95.0f) * 0.01f;
            float lean_angle_cd = brake_gain * abs_vel_cms * (1.0f+500.0f/(abs_vel_cms+60.0f));
            if (velocity < 0) {
                lean_angle_cd = -lean_angle_cd;
            }
            bf_angles[i] = lean_angle_cd;
        }
        ef_output.zero();
    }

    ef_output += xy_I;
    ef_output *= copter.aparm.angle_max;

    // convert to body frame
    bf_angles += copter.ahrs.earth_to_body2D(ef_output);

    // set limited flag to prevent integrator windup
    limited = fabsf(bf_angles.x) > copter.aparm.angle_max || fabsf(bf_angles.y) > copter.aparm.angle_max;

    // constrain to angle limit
    bf_angles.x = constrain_float(bf_angles.x, -copter.aparm.angle_max, copter.aparm.angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -copter.aparm.angle_max, copter.aparm.angle_max);

// @LoggerMessage: FHLD
// @Description: FlowHold mode messages
// @URL: https://ardupilot.org/copter/docs/flowhold-mode.html
// @Field: TimeUS: Time since system startup
// @Field: SFx: Filtered flow rate, X-Axis
// @Field: SFy: Filtered flow rate, Y-Axis
// @Field: Ax: Target lean angle, X-Axis
// @Field: Ay: Target lean angle, Y-Axis
// @Field: Qual: Flow sensor quality. If this value falls below FHLD_QUAL_MIN parameter, FlowHold will act just like AltHold.
// @Field: Ix: Integral part of PI controller, X-Axis
// @Field: Iy: Integral part of PI controller, Y-Axis

    if (log_counter++ % 20 == 0) {
        AP::logger().WriteStreaming("FHLD", "TimeUS,SFx,SFy,Ax,Ay,Qual,Ix,Iy", "Qfffffff",
                                               AP_HAL::micros64(),
                                               (double)sensor_flow.x, (double)sensor_flow.y,
                                               (double)bf_angles.x, (double)bf_angles.y,
                                               (double)quality_filtered,
                                               (double)xy_I.x, (double)xy_I.y);
    }
}

// flowhold_run - runs the flowhold controller
// should be called at 100hz or more
void ModeFlowHold::run()
{
    update_height_estimate();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // check for filter change
    if (!is_equal(flow_filter.get_cutoff_freq(), flow_filter_hz.get())) {
        flow_filter.set_cutoff_frequency(flow_filter_hz.get());
    }

    // get pilot desired climb rate
    float target_climb_rate = copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), copter.g.pilot_speed_up);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(copter.channel_yaw->norm_input_dz());

    // Flow Hold State Machine Determination
    AltHoldModeState flowhold_state = get_alt_hold_state(target_climb_rate);

    if (copter.optflow.healthy()) {
        const float filter_constant = 0.95;
        quality_filtered = filter_constant * quality_filtered + (1-filter_constant) * copter.optflow.quality();
    } else {
        quality_filtered = 0;
    }

    // Flow Hold State Machine
    switch (flowhold_state) {

    case AltHold_MotorStopped:
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        copter.attitude_control->reset_rate_controller_I_terms();
        copter.attitude_control->reset_yaw_target_and_rate();
        copter.pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        flow_pi_xy.reset_I();
        break;

    case AltHold_Takeoff:
        // set motors to full range
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Flying:
        copter.motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // flowhold attitude target calculations
    Vector2f bf_angles;

    // calculate alt-hold angles
    int16_t roll_in = copter.channel_roll->get_control_in();
    int16_t pitch_in = copter.channel_pitch->get_control_in();
    float angle_max = copter.aparm.angle_max;
    get_pilot_desired_lean_angles(bf_angles.x, bf_angles.y, angle_max, attitude_control->get_althold_lean_angle_max_cd());

    if (quality_filtered >= flow_min_quality &&
        AP_HAL::millis() - copter.arm_time_ms > 3000) {
        // don't use for first 3s when we are just taking off
        Vector2f flow_angles;

        flowhold_flow_to_angle(flow_angles, (roll_in != 0) || (pitch_in != 0));
        flow_angles.x = constrain_float(flow_angles.x, -angle_max/2, angle_max/2);
        flow_angles.y = constrain_float(flow_angles.y, -angle_max/2, angle_max/2);
        bf_angles += flow_angles;
    }
    bf_angles.x = constrain_float(bf_angles.x, -angle_max, angle_max);
    bf_angles.y = constrain_float(bf_angles.y, -angle_max, angle_max);

#if AC_AVOID_ENABLED == ENABLED
    // apply avoidance
    copter.avoid.adjust_roll_pitch(bf_angles.x, bf_angles.y, copter.aparm.angle_max);
#endif

    // call attitude controller
    copter.attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(bf_angles.x, bf_angles.y, target_yaw_rate);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

/*
  update height estimate using integrated accelerometer ratio with optical flow
 */
void ModeFlowHold::update_height_estimate(void)
{
    float ins_height = copter.inertial_nav.get_position_z_up_cm() * 0.01;

#if 1
    // assume on ground when disarmed, or if we have only just started spooling the motors up
    if (!hal.util->get_soft_armed() ||
        copter.motors->get_desired_spool_state() != AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED ||
        AP_HAL::millis() - copter.arm_time_ms < 1500) {
        height_offset = -ins_height;
        last_ins_height = ins_height;
        return;
    }
#endif

    // get delta velocity in body frame
    Vector3f delta_vel;
    float delta_vel_dt;
    if (!copter.ins.get_delta_velocity(delta_vel, delta_vel_dt)) {
        return;
    }

    // integrate delta velocity in earth frame
    const Matrix3f &rotMat = copter.ahrs.get_rotation_body_to_ned();
    delta_vel = rotMat * delta_vel;
    delta_velocity_ne.x += delta_vel.x;
    delta_velocity_ne.y += delta_vel.y;

    if (!copter.optflow.healthy()) {
        // can't update height model with no flow sensor
        last_flow_ms = AP_HAL::millis();
        delta_velocity_ne.zero();
        return;
    }

    if (last_flow_ms == 0) {
        // just starting up
        last_flow_ms = copter.optflow.last_update();
        delta_velocity_ne.zero();
        height_offset = 0;
        return;
    }

    if (copter.optflow.last_update() == last_flow_ms) {
        // no new flow data
        return;
    }

    // convert delta velocity back to body frame to match the flow sensor
    Vector2f delta_vel_bf = copter.ahrs.earth_to_body2D(delta_velocity_ne);

    // and convert to an rate equivalent, to be comparable to flow
    Vector2f delta_vel_rate(-delta_vel_bf.y, delta_vel_bf.x);

    // get body flow rate in radians per second
    Vector2f flow_rate_rps = copter.optflow.flowRate() - copter.optflow.bodyRate();

    uint32_t dt_ms = copter.optflow.last_update() - last_flow_ms;
    if (dt_ms > 500) {
        // too long between updates, ignore
        last_flow_ms = copter.optflow.last_update();
        delta_velocity_ne.zero();
        last_flow_rate_rps = flow_rate_rps;
        last_ins_height = ins_height;
        height_offset = 0;
        return;        
    }

    /*
      basic equation is:
      height_m = delta_velocity_mps / delta_flowrate_rps;
     */

    // get delta_flowrate_rps
    Vector2f delta_flowrate = flow_rate_rps - last_flow_rate_rps;
    last_flow_rate_rps = flow_rate_rps;
    last_flow_ms = copter.optflow.last_update();

    /*
      update height estimate
     */
    const float min_velocity_change = 0.04;
    const float min_flow_change = 0.04;
    const float height_delta_max = 0.25;

    /*
      for each axis update the height estimate
     */
    float delta_height = 0;
    uint8_t total_weight = 0;
    float height_estimate = ins_height + height_offset;

    for (uint8_t i=0; i<2; i++) {
        // only use height estimates when we have significant delta-velocity and significant delta-flow
        float abs_flow = fabsf(delta_flowrate[i]);
        if (abs_flow < min_flow_change ||
            fabsf(delta_vel_rate[i]) < min_velocity_change) {
            continue;
        }
        // get instantaneous height estimate
        float height = delta_vel_rate[i] / delta_flowrate[i];
        if (height <= 0) {
            // discard negative heights
            continue;
        }
        delta_height += (height - height_estimate) * abs_flow;
        total_weight += abs_flow;
    }
    if (total_weight > 0) {
        delta_height /= total_weight;
    }

    if (delta_height < 0) {
        // bias towards lower heights, as we'd rather have too low
        // gain than have oscillation. This also compensates a bit for
        // the discard of negative heights above
        delta_height *= 2;
    }

    // don't update height by more than height_delta_max, this is a simple way of rejecting noise
    float new_offset = height_offset + constrain_float(delta_height, -height_delta_max, height_delta_max);

    // apply a simple filter
    height_offset = 0.8 * height_offset + 0.2 * new_offset;

    if (ins_height + height_offset < height_min) {
        // height estimate is never allowed below the minimum
        height_offset = height_min - ins_height;
    }

    // new height estimate for logging
    height_estimate = ins_height + height_offset;

// @LoggerMessage: FHXY
// @Description: Height estimation using optical flow sensor 
// @Field: TimeUS: Time since system startup
// @Field: DFx: Delta flow rate, X-Axis
// @Field: DFy: Delta flow rate, Y-Axis
// @Field: DVx: Integrated delta velocity rate, X-Axis
// @Field: DVy: Integrated delta velocity rate, Y-Axis
// @Field: Hest: Estimated Height
// @Field: DH: Delta Height
// @Field: Hofs: Height offset
// @Field: InsH: Height estimate from inertial navigation library
// @Field: LastInsH: Last used INS height in optical flow sensor height estimation calculations 
// @Field: DTms: Time between optical flow sensor updates. This should be less than 500ms for performing the height estimation calculations

    AP::logger().WriteStreaming("FHXY", "TimeUS,DFx,DFy,DVx,DVy,Hest,DH,Hofs,InsH,LastInsH,DTms", "QfffffffffI",
                                           AP_HAL::micros64(),
                                           (double)delta_flowrate.x,
                                           (double)delta_flowrate.y,
                                           (double)delta_vel_rate.x,
                                           (double)delta_vel_rate.y,
                                           (double)height_estimate,
                                           (double)delta_height,
                                           (double)height_offset,
                                           (double)ins_height,
                                           (double)last_ins_height,
                                           dt_ms);
    gcs().send_named_float("HEST", height_estimate);
    delta_velocity_ne.zero();
    last_ins_height = ins_height;
}

#endif // AP_OPTICALFLOW_ENABLED
Mode_follow.cpp
#include "Copter.h"

#if MODE_FOLLOW_ENABLED == ENABLED

/*
 * mode_follow.cpp - follow another mavlink-enabled vehicle by system id
 *
 * TODO: stick control to move around on sphere
 * TODO: stick control to change sphere diameter
 * TODO: "channel 7 option" to lock onto "pointed at" target
 * TODO: do better in terms of loitering around the moving point; may need a PID?  Maybe use loiter controller somehow?
 * TODO: extrapolate target vehicle position using its velocity and acceleration
 * TODO: ensure AC_AVOID_ENABLED is true because we rely on it velocity limiting functions
 */

// initialise follow mode
bool ModeFollow::init(const bool ignore_checks)
{
    if (!g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }
    // re-use guided mode
    return ModeGuided::init(ignore_checks);
}

// perform cleanup required when leaving follow mode
void ModeFollow::exit()
{
    g2.follow.clear_offsets_if_required();
}

void ModeFollow::run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // re-use guided mode's velocity controller
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode

    // variables to be sent to velocity controller
    Vector3f desired_velocity_neu_cms;
    bool use_yaw = false;
    float yaw_cd = 0.0f;

    Vector3f dist_vec;  // vector to lead vehicle
    Vector3f dist_vec_offs;  // vector to lead vehicle + offset
    Vector3f vel_of_target;  // velocity of lead vehicle
    if (g2.follow.get_target_dist_and_vel_ned(dist_vec, dist_vec_offs, vel_of_target)) {
        // convert dist_vec_offs to cm in NEU
        const Vector3f dist_vec_offs_neu(dist_vec_offs.x * 100.0f, dist_vec_offs.y * 100.0f, -dist_vec_offs.z * 100.0f);

        // calculate desired velocity vector in cm/s in NEU
        const float kp = g2.follow.get_pos_p().kP();
        desired_velocity_neu_cms.x = (vel_of_target.x * 100.0f) + (dist_vec_offs_neu.x * kp);
        desired_velocity_neu_cms.y = (vel_of_target.y * 100.0f) + (dist_vec_offs_neu.y * kp);
        desired_velocity_neu_cms.z = (-vel_of_target.z * 100.0f) + (dist_vec_offs_neu.z * kp);

        // scale desired velocity to stay within horizontal speed limit
        float desired_speed_xy = safe_sqrt(sq(desired_velocity_neu_cms.x) + sq(desired_velocity_neu_cms.y));
        if (!is_zero(desired_speed_xy) && (desired_speed_xy > pos_control->get_max_speed_xy_cms())) {
            const float scalar_xy = pos_control->get_max_speed_xy_cms() / desired_speed_xy;
            desired_velocity_neu_cms.x *= scalar_xy;
            desired_velocity_neu_cms.y *= scalar_xy;
            desired_speed_xy = pos_control->get_max_speed_xy_cms();
        }

        // limit desired velocity to be between maximum climb and descent rates
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -fabsf(pos_control->get_max_speed_down_cms()), pos_control->get_max_speed_up_cms());

        // unit vector towards target position (i.e. vector to lead vehicle + offset)
        Vector3f dir_to_target_neu = dist_vec_offs_neu;
        const float dir_to_target_neu_len = dir_to_target_neu.length();
        if (!is_zero(dir_to_target_neu_len)) {
            dir_to_target_neu /= dir_to_target_neu_len;
        }

        // create horizontal desired velocity vector (required for slow down calculations)
        Vector2f desired_velocity_xy_cms(desired_velocity_neu_cms.x, desired_velocity_neu_cms.y);

        // create horizontal unit vector towards target (required for slow down calculations)
        Vector2f dir_to_target_xy(desired_velocity_xy_cms.x, desired_velocity_xy_cms.y);
        if (!dir_to_target_xy.is_zero()) {
            dir_to_target_xy.normalize();
        }

        // slow down horizontally as we approach target (use 1/2 of maximum deceleration for gentle slow down)
        const float dist_to_target_xy = Vector2f(dist_vec_offs_neu.x, dist_vec_offs_neu.y).length();
        copter.avoid.limit_velocity_2D(pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss() * 0.5f, desired_velocity_xy_cms, dir_to_target_xy, dist_to_target_xy, copter.G_Dt);
        // copy horizontal velocity limits back to 3d vector
        desired_velocity_neu_cms.x = desired_velocity_xy_cms.x;
        desired_velocity_neu_cms.y = desired_velocity_xy_cms.y;

        // limit vertical desired_velocity_neu_cms to slow as we approach target (we use 1/2 of maximum deceleration for gentle slow down)
        const float des_vel_z_max = copter.avoid.get_max_speed(pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss() * 0.5f, fabsf(dist_vec_offs_neu.z), copter.G_Dt);
        desired_velocity_neu_cms.z = constrain_float(desired_velocity_neu_cms.z, -des_vel_z_max, des_vel_z_max);

        // limit the velocity for obstacle/fence avoidance
        copter.avoid.adjust_velocity(desired_velocity_neu_cms, pos_control->get_pos_xy_p().kP().get(), pos_control->get_max_accel_xy_cmss(), pos_control->get_pos_z_p().kP().get(), pos_control->get_max_accel_z_cmss(), G_Dt);

        // calculate vehicle heading
        switch (g2.follow.get_yaw_behave()) {
            case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                if (dist_vec.xy().length_squared() > 1.0) {
                    yaw_cd = get_bearing_cd(Vector2f{}, dist_vec.xy());
                    use_yaw = true;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                float target_hdg = 0.0f;
                if (g2.follow.get_target_heading_deg(target_hdg)) {
                    yaw_cd = target_hdg * 100.0f;
                    use_yaw = true;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                if (desired_velocity_neu_cms.xy().length_squared() > (100.0 * 100.0)) {
                    yaw_cd = get_bearing_cd(Vector2f{}, desired_velocity_neu_cms.xy());
                    use_yaw = true;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_NONE:
            default:
                // do nothing
               break;

        }
    }

    // log output at 10hz
    uint32_t now = AP_HAL::millis();
    bool log_request = false;
    if ((now - last_log_ms >= 100) || (last_log_ms == 0)) {
        log_request = true;
        last_log_ms = now;
    }
    // re-use guided mode's velocity controller (takes NEU)
    ModeGuided::set_velocity(desired_velocity_neu_cms, use_yaw, yaw_cd, false, 0.0f, false, log_request);

    ModeGuided::run();
}

uint32_t ModeFollow::wp_distance() const
{
    return g2.follow.get_distance_to_target() * 100;
}

int32_t ModeFollow::wp_bearing() const
{
    return g2.follow.get_bearing_to_target() * 100;
}

/*
  get target position for mavlink reporting
 */
bool ModeFollow::get_wp(Location &loc) const
{
    float dist = g2.follow.get_distance_to_target();
    float bearing = g2.follow.get_bearing_to_target();
    loc = copter.current_loc;
    loc.offset_bearing(bearing, dist);
    return true;
}

#endif // MODE_FOLLOW_ENABLED == ENABLED
Mode_guided.cpp
#include "Copter.h"

#if MODE_GUIDED_ENABLED == ENABLED

/*
 * Init and run calls for guided flight mode
 */

static Vector3p guided_pos_target_cm;       // position target (used by posvel controller only)
bool guided_pos_terrain_alt;                // true if guided_pos_target_cm.z is an alt above terrain
static Vector3f guided_vel_target_cms;      // velocity target (used by pos_vel_accel controller and vel_accel controller)
static Vector3f guided_accel_target_cmss;   // acceleration target (used by pos_vel_accel controller vel_accel controller and accel controller)
static uint32_t update_time_ms;             // system time of last target update to pos_vel_accel, vel_accel or accel controller

struct {
    uint32_t update_time_ms;
    Quaternion attitude_quat;
    Vector3f ang_vel;
    float yaw_rate_cds;
    float climb_rate_cms;   // climb rate in cms.  Used if use_thrust is false
    float thrust;           // thrust from -1 to 1.  Used if use_thrust is true
    bool use_yaw_rate;
    bool use_thrust;
} static guided_angle_state;

struct Guided_Limit {
    uint32_t timeout_ms;  // timeout (in seconds) from the time that guided is invoked
    float alt_min_cm;   // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;   // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm; // horizontal position limit in cm from where guided mode was initiated (0 = no limit)
    uint32_t start_time;// system time in milliseconds that control was handed to the external computer
    Vector3f start_pos; // start position as a distance from home in cm.  used for checking horiz_max limit
} guided_limit;

// init - initialise guided controller
bool ModeGuided::init(bool ignore_checks)
{
    // start in velaccel control mode
    velaccel_control_start();
    guided_vel_target_cms.zero();
    guided_accel_target_cmss.zero();
    send_notification = false;

    // clear pause state when entering guided mode
    _paused = false;

    return true;
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeGuided::run()
{
    // run pause control if the vehicle is paused
    if (_paused) {
        pause_control_run();
        return;
    }

    // call the correct auto controller
    switch (guided_mode) {

    case SubMode::TakeOff:
        // run takeoff controller
        takeoff_run();
        break;

    case SubMode::WP:
        // run waypoint controller
        wp_control_run();
        if (send_notification && wp_nav->reached_wp_destination()) {
            send_notification = false;
            gcs().send_mission_item_reached_message(0);
        }
        break;

    case SubMode::Pos:
        // run position controller
        pos_control_run();
        break;

    case SubMode::Accel:
        accel_control_run();
        break;

    case SubMode::VelAccel:
        velaccel_control_run();
        break;

    case SubMode::PosVelAccel:
        posvelaccel_control_run();
        break;

    case SubMode::Angle:
        angle_control_run();
        break;
    }
 }

bool ModeGuided::allows_arming(AP_Arming::Method method) const
{
    // always allow arming from the ground station
    if (method == AP_Arming::Method::MAVLINK) {
        return true;
    }

    // optionally allow arming from the transmitter
    return (copter.g2.guided_options & (uint32_t)Options::AllowArmingFromTX) != 0;
};

// initialises position controller to implement take-off
// takeoff_alt_cm is interpreted as alt-above-home (in cm) or alt-above-terrain if a rangefinder is available
bool ModeGuided::do_user_takeoff_start(float takeoff_alt_cm)
{
    // calculate target altitude and frame (either alt-above-ekf-origin or alt-above-terrain)
    int32_t alt_target_cm;
    bool alt_target_terrain = false;
    if (wp_nav->rangefinder_used_and_healthy() &&
        wp_nav->get_terrain_source() == AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER &&
        takeoff_alt_cm < copter.rangefinder.max_distance_cm_orient(ROTATION_PITCH_270)) {
        // can't takeoff downwards
        if (takeoff_alt_cm <= copter.rangefinder_state.alt_cm) {
            return false;
        }
        // provide target altitude as alt-above-terrain
        alt_target_cm = takeoff_alt_cm;
        alt_target_terrain = true;
    } else {
        // interpret altitude as alt-above-home
        Location target_loc = copter.current_loc;
        target_loc.set_alt_cm(takeoff_alt_cm, Location::AltFrame::ABOVE_HOME);

        // provide target altitude as alt-above-ekf-origin
        if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_target_cm)) {
            // this should never happen but we reject the command just in case
            return false;
        }
    }

    guided_mode = SubMode::TakeOff;

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // initialise alt for WP_NAVALT_MIN and set completion alt
    auto_takeoff_start(alt_target_cm, alt_target_terrain);

    // record takeoff has not completed
    takeoff_complete = false;

    return true;
}

// initialise guided mode's waypoint navigation controller
void ModeGuided::wp_control_start()
{
    // set to position control mode
    guided_mode = SubMode::WP;

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);
    if (!wp_nav->set_wp_destination(stopping_point, false)) {
        // this should never happen because terrain data is not used
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

// run guided mode's waypoint navigation controller
void ModeGuided::wp_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
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
        attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_thrust_vector_heading(wp_nav->get_thrust_vector(), auto_yaw.yaw());
    }
}

// initialise position controller
void ModeGuided::pva_control_start()
{
    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise velocity controller
    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    // initialise yaw
    auto_yaw.set_mode_to_default(false);

    // initialise terrain alt
    guided_pos_terrain_alt = false;
}

// initialise guided mode's position controller
void ModeGuided::pos_control_start()
{
    // set to position control mode
    guided_mode = SubMode::Pos;

    // initialise position controller
    pva_control_start();
}

// initialise guided mode's velocity controller
void ModeGuided::accel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = SubMode::Accel;

    // initialise position controller
    pva_control_start();
}

// initialise guided mode's velocity and acceleration controller
void ModeGuided::velaccel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = SubMode::VelAccel;

    // initialise position controller
    pva_control_start();
}

// initialise guided mode's position, velocity and acceleration controller
void ModeGuided::posvelaccel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = SubMode::PosVelAccel;

    // initialise position controller
    pva_control_start();
}

bool ModeGuided::is_taking_off() const
{
    return guided_mode == SubMode::TakeOff && !takeoff_complete;
}

// initialise guided mode's angle controller
void ModeGuided::angle_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = SubMode::Angle;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise targets
    guided_angle_state.update_time_ms = millis();
    guided_angle_state.attitude_quat.initialise();
    guided_angle_state.ang_vel.zero();
    guided_angle_state.climb_rate_cms = 0.0f;
    guided_angle_state.yaw_rate_cds = 0.0f;
    guided_angle_state.use_yaw_rate = false;

    // pilot always controls yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

// set_destination - sets guided mode's target destination
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool ModeGuided::set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool terrain_alt)
{
#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination, terrain_alt ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // if configured to use wpnav for position control
    if (use_wpnav_for_position_control()) {
        // ensure we are in position control mode
        if (guided_mode != SubMode::WP) {
            wp_control_start();
        }

        // set yaw state
        set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

        // no need to check return status because terrain data is not used
        wp_nav->set_wp_destination(destination, terrain_alt);

        // log target
        copter.Log_Write_Guided_Position_Target(guided_mode, destination, terrain_alt, Vector3f(), Vector3f());
        send_notification = true;
        return true;
    }

    // if configured to use position controller for position control
    // ensure we are in position control mode
    if (guided_mode != SubMode::Pos) {
        pos_control_start();
    }

    // initialise terrain following if needed
    if (terrain_alt) {
        // get current alt above terrain
        float origin_terr_offset;
        if (!wp_nav->get_terrain_offset(origin_terr_offset)) {
            // if we don't have terrain altitude then stop
            init(true);
            return false;
        }
        // convert origin to alt-above-terrain if necessary
        if (!guided_pos_terrain_alt) {
            // new destination is alt-above-terrain, previous destination was alt-above-ekf-origin
            pos_control->set_pos_offset_z_cm(origin_terr_offset);
        }
    } else {
        pos_control->set_pos_offset_z_cm(0.0);
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // set position target and zero velocity and acceleration
    guided_pos_target_cm = destination.topostype();
    guided_pos_terrain_alt = terrain_alt;
    guided_vel_target_cms.zero();
    guided_accel_target_cmss.zero();
    update_time_ms = millis();

    // log target
    copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_cm.tofloat(), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);

    send_notification = true;

    return true;
}

bool ModeGuided::get_wp(Location& destination) const
{
    switch (guided_mode) {
    case SubMode::WP:
        return wp_nav->get_oa_wp_destination(destination);
    case SubMode::Pos:
        destination = Location(guided_pos_target_cm.tofloat(), guided_pos_terrain_alt ? Location::AltFrame::ABOVE_TERRAIN : Location::AltFrame::ABOVE_ORIGIN);
        return true;
    default:
        return false;
    }

    // should never get here but just in case
    return false;
}

// sets guided mode's target from a Location object
// returns false if destination could not be set (probably caused by missing terrain data)
// or if the fence is enabled and guided waypoint is outside the fence
bool ModeGuided::set_destination(const Location& dest_loc, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
#if AC_FENCE == ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // if using wpnav for position control
    if (use_wpnav_for_position_control()) {
        if (guided_mode != SubMode::WP) {
            wp_control_start();
        }

        if (!wp_nav->set_wp_destination_loc(dest_loc)) {
            // failure to set destination can only be because of missing terrain data
            AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
            // failure is propagated to GCS with NAK
            return false;
        }

        // set yaw state
        set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

        // log target
        copter.Log_Write_Guided_Position_Target(guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt), (dest_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN), Vector3f(), Vector3f());
        send_notification = true;
        return true;
    }

    // if configured to use position controller for position control
    // ensure we are in position control mode
    if (guided_mode != SubMode::Pos) {
        pos_control_start();
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // set position target and zero velocity and acceleration
    Vector3f pos_target_f;
    bool terrain_alt;
    if (!wp_nav->get_vector_NEU(dest_loc, pos_target_f, terrain_alt)) {
        return false;
    }

    // initialise terrain following if needed
    if (terrain_alt) {
        // get current alt above terrain
        float origin_terr_offset;
        if (!wp_nav->get_terrain_offset(origin_terr_offset)) {
            // if we don't have terrain altitude then stop
            init(true);
            return false;
        }
        // convert origin to alt-above-terrain if necessary
        if (!guided_pos_terrain_alt) {
            // new destination is alt-above-terrain, previous destination was alt-above-ekf-origin
            pos_control->set_pos_offset_z_cm(origin_terr_offset);
        }
    } else {
        pos_control->set_pos_offset_z_cm(0.0);
    }

    guided_pos_target_cm = pos_target_f.topostype();
    guided_pos_terrain_alt = terrain_alt;
    guided_vel_target_cms.zero();
    guided_accel_target_cmss.zero();
    update_time_ms = millis();

    // log target
    copter.Log_Write_Guided_Position_Target(guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);

    send_notification = true;

    return true;
}

// set_velaccel - sets guided mode's target velocity and acceleration
void ModeGuided::set_accel(const Vector3f& acceleration, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool log_request)
{
    // check we are in velocity control mode
    if (guided_mode != SubMode::Accel) {
        accel_control_start();
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // set velocity and acceleration targets and zero position
    guided_pos_target_cm.zero();
    guided_pos_terrain_alt = false;
    guided_vel_target_cms.zero();
    guided_accel_target_cmss = acceleration;
    update_time_ms = millis();

    // log target
    if (log_request) {
        copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_cm.tofloat(), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);
    }
}

// set_velocity - sets guided mode's target velocity
void ModeGuided::set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool log_request)
{
    set_velaccel(velocity, Vector3f(), use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw, log_request);
}

// set_velaccel - sets guided mode's target velocity and acceleration
void ModeGuided::set_velaccel(const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw, bool log_request)
{
    // check we are in velocity control mode
    if (guided_mode != SubMode::VelAccel) {
        velaccel_control_start();
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    // set velocity and acceleration targets and zero position
    guided_pos_target_cm.zero();
    guided_pos_terrain_alt = false;
    guided_vel_target_cms = velocity;
    guided_accel_target_cmss = acceleration;
    update_time_ms = millis();

    // log target
    if (log_request) {
        copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_cm.tofloat(), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);
    }
}

// set_destination_posvel - set guided mode position and velocity target
bool ModeGuided::set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
    return set_destination_posvelaccel(destination, velocity, Vector3f(), use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);
}

// set_destination_posvelaccel - set guided mode position, velocity and acceleration target
bool ModeGuided::set_destination_posvelaccel(const Vector3f& destination, const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw)
{
#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
    if (!copter.fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // check we are in velocity control mode
    if (guided_mode != SubMode::PosVelAccel) {
        posvelaccel_control_start();
    }

    // set yaw state
    set_yaw_state(use_yaw, yaw_cd, use_yaw_rate, yaw_rate_cds, relative_yaw);

    update_time_ms = millis();
    guided_pos_target_cm = destination.topostype();
    guided_pos_terrain_alt = false;
    guided_vel_target_cms = velocity;
    guided_accel_target_cmss = acceleration;

    // log target
    copter.Log_Write_Guided_Position_Target(guided_mode, guided_pos_target_cm.tofloat(), guided_pos_terrain_alt, guided_vel_target_cms, guided_accel_target_cmss);
    return true;
}

// returns true if GUIDED_OPTIONS param suggests SET_ATTITUDE_TARGET's "thrust" field should be interpreted as thrust instead of climb rate
bool ModeGuided::set_attitude_target_provides_thrust() const
{
    return ((copter.g2.guided_options.get() & uint32_t(Options::SetAttitudeTarget_ThrustAsThrust)) != 0);
}

// returns true if GUIDED_OPTIONS param specifies position should be controlled (when velocity and/or acceleration control is active)
bool ModeGuided::stabilizing_pos_xy() const
{
    return !((copter.g2.guided_options.get() & uint32_t(Options::DoNotStabilizePositionXY)) != 0);
}

// returns true if GUIDED_OPTIONS param specifies velocity should  be controlled (when acceleration control is active)
bool ModeGuided::stabilizing_vel_xy() const
{
    return !((copter.g2.guided_options.get() & uint32_t(Options::DoNotStabilizeVelocityXY)) != 0);
}

// returns true if GUIDED_OPTIONS param specifies waypoint navigation should be used for position control (allow path planning to be used but updates must be slower)
bool ModeGuided::use_wpnav_for_position_control() const
{
    return ((copter.g2.guided_options.get() & uint32_t(Options::WPNavUsedForPosControl)) != 0);
}

// Sets guided's angular target submode: Using a rotation quaternion, angular velocity, and climbrate or thrust (depends on user option)
// attitude_quat: IF zero: ang_vel (angular velocity) must be provided even if all zeroes
//                IF non-zero: attitude_control is performed using both the attitude quaternion and angular velocity
// ang_vel: angular velocity (rad/s)
// climb_rate_cms_or_thrust: represents either the climb_rate (cm/s) or thrust scaled from [0, 1], unitless
// use_thrust: IF true: climb_rate_cms_or_thrust represents thrust
//             IF false: climb_rate_cms_or_thrust represents climb_rate (cm/s)
void ModeGuided::set_angle(const Quaternion &attitude_quat, const Vector3f &ang_vel, float climb_rate_cms_or_thrust, bool use_thrust)
{
    // check we are in velocity control mode
    if (guided_mode != SubMode::Angle) {
        angle_control_start();
    }

    guided_angle_state.attitude_quat = attitude_quat;
    guided_angle_state.ang_vel = ang_vel;

    guided_angle_state.use_thrust = use_thrust;
    if (use_thrust) {
        guided_angle_state.thrust = climb_rate_cms_or_thrust;
        guided_angle_state.climb_rate_cms = 0.0f;
    } else {
        guided_angle_state.thrust = 0.0f;
        guided_angle_state.climb_rate_cms = climb_rate_cms_or_thrust;
    }

    guided_angle_state.update_time_ms = millis();

    // convert quaternion to euler angles
    float roll_rad, pitch_rad, yaw_rad;
    attitude_quat.to_euler(roll_rad, pitch_rad, yaw_rad);

    // log target
    copter.Log_Write_Guided_Attitude_Target(guided_mode, roll_rad, pitch_rad, yaw_rad, ang_vel, guided_angle_state.thrust, guided_angle_state.climb_rate_cms * 0.01);
}

// takeoff_run - takeoff in guided mode
//      called by guided_run at 100hz or more
void ModeGuided::takeoff_run()
{
    auto_takeoff_run();
    if (auto_takeoff_complete && !takeoff_complete) {
        takeoff_complete = true;
#if LANDING_GEAR_ENABLED == ENABLED
        // optionally retract landing gear
        copter.landinggear.retract_after_takeoff();
#endif
    }
}

// pos_control_run - runs the guided position controller
// called from guided_run
void ModeGuided::pos_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;

    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // calculate terrain adjustments
    float terr_offset = 0.0f;
    if (guided_pos_terrain_alt && !wp_nav->get_terrain_offset(terr_offset)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // send position and velocity targets to position controller
    guided_accel_target_cmss.zero();
    guided_vel_target_cms.zero();

    // stop rotating if no updates received within timeout_ms
    if (millis() - update_time_ms > get_timeout_ms()) {
        if ((auto_yaw.mode() == AUTO_YAW_RATE) || (auto_yaw.mode() == AUTO_YAW_ANGLE_RATE)) {
            auto_yaw.set_rate(0.0f);
        }
    }

    float pos_offset_z_buffer = 0.0; // Vertical buffer size in m
    if (guided_pos_terrain_alt) {
        pos_offset_z_buffer = MIN(copter.wp_nav->get_terrain_margin() * 100.0, 0.5 * fabsF(guided_pos_target_cm.z));
    }
    pos_control->input_pos_xyz(guided_pos_target_cm, terr_offset, pos_offset_z_buffer);

    // run position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from position controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from position controller, yaw rate from mavlink command or mission item
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), auto_yaw.rate_cds());
    } else {
        // roll & pitch from position controller, yaw heading from GCS or auto_heading()
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.yaw(), auto_yaw.rate_cds());
    }
}

// velaccel_control_run - runs the guided velocity controller
// called from guided_run
void ModeGuided::accel_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        guided_vel_target_cms.zero();
        guided_accel_target_cmss.zero();
        if ((auto_yaw.mode() == AUTO_YAW_RATE) || (auto_yaw.mode() == AUTO_YAW_ANGLE_RATE)) {
            auto_yaw.set_rate(0.0f);
        }
        pos_control->input_vel_accel_xy(guided_vel_target_cms.xy(), guided_accel_target_cmss.xy(), false);
        pos_control->input_vel_accel_z(guided_vel_target_cms.z, guided_accel_target_cmss.z, false, false);
    } else {
        // update position controller with new target
        pos_control->input_accel_xy(guided_accel_target_cmss);
        if (!stabilizing_vel_xy()) {
            // set position and velocity errors to zero
            pos_control->stop_vel_xy_stabilisation();
        } else if (!stabilizing_pos_xy()) {
            // set position errors to zero
            pos_control->stop_pos_xy_stabilisation();
        }
        pos_control->input_accel_z(guided_accel_target_cmss.z);
    }

    // call velocity controller which includes z axis controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from position controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from position controller, yaw rate from mavlink command or mission item
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), auto_yaw.rate_cds());
    } else {
        // roll & pitch from position controller, yaw heading from GCS or auto_heading()
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.yaw(), auto_yaw.rate_cds());
    }
}

// velaccel_control_run - runs the guided velocity and acceleration controller
// called from guided_run
void ModeGuided::velaccel_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        guided_vel_target_cms.zero();
        guided_accel_target_cmss.zero();
        if ((auto_yaw.mode() == AUTO_YAW_RATE) || (auto_yaw.mode() == AUTO_YAW_ANGLE_RATE)) {
            auto_yaw.set_rate(0.0f);
        }
    }

    bool do_avoid = false;
#if AC_AVOID_ENABLED
    // limit the velocity for obstacle/fence avoidance
    copter.avoid.adjust_velocity(guided_vel_target_cms, pos_control->get_pos_xy_p().kP(), pos_control->get_max_accel_xy_cmss(), pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), G_Dt);
    do_avoid = copter.avoid.limits_active();
#endif

    // update position controller with new target

    if (!stabilizing_vel_xy() && !do_avoid) {
        // set the current commanded xy vel to the desired vel
        guided_vel_target_cms.x = pos_control->get_vel_desired_cms().x;
        guided_vel_target_cms.y = pos_control->get_vel_desired_cms().y;
    }
    pos_control->input_vel_accel_xy(guided_vel_target_cms.xy(), guided_accel_target_cmss.xy(), false);
    if (!stabilizing_vel_xy() && !do_avoid) {
        // set position and velocity errors to zero
        pos_control->stop_vel_xy_stabilisation();
    } else if (!stabilizing_pos_xy() && !do_avoid) {
        // set position errors to zero
        pos_control->stop_pos_xy_stabilisation();
    }
    pos_control->input_vel_accel_z(guided_vel_target_cms.z, guided_accel_target_cmss.z, false, false);

    // call velocity controller which includes z axis controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from position controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from position controller, yaw rate from mavlink command or mission item
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), auto_yaw.rate_cds());
    } else {
        // roll & pitch from position controller, yaw heading from GCS or auto_heading()
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.yaw(), auto_yaw.rate_cds());
    }
}

// pause_control_run - runs the guided mode pause controller
// called from guided_run
void ModeGuided::pause_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set the horizontal velocity and acceleration targets to zero
    Vector2f vel_xy, accel_xy;
    pos_control->input_vel_accel_xy(vel_xy, accel_xy, false);

    // set the vertical velocity and acceleration targets to zero
    float vel_z = 0.0;
    pos_control->input_vel_accel_z(vel_z, 0.0, false, false);

    // call velocity controller which includes z axis controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0);
}

// posvelaccel_control_run - runs the guided position, velocity and acceleration controller
// called from guided_run
void ModeGuided::posvelaccel_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;

    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // set velocity to zero and stop rotating if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - update_time_ms > get_timeout_ms()) {
        guided_vel_target_cms.zero();
        guided_accel_target_cmss.zero();
        if ((auto_yaw.mode() == AUTO_YAW_RATE) || (auto_yaw.mode() == AUTO_YAW_ANGLE_RATE)) {
            auto_yaw.set_rate(0.0f);
        }
    }

    // send position and velocity targets to position controller
    if (!stabilizing_vel_xy()) {
        // set the current commanded xy pos to the target pos and xy vel to the desired vel
        guided_pos_target_cm.x = pos_control->get_pos_target_cm().x;
        guided_pos_target_cm.y = pos_control->get_pos_target_cm().y;
        guided_vel_target_cms.x = pos_control->get_vel_desired_cms().x;
        guided_vel_target_cms.y = pos_control->get_vel_desired_cms().y;
    } else if (!stabilizing_pos_xy()) {
        // set the current commanded xy pos to the target pos
        guided_pos_target_cm.x = pos_control->get_pos_target_cm().x;
        guided_pos_target_cm.y = pos_control->get_pos_target_cm().y;
    }
    pos_control->input_pos_vel_accel_xy(guided_pos_target_cm.xy(), guided_vel_target_cms.xy(), guided_accel_target_cmss.xy(), false);
    if (!stabilizing_vel_xy()) {
        // set position and velocity errors to zero
        pos_control->stop_vel_xy_stabilisation();
    } else if (!stabilizing_pos_xy()) {
        // set position errors to zero
        pos_control->stop_pos_xy_stabilisation();
    }

    // guided_pos_target z-axis should never be a terrain altitude
    if (guided_pos_terrain_alt) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }

    float pz = guided_pos_target_cm.z;
    pos_control->input_pos_vel_accel_z(pz, guided_vel_target_cms.z, guided_accel_target_cmss.z, false);
    guided_pos_target_cm.z = pz;

    // run position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from position controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from position controller, yaw rate from mavlink command or mission item
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), auto_yaw.rate_cds());
    } else {
        // roll & pitch from position controller, yaw heading from GCS or auto_heading()
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.yaw(), auto_yaw.rate_cds());
    }
}

// angle_control_run - runs the guided angle controller
// called from guided_run
void ModeGuided::angle_control_run()
{
    float climb_rate_cms = 0.0f;
    if (!guided_angle_state.use_thrust) {
        // constrain climb rate
        climb_rate_cms = constrain_float(guided_angle_state.climb_rate_cms, -wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up());

        // get avoidance adjusted climb rate
        climb_rate_cms = get_avoidance_adjusted_climbrate(climb_rate_cms);
    }

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - guided_angle_state.update_time_ms > get_timeout_ms()) {
        guided_angle_state.attitude_quat.initialise();
        guided_angle_state.ang_vel.zero();
        climb_rate_cms = 0.0f;
        if (guided_angle_state.use_thrust) {
            // initialise vertical velocity controller
            pos_control->init_z_controller();
            guided_angle_state.use_thrust = false;
        }
    }

    // interpret positive climb rate or thrust as triggering take-off
    const bool positive_thrust_or_climbrate = is_positive(guided_angle_state.use_thrust ? guided_angle_state.thrust : climb_rate_cms);
    if (motors->armed() && positive_thrust_or_climbrate) {
        copter.set_auto_armed(true);
    }

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || !copter.ap.auto_armed || (copter.ap.land_complete && !positive_thrust_or_climbrate)) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // TODO: use get_alt_hold_state
    // landed with positive desired climb rate, takeoff
    if (copter.ap.land_complete && (guided_angle_state.climb_rate_cms > 0.0f)) {
        zero_throttle_and_relax_ac();
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
            set_land_complete(false);
            set_throttle_takeoff();
        }
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // call attitude controller
    if (guided_angle_state.attitude_quat.is_zero()) {
        attitude_control->input_rate_bf_roll_pitch_yaw(ToDeg(guided_angle_state.ang_vel.x) * 100.0f, ToDeg(guided_angle_state.ang_vel.y) * 100.0f, ToDeg(guided_angle_state.ang_vel.z) * 100.0f);
    } else {
        attitude_control->input_quaternion(guided_angle_state.attitude_quat, guided_angle_state.ang_vel);
    }

    // call position controller
    if (guided_angle_state.use_thrust) {
        attitude_control->set_throttle_out(guided_angle_state.thrust, true, copter.g.throttle_filt);
    } else {
        pos_control->set_pos_target_z_from_climb_rate_cm(climb_rate_cms);
        pos_control->update_z_controller();
    }
}

// helper function to set yaw state and targets
void ModeGuided::set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (use_yaw && relative_angle) {
        auto_yaw.set_fixed_yaw(yaw_cd * 0.01f, 0.0f, 0, relative_angle);
    } else if (use_yaw && use_yaw_rate) {
        auto_yaw.set_yaw_angle_rate(yaw_cd * 0.01f, yaw_rate_cds * 0.01f);
    } else if (use_yaw && !use_yaw_rate) {
        auto_yaw.set_yaw_angle_rate(yaw_cd * 0.01f, 0.0f);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate(yaw_rate_cds);
    } else {
        auto_yaw.set_mode_to_default(false);
    }
}

// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool ModeGuided::use_pilot_yaw(void) const
{
    return (copter.g2.guided_options.get() & uint32_t(Options::IgnorePilotYaw)) == 0;
}

// Guided Limit code

// limit_clear - clear/turn off guided limits
void ModeGuided::limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}

// limit_set - set guided timeout and movement limits
void ModeGuided::limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}

// limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_guided_start function
void ModeGuided::limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time = AP_HAL::millis();

    // initialise start position from current position
    guided_limit.start_pos = inertial_nav.get_position_neu_cm();
}

// limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_GUIDED_ENABLE mission command
bool ModeGuided::limit_check()
{
    // check if we have passed the timeout
    if ((guided_limit.timeout_ms > 0) && (millis() - guided_limit.start_time >= guided_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position_neu_cm();

    // check if we have gone below min alt
    if (!is_zero(guided_limit.alt_min_cm) && (curr_pos.z < guided_limit.alt_min_cm)) {
        return true;
    }

    // check if we have gone above max alt
    if (!is_zero(guided_limit.alt_max_cm) && (curr_pos.z > guided_limit.alt_max_cm)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (guided_limit.horiz_max_cm > 0.0f) {
        const float horiz_move = get_horizontal_distance_cm(guided_limit.start_pos.xy(), curr_pos.xy());
        if (horiz_move > guided_limit.horiz_max_cm) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}

const Vector3p &ModeGuided::get_target_pos() const
{
    return guided_pos_target_cm;
}

const Vector3f& ModeGuided::get_target_vel() const
{
    return guided_vel_target_cms;
}

const Vector3f& ModeGuided::get_target_accel() const
{
    return guided_accel_target_cmss;
}

uint32_t ModeGuided::wp_distance() const
{
    switch(guided_mode) {
    case SubMode::WP:
        return wp_nav->get_wp_distance_to_destination();
    case SubMode::Pos:
        return get_horizontal_distance_cm(inertial_nav.get_position_xy_cm(), guided_pos_target_cm.tofloat().xy());
    case SubMode::PosVelAccel:
        return pos_control->get_pos_error_xy_cm();
        break;
    default:
        return 0;
    }
}

int32_t ModeGuided::wp_bearing() const
{
    switch(guided_mode) {
    case SubMode::WP:
        return wp_nav->get_wp_bearing_to_destination();
    case SubMode::Pos:
        return get_bearing_cd(inertial_nav.get_position_xy_cm(), guided_pos_target_cm.tofloat().xy());
    case SubMode::PosVelAccel:
        return pos_control->get_bearing_to_target_cd();
        break;
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::Angle:
        // these do not have bearings
        return 0;
    }
    // compiler guarantees we don't get here
    return 0.0;
}

float ModeGuided::crosstrack_error() const
{
    switch (guided_mode) {
    case SubMode::WP:
        return wp_nav->crosstrack_error();
    case SubMode::Pos:
    case SubMode::TakeOff:
    case SubMode::Accel:
    case SubMode::VelAccel:
    case SubMode::PosVelAccel:
        return pos_control->crosstrack_error();
    case SubMode::Angle:
        // no track to have a crosstrack to
        return 0;
    }
    // compiler guarantees we don't get here
    return 0;
}

// return guided mode timeout in milliseconds. Only used for velocity, acceleration, angle control, and angular rates
uint32_t ModeGuided::get_timeout_ms() const
{
    return MAX(copter.g2.guided_timeout, 0.1) * 1000;
}

// pause guide mode
bool ModeGuided::pause()
{
    _paused = true;
    return true;
}

// resume guided mode
bool ModeGuided::resume()
{
    _paused = false;
    return true;
}

#endif
Mode_guided_nogps.cpp
#include "Copter.h"

#if MODE_GUIDED_NOGPS_ENABLED == ENABLED

/*
 * Init and run calls for guided_nogps flight mode
 */

// initialise guided_nogps controller
bool ModeGuidedNoGPS::init(bool ignore_checks)
{
    // start in angle control mode
    ModeGuided::angle_control_start();
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void ModeGuidedNoGPS::run()
{
    // run angle controller
    ModeGuided::angle_control_run();
}

#endif
Mode_land.cpp
#include "Copter.h"

// land_init - initialise land controller
bool ModeLand::init(bool ignore_checks)
{
    // check if we have GPS and decide which LAND we're going to do
    control_position = copter.position_ok();

    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialise the horizontal position controller
    if (control_position && !pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    land_start_time = millis();
    land_pause = false;

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.land_repo_active = false;

    // this will be set true if prec land is later active
    copter.ap.prec_land_active = false;

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

#if LANDING_GEAR_ENABLED == ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif

#if AC_FENCE == ENABLED
    // disable the fence on landing
    copter.fence.auto_disable_fence_for_landing();
#endif

#if PRECISION_LANDING == ENABLED
    // initialise precland state machine
    copter.precland_statemachine.init();
#endif

    return true;
}

// land_run - runs the land controller
// should be called at 100hz or more
void ModeLand::run()
{
    if (control_position) {
        gps_run();
    } else {
        nogps_run();
    }
}

// land_gps_run - runs the land controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
void ModeLand::gps_run()
{
    // disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // Land State Machine Determination
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
    } else {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // pause before beginning land descent
        if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
            land_pause = false;
        }

        // run normal landing or precision landing (if enabled)
        land_run_normal_or_precland(land_pause);
    }
}

// land_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void ModeLand::nogps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;

    // process pilot inputs
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            AP::logger().Write_Event(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // disarm when the landing detector says we've landed
    if (copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // Land State Machine Determination
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
    } else {
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // pause before beginning land descent
        if (land_pause && millis()-land_start_time >= LAND_WITH_DELAY_MS) {
            land_pause = false;
        }

        land_run_vertical_control(land_pause);
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
}

// do_not_use_GPS - forces land-mode to not use the GPS but instead rely on pilot input for roll and pitch
//  called during GPS failsafe to ensure that if we were already in LAND mode that we do not use the GPS
//  has no effect if we are not already in LAND mode
void ModeLand::do_not_use_GPS()
{
    control_position = false;
}

// set_mode_land_with_pause - sets mode to LAND and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_land_with_pause(ModeReason reason)
{
    set_mode(Mode::Number::LAND, reason);
    mode_land.set_land_pause(true);

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}

// landing_with_GPS - returns true if vehicle is landing using GPS
bool Copter::landing_with_GPS()
{
    return (flightmode->mode_number() == Mode::Number::LAND &&
            mode_land.controlling_position());
}
Mode_loiter.cpp 
#include "Copter.h"

#if MODE_LOITER_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeLoiter::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

#if PRECISION_LANDING == ENABLED
    _precision_loiter_active = false;
#endif

    return true;
}

#if PRECISION_LANDING == ENABLED
bool ModeLoiter::do_precision_loiter()
{
    if (!_precision_loiter_enabled) {
        return false;
    }
    if (copter.ap.land_complete_maybe) {
        return false;        // don't move on the ground
    }
    // if the pilot *really* wants to move the vehicle, let them....
    if (loiter_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }
    if (!copter.precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    return true;
}

void ModeLoiter::precision_loiter_xy()
{
    loiter_nav->clear_pilot_desired_acceleration();
    Vector2f target_pos, target_vel;
    if (!copter.precland.get_target_position_cm(target_pos)) {
        target_pos = inertial_nav.get_position_xy_cm();
    }
    // get the velocity of the target
    copter.precland.get_target_velocity_cms(inertial_nav.get_velocity_xy_cms(), target_vel);

    Vector2f zero;
    Vector2p landing_pos = target_pos.topostype();
    // target vel will remain zero if landing target is stationary
    pos_control->input_pos_vel_accel_xy(landing_pos, target_vel, zero);
    // run pos controller
    pos_control->update_xy_controller();
}
#endif

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeLoiter::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);

    // Loiter State Machine
    switch (loiter_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate);
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate);
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if PRECISION_LANDING == ENABLED
        bool precision_loiter_old_state = _precision_loiter_active;
        if (do_precision_loiter()) {
            precision_loiter_xy();
            _precision_loiter_active = true;
        } else {
            _precision_loiter_active = false;
        }
        if (precision_loiter_old_state && !_precision_loiter_active) {
            // prec loiter was active, not any more, let's init again as user takes control
            loiter_nav->init_target();
        }
        // run loiter controller if we are not doing prec loiter
        if (!_precision_loiter_active) {
            loiter_nav->update();
        }
#else
        loiter_nav->update();
#endif

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

uint32_t ModeLoiter::wp_distance() const
{
    return loiter_nav->get_distance_to_target();
}

int32_t ModeLoiter::wp_bearing() const
{
    return loiter_nav->get_bearing_to_target();
}

#endif
Mode_poshold.cpp
#include "Copter.h"

#if MODE_POSHOLD_ENABLED == ENABLED

/*
 * Init and run calls for PosHold flight mode
 *     PosHold tries to improve upon regular loiter by mixing the pilot input with the loiter controller
 */

#define POSHOLD_SPEED_0                         10      // speed below which it is always safe to switch to loiter

// 400hz loop update rate
#define POSHOLD_BRAKE_TIME_ESTIMATE_MAX         (600*4) // max number of cycles the brake will be applied before we switch to loiter
#define POSHOLD_BRAKE_TO_LOITER_TIMER           (150*4) // Number of cycles to transition from brake mode to loiter mode.  Must be lower than POSHOLD_LOITER_STAB_TIMER
#define POSHOLD_WIND_COMP_START_TIMER           (150*4) // Number of cycles to start wind compensation update after loiter is engaged
#define POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER   (50*4)  // Set it from 100 to 200, the number of centiseconds loiter and manual commands are mixed to make a smooth transition.
#define POSHOLD_SMOOTH_RATE_FACTOR              0.0125f // filter applied to pilot's roll/pitch input as it returns to center.  A lower number will cause the roll/pitch to return to zero more slowly if the brake_rate is also low.
#define POSHOLD_WIND_COMP_TIMER_10HZ            40      // counter value used to reduce wind compensation to 10hz
#define LOOP_RATE_FACTOR                        4       // used to adapt PosHold params to loop_rate
#define TC_WIND_COMP                            0.0025f // Time constant for poshold_update_wind_comp_estimate()

// definitions that are independent of main loop rate
#define POSHOLD_STICK_RELEASE_SMOOTH_ANGLE      1800    // max angle required (in centi-degrees) after which the smooth stick release effect is applied
#define POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX    10      // wind compensation estimates will only run when velocity is at or below this speed in cm/s
#define POSHOLD_WIND_COMP_LEAN_PCT_MAX          0.6666f // wind compensation no more than 2/3rds of angle max to ensure pilot can always override

// poshold_init - initialise PosHold controller
bool ModePosHold::init(bool ignore_checks)
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise lean angles to current attitude
    pilot_roll = 0.0f;
    pilot_pitch = 0.0f;

    // compute brake_gain
    brake.gain = (15.0f * (float)g.poshold_brake_rate + 95.0f) * 0.01f;

    if (copter.ap.land_complete) {
        // if landed begin in loiter mode
        roll_mode = RPMode::LOITER;
        pitch_mode = RPMode::LOITER;
    } else {
        // if not landed start in pilot override to avoid hard twitch
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
    }

    // initialise loiter
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // initialise wind_comp each time PosHold is switched on
    init_wind_comp_estimate();

    return true;
}

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
void ModePosHold::run()
{
    float controller_to_pilot_roll_mix; // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    float controller_to_pilot_pitch_mix;    // mix of controller and pilot controls.  0 = fully last controller controls, 1 = fully pilot controls
    const Vector3f& vel = inertial_nav.get_velocity_neu_cms();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    loiter_nav->clear_pilot_desired_acceleration();

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    // get pilot desired climb rate (for alt-hold mode and take-off)
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Pos Hold State Machine Determination
    AltHoldModeState poshold_state = get_alt_hold_state(target_climb_rate);

    // state machine
    switch (poshold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        loiter_nav->update(false);

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;

        // initialise wind compensation estimate
        init_wind_comp_estimate();
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);

        // init and update loiter although pilot is controlling lean angles
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        loiter_nav->update(false);

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHold_Landed_Ground_Idle:
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        loiter_nav->update(false);
        attitude_control->reset_yaw_target_and_rate();
        init_wind_comp_estimate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero

        // set poshold state to pilot override
        roll_mode = RPMode::PILOT_OVERRIDE;
        pitch_mode = RPMode::PILOT_OVERRIDE;
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // poshold specific behaviour to calculate desired roll, pitch angles
    // convert inertial nav earth-frame velocities to body-frame
    // To-Do: move this to AP_Math (or perhaps we already have a function to do this)
    float vel_fw = vel.x*ahrs.cos_yaw() + vel.y*ahrs.sin_yaw();
    float vel_right = -vel.x*ahrs.sin_yaw() + vel.y*ahrs.cos_yaw();

    // If not in LOITER, retrieve latest wind compensation lean angles related to current yaw
    if (roll_mode != RPMode::LOITER || pitch_mode != RPMode::LOITER) {
        get_wind_comp_lean_angles(wind_comp_roll, wind_comp_pitch);
    }

    // Roll state machine
    //  Each state (aka mode) is responsible for:
    //      1. dealing with pilot input
    //      2. calculating the final roll output to the attitude controller
    //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
    switch (roll_mode) {

        case RPMode::PILOT_OVERRIDE:
            // update pilot desired roll angle using latest radio input
            //  this filters the input so that it returns to zero no faster than the brake-rate
            update_pilot_lean_angle(pilot_roll, target_roll);

            // switch to BRAKE mode for next iteration if no pilot input
            if (is_zero(target_roll) && (fabsf(pilot_roll) < 2 * g.poshold_brake_rate)) {
                // initialise BRAKE mode
                roll_mode = RPMode::BRAKE;        // Set brake roll mode
                brake.roll = 0.0f;                  // initialise braking angle to zero
                brake.angle_max_roll = 0.0f;        // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                brake.timeout_roll = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // number of cycles the brake will be applied, updated during braking mode.
                brake.time_updated_roll = false;   // flag the braking time can be re-estimated
            }

            // final lean angle should be pilot input plus wind compensation
            roll = pilot_roll + wind_comp_roll;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // calculate brake.roll angle to counter-act velocity
            update_brake_angle_from_velocity(brake.roll, vel_right);

            // update braking time estimate
            if (!brake.time_updated_roll) {
                // check if brake angle is increasing
                if (fabsf(brake.roll) >= brake.angle_max_roll) {
                    brake.angle_max_roll = fabsf(brake.roll);
                } else {
                    // braking angle has started decreasing so re-estimate braking time
                    brake.timeout_roll = 1+(uint16_t)(LOOP_RATE_FACTOR*15L*(int32_t)(fabsf(brake.roll))/(10L*(int32_t)g.poshold_brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                    brake.time_updated_roll = true;
                }
            }

            // if velocity is very low reduce braking time to 0.5seconds
            if ((fabsf(vel_right) <= POSHOLD_SPEED_0) && (brake.timeout_roll > 50*LOOP_RATE_FACTOR)) {
                brake.timeout_roll = 50*LOOP_RATE_FACTOR;
            }

            // reduce braking timer
            if (brake.timeout_roll > 0) {
                brake.timeout_roll--;
            } else {
                // indicate that we are ready to move to Loiter.
                // Loiter will only actually be engaged once both roll_mode and pitch_mode are changed to RPMode::BRAKE_READY_TO_LOITER
                //  logic for engaging loiter is handled below the roll and pitch mode switch statements
                roll_mode = RPMode::BRAKE_READY_TO_LOITER;
            }

            // final lean angle is braking angle + wind compensation angle
            roll = brake.roll + wind_comp_roll;

            // check for pilot input
            if (!is_zero(target_roll)) {
                // init transition to pilot override
                roll_controller_to_pilot_override();
            }
            break;

        case RPMode::BRAKE_TO_LOITER:
        case RPMode::LOITER:
            // these modes are combined roll-pitch modes and are handled below
            break;

        case RPMode::CONTROLLER_TO_PILOT_OVERRIDE:
            // update pilot desired roll angle using latest radio input
            //  this filters the input so that it returns to zero no faster than the brake-rate
            update_pilot_lean_angle(pilot_roll, target_roll);

            // count-down loiter to pilot timer
            if (controller_to_pilot_timer_roll > 0) {
                controller_to_pilot_timer_roll--;
            } else {
                // when timer runs out switch to full pilot override for next iteration
                roll_mode = RPMode::PILOT_OVERRIDE;
            }

            // calculate controller_to_pilot mix ratio
            controller_to_pilot_roll_mix = (float)controller_to_pilot_timer_roll / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

            // mix final loiter lean angle and pilot desired lean angles
            roll = mix_controls(controller_to_pilot_roll_mix, controller_final_roll, pilot_roll + wind_comp_roll);
            break;
    }

    // Pitch state machine
    //  Each state (aka mode) is responsible for:
    //      1. dealing with pilot input
    //      2. calculating the final pitch output to the attitude contpitcher
    //      3. checking if the state (aka mode) should be changed and if 'yes' perform any required initialisation for the new state
    switch (pitch_mode) {

        case RPMode::PILOT_OVERRIDE:
            // update pilot desired pitch angle using latest radio input
            //  this filters the input so that it returns to zero no faster than the brake-rate
            update_pilot_lean_angle(pilot_pitch, target_pitch);

            // switch to BRAKE mode for next iteration if no pilot input
            if (is_zero(target_pitch) && (fabsf(pilot_pitch) < 2 * g.poshold_brake_rate)) {
                // initialise BRAKE mode
                pitch_mode = RPMode::BRAKE;       // set brake pitch mode
                brake.pitch = 0.0f;                 // initialise braking angle to zero
                brake.angle_max_pitch = 0.0f;       // reset brake_angle_max so we can detect when vehicle begins to flatten out during braking
                brake.timeout_pitch = POSHOLD_BRAKE_TIME_ESTIMATE_MAX; // number of cycles the brake will be applied, updated during braking mode.
                brake.time_updated_pitch = false;   // flag the braking time can be re-estimated
            }

            // final lean angle should be pilot input plus wind compensation
            pitch = pilot_pitch + wind_comp_pitch;
            break;

        case RPMode::BRAKE:
        case RPMode::BRAKE_READY_TO_LOITER:
            // calculate brake_pitch angle to counter-act velocity
            update_brake_angle_from_velocity(brake.pitch, -vel_fw);

            // update braking time estimate
            if (!brake.time_updated_pitch) {
                // check if brake angle is increasing
                if (fabsf(brake.pitch) >= brake.angle_max_pitch) {
                    brake.angle_max_pitch = fabsf(brake.pitch);
                } else {
                    // braking angle has started decreasing so re-estimate braking time
                    brake.timeout_pitch = 1+(uint16_t)(LOOP_RATE_FACTOR*15L*(int32_t)(fabsf(brake.pitch))/(10L*(int32_t)g.poshold_brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                    brake.time_updated_pitch = true;
                }
            }

            // if velocity is very low reduce braking time to 0.5seconds
            if ((fabsf(vel_fw) <= POSHOLD_SPEED_0) && (brake.timeout_pitch > 50*LOOP_RATE_FACTOR)) {
                brake.timeout_pitch = 50*LOOP_RATE_FACTOR;
            }

            // reduce braking timer
            if (brake.timeout_pitch > 0) {
                brake.timeout_pitch--;
            } else {
                // indicate that we are ready to move to Loiter.
                // Loiter will only actually be engaged once both pitch_mode and pitch_mode are changed to RPMode::BRAKE_READY_TO_LOITER
                //  logic for engaging loiter is handled below the pitch and pitch mode switch statements
                pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
            }

            // final lean angle is braking angle + wind compensation angle
            pitch = brake.pitch + wind_comp_pitch;

            // check for pilot input
            if (!is_zero(target_pitch)) {
                // init transition to pilot override
                pitch_controller_to_pilot_override();
            }
            break;

        case RPMode::BRAKE_TO_LOITER:
        case RPMode::LOITER:
            // these modes are combined pitch-pitch modes and are handled below
            break;

        case RPMode::CONTROLLER_TO_PILOT_OVERRIDE:
            // update pilot desired pitch angle using latest radio input
            //  this filters the input so that it returns to zero no faster than the brake-rate
            update_pilot_lean_angle(pilot_pitch, target_pitch);

            // count-down loiter to pilot timer
            if (controller_to_pilot_timer_pitch > 0) {
                controller_to_pilot_timer_pitch--;
            } else {
                // when timer runs out switch to full pilot override for next iteration
                pitch_mode = RPMode::PILOT_OVERRIDE;
            }

            // calculate controller_to_pilot mix ratio
            controller_to_pilot_pitch_mix = (float)controller_to_pilot_timer_pitch / (float)POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;

            // mix final loiter lean angle and pilot desired lean angles
            pitch = mix_controls(controller_to_pilot_pitch_mix, controller_final_pitch, pilot_pitch + wind_comp_pitch);
            break;
    }

    //
    // Shared roll & pitch states (RPMode::BRAKE_TO_LOITER and RPMode::LOITER)
    //

    // switch into LOITER mode when both roll and pitch are ready
    if (roll_mode == RPMode::BRAKE_READY_TO_LOITER && pitch_mode == RPMode::BRAKE_READY_TO_LOITER) {
        roll_mode = RPMode::BRAKE_TO_LOITER;
        pitch_mode = RPMode::BRAKE_TO_LOITER;
        brake.to_loiter_timer = POSHOLD_BRAKE_TO_LOITER_TIMER;
        // init loiter controller
        loiter_nav->init_target(inertial_nav.get_position_xy_cm());
        // set delay to start of wind compensation estimate updates
        wind_comp_start_timer = POSHOLD_WIND_COMP_START_TIMER;
    }

    // roll-mode is used as the combined roll+pitch mode when in BRAKE_TO_LOITER or LOITER modes
    if (roll_mode == RPMode::BRAKE_TO_LOITER || roll_mode == RPMode::LOITER) {

        // force pitch mode to be same as roll_mode just to keep it consistent (it's not actually used in these states)
        pitch_mode = roll_mode;

        // handle combined roll+pitch mode
        switch (roll_mode) {
            case RPMode::BRAKE_TO_LOITER: {
                // reduce brake_to_loiter timer
                if (brake.to_loiter_timer > 0) {
                    brake.to_loiter_timer--;
                } else {
                    // progress to full loiter on next iteration
                    roll_mode = RPMode::LOITER;
                    pitch_mode = RPMode::LOITER;
                }

                // mix of brake and loiter controls.  0 = fully brake
                // controls, 1 = fully loiter controls
                const float brake_to_loiter_mix = (float)brake.to_loiter_timer / (float)POSHOLD_BRAKE_TO_LOITER_TIMER;

                // calculate brake.roll and pitch angles to counter-act velocity
                update_brake_angle_from_velocity(brake.roll, vel_right);
                update_brake_angle_from_velocity(brake.pitch, -vel_fw);

                // run loiter controller
                loiter_nav->update(false);

                // calculate final roll and pitch output by mixing loiter and brake controls
                roll = mix_controls(brake_to_loiter_mix, brake.roll + wind_comp_roll, loiter_nav->get_roll());
                pitch = mix_controls(brake_to_loiter_mix, brake.pitch + wind_comp_pitch, loiter_nav->get_pitch());

                // check for pilot input
                if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                    // if roll input switch to pilot override for roll
                    if (!is_zero(target_roll)) {
                        // init transition to pilot override
                        roll_controller_to_pilot_override();
                        // switch pitch-mode to brake (but ready to go back to loiter anytime)
                        // no need to reset brake.pitch here as wind comp has not been updated since last brake.pitch computation
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                    }
                    // if pitch input switch to pilot override for pitch
                    if (!is_zero(target_pitch)) {
                        // init transition to pilot override
                        pitch_controller_to_pilot_override();
                        if (is_zero(target_roll)) {
                            // switch roll-mode to brake (but ready to go back to loiter anytime)
                            // no need to reset brake.roll here as wind comp has not been updated since last brake.roll computation
                            roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                        }
                    }
                }
                break;
            }
            case RPMode::LOITER:
                // run loiter controller
                loiter_nav->update(false);

                // set roll angle based on loiter controller outputs
                roll = loiter_nav->get_roll();
                pitch = loiter_nav->get_pitch();

                // update wind compensation estimate
                update_wind_comp_estimate();

                // check for pilot input
                if (!is_zero(target_roll) || !is_zero(target_pitch)) {
                    // if roll input switch to pilot override for roll
                    if (!is_zero(target_roll)) {
                        // init transition to pilot override
                        roll_controller_to_pilot_override();
                        // switch pitch-mode to brake (but ready to go back to loiter anytime)
                        pitch_mode = RPMode::BRAKE_READY_TO_LOITER;
                        // reset brake.pitch because wind_comp is now different and should give the compensation of the whole previous loiter angle
                        brake.pitch = 0.0f;
                    }
                    // if pitch input switch to pilot override for pitch
                    if (!is_zero(target_pitch)) {
                        // init transition to pilot override
                        pitch_controller_to_pilot_override();
                        // if roll not overriden switch roll-mode to brake (but be ready to go back to loiter any time)
                        if (is_zero(target_roll)) {
                            roll_mode = RPMode::BRAKE_READY_TO_LOITER;
                            brake.roll = 0.0f;
                        }
                            // if roll not overridden switch roll-mode to brake (but be ready to go back to loiter any time)
                    }
                }
                break;

            default:
                // do nothing for uncombined roll and pitch modes
                break;
        }
    }

    // constrain target pitch/roll angles
    float angle_max = copter.aparm.angle_max;
    roll = constrain_float(roll, -angle_max, angle_max);
    pitch = constrain_float(pitch, -angle_max, angle_max);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll, pitch, target_yaw_rate);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

// poshold_update_pilot_lean_angle - update the pilot's filtered lean angle with the latest raw input received
void ModePosHold::update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw)
{
    // if raw input is large or reversing the vehicle's lean angle immediately set the fitlered angle to the new raw angle
    if ((lean_angle_filtered > 0 && lean_angle_raw < 0) || (lean_angle_filtered < 0 && lean_angle_raw > 0) || (fabsf(lean_angle_raw) > POSHOLD_STICK_RELEASE_SMOOTH_ANGLE)) {
        lean_angle_filtered = lean_angle_raw;
    } else {
        // lean_angle_raw must be pulling lean_angle_filtered towards zero, smooth the decrease
        if (lean_angle_filtered > 0) {
            // reduce the filtered lean angle at 5% or the brake rate (whichever is faster).
            lean_angle_filtered -= MAX(lean_angle_filtered * POSHOLD_SMOOTH_RATE_FACTOR, MAX(1.0f, g.poshold_brake_rate/(float)LOOP_RATE_FACTOR));
            // do not let the filtered angle fall below the pilot's input lean angle.
            // the above line pulls the filtered angle down and the below line acts as a catch
            lean_angle_filtered = MAX(lean_angle_filtered, lean_angle_raw);
        }else{
            lean_angle_filtered += MAX(-lean_angle_filtered * POSHOLD_SMOOTH_RATE_FACTOR, MAX(1.0f, g.poshold_brake_rate/(float)LOOP_RATE_FACTOR));
            lean_angle_filtered = MIN(lean_angle_filtered, lean_angle_raw);
        }
    }
}

// mix_controls - mixes two controls based on the mix_ratio
//  mix_ratio of 1 = use first_control completely, 0 = use second_control completely, 0.5 = mix evenly
float ModePosHold::mix_controls(float mix_ratio, float first_control, float second_control)
{
    mix_ratio = constrain_float(mix_ratio, 0.0f, 1.0f);
    return mix_ratio * first_control + (1.0f - mix_ratio) * second_control;
}

// update_brake_angle_from_velocity - updates the brake_angle based on the vehicle's velocity and brake_gain
//  brake_angle is slewed with the wpnav.poshold_brake_rate and constrained by the wpnav.poshold_braking_angle_max
//  velocity is assumed to be in the same direction as lean angle so for pitch you should provide the velocity backwards (i.e. -ve forward velocity)
void ModePosHold::update_brake_angle_from_velocity(float &brake_angle, float velocity)
{
    float lean_angle;
    float brake_rate = g.poshold_brake_rate;

    brake_rate /= (float)LOOP_RATE_FACTOR;
    if (brake_rate <= 1.0f) {
        brake_rate = 1.0f;
    }

    // calculate velocity-only based lean angle
    if (velocity >= 0) {
        lean_angle = -brake.gain * velocity * (1.0f + 500.0f / (velocity + 60.0f));
    } else {
        lean_angle = -brake.gain * velocity * (1.0f + 500.0f / (-velocity + 60.0f));
    }

    // do not let lean_angle be too far from brake_angle
    brake_angle = constrain_float(lean_angle, brake_angle - brake_rate, brake_angle + brake_rate);

    // constrain final brake_angle
    brake_angle = constrain_float(brake_angle, -(float)g.poshold_brake_angle_max, (float)g.poshold_brake_angle_max);
}

// initialise wind compensation estimate back to zero
void ModePosHold::init_wind_comp_estimate()
{
    wind_comp_ef.zero();
    wind_comp_timer = 0;
    wind_comp_roll = 0.0f;
    wind_comp_pitch = 0.0f;
}

// update_wind_comp_estimate - updates wind compensation estimate
//  should be called at the maximum loop rate when loiter is engaged
void ModePosHold::update_wind_comp_estimate()
{
    // check wind estimate start has not been delayed
    if (wind_comp_start_timer > 0) {
        wind_comp_start_timer--;
        return;
    }

    // check horizontal velocity is low
    if (inertial_nav.get_speed_xy_cms() > POSHOLD_WIND_COMP_ESTIMATE_SPEED_MAX) {
        return;
    }

    // get position controller accel target
    const Vector3f& accel_target = pos_control->get_accel_target_cmss();

    // update wind compensation in earth-frame lean angles
    if (is_zero(wind_comp_ef.x)) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        wind_comp_ef.x = accel_target.x;
    } else {
        // low pass filter the position controller's lean angle output
        wind_comp_ef.x = (1.0f-TC_WIND_COMP)*wind_comp_ef.x + TC_WIND_COMP*accel_target.x;
    }
    if (is_zero(wind_comp_ef.y)) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        wind_comp_ef.y = accel_target.y;
    } else {
        // low pass filter the position controller's lean angle output
        wind_comp_ef.y = (1.0f-TC_WIND_COMP)*wind_comp_ef.y + TC_WIND_COMP*accel_target.y;
    }

    // limit acceleration
    const float accel_lim_cmss = tanf(radians(POSHOLD_WIND_COMP_LEAN_PCT_MAX * copter.aparm.angle_max * 0.01f)) * 981.0f;
    const float wind_comp_ef_len = wind_comp_ef.length();
    if (!is_zero(accel_lim_cmss) && (wind_comp_ef_len > accel_lim_cmss)) {
        wind_comp_ef *= accel_lim_cmss / wind_comp_ef_len;
    }
}

// get_wind_comp_lean_angles - retrieve wind compensation angles in body frame roll and pitch angles
//  should be called at the maximum loop rate
void ModePosHold::get_wind_comp_lean_angles(float &roll_angle, float &pitch_angle)
{
    // reduce rate to 10hz
    wind_comp_timer++;
    if (wind_comp_timer < POSHOLD_WIND_COMP_TIMER_10HZ) {
        return;
    }
    wind_comp_timer = 0;

    // convert earth frame desired accelerations to body frame roll and pitch lean angles
    roll_angle = atanf((-wind_comp_ef.x*ahrs.sin_yaw() + wind_comp_ef.y*ahrs.cos_yaw())/(GRAVITY_MSS*100))*(18000.0f/M_PI);
    pitch_angle = atanf(-(wind_comp_ef.x*ahrs.cos_yaw() + wind_comp_ef.y*ahrs.sin_yaw())/(GRAVITY_MSS*100))*(18000.0f/M_PI);
}

// roll_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
void ModePosHold::roll_controller_to_pilot_override()
{
    roll_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_timer_roll = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    // initialise pilot_roll to 0, wind_comp will be updated to compensate and poshold_update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    pilot_roll = 0.0f;
    // store final controller output for mixing with pilot input
    controller_final_roll = roll;
}

// pitch_controller_to_pilot_override - initialises transition from a controller submode (brake or loiter) to a pilot override on roll axis
void ModePosHold::pitch_controller_to_pilot_override()
{
    pitch_mode = RPMode::CONTROLLER_TO_PILOT_OVERRIDE;
    controller_to_pilot_timer_pitch = POSHOLD_CONTROLLER_TO_PILOT_MIX_TIMER;
    // initialise pilot_pitch to 0, wind_comp will be updated to compensate and update_pilot_lean_angle function shall not smooth this transition at next iteration. so 0 is the right value
    pilot_pitch = 0.0f;
    // store final loiter outputs for mixing with pilot input
    controller_final_pitch = pitch;
}

#endif
Mode_rtl.cpp
#include "Copter.h"

#if MODE_RTL_ENABLED == ENABLED

/*
 * Init and run calls for RTL flight mode
 *
 * There are two parts to RTL, the high level decision making which controls which state we are in
 * and the lower implementation of the waypoint or landing controllers within those states
 */

// rtl_init - initialise rtl controller
bool ModeRTL::init(bool ignore_checks)
{
    if (!ignore_checks) {
        if (!AP::ahrs().home_is_set()) {
            return false;
        }
    }
    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init(g.rtl_speed_cms);
    _state = SubMode::STARTING;
    _state_complete = true; // see run() method below
    terrain_following_allowed = !copter.failsafe.terrain;
    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    copter.ap.land_repo_active = false;

    // this will be set true if prec land is later active
    copter.ap.prec_land_active = false;

#if PRECISION_LANDING == ENABLED
    // initialise precland state machine
    copter.precland_statemachine.init();
#endif

    return true;
}

// re-start RTL with terrain following disabled
void ModeRTL::restart_without_terrain()
{
    AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::RESTARTED_RTL);
    terrain_following_allowed = false;
    _state = SubMode::STARTING;
    _state_complete = true;
    gcs().send_text(MAV_SEVERITY_CRITICAL,"Restarting RTL - Terrain data missing");
}

ModeRTL::RTLAltType ModeRTL::get_alt_type() const
{
    // sanity check parameter
    if (g.rtl_alt_type < 0 || g.rtl_alt_type > (int)RTLAltType::RTL_ALTTYPE_TERRAIN) {
        return RTLAltType::RTL_ALTTYPE_RELATIVE;
    }
    return (RTLAltType)g.rtl_alt_type.get();
}

// rtl_run - runs the return-to-launch controller
// should be called at 100hz or more
void ModeRTL::run(bool disarm_on_land)
{
    if (!motors->armed()) {
        return;
    }

    // check if we need to move to next state
    if (_state_complete) {
        switch (_state) {
        case SubMode::STARTING:
            build_path();
            climb_start();
            break;
        case SubMode::INITIAL_CLIMB:
            return_start();
            break;
        case SubMode::RETURN_HOME:
            loiterathome_start();
            break;
        case SubMode::LOITER_AT_HOME:
            if (rtl_path.land || copter.failsafe.radio) {
                land_start();
            }else{
                descent_start();
            }
            break;
        case SubMode::FINAL_DESCENT:
            // do nothing
            break;
        case SubMode::LAND:
            // do nothing - rtl_land_run will take care of disarming motors
            break;
        }
    }

    // call the correct run function
    switch (_state) {

    case SubMode::STARTING:
        // should not be reached:
        _state = SubMode::INITIAL_CLIMB;
        FALLTHROUGH;

    case SubMode::INITIAL_CLIMB:
        climb_return_run();
        break;

    case SubMode::RETURN_HOME:
        climb_return_run();
        break;

    case SubMode::LOITER_AT_HOME:
        loiterathome_run();
        break;

    case SubMode::FINAL_DESCENT:
        descent_run();
        break;

    case SubMode::LAND:
        land_run(disarm_on_land);
        break;
    }
}

// rtl_climb_start - initialise climb to RTL altitude
void ModeRTL::climb_start()
{
    _state = SubMode::INITIAL_CLIMB;
    _state_complete = false;

    // set the destination
    if (!wp_nav->set_wp_destination_loc(rtl_path.climb_target) || !wp_nav->set_wp_destination_next_loc(rtl_path.return_target)) {
        // this should not happen because rtl_build_path will have checked terrain data was available
        gcs().send_text(MAV_SEVERITY_CRITICAL,"RTL: unexpected error setting climb target");
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        copter.set_mode(Mode::Number::LAND, ModeReason::TERRAIN_FAILSAFE);
        return;
    }

    // hold current yaw during initial climb
    auto_yaw.set_mode(AUTO_YAW_HOLD);
}

// rtl_return_start - initialise return to home
void ModeRTL::return_start()
{
    _state = SubMode::RETURN_HOME;
    _state_complete = false;

    if (!wp_nav->set_wp_destination_loc(rtl_path.return_target)) {
        // failure must be caused by missing terrain data, restart RTL
        restart_without_terrain();
    }

    // initialise yaw to point home (maybe)
    auto_yaw.set_mode_to_default(true);
}

// rtl_climb_return_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
void ModeRTL::climb_return_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_thrust_vector_heading(wp_nav->get_thrust_vector(), auto_yaw.yaw());
    }

    // check if we've completed this stage of RTL
    _state_complete = wp_nav->reached_wp_destination();
}

// loiterathome_start - initialise return to home
void ModeRTL::loiterathome_start()
{
    _state = SubMode::LOITER_AT_HOME;
    _state_complete = false;
    _loiter_start_time = millis();

    // yaw back to initial take-off heading yaw unless pilot has already overridden yaw
    if (auto_yaw.default_mode(true) != AUTO_YAW_HOLD) {
        auto_yaw.set_mode(AUTO_YAW_RESETTOARMEDYAW);
    } else {
        auto_yaw.set_mode(AUTO_YAW_HOLD);
    }
}

// rtl_climb_return_descent_run - implements the initial climb, return home and descent portions of RTL which all rely on the wp controller
//      called by rtl_run at 100hz or more
void ModeRTL::loiterathome_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_thrust_vector_heading(wp_nav->get_thrust_vector(), auto_yaw.yaw());
    }

    // check if we've completed this stage of RTL
    if ((millis() - _loiter_start_time) >= (uint32_t)g.rtl_loiter_time.get()) {
        if (auto_yaw.mode() == AUTO_YAW_RESETTOARMEDYAW) {
            // check if heading is within 2 degrees of heading when vehicle was armed
            if (abs(wrap_180_cd(ahrs.yaw_sensor-copter.initial_armed_bearing)) <= 200) {
                _state_complete = true;
            }
        } else {
            // we have loitered long enough
            _state_complete = true;
        }
    }
}

// rtl_descent_start - initialise descent to final alt
void ModeRTL::descent_start()
{
    _state = SubMode::FINAL_DESCENT;
    _state_complete = false;

    // initialise altitude target to stopping point
    pos_control->init_z_controller_stopping_point();

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

#if LANDING_GEAR_ENABLED == ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif

#if AC_FENCE == ENABLED
    // disable the fence on landing
    copter.fence.auto_disable_fence_for_landing();
#endif
}

// rtl_descent_run - implements the final descent to the RTL_ALT
//      called by rtl_run at 100hz or more
void ModeRTL::descent_run()
{
    Vector2f vel_correction;
    float target_yaw_rate = 0.0f;

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // process pilot's input
    if (!copter.failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && copter.rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            AP::logger().Write_Event(LogEvent::LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!copter.set_mode(Mode::Number::LOITER, ModeReason::THROTTLE_LAND_ESCAPE)) {
                copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::THROTTLE_LAND_ESCAPE);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // convert pilot input to reposition velocity
            vel_correction = get_pilot_desired_velocity(wp_nav->get_wp_acceleration() * 0.5);

            // record if pilot has overridden roll or pitch
            if (!vel_correction.is_zero()) {
                if (!copter.ap.land_repo_active) {
                    AP::logger().Write_Event(LogEvent::LAND_REPO_ACTIVE);
                }
                copter.ap.land_repo_active = true;
            }
        }

        if (g.land_repositioning || use_pilot_yaw()) {
            // get pilot's desired yaw rate
            target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    Vector2f accel;
    pos_control->input_vel_accel_xy(vel_correction, accel);
    pos_control->update_xy_controller();

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->set_alt_target_with_slew(rtl_path.descent_target.alt);
    pos_control->update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate);

    // check if we've reached within 20cm of final altitude
    _state_complete = labs(rtl_path.descent_target.alt - copter.current_loc.alt) < 20;
}

// land_start - initialise controllers to loiter over home
void ModeRTL::land_start()
{
    _state = SubMode::LAND;
    _state_complete = false;

    // set horizontal speed and acceleration limits
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialise loiter target destination
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

#if LANDING_GEAR_ENABLED == ENABLED
    // optionally deploy landing gear
    copter.landinggear.deploy_for_landing();
#endif

#if AC_FENCE == ENABLED
    // disable the fence on landing
    copter.fence.auto_disable_fence_for_landing();
#endif
}

bool ModeRTL::is_landing() const
{
    return _state == SubMode::LAND;
}

// land_run - run the landing controllers to put the aircraft on the ground
// called by rtl_run at 100hz or more
void ModeRTL::land_run(bool disarm_on_land)
{
    // check if we've completed this stage of RTL
    _state_complete = copter.ap.land_complete;

    // disarm when the landing detector says we've landed
    if (disarm_on_land && copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        copter.arming.disarm(AP_Arming::Method::LANDED);
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run normal landing or precision landing (if enabled)
    land_run_normal_or_precland();
}

void ModeRTL::build_path()
{
    // origin point is our stopping point
    Vector3p stopping_point;
    pos_control->get_stopping_point_xy_cm(stopping_point.xy());
    pos_control->get_stopping_point_z_cm(stopping_point.z);
    rtl_path.origin_point = Location(stopping_point.tofloat(), Location::AltFrame::ABOVE_ORIGIN);
    rtl_path.origin_point.change_alt_frame(Location::AltFrame::ABOVE_HOME);

    // compute return target
    compute_return_target();

    // climb target is above our origin point at the return altitude
    rtl_path.climb_target = Location(rtl_path.origin_point.lat, rtl_path.origin_point.lng, rtl_path.return_target.alt, rtl_path.return_target.get_alt_frame());

    // descent target is below return target at rtl_alt_final
    rtl_path.descent_target = Location(rtl_path.return_target.lat, rtl_path.return_target.lng, g.rtl_alt_final, Location::AltFrame::ABOVE_HOME);

    // set land flag
    rtl_path.land = g.rtl_alt_final <= 0;
}

// compute the return target - home or rally point
//   return target's altitude is updated to a higher altitude that the vehicle can safely return at (frame may also be set)
void ModeRTL::compute_return_target()
{
    // set return target to nearest rally point or home position (Note: alt is absolute)
#if AC_RALLY == ENABLED
    rtl_path.return_target = copter.rally.calc_best_rally_or_home_location(copter.current_loc, ahrs.get_home().alt);
#else
    rtl_path.return_target = ahrs.get_home();
#endif

    // curr_alt is current altitude above home or above terrain depending upon use_terrain
    int32_t curr_alt = copter.current_loc.alt;

    // determine altitude type of return journey (alt-above-home, alt-above-terrain using range finder or alt-above-terrain using terrain database)
    ReturnTargetAltType alt_type = ReturnTargetAltType::RELATIVE;
    if (terrain_following_allowed && (get_alt_type() == RTLAltType::RTL_ALTTYPE_TERRAIN)) {
        // convert RTL_ALT_TYPE and WPNAV_RFNG_USE parameters to ReturnTargetAltType
        switch (wp_nav->get_terrain_source()) {
        case AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE:
            alt_type = ReturnTargetAltType::RELATIVE;
            AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::RTL_MISSING_RNGFND);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: no terrain data, using alt-above-home");
            break;
        case AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER:
            alt_type = ReturnTargetAltType::RANGEFINDER;
            break;
        case AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
            alt_type = ReturnTargetAltType::TERRAINDATABASE;
            break;
        }
    }

    // set curr_alt and return_target.alt from range finder
    if (alt_type == ReturnTargetAltType::RANGEFINDER) {
        if (copter.get_rangefinder_height_interpolated_cm(curr_alt)) {
            // set return_target.alt
            rtl_path.return_target.set_alt_cm(MAX(curr_alt + MAX(0, g.rtl_climb_min), MAX(g.rtl_altitude, RTL_ALT_MIN)), Location::AltFrame::ABOVE_TERRAIN);
        } else {
            // fallback to relative alt and warn user
            alt_type = ReturnTargetAltType::RELATIVE;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: rangefinder unhealthy, using alt-above-home");
            AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::RTL_MISSING_RNGFND);
        }
    }

    // set curr_alt and return_target.alt from terrain database
    if (alt_type == ReturnTargetAltType::TERRAINDATABASE) {
        // set curr_alt to current altitude above terrain
        // convert return_target.alt from an abs (above MSL) to altitude above terrain
        //   Note: the return_target may be a rally point with the alt set above the terrain alt (like the top of a building)
        int32_t curr_terr_alt;
        if (copter.current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, curr_terr_alt) &&
            rtl_path.return_target.change_alt_frame(Location::AltFrame::ABOVE_TERRAIN)) {
            curr_alt = curr_terr_alt;
        } else {
            // fallback to relative alt and warn user
            alt_type = ReturnTargetAltType::RELATIVE;
            AP::logger().Write_Error(LogErrorSubsystem::TERRAIN, LogErrorCode::MISSING_TERRAIN_DATA);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "RTL: no terrain data, using alt-above-home");
        }
    }

    // for the default case we must convert return-target alt (which is an absolute alt) to alt-above-home
    if (alt_type == ReturnTargetAltType::RELATIVE) {
        if (!rtl_path.return_target.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
            // this should never happen but just in case
            rtl_path.return_target.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
            gcs().send_text(MAV_SEVERITY_WARNING, "RTL: unexpected error calculating target alt");
        }
    }

    // set new target altitude to return target altitude
    // Note: this is alt-above-home or terrain-alt depending upon rtl_alt_type
    // Note: ignore negative altitudes which could happen if user enters negative altitude for rally point or terrain is higher at rally point compared to home
    int32_t target_alt = MAX(rtl_path.return_target.alt, 0);

    // increase target to maximum of current altitude + climb_min and rtl altitude
    target_alt = MAX(target_alt, curr_alt + MAX(0, g.rtl_climb_min));
    target_alt = MAX(target_alt, MAX(g.rtl_altitude, RTL_ALT_MIN));

    // reduce climb if close to return target
    float rtl_return_dist_cm = rtl_path.return_target.get_distance(rtl_path.origin_point) * 100.0f;
    // don't allow really shallow slopes
    if (g.rtl_cone_slope >= RTL_MIN_CONE_SLOPE) {
        target_alt = MAX(curr_alt, MIN(target_alt, MAX(rtl_return_dist_cm*g.rtl_cone_slope, curr_alt+RTL_ABS_MIN_CLIMB)));
    }

    // set returned target alt to new target_alt (don't change altitude type)
    rtl_path.return_target.set_alt_cm(target_alt, (alt_type == ReturnTargetAltType::RELATIVE) ? Location::AltFrame::ABOVE_HOME : Location::AltFrame::ABOVE_TERRAIN);

#if AC_FENCE == ENABLED
    // ensure not above fence altitude if alt fence is enabled
    // Note: because the rtl_path.climb_target's altitude is simply copied from the return_target's altitude,
    //       if terrain altitudes are being used, the code below which reduces the return_target's altitude can lead to
    //       the vehicle not climbing at all as RTL begins.  This can be overly conservative and it might be better
    //       to apply the fence alt limit independently on the origin_point and return_target
    if ((copter.fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        // get return target as alt-above-home so it can be compared to fence's alt
        if (rtl_path.return_target.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_alt)) {
            float fence_alt = copter.fence.get_safe_alt_max()*100.0f;
            if (target_alt > fence_alt) {
                // reduce target alt to the fence alt
                rtl_path.return_target.alt -= (target_alt - fence_alt);
            }
        }
    }
#endif

    // ensure we do not descend
    rtl_path.return_target.alt = MAX(rtl_path.return_target.alt, curr_alt);
}

bool ModeRTL::get_wp(Location& destination) const
{
    // provide target in states which use wp_nav
    switch (_state) {
    case SubMode::STARTING:
    case SubMode::INITIAL_CLIMB:
    case SubMode::RETURN_HOME:
    case SubMode::LOITER_AT_HOME:
    case SubMode::FINAL_DESCENT:
        return wp_nav->get_oa_wp_destination(destination);
    case SubMode::LAND:
        return false;
    }

    // we should never get here but just in case
    return false;
}

uint32_t ModeRTL::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

int32_t ModeRTL::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}

// returns true if pilot's yaw input should be used to adjust vehicle's heading
bool ModeRTL::use_pilot_yaw(void) const
{
    return (copter.g2.rtl_options.get() & uint32_t(Options::IgnorePilotYaw)) == 0;
}

#endif
Mode_smart_rtl.cpp
#include "Copter.h"

#if MODE_SMARTRTL_ENABLED == ENABLED

/*
 * Init and run calls for Smart_RTL flight mode
 *
 * This code uses the SmartRTL path that is already in memory, and feeds it into WPNav, one point at a time.
 * Once the copter is close to home, it will run a standard land controller.
 */

bool ModeSmartRTL::init(bool ignore_checks)
{
    if (g2.smart_rtl.is_active()) {
        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();

        // set current target to a reasonable stopping point
        Vector3p stopping_point;
        pos_control->get_stopping_point_xy_cm(stopping_point.xy());
        pos_control->get_stopping_point_z_cm(stopping_point.z);
        wp_nav->set_wp_destination(stopping_point.tofloat());

        // initialise yaw to obey user parameter
        auto_yaw.set_mode_to_default(true);

        // wait for cleanup of return path
        smart_rtl_state = SubMode::WAIT_FOR_PATH_CLEANUP;
        return true;
    }

    return false;
}

// perform cleanup required when leaving smart_rtl
void ModeSmartRTL::exit()
{
    g2.smart_rtl.cancel_request_for_thorough_cleanup();
}

void ModeSmartRTL::run()
{
    switch (smart_rtl_state) {
        case SubMode::WAIT_FOR_PATH_CLEANUP:
            wait_cleanup_run();
            break;
        case SubMode::PATH_FOLLOW:
            path_follow_run();
            break;
        case SubMode::PRELAND_POSITION:
            pre_land_position_run();
            break;
        case SubMode::DESCEND:
            descent_run(); // Re-using the descend method from normal rtl mode.
            break;
        case SubMode::LAND:
            land_run(true); // Re-using the land method from normal rtl mode.
            break;
    }
}

bool ModeSmartRTL::is_landing() const
{
    return smart_rtl_state == SubMode::LAND;
}

void ModeSmartRTL::wait_cleanup_run()
{
    // hover at current target position
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_heading(wp_nav->get_thrust_vector(), auto_yaw.yaw());

    // check if return path is computed and if yes, begin journey home
    if (g2.smart_rtl.request_thorough_cleanup()) {
        path_follow_last_pop_fail_ms = 0;
        smart_rtl_state = SubMode::PATH_FOLLOW;
    }
}

void ModeSmartRTL::path_follow_run()
{
    float target_yaw_rate = 0.0f;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // if we are close to current target point, switch the next point to be our target.
    if (wp_nav->reached_wp_destination()) {
        Vector3f dest_NED;
        // this pop_point can fail if the IO task currently has the
        // path semaphore.
        if (g2.smart_rtl.pop_point(dest_NED)) {
            path_follow_last_pop_fail_ms = 0;
            if (g2.smart_rtl.get_num_points() == 0) {
                // this is the very last point, add 2m to the target alt and move to pre-land state
                dest_NED.z -= 2.0f;
                smart_rtl_state = SubMode::PRELAND_POSITION;
                wp_nav->set_wp_destination_NED(dest_NED);
            } else {
                // peek at the next point.  this can fail if the IO task currently has the path semaphore
                Vector3f next_dest_NED;
                if (g2.smart_rtl.peek_point(next_dest_NED)) {
                    wp_nav->set_wp_destination_NED(dest_NED);
                    if (g2.smart_rtl.get_num_points() == 1) {
                        // this is the very last point, add 2m to the target alt
                        next_dest_NED.z -= 2.0f;
                    }
                    wp_nav->set_wp_destination_next_NED(next_dest_NED);
                } else {
                    // this can only happen if peek failed to take the semaphore
                    // send next point anyway which will cause the vehicle to slow at the next point
                    wp_nav->set_wp_destination_NED(dest_NED);
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                }
            }
        } else if (g2.smart_rtl.get_num_points() == 0) {
            // We should never get here; should always have at least
            // two points and the "zero points left" is handled above.
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            smart_rtl_state = SubMode::PRELAND_POSITION;
        } else if (path_follow_last_pop_fail_ms == 0) {
            // first time we've failed to pop off (ever, or after a success)
            path_follow_last_pop_fail_ms = AP_HAL::millis();
        } else if (AP_HAL::millis() - path_follow_last_pop_fail_ms > 10000) {
            // we failed to pop a point off for 10 seconds.  This is
            // almost certainly a bug.
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            smart_rtl_state = SubMode::PRELAND_POSITION;
        }
    }

    // update controllers
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(wp_nav->get_thrust_vector(), target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_thrust_vector_heading(wp_nav->get_thrust_vector(), auto_yaw.yaw());
    }
}

void ModeSmartRTL::pre_land_position_run()
{
    // if we are close to 2m above start point, we are ready to land.
    if (wp_nav->reached_wp_destination()) {
        // choose descend and hold, or land based on user parameter rtl_alt_final
        if (g.rtl_alt_final <= 0 || copter.failsafe.radio) {
            land_start();
            smart_rtl_state = SubMode::LAND;
        } else {
            set_descent_target_alt(copter.g.rtl_alt_final);
            descent_start();
            smart_rtl_state = SubMode::DESCEND;
        }
    }

    // update controllers
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    wp_nav->update_wpnav();
    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_heading(wp_nav->get_thrust_vector(), auto_yaw.yaw());
}

// save current position for use by the smart_rtl flight mode
void ModeSmartRTL::save_position()
{
    const bool should_save_position = motors->armed() && (copter.flightmode->mode_number() != Mode::Number::SMART_RTL);

    copter.g2.smart_rtl.update(copter.position_ok(), should_save_position);
}

bool ModeSmartRTL::get_wp(Location& destination) const
{
    // provide target in states which use wp_nav
    switch (smart_rtl_state) {
    case SubMode::WAIT_FOR_PATH_CLEANUP:
    case SubMode::PATH_FOLLOW:
    case SubMode::PRELAND_POSITION:
    case SubMode::DESCEND:
        return wp_nav->get_wp_destination_loc(destination);
    case SubMode::LAND:
        return false;
    }

    // we should never get here but just in case
    return false;
}

uint32_t ModeSmartRTL::wp_distance() const
{
    return wp_nav->get_wp_distance_to_destination();
}

int32_t ModeSmartRTL::wp_bearing() const
{
    return wp_nav->get_wp_bearing_to_destination();
}

bool ModeSmartRTL::use_pilot_yaw() const
{
    return g2.smart_rtl.use_pilot_yaw();
}

#endif
Mode_sport.cpp
#include "Copter.h"

#if MODE_SPORT_ENABLED == ENABLED

/*
 * Init and run calls for sport flight mode
 */

// sport_init - initialise sport controller
bool ModeSport::init(bool ignore_checks)
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    return true;
}

// sport_run - runs the sport controller
// should be called at 100hz or more
void ModeSport::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform
    update_simple_mode();

    // get pilot's desired roll and pitch rates

    // calculate rate requests
    float target_roll_rate = channel_roll->get_control_in() * g2.acro_rp_rate * 100.0 / ROLL_PITCH_YAW_INPUT_MAX;
    float target_pitch_rate = channel_pitch->get_control_in() * g2.acro_rp_rate * 100.0 / ROLL_PITCH_YAW_INPUT_MAX;

    // get attitude targets
    const Vector3f att_target = attitude_control->get_att_target_euler_cd();

    // Calculate trainer mode earth frame rate command for roll
    int32_t roll_angle = wrap_180_cd(att_target.x);
    target_roll_rate -= constrain_int32(roll_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_roll;

    // Calculate trainer mode earth frame rate command for pitch
    int32_t pitch_angle = wrap_180_cd(att_target.y);
    target_pitch_rate -= constrain_int32(pitch_angle, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * g.acro_balance_pitch;

    const float angle_max = copter.aparm.angle_max;
    if (roll_angle > angle_max){
        target_roll_rate +=  sqrt_controller(angle_max - roll_angle, g2.acro_rp_rate * 100.0 / ACRO_LEVEL_MAX_OVERSHOOT, attitude_control->get_accel_roll_max_cdss(), G_Dt);
    }else if (roll_angle < -angle_max) {
        target_roll_rate +=  sqrt_controller(-angle_max - roll_angle, g2.acro_rp_rate * 100.0 / ACRO_LEVEL_MAX_OVERSHOOT, attitude_control->get_accel_roll_max_cdss(), G_Dt);
    }

    if (pitch_angle > angle_max){
        target_pitch_rate +=  sqrt_controller(angle_max - pitch_angle, g2.acro_rp_rate * 100.0 / ACRO_LEVEL_MAX_OVERSHOOT, attitude_control->get_accel_pitch_max_cdss(), G_Dt);
    }else if (pitch_angle < -angle_max) {
        target_pitch_rate +=  sqrt_controller(-angle_max - pitch_angle, g2.acro_rp_rate * 100.0 / ACRO_LEVEL_MAX_OVERSHOOT, attitude_control->get_accel_pitch_max_cdss(), G_Dt);
    }

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Sport State Machine Determination
    AltHoldModeState sport_state = get_alt_hold_state(target_climb_rate);

    // State Machine
    switch (sport_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_rate_roll_pitch_yaw(target_roll_rate, target_pitch_rate, target_yaw_rate);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

#endif
Mode_stabilize.cpp
#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                       true,
                                       g.throttle_filt);
}
Mode_stabilize_heli.cpp
#include "Copter.h"

#if FRAME_CONFIG == HELI_FRAME
/*
 * Init and run calls for stabilize flight mode for trad heli
 */

// stabilize_init - initialise stabilize controller
bool ModeStabilize_Heli::init(bool ignore_checks)
{
    // be aware that when adding code to this function that it is *NOT
    // RUN* at vehicle startup!

    // set stab collective true to use stabilize scaled collective pitch range
    copter.input_manager.set_use_stab_col(true);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize_Heli::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    // get pilot's desired throttle
    pilot_throttle_scaled = copter.input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());

    // Tradheli should not reset roll, pitch, yaw targets when motors are not runup while flying, because
    // we may be in autorotation flight.  This is so that the servos move in a realistic fashion while disarmed
    // for operational checks. Also, unlike multicopters we do not set throttle (i.e. collective pitch) to zero
    // so the swash servos move.

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else {
        // heli will not let the spool state progress to THROTTLE_UNLIMITED until motor interlock is enabled
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
        break;
    case AP_Motors::SpoolState::GROUND_IDLE:
        // If aircraft is landed, set target heading to current and reset the integrator
        // Otherwise motors could be at ground idle for practice autorotation
        if ((motors->init_targets_on_arming() && motors->using_leaky_integrator()) || (copter.ap.land_complete && !motors->using_leaky_integrator())) {
            attitude_control->reset_yaw_target_and_rate(false);
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        if (copter.ap.land_complete && !motors->using_leaky_integrator()) {
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle - note that TradHeli does not used angle-boost
    attitude_control->set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
}

#endif  //HELI_FRAME
Mode_systemid.cpp
#include "Copter.h"

#if MODE_SYSTEMID_ENABLED == ENABLED

/*
 * Init and run calls for systemId, flight mode
 */

const AP_Param::GroupInfo ModeSystemId::var_info[] = {

    // @Param: _AXIS
    // @DisplayName: System identification axis
    // @Description: Controls which axis are being excited.  Set to non-zero to see more parameters
    // @User: Standard
    // @Values: 0:None, 1:Input Roll Angle, 2:Input Pitch Angle, 3:Input Yaw Angle, 4:Recovery Roll Angle, 5:Recovery Pitch Angle, 6:Recovery Yaw Angle, 7:Rate Roll, 8:Rate Pitch, 9:Rate Yaw, 10:Mixer Roll, 11:Mixer Pitch, 12:Mixer Yaw, 13:Mixer Thrust
    AP_GROUPINFO_FLAGS("_AXIS", 1, ModeSystemId, axis, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _MAGNITUDE
    // @DisplayName: System identification Chirp Magnitude
    // @Description: Magnitude of sweep in deg, deg/s and 0-1 for mixer outputs.
    // @User: Standard
    AP_GROUPINFO("_MAGNITUDE", 2, ModeSystemId, waveform_magnitude, 15),

    // @Param: _F_START_HZ
    // @DisplayName: System identification Start Frequency
    // @Description: Frequency at the start of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_START_HZ", 3, ModeSystemId, frequency_start, 0.5f),

    // @Param: _F_STOP_HZ
    // @DisplayName: System identification Stop Frequency
    // @Description: Frequency at the end of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_STOP_HZ", 4, ModeSystemId, frequency_stop, 40),

    // @Param: _T_FADE_IN
    // @DisplayName: System identification Fade in time
    // @Description: Time to reach maximum amplitude of sweep
    // @Range: 0 20
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_IN", 5, ModeSystemId, time_fade_in, 15),

    // @Param: _T_REC
    // @DisplayName: System identification Total Sweep length
    // @Description: Time taken to complete the sweep
    // @Range: 0 255
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_REC", 6, ModeSystemId, time_record, 70),

    // @Param: _T_FADE_OUT
    // @DisplayName: System identification Fade out time
    // @Description: Time to reach zero amplitude at the end of the sweep
    // @Range: 0 5
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_OUT", 7, ModeSystemId, time_fade_out, 2),

    AP_GROUPEND
};

ModeSystemId::ModeSystemId(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#define SYSTEM_ID_DELAY     1.0f      // time in seconds waited after system id mode change for frequency sweep injection

// systemId_init - initialise systemId controller
bool ModeSystemId::init(bool ignore_checks)
{
    // check if enabled
    if (axis == 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "No axis selected, SID_AXIS = 0");
        return false;
    }

    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && copter.ap.land_complete && !copter.flightmode->has_manual_throttle()) {
        return false;
    }

#if FRAME_CONFIG == HELI_FRAME
    copter.input_manager.set_use_stab_col(true);
#endif

    att_bf_feedforward = attitude_control->get_bf_feedforward();
    waveform_time = 0.0f;
    time_const_freq = 2.0f / frequency_start; // Two full cycles at the starting frequency
    systemid_state = SystemIDModeState::SYSTEMID_STATE_TESTING;
    log_subsample = 0;

    chirp_input.init(time_record, frequency_start, frequency_stop, time_fade_in, time_fade_out, time_const_freq);

    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Starting: axis=%d", (unsigned)axis);

    copter.Log_Write_SysID_Setup(axis, waveform_magnitude, frequency_start, frequency_stop, time_fade_in, time_const_freq, time_record, time_fade_out);

    return true;
}

// systemId_exit - clean up systemId controller before exiting
void ModeSystemId::exit()
{
    // reset the feedfoward enabled parameter to the initialized state
    attitude_control->bf_feedforward(att_bf_feedforward);
}

// systemId_run - runs the systemId controller
// should be called at 100hz or more
void ModeSystemId::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    // Tradheli doesn't set spool state to ground idle when throttle stick is zero.  Ground idle only set when
    // motor interlock is disabled.
    } else if (copter.ap.throttle_zero && !copter.is_tradheli()) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        // Tradheli initializes targets when going from disarmed to armed state. 
        // init_targets_on_arming is always set true for multicopter.
        if (motors->init_targets_on_arming()) {
            attitude_control->reset_yaw_target_and_rate();
            attitude_control->reset_rate_controller_I_terms_smoothly();
        }
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // get pilot's desired throttle
#if FRAME_CONFIG == HELI_FRAME
    float pilot_throttle_scaled = copter.input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());
#else
    float pilot_throttle_scaled = get_pilot_desired_throttle();
#endif

    if ((systemid_state == SystemIDModeState::SYSTEMID_STATE_TESTING) &&
        (!is_positive(frequency_start) || !is_positive(frequency_stop) || is_negative(time_fade_in) || !is_positive(time_record) || is_negative(time_fade_out) || (time_record <= time_const_freq))) {
        systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
        gcs().send_text(MAV_SEVERITY_INFO, "SystemID Parameter Error");
    }

    waveform_time += G_Dt;
    waveform_sample = chirp_input.update(waveform_time - SYSTEM_ID_DELAY, waveform_magnitude);
    waveform_freq_rads = chirp_input.get_frequency_rads();

    switch (systemid_state) {
        case SystemIDModeState::SYSTEMID_STATE_STOPPED:
            attitude_control->bf_feedforward(att_bf_feedforward);
            break;
        case SystemIDModeState::SYSTEMID_STATE_TESTING:

            if (copter.ap.land_complete) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: Landed");
                break;
            }
            if (attitude_control->lean_angle_deg()*100 > attitude_control->lean_angle_max_cd()) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: lean=%f max=%f", (double)attitude_control->lean_angle_deg(), (double)attitude_control->lean_angle_max_cd());
                break;
            }
            if (waveform_time > SYSTEM_ID_DELAY + time_fade_in + time_const_freq + time_record + time_fade_out) {
                systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Finished");
                break;
            }

            switch ((AxisType)axis.get()) {
                case AxisType::NONE:
                    systemid_state = SystemIDModeState::SYSTEMID_STATE_STOPPED;
                    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: axis = 0");
                    break;
                case AxisType::INPUT_ROLL:
                    target_roll += waveform_sample*100.0f;
                    break;
                case AxisType::INPUT_PITCH:
                    target_pitch += waveform_sample*100.0f;
                    break;
                case AxisType::INPUT_YAW:
                    target_yaw_rate += waveform_sample*100.0f;
                    break;
                case AxisType::RECOVER_ROLL:
                    target_roll += waveform_sample*100.0f;
                    attitude_control->bf_feedforward(false);
                    break;
                case AxisType::RECOVER_PITCH:
                    target_pitch += waveform_sample*100.0f;
                    attitude_control->bf_feedforward(false);
                    break;
                case AxisType::RECOVER_YAW:
                    target_yaw_rate += waveform_sample*100.0f;
                    attitude_control->bf_feedforward(false);
                    break;
                case AxisType::RATE_ROLL:
                    attitude_control->rate_bf_roll_sysid(radians(waveform_sample));
                    break;
                case AxisType::RATE_PITCH:
                    attitude_control->rate_bf_pitch_sysid(radians(waveform_sample));
                    break;
                case AxisType::RATE_YAW:
                    attitude_control->rate_bf_yaw_sysid(radians(waveform_sample));
                    break;
                case AxisType::MIX_ROLL:
                    attitude_control->actuator_roll_sysid(waveform_sample);
                    break;
                case AxisType::MIX_PITCH:
                    attitude_control->actuator_pitch_sysid(waveform_sample);
                    break;
                case AxisType::MIX_YAW:
                    attitude_control->actuator_yaw_sysid(waveform_sample);
                    break;
                case AxisType::MIX_THROTTLE:
                    pilot_throttle_scaled += waveform_sample;
                    break;
            }
            break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    if (copter.is_tradheli()) {
        attitude_control->set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
    } else {
        attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
    }

    if (log_subsample <= 0) {
        log_data();
        if (copter.should_log(MASK_LOG_ATTITUDE_FAST) && copter.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 1;
        } else if (copter.should_log(MASK_LOG_ATTITUDE_FAST)) {
            log_subsample = 2;
        } else if (copter.should_log(MASK_LOG_ATTITUDE_MED)) {
            log_subsample = 4;
        } else {
            log_subsample = 8;
        }
    }
    log_subsample -= 1;
}

// log system id and attitude
void ModeSystemId::log_data() const
{
    Vector3f delta_angle;
    float delta_angle_dt;
    copter.ins.get_delta_angle(delta_angle, delta_angle_dt);

    Vector3f delta_velocity;
    float delta_velocity_dt;
    copter.ins.get_delta_velocity(delta_velocity, delta_velocity_dt);

    if (is_positive(delta_angle_dt) && is_positive(delta_velocity_dt)) {
        copter.Log_Write_SysID_Data(waveform_time, waveform_sample, waveform_freq_rads / (2 * M_PI), degrees(delta_angle.x / delta_angle_dt), degrees(delta_angle.y / delta_angle_dt), degrees(delta_angle.z / delta_angle_dt), delta_velocity.x / delta_velocity_dt, delta_velocity.y / delta_velocity_dt, delta_velocity.z / delta_velocity_dt);
    }

    // Full rate logging of attitude, rate and pid loops
    copter.Log_Write_Attitude();
}

#endif
Mode_throw.cpp
#include "Copter.h"

#if MODE_THROW_ENABLED == ENABLED

// throw_init - initialise throw controller
bool ModeThrow::init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to use throw to start
    return false;
#endif

    // do not enter the mode when already armed or when flying
    if (motors->armed()) {
        return false;
    }

    // init state
    stage = Throw_Disarmed;
    nextmode_attempted = false;

    // initialise pos controller speed and acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), BRAKE_MODE_DECEL_RATE);

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

    return true;
}

// runs the throw to start controller
// should be called at 100hz or more
void ModeThrow::run()
{
    /* Throw State Machine
    Throw_Disarmed - motors are off
    Throw_Detecting -  motors are on and we are waiting for the throw
    Throw_Uprighting - the throw has been detected and the copter is being uprighted
    Throw_HgtStabilise - the copter is kept level and  height is stabilised about the target height
    Throw_PosHold - the copter is kept at a constant position and height
    */

    if (!motors->armed()) {
        // state machine entry is always from a disarmed state
        stage = Throw_Disarmed;

    } else if (stage == Throw_Disarmed && motors->armed()) {
        gcs().send_text(MAV_SEVERITY_INFO,"waiting for throw");
        stage = Throw_Detecting;

    } else if (stage == Throw_Detecting && throw_detected()){
        gcs().send_text(MAV_SEVERITY_INFO,"throw detected - spooling motors");
        stage = Throw_Wait_Throttle_Unlimited;

        // Cancel the waiting for throw tone sequence
        AP_Notify::flags.waiting_for_throw = false;

    } else if (stage == Throw_Wait_Throttle_Unlimited &&
               motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        gcs().send_text(MAV_SEVERITY_INFO,"throttle is unlimited - uprighting");
        stage = Throw_Uprighting;
    } else if (stage == Throw_Uprighting && throw_attitude_good()) {
        gcs().send_text(MAV_SEVERITY_INFO,"uprighted - controlling height");
        stage = Throw_HgtStabilise;

        // initialise the z controller
        pos_control->init_z_controller_no_descent();

        // initialise the demanded height to 3m above the throw height
        // we want to rapidly clear surrounding obstacles
        if (g2.throw_type == ThrowType::Drop) {
            pos_control->set_pos_target_z_cm(inertial_nav.get_position_z_up_cm() - 100);
        } else {
            pos_control->set_pos_target_z_cm(inertial_nav.get_position_z_up_cm() + 300);
        }

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);

    } else if (stage == Throw_HgtStabilise && throw_height_good()) {
        gcs().send_text(MAV_SEVERITY_INFO,"height achieved - controlling position");
        stage = Throw_PosHold;

        // initialise position controller
        pos_control->init_xy_controller();

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        copter.set_auto_armed(true);
    } else if (stage == Throw_PosHold && throw_position_good()) {
        if (!nextmode_attempted) {
            switch ((Mode::Number)g2.throw_nextmode.get()) {
                case Mode::Number::AUTO:
                case Mode::Number::GUIDED:
                case Mode::Number::RTL:
                case Mode::Number::LAND:
                case Mode::Number::BRAKE:
                case Mode::Number::LOITER:
                    set_mode((Mode::Number)g2.throw_nextmode.get(), ModeReason::THROW_COMPLETE);
                    break;
                default:
                    // do nothing
                    break;
            }
            nextmode_attempted = true;
        }
    }

    // Throw State Processing
    switch (stage) {

    case Throw_Disarmed:

        // prevent motors from rotating before the throw is detected unless enabled by the user
        if (g.throw_motor_start == PreThrowMotorState::RUNNING) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        }

        // demand zero throttle (motors will be stopped anyway) and continually reset the attitude controller
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        break;

    case Throw_Detecting:

        // prevent motors from rotating before the throw is detected unless enabled by the user
        if (g.throw_motor_start == PreThrowMotorState::RUNNING) {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        } else {
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        }

        // Hold throttle at zero during the throw and continually reset the attitude controller
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_throttle_out(0,true,g.throttle_filt);

        // Play the waiting for throw tone sequence to alert the user
        AP_Notify::flags.waiting_for_throw = true;

        break;

    case Throw_Wait_Throttle_Unlimited:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        break;

    case Throw_Uprighting:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // demand a level roll/pitch attitude with zero yaw rate
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

        // output 50% throttle and turn off angle boost to maximise righting moment
        attitude_control->set_throttle_out(0.5f, false, g.throttle_filt);

        break;

    case Throw_HgtStabilise:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);

        // call height controller
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
        pos_control->update_z_controller();

        break;

    case Throw_PosHold:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // use position controller to stop
        Vector2f vel;
        Vector2f accel;
        pos_control->input_vel_accel_xy(vel, accel);
        pos_control->update_xy_controller();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);

        // call height controller
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
        pos_control->update_z_controller();

        break;
    }

    // log at 10hz or if stage changes
    uint32_t now = AP_HAL::millis();
    if ((stage != prev_stage) || (now - last_log_ms) > 100) {
        prev_stage = stage;
        last_log_ms = now;
        const float velocity = inertial_nav.get_velocity_neu_cms().length();
        const float velocity_z = inertial_nav.get_velocity_z_up_cms();
        const float accel = copter.ins.get_accel().length();
        const float ef_accel_z = ahrs.get_accel_ef().z;
        const bool throw_detect = (stage > Throw_Detecting) || throw_detected();
        const bool attitude_ok = (stage > Throw_Uprighting) || throw_attitude_good();
        const bool height_ok = (stage > Throw_HgtStabilise) || throw_height_good();
        const bool pos_ok = (stage > Throw_PosHold) || throw_position_good();
        
// @LoggerMessage: THRO
// @Description: Throw Mode messages
// @URL: https://ardupilot.org/copter/docs/throw-mode.html
// @Field: TimeUS: Time since system startup
// @Field: Stage: Current stage of the Throw Mode
// @Field: Vel: Magnitude of the velocity vector
// @Field: VelZ: Vertical Velocity
// @Field: Acc: Magnitude of the vector of the current acceleration
// @Field: AccEfZ: Vertical earth frame accelerometer value
// @Field: Throw: True if a throw has been detected since entering this mode
// @Field: AttOk: True if the vehicle is upright 
// @Field: HgtOk: True if the vehicle is within 50cm of the demanded height
// @Field: PosOk: True if the vehicle is within 50cm of the demanded horizontal position
        
        AP::logger().WriteStreaming(
            "THRO",
            "TimeUS,Stage,Vel,VelZ,Acc,AccEfZ,Throw,AttOk,HgtOk,PosOk",
            "s-nnoo----",
            "F-0000----",
            "QBffffbbbb",
            AP_HAL::micros64(),
            (uint8_t)stage,
            (double)velocity,
            (double)velocity_z,
            (double)accel,
            (double)ef_accel_z,
            throw_detect,
            attitude_ok,
            height_ok,
            pos_ok);
    }
}

bool ModeThrow::throw_detected()
{
    // Check that we have a valid navigation solution
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    if (!filt_status.flags.attitude || !filt_status.flags.horiz_pos_abs || !filt_status.flags.vert_pos) {
        return false;
    }

    // Check for high speed (>500 cm/s)
    bool high_speed = inertial_nav.get_velocity_neu_cms().length_squared() > (THROW_HIGH_SPEED * THROW_HIGH_SPEED);

    // check for upwards or downwards trajectory (airdrop) of 50cm/s
    bool changing_height;
    if (g2.throw_type == ThrowType::Drop) {
        changing_height = inertial_nav.get_velocity_z_up_cms() < -THROW_VERTICAL_SPEED;
    } else {
        changing_height = inertial_nav.get_velocity_z_up_cms() > THROW_VERTICAL_SPEED;
    }

    // Check the vertical acceleraton is greater than 0.25g
    bool free_falling = ahrs.get_accel_ef().z > -0.25 * GRAVITY_MSS;

    // Check if the accel length is < 1.0g indicating that any throw action is complete and the copter has been released
    bool no_throw_action = copter.ins.get_accel().length() < 1.0f * GRAVITY_MSS;

    // High velocity or free-fall combined with increasing height indicate a possible air-drop or throw release
    bool possible_throw_detected = (free_falling || high_speed) && changing_height && no_throw_action;

    // Record time and vertical velocity when we detect the possible throw
    if (possible_throw_detected && ((AP_HAL::millis() - free_fall_start_ms) > 500)) {
        free_fall_start_ms = AP_HAL::millis();
        free_fall_start_velz = inertial_nav.get_velocity_z_up_cms();
    }

    // Once a possible throw condition has been detected, we check for 2.5 m/s of downwards velocity change in less than 0.5 seconds to confirm
    bool throw_condition_confirmed = ((AP_HAL::millis() - free_fall_start_ms < 500) && ((inertial_nav.get_velocity_z_up_cms() - free_fall_start_velz) < -250.0f));

    // start motors and enter the control mode if we are in continuous freefall
    return throw_condition_confirmed;
}

bool ModeThrow::throw_attitude_good() const
{
    // Check that we have uprighted the copter
    const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
    return (rotMat.c.z > 0.866f); // is_upright
}

bool ModeThrow::throw_height_good() const
{
    // Check that we are within 0.5m of the demanded height
    return (pos_control->get_pos_error_z_cm() < 50.0f);
}

bool ModeThrow::throw_position_good() const
{
    // check that our horizontal position error is within 50cm
    return (pos_control->get_pos_error_xy_cm() < 50.0f);
}

#endif
Mode_turtle.cpp
#include "Copter.h"

#if MODE_TURTLE_ENABLED == ENABLED

#define CRASH_FLIP_EXPO 35.0f
#define CRASH_FLIP_STICK_MINF 0.15f
#define power3(x) ((x) * (x) * (x))

bool ModeTurtle::init(bool ignore_checks)
{
    // do not enter the mode when already armed or when flying
    if (motors->armed() || SRV_Channels::get_dshot_esc_type() == 0) {
        return false;
    }

    // perform minimal arming checks
    if (!copter.mavlink_motor_control_check(*gcs().chan(0), true, "Turtle Mode")) {
        return false;
    }

    // do not enter the mode if sticks are not centered
    if (!is_zero(channel_pitch->norm_input_dz())
        || !is_zero(channel_roll->norm_input_dz())
        || !is_zero(channel_yaw->norm_input_dz())) {
        return false;
    }
    // reverse the motors
    hal.rcout->disable_channel_mask_updates();
    change_motor_direction(true);

    // disable throttle and gps failsafe
    g.failsafe_throttle = FS_THR_DISABLED;
    g.failsafe_gcs = FS_GCS_DISABLED;
    g.fs_ekf_action = 0;

    // arm
    motors->armed(true);
    hal.util->set_soft_armed(true);

    return true;
}

bool ModeTurtle::allows_arming(AP_Arming::Method method) const
{
    return true;
}

void ModeTurtle::exit()
{
    // disarm
    motors->armed(false);
    hal.util->set_soft_armed(false);

    // un-reverse the motors
    change_motor_direction(false);
    hal.rcout->enable_channel_mask_updates();

    // re-enable failsafes
    g.failsafe_throttle.load();
    g.failsafe_gcs.load();
    g.fs_ekf_action.load();
}

void ModeTurtle::change_motor_direction(bool reverse)
{
    AP_HAL::RCOutput::BLHeliDshotCommand direction = reverse ? AP_HAL::RCOutput::DSHOT_REVERSE : AP_HAL::RCOutput::DSHOT_NORMAL;
    AP_HAL::RCOutput::BLHeliDshotCommand inverse_direction = reverse ? AP_HAL::RCOutput::DSHOT_NORMAL : AP_HAL::RCOutput::DSHOT_REVERSE;

    if (!hal.rcout->get_reversed_mask()) {
        hal.rcout->send_dshot_command(direction, AP_HAL::RCOutput::ALL_CHANNELS, 0, 10, true);
    } else {
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
            if (!motors->is_motor_enabled(i)) {
                continue;
            }

            if ((hal.rcout->get_reversed_mask() & (1U << i)) == 0) {
                hal.rcout->send_dshot_command(direction, i, 0, 10, true);
            } else {
                hal.rcout->send_dshot_command(inverse_direction, i, 0, 10, true);
            }
        }
    }
}

void ModeTurtle::run()
{
    const float flip_power_factor = 1.0f - CRASH_FLIP_EXPO * 0.01f;
    const bool norc = copter.failsafe.radio || !copter.ap.rc_receiver_present;
    const float stick_deflection_pitch = norc ? 0.0f : channel_pitch->norm_input_dz();
    const float stick_deflection_roll = norc ? 0.0f : channel_roll->norm_input_dz();
    const float stick_deflection_yaw = norc ? 0.0f : channel_yaw->norm_input_dz();

    const float stick_deflection_pitch_abs = fabsf(stick_deflection_pitch);
    const float stick_deflection_roll_abs = fabsf(stick_deflection_roll);
    const float stick_deflection_yaw_abs = fabsf(stick_deflection_yaw);

    const float stick_deflection_pitch_expo = flip_power_factor * stick_deflection_pitch_abs + power3(stick_deflection_pitch_abs) * (1 - flip_power_factor);
    const float stick_deflection_roll_expo = flip_power_factor * stick_deflection_roll_abs + power3(stick_deflection_roll_abs) * (1 - flip_power_factor);
    const float stick_deflection_yaw_expo = flip_power_factor * stick_deflection_yaw_abs + power3(stick_deflection_yaw_abs) * (1 - flip_power_factor);

    float sign_pitch = stick_deflection_pitch < 0 ? -1 : 1;
    float sign_roll = stick_deflection_roll < 0 ? 1 : -1;

    float stick_deflection_length = sqrtf(sq(stick_deflection_pitch_abs) + sq(stick_deflection_roll_abs));
    float stick_deflection_expo_length = sqrtf(sq(stick_deflection_pitch_expo) + sq(stick_deflection_roll_expo));

    if (stick_deflection_yaw_abs > MAX(stick_deflection_pitch_abs, stick_deflection_roll_abs)) {
        // If yaw is the dominant, disable pitch and roll
        stick_deflection_length = stick_deflection_yaw_abs;
        stick_deflection_expo_length = stick_deflection_yaw_expo;
        sign_roll = 0;
        sign_pitch = 0;
    }

    const float cos_phi = (stick_deflection_length > 0) ? (stick_deflection_pitch_abs + stick_deflection_roll_abs) / (sqrtf(2.0f) * stick_deflection_length) : 0;
    const float cos_threshold = sqrtf(3.0f) / 2.0f; // cos(PI/6.0f)

    if (cos_phi < cos_threshold) {
        // Enforce either roll or pitch exclusively, if not on diagonal
        if (stick_deflection_roll_abs > stick_deflection_pitch_abs) {
            sign_pitch = 0;
        } else {
            sign_roll = 0;
        }
    }

    // Apply a reasonable amount of stick deadband
    const float crash_flip_stick_min_expo = flip_power_factor * CRASH_FLIP_STICK_MINF + power3(CRASH_FLIP_STICK_MINF) * (1 - flip_power_factor);
    const float flip_stick_range = 1.0f - crash_flip_stick_min_expo;
    const float flip_power = MAX(0.0f, stick_deflection_expo_length - crash_flip_stick_min_expo) / flip_stick_range;

    // at this point we have a power value in the range 0..1

    // notmalise the roll and pitch input to match the motors
    Vector2f input{sign_roll, sign_pitch};
    motors_input = input.normalized() * 0.5;
    // we bypass spin min and friends in the deadzone because we only want spin up when the sticks are moved
    motors_output = !is_zero(flip_power) ? motors->thrust_to_actuator(flip_power) : 0.0f;
}

// actually write values to the motors
void ModeTurtle::output_to_motors()
{
    // check if motor are allowed to spin
    const bool allow_output = motors->armed() && motors->get_interlock();

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (!motors->is_motor_enabled(i)) {
            continue;
        }

        const Vector2f output{motors->get_roll_factor(i), motors->get_pitch_factor(i)};
        // if output aligns with input then use this motor
        if (!allow_output || (motors_input - output).length() > 0.5) {
            motors->rc_write(i, motors->get_pwm_output_min());
            continue;
        }

        int16_t pwm = motors->get_pwm_output_min() + (motors->get_pwm_output_max() - motors->get_pwm_output_min()) * motors_output;

        motors->rc_write(i, pwm);
    }
}

#endif
Mode_zigzag.cpp
#include "Copter.h"

#if MODE_ZIGZAG_ENABLED == ENABLED

/*
* Init and run calls for zigzag flight mode
*/

#define ZIGZAG_WP_RADIUS_CM 300
#define ZIGZAG_LINE_INFINITY -1

const AP_Param::GroupInfo ModeZigZag::var_info[] = {
    // @Param: AUTO_ENABLE
    // @DisplayName: ZigZag auto enable/disable
    // @Description: Allows you to enable (1) or disable (0) ZigZag auto feature
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("AUTO_ENABLE", 1, ModeZigZag, _auto_enabled, 0, AP_PARAM_FLAG_ENABLE),

#if SPRAYER_ENABLED == ENABLED
    // @Param: SPRAYER
    // @DisplayName: Auto sprayer in ZigZag
    // @Description: Enable the auto sprayer in ZigZag mode. SPRAY_ENABLE = 1 and SERVOx_FUNCTION = 22(SprayerPump) / 23(SprayerSpinner) also must be set. This makes the sprayer on while moving to destination A or B. The sprayer will stop if the vehicle reaches destination or the flight mode is changed from ZigZag to other.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("SPRAYER", 2, ModeZigZag, _spray_enabled, 0),
#endif // SPRAYER_ENABLED == ENABLED

    // @Param: WP_DELAY
    // @DisplayName: The delay for zigzag waypoint
    // @Description: Waiting time after reached the destination
    // @Units: s
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("WP_DELAY", 3, ModeZigZag, _wp_delay, 0),

    // @Param: SIDE_DIST
    // @DisplayName: Sideways distance in ZigZag auto
    // @Description: The distance to move sideways in ZigZag mode
    // @Units: m
    // @Range: 0.1 100
    // @User: Advanced
    AP_GROUPINFO("SIDE_DIST", 4, ModeZigZag, _side_dist, 4),

    // @Param: DIRECTION
    // @DisplayName: Sideways direction in ZigZag auto
    // @Description: The direction to move sideways in ZigZag mode
    // @Values: 0:forward, 1:right, 2:backward, 3:left
    // @User: Advanced
    AP_GROUPINFO("DIRECTION", 5, ModeZigZag, _direction, 0),

    // @Param: LINE_NUM
    // @DisplayName: Total number of lines
    // @Description: Total number of lines for ZigZag auto if 1 or more. -1: Infinity, 0: Just moving to sideways
    // @Range: -1 32767
    // @User: Advanced
    AP_GROUPINFO("LINE_NUM", 6, ModeZigZag, _line_num, 0),

    AP_GROUPEND
};

ModeZigZag::ModeZigZag(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise zigzag controller
bool ModeZigZag::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        // apply simple mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        float target_roll, target_pitch;
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // initialise waypoint state
    stage = STORING_POINTS;
    dest_A.zero();
    dest_B.zero();

    // initialize zigzag auto
    init_auto();

    return true;
}

// perform cleanup required when leaving zigzag mode
void ModeZigZag::exit()
{
    // The sprayer will stop if the flight mode is changed from ZigZag to other
    spray(false);
}

// run the zigzag controller
// should be called at 100hz or more
void ModeZigZag::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // set the direction and the total number of lines
    zigzag_direction = (Direction)constrain_int16(_direction, 0, 3);
    line_num = constrain_int16(_line_num, ZIGZAG_LINE_INFINITY, 32767);

    // auto control
    if (stage == AUTO) {
        if (is_disarmed_or_landed() || !motors->get_interlock()) {
            // vehicle should be under manual control when disarmed or landed
            return_to_manual_control(false);
        } else if (reached_destination()) {
            // if vehicle has reached destination switch to manual control or moving to A or B
            AP_Notify::events.waypoint_complete = 1;
            if (is_auto) {
                if (line_num == ZIGZAG_LINE_INFINITY || line_count < line_num) {
                    if (auto_stage == AutoState::SIDEWAYS) {
                        save_or_move_to_destination((ab_dest_stored == Destination::A) ? Destination::B : Destination::A);
                    } else {
                        // spray off
                        spray(false);
                        move_to_side();
                    }
                } else {
                    init_auto();
                    return_to_manual_control(true);
                }
            } else {
                return_to_manual_control(true);
            }
        } else {
            auto_control();
        }
    }

    // manual control
    if (stage == STORING_POINTS || stage == MANUAL_REGAIN) {
        // receive pilot's inputs, do position and attitude control
        manual_control();
    }
}

// save current position as A or B.  If both A and B have been saved move to the one specified
void ModeZigZag::save_or_move_to_destination(Destination ab_dest)
{
    // get current position as an offset from EKF origin
    const Vector2f curr_pos {inertial_nav.get_position_xy_cm()};

    // handle state machine changes
    switch (stage) {

        case STORING_POINTS:
            if (ab_dest == Destination::A) {
                // store point A
                dest_A = curr_pos;
                gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: point A stored");
                AP::logger().Write_Event(LogEvent::ZIGZAG_STORE_A);
            } else {
                // store point B
                dest_B = curr_pos;
                gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: point B stored");
                AP::logger().Write_Event(LogEvent::ZIGZAG_STORE_B);
            }
            // if both A and B have been stored advance state
            if (!dest_A.is_zero() && !dest_B.is_zero() && !is_zero((dest_B - dest_A).length_squared())) {
                stage = MANUAL_REGAIN;
                spray(false);
            } else if (!dest_A.is_zero() || !dest_B.is_zero()) {
                // if only A or B have been stored, spray on
                spray(true);
            }
            break;

        case AUTO:
        case MANUAL_REGAIN:
            // A and B have been defined, move vehicle to destination A or B
            Vector3f next_dest;
            bool terr_alt;
            if (calculate_next_dest(ab_dest, stage == AUTO, next_dest, terr_alt)) {
                wp_nav->wp_and_spline_init();
                if (wp_nav->set_wp_destination(next_dest, terr_alt)) {
                    stage = AUTO;
                    auto_stage = AutoState::AB_MOVING;
                    ab_dest_stored = ab_dest;
                    // spray on while moving to A or B
                    spray(true);
                    reach_wp_time_ms = 0;
                    if (is_auto == false || line_num == ZIGZAG_LINE_INFINITY) {
                        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: moving to %s", (ab_dest == Destination::A) ? "A" : "B");
                    } else {
                        line_count++;
                        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: moving to %s (line %d/%d)", (ab_dest == Destination::A) ? "A" : "B", line_count, line_num);
                    }
                }
            }
            break;
    }
}

void ModeZigZag::move_to_side()
{
    if (!dest_A.is_zero() && !dest_B.is_zero() && !is_zero((dest_B - dest_A).length_squared())) {
        Vector3f next_dest;
        bool terr_alt;
        if (calculate_side_dest(next_dest, terr_alt)) {
            wp_nav->wp_and_spline_init();
            if (wp_nav->set_wp_destination(next_dest, terr_alt)) {
                stage = AUTO;
                auto_stage = AutoState::SIDEWAYS;
                current_dest = next_dest;
                current_terr_alt = terr_alt;
                reach_wp_time_ms = 0;
                char const *dir[] = {"forward", "right", "backward", "left"};
                gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: moving to %s", dir[(uint8_t)zigzag_direction]);
            }
        }
    }
}

// return manual control to the pilot
void ModeZigZag::return_to_manual_control(bool maintain_target)
{
    if (stage == AUTO) {
        stage = MANUAL_REGAIN;
        spray(false);
        loiter_nav->clear_pilot_desired_acceleration();
        if (maintain_target) {
            const Vector3f& wp_dest = wp_nav->get_wp_destination();
            loiter_nav->init_target(wp_dest.xy());
            if (wp_nav->origin_and_destination_are_terrain_alt()) {
                copter.surface_tracking.set_target_alt_cm(wp_dest.z);
            }
        } else {
            loiter_nav->init_target();
        }
        is_auto = false;
        gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: manual control");
    }
}

// fly the vehicle to closest point on line perpendicular to dest_A or dest_B
void ModeZigZag::auto_control()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    const bool wpnav_ok = wp_nav->update_wpnav();

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    // call attitude controller
    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);

    // if wpnav failed (because of lack of terrain data) switch back to pilot control for next iteration
    if (!wpnav_ok) {
        return_to_manual_control(false);
    }
}

// manual_control - process manual control
void ModeZigZag::manual_control()
{
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        // make sure the climb rate is in the given range, prevent floating point errors
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we
        // do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // althold state machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

// return true if vehicle is within a small area around the destination
bool ModeZigZag::reached_destination()
{
    // check if wp_nav believes it has reached the destination
    if (!wp_nav->reached_wp_destination()) {
        return false;
    }

    // check distance to destination
    if (wp_nav->get_wp_distance_to_destination() > ZIGZAG_WP_RADIUS_CM) {
        return false;
    }

    // wait at time which is set in zigzag_wp_delay
    uint32_t now = AP_HAL::millis();
    if (reach_wp_time_ms == 0) {
        reach_wp_time_ms = now;
    }
    return ((now - reach_wp_time_ms) >= (uint16_t)constrain_int16(_wp_delay, 0, 127) * 1000);
}

// calculate next destination according to vector A-B and current position
// use_wpnav_alt should be true if waypoint controller's altitude target should be used, false for position control or current altitude target
// terrain_alt is returned as true if the next_dest should be considered a terrain alt
bool ModeZigZag::calculate_next_dest(Destination ab_dest, bool use_wpnav_alt, Vector3f& next_dest, bool& terrain_alt) const
{
    // define start_pos as either destination A or B
    Vector2f start_pos = (ab_dest == Destination::A) ? dest_A : dest_B;

    // calculate vector from A to B
    Vector2f AB_diff = dest_B - dest_A;

    // check distance between A and B
    if (is_zero(AB_diff.length_squared())) {
        return false;
    }

    // get distance from vehicle to start_pos
    const Vector2f curr_pos2d {inertial_nav.get_position_xy_cm()};
    Vector2f veh_to_start_pos = curr_pos2d - start_pos;

    // lengthen AB_diff so that it is at least as long as vehicle is from start point
    // we need to ensure that the lines perpendicular to AB are long enough to reach the vehicle
    float scalar = 1.0f;
    if (veh_to_start_pos.length_squared() > AB_diff.length_squared()) {
        scalar = veh_to_start_pos.length() / AB_diff.length();
    }

    // create a line perpendicular to AB but originating at start_pos
    Vector2f perp1 = start_pos + Vector2f(-AB_diff[1] * scalar, AB_diff[0] * scalar);
    Vector2f perp2 = start_pos + Vector2f(AB_diff[1] * scalar, -AB_diff[0] * scalar);

    // find the closest point on the perpendicular line
    const Vector2f closest2d = Vector2f::closest_point(curr_pos2d, perp1, perp2);
    next_dest.x = closest2d.x;
    next_dest.y = closest2d.y;

    if (use_wpnav_alt) {
        // get altitude target from waypoint controller
        terrain_alt = wp_nav->origin_and_destination_are_terrain_alt();
        next_dest.z = wp_nav->get_wp_destination().z;
    } else {
        // if we have a downward facing range finder then use terrain altitude targets
        terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
        if (terrain_alt) {
            if (!copter.surface_tracking.get_target_alt_cm(next_dest.z)) {
                next_dest.z = copter.rangefinder_state.alt_cm_filt.get();
            }
        } else {
            next_dest.z = pos_control->is_active_z() ? pos_control->get_pos_target_z_cm() : inertial_nav.get_position_z_up_cm();
        }
    }

    return true;
}

// calculate side destination according to vertical vector A-B and current position
// terrain_alt is returned as true if the next_dest should be considered a terrain alt
bool ModeZigZag::calculate_side_dest(Vector3f& next_dest, bool& terrain_alt) const
{
    // calculate vector from A to B
    Vector2f AB_diff = dest_B - dest_A;

    // calculate a vertical right or left vector for AB from the current yaw direction
    Vector2f AB_side;
    if (zigzag_direction == Direction::RIGHT || zigzag_direction == Direction::LEFT) {
        float yaw_ab_sign = (-ahrs.sin_yaw() * AB_diff[1]) + (ahrs.cos_yaw() * -AB_diff[0]);
        if (is_positive(yaw_ab_sign * (zigzag_direction == Direction::RIGHT ? 1 : -1))) {
            AB_side = Vector2f(AB_diff[1], -AB_diff[0]);
        } else {
            AB_side = Vector2f(-AB_diff[1], AB_diff[0]);
        }
    } else {
        float yaw_ab_sign = (ahrs.cos_yaw() * AB_diff[1]) + (ahrs.sin_yaw() * -AB_diff[0]);
        if (is_positive(yaw_ab_sign * (zigzag_direction == Direction::FORWARD ? 1 : -1))) {
            AB_side = Vector2f(AB_diff[1], -AB_diff[0]);
        } else {
            AB_side = Vector2f(-AB_diff[1], AB_diff[0]);
        }
    }

    // check distance the vertical vector between A and B
    if (is_zero(AB_side.length_squared())) {
        return false;
    }

    // adjust AB_side length to zigzag_side_dist
    float scalar = constrain_float(_side_dist, 0.1f, 100.0f) * 100 / safe_sqrt(AB_side.length_squared());

    // get distance from vehicle to start_pos
    const Vector2f curr_pos2d {inertial_nav.get_position_xy_cm()};
    next_dest.x = curr_pos2d.x + (AB_side.x * scalar);
    next_dest.y = curr_pos2d.y + (AB_side.y * scalar);

    // if we have a downward facing range finder then use terrain altitude targets
    terrain_alt = copter.rangefinder_alt_ok() && wp_nav->rangefinder_used_and_healthy();
    if (terrain_alt) {
        if (!copter.surface_tracking.get_target_alt_cm(next_dest.z)) {
            next_dest.z = copter.rangefinder_state.alt_cm_filt.get();
        }
    } else {
        next_dest.z = pos_control->is_active_z() ? pos_control->get_pos_target_z_cm() : inertial_nav.get_position_z_up_cm();
    }

    return true;
}

// run zigzag auto feature which is automate both AB and sideways
void ModeZigZag::run_auto()
{
    // exit immediately if we are disabled
    if (!_auto_enabled) {
        return;
    }

    // make sure both A and B point are registered and not when moving to A or B
    if (stage != MANUAL_REGAIN) {
        return;
    }

    is_auto = true;
    // resume if zigzag auto is suspended
    if (is_suspended && line_count <= line_num) {
        // resume the stage when it was suspended
        if (auto_stage == AutoState::AB_MOVING) {
            line_count--;
            save_or_move_to_destination(ab_dest_stored);
        } else if (auto_stage == AutoState::SIDEWAYS) {
            wp_nav->wp_and_spline_init();
            if (wp_nav->set_wp_destination(current_dest, current_terr_alt)) {
                stage = AUTO;
                reach_wp_time_ms = 0;
                char const *dir[] = {"forward", "right", "backward", "left"};
                gcs().send_text(MAV_SEVERITY_INFO, "ZigZag: moving to %s", dir[(uint8_t)zigzag_direction]);
            }
        }
    } else {
        move_to_side();
    }
}

// suspend zigzag auto
void ModeZigZag::suspend_auto()
{
    // exit immediately if we are disabled
    if (!_auto_enabled) {
        return;
    }

    if (auto_stage != AutoState::MANUAL) {
        is_suspended = true;
        return_to_manual_control(true);
    }
}

// initialize zigzag auto
void ModeZigZag::init_auto()
{
    is_auto = false;
    auto_stage = AutoState::MANUAL;
    line_count = 0;
    is_suspended = false;
}

// spray on / off
void ModeZigZag::spray(bool b)
{
#if SPRAYER_ENABLED == ENABLED
    if (_spray_enabled) {
        copter.sprayer.run(b);
    }
#endif
}

#endif // MODE_ZIGZAG_ENABLED == ENABLED
Motor_test.cpp
#include "Copter.h"

/*
  mavlink motor test - implements the MAV_CMD_DO_MOTOR_TEST mavlink command so that the GCS/pilot can test an individual motor or flaps
                       to ensure proper wiring, rotation.
 */

// motor test definitions
#define MOTOR_TEST_TIMEOUT_SEC          600     // max timeout is 10 minutes (600 seconds)

static uint32_t motor_test_start_ms;        // system time the motor test began
static uint32_t motor_test_timeout_ms;      // test will timeout this many milliseconds after the motor_test_start_ms
static uint8_t motor_test_seq;              // motor sequence number of motor being tested
static uint8_t motor_test_count;            // number of motors to test
static uint8_t motor_test_throttle_type;    // motor throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through)
static float motor_test_throttle_value;  // throttle to be sent to motor, value depends upon it's type

// motor_test_output - checks for timeout and sends updates to motors objects
void Copter::motor_test_output()
{
    // exit immediately if the motor test is not running
    if (!ap.motor_test) {
        return;
    }

    EXPECT_DELAY_MS(2000);

    // check for test timeout
    uint32_t now = AP_HAL::millis();
    if ((now - motor_test_start_ms) >= motor_test_timeout_ms) {
        if (motor_test_count > 1) {
            if (now - motor_test_start_ms < motor_test_timeout_ms*1.5) {
                // output zero for 50% of the test time
                motors->output_min();
            } else {
                // move onto next motor
                motor_test_seq++;
                motor_test_count--;
                motor_test_start_ms = now;
                if (!motors->armed()) {
                    motors->armed(true);
                    hal.util->set_soft_armed(true);
                }
            }
            return;
        }
        // stop motor test
        motor_test_stop();
    } else {
        int16_t pwm = 0;   // pwm that will be output to the motors

        // calculate pwm based on throttle type
        switch (motor_test_throttle_type) {

            case MOTOR_TEST_COMPASS_CAL:
                compass.set_voltage(battery.voltage());
                compass.per_motor_calibration_update();
                FALLTHROUGH;

            case MOTOR_TEST_THROTTLE_PERCENT:
                // sanity check motor_test_throttle value
#if FRAME_CONFIG != HELI_FRAME
                if (motor_test_throttle_value <= 100) {
                    int16_t pwm_min = motors->get_pwm_output_min();
                    int16_t pwm_max = motors->get_pwm_output_max();
                    pwm = (int16_t) (pwm_min + (pwm_max - pwm_min) * motor_test_throttle_value * 1e-2f);
                }
#endif
                break;

            case MOTOR_TEST_THROTTLE_PWM:
                pwm = (int16_t)motor_test_throttle_value;
                break;

            case MOTOR_TEST_THROTTLE_PILOT:
                pwm = channel_throttle->get_radio_in();
                break;

            default:
                motor_test_stop();
                return;
        }

        // sanity check throttle values
        if (pwm >= RC_Channel::RC_MIN_LIMIT_PWM && pwm <= RC_Channel::RC_MAX_LIMIT_PWM) {
            // turn on motor to specified pwm value
            motors->output_test_seq(motor_test_seq, pwm);
        } else {
            motor_test_stop();
        }
    }
}

// mavlink_motor_test_check - perform checks before motor tests can begin
//  return true if tests can continue, false if not
bool Copter::mavlink_motor_control_check(const GCS_MAVLINK &gcs_chan, bool check_rc, const char* mode)
{
    // check board has initialised
    if (!ap.initialised) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"%s: Board initialising", mode);
        return false;
    }

    // check rc has been calibrated
    if (check_rc && !arming.rc_calibration_checks(true)) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"%s: RC not calibrated", mode);
        return false;
    }

    // ensure we are landed
    if (!ap.land_complete) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"%s: vehicle not landed", mode);
        return false;
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"%s: Safety switch", mode);
        return false;
    }

    // check E-Stop is not active
    if (SRV_Channels::get_emergency_stop()) {
        gcs_chan.send_text(MAV_SEVERITY_CRITICAL,"%s: Motor Emergency Stopped", mode);
        return false;
    }

    // if we got this far the check was successful and the motor test can continue
    return true;
}

// mavlink_motor_test_start - start motor test - spin a single motor at a specified pwm
//  returns MAV_RESULT_ACCEPTED on success, MAV_RESULT_FAILED on failure
MAV_RESULT Copter::mavlink_motor_test_start(const GCS_MAVLINK &gcs_chan, uint8_t motor_seq, uint8_t throttle_type, float throttle_value,
                                         float timeout_sec, uint8_t motor_count)
{
    if (motor_count == 0) {
        motor_count = 1;
    }
    // if test has not started try to start it
    if (!ap.motor_test) {
        /* perform checks that it is ok to start test
           The RC calibrated check can be skipped if direct pwm is
           supplied
        */
        if (!mavlink_motor_control_check(gcs_chan, throttle_type != 1, "Motor Test")) {
            return MAV_RESULT_FAILED;
        } else {
            // start test
            gcs().send_text(MAV_SEVERITY_INFO, "starting motor test");
            ap.motor_test = true;

            EXPECT_DELAY_MS(3000);
            // enable and arm motors
            if (!motors->armed()) {
                enable_motor_output();
                motors->armed(true);
                hal.util->set_soft_armed(true);
            }

            // disable throttle and gps failsafe
            g.failsafe_throttle = FS_THR_DISABLED;
            g.failsafe_gcs = FS_GCS_DISABLED;
            g.fs_ekf_action = 0;

            // turn on notify leds
            AP_Notify::flags.esc_calibration = true;
        }
    }

    // set timeout
    motor_test_start_ms = AP_HAL::millis();
    motor_test_timeout_ms = MIN(timeout_sec, MOTOR_TEST_TIMEOUT_SEC) * 1000;

    // store required output
    motor_test_seq = motor_seq;
    motor_test_count = motor_count;
    motor_test_throttle_type = throttle_type;
    motor_test_throttle_value = throttle_value;

    if (motor_test_throttle_type == MOTOR_TEST_COMPASS_CAL) {
        compass.per_motor_calibration_start();
    }            

    // return success
    return MAV_RESULT_ACCEPTED;
}

// motor_test_stop - stops the motor test
void Copter::motor_test_stop()
{
    // exit immediately if the test is not running
    if (!ap.motor_test) {
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "finished motor test");    

    // flag test is complete
    ap.motor_test = false;

    // disarm motors
    motors->armed(false);
    hal.util->set_soft_armed(false);

    // reset timeout
    motor_test_start_ms = 0;
    motor_test_timeout_ms = 0;

    // re-enable failsafes
    g.failsafe_throttle.load();
    g.failsafe_gcs.load();
    g.fs_ekf_action.load();

    if (motor_test_throttle_type == MOTOR_TEST_COMPASS_CAL) {
        compass.per_motor_calibration_end();
    }

    // turn off notify leds
    AP_Notify::flags.esc_calibration = false;
}
Motors.cpp
#include "Copter.h"

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

static uint32_t auto_disarm_begin;

// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
void Copter::arm_motors_check()
{
    static int16_t arming_counter;

    // check if arming/disarm using rudder is allowed
    AP_Arming::RudderArming arming_rudder = arming.get_rudder_arming_type();
    if (arming_rudder == AP_Arming::RudderArming::IS_DISABLED) {
        return;
    }

#if TOY_MODE_ENABLED == ENABLED
    if (g2.toy_mode.enabled()) {
        // not armed with sticks in toy mode
        return;
    }
#endif

    // ensure throttle is down
    if (channel_throttle->get_control_in() > 0) {
        arming_counter = 0;
        return;
    }

    int16_t yaw_in = channel_yaw->get_control_in();

    // full right
    if (yaw_in > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if (arming_counter <= AUTO_TRIM_DELAY) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors->armed()) {
            // reset arming counter if arming fail
            if (!arming.arm(AP_Arming::Method::RUDDER)) {
                arming_counter = 0;
            }
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors->armed() && flightmode->mode_number() == Mode::Number::STABILIZE) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim start");
            auto_trim_counter = 250;
            auto_trim_started = false;
            // ensure auto-disarm doesn't trigger immediately
            auto_disarm_begin = millis();
        }

    // full left and rudder disarming is enabled
    } else if ((yaw_in < -4000) && (arming_rudder == AP_Arming::RudderArming::ARMDISARM)) {
        if (!flightmode->has_manual_throttle() && !ap.land_complete) {
            arming_counter = 0;
            return;
        }

        // increase the counter to a maximum of 1 beyond the disarm delay
        if (arming_counter <= DISARM_DELAY) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors->armed()) {
            arming.disarm(AP_Arming::Method::RUDDER);
        }

    // Yaw is centered so reset arming counter
    } else {
        arming_counter = 0;
    }
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
void Copter::auto_disarm_check()
{
    uint32_t tnow_ms = millis();
    uint32_t disarm_delay_ms = 1000*constrain_int16(g.disarm_delay, 0, 127);

    // exit immediately if we are already disarmed, or if auto
    // disarming is disabled
    if (!motors->armed() || disarm_delay_ms == 0 || flightmode->mode_number() == Mode::Number::THROW) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // if the rotor is still spinning, don't initiate auto disarm
    if (motors->get_spool_state() > AP_Motors::SpoolState::GROUND_IDLE) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // always allow auto disarm if using interlock switch or motors are Emergency Stopped
    if ((ap.using_interlock && !motors->get_interlock()) || SRV_Channels::get_emergency_stop()) {
#if FRAME_CONFIG != HELI_FRAME
        // use a shorter delay if using throttle interlock switch or Emergency Stop, because it is less
        // obvious the copter is armed as the motors will not be spinning
        disarm_delay_ms /= 2;
#endif
    } else {
        bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;
        bool thr_low;
        if (flightmode->has_manual_throttle() || !sprung_throttle_stick) {
            thr_low = ap.throttle_zero;
        } else {
            float deadband_top = get_throttle_mid() + g.throttle_deadzone;
            thr_low = channel_throttle->get_control_in() <= deadband_top;
        }

        if (!thr_low || !ap.land_complete) {
            // reset timer
            auto_disarm_begin = tnow_ms;
        }
    }

    // disarm once timer expires
    if ((tnow_ms-auto_disarm_begin) >= disarm_delay_ms) {
        arming.disarm(AP_Arming::Method::DISARMDELAY);
        auto_disarm_begin = tnow_ms;
    }
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Copter::motors_output()
{
#if ADVANCED_FAILSAFE == ENABLED
    // this is to allow the failsafe module to deliberately crash
    // the vehicle. Only used in extreme circumstances to meet the
    // OBC rules
    if (g2.afs.should_crash_vehicle()) {
        g2.afs.terminate_vehicle();
        if (!g2.afs.terminating_vehicle_via_landing()) {
            return;
        }
        // landing must continue to run the motors output
    }
#endif

    // Update arming delay state
    if (ap.in_arming_delay && (!motors->armed() || millis()-arm_time_ms > ARMING_DELAY_SEC*1.0e3f || flightmode->mode_number() == Mode::Number::THROW)) {
        ap.in_arming_delay = false;
    }

    // output any servo channels
    SRV_Channels::calc_pwm();

    // cork now, so that all channel outputs happen at once
    SRV_Channels::cork();

    // update output on any aux channels, for manual passthru
    SRV_Channels::output_ch_all();

    // update motors interlock state
    bool interlock = motors->armed() && !ap.in_arming_delay && (!ap.using_interlock || ap.motor_interlock_switch) && !SRV_Channels::get_emergency_stop();
    if (!motors->get_interlock() && interlock) {
        motors->set_interlock(true);
        AP::logger().Write_Event(LogEvent::MOTORS_INTERLOCK_ENABLED);
    } else if (motors->get_interlock() && !interlock) {
        motors->set_interlock(false);
        AP::logger().Write_Event(LogEvent::MOTORS_INTERLOCK_DISABLED);
    }

    if (ap.motor_test) {
        // check if we are performing the motor test
        motor_test_output();
    } else {
        // send output signals to motors
        flightmode->output_to_motors();
    }

    // push all channels
    SRV_Channels::push();
}

// check for pilot stick input to trigger lost vehicle alarm
void Copter::lost_vehicle_check()
{
    static uint8_t soundalarm_counter;

    // disable if aux switch is setup to vehicle alarm as the two could interfere
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::LOST_VEHICLE_SOUND)) {
        return;
    }

    // ensure throttle is down, motors not armed, pitch and roll rc at max. Note: rc1=roll rc2=pitch
    if (ap.throttle_zero && !motors->armed() && (channel_roll->get_control_in() > 4000) && (channel_pitch->get_control_in() > 4000)) {
        if (soundalarm_counter >= LOST_VEHICLE_DELAY) {
            if (AP_Notify::flags.vehicle_lost == false) {
                AP_Notify::flags.vehicle_lost = true;
                gcs().send_text(MAV_SEVERITY_NOTICE,"Locate Copter alarm");
            }
        } else {
            soundalarm_counter++;
        }
    } else {
        soundalarm_counter = 0;
        if (AP_Notify::flags.vehicle_lost == true) {
            AP_Notify::flags.vehicle_lost = false;
        }
    }
}
Navigation.cpp
#include "Copter.h"

// run_nav_updates - top level call for the autopilot
// ensures calculations such as "distance to waypoint" are calculated before autopilot makes decisions
// To-Do - rename and move this function to make it's purpose more clear
void Copter::run_nav_updates(void)
{
    update_super_simple_bearing(false);
}

// distance between vehicle and home in cm
uint32_t Copter::home_distance()
{
    if (position_ok()) {
        _home_distance = current_loc.get_distance(ahrs.get_home()) * 100;
    }
    return _home_distance;
}

// The location of home in relation to the vehicle in centi-degrees
int32_t Copter::home_bearing()
{
    if (position_ok()) {
        _home_bearing = current_loc.get_bearing_to(ahrs.get_home());
    }
    return _home_bearing;
}
Precision_landing.cpp
//
// functions to support precision landing
//

#include "Copter.h"

#if PRECISION_LANDING == ENABLED

void Copter::init_precland()
{
    copter.precland.init(400);
}

void Copter::update_precland()
{
    // alt will be unused if we pass false through as the second parameter:
    return precland.update(rangefinder_state.alt_cm_glitch_protected,
                           rangefinder_alt_ok());
}
#endif
Radio.cpp
#include "Copter.h"


// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

void Copter::default_dead_zones()
{
    channel_roll->set_default_dead_zone(20);
    channel_pitch->set_default_dead_zone(20);
#if FRAME_CONFIG == HELI_FRAME
    channel_throttle->set_default_dead_zone(10);
    channel_yaw->set_default_dead_zone(15);
#else
    channel_throttle->set_default_dead_zone(30);
    channel_yaw->set_default_dead_zone(20);
#endif
    rc().channel(CH_6)->set_default_dead_zone(0);
}

void Copter::init_rc_in()
{
    channel_roll     = rc().channel(rcmap.roll()-1);
    channel_pitch    = rc().channel(rcmap.pitch()-1);
    channel_throttle = rc().channel(rcmap.throttle()-1);
    channel_yaw      = rc().channel(rcmap.yaw()-1);

    // set rc channel ranges
    channel_roll->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    channel_pitch->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    channel_yaw->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    channel_throttle->set_range(1000);

    // set default dead zones
    default_dead_zones();

    // initialise throttle_zero flag
    ap.throttle_zero = true;
}

 // init_rc_out -- initialise motors
void Copter::init_rc_out()
{
    motors->set_loop_rate(scheduler.get_loop_rate_hz());
    motors->init((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());

    // enable aux servos to cope with multiple output channels per motor
    SRV_Channels::enable_aux_servos();

    // update rate must be set after motors->init() to allow for motor mapping
    motors->set_update_rate(g.rc_speed);

#if FRAME_CONFIG != HELI_FRAME
    if (channel_throttle->configured_in_storage()) {
        // throttle inputs setup, use those to set motor PWM min and max if not already configured
        motors->convert_pwm_min_max_param(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
    } else {
        // throttle inputs default, force set motor PWM min and max to defaults so they will not be over-written by a future change in RC min / max
        motors->convert_pwm_min_max_param(1000, 2000);
    }
    motors->update_throttle_range();
#else
    // setup correct scaling for ESCs like the UAVCAN ESCs which
    // take a proportion of speed.
    hal.rcout->set_esc_scaling(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
#endif

    // refresh auxiliary channel to function map
    SRV_Channels::update_aux_servo_function();

#if FRAME_CONFIG != HELI_FRAME
    /*
      setup a default safety ignore mask, so that servo gimbals can be active while safety is on
     */
    uint16_t safety_ignore_mask = (~copter.motors->get_motor_mask()) & 0x3FFF;
    BoardConfig.set_default_safety_ignore_mask(safety_ignore_mask);
#endif
}


// enable_motor_output() - enable and output lowest possible value to motors
void Copter::enable_motor_output()
{
    // enable motors
    motors->output_min();
}

void Copter::read_radio()
{
    const uint32_t tnow_ms = millis();

    if (rc().read_input()) {
        ap.new_radio_frame = true;

        set_throttle_and_failsafe(channel_throttle->get_radio_in());
        set_throttle_zero_flag(channel_throttle->get_control_in());

        // RC receiver must be attached if we've just got input
        ap.rc_receiver_present = true;

        // pass pilot input through to motors (used to allow wiggling servos while disarmed on heli, single, coax copters)
        radio_passthrough_to_motors();

        const float dt = (tnow_ms - last_radio_update_ms)*1.0e-3f;
        rc_throttle_control_in_filter.apply(channel_throttle->get_control_in(), dt);
        last_radio_update_ms = tnow_ms;
        return;
    }

    // No radio input this time
    if (failsafe.radio) {
        // already in failsafe!
        return;
    }

    const uint32_t elapsed = tnow_ms - last_radio_update_ms;
    // turn on throttle failsafe if no update from the RC Radio for 500ms or 1000ms if we are using RC_OVERRIDE
    const uint32_t timeout = RC_Channels::has_active_overrides() ? FS_RADIO_RC_OVERRIDE_TIMEOUT_MS : FS_RADIO_TIMEOUT_MS;
    if (elapsed < timeout) {
        // not timed out yet
        return;
    }
    if (!g.failsafe_throttle) {
        // throttle failsafe not enabled
        return;
    }
    if (!ap.rc_receiver_present && !motors->armed()) {
        // we only failsafe if we are armed OR we have ever seen an RC receiver
        return;
    }

    // Nobody ever talks to us.  Log an error and enter failsafe.
    AP::logger().Write_Error(LogErrorSubsystem::RADIO, LogErrorCode::RADIO_LATE_FRAME);
    set_failsafe_radio(true);
}

#define FS_COUNTER 3        // radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value
void Copter::set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    // if failsafe not enabled pass through throttle and exit
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        return;
    }

    //check for low throttle value
    if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (failsafe.radio || !(ap.rc_receiver_present || motors->armed())) {
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are received
        failsafe.radio_counter++;
        if( failsafe.radio_counter >= FS_COUNTER ) {
            failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe_radio(true);
        }
    }else{
        // we have a good throttle so reduce failsafe counter
        failsafe.radio_counter--;
        if( failsafe.radio_counter <= 0 ) {
            failsafe.radio_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (failsafe.radio) {
                set_failsafe_radio(false);
            }
        }
        // pass through throttle
    }
}

#define THROTTLE_ZERO_DEBOUNCE_TIME_MS 400
// set_throttle_zero_flag - set throttle_zero flag from debounced throttle control
// throttle_zero is used to determine if the pilot intends to shut down the motors
// Basically, this signals when we are not flying.  We are either on the ground
// or the pilot has shut down the copter in the air and it is free-falling
void Copter::set_throttle_zero_flag(int16_t throttle_control)
{
    static uint32_t last_nonzero_throttle_ms = 0;
    uint32_t tnow_ms = millis();

    // if not using throttle interlock and non-zero throttle and not E-stopped,
    // or using motor interlock and it's enabled, then motors are running, 
    // and we are flying. Immediately set as non-zero
    if ((!ap.using_interlock && (throttle_control > 0) && !SRV_Channels::get_emergency_stop()) ||
        (ap.using_interlock && motors->get_interlock()) ||
        ap.armed_with_airmode_switch || air_mode == AirMode::AIRMODE_ENABLED) {
        last_nonzero_throttle_ms = tnow_ms;
        ap.throttle_zero = false;
    } else if (tnow_ms - last_nonzero_throttle_ms > THROTTLE_ZERO_DEBOUNCE_TIME_MS) {
        ap.throttle_zero = true;
    }
}

// pass pilot's inputs to motors library (used to allow wiggling servos while disarmed on heli, single, coax copters)
void Copter::radio_passthrough_to_motors()
{
    motors->set_radio_passthrough(channel_roll->norm_input(),
                                  channel_pitch->norm_input(),
                                  channel_throttle->get_control_in_zero_dz()*0.001f,
                                  channel_yaw->norm_input());
}

/*
  return the throttle input for mid-stick as a control-in value
 */
int16_t Copter::get_throttle_mid(void)
{
#if TOY_MODE_ENABLED == ENABLED
    if (g2.toy_mode.enabled()) {
        return g2.toy_mode.get_throttle_mid();
    }
#endif
    return channel_throttle->get_control_mid();
}
Sensors.cpp
#include "Copter.h"

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();

    baro_alt = barometer.get_altitude() * 100.0f;

    motors->set_air_density_ratio(barometer.get_air_density_ratio());
}

void Copter::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
   rangefinder.set_log_rfnd_bit(MASK_LOG_CTUN);
   rangefinder.init(ROTATION_PITCH_270);
   rangefinder_state.alt_cm_filt.set_cutoff_frequency(g2.rangefinder_filt);
   rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);

   // upward facing range finder
   rangefinder_up_state.alt_cm_filt.set_cutoff_frequency(g2.rangefinder_filt);
   rangefinder_up_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_90);
#endif
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

#if RANGEFINDER_TILT_CORRECTION == ENABLED
    const float tilt_correction = MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
#else
    const float tilt_correction = 1.0f;
#endif

    // iterate through downward and upward facing lidar
    struct {
        RangeFinderState &state;
        enum Rotation orientation;
    } rngfnd[2] = {{rangefinder_state, ROTATION_PITCH_270}, {rangefinder_up_state, ROTATION_PITCH_90}};

    for (uint8_t i=0; i < ARRAY_SIZE(rngfnd); i++) {
        // local variables to make accessing simpler
        RangeFinderState &rf_state = rngfnd[i].state;
        enum Rotation rf_orient = rngfnd[i].orientation;

        // update health
        rf_state.alt_healthy = ((rangefinder.status_orient(rf_orient) == RangeFinder::Status::Good) &&
                                (rangefinder.range_valid_count_orient(rf_orient) >= RANGEFINDER_HEALTH_MAX));

        // tilt corrected but unfiltered, not glitch protected alt
        rf_state.alt_cm = tilt_correction * rangefinder.distance_cm_orient(rf_orient);

        // remember inertial alt to allow us to interpolate rangefinder
        rf_state.inertial_alt_cm = inertial_nav.get_position_z_up_cm();

        // glitch handling.  rangefinder readings more than RANGEFINDER_GLITCH_ALT_CM from the last good reading
        // are considered a glitch and glitch_count becomes non-zero
        // glitches clear after RANGEFINDER_GLITCH_NUM_SAMPLES samples in a row.
        // glitch_cleared_ms is set so surface tracking (or other consumers) can trigger a target reset
        const int32_t glitch_cm = rf_state.alt_cm - rf_state.alt_cm_glitch_protected;
        if (glitch_cm >= RANGEFINDER_GLITCH_ALT_CM) {
            rf_state.glitch_count = MAX(rf_state.glitch_count+1, 1);
        } else if (glitch_cm <= -RANGEFINDER_GLITCH_ALT_CM) {
            rf_state.glitch_count = MIN(rf_state.glitch_count-1, -1);
        } else {
            rf_state.glitch_count = 0;
            rf_state.alt_cm_glitch_protected = rf_state.alt_cm;
        }
        if (abs(rf_state.glitch_count) >= RANGEFINDER_GLITCH_NUM_SAMPLES) {
            // clear glitch and record time so consumers (i.e. surface tracking) can reset their target altitudes
            rf_state.glitch_count = 0;
            rf_state.alt_cm_glitch_protected = rf_state.alt_cm;
            rf_state.glitch_cleared_ms = AP_HAL::millis();
        }

        // filter rangefinder altitude
        uint32_t now = AP_HAL::millis();
        const bool timed_out = now - rf_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS;
        if (rf_state.alt_healthy) {
            if (timed_out) {
                // reset filter if we haven't used it within the last second
                rf_state.alt_cm_filt.reset(rf_state.alt_cm);
            } else {
                rf_state.alt_cm_filt.apply(rf_state.alt_cm, 0.05f);
            }
            rf_state.last_healthy_ms = now;
        }

        // send downward facing lidar altitude and health to the libraries that require it
        if (rf_orient == ROTATION_PITCH_270) {
            if (rangefinder_state.alt_healthy || timed_out) {
                wp_nav->set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
#if MODE_CIRCLE_ENABLED
                circle_nav->set_rangefinder_alt(rangefinder_state.enabled && wp_nav->rangefinder_used(), rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
#endif
#if HAL_PROXIMITY_ENABLED
                g2.proximity.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
#endif
            }
        }
    }

#else
    // downward facing rangefinder
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;

    // upward facing rangefinder
    rangefinder_up_state.enabled = false;
    rangefinder_up_state.alt_healthy = false;
    rangefinder_up_state.alt_cm = 0;
#endif
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok() const
{
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_up_ok() const
{
    return (rangefinder_up_state.enabled && rangefinder_up_state.alt_healthy);
}

/*
  get inertially interpolated rangefinder height. Inertial height is
  recorded whenever we update the rangefinder height, then we use the
  difference between the inertial height at that time and the current
  inertial height to give us interpolation of height from rangefinder
 */
bool Copter::get_rangefinder_height_interpolated_cm(int32_t& ret)
{
    if (!rangefinder_alt_ok()) {
        return false;
    }
    ret = rangefinder_state.alt_cm_filt.get();
    float inertial_alt_cm = inertial_nav.get_position_z_up_cm();
    ret += inertial_alt_cm - rangefinder_state.inertial_alt_cm;
    return true;
}
Standby.cpp
#include "Copter.h"

// Run standby functions at approximately 100 Hz to limit maximum variable build up
//
// When standby is active:
//      all I terms are continually reset
//      heading error is reset to zero
//      position errors are reset to zero
//      crash_check is disabled
//      thrust_loss_check is disabled
//      parachute_check is disabled
//      hover throttle learn is disabled
//      and landing detection is disabled.
void Copter::standby_update()
{
    if (!standby_active) {
        return;
    }

    attitude_control->reset_rate_controller_I_terms();
    attitude_control->reset_yaw_target_and_rate();
    pos_control->standby_xyz_reset();
}
Surface_tracking.cpp
#include "Copter.h"

// update_surface_offset - manages the vertical offset of the position controller to follow the measured ground or ceiling
//   level measured using the range finder.
void Copter::SurfaceTracking::update_surface_offset()
{
#if RANGEFINDER_ENABLED == ENABLED
    // check for timeout
    const uint32_t now_ms = millis();
    const bool timeout = (now_ms - last_update_ms) > SURFACE_TRACKING_TIMEOUT_MS;

    // check tracking state and that range finders are healthy
    if (((surface == Surface::GROUND) && copter.rangefinder_alt_ok() && (copter.rangefinder_state.glitch_count == 0)) ||
        ((surface == Surface::CEILING) && copter.rangefinder_up_ok() && (copter.rangefinder_up_state.glitch_count == 0))) {

        // calculate surfaces height above the EKF origin
        // e.g. if vehicle is 10m above the EKF origin and rangefinder reports alt of 3m.  curr_surface_alt_above_origin_cm is 7m (or 700cm)
        RangeFinderState &rf_state = (surface == Surface::GROUND) ? copter.rangefinder_state : copter.rangefinder_up_state;
        const float dir = (surface == Surface::GROUND) ? 1.0f : -1.0f;
        const float curr_surface_alt_above_origin_cm = copter.inertial_nav.get_position_z_up_cm() - dir * rf_state.alt_cm;

        // update position controller target offset to the surface's alt above the EKF origin
        copter.pos_control->set_pos_offset_target_z_cm(curr_surface_alt_above_origin_cm);
        last_update_ms = now_ms;
        valid_for_logging = true;

        // reset target altitude if this controller has just been engaged
        // target has been changed between upwards vs downwards
        // or glitch has cleared
        if (timeout ||
            reset_target ||
            (last_glitch_cleared_ms != rf_state.glitch_cleared_ms)) {
            copter.pos_control->set_pos_offset_z_cm(curr_surface_alt_above_origin_cm);
            reset_target = false;
            last_glitch_cleared_ms = rf_state.glitch_cleared_ms;
        }

    } else {
        // reset position controller offsets if surface tracking is inactive
        // flag target should be reset when/if it next becomes active
        if (timeout) {
            copter.pos_control->set_pos_offset_z_cm(0);
            copter.pos_control->set_pos_offset_target_z_cm(0);
            reset_target = true;
        }
    }
#else
    copter.pos_control->set_pos_offset_z_cm(0);
    copter.pos_control->set_pos_offset_target_z_cm(0);
#endif
}


// get target altitude (in cm) above ground
// returns true if there is a valid target
bool Copter::SurfaceTracking::get_target_alt_cm(float &target_alt_cm) const
{
    // fail if we are not tracking downwards
    if (surface != Surface::GROUND) {
        return false;
    }
    // check target has been updated recently
    if (AP_HAL::millis() - last_update_ms > SURFACE_TRACKING_TIMEOUT_MS) {
        return false;
    }
    target_alt_cm = (copter.pos_control->get_pos_target_z_cm() - copter.pos_control->get_pos_offset_z_cm());
    return true;
}

// set target altitude (in cm) above ground
void Copter::SurfaceTracking::set_target_alt_cm(float _target_alt_cm)
{
    // fail if we are not tracking downwards
    if (surface != Surface::GROUND) {
        return;
    }
    copter.pos_control->set_pos_offset_z_cm(copter.inertial_nav.get_position_z_up_cm() - _target_alt_cm);
    last_update_ms = AP_HAL::millis();
}

bool Copter::SurfaceTracking::get_target_dist_for_logging(float &target_dist) const
{
    if (!valid_for_logging || (surface == Surface::NONE)) {
        return false;
    }

    const float dir = (surface == Surface::GROUND) ? 1.0f : -1.0f;
    target_dist = dir * (copter.pos_control->get_pos_target_z_cm() - copter.pos_control->get_pos_offset_z_cm()) * 0.01f;
    return true;
}

float Copter::SurfaceTracking::get_dist_for_logging() const
{
    return ((surface == Surface::CEILING) ? copter.rangefinder_up_state.alt_cm : copter.rangefinder_state.alt_cm) * 0.01f;
}

// set direction
void Copter::SurfaceTracking::set_surface(Surface new_surface)
{
    if (surface == new_surface) {
        return;
    }
    // check we have a range finder in the correct direction
    if ((new_surface == Surface::GROUND) && !copter.rangefinder.has_orientation(ROTATION_PITCH_270)) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "SurfaceTracking: no downward rangefinder");
        AP_Notify::events.user_mode_change_failed = 1;
        return;
    }
    if ((new_surface == Surface::CEILING) && !copter.rangefinder.has_orientation(ROTATION_PITCH_90)) {
        copter.gcs().send_text(MAV_SEVERITY_WARNING, "SurfaceTracking: no upward rangefinder");
        AP_Notify::events.user_mode_change_failed = 1;
        return;
    }
    surface = new_surface;
    reset_target = true;
}
System.cpp
#include "Copter.h"
#include <AP_ESC_Telem/AP_ESC_Telem.h>

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

static void failsafe_check_static()
{
    copter.failsafe_check();
}

void Copter::init_ardupilot()
{

#if STATS_ENABLED == ENABLED
    // initialise stats module
    g2.stats.init();
#endif

    BoardConfig.init();
#if HAL_MAX_CAN_PROTOCOL_DRIVERS
    can_mgr.init();
#endif

    // init cargo gripper
#if GRIPPER_ENABLED == ENABLED
    g2.gripper.init();
#endif

#if AC_FENCE == ENABLED
    fence.init();
#endif

    // init winch
#if WINCH_ENABLED == ENABLED
    g2.winch.init();
#endif

    // initialise notify system
    notify.init();
    notify_flight_mode();

    // initialise battery monitor
    battery.init();

    // Init RSSI
    rssi.init();
    
    barometer.init();

    // setup telem slots with serial ports
    gcs().setup_uarts();

#if OSD_ENABLED == ENABLED
    osd.init();
#endif

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // update motor interlock state
    update_using_interlock();

#if FRAME_CONFIG == HELI_FRAME
    // trad heli specific initialisation
    heli_init();
#endif
#if FRAME_CONFIG == HELI_FRAME
    input_manager.set_loop_rate(scheduler.get_loop_rate_hz());
#endif

    init_rc_in();               // sets up rc channels from radio

    // initialise surface to be tracked in SurfaceTracking
    // must be before rc init to not override inital switch position
    surface_tracking.init((SurfaceTracking::Surface)copter.g2.surftrak_mode.get());

    // allocate the motors class
    allocate_motors();

    // initialise rc channels including setting mode
    rc().convert_options(RC_Channel::AUX_FUNC::ARMDISARM_UNUSED, RC_Channel::AUX_FUNC::ARMDISARM_AIRMODE);
    rc().init();

    // sets up motors and output to escs
    init_rc_out();

    // check if we should enter esc calibration mode
    esc_calibration_startup_check();

    // motors initialised so parameters can be sent
    ap.initialised_params = true;

    relay.init();

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
     */
    hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

    // Do GPS init
    gps.set_log_gps_bit(MASK_LOG_GPS);
    gps.init(serial_manager);

    AP::compass().set_log_bit(MASK_LOG_COMPASS);
    AP::compass().init();

#if AP_AIRSPEED_ENABLED
    airspeed.set_log_bit(MASK_LOG_IMU);
#endif

#if AC_OAPATHPLANNER_ENABLED == ENABLED
    g2.oa.init();
#endif

    attitude_control->parameter_sanity_check();

#if AP_OPTICALFLOW_ENABLED
    // initialise optical flow sensor
    optflow.init(MASK_LOG_OPTFLOW);
#endif      // AP_OPTICALFLOW_ENABLED

#if HAL_MOUNT_ENABLED
    // initialise camera mount
    camera_mount.init();
#endif

#if PRECISION_LANDING == ENABLED
    // initialise precision landing
    init_precland();
#endif

#if LANDING_GEAR_ENABLED == ENABLED
    // initialise landing gear position
    landinggear.init();
#endif

#ifdef USERHOOK_INIT
    USERHOOK_INIT
#endif

    // read Baro pressure at ground
    //-----------------------------
    barometer.set_log_baro_bit(MASK_LOG_IMU);
    barometer.calibrate();

    // initialise rangefinder
    init_rangefinder();

#if HAL_PROXIMITY_ENABLED
    // init proximity sensor
    g2.proximity.init();
#endif

#if BEACON_ENABLED == ENABLED
    // init beacons used for non-gps position estimation
    g2.beacon.init();
#endif

#if RPM_ENABLED == ENABLED
    // initialise AP_RPM library
    rpm_sensor.init();
#endif

#if MODE_AUTO_ENABLED == ENABLED
    // initialise mission library
    mode_auto.mission.init();
#endif

#if MODE_SMARTRTL_ENABLED == ENABLED
    // initialize SmartRTL
    g2.smart_rtl.init();
#endif

    // initialise AP_Logger library
    logger.setVehicle_Startup_Writer(FUNCTOR_BIND(&copter, &Copter::Log_Write_Vehicle_Startup_Messages, void));

    startup_INS_ground();

#if AP_SCRIPTING_ENABLED
    g2.scripting.init();
#endif // AP_SCRIPTING_ENABLED

    // set landed flags
    set_land_complete(true);
    set_land_complete_maybe(true);

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

    // enable CPU failsafe
    failsafe_enable();

    ins.set_log_raw_bit(MASK_LOG_IMU_RAW);

    // enable output to motors
    if (arming.rc_calibration_checks(true)) {
        enable_motor_output();
    }

    // attempt to set the intial_mode, else set to STABILIZE
    if (!set_mode((enum Mode::Number)g.initial_mode.get(), ModeReason::INITIALISED)) {
        // set mode to STABILIZE will trigger mode change notification to pilot
        set_mode(Mode::Number::STABILIZE, ModeReason::UNAVAILABLE);
    }

    // flag that initialisation has completed
    ap.initialised = true;
}


//******************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//******************************************************************************
void Copter::startup_INS_ground()
{
    // initialise ahrs (may push imu calibration into the mpu6000 if using that device).
    ahrs.init();
    ahrs.set_vehicle_class(AP_AHRS::VehicleClass::COPTER);

    // Warm up and calibrate gyro offsets
    ins.init(scheduler.get_loop_rate_hz());

    // reset ahrs including gyro bias
    ahrs.reset();
}

// position_ok - returns true if the horizontal absolute position is ok and home position is set
bool Copter::position_ok() const
{
    // return false if ekf failsafe has triggered
    if (failsafe.ekf) {
        return false;
    }

    // check ekf position estimate
    return (ekf_has_absolute_position() || ekf_has_relative_position());
}

// ekf_has_absolute_position - returns true if the EKF can provide an absolute WGS-84 position estimate
bool Copter::ekf_has_absolute_position() const
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow navigation with dcm position
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal position
    if (!motors->armed()) {
        return ((filt_status.flags.horiz_pos_abs || filt_status.flags.pred_horiz_pos_abs));
    } else {
        // once armed we require a good absolute position and EKF must not be in const_pos_mode
        return (filt_status.flags.horiz_pos_abs && !filt_status.flags.const_pos_mode);
    }
}

// ekf_has_relative_position - returns true if the EKF can provide a position estimate relative to it's starting position
bool Copter::ekf_has_relative_position() const
{
    // return immediately if EKF not used
    if (!ahrs.have_inertial_nav()) {
        return false;
    }

    // return immediately if neither optflow nor visual odometry is enabled
    bool enabled = false;
#if AP_OPTICALFLOW_ENABLED
    if (optflow.enabled()) {
        enabled = true;
    }
#endif
#if HAL_VISUALODOM_ENABLED
    if (visual_odom.enabled()) {
        enabled = true;
    }
#endif
    if (!enabled) {
        return false;
    }

    // get filter status from EKF
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // if disarmed we accept a predicted horizontal relative position
    if (!motors->armed()) {
        return (filt_status.flags.pred_horiz_pos_rel);
    } else {
        return (filt_status.flags.horiz_pos_rel && !filt_status.flags.const_pos_mode);
    }
}

// returns true if the ekf has a good altitude estimate (required for modes which do AltHold)
bool Copter::ekf_alt_ok() const
{
    if (!ahrs.have_inertial_nav()) {
        // do not allow alt control with only dcm
        return false;
    }

    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    // require both vertical velocity and position
    return (filt_status.flags.vert_vel && filt_status.flags.vert_pos);
}

// update_auto_armed - update status of auto_armed flag
void Copter::update_auto_armed()
{
    // disarm checks
    if(ap.auto_armed){
        // if motors are disarmed, auto_armed should also be false
        if(!motors->armed()) {
            set_auto_armed(false);
            return;
        }
        // if in stabilize or acro flight mode and throttle is zero, auto-armed should become false
        if(flightmode->has_manual_throttle() && ap.throttle_zero && !failsafe.radio) {
            set_auto_armed(false);
        }

    }else{
        // arm checks
        
        // for tradheli if motors are armed and throttle is above zero and the motor is started, auto_armed should be true
        if(motors->armed() && ap.using_interlock) {
            if(!ap.throttle_zero && motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
                set_auto_armed(true);
            }
        // if motors are armed and throttle is above zero auto_armed should be true
        // if motors are armed and we are in throw mode, then auto_armed should be true
        } else if (motors->armed() && !ap.using_interlock) {
            if(!ap.throttle_zero || flightmode->mode_number() == Mode::Number::THROW) {
                set_auto_armed(true);
            }
        }
    }
}

/*
  should we log a message type now?
 */
bool Copter::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    ap.logging_started = logger.logging_started();
    return logger.should_log(mask);
#else
    return false;
#endif
}

/*
  allocate the motors class
 */
void Copter::allocate_motors(void)
{
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
#if FRAME_CONFIG != HELI_FRAME
        case AP_Motors::MOTOR_FRAME_QUAD:
        case AP_Motors::MOTOR_FRAME_HEXA:
        case AP_Motors::MOTOR_FRAME_Y6:
        case AP_Motors::MOTOR_FRAME_OCTA:
        case AP_Motors::MOTOR_FRAME_OCTAQUAD:
        case AP_Motors::MOTOR_FRAME_DODECAHEXA:
        case AP_Motors::MOTOR_FRAME_DECA:
        case AP_Motors::MOTOR_FRAME_SCRIPTING_MATRIX:
        default:
            motors = new AP_MotorsMatrix(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_TRI:
            motors = new AP_MotorsTri(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsTri::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_TRICOPTER);
            break;
        case AP_Motors::MOTOR_FRAME_SINGLE:
            motors = new AP_MotorsSingle(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsSingle::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_COAX:
            motors = new AP_MotorsCoax(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsCoax::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_TAILSITTER:
            motors = new AP_MotorsTailsitter(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsTailsitter::var_info;
            break;
        case AP_Motors::MOTOR_FRAME_6DOF_SCRIPTING:
#if AP_SCRIPTING_ENABLED
            motors = new AP_MotorsMatrix_6DoF_Scripting(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix_6DoF_Scripting::var_info;
#endif // AP_SCRIPTING_ENABLED
            break;
        case AP_Motors::MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX:
#if AP_SCRIPTING_ENABLED
            motors = new AP_MotorsMatrix_Scripting_Dynamic(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsMatrix_Scripting_Dynamic::var_info;
#endif // AP_SCRIPTING_ENABLED
            break;
#else // FRAME_CONFIG == HELI_FRAME
        case AP_Motors::MOTOR_FRAME_HELI_DUAL:
            motors = new AP_MotorsHeli_Dual(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Dual::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;

        case AP_Motors::MOTOR_FRAME_HELI_QUAD:
            motors = new AP_MotorsHeli_Quad(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Quad::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;
            
        case AP_Motors::MOTOR_FRAME_HELI:
        default:
            motors = new AP_MotorsHeli_Single(copter.scheduler.get_loop_rate_hz());
            motors_var_info = AP_MotorsHeli_Single::var_info;
            AP_Param::set_frame_type_flags(AP_PARAM_FRAME_HELI);
            break;
#endif
    }
    if (motors == nullptr) {
        AP_BoardConfig::allocation_error("FRAME_CLASS=%u", (unsigned)g2.frame_class.get());
    }
    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    ahrs_view = ahrs.create_view(ROTATION_NONE);
    if (ahrs_view == nullptr) {
        AP_BoardConfig::allocation_error("AP_AHRS_View");
    }

    const struct AP_Param::GroupInfo *ac_var_info;

#if FRAME_CONFIG != HELI_FRAME
    if ((AP_Motors::motor_frame_class)g2.frame_class.get() == AP_Motors::MOTOR_FRAME_6DOF_SCRIPTING) {
#if AP_SCRIPTING_ENABLED
        attitude_control = new AC_AttitudeControl_Multi_6DoF(*ahrs_view, aparm, *motors, scheduler.get_loop_period_s());
        ac_var_info = AC_AttitudeControl_Multi_6DoF::var_info;
#endif // AP_SCRIPTING_ENABLED
    } else {
        attitude_control = new AC_AttitudeControl_Multi(*ahrs_view, aparm, *motors, scheduler.get_loop_period_s());
        ac_var_info = AC_AttitudeControl_Multi::var_info;
    }
#else
    attitude_control = new AC_AttitudeControl_Heli(*ahrs_view, aparm, *motors, scheduler.get_loop_period_s());
    ac_var_info = AC_AttitudeControl_Heli::var_info;
#endif
    if (attitude_control == nullptr) {
        AP_BoardConfig::allocation_error("AttitudeControl");
    }
    AP_Param::load_object_from_eeprom(attitude_control, ac_var_info);
        
    pos_control = new AC_PosControl(*ahrs_view, inertial_nav, *motors, *attitude_control, scheduler.get_loop_period_s());
    if (pos_control == nullptr) {
        AP_BoardConfig::allocation_error("PosControl");
    }
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);

#if AC_OAPATHPLANNER_ENABLED == ENABLED
    wp_nav = new AC_WPNav_OA(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
#else
    wp_nav = new AC_WPNav(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
#endif
    if (wp_nav == nullptr) {
        AP_BoardConfig::allocation_error("WPNav");
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    loiter_nav = new AC_Loiter(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (loiter_nav == nullptr) {
        AP_BoardConfig::allocation_error("LoiterNav");
    }
    AP_Param::load_object_from_eeprom(loiter_nav, loiter_nav->var_info);

#if MODE_CIRCLE_ENABLED == ENABLED
    circle_nav = new AC_Circle(inertial_nav, *ahrs_view, *pos_control);
    if (circle_nav == nullptr) {
        AP_BoardConfig::allocation_error("CircleNav");
    }
    AP_Param::load_object_from_eeprom(circle_nav, circle_nav->var_info);
#endif

    // reload lines from the defaults file that may now be accessible
    AP_Param::reload_defaults_file(true);
    
    // now setup some frame-class specific defaults
    switch ((AP_Motors::motor_frame_class)g2.frame_class.get()) {
    case AP_Motors::MOTOR_FRAME_Y6:
        attitude_control->get_rate_roll_pid().kP().set_default(0.1);
        attitude_control->get_rate_roll_pid().kD().set_default(0.006);
        attitude_control->get_rate_pitch_pid().kP().set_default(0.1);
        attitude_control->get_rate_pitch_pid().kD().set_default(0.006);
        attitude_control->get_rate_yaw_pid().kP().set_default(0.15);
        attitude_control->get_rate_yaw_pid().kI().set_default(0.015);
        break;
    case AP_Motors::MOTOR_FRAME_TRI:
        attitude_control->get_rate_yaw_pid().filt_D_hz().set_default(100);
        break;
    default:
        break;
    }

    // brushed 16kHz defaults to 16kHz pulses
    if (motors->is_brushed_pwm_type()) {
        g.rc_speed.set_default(16000);
    }
    
    // upgrade parameters. This must be done after allocating the objects
    convert_pid_parameters();
#if FRAME_CONFIG == HELI_FRAME
    convert_tradheli_parameters();
#endif

    // param count could have changed
    AP_Param::invalidate_count();
}

bool Copter::is_tradheli() const
{
#if FRAME_CONFIG == HELI_FRAME
    return true;
#else
    return false;
#endif
}
Takeoff.cpp
#include "Copter.h"

Mode::_TakeOff Mode::takeoff;

bool Mode::auto_takeoff_no_nav_active = false;
float Mode::auto_takeoff_no_nav_alt_cm = 0;
float Mode::auto_takeoff_start_alt_cm = 0;
float Mode::auto_takeoff_complete_alt_cm = 0;
bool Mode::auto_takeoff_terrain_alt = false;
bool Mode::auto_takeoff_complete = false;
Vector3p Mode::auto_takeoff_complete_pos;

// This file contains the high-level takeoff logic for Loiter, PosHold, AltHold, Sport modes.
//   The take-off can be initiated from a GCS NAV_TAKEOFF command which includes a takeoff altitude
//   A safe takeoff speed is calculated and used to calculate a time_ms
//   the pos_control target is then slowly increased until time_ms expires

bool Mode::do_user_takeoff_start(float takeoff_alt_cm)
{
    copter.flightmode->takeoff.start(takeoff_alt_cm);
    return true;
}

// initiate user takeoff - called when MAVLink TAKEOFF command is received
bool Mode::do_user_takeoff(float takeoff_alt_cm, bool must_navigate)
{
    if (!copter.motors->armed()) {
        return false;
    }
    if (!copter.ap.land_complete) {
        // can't takeoff again!
        return false;
    }
    if (!has_user_takeoff(must_navigate)) {
        // this mode doesn't support user takeoff
        return false;
    }
    if (takeoff_alt_cm <= copter.current_loc.alt) {
        // can't takeoff downwards...
        return false;
    }

    // Vehicles using motor interlock should return false if motor interlock is disabled.
    // Interlock must be enabled to allow the controller to spool up the motor(s) for takeoff.
    if (!motors->get_interlock() && copter.ap.using_interlock) {
        return false;
    }

    if (!do_user_takeoff_start(takeoff_alt_cm)) {
        return false;
    }

    copter.set_auto_armed(true);
    return true;
}

// start takeoff to specified altitude above home in centimeters
void Mode::_TakeOff::start(float alt_cm)
{
    // indicate we are taking off
    copter.set_land_complete(false);
    // tell position controller to reset alt target and reset I terms
    copter.flightmode->set_throttle_takeoff();

    // initialise takeoff state
    _running = true;
    take_off_start_alt = copter.pos_control->get_pos_target_z_cm();
    take_off_complete_alt  = take_off_start_alt + alt_cm;
}

// stop takeoff
void Mode::_TakeOff::stop()
{
    _running = false;
}

// do_pilot_takeoff - controls the vertical position controller during the process of taking off
//  take off is complete when the vertical target reaches the take off altitude.
//  climb is cancelled if pilot_climb_rate_cm becomes negative
//  sets take off to complete when target altitude is within 1% of the take off altitude
void Mode::_TakeOff::do_pilot_takeoff(float& pilot_climb_rate_cm)
{
    // return pilot_climb_rate if take-off inactive
    if (!_running) {
        return;
    }

    float pos_z = take_off_complete_alt;
    float vel_z = pilot_climb_rate_cm;

    // command the aircraft to the take off altitude and current pilot climb rate
    copter.pos_control->input_pos_vel_accel_z(pos_z, vel_z, 0);

    // stop take off early and return if negative climb rate is commanded or we are within 0.1% of our take off altitude
    if (is_negative(pilot_climb_rate_cm) ||
        (take_off_complete_alt  - take_off_start_alt) * 0.999f < copter.pos_control->get_pos_target_z_cm() - take_off_start_alt) {
        stop();
    }
}

// auto_takeoff_run - controls the vertical position controller during the process of taking off in auto modes
// auto_takeoff_complete set to true when target altitude is within 10% of the take off altitude and less than 50% max climb rate
void Mode::auto_takeoff_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || !copter.ap.auto_armed) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // get terrain offset
    float terr_offset = 0.0f;
    if (auto_takeoff_terrain_alt && !wp_nav->get_terrain_offset(terr_offset)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "auto takeoff: failed to get terrain offset");
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && copter.flightmode->use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // aircraft stays in landed state until rotor speed run up has finished
    if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        set_land_complete(false);
    } else {
        // motors have not completed spool up yet so relax navigation and position controllers
        pos_control->relax_velocity_controller_xy();
        pos_control->update_xy_controller();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        pos_control->update_z_controller();
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), auto_yaw.rate_cds());
        return;
    }

    // check if we are not navigating because of low altitude
    if (auto_takeoff_no_nav_active) {
        // check if vehicle has reached no_nav_alt threshold
        if (inertial_nav.get_position_z_up_cm() >= auto_takeoff_no_nav_alt_cm) {
            auto_takeoff_no_nav_active = false;
        }
        pos_control->relax_velocity_controller_xy();
    } else {
        Vector2f vel;
        Vector2f accel;
        pos_control->input_vel_accel_xy(vel, accel);
    }
    pos_control->update_xy_controller();

    // command the aircraft to the take off altitude
    float pos_z = auto_takeoff_complete_alt_cm + terr_offset;
    float vel_z = 0.0;
    copter.pos_control->input_pos_vel_accel_z(pos_z, vel_z, 0.0);
    
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from position controller, yaw rate from pilot
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from position controller, yaw rate from mavlink command or mission item
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), auto_yaw.rate_cds());
    } else {
        // roll & pitch from position controller, yaw heading from GCS or auto_heading()
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.yaw(), auto_yaw.rate_cds());
    }

    // handle takeoff completion
    bool reached_altitude = (copter.pos_control->get_pos_target_z_cm() - auto_takeoff_start_alt_cm) >= ((auto_takeoff_complete_alt_cm + terr_offset - auto_takeoff_start_alt_cm) * 0.90);
    bool reached_climb_rate = copter.pos_control->get_vel_desired_cms().z < copter.pos_control->get_max_speed_up_cms() * 0.1;
    auto_takeoff_complete = reached_altitude && reached_climb_rate;

    // calculate completion for location in case it is needed for a smooth transition to wp_nav
    if (auto_takeoff_complete) {
        const Vector3p& complete_pos = copter.pos_control->get_pos_target_cm();
        auto_takeoff_complete_pos = Vector3p{complete_pos.x, complete_pos.y, pos_z};
    }
}

void Mode::auto_takeoff_start(float complete_alt_cm, bool terrain_alt)
{
    auto_takeoff_start_alt_cm = inertial_nav.get_position_z_up_cm();
    auto_takeoff_complete_alt_cm = complete_alt_cm;
    auto_takeoff_terrain_alt = terrain_alt;
    auto_takeoff_complete = false;
    if ((g2.wp_navalt_min > 0) && (is_disarmed_or_landed() || !motors->get_interlock())) {
        // we are not flying, climb with no navigation to current alt-above-ekf-origin + wp_navalt_min
        auto_takeoff_no_nav_alt_cm = auto_takeoff_start_alt_cm + g2.wp_navalt_min * 100;
        auto_takeoff_no_nav_active = true;
    } else {
        auto_takeoff_no_nav_active = false;
    }
}

// return takeoff final position if takeoff has completed successfully
bool Mode::auto_takeoff_get_position(Vector3p& complete_pos)
{
    // only provide location if takeoff has completed
    if (!auto_takeoff_complete) {
        return false;
    }

    complete_pos = auto_takeoff_complete_pos;
    return true;
}

bool Mode::is_taking_off() const
{
    if (!has_user_takeoff(false)) {
        return false;
    }
    if (copter.ap.land_complete) {
        return false;
    }
    return takeoff.running();
}
Terrain.cpp
#include "Copter.h"

// update terrain data
void Copter::terrain_update()
{
#if AP_TERRAIN_AVAILABLE
    terrain.update();

    // tell the rangefinder our height, so it can go into power saving
    // mode if available
#if RANGEFINDER_ENABLED == ENABLED
    float height;
    if (terrain.height_above_terrain(height, true)) {
        rangefinder.set_estimated_terrain_height(height);
    }
#endif
#endif
}

// log terrain data - should be called at 1hz
void Copter::terrain_logging()
{
#if AP_TERRAIN_AVAILABLE
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data();
    }
#endif
}
Toy_mode.cpp
#include "Copter.h"

#if TOY_MODE_ENABLED == ENABLED

// times in 0.1s units
#define TOY_COMMAND_DELAY 15
#define TOY_LONG_PRESS_COUNT 15
#define TOY_LAND_MANUAL_DISARM_COUNT 40
#define TOY_LAND_DISARM_COUNT 1
#define TOY_LAND_ARM_COUNT 1
#define TOY_RIGHT_PRESS_COUNT 1
#define TOY_ACTION_DELAY_MS 200
#define TOY_DESCENT_SLOW_HEIGHT 5
#define TOY_DESCENT_SLOW_RAMP 3
#define TOY_DESCENT_SLOW_MIN 300
#define TOY_RESET_TURTLE_TIME 5000

#define ENABLE_LOAD_TEST 0

const AP_Param::GroupInfo ToyMode::var_info[] = {

    // @Param: _ENABLE
    // @DisplayName: tmode enable 
    // @Description: tmode (or "toy" mode) gives a simplified user interface designed for mass market drones. Version1 is for the SkyViper V2450GPS. Version2 is for the F412 based boards
    // @Values: 0:Disabled,1:EnableVersion1,2:EnableVersion2
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLE", 1, ToyMode, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _MODE1
    // @DisplayName: Tmode first mode
    // @Description: This is the initial mode when the vehicle is first turned on. This mode is assumed to not require GPS
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:FlowHold
    // @User: Standard
    AP_GROUPINFO("_MODE1", 2, ToyMode, primary_mode[0], (float)Mode::Number::ALT_HOLD),

    // @Param: _MODE2
    // @DisplayName: Tmode second mode
    // @Description: This is the secondary mode. This mode is assumed to require GPS
    // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:FlowHold
    // @User: Standard
    AP_GROUPINFO("_MODE2", 3, ToyMode, primary_mode[1], (float)Mode::Number::LOITER),

    // @Param: _ACTION1
    // @DisplayName: Tmode action 1
    // @Description: This is the action taken when the left action button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION1", 4, ToyMode, actions[0], ACTION_TOGGLE_VIDEO),

    // @Param: _ACTION2
    // @DisplayName: Tmode action 2
    // @Description: This is the action taken when the right action button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION2", 5, ToyMode, actions[1], ACTION_TAKE_PHOTO),

    // @Param: _ACTION3
    // @DisplayName: Tmode action 3
    // @Description: This is the action taken when the power button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION3", 6, ToyMode, actions[2], ACTION_DISARM),

    // @Param: _ACTION4
    // @DisplayName: Tmode action 4
    // @Description: This is the action taken when the left action button is pressed while the left (Mode) button is held down
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION4", 7, ToyMode, actions[3], ACTION_NONE),

    // @Param: _ACTION5
    // @DisplayName: Tmode action 5
    // @Description: This is the action taken when the right action is pressed while the left (Mode) button is held down
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION5", 8, ToyMode, actions[4], ACTION_NONE),

    // @Param: _ACTION6
    // @DisplayName: Tmode action 6
    // @Description: This is the action taken when the power button is pressed while the left (Mode) button is held down
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_ACTION6", 9, ToyMode, actions[5], ACTION_NONE),

    // @Param: _LEFT
    // @DisplayName: Tmode left action
    // @Description: This is the action taken when the left (Mode) button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_LEFT", 10, ToyMode, actions[6], ACTION_TOGGLE_MODE),

    // @Param: _LEFT_LONG
    // @DisplayName: Tmode left long action
    // @Description: This is the action taken when the left (Mode) button is long-pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest,24:ModeFlowHold
    // @User: Standard
    AP_GROUPINFO("_LEFT_LONG", 11, ToyMode, actions[7], ACTION_NONE),

    // @Param: _TRIM_AUTO
    // @DisplayName: Stick auto trim limit
    // @Description: This is the amount of automatic stick trim that can be applied when disarmed with sticks not moving. It is a PWM limit value away from 1500
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("_TRIM_AUTO", 12, ToyMode, trim_auto, 50),

    // @Param: _RIGHT
    // @DisplayName: Tmode right action
    // @Description: This is the action taken when the right (Return) button is pressed
    // @Values: 0:None,1:TakePhoto,2:ToggleVideo,3:ModeAcro,4:ModeAltHold,5:ModeAuto,6:ModeLoiter,7:ModeRTL,8:ModeCircle,9:ModeLand,10:ModeDrift,11:ModeSport,12:ModeAutoTune,13:ModePosHold,14:ModeBrake,15:ModeThrow,16:Flip,17:ModeStabilize,18:Disarm,19:ToggleMode,20:Arm-Land-RTL,21:ToggleSimpleMode,22:ToggleSuperSimpleMode,23:MotorLoadTest
    // @User: Standard
    AP_GROUPINFO("_RIGHT", 13, ToyMode, actions[8], ACTION_ARM_LAND_RTL),

    // @Param: _FLAGS
    // @DisplayName: Tmode flags
    // @Description: Bitmask of flags to change the behaviour of tmode. DisarmOnLowThrottle means to disarm if throttle is held down for 1 second when landed. ArmOnHighThrottle means to arm if throttle is above 80% for 1 second. UpgradeToLoiter means to allow takeoff in LOITER mode by switching to ALT_HOLD, then auto-upgrading to LOITER once GPS is available. RTLStickCancel means that on large stick inputs in RTL mode that LOITER mode is engaged
    // @Bitmask: 0:DisarmOnLowThrottle,1:ArmOnHighThrottle,2:UpgradeToLoiter,3:RTLStickCancel
    // @User: Standard
    AP_GROUPINFO("_FLAGS", 14, ToyMode, flags, FLAG_THR_DISARM),

    // @Param: _VMIN
    // @DisplayName: Min voltage for output limiting
    // @Description: This is the battery voltage below which no output limiting is done
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_VMIN", 15, ToyMode, filter.volt_min, 3.5),

    // @Param: _VMAX
    // @DisplayName: Max voltage for output limiting
    // @Description: This is the battery voltage above which thrust min is used
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_VMAX", 16, ToyMode, filter.volt_max, 3.8),
    
    // @Param: _TMIN
    // @DisplayName: Min thrust multiplier
    // @Description: This sets the thrust multiplier when voltage is high
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_TMIN", 17, ToyMode, filter.thrust_min, 1.0),

    // @Param: _TMAX
    // @DisplayName: Max thrust multiplier
    // @Description: This sets the thrust multiplier when voltage is low
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_TMAX", 18, ToyMode, filter.thrust_max, 1.0),

#if ENABLE_LOAD_TEST
    // @Param: _LOAD_MUL
    // @DisplayName: Load test multiplier
    // @Description: This scales the load test output, as a value between 0 and 1
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("_LOAD_MUL", 19, ToyMode, load_test.load_mul, 1.0),
    
    // @Param: _LOAD_FILT
    // @DisplayName: Load test filter
    // @Description: This filters the load test output. A value of 1 means no filter. 2 means values are repeated once. 3 means values are repeated 3 times, etc
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("_LOAD_FILT", 20, ToyMode, load_test.load_filter, 1),
    
    // @Param: _LOAD_TYPE
    // @DisplayName: Load test type
    // @Description: This sets the type of load test
    // @Values: 0:ConstantThrust,1:LogReplay1,2:LogReplay2
    // @User: Advanced
    AP_GROUPINFO("_LOAD_TYPE", 21, ToyMode, load_test.load_type, LOAD_TYPE_LOG1),
#endif
   
    AP_GROUPEND
};

ToyMode::ToyMode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  special mode handling for toys
 */
void ToyMode::update()
{
    if (!enable) {
        // not enabled
        return;
    }

#if ENABLE_LOAD_TEST
    if (!copter.motors->armed()) {
        load_test.running = false;
    }
#endif

    // keep filtered battery voltage for thrust limiting
    filtered_voltage = 0.99 * filtered_voltage + 0.01 * copter.battery.voltage();
    
    // update LEDs
    blink_update();
    
    if (!done_first_update) {
        done_first_update = true;
        copter.set_mode(Mode::Number(primary_mode[0].get()), ModeReason::TOY_MODE);
        copter.motors->set_thrust_compensation_callback(FUNCTOR_BIND_MEMBER(&ToyMode::thrust_limiting, void, float *, uint8_t));
    }

    // check if we should auto-trim
    if (trim_auto > 0) {
        trim_update();
    }
            
    // set ALT_HOLD as indoors for the EKF (disables GPS vertical velocity fusion)
#if 0
    copter.ahrs.set_indoor_mode(copter.flightmode->mode_number() == ALT_HOLD || copter.flightmode->mode_number() == FLOWHOLD);
#endif
    
    bool left_button = false;
    bool right_button = false;
    bool left_action_button = false;
    bool right_action_button = false;
    bool power_button = false;
    bool left_change = false;
    
    uint16_t ch5_in = RC_Channels::get_radio_in(CH_5);
    uint16_t ch6_in = RC_Channels::get_radio_in(CH_6);
    uint16_t ch7_in = RC_Channels::get_radio_in(CH_7);

    if (copter.failsafe.radio || ch5_in < 900) {
        // failsafe handling is outside the scope of toy mode, it does
        // normal failsafe actions, just setup a blink pattern
        green_blink_pattern = BLINK_NO_RX;
        red_blink_pattern = BLINK_NO_RX;
        red_blink_index = green_blink_index;
        return;
    }

    uint32_t now = AP_HAL::millis();
    
    if (is_v2450_buttons()) {
        // V2450 button mapping from cypress radio. It maps the
        // buttons onto channels 5, 6 and 7 in a complex way, with the
        // left button latching
        left_change = ((ch5_in > 1700 && last_ch5 <= 1700) || (ch5_in <= 1700 && last_ch5 > 1700));
        
        last_ch5 = ch5_in;
                        
        // get buttons from channels
        left_button = (ch5_in > 2050 || (ch5_in > 1050 && ch5_in < 1150));
        right_button = (ch6_in > 1500);
        uint8_t ch7_bits = (ch7_in>1000)?uint8_t((ch7_in-1000)/100):0;
        left_action_button = (ch7_bits&1) != 0;
        right_action_button = (ch7_bits&2) != 0;
        power_button = (ch7_bits&4) != 0;
    } else if (is_f412_buttons()) {
        // F412 button setup for cc2500 radio. This maps the 6 buttons
        // onto channels 5 and 6, with no latching
        uint8_t ch5_bits = (ch5_in>1000)?uint8_t((ch5_in-1000)/100):0;
        uint8_t ch6_bits = (ch6_in>1000)?uint8_t((ch6_in-1000)/100):0;
        left_button = (ch5_bits & 4) != 0;
        right_button = (ch5_bits & 2) != 0;
        right_action_button = (ch6_bits & 1) != 0;
        left_action_button = (ch6_bits & 2) != 0;
        power_button = (ch6_bits & 4) != 0;
        left_change = (left_button != last_left_button);
        last_left_button = left_button;
    }
    
    // decode action buttons into an action
    uint8_t action_input = 0;    
    if (left_action_button) {
        action_input = 1;
    } else if (right_action_button) {
        action_input = 2;
    } else if (power_button) {
        action_input = 3;
    }
    
    if (action_input != 0 && left_button) {
        // combined button actions
        action_input += 3;
        left_press_counter = 0;
    } else if (left_button) {
        left_press_counter++;
    } else {
        left_press_counter = 0;
    }

    bool reset_combination = left_action_button && right_action_button;
    if (reset_combination && abs(copter.ahrs.roll_sensor) > 160) {
        /*
          if both shoulder buttons are pressed at the same time for 5
          seconds while the vehicle is inverted then we send a
          WIFIRESET message to the sonix to reset SSID and password
        */
        if (reset_turtle_start_ms == 0) {
            reset_turtle_start_ms = now;
        }
        if (now - reset_turtle_start_ms > TOY_RESET_TURTLE_TIME) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: WiFi reset");
            reset_turtle_start_ms = 0;
            send_named_int("WIFIRESET", 1);
        }
    } else {
        reset_turtle_start_ms = 0;
    }
    if (reset_combination) {
        // don't act on buttons when combination pressed
        action_input = 0;
        left_press_counter = 0;
    }

    /*
      work out commanded action, if any
     */
    enum toy_action action = action_input?toy_action(actions[action_input-1].get()):ACTION_NONE;
   
    // check for long left button press
    if (action == ACTION_NONE && left_press_counter > TOY_LONG_PRESS_COUNT) {
        left_press_counter = -TOY_COMMAND_DELAY;
        action = toy_action(actions[7].get());
        ignore_left_change = true;
    }

    // cope with long left press triggering a left change
    if (ignore_left_change && left_change) {
        left_change = false;
        ignore_left_change = false;
    }

    if (is_v2450_buttons()) {
        // check for left button latching change
        if (action == ACTION_NONE && left_change) {
            action = toy_action(actions[6].get());
        }
    } else if (is_f412_buttons()) {
        if (action == ACTION_NONE && left_change && !left_button) {
            // left release
            action = toy_action(actions[6].get());
        }

    }

    // check for right button
    if (action == ACTION_NONE && right_button) {
        right_press_counter++;
        if (right_press_counter >= TOY_RIGHT_PRESS_COUNT) {
            action = toy_action(actions[8].get());
            right_press_counter = -TOY_COMMAND_DELAY;
        }
    } else {
        right_press_counter = 0;
    }

    /*
      some actions shouldn't repeat too fast
     */
    switch (action) {
    case ACTION_TOGGLE_VIDEO:
    case ACTION_TOGGLE_MODE:
    case ACTION_TOGGLE_SIMPLE:
    case ACTION_TOGGLE_SSIMPLE:
    case ACTION_ARM_LAND_RTL:
    case ACTION_LOAD_TEST:
    case ACTION_MODE_FLOW:
        if (last_action == action ||
            now - last_action_ms < TOY_ACTION_DELAY_MS) {
            // for the above actions, button must be released before
            // it will activate again
            last_action = action;
            action = ACTION_NONE;
        }
        break;
        
    case ACTION_TAKE_PHOTO:
        // allow photo continuous shooting
        if (now - last_action_ms < TOY_ACTION_DELAY_MS) {
            last_action = action;
            action = ACTION_NONE;
        }
        break;

    default:
        last_action = action;
        break;
    }
    
    if (action != ACTION_NONE) {
        gcs().send_text(MAV_SEVERITY_INFO, "Tmode: action %u", action);
        last_action_ms = now;
    }

    // we use 150 for throttle_at_min to cope with varying stick throws
    bool throttle_at_min =
        copter.channel_throttle->get_control_in() < 150;

    // throttle threshold for throttle arming
    bool throttle_near_max =
        copter.channel_throttle->get_control_in() > 700;
    
    /*
      disarm if throttle is low for 1 second when landed
     */
    if ((flags & FLAG_THR_DISARM) && throttle_at_min && copter.motors->armed() && copter.ap.land_complete) {
        throttle_low_counter++;
        const uint8_t disarm_limit = copter.flightmode->has_manual_throttle()?TOY_LAND_MANUAL_DISARM_COUNT:TOY_LAND_DISARM_COUNT;
        if (throttle_low_counter >= disarm_limit) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: throttle disarm");
            copter.arming.disarm(AP_Arming::Method::TOYMODELANDTHROTTLE);
        }
    } else {
        throttle_low_counter = 0;
    }

    /*
      arm if throttle is high for 1 second when landed
     */
    if ((flags & FLAG_THR_ARM) && throttle_near_max && !copter.motors->armed()) {
        throttle_high_counter++;
        if (throttle_high_counter >= TOY_LAND_ARM_COUNT) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: throttle arm");
            arm_check_compass();
            if (!copter.arming.arm(AP_Arming::Method::MAVLINK) && (flags & FLAG_UPGRADE_LOITER) && copter.flightmode->mode_number() == Mode::Number::LOITER) {
                /*
                  support auto-switching to ALT_HOLD, then upgrade to LOITER once GPS available
                 */
                if (set_and_remember_mode(Mode::Number::ALT_HOLD, ModeReason::TOY_MODE)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Tmode: ALT_HOLD update arm");
#if AC_FENCE == ENABLED
                    copter.fence.enable(false);
#endif
                    if (!copter.arming.arm(AP_Arming::Method::MAVLINK)) {
                        // go back to LOITER
                        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: ALT_HOLD arm failed");
                        set_and_remember_mode(Mode::Number::LOITER, ModeReason::TOY_MODE);
                    } else {
                        upgrade_to_loiter = true;
#if 0
                        AP_Notify::flags.hybrid_loiter = true;
#endif
                    }
                }
            } else {
                throttle_arm_ms = AP_HAL::millis();
            }
        }
    } else {
        throttle_high_counter = 0;
    }

    if (upgrade_to_loiter) {
        if (!copter.motors->armed() || copter.flightmode->mode_number() != Mode::Number::ALT_HOLD) {
            upgrade_to_loiter = false;
#if 0
            AP_Notify::flags.hybrid_loiter = false;
#endif
        } else if (copter.position_ok() && set_and_remember_mode(Mode::Number::LOITER, ModeReason::TOY_MODE)) {
#if AC_FENCE == ENABLED
            copter.fence.enable(true);
#endif
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: LOITER update");            
        }
    }

    if (copter.flightmode->mode_number() == Mode::Number::RTL && (flags & FLAG_RTL_CANCEL) && throttle_near_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Tmode: RTL cancel");        
        set_and_remember_mode(Mode::Number::LOITER, ModeReason::TOY_MODE);
    }
    
    enum Mode::Number old_mode = copter.flightmode->mode_number();
    enum Mode::Number new_mode = old_mode;

    /*
      implement actions
     */
    switch (action) {
    case ACTION_NONE:
        break;

    case ACTION_TAKE_PHOTO:
        send_named_int("SNAPSHOT", 1);
        break;

    case ACTION_TOGGLE_VIDEO:
        send_named_int("VIDEOTOG", 1);
        break;

    case ACTION_MODE_ACRO:
#if MODE_ACRO_ENABLED == ENABLED
        new_mode = Mode::Number::ACRO;
#else
        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: ACRO is disabled");
#endif
        break;

    case ACTION_MODE_ALTHOLD:
        new_mode = Mode::Number::ALT_HOLD;
        break;

    case ACTION_MODE_AUTO:
        new_mode = Mode::Number::AUTO;
        break;

    case ACTION_MODE_LOITER:
        new_mode = Mode::Number::LOITER;
        break;

    case ACTION_MODE_RTL:
        new_mode = Mode::Number::RTL;
        break;

    case ACTION_MODE_CIRCLE:
        new_mode = Mode::Number::CIRCLE;
        break;

    case ACTION_MODE_LAND:
        new_mode = Mode::Number::LAND;
        break;

    case ACTION_MODE_DRIFT:
        new_mode = Mode::Number::DRIFT;
        break;

    case ACTION_MODE_SPORT:
        new_mode = Mode::Number::SPORT;
        break;

    case ACTION_MODE_AUTOTUNE:
        new_mode = Mode::Number::AUTOTUNE;
        break;

    case ACTION_MODE_POSHOLD:
        new_mode = Mode::Number::POSHOLD;
        break;

    case ACTION_MODE_BRAKE:
        new_mode = Mode::Number::BRAKE;
        break;

    case ACTION_MODE_THROW:
#if MODE_THROW_ENABLED == ENABLED
        new_mode = Mode::Number::THROW;
#else
        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: THROW is disabled");
#endif
        break;

    case ACTION_MODE_FLIP:
        new_mode = Mode::Number::FLIP;
        break;

    case ACTION_MODE_STAB:
        new_mode = Mode::Number::STABILIZE;
        break;

    case ACTION_MODE_FLOW:
        // toggle flow hold
        if (old_mode != Mode::Number::FLOWHOLD) {
            new_mode = Mode::Number::FLOWHOLD;
        } else {
            new_mode = Mode::Number::ALT_HOLD;
        }
        break;
        
    case ACTION_DISARM:
        if (copter.motors->armed()) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: Force disarm");
            copter.arming.disarm(AP_Arming::Method::TOYMODELANDFORCE);
        }
        break;

    case ACTION_TOGGLE_MODE:
        last_mode_choice = (last_mode_choice+1) % 2;
        new_mode = Mode::Number(primary_mode[last_mode_choice].get());
        break;

    case ACTION_TOGGLE_SIMPLE:
        copter.set_simple_mode(bool(copter.simple_mode)?Copter::SimpleMode::NONE:Copter::SimpleMode::SIMPLE);
        break;

    case ACTION_TOGGLE_SSIMPLE:
        copter.set_simple_mode(bool(copter.simple_mode)?Copter::SimpleMode::NONE:Copter::SimpleMode::SUPERSIMPLE);
        break;
        
    case ACTION_ARM_LAND_RTL:
        if (!copter.motors->armed()) {
            action_arm();
        } else if (old_mode == Mode::Number::RTL) {
            // switch between RTL and LOITER when in GPS modes
            new_mode = Mode::Number::LOITER;
        } else if (old_mode == Mode::Number::LAND) {
            if (last_set_mode == Mode::Number::LAND || !copter.position_ok()) {
                // this is a land that we asked for, or we don't have good positioning
                new_mode = Mode::Number::ALT_HOLD;
            } else if (copter.landing_with_GPS()) {
                new_mode = Mode::Number::LOITER;
            } else {
                new_mode = Mode::Number::ALT_HOLD;
            }
        } else if (copter.flightmode->requires_GPS()) {
            // if we're in a GPS mode, then RTL
            new_mode = Mode::Number::RTL;
        } else {
            // if we're in a non-GPS mode, then LAND
            new_mode = Mode::Number::LAND;
        }
        break;

    case ACTION_LOAD_TEST:
#if ENABLE_LOAD_TEST
        if (copter.motors->armed() && !load_test.running) {
            break;
        }
        if (load_test.running) {
            load_test.running = false;
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: load_test off");
            copter.init_disarm_motors();
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::TOY_MODE);
        } else {
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::TOY_MODE);
#if AC_FENCE == ENABLED
            copter.fence.enable(false);
#endif
            if (copter.arming.arm(AP_Arming::Method::MAVLINK)) {
                load_test.running = true;
                gcs().send_text(MAV_SEVERITY_INFO, "Tmode: load_test on");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Tmode: load_test failed");
            }
        }
#endif
        break;
    }

    if (!copter.motors->armed() && (copter.flightmode->mode_number() == Mode::Number::LAND || copter.flightmode->mode_number() == Mode::Number::RTL)) {
        // revert back to last primary flight mode if disarmed after landing
        new_mode = Mode::Number(primary_mode[last_mode_choice].get());
    }
    
    if (new_mode != copter.flightmode->mode_number()) {
        load_test.running = false;
#if AC_FENCE == ENABLED
        copter.fence.enable(false);
#endif
        if (set_and_remember_mode(new_mode, ModeReason::TOY_MODE)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: mode %s", copter.flightmode->name4());
            // force fence on in all GPS flight modes
#if AC_FENCE == ENABLED
            if (copter.flightmode->requires_GPS()) {
                copter.fence.enable(true);
            }
#endif
        } else {
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: %u FAILED", (unsigned)new_mode);
            if (new_mode == Mode::Number::RTL) {
                // if we can't RTL then land
                gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: LANDING");
                set_and_remember_mode(Mode::Number::LAND, ModeReason::TOY_MODE);
#if AC_FENCE == ENABLED
                if (copter.landing_with_GPS()) {
                    copter.fence.enable(true);
                }
#endif
            }
        }
    }
}


/*
  set a mode, remembering what mode we set, and the previous mode we were in
 */
bool ToyMode::set_and_remember_mode(Mode::Number mode, ModeReason reason)
{
    if (copter.flightmode->mode_number() == mode) {
        return true;
    }
    if (!copter.set_mode(mode, reason)) {
        return false;
    }
    last_set_mode = mode;
    return true;
}

/*
  automatic stick trimming. This works while disarmed by looking for
  zero rc-input changes for 4 seconds, and assuming sticks are
  centered. Trim is saved
 */
void ToyMode::trim_update(void)
{
    if (hal.util->get_soft_armed() || copter.failsafe.radio) {
        // only when disarmed and with RC link
        trim.start_ms = 0;
        return;
    }

    // get throttle mid from channel trim
    uint16_t throttle_trim = copter.channel_throttle->get_radio_trim();
    if (abs(throttle_trim - 1500) <= trim_auto) {
        RC_Channel *c = copter.channel_throttle;
        uint16_t ch_min = c->get_radio_min();
        uint16_t ch_max = c->get_radio_max();
        // remember the throttle midpoint
        int16_t new_value = 1000UL * (throttle_trim - ch_min) / (ch_max - ch_min);
        if (new_value != throttle_mid) {
            throttle_mid = new_value;
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: thr mid %d",
                                             throttle_mid);
        }
    }
    
    uint16_t chan[4];
    if (rc().get_radio_in(chan, 4) != 4) {
        trim.start_ms = 0;
        return;
    }

    const uint16_t noise_limit = 2;
    for (uint8_t i=0; i<4; i++) {
        if (abs(chan[i] - 1500) > trim_auto) {
            // not within limit
            trim.start_ms = 0;
            return;
        }
    }

    uint32_t now = AP_HAL::millis();
    
    if (trim.start_ms == 0) {
        // start timer
        memcpy(trim.chan, chan, 4*sizeof(uint16_t));
        trim.start_ms = now;
        return;
    }

    
    for (uint8_t i=0; i<4; i++) {
        if (abs(trim.chan[i] - chan[i]) > noise_limit) {
            // detected stick movement
            memcpy(trim.chan, chan, 4*sizeof(uint16_t));
            trim.start_ms = now;
            return;
        }
    }

    if (now - trim.start_ms < 4000) {
        // not steady for long enough yet
        return;
    }

    // reset timer so we don't trigger too often
    trim.start_ms = 0;
    
    uint8_t need_trim = 0;
    for (uint8_t i=0; i<4; i++) {
        RC_Channel *c = RC_Channels::rc_channel(i);
        if (c && abs(chan[i] - c->get_radio_trim()) > noise_limit) {
            need_trim |= 1U<<i;
        }
    }
    if (need_trim == 0) {
        return;
    }
    for (uint8_t i=0; i<4; i++) {
        if (need_trim & (1U<<i)) {
            RC_Channel *c = RC_Channels::rc_channel(i);
            c->set_and_save_radio_trim(chan[i]);
        }
    }

    gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: trim %u %u %u %u",
                                     chan[0], chan[1], chan[2], chan[3]);
}

/*
  handle arming action
 */
void ToyMode::action_arm(void)
{
    bool needs_gps = copter.flightmode->requires_GPS();

    // don't arm if sticks aren't in deadzone, to prevent pot problems
    // on TX causing flight control issues
    bool sticks_centered =
        copter.channel_roll->get_control_in() == 0 &&
        copter.channel_pitch->get_control_in() == 0 &&
        copter.channel_yaw->get_control_in() == 0;

    if (!sticks_centered) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: sticks not centered");
        return;
    }

    arm_check_compass();
    
    if (needs_gps && copter.arming.gps_checks(false)) {
#if AC_FENCE == ENABLED
        // we want GPS and checks are passing, arm and enable fence
        copter.fence.enable(true);
#endif
        copter.arming.arm(AP_Arming::Method::RUDDER);
        if (!copter.motors->armed()) {
            AP_Notify::events.arming_failed = true;
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: GPS arming failed");
        } else {
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: GPS armed motors");
        }
    } else if (needs_gps) {
        // notify of arming fail
        AP_Notify::events.arming_failed = true;
        gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: GPS arming failed");
    } else {
#if AC_FENCE == ENABLED
        // non-GPS mode
        copter.fence.enable(false);
#endif
        copter.arming.arm(AP_Arming::Method::RUDDER);
        if (!copter.motors->armed()) {
            AP_Notify::events.arming_failed = true;
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: non-GPS arming failed");
        } else {
            gcs().send_text(MAV_SEVERITY_ERROR, "Tmode: non-GPS armed motors");
        }
    }
}

/*
  adjust throttle for throttle takeoff
  This prevents sudden climbs when using throttle for arming
*/
void ToyMode::throttle_adjust(float &throttle_control)
{
    uint32_t now = AP_HAL::millis();
    const uint32_t soft_start_ms = 5000;
    const uint16_t throttle_start = 600 + copter.g.throttle_deadzone;
    if (!copter.motors->armed() && (flags & FLAG_THR_ARM)) {
        throttle_control = MIN(throttle_control, 500);
    } else if (now - throttle_arm_ms < soft_start_ms) {
        float p = (now - throttle_arm_ms) / float(soft_start_ms);
        throttle_control = MIN(throttle_control, throttle_start + p * (1000 - throttle_start));
    }

    // limit descent rate close to the ground
    float height = copter.inertial_nav.get_position_z_up_cm() * 0.01 - copter.arming_altitude_m;
    if (throttle_control < 500 &&
        height < TOY_DESCENT_SLOW_HEIGHT + TOY_DESCENT_SLOW_RAMP &&
        copter.motors->armed() && !copter.ap.land_complete) {
        float limit = linear_interpolate(TOY_DESCENT_SLOW_MIN, 0, height,
                                         TOY_DESCENT_SLOW_HEIGHT, TOY_DESCENT_SLOW_HEIGHT+TOY_DESCENT_SLOW_RAMP);
        if (throttle_control < limit) {
            // limit descent rate close to the ground
            throttle_control = limit;
        }
    }
}


/*
  update blinking. Blinking is done with a 16 bit pattern for each
  LED. A count can be set for a pattern, which makes the pattern
  persist until the count is zero. When it is zero the normal pattern
  settings based on system status are used
 */
void ToyMode::blink_update(void)
{
    if (red_blink_pattern & (1U<<red_blink_index)) {
        copter.relay.on(1);
    } else {
        copter.relay.off(1);
    }
    if (green_blink_pattern & (1U<<green_blink_index)) {
        copter.relay.on(0);
    } else {
        copter.relay.off(0);
    }
    green_blink_index = (green_blink_index+1) % 16;
    red_blink_index = (red_blink_index+1) % 16;
    if (green_blink_index == 0 && green_blink_count > 0) {
        green_blink_count--;
    }
    if (red_blink_index == 0 && red_blink_count > 0) {
        red_blink_count--;
    }

    // let the TX know we are recording video
    uint32_t now = AP_HAL::millis();
    if (now - last_video_ms < 1000) {
        AP_Notify::flags.video_recording = 1;
    } else {
        AP_Notify::flags.video_recording = 0;
    }
    
    if (red_blink_count > 0 && green_blink_count > 0) {
        return;
    }
    
    // setup normal patterns based on flight mode and arming
    uint16_t pattern = 0;

    // full on when we can see the TX, except for battery failsafe,
    // when we blink rapidly
    if (copter.motors->armed() && AP_Notify::flags.failsafe_battery) {
        pattern = BLINK_8;
    } else if (!copter.motors->armed() && (blink_disarm > 0)) {
        pattern = BLINK_8;
        blink_disarm--;
    } else {
        pattern = BLINK_FULL;
    }
    
    if (copter.motors->armed()) {
        blink_disarm = 4;
    }
    
    if (red_blink_count == 0) {
        red_blink_pattern = pattern;
    }
    if (green_blink_count == 0) {
        green_blink_pattern = pattern;
    }
    if (red_blink_count == 0 && green_blink_count == 0) {
        // get LEDs in sync
        red_blink_index = green_blink_index;
    }
}

// handle a mavlink message
void ToyMode::handle_message(const mavlink_message_t &msg)
{
    if (msg.msgid != MAVLINK_MSG_ID_NAMED_VALUE_INT) {
        return;
    }
    mavlink_named_value_int_t m;
    mavlink_msg_named_value_int_decode(&msg, &m);
    if (strncmp(m.name, "BLINKR", 10) == 0) {
        red_blink_pattern = (uint16_t)m.value;
        red_blink_count = m.value >> 16;
        red_blink_index = 0;
    } else if (strncmp(m.name, "BLINKG", 10) == 0) {
        green_blink_pattern = (uint16_t)m.value;
        green_blink_count = m.value >> 16;
        green_blink_index = 0;
    } else if (strncmp(m.name, "VNOTIFY", 10) == 0) {
        // taking photos or video
        if (green_blink_pattern != BLINK_2) {
            green_blink_index = 0;
        }
        green_blink_pattern = BLINK_2;
        green_blink_count = 1;
        last_video_ms = AP_HAL::millis();
        // immediately update AP_Notify recording flag
        AP_Notify::flags.video_recording = 1;
    } else if (strncmp(m.name, "WIFICHAN", 10) == 0) {
#if HAL_RCINPUT_WITH_AP_RADIO
        AP_Radio *radio = AP_Radio::get_singleton();
        if (radio) {
            radio->set_wifi_channel(m.value);
        }
#endif
    } else if (strncmp(m.name, "LOGDISARM", 10) == 0) {
        enum ap_var_type vtype;
        AP_Int8 *log_disarmed = (AP_Int8 *)AP_Param::find("LOG_DISARMED", &vtype);
        if (log_disarmed) {
            log_disarmed->set(int8_t(m.value));
        }
    }
}

/*
  send a named int to primary telem channel
 */
void ToyMode::send_named_int(const char *name, int32_t value)
{
    mavlink_msg_named_value_int_send(MAVLINK_COMM_1, AP_HAL::millis(), name, value);
}

/*
  limit maximum thrust based on voltage
 */
void ToyMode::thrust_limiting(float *thrust, uint8_t num_motors)
{
    float thrust_mul = linear_interpolate(filter.thrust_max, filter.thrust_min, filtered_voltage, filter.volt_min, filter.volt_max);
    for (uint8_t i=0; i<num_motors; i++) {
        thrust[i] *= thrust_mul;
    }
    uint16_t pwm[4];
    hal.rcout->read(pwm, 4);

// @LoggerMessage: THST
// @Description: Maximum thrust limitation based on battery voltage in Toy Mode
// @Field: TimeUS: Time since system startup
// @Field: Vol: Filtered battery voltage
// @Field: Mul: Thrust multiplier between 0 and 1 to limit the output thrust based on battery voltage
// @Field: M1: Motor 1 pwm output
// @Field: M2: Motor 2 pwm output
// @Field: M3: Motor 3 pwm output
// @Field: M4: Motor 4 pwm output

    if (motor_log_counter++ % 10 == 0) {
        AP::logger().WriteStreaming("THST", "TimeUS,Vol,Mul,M1,M2,M3,M4", "QffHHHH",
                                               AP_HAL::micros64(),
                                               (double)filtered_voltage,
                                               (double)thrust_mul,
                                               pwm[0], pwm[1], pwm[2], pwm[3]);
    }
                                           
}

#if ENABLE_LOAD_TEST
/*
  run a motor load test - used for endurance checking in factory tests
 */
void ToyMode::load_test_run(void)
{
    uint16_t pwm[4] {};
    switch ((enum load_type)load_test.load_type.get()) {
    case LOAD_TYPE_LOG1:
        for (uint8_t i=0; i<4; i++) {
            pwm[i] = load_data1[load_test.row].m[i];
        }
        load_test.filter_counter++;
        if (load_test.filter_counter >= load_test.load_filter.get()) {
            load_test.filter_counter = 0;
            load_test.row = (load_test.row + 1) % ARRAY_SIZE(load_data1);
        }
        break;
        
    case LOAD_TYPE_LOG2:
        // like log1, but all the same
        for (uint8_t i=0; i<4; i++) {
            pwm[i] = load_data1[load_test.row].m[0];
        }
        load_test.filter_counter++;
        if (load_test.filter_counter >= load_test.load_filter.get()) {
            load_test.filter_counter = 0;
            load_test.row = (load_test.row + 1) % ARRAY_SIZE(load_data1);
        }
        break;

    case LOAD_TYPE_CONSTANT:
        for (uint8_t i=0; i<4; i++) {
            pwm[i] = 500;
        }
        break;
    default:
        return;
    }
    for (uint8_t i=0; i<4; i++) {
        pwm[i] *= load_test.load_mul;
        // write, with conversion to 1000 to 2000 range
        hal.rcout->write(i, 1000 + pwm[i]*2);
    }

    if (copter.failsafe.battery) {
        gcs().send_text(MAV_SEVERITY_INFO, "Tmode: load_test off (battery)");
        copter.init_disarm_motors();
        load_test.running = false;
    }    
}
#endif // ENABLE_LOAD_TEST

/*
  if we try to arm and the compass is out of range then we enable
  inflight compass learning
 */
void ToyMode::arm_check_compass(void)
{
    // check for unreasonable compass offsets
    Vector3f offsets = copter.compass.get_offsets();
    float field = copter.compass.get_field().length();
    
    char unused_compass_configured_error_message[20];
    if (offsets.length() > copter.compass.get_offsets_max() ||
        field < 200 || field > 800 ||
        !copter.compass.configured(unused_compass_configured_error_message, ARRAY_SIZE(unused_compass_configured_error_message))) {
        if (copter.compass.get_learn_type() != Compass::LEARN_INFLIGHT) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tmode: enable compass learning");
            copter.compass.set_learn_type(Compass::LEARN_INFLIGHT, false);
        }
    }
}

#endif // TOY_MODE_ENABLED
Tuning.cpp
#include "Copter.h"

/*
 * Function to update various parameters in flight using the ch6 tuning knob
 * This should not be confused with the AutoTune feature which can be found in control_autotune.cpp
 */

// tuning - updates parameters based on the ch6 tuning knob's position
//  should be called at 3.3hz
void Copter::tuning()
{
    const RC_Channel *rc6 = rc().channel(CH_6);

    // exit immediately if the tuning function is not set or min and max are both zero
    if ((g.radio_tuning <= 0) || (is_zero(g2.tuning_min.get()) && is_zero(g2.tuning_max.get()))) {
        return;
    }

    // exit immediately when radio failsafe is invoked or transmitter has not been turned on
    if (failsafe.radio || failsafe.radio_counter != 0 || rc6->get_radio_in() == 0) {
        return;
    }

    // exit immediately if a function is assigned to channel 6
    if ((RC_Channel::aux_func_t)rc6->option.get() != RC_Channel::AUX_FUNC::DO_NOTHING) {
        return;
    }

    const uint16_t radio_in = rc6->get_radio_in();
    float tuning_value = linear_interpolate(g2.tuning_min, g2.tuning_max, radio_in, rc6->get_radio_min(), rc6->get_radio_max());
    Log_Write_Parameter_Tuning(g.radio_tuning, tuning_value, g2.tuning_min, g2.tuning_max);

    switch(g.radio_tuning) {

    // Roll, Pitch tuning
    case TUNING_STABILIZE_ROLL_PITCH_KP:
        attitude_control->get_angle_roll_p().kP(tuning_value);
        attitude_control->get_angle_pitch_p().kP(tuning_value);
        break;

    case TUNING_RATE_ROLL_PITCH_KP:
        attitude_control->get_rate_roll_pid().kP(tuning_value);
        attitude_control->get_rate_pitch_pid().kP(tuning_value);
        break;

    case TUNING_RATE_ROLL_PITCH_KI:
        attitude_control->get_rate_roll_pid().kI(tuning_value);
        attitude_control->get_rate_pitch_pid().kI(tuning_value);
        break;

    case TUNING_RATE_ROLL_PITCH_KD:
        attitude_control->get_rate_roll_pid().kD(tuning_value);
        attitude_control->get_rate_pitch_pid().kD(tuning_value);
        break;

    // Yaw tuning
    case TUNING_STABILIZE_YAW_KP:
        attitude_control->get_angle_yaw_p().kP(tuning_value);
        break;

    case TUNING_YAW_RATE_KP:
        attitude_control->get_rate_yaw_pid().kP(tuning_value);
        break;

    case TUNING_YAW_RATE_KD:
        attitude_control->get_rate_yaw_pid().kD(tuning_value);
        break;

    // Altitude and throttle tuning
    case TUNING_ALTITUDE_HOLD_KP:
        pos_control->get_pos_z_p().kP(tuning_value);
        break;

    case TUNING_THROTTLE_RATE_KP:
        pos_control->get_vel_z_pid().kP(tuning_value);
        break;

    case TUNING_ACCEL_Z_KP:
        pos_control->get_accel_z_pid().kP(tuning_value);
        break;

    case TUNING_ACCEL_Z_KI:
        pos_control->get_accel_z_pid().kI(tuning_value);
        break;

    case TUNING_ACCEL_Z_KD:
        pos_control->get_accel_z_pid().kD(tuning_value);
        break;

    // Loiter and navigation tuning
    case TUNING_LOITER_POSITION_KP:
        pos_control->get_pos_xy_p().kP(tuning_value);
        break;

    case TUNING_VEL_XY_KP:
        pos_control->get_vel_xy_pid().kP(tuning_value);
        break;

    case TUNING_VEL_XY_KI:
        pos_control->get_vel_xy_pid().kI(tuning_value);
        break;

    case TUNING_WP_SPEED:
        wp_nav->set_speed_xy(tuning_value);
        break;

#if MODE_ACRO_ENABLED == ENABLED || MODE_SPORT_ENABLED == ENABLED
    // Acro roll pitch rates
    case TUNING_ACRO_RP_RATE:
        g2.acro_rp_rate = tuning_value;
        break;
#endif

#if MODE_ACRO_ENABLED == ENABLED || MODE_DRIFT_ENABLED == ENABLED
    // Acro yaw rate
    case TUNING_ACRO_YAW_RATE:
        g2.acro_y_rate = tuning_value;
        break;
#endif

#if FRAME_CONFIG == HELI_FRAME
    case TUNING_HELI_EXTERNAL_GYRO:
        motors->ext_gyro_gain(tuning_value);
        break;

    case TUNING_RATE_PITCH_FF:
        attitude_control->get_rate_pitch_pid().ff(tuning_value);
        break;

    case TUNING_RATE_ROLL_FF:
        attitude_control->get_rate_roll_pid().ff(tuning_value);
        break;

    case TUNING_RATE_YAW_FF:
        attitude_control->get_rate_yaw_pid().ff(tuning_value);
        break;
#endif

    case TUNING_DECLINATION:
        compass.set_declination(ToRad(tuning_value), false);     // 2nd parameter is false because we do not want to save to eeprom because this would have a performance impact
        break;

#if MODE_CIRCLE_ENABLED == ENABLED
    case TUNING_CIRCLE_RATE:
        circle_nav->set_rate(tuning_value);
        break;
#endif

    case TUNING_RC_FEEL_RP:
        attitude_control->set_input_tc(tuning_value);
        break;

    case TUNING_RATE_PITCH_KP:
        attitude_control->get_rate_pitch_pid().kP(tuning_value);
        break;

    case TUNING_RATE_PITCH_KI:
        attitude_control->get_rate_pitch_pid().kI(tuning_value);
        break;

    case TUNING_RATE_PITCH_KD:
        attitude_control->get_rate_pitch_pid().kD(tuning_value);
        break;

    case TUNING_RATE_ROLL_KP:
        attitude_control->get_rate_roll_pid().kP(tuning_value);
        break;

    case TUNING_RATE_ROLL_KI:
        attitude_control->get_rate_roll_pid().kI(tuning_value);
        break;

    case TUNING_RATE_ROLL_KD:
        attitude_control->get_rate_roll_pid().kD(tuning_value);
        break;

#if FRAME_CONFIG != HELI_FRAME
    case TUNING_RATE_MOT_YAW_HEADROOM:
        motors->set_yaw_headroom(tuning_value);
        break;
#endif

     case TUNING_RATE_YAW_FILT:
         attitude_control->get_rate_yaw_pid().filt_E_hz(tuning_value);
         break;

     case TUNING_SYSTEM_ID_MAGNITUDE:
#if MODE_SYSTEMID_ENABLED == ENABLED
         copter.mode_systemid.set_magnitude(tuning_value);
#endif
         break;
    }
}


