
#include "odrive_main.h"


Controller::Controller(ControllerConfig_t& config) :
    config_(config)
{}

void Controller::reset() {
    pos_setpoint_ = 0.0f;
    vel_setpoint_ = 0.0f;
    vel_integrator_current_ = 0.0f;
    current_setpoint_ = 0.0f;
}

//--------------------------------
// Command Handling
//--------------------------------

void Controller::set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward) {
    pos_setpoint_ = pos_setpoint;
    vel_setpoint_ = vel_feed_forward;
    current_setpoint_ = current_feed_forward;
    config_.control_mode = CTRL_MODE_POSITION_CONTROL;
#ifdef DEBUG_PRINT
    printf("POSITION_CONTROL %6.0f %3.3f %3.3f\n", pos_setpoint, vel_setpoint_, current_setpoint_);
#endif
}

void Controller::set_mixed_pos_setpoint(bool both,float sagittal, float frontal) {
    theta_s_desired_ = sagittal;
    theta_f_desired_ = frontal;
    config_.control_mode = CTRL_MODE_MIXED_IMPEDANCE_CONTROL;
    if(both) axis_->other_->controller_.set_mixed_pos_setpoint(false,sagittal, frontal);
}

void Controller::set_mixed_setpoint(bool both,float sagittal, float frontal, float sagittal_vel, float frontal_vel) {
    theta_s_desired_ = sagittal;
    theta_f_desired_ = frontal;
    theta_s_dot_desired_ = sagittal_vel;
    theta_f_dot_desired_ = frontal_vel;
    config_.control_mode = CTRL_MODE_MIXED_IMPEDANCE_CONTROL;
    if(both) axis_->other_->controller_.set_mixed_setpoint(false,sagittal, frontal, sagittal_vel, frontal_vel);
}

void Controller::set_mixed_gains(bool both, float kpS, float kpF, float kdS, float kdF){
    float rad2cpr = (2*PI/axis_->encoder_.config_.cpr);

    if(kpS > (100.0*config_.gear_ratio)) kpS = 100*config_.gear_ratio;
    if(kpS > (100.0*config_.gear_ratio)) kpS = 100*config_.gear_ratio; //Need 140
    if(kpS > (0.5*config_.gear_ratio)) kpS = 0.5*config_.gear_ratio; //Need 1.15
    if(kpS > (0.5*config_.gear_ratio)) kpS = 0.5*config_.gear_ratio;
    config_.pos_gain = kpS;
    config_.pos_gain2 = kpF;
    config_.vel_gain = kdS;
    config_.vel_gain2 = kdF;
    if(both) axis_->other_->controller_.set_mixed_gains(false, kpS, kpF, kdS, kdF);
}

void Controller::set_vel_setpoint(float vel_setpoint, float current_feed_forward) {
    vel_setpoint_ = vel_setpoint;
    current_setpoint_ = current_feed_forward;
    config_.control_mode = CTRL_MODE_VELOCITY_CONTROL;
#ifdef DEBUG_PRINT
    printf("VELOCITY_CONTROL %3.3f %3.3f\n", vel_setpoint_, motor->current_setpoint_);
#endif
}

void Controller::set_current_setpoint(float current_setpoint) {
    current_setpoint_ = current_setpoint;
    config_.control_mode = CTRL_MODE_CURRENT_CONTROL;
#ifdef DEBUG_PRINT
    printf("CURRENT_CONTROL %3.3f\n", current_setpoint_);
#endif
}

void Controller::start_anticogging_calibration() {
    // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
    if (anticogging_.cogging_map != NULL && axis_->error_ == Axis::ERROR_NONE) {
        anticogging_.calib_anticogging = true;
    }
}

/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
    if (anticogging_.calib_anticogging && anticogging_.cogging_map != NULL) {
        float pos_err = anticogging_.index - pos_estimate;
        if (fabsf(pos_err) <= anticogging_.calib_pos_threshold &&
            fabsf(vel_estimate) < anticogging_.calib_vel_threshold) {
            anticogging_.cogging_map[anticogging_.index++] = vel_integrator_current_;
        }
        if (anticogging_.index < axis_->encoder_.config_.cpr) { // TODO: remove the dependency on encoder CPR
            set_pos_setpoint(anticogging_.index, 0.0f, 0.0f);
            return false;
        } else {
            anticogging_.index = 0;
            set_pos_setpoint(0.0f, 0.0f, 0.0f);  // Send the motor home
            anticogging_.use_anticogging = true;  // We're good to go, enable anti-cogging
            anticogging_.calib_anticogging = false;
            return true;
        }
    }
    return false;
}

bool Controller::update(float pos_estimate, float vel_estimate, float* current_setpoint_output) {
    // Only runs if anticogging_.calib_anticogging is true; non-blocking
    anticogging_calibration(pos_estimate, vel_estimate);

    float Iq = 0; //No feedforward for impedance control

    // Impedance Control
    if(config_.control_mode == CTRL_MODE_IMPEDANCE_CONTROL){ //Use simple PD
        float vel_des = vel_setpoint_;
        
        // Velocity limiting
        float vel_lim = config_.vel_limit;
        if (vel_des > vel_lim) vel_des = vel_lim;
        if (vel_des < -vel_lim) vel_des = -vel_lim;

        // Anti-cogging is enabled after calibration
        // We get the current position and apply a current feed-forward
        // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
        if (anticogging_.use_anticogging) {
            Iq += anticogging_.cogging_map[mod(static_cast<int>(pos_estimate), axis_->encoder_.config_.cpr)];
        }

        float pos_err = pos_setpoint_ - pos_estimate;
        Iq += config_.pos_gain * pos_err; //Stiffness Controller
        float v_err = vel_des - vel_estimate;
        Iq += config_.vel_gain * v_err;  //Damping Controller

        Iq/=config_.gear_ratio; //Convert from joint N*m to motor N*m.
        Iq/=config_.torque_constant; //Convert from N*m to A. 

        // Current limiting
        float Ilim = std::min(axis_->motor_.config_.current_lim, axis_->motor_.current_control_.max_allowed_current);
        if (Iq > Ilim) {
            Iq = Ilim;
        }
        if (Iq < -Ilim) {
            Iq = -Ilim;
        }
    }else 
    if(config_.control_mode == CTRL_MODE_MIXED_IMPEDANCE_CONTROL){
        float vel_des = vel_setpoint_;
        
        // Velocity limiting
        float vel_lim = config_.vel_limit;
        if (vel_des > vel_lim) vel_des = vel_lim;
        if (vel_des < -vel_lim) vel_des = -vel_lim;

        // Anti-cogging is enabled after calibration
        // We get the current position and apply a current feed-forward
        // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
        if (anticogging_.use_anticogging) {
            Iq += anticogging_.cogging_map[mod(static_cast<int>(pos_estimate), axis_->encoder_.config_.cpr)];
        }

        //Mix and Unmix
        int32_t myCpr = axis_->encoder_.config_.cpr;

        int axmod = 0;
        if(!axis_->axis_num_){ //if Axis 0
            axmod = 1;
        }else{ //Else we are Axis 1
            axmod = -1;
        }

        //This calculation is done in radians. Control gain units are in Nm/rad and Nms/rad
        if(axis_->config_.use_pll_){
            theta_s_ = axmod*0.5f*(2*PI/myCpr)*(pos_estimate - axis_->other_->encoder_.pos_estimate_)/config_.gear_ratio; //perform these operations in joint space
            theta_f_ =       0.5f*(2*PI/myCpr)*(pos_estimate + axis_->other_->encoder_.pos_estimate_)/config_.gear_ratio;
            theta_s_dot_ = axmod*0.5f*(2*PI/myCpr)*(vel_estimate - axis_->other_->encoder_.pll_vel_)/config_.gear_ratio;
            theta_f_dot_ =       0.5f*(2*PI/myCpr)*(vel_estimate + axis_->other_->encoder_.pll_vel_)/config_.gear_ratio;
        }else{
            theta_s_ = axmod*0.5f*(2*PI/myCpr)*(pos_estimate - axis_->other_->encoder_avg_)/config_.gear_ratio;
            theta_f_ =       0.5f*(2*PI/myCpr)*(pos_estimate + axis_->other_->encoder_avg_)/config_.gear_ratio;
            theta_s_dot_ = axmod*0.5f*(2*PI/myCpr)*(vel_estimate - axis_->other_->vel_avg_)/config_.gear_ratio;
            theta_f_dot_ =       0.5f*(2*PI/myCpr)*(vel_estimate + axis_->other_->vel_avg_)/config_.gear_ratio;
        }

        torque_s_ = config_.pos_gain *(theta_s_desired_ - theta_s_) + config_.vel_gain *(theta_s_dot_desired_ - theta_s_dot_);
        torque_f_ = config_.pos_gain2*(theta_f_desired_ - theta_f_) + config_.vel_gain2*(theta_f_dot_desired_ - theta_f_dot_);
        Iq += 0.5f*(torque_f_+axmod*torque_s_);

        Iq/=config_.gear_ratio; //Convert from joint N*m to motor N*m.
        Iq/=config_.torque_constant; //Convert from N*m to A. 

        // Current limiting
        float Ilim = std::min(axis_->motor_.config_.current_lim, axis_->motor_.current_control_.max_allowed_current);
        if (Iq > Ilim) {
            Iq = Ilim;
        }
        if (Iq < -Ilim) {
            Iq = -Ilim;
        }
    }else{
        // Position control
        // TODO Decide if we want to use encoder or pll position here
        float vel_des = vel_setpoint_;
        if (config_.control_mode >= CTRL_MODE_POSITION_CONTROL) {
            float pos_err = pos_setpoint_ - pos_estimate;
            vel_des += config_.pos_gain * pos_err;
        }

        // Velocity limiting
        float vel_lim = config_.vel_limit;
        if (vel_des > vel_lim) vel_des = vel_lim;
        if (vel_des < -vel_lim) vel_des = -vel_lim;

        // Velocity control
        Iq = current_setpoint_;

        // Anti-cogging is enabled after calibration
        // We get the current position and apply a current feed-forward
        // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
        if (anticogging_.use_anticogging) {
            Iq += anticogging_.cogging_map[mod(static_cast<int>(pos_estimate), axis_->encoder_.config_.cpr)];
        }

        float v_err = vel_des - vel_estimate;
        if (config_.control_mode >= CTRL_MODE_VELOCITY_CONTROL) {
            Iq += config_.vel_gain * v_err;
        }

        // Velocity integral action before limiting
        Iq += vel_integrator_current_;
        
        // Current limiting
        float Ilim = std::min(axis_->motor_.config_.current_lim, axis_->motor_.current_control_.max_allowed_current);
        bool limited = false;
        if (Iq > Ilim) {
            limited = true;
            Iq = Ilim;
        }
        if (Iq < -Ilim) {
            limited = true;
            Iq = -Ilim;
        }
        
        // Velocity integrator (behaviour dependent on limiting)
        if (config_.control_mode < CTRL_MODE_VELOCITY_CONTROL) {
            // reset integral if not in use
            vel_integrator_current_ = 0.0f;
        } else {
            if (limited) {
                // TODO make decayfactor configurable
                vel_integrator_current_ *= 0.99f;
            } else {
                vel_integrator_current_ += (config_.vel_integrator_gain * current_meas_period) * v_err;
            }
        }
    }



    if (current_setpoint_output) *current_setpoint_output = Iq;
    return true;
}
