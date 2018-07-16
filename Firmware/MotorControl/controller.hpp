#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

// Note: these should be sorted from lowest level of control to
// highest level of control, to allow "<" style comparisons.
typedef enum {
    CTRL_MODE_VOLTAGE_CONTROL = 0,
    CTRL_MODE_CURRENT_CONTROL = 1,
    CTRL_MODE_VELOCITY_CONTROL = 2,
    CTRL_MODE_POSITION_CONTROL = 3, 
    CTRL_MODE_IMPEDANCE_CONTROL = 4, 
    CTRL_MODE_MIXED_IMPEDANCE_CONTROL = 5,
} Motor_control_mode_t;

struct ControllerConfig_t {
    Motor_control_mode_t control_mode = CTRL_MODE_MIXED_IMPEDANCE_CONTROL;  //see: Motor_control_mode_t
    float pos_gain = 0.001f;  // [(counts/s) / counts] or [N*m/rad]
    float vel_gain = 0.0001f;  // [A/(counts/s)] or [N*m*s/rad]
    float pos_gain2 = 0.001f;  // [(counts/s) / counts] or [N*m/rad] FOR MIXED MODE
    float vel_gain2 = 0.0001f;  // [A/(counts/s)] or [N*m*s/rad]
    float torque_constant = 0.45f; //[N*m/A], only used in Impedance Control mode
    float gear_ratio = (32.0f/15.0f); //[N*m/A], only used in Impedance Control mode
    // float vel_gain = 5.0f / 200.0f, // [A/(rad/s)] <sensorless example>
    float vel_integrator_gain = 10.0f / 10000.0f;  // [A/(counts/s * s)]
    float vel_limit = 20000.0f;           // [counts/s]
};

class Controller {
public:
    Controller(ControllerConfig_t& config);
    void reset();

    void set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward);
    void set_vel_setpoint(float vel_setpoint, float current_feed_forward);
    void set_current_setpoint(float current_setpoint);
    
    // TODO: make this more similar to other calibration loops
    void start_anticogging_calibration();
    bool anticogging_calibration(float pos_estimate, float vel_estimate);

    bool update(float pos_estimate, float vel_estimate, float* current_setpoint);

    ControllerConfig_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    void set_mixed_pos_setpoint(bool both,float sagittal, float frontal);
    void set_mixed_setpoint(bool both,float sagittal, float frontal, float sagittal_vel, float frontal_vel);
    void set_mixed_gains(bool both,float kpS, float kpF, float kdS, float kdF);

    // TODO: anticogging overhaul:
    // - expose selected (all?) variables on protocol
    // - make calibration user experience similar to motor & encoder calibration
    // - use python tools to Fourier transform and write back the smoothed map or Fourier coefficients
    // - make the calibration persistent

    typedef struct {
        int index;
        float *cogging_map;
        bool use_anticogging;
        bool calib_anticogging;
        float calib_pos_threshold;
        float calib_vel_threshold;
    } Anticogging_t;
    Anticogging_t anticogging_ = {
        .index = 0,
        .cogging_map = nullptr,
        .use_anticogging = false,
        .calib_anticogging = false,
        .calib_pos_threshold = 1.0f,
        .calib_vel_threshold = 1.0f,
    };

    // variables exposed on protocol
    float pos_setpoint_ = 0.0f;
    float vel_setpoint_ = 0.0f;

    float theta_s_desired_ = 0.0f;
    float theta_f_desired_ = 0.0f;
    float theta_s_dot_desired_ = 0.0f;
    float theta_f_dot_desired_ = 0.0f;
    float theta_s_ = 0.0f; //This is in RADIANS
    float theta_f_ = 0.0f;
    float theta_s_dot_ = 0.0f;
    float theta_f_dot_ = 0.0f;
    float torque_s_ = 0.0f; //Sagittal Torque
    float torque_f_ = 0.0f; //Frontal Torque

    // float vel_setpoint = 800.0f; <sensorless example>
    float vel_integrator_current_ = 0.0f;  // [A]
    float current_setpoint_ = 0.0f;        // [A]

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_property("pos_setpoint", &pos_setpoint_),
            make_protocol_property("vel_setpoint", &vel_setpoint_),
            make_protocol_property("theta_s_desired", &theta_s_desired_),
            make_protocol_property("theta_f_desired", &theta_f_desired_),
            make_protocol_property("theta_s_dot_desired", &theta_s_dot_desired_),
            make_protocol_property("theta_f_dot_desired", &theta_f_dot_desired_),
            make_protocol_property("theta_s", &theta_s_),
            make_protocol_property("theta_f", &theta_f_),
            make_protocol_property("theta_s_dot", &theta_s_dot_),
            make_protocol_property("theta_f_dot", &theta_f_dot_),
            make_protocol_property("vel_integrator_current", &vel_integrator_current_),
            make_protocol_property("current_setpoint", &current_setpoint_),
            make_protocol_object("config",
                make_protocol_property("control_mode", &config_.control_mode),
                make_protocol_property("pos_gain", &config_.pos_gain),
                make_protocol_property("vel_gain", &config_.vel_gain),
                make_protocol_property("pos_gain2", &config_.pos_gain2),
                make_protocol_property("vel_gain2", &config_.vel_gain2),
                make_protocol_property("torque_constant", &config_.torque_constant),
                make_protocol_property("gear_ratio", &config_.gear_ratio),
                make_protocol_property("vel_integrator_gain", &config_.vel_integrator_gain),
                make_protocol_property("vel_limit", &config_.vel_limit)
            ),
            make_protocol_function("set_pos_setpoint", *this, &Controller::set_pos_setpoint,
                "pos_setpoint",
                "vel_feed_forward",
                "current_feed_forward"),
            make_protocol_function("set_mixed_pos_setpoint", *this, &Controller::set_mixed_pos_setpoint,
                "both",
                "sagittal", 
                "frontal"),
            make_protocol_function("set_mixed_setpoint", *this, &Controller::set_mixed_setpoint,
                "both",
                "sagittal", 
                "frontal", 
                "sagittal_vel", 
                "frontal_vel"),
            make_protocol_function("set_mixed_gains", *this, &Controller::set_mixed_gains,
                "both",
                "kpS", 
                "kpF", 
                "kdS", 
                "kdF"),
            make_protocol_function("set_vel_setpoint", *this, &Controller::set_vel_setpoint,
                "vel_setpoint",
                "current_feed_forward"),
            make_protocol_function("set_current_setpoint", *this, &Controller::set_current_setpoint,
                "current_setpoint"),
            make_protocol_function("start_anticogging_calibration", *this, &Controller::start_anticogging_calibration)
        );
    }
};

#endif // __CONTROLLER_HPP
