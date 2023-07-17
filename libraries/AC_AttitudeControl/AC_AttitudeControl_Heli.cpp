#include "AC_AttitudeControl_Heli.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

// GLOBAL VARIABLE
const int16_t MAX_DELAY = 400; //maximum delay buffer (=hc/dt)

int k = 1; //buffer
uint8_t h = 0; 
//uint8_t H = 120; //delay test

float dt = 0.0025f;
float x_d1 = 0.0f;
float x_1 = 0.0f;
float x_2 = 0.0f;
float u_k = 0.0f;
float u_k_delayed = 0.0f;
float d_est = 0.0f;
float z_k = 0.0f;
float z_k_1 = 0.0f;
float d_k_h = 0.0f;
float Input[MAX_DELAY] = {0}; 
float x_p3_[2][MAX_DELAY] = {0};
float Xp_h_[2][MAX_DELAY] = {0};
float Xp1_h_[2][MAX_DELAY] = {0};
float y2 = 0.0f;

/* 
Matrix3f eye2 (1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 0.0f);  
Matrix3f eye3 (1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 1.0f);                    
Matrix3f A (1.0f, dt, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f);
Vector3f B (0.0f, dt, 0.0f); 
Vector3f x_k (0.0f, 0.0f, 0.0f);
Vector3f x_d (0.0f, 0.0f, 0.0f);
Vector3f x_p3_k (0.0f, 0.0f, 0.0f);
Vector3f x_p_hat2 (0.0f, 0.0f, 0.0f);
Vector3f Xp (0.0f, 0.0f, 0.0f);
Vector3f Xp1 (0.0f, 0.0f, 0.0f);
Vector3f Xp2 (0.0f, 0.0f, 0.0f);
Vector3f Xp_h (0.0f, 0.0f, 0.0f);
Vector3f Xp1_h (0.0f, 0.0f, 0.0f);
Vector3f x_k_1 (0.0f, 0.0f, 0.0f);
Vector3f e_chi (0.0f, 0.0f, 0.0f);
Vector3f Xi_k (0.0f, 0.0f,0.0f);
Vector3f Xi_k_1 (0.0f, 0.0f,0.0f);
Vector3f delta_Xi_k (0.0f, 0.0f,0.0f); 
Vector3f c(0.0f, 0.0f, 0.0f);
Matrix3f Ap1(0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f);
Matrix3f Ap(0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f); 
Vector3f Bp(0.0f, 0.0f, 0.0f);
Vector3f Cp(0.0f, 0.0f, 0.0f); 
*/
 
MatrixN<float,2> eye2 (1.0f, 0.0f,
                        0.0f, 1.0f);
MatrixN<float,3> eye3 (1.0f, 0.0f, 0.0f,
                        0.0f, 1.0f, 0.0f,
                        0.0f, 0.0f, 1.0f);
MatrixN<float,2> A (1.0f, dt,
                    0.0f, 1.0f);
VectorN<float,2> B (0.0f, dt); 
VectorN<float,2> K (0.0f, 0.0f);
VectorN<float,2> L (0.0f, 0.0f);
VectorN<float,2> x_k (0.0f, 0.0f);
VectorN<float,2> x_d (0.0f, 0.0f);
VectorN<float,2> x_p3_k (0.0f, 0.0f);
VectorN<float,2> x_p_hat2 (0.0f, 0.0f);
VectorN<float,2> Xp (0.0f, 0.0f);
VectorN<float,2> Xp1 (0.0f, 0.0f);
VectorN<float,2> Xp2 (0.0f, 0.0f);
VectorN<float,2> Xp_h (0.0f, 0.0f);
VectorN<float,2> Xp1_h (0.0f, 0.0f);
VectorN<float,2> x_k_1 (0.0f, 0.0f);
VectorN<float,2> e_chi (0.0f, 0.0f);
VectorN<float,3> Xi_k (0.0f, 0.0f,0.0f);
VectorN<float,3> Xi_k_1 (0.0f, 0.0f,0.0f);
VectorN<float,3> delta_Xi_k (0.0f, 0.0f,0.0f); 
VectorN<float,3> c (0.0f, 0.0f, 0.0f);
MatrixN<float,3> Ap1 (0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f);
MatrixN<float,3> Ap (0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, 0.0f);
VectorN<float,3> Bp (0.0f, 0.0f, 0.0f);
VectorN<float,3> Cp (0.0f, 0.0f, 0.0f); 

Quaternion attitude_quat;

unsigned long long nchoosek(int n, int m);
//Vector3f Numerical_Integral1(float fn[MAX_DELAY], float dt_, Matrix3f  A_, Vector3f B_, Vector3f delta_Xi_k_, float d_est_, int h_);
//float Cp_Xi(int i, float dt_, int h_, Vector3f delta_Xi_k_, float d_est_);
VectorN<float,2> Numerical_Integral1(float* fn, float dt_, MatrixN<float,2>  A_, VectorN<float,2> B_, VectorN<float,3> delta_Xi_k_, float d_est_, int h_);
float Cp_Xi(int i, float dt_, int h_, VectorN<float,3> delta_Xi_k_, float d_est_);

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Heli::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    // @Param: HOVR_ROL_TRM
    // @DisplayName: Hover Roll Trim
    // @Description: Trim the hover roll angle to counter tail rotor thrust in a hover
    // @Units: cdeg
    // @Increment: 10
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("HOVR_ROL_TRM",    1, AC_AttitudeControl_Heli, _hover_roll_trim, AC_ATTITUDE_HELI_HOVER_ROLL_TRIM_DEFAULT),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.0 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.0 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_ILMI
    // @DisplayName: Roll axis rate controller I-term leak minimum
    // @Description: Point below which I-term will not leak down
    // @Range: 0 1
    // @User: Advanced

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0.05 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 2, AC_AttitudeControl_Heli, AC_HELI_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
    // @Range: 0.0 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_ILMI
    // @DisplayName: Pitch axis rate controller I-term leak minimum
    // @Description: Point below which I-term will not leak down
    // @Range: 0 1
    // @User: Advanced

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0.05 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 3, AC_AttitudeControl_Heli, AC_HELI_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.180 0.60
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.01 0.2
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_ILMI
    // @DisplayName: Yaw axis rate controller I-term leak minimum
    // @Description: Point below which I-term will not leak down
    // @Range: 0 1
    // @User: Advanced

    // @Param: RAT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FF
    // @DisplayName: Yaw axis rate controller feed forward
    // @Description: Yaw axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FLTT
    // @DisplayName: Yaw axis rate controller target frequency in Hz
    // @Description: Yaw axis rate controller target frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTE
    // @DisplayName: Yaw axis rate controller error frequency in Hz
    // @Description: Yaw axis rate controller error frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTD
    // @DisplayName: Yaw axis rate controller derivative frequency in Hz
    // @Description: Yaw axis rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 4, AC_AttitudeControl_Heli, AC_HELI_PID),

    // @Param: PIRO_COMP
    // @DisplayName: Piro Comp Enable
    // @Description: Pirouette compensation enabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("PIRO_COMP",    5, AC_AttitudeControl_Heli, _piro_comp_enabled, 0),
    
    AP_GROUPINFO("TD_K_1",    6, AC_AttitudeControl_Heli, _TD_K_1, 0.5),
    AP_GROUPINFO("TD_K_2",    7, AC_AttitudeControl_Heli, _TD_K_2, 0.01),
    AP_GROUPINFO("TD_L_1",    8, AC_AttitudeControl_Heli, _TD_L_1, 0.0),
    AP_GROUPINFO("TD_L_2",    9, AC_AttitudeControl_Heli, _TD_L_2, 0.0),
    AP_GROUPINFO("TD_ON_OFF",    10, AC_AttitudeControl_Heli, _TD_ON_OFF, 0),
    AP_GROUPINFO("TD_OMG",    11, AC_AttitudeControl_Heli, _TD_OMG, 0.0),
    AP_GROUPINFO("TD_Q",    12, AC_AttitudeControl_Heli, _TD_Q, 0),
    AP_GROUPINFO("TD_H",    13, AC_AttitudeControl_Heli, _TD_H, 0.05),
    AP_GROUPINFO("TD_TEST",    14, AC_AttitudeControl_Heli, _TD_TEST, 0),

    AP_GROUPEND
};

// passthrough_bf_roll_pitch_rate_yaw - passthrough the pilots roll and pitch inputs directly to swashplate for flybar acro mode
void AC_AttitudeControl_Heli::passthrough_bf_roll_pitch_rate_yaw(float roll_passthrough, float pitch_passthrough, float yaw_rate_bf_cds)
{
    // convert from centidegrees on public interface to radians
    float yaw_rate_bf_rads = radians(yaw_rate_bf_cds * 0.01f);

    // store roll, pitch and passthroughs
    // NOTE: this abuses yaw_rate_bf_rads
    _passthrough_roll = roll_passthrough;
    _passthrough_pitch = pitch_passthrough;
    _passthrough_yaw = degrees(yaw_rate_bf_rads) * 100.0f;

    // set rate controller to use pass through
    _flags_heli.flybar_passthrough = true;

    // set bf rate targets to current body frame rates (i.e. relax and be ready for vehicle to switch out of acro)
    _ang_vel_target.x = _ahrs.get_gyro().x;
    _ang_vel_target.y = _ahrs.get_gyro().y;

    // accel limit desired yaw rate
    if (get_accel_yaw_max_radss() > 0.0f) {
        float rate_change_limit_rads = get_accel_yaw_max_radss() * _dt;
        float rate_change_rads = yaw_rate_bf_rads - _ang_vel_target.z;
        rate_change_rads = constrain_float(rate_change_rads, -rate_change_limit_rads, rate_change_limit_rads);
        _ang_vel_target.z += rate_change_rads;
    } else {
        _ang_vel_target.z = yaw_rate_bf_rads;
    }

    integrate_bf_rate_error_to_angle_errors();
    _att_error_rot_vec_rad.x = 0;
    _att_error_rot_vec_rad.y = 0;

    // update our earth-frame angle targets
    Vector3f att_error_euler_rad;

    // convert angle error rotation vector into 321-intrinsic euler angle difference
    // NOTE: this results an an approximation linearized about the vehicle's attitude
    if (ang_vel_to_euler_rate(Vector3f(_ahrs.roll, _ahrs.pitch, _ahrs.yaw), _att_error_rot_vec_rad, att_error_euler_rad)) {
        _euler_angle_target.x = wrap_PI(att_error_euler_rad.x + _ahrs.roll);
        _euler_angle_target.y = wrap_PI(att_error_euler_rad.y + _ahrs.pitch);
        _euler_angle_target.z = wrap_2PI(att_error_euler_rad.z + _ahrs.yaw);
    }

    // handle flipping over pitch axis
    if (_euler_angle_target.y > M_PI / 2.0f) {
        _euler_angle_target.x = wrap_PI(_euler_angle_target.x + M_PI);
        _euler_angle_target.y = wrap_PI(M_PI - _euler_angle_target.x);
        _euler_angle_target.z = wrap_2PI(_euler_angle_target.z + M_PI);
    }
    if (_euler_angle_target.y < -M_PI / 2.0f) {
        _euler_angle_target.x = wrap_PI(_euler_angle_target.x + M_PI);
        _euler_angle_target.y = wrap_PI(-M_PI - _euler_angle_target.x);
        _euler_angle_target.z = wrap_2PI(_euler_angle_target.z + M_PI);
    }

    // convert body-frame angle errors to body-frame rate targets
    _ang_vel_body = update_ang_vel_target_from_att_error(_att_error_rot_vec_rad);

    // set body-frame roll/pitch rate target to current desired rates which are the vehicle's actual rates
    _ang_vel_body.x = _ang_vel_target.x;
    _ang_vel_body.y = _ang_vel_target.y;

    // add desired target to yaw
    _ang_vel_body.z += _ang_vel_target.z;
    _thrust_error_angle = _att_error_rot_vec_rad.xy().length();
}

void AC_AttitudeControl_Heli::integrate_bf_rate_error_to_angle_errors()
{
    // Integrate the angular velocity error into the attitude error
    _att_error_rot_vec_rad += (_ang_vel_target - _ahrs.get_gyro()) * _dt;

    // Constrain attitude error
    _att_error_rot_vec_rad.x = constrain_float(_att_error_rot_vec_rad.x, -AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD, AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD);
    _att_error_rot_vec_rad.y = constrain_float(_att_error_rot_vec_rad.y, -AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD, AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD);
    _att_error_rot_vec_rad.z = constrain_float(_att_error_rot_vec_rad.z, -AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD, AC_ATTITUDE_HELI_ACRO_OVERSHOOT_ANGLE_RAD);
}

// subclass non-passthrough too, for external gyro, no flybar
void AC_AttitudeControl_Heli::input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    _passthrough_yaw = yaw_rate_bf_cds;

    AC_AttitudeControl::input_rate_bf_roll_pitch_yaw(roll_rate_bf_cds, pitch_rate_bf_cds, yaw_rate_bf_cds);
}

//
// rate controller (body-frame) methods
//

// rate_controller_run - run lowest level rate controller and send outputs to the motors
// should be called at 100hz or more
void AC_AttitudeControl_Heli::rate_controller_run()
{	
    _ang_vel_body += _sysid_ang_vel_body;

    Vector3f gyro_latest = _ahrs.get_gyro_latest();

    // call rate controllers and send output to motors object
    // if using a flybar passthrough roll and pitch directly to motors
    if (_flags_heli.flybar_passthrough) {
        _motors.set_roll(_passthrough_roll / 4500.0f);
        _motors.set_pitch(_passthrough_pitch / 4500.0f);
    } else {
         if (_TD_ON_OFF == 0 || _TD_ON_OFF == 2){ // PPID
        rate_bf_to_motor_roll_pitch(gyro_latest, _ang_vel_body.x, _ang_vel_body.y); // state rate, target roll rate, target pitch rate
        }else{ // time delay on
            exp_pbc_time_delay_system_roll(gyro_latest, _euler_angle_target.x , _ang_vel_body.y); // state rate, target roll, target pitch rate  
        }
        // rate_bf_to_motor_roll_pitch(gyro_latest, _ang_vel_body.x, _ang_vel_body.y); // for time delay pid test

    }
    if (_flags_heli.tail_passthrough) {
        _motors.set_yaw(_passthrough_yaw / 4500.0f);
    } else {
        _motors.set_yaw(rate_target_to_motor_yaw(gyro_latest.z, _ang_vel_body.z));
    }

    _sysid_ang_vel_body.zero();
    _actuator_sysid.zero();

}

void AC_AttitudeControl_Heli::initABCp() {

    for(int i=1; i<=3; i++)
    {
        c[i-1] = nchoosek(3,i)*pow(_TD_OMG, i);
    } 

    for(int i=1; i<=2; i++)
    {
        Ap1[i-1][0] = -c[i-1];
        Ap1[i-1][i] = 1.0f;
    }

    Ap1[2][0] = -c[2];
    Ap = eye3 + Ap1 * dt;

    for(int i = 0; i < 3; i++)
    {
        Bp[i] = dt * c[i];
    }

    for(int i=1; i<=3; i++)
    {
        Cp[i-1] = dt*nchoosek(_TD_H/dt,i);
        //Cp[i-1] = dt*nchoosek(H,i); //delay test

    }

    Cp[0] = Cp[0] / dt;
} 
// nCr 
unsigned long long nchoosek(int n, int m) {
  unsigned long long result, result_1, result_2;
  result_1 = result_2 = 1;
  for (int i = n; i > n - m; i--) {
    result_1 *= i;
  }
  
  for (int i = 1; i <= m; i++) {
    result_2 *= i;
  }
  
  result = result_1 / result_2;
  return result;
}

//Vector3f Numerical_Integral1(float fn[MAX_DELAY], float dt_, Matrix3f  A_, Vector3f B_, Vector3f delta_Xi_k_, float d_est_, int h_){
VectorN<float,2> Numerical_Integral1(float* fn, float dt_, MatrixN<float,2>  A_, VectorN<float,2> B_, VectorN<float,3> delta_Xi_k_, float d_est_, int h_){
    //Vector3f y (0.0f, 0.0f, 0.0f);
    VectorN<float,2> y (0.0f, 0.0f);
    
    for(int i = 1; i <= h_; i++) {    
        y +=  A_.power(h_ - i) * B_ * (fn[i -1]+ Cp_Xi(i-1, dt_, h_, delta_Xi_k_, d_est_));
    }

    return y;
}

//float Cp_Xi(int i, float dt_, int h_, Vector3f delta_Xi_k_, float d_est_){
float Cp_Xi(int i, float dt_, int h_, VectorN<float,3> delta_Xi_k_, float d_est_){
    //Vector3f Cp_ (1.0f, 1.0f,1.0f);
    VectorN<float,3> Cp_ (1.0f, 1.0f, 1.0f);

    if (i <= 2){
        for (int j=2;j<=i+1;j++){
            Cp_[j-1] = dt_*nchoosek(i,j-1);
        }
        Cp_[0] = 1.0f/dt_;
        for(int j = 0; j <= i; j++)
        {
            y2 = Cp_[j] * delta_Xi_k_[j];
        }
        y2 += d_est_;
    }else{
        for (int j=2;j<=3;j++){
            Cp_[j-1] = dt_*nchoosek(i,j-1);
        }
        Cp_[0] = 1.0f/dt_;
        for(int j = 0; j < 3; j++)
        {
            y2 = Cp_[j] * delta_Xi_k_[j];
        }
        y2 += d_est_;
    }

    return y2;    
}


// Main
void AC_AttitudeControl_Heli::exp_pbc_time_delay_system_roll(const Vector3f &rate_rads, float roll_target_rads, float rate_pitch_target_rads)
{   
    // Logger
    if (_TD_TEST == 1){
    AP::logger().Write_X(x_1,x_2,x_p3_k[0],x_p3_k[1],d_est,d_k_h);
    }

    // BUFFER
    h = _TD_H/dt;
    //h = H;// delay test
    // REFERENCE TRAJECTORY
    x_d1 = roll_target_rads;
    // float x_d2 = 0.0f;
    //x_d = Vector3f(x_d1, 0.0f ,0.0f);
    x_d = {x_d1, 0.0f};

    // DEFINE ERROR
    if (_TD_Q == 0){
        e_chi = x_p3_k - x_d;
    }
    else if (_TD_Q == 1){
        Xp = x_k + (x_p3_k - x_p_hat2);
        e_chi = Xp - x_d;
    }
    else if (_TD_Q == 2){
        Xp = x_k + (x_p3_k - x_p_hat2);
        Xp1 = x_k + (Xp - Xp_h);
        e_chi = Xp1 - x_d;
    }
    else if (_TD_Q == 3){
        Xp = x_k + (x_p3_k - x_p_hat2);
        Xp1 = x_k + (Xp - Xp_h);
        Xp2 = x_k + (Xp1 - Xp1_h);
        e_chi = Xp2 - x_d;
    }

    // CONTROLLER INPUT
    //Vector3f K (_TD_K_1, _TD_K_2, 0);
    K = {_TD_K_1, _TD_K_2};
    u_k = - K * e_chi - d_k_h;
    u_k_delayed = Input[k-1];

    // DOB
    //Vector3f L (_TD_L_1, _TD_L_2,0);
    L = {_TD_L_1, _TD_L_2};
    z_k_1 = z_k + L*((A-eye2)*x_k + B*(u_k_delayed+d_est));
    d_est = L*x_k - z_k;

    // PREDICTOR FOR d(t+h)
    Xi_k_1 = Ap*Xi_k + Bp*d_est;
    delta_Xi_k = Xi_k_1-Xi_k; // modified
    //delta_Xi_k = Vector3f(Xi_k_1[1] - Xi_k[1], Xi_k_1[2] - Xi_k[2], Xi_k_1[3] - Xi_k[3]);
    d_k_h = Cp*delta_Xi_k + d_est;
    
    // CONSTRAINTED OUTPUT TO MOTOR
    float roll_out = u_k_delayed;
    if (fabsf(roll_out) > AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        roll_out = constrain_float(roll_out, -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
        _flags_heli.limit_roll = true;
    } else {
        _flags_heli.limit_roll = false;
    }
    _motors.set_roll(roll_out); 

    // STATE PREDICTION
    x_p3_k = A.power(h)*x_k + Numerical_Integral1(Input,dt,A,B,delta_Xi_k,d_est,h);
    
    //_TD_Q == 0-> do nothing
    if (_TD_Q == 1){ 
    x_p_hat2[0] = x_p3_[0][k-1];
    x_p_hat2[1] = x_p3_[1][k-1];

    //Next step
    x_p3_[0][k-1] = x_p3_k[0];
    x_p3_[1][k-1] = x_p3_k[1];
    }

    else if (_TD_Q == 2){
    x_p_hat2[0] = x_p3_[0][k-1];
    x_p_hat2[1] = x_p3_[1][k-1];

    Xp_h[0] = Xp_h_[0][k-1];
    Xp_h[1] = Xp_h_[1][k-1];

    //Next step
    x_p3_[0][k-1] = x_p3_k[0];
    x_p3_[1][k-1] = x_p3_k[1];

    Xp_h_[0][k-1] = Xp[0];
    Xp_h_[1][k-1] = Xp[1];
    }

    else if (_TD_Q == 3){
    x_p_hat2[0] = x_p3_[0][k-1];
    x_p_hat2[1] = x_p3_[1][k-1];

    Xp_h[0] = Xp_h_[0][k-1];
    Xp_h[1] = Xp_h_[1][k-1];

    Xp1_h[0] = Xp1_h_[0][k-1];
    Xp1_h[1] = Xp1_h_[1][k-1];

    //Next step
    x_p3_[0][k-1] = x_p3_k[0];
    x_p3_[1][k-1] = x_p3_k[1];

    Xp_h_[0][k-1] = Xp[0];
    Xp_h_[1][k-1] = Xp[1];

    Xp1_h_[0][k-1] = Xp1[0];
    Xp1_h_[1][k-1] = Xp1[1];
    }

    // CURRENT STATE
    AP::ahrs().get_quat_body_to_ned(attitude_quat);
    x_1 = attitude_quat.get_euler_roll();
    x_2 = rate_rads.x;
    //x_k_1= Vector3f(x_1,x_2,0.0f);
    x_k_1 = {x_1,x_2}; 

    // FOR NEXT STEP
    Input[k-1] = u_k;
    x_k = x_k_1;
    z_k = z_k_1;
    Xi_k = Xi_k_1;
    
    // NEXT k index 
    k = (k % h)+1;

    // output for pitch (ppid)
    if (_flags_heli.leaky_i) {
        _pid_rate_pitch.update_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
    float pitch_pid = _pid_rate_pitch.update_all(rate_pitch_target_rads, rate_rads.y, dt, _motors.limit.pitch) + _actuator_sysid.y;
    // use pid library to calculate ff
    float pitch_ff = _pid_rate_pitch.get_ff();
    // add feed forward and final output
    float pitch_out = pitch_pid + pitch_ff;
    // constrain output and update limit flags
    if (fabsf(pitch_out) > AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        pitch_out = constrain_float(pitch_out, -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
        _flags_heli.limit_pitch = true;
    }else{
        _flags_heli.limit_pitch = false;
    }
    // output to motors
    _motors.set_pitch(pitch_out); 
    } 

}


// Update Alt_Hold angle maximum
void AC_AttitudeControl_Heli::update_althold_lean_angle_max(float throttle_in)
{
    float althold_lean_angle_max = acosf(constrain_float(throttle_in / AC_ATTITUDE_HELI_ANGLE_LIMIT_THROTTLE_MAX, 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt / (_dt + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max);
}

//
// private methods
//

//
// body-frame rate controller
//

// rate_bf_to_motor_roll_pitch - ask the rate controller to calculate the motor outputs to achieve the target rate in radians/second
void AC_AttitudeControl_Heli::rate_bf_to_motor_roll_pitch(const Vector3f &rate_rads, float rate_roll_target_rads, float rate_pitch_target_rads)
{

   
    if (_flags_heli.leaky_i) {
        _pid_rate_roll.update_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
    }
    float roll_pid = _pid_rate_roll.update_all(rate_roll_target_rads, rate_rads.x, _dt, _motors.limit.roll) + _actuator_sysid.x;
    float roll_ff = _pid_rate_roll.get_ff();
    float roll_out = roll_pid + roll_ff;
    if (fabsf(roll_out) > AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        roll_out = constrain_float(roll_out, -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
        _flags_heli.limit_roll = true;
    } else {
        _flags_heli.limit_roll = false;
    }    
    // JH 07_12_23 for time delay pid test
    if(_TD_ON_OFF==2){
        h = _TD_H/dt;
        u_k_delayed = Input[k - 1];
        _motors.set_roll(u_k_delayed);  
        u_k = roll_out;
        Input[k-1] = u_k;
        k = (k % h)+1;
    }else{
        _motors.set_roll(roll_out);
    }
if (_flags_heli.leaky_i) {
        _pid_rate_pitch.update_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
    }
    float pitch_pid = _pid_rate_pitch.update_all(rate_pitch_target_rads, rate_rads.y, _dt, _motors.limit.pitch) + _actuator_sysid.y;
    // use pid library to calculate ff
    float pitch_ff = _pid_rate_pitch.get_ff();
    // add feed forward and final output
    float pitch_out = pitch_pid + pitch_ff;
    // constrain output and update limit flags
    if (fabsf(pitch_out) > AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX) {
        pitch_out = constrain_float(pitch_out, -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);
        _flags_heli.limit_pitch = true;
    } else {
        _flags_heli.limit_pitch = false;
    }
    // output to motors
    _motors.set_pitch(pitch_out);    
    
    // Piro-Comp, or Pirouette Compensation is a pre-compensation calculation, which basically rotates the Roll and Pitch Rate I-terms as the
    // helicopter rotates in yaw.  Much of the built-up I-term is needed to tip the disk into the incoming wind.  Fast yawing can create an instability
    // as the built-up I-term in one axis must be reduced, while the other increases.  This helps solve that by rotating the I-terms before the error occurs.
    // It does assume that the rotor aerodynamics and mechanics are essentially symmetrical about the main shaft, which is a generally valid assumption. 
    if (_piro_comp_enabled) {

        // used to hold current I-terms while doing piro comp:
        const float piro_roll_i = _pid_rate_roll.get_i();
        const float piro_pitch_i = _pid_rate_pitch.get_i();

        Vector2f yawratevector;
        yawratevector.x     = cosf(-rate_rads.z * _dt);
        yawratevector.y     = sinf(-rate_rads.z * _dt);
        yawratevector.normalize();

        _pid_rate_roll.set_integrator(piro_roll_i * yawratevector.x - piro_pitch_i * yawratevector.y);
        _pid_rate_pitch.set_integrator(piro_pitch_i * yawratevector.x + piro_roll_i * yawratevector.y);
    }

}

// rate_bf_to_motor_yaw - ask the rate controller to calculate the motor outputs to achieve the target rate in radians/second
float AC_AttitudeControl_Heli::rate_target_to_motor_yaw(float rate_yaw_actual_rads, float rate_target_rads)
{
    if (!((AP_MotorsHeli&)_motors).rotor_runup_complete()) {
        _pid_rate_yaw.update_leaky_i(AC_ATTITUDE_HELI_RATE_INTEGRATOR_LEAK_RATE);
    }

    float pid = _pid_rate_yaw.update_all(rate_target_rads, rate_yaw_actual_rads, _dt,  _motors.limit.yaw) + _actuator_sysid.z;

    // use pid library to calculate ff
    float vff = _pid_rate_yaw.get_ff()*_feedforward_scalar;

    // add feed forward
    float yaw_out = pid + vff;

    // constrain output and update limit flag
    if (fabsf(yaw_out) > AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX) {
        yaw_out = constrain_float(yaw_out, -AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX);
        _flags_heli.limit_yaw = true;
    } else {
        _flags_heli.limit_yaw = false;
    }

    // output to motors
    return yaw_out;
}

//
// throttle functions
//

void AC_AttitudeControl_Heli::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    _motors.set_throttle(throttle_in);
    // Clear angle_boost for logging purposes
    _angle_boost = 0.0f;
}

// Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl_Heli::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)
{
    if (_inverted_flight) {
        euler_roll_angle_cd = wrap_180_cd(euler_roll_angle_cd + 18000);
    }
    AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_rate_cds);
}

// Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
void AC_AttitudeControl_Heli::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw)
{
    if (_inverted_flight) {
        euler_roll_angle_cd = wrap_180_cd(euler_roll_angle_cd + 18000);
    }
    AC_AttitudeControl::input_euler_angle_roll_pitch_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_angle_cd, slew_yaw);
}
