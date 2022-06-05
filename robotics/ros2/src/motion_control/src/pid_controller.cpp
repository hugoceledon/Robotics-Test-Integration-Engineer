/*! @package pid_controller
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include "motion_control/pid_controller.hpp"

PIDController::PIDController() {}

float PIDController::ThrottlePID(float ref_vx, float cur_vx, double dt)
{
    /********************************************
     * PID Throttle Controller
     ********************************************/
    if (!m_throttle_ctrl){
        // If controller is set to be off on envVars
        return ref_vx;
    }
    if (ref_vx == 0){
        // If reference velocity is zero then restart integral error and return 0
        m_vx_int_error = 0;
        return 0;
    }
    // Get previous error
    float _prev_error = m_vx_prop_ek1;
    // Calculate current error
    m_vx_prop_ek1 = ref_vx - cur_vx;
    // calculate integral error
    m_vx_int_error += m_vx_prop_ek1 * dt;
    /********************************************
     * PID Formula
     * Kp * currentError
     * Ki * integralError
     * Kd * differentialError
     * 
     * Differential error is given by:
     * ( currentError - previousError ) / dt
     * TODO: Max values control
     ********************************************/
    float output = (m_kp_thr * m_vx_prop_ek1) + ( m_vx_int_error * m_ki_thr) + ( m_kd_thr * ( m_vx_prop_ek1 - _prev_error ) / dt );
    return output;
    /********************************************
     * END CODE
     *  ********************************************/
}

float PIDController::SteeringPID(float ref_wz, float cur_wz, double dt)
{
    /********************************************
     * DEFINE YOUR AMAZING PID CONTROLLER
     * Find Documentation here:
     * https://www.elprocus.com/the-working-of-a-pid-controller/
     *
     * FeedForward:
     * https://zhuanlan.zhihu.com/p/382010500#:~:text=In%20many%20applications,dynamic%20models%20used.
     * "Combined FeedForward and Feedback Control"
     ********************************************/
    m_steering_ctrl

    ref_wz

    m_wz_int_error
    m_wz_prop_ek1

    m_kp_str
    m_ki_str
    m_kd_str
    /********************************************
     * END CODE
     *  ********************************************/
}
