#include <PID.hpp>
#include <math.h>

void PIDController_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->proportional = 0.0f;
	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;

}

float PIDController_Update(PIDController *pid, float setpoint, float measurement, float dt, bool foo) {

	/*
	 * dt
	 */
	pid->T = dt;

	/*
	* Error signal
	*/
    float error = setpoint - measurement;

    if(foo) if(-3 < error && error < 3){
    	error = 0;
    }


	/*
	* Proportional
	*/
    pid->proportional = pid->Kp * error;


//	/*
//	* Integral
//	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if(pid->Ki == 0){
    	pid->integrator = 0;
    }
    else if (pid->integrator > pid->limMaxInt) {
        pid->integrator = pid->limMaxInt;
    } else if (pid->integrator < pid->limMinInt) {
        pid->integrator = pid->limMinInt;
    }

	/*
	* Derivative (band-limited differentiator)
	*/
		
//    pid->differentiator = (-2.0f * pid->Kd * (measurement - pid->prevMeasurement) + (2.0f * pid->tau - pid->T) * pid->differentiator)
//                        		/ (2.0f * pid->tau + pid->T);

    pid->differentiator = (pid->Kd * (measurement - pid->prevMeasurement) + pid->tau*pid->differentiator)/(pid->tau + pid->T);

	/*
	* Compute output and apply limits
	*/
    pid->out = pid->proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {
//        pid->integrator += pid->limMax - pid->out;
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin) {
//        pid->integrator += pid->limMin - pid->out;
        pid->out = pid->limMin;
    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return pid->out;

}
