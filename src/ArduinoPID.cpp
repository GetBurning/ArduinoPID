/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <ArduinoPID.h>

PID::PID(double* Input, double* Output, double* Setpoint,
	 double* pError, double* iError, double* dError) {
  myOutput = Output; myInput = Input; mySetpoint = Setpoint;
  mypError = pError; myiError = iError; mydError = dError;
  active = false;
  c = {100, 0, // Default controller Sample Time is 0.1 seconds
       FORWARD,
       1, 1, 1, // Default tuning gains
       P_ON_ERROR,
       // Default output limit corresponds to the arduino pwm limits
       0, 255, 0, 255, 0};
  PID::SetTunings(c.Kp, c.Ki, c.Kd);
}

void PID::reset() {
  lastInput = *myInput;
  lastOutput = *myOutput;
  *myiError = *myOutput;
  if (*myiError > c.iErrorMax) *myiError = c.iErrorMax;
  else if (*myiError < c.iErrorMin) *myiError = c.iErrorMin;
}

void PID::begin() {
  PID::reset();
  active = true;
}

bool PID::begin(double Kp, double Ki, double Kd,
		PID_p_mode_t pMode, PID_direction_t Direction) {
  bool r;
  PID::SetDirection(Direction);
  PID::SetPMode(pMode);
  r = PID::SetTunings(Kp, Ki, Kd);
  if (!r) return r;
  PID::begin();
  return true;
}
bool PID::begin(pid_config conf) {
  bool r;
  r = PID::SetConfig(conf);
  if (!r) return r;
  PID::begin();
  return true;
}
bool PID::SetConfig(pid_config conf) {
  bool r;
  r = PID::SetSampleInterval(conf.sampleInterval);
  if (!r) return r;
  r = PID::SetTunings(conf.Kp, conf.Ki, conf.Kd);
  if (!r) return r;
  c = conf;
  return true;
}
bool PID::SetTunings(double Kp, double Ki, double Kd) {
  if (Kp<0 || Ki<0 || Kd<0) return false;
  c.Kp = Kp; c.Ki = Ki; c.Kd = Kd;

  double SampleIntervalInSec = ((double)c.sampleInterval)/1000;
  kp = Kp;
  ki = Ki * SampleIntervalInSec;
  kd = Kd / SampleIntervalInSec;

  if (c.direction == REVERSE) {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
  return true;
}

bool PID::SetSampleInterval(uint16_t Interval) {
  if (Interval == 0) return false;
  double ratio  = (double)Interval / (double)c.sampleInterval;
  ki *= ratio;
  kd /= ratio;
  c.sampleInterval = Interval;
  return true;
}

void PID::SetDeadInterval(uint16_t Interval) {
  c.deadInterval = Interval;
}

void PID::SetDeadBand(double Band) {
  c.deadband = Band;
}

bool PID::SetOutputBounds(double Min, double Max) {
  if(Min >= Max) return false;
  c.outMin = Min;
  c.outMax = Max;
  if (active) {
    if (*myOutput > c.outMax) *myOutput = c.outMax;
    else if (*myOutput < c.outMin) *myOutput = c.outMin;
  }
  return true;
}

bool PID::SetIErrorBounds(double Min, double Max) {
  if (Min >= Max) return false;
  c.iErrorMin = Min;
  c.iErrorMax = Max;
  if (active) {
    if (*myiError > c.iErrorMax) *myiError = c.iErrorMax;
    else if (*myiError < c.iErrorMin) *myiError = c.iErrorMin;
  }
  return true;
}

/* SetDirection(...)*************************************************
 * The PID will either be connected to a FORWARD acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetDirection(PID_direction_t Direction) {
   if (active && Direction != c.direction) {
     kp = (0 - kp);
     ki = (0 - ki);
     kd = (0 - kd);
   }
   c.direction = Direction;
}
bool PID::compute(uint32_t now) {
  if (!active) return false;
  // Compute all the working error variables
  double input = *myInput;
  double error = *mySetpoint - input;
  double pError, iError = *myiError, dError;
  double dInput = 0 - (input - lastInput);
  double output = 0;
  lastInput = input;
  /* Use Proportional on Error if P_ON_ERROR is specified
     Else use Proportional on Measurement */
  if (c.pMode == P_ON_ERROR) {
    pError = kp * error;
    output = pError;
  } else {
    pError = kp * dInput;
    iError += pError;
  }
  iError += (ki * error);
  dError = kd * dInput;
  
  // Bound the integral error to avoid windup
  if (iError > c.iErrorMax) iError = c.iErrorMax;
  else if (iError < c.iErrorMin) iError = c.iErrorMin;

  // Compute Rest of PID Output
  output += iError + dError;
  
  *mypError = pError; *myiError = iError; *mydError = dError;
  
  if (output > c.outMax) output = c.outMax;
  else if (output < c.outMin) output = c.outMin;
  *myOutput = output;

  if (c.deadInterval == 0 || (now - lastChangeTime) >= c.deadInterval) {
    lastChangeTime = now;
    if (abs(lastOutput - output) >= c.deadband) {
      lastOutput = output;
      return true;
    }
  }
  return false;
}
