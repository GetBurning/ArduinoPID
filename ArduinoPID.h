#ifndef ArduinoPID_h
#define ArduinoPID_h
#define LIBRARY_VERSION 2.0.0

typedef enum {FORWARD, REVERSE} PID_direction_t;
typedef enum {P_ON_MEASUREMENT, P_ON_ERROR} PID_p_mode_t;

struct pid_config {
  uint16_t sampleInterval, deadInterval;
  PID_direction_t direction;
  double Kp, Ki, Kd;
  PID_p_mode_t pMode;
  double outMin, outMax, iErrorMin, iErrorMax, deadband;
};

class PID {
public:
  //commonly used functions **************************************************************************
  // * Constructor, links the PID to the Input, Output, Setpoint and errors.
  PID(double* Input, double* Output, double* Setpoint,
      double* pError, double* iError, double* dError);
  void begin(void);
  bool begin(double Kp, double Ki, double Kd,
	     PID_p_mode_t pMode = P_ON_ERROR,
	     PID_direction_t Direction = FORWARD);
  bool begin(pid_config conf);
  void reset(void);
  inline void end(void) { active = false; };
  inline void SetPMode(PID_p_mode_t pMode) { c.pMode = pMode; };
  // * Sets tuning gains.
  bool SetTunings(double Kp, double Ki, double Kd);
  // * Sets the Direction, or "Action" of the controller.
  //   FORWARD means the output will increase when error is positive.
  //   REVERSE means the opposite.
  void SetDirection(PID_direction_t Direction);
  // * Sets the frequency, in milliseconds, with which the PID calculation is performed.
  //   Default is 100
  bool SetSampleInterval(uint16_t Interval);
  // * Sets the dead interval, in milliseconds in which no action occurs.
  //   Default is 0
  void SetDeadInterval(uint16_t Interval);
  // * Sets the deadband in which no action occurs.
  //   Default is 0
  void SetDeadBand(double Band);
  // * Bounds the output to a specific range, 0-255 by default,
  //   but it's likely the user will want to change this depending on the application
  bool SetOutputBounds(double Min, double Max);
  // * Bounds the integral error to a specific range, 0-255 by default, to avoid an integral windup.
  bool SetIErrorBounds(double Min, double Max);

  bool SetConfig(pid_config conf);
  // Check if sample interval has passed and perform the PID calculation.
  // should be called every time loop() cycles.
  // Returns true if new output is computed, false otherwise.
  inline bool check(uint32_t now) {
    if ((now - lastTime) >= c.sampleInterval) {
      lastTime = now;
      return PID::compute(now);
    }
    return false;
  };
  inline bool check(void) {
    unsigned long now = millis();
    return PID::check(now);
  };

  // This, as they say, is where the magic happens.
  // This function should only be called directly from timer that executes every sampleInterval.
  // Returns true if new output is computed, false otherwise.
  bool compute(uint32_t now);

  //Display functions ****************************************************************
  inline double GetKp() { return c.Kp; };  // These functions query the pid for interal values.
  inline double GetKi() { return c.Ki; };  //  they were created mainly for the pid front-end,
  inline double GetKd() { return c.Kd; };  // where it's important to know what is actually inside the PID.
  inline bool isActive() { return active; };
  inline PID_direction_t GetDirection() { return c.direction; };
  inline PID_p_mode_t GetPMode() { return c.pMode; };
  inline uint16_t GetSampleInterval() { return c.sampleInterval; };
  inline uint16_t GetDeadInterval() { return c.deadInterval; };
  inline double GetDeadBand() { return c.deadband; };
  //Convenience setters
  inline bool setKp(double Kp) {return SetTunings(Kp, c.Ki, c.Kd);}
  inline bool setKi(double Ki) {return SetTunings(c.Kp, Ki, c.Kd);}
  inline bool setKd(double Kd) {return SetTunings(c.Kp, c.Ki, Kd);}

private:
  bool active;
  pid_config c;
  // * (P)roportional,(I)ntegral, (D)erivative tuning gains
  double kp, ki, kd;

  // * Pointers to the Input, Output, Setpoint and error variables
  // This creates a hard link between the variables and the PID,
  // freeing the user from having to constantly ask or tell us
  // what these values are.  with pointers we'll just know.
  double *myInput, *myOutput, *mySetpoint, *mypError, *myiError, *mydError;
  unsigned long lastTime, lastChangeTime;
  double lastInput, lastOutput;
};
#endif
