#ifndef EARTH_POSITION_FILTER_H
#define EARTH_POSITION_FILTER_H

#include <Arduino.h>

class EarthPositionFilter
{
  protected:
    // state variables (position, velocity), (x)
    double pos_m;
    double vel_m_per_sec;
    // state covariance matrix, (P)
    double P[2][2];
    // state transition matrix, (F)
    // { {1, dt}, {0, 1} }
    double dt;
    // control input matrix, (B)
    // { 0.5*a*dt^2, dt }
    double dt_dt;
    // process noise matrix, (Q)
    // { {0.25*dt^4, 0.5*dt^3}, {0.5*dt^3, dt^2} } * process_variance
    double dt_dt_dt_2, dt_dt_dt_dt_4;
    double proc_var;
    // measurement matrix, (H)
    // { 1, 0 }
    // measurement noise matrix, (R)
    double R;

  public:
    EarthPositionFilter() :
      pos_m{0.0}, vel_m_per_sec{0.0},
      P{100.0, 0.0, 0.0, 100.0},
      proc_var{9.0}, R{25.0} {}

    // getter functions
    double get_pos_m();
    double get_vel_m_per_sec();
    void   get_P(double (&)[2][2]);
    double get_P(uint8_t, uint8_t);
    double get_deltat();
    double get_proc_var();
    double get_R();
    // setter functions
    void   set_pos_m(double);
    void   set_vel_m_per_sec(double);
    void   set_P(const double (&&)[2][2]);
    void   set_deltat(double);
    void   set_proc_var(double);
    void   set_R(double);

    // predict and update functions
    void predict(double);
    void update(double);
};

#endif  // EARTH_POSITION_FILTER_H
