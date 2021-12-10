#include "pid.h"
#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

class PIDImpl
{
public:
    PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
    ~PIDImpl();
    double calculate( double setpoint, double pv );

private:
    double _dt;
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;
};


PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID()
{
    delete pimpl;
}


// Uygulama
PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
        _dt(dt),
        _max(max),
        _min(min),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki),
        _pre_error(0),
        _integral(0)
{
}

double PIDImpl::calculate( double setpoint, double pv )
{

    //Hata
    double error = setpoint - pv;

    // Orantýlý kýsým
    double Pout = _Kp * error;

    // 	Ýntegral kýsmý
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Türev kýsmý
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Total output
    double output = Pout + Iout + Dout;

    // max/min ile sýnýrlama
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Hatayý önceki hataya kaydet
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}
