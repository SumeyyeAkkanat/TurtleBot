
#ifndef PID_H
#define PID_H
class PIDImpl;
class PID
{
public:
    // Kp -  Orant�l� kazan�
    // Ki -  �ntegral kazanc
    // Kd -  T�revsel kazan�
    // dt -  D�ng�  aral��� s�resi
    // max - ��lenen de�i�kenin maksimum de�eri
    // min - ��lenen de�i�kenin mminimumde�eri
    PID( double dt, double max, double min, double Kp, double Kd, double Ki );

    // Bir ayar noktas� ve mevcut i�lem de�eri verilen i�lenmi� de�i�keni d�nd�r�r
    double calculate( double setpoint, double pv );
    ~PID();

private:
    PIDImpl *pimpl;
};


#endif //PID_H
