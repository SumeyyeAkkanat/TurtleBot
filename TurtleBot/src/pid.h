
#ifndef PID_H
#define PID_H
class PIDImpl;
class PID
{
public:
    // Kp -  Orantýlý kazanç
    // Ki -  Ýntegral kazanc
    // Kd -  Türevsel kazanç
    // dt -  Döngü  aralýðý süresi
    // max - Ýþlenen deðiþkenin maksimum deðeri
    // min - Ýþlenen deðiþkenin mminimumdeðeri
    PID( double dt, double max, double min, double Kp, double Kd, double Ki );

    // Bir ayar noktasý ve mevcut iþlem deðeri verilen iþlenmiþ deðiþkeni döndürür
    double calculate( double setpoint, double pv );
    ~PID();

private:
    PIDImpl *pimpl;
};


#endif //PID_H
