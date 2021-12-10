#ifndef PROJECT_VECTOR2D_H
#define PROJECT_VECTOR2D_H

#include <iostream>
#include<cmath>


class VECTOR2D
{

public:
    double x, y;  // Her eksen boyunca bileþen (kartezyen)

    //3d vector oluþturma
    VECTOR2D(double initial_x = 0.0, double initial_y = 0.0)
    {
        x = initial_x;
        y = initial_y;
    }

    double getMagnitude() // Vektör büyüklüðü
    {
        return sqrt(x*x + y*y);
    }
    double getMagnitudeSquared() // Vektörün büyüklüðünün karesi
    {
        return (x*x + y*y);
    }

    // overloading -, *, ^, +, ==, +=
    VECTOR2D operator-(const VECTOR2D& vec) // Ýki vektörü çýkarma
    {
        return VECTOR2D(x - vec.x, y - vec.y);
    }
    double operator*(const VECTOR2D& vec)						// Ýki vektörün çarpýmý
    {
        return x*vec.x + y*vec.y;
    }
    VECTOR2D operator^(double scalar)						// Bir vektör ve bir skalerin çarpýmý
    {
        return VECTOR2D(x*scalar, y*scalar);
    }
    VECTOR2D operator+(const VECTOR2D& vec)						// 2 vektör ekleme
    {
        return VECTOR2D(x + vec.x, y + vec.y);
    }
    bool operator==(const VECTOR2D& vec)						// 2 vektörü karþýlaþtýrma
    {
        if (x==vec.x && y==vec.y)
            return true;
        return false;
    }
    void operator+=(const VECTOR2D& vec)
    {
        x += vec.x;
        y += vec.y;
    }
};

#endif //PROJECT_VECTOR2D_H
