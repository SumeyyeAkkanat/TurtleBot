#ifndef PROJECT_VECTOR2D_H
#define PROJECT_VECTOR2D_H

#include <iostream>
#include<cmath>


class VECTOR2D
{

public:
    double x, y;  // Her eksen boyunca bile�en (kartezyen)

    //3d vector olu�turma
    VECTOR2D(double initial_x = 0.0, double initial_y = 0.0)
    {
        x = initial_x;
        y = initial_y;
    }

    double getMagnitude() // Vekt�r b�y�kl���
    {
        return sqrt(x*x + y*y);
    }
    double getMagnitudeSquared() // Vekt�r�n b�y�kl���n�n karesi
    {
        return (x*x + y*y);
    }

    // overloading -, *, ^, +, ==, +=
    VECTOR2D operator-(const VECTOR2D& vec) // �ki vekt�r� ��karma
    {
        return VECTOR2D(x - vec.x, y - vec.y);
    }
    double operator*(const VECTOR2D& vec)						// �ki vekt�r�n �arp�m�
    {
        return x*vec.x + y*vec.y;
    }
    VECTOR2D operator^(double scalar)						// Bir vekt�r ve bir skalerin �arp�m�
    {
        return VECTOR2D(x*scalar, y*scalar);
    }
    VECTOR2D operator+(const VECTOR2D& vec)						// 2 vekt�r ekleme
    {
        return VECTOR2D(x + vec.x, y + vec.y);
    }
    bool operator==(const VECTOR2D& vec)						// 2 vekt�r� kar��la�t�rma
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
