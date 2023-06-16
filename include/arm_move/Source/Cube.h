#include <iostream>

/**
 * Defines the forbidden zone
 * 
 * It receives two corners of the area  
 */
class Cube
{
private:
    double x_min, y_min, z_min, x_max, y_max, z_max;
public:
    Cube(double _x_min, double _y_min, double _z_min, double _x_max, double _y_max, double _z_max);
    
    bool check_point_inside(double x, double y, double z);
};

Cube::Cube(double _x_min, double _y_min, double _z_min, double _x_max, double _y_max, double _z_max): x_min(_x_min), y_min(_y_min), z_min(_z_min), x_max(_x_max), y_max(_y_max), z_max(_z_max)
{
}

/**
 * Check if a point is inside the forbiddeen zone
 * 
 */
bool Cube::check_point_inside(double x, double y, double z)
{
    if(abs(x_min) < abs(x) && abs(x) < abs(x_max) && abs(y_min) < abs(y) && abs(y) < abs(y_max) && abs(z_min) < abs(z) && abs(z) < abs(z_max))
        return true;
    
    return false;
}

#pragma once