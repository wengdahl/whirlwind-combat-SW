#include "position.h"
#include <cstdio>
#include <math.h>
#include <time.h>

#define DISTANCE_TO_SENSOR 0.03 // meters
#define INV_DIST (1.0/DISTANCE_TO_SENSOR)

clock_t start = 0;

static float omega = 0;
static int theta = 0; // in units of uDegrees due to short tick duration

// Average last NU
#define ALPHA 0.8
float alpha_filt = 0.0; // rad/s


// Centripetal accel in m/s^2
void updateAccel(float a_centr, int tare){
    // Update filtered value
    alpha_filt = ALPHA*a_centr + (1-ALPHA)*alpha_filt;

    // Calculate avg rotational speed
    omega = sqrt(abs((int)(alpha_filt*INV_DIST)));

    if(alpha_filt < 0) omega = -omega;

    // a = v^2/r
    // sqrt(a*r) = v
    // v = w*r
}

// T_rot in microseconds
void updateHeading(int t_rot){
    theta += (omega*t_rot*(57.2958));// 12 - guess and check correction factor

    
    theta = theta %(360*1000000);
}

int getOmegaAvg(){
    return 57.2958*omega;
}

int getHeadingAngle(){
    return theta/1000000;
}
