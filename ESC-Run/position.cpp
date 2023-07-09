#include "position.h"
#include <cstdio>
#include <math.h>
#include <time.h>

#define NUM_MEASURE 50
#define DISTANCE_TO_SENSOR 0.03
#define INV_DIST (1/DISTANCE_TO_SENSOR)

clock_t start = 0;

static float omega = 0;
static int theta = 0;

// Average last NUM_MEASURE samples
int alpha_array[NUM_MEASURE];
int array_ind = 0;

void updateAccel(float a_centr, int tare){
    // Update array - do this before normalization to squash noise
    alpha_array[array_ind] = a_centr;
    array_ind++;
    array_ind = array_ind % NUM_MEASURE;

    int alpha_sum = 0;
    for(int i =0; i < NUM_MEASURE; i++)
        alpha_sum += alpha_array[i];
    int alpha = alpha_sum*tare/(NUM_MEASURE*10);
    // Calculate avg rotational speed
    omega = sqrt(abs((int)(alpha*INV_DIST)));
}

void updateHeading(int t_rot){
    theta += 6*omega*t_rot*(57.2958)/(1000*1000);// 12 - guess and check correction factor
    theta = theta %360;
}

int getOmegaAvg(){
    return 57.2958*omega;
}

int getHeadingAngle(){
    return theta;
}
