#include <math.h>

float esc_boundary(float valeur) {
    if (valeur < 1000.0f) {
        return 1000.0f;
    } 
    if (valeur > 1800.0f) {
        return 1800.0f;
    }
    return valeur;
}

float throttle_boundary(float throttle){
    if (throttle < 1000.0f){
        return 1000.0f;
    }
    if (throttle > 1500.0f){
        return 1500.0f;
    }
    return throttle;
}

float correction_motorB(float esc) {
    float esc_boundary = esc;
    if(esc >1800){
        esc_boundary = 1800;
    }
    if (esc < 1000){
        esc_boundary = 1000;
    }

    float a = 1.9821e-4;   // Example coefficient for x^2
    float b = 2.3963e-1;    // Example coefficient for x
    float c = 538.83;

    return a*pow(esc_boundary, 2) + b*pow(esc_boundary, 1) + c;
}

// [-7.77285340e-25  1.03941290e-20 -6.10665402e-17  2.06678591e-13
//  -4.43572089e-10  6.25193814e-07 -5.77738878e-04  3.36839203e-01
//  -1.12127832e+02  1.61768872e+04]