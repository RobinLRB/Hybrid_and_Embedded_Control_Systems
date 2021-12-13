#include <stdio.h>


double w ;
double v ;
int K_psi = 6;
int theta_r;
int theta_g;
int K_w = 15;
int K_psi_2 = 5;
int p;
int control = 0;        //'0' states that
int state = 0;
double error_r = 0.5;   //Error that can be accepted finally for the rotation control
double error_g2g =0.5;  //Error that can be accepted finally for the go_to_goal control
double delta_r;
double delta_g2g;
