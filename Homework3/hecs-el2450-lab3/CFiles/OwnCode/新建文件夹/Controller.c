/* Task 6: Rotation w */
switch (control) {
    case 0 : // rotation control
        if (error_r > )
        v = 0.0;
        theta_r = atan2(yg - y0, xg - x0) * 180 / PI;
//normalized_angle = fmod(theta_r - theta + 180, 360) - 180;
        w = K_psi * (theta_r - theta);
        left = int((2 * v - w) / 2);
        right = int((2 * v + w) / 2);
        if ()
/* Task 8: Translation v to origin */

//w = 0.0;

//v = K_w * (cos(theta * PI / 180) * (x0 - x) + sin(theta * PI / 180) * (y0 - y));

//left = int((2 * v - w) / 2);
//right = int((2 * v + w) / 2);

/* Task 10: Translation v to goal */

//w = 0.0;

//v = K_w * (cos(theta * PI / 180) * (xg - x) + sin(theta * PI / 180) * (yg - y));

//left = int((2 * v - w) / 2);
//right = int((2 * v + w) / 2);

/*Task 11*/

v =0.0;
theta_g =  atan2(yg - y, xg - x);
w = K_psi_2 * (sin(theta_g) * (x - x0 + p * cos(theta)) - cos(theta_g) * (y - y0 + p * sin(theta)));

left = int((2 * v - w) / 2);
right = int((2 * v + w) / 2);






}
