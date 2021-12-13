/* Task 6: Rotation w */

theta_r = atan2(yg - y, xg - x) * 180 / PI;

v = 0.0;

//normalized_angle = fmod(theta_r - theta + 180, 360) - 180;

w = K_psi * (theta_r - theta);

left = int((2 * v - w) / 2);
right = int((2 * v + w) / 2);

/* Task 8: Translation v to origin */

w = 0.0;

v = K_w * (cos(theta * PI / 180) * (x0 - x) + sin(theta * PI / 180) * (y0 - y));

left = int((2 * v - w) / 2);
right = int((2 * v + w) / 2);

/* Task 10: Translation v to goal */

w = 0.0;

v = K_w * (cos(theta * PI / 180) * (xg - x) + sin(theta * PI / 180) * (yg - y));

left = int((2 * v - w) / 2);
right = int((2 * v + w) / 2);
