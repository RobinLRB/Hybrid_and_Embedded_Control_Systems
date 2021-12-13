theta_r = atan2(yg - y0, xg - x0) * 180 / PI;

error_rotation = theta_r - theta;
if (state == 1 && abs(error_rotation) <= epsilon_rotation)
{
    state = 2;
}

error_go_to_goal = cos(theta * PI / 180) * (xg - x) + sin(theta * PI / 180) * (yg - y);
if (state == 2 && abs(error_go_to_goal) <= epsilon_go_to_goal)
{
    state = 3;
}

Serial.print("Current state:");
Serial.print(state, DEC);
Serial.print("\nError rotation:");
Serial.print(error_rotation, 3);
Serial.print("\nError go to goal:");
Serial.print(error_go_to_goal, 3);

if (state == 1)
{
    w = K_psi * (theta_r - theta);
    v = K_w * (cos(theta * PI / 180) * (x0 - x) + sin(theta * PI / 180) * (y0 - y));
}
else if (state == 2)
{
    w = K_psi * (sin(theta_r * PI / 180) * (x + p * cos(theta * PI / 180) - x0) -
                 cos(theta_r * PI / 180) * (y + p * sin(theta * PI / 180) - y0));
    v = K_w * (cos(theta * PI / 180) * (xg - x) + sin(theta * PI / 180) * (yg - y));
}
else if (state == 3)
{
    w = 0;
    v = 0;
}

left = int((2 * v - w) / 2);
right = int((2 * v + w) / 2);
