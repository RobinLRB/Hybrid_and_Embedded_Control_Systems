v = 0.0;
		theta_r = atan2(yg - y, xg - x) * 180 / PI;
		//normalized_angle = fmod(theta_r - theta + 180, 360) - 180;
		w = K_psi * (theta_r - theta);

		left = int((2 * v - w) / 2);
		right = int((2 * v + w) / 2);
		

Serial.print("theta_r = ");
Serial.print(theta_r , 1);
Serial.print(".\n");
