package org.usfirst.frc.team1736.lib.HAL;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

public class JoyStickScaler {
	static double input ;
	static Calibration exponent1 = new Calibration("Joystick Exponential Scale", 2.0, 1.0, 10.0);
	
	
	public static double joyscale(double input){
		double output;
	
	
	
	if (input>=0) {
	    output = Math.pow(input, exponent1.get());
	}
	else{
	    output = -Math.pow(-input, exponent1.get());
	}
	return output;
	}
	
	
	
	
	
	
}
//-((-input)^(exponent))