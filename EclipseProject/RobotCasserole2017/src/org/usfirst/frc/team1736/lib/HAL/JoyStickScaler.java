package org.usfirst.frc.team1736.lib.HAL;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

public class JoyStickScaler {
	static double input ;
	static Calibration exponent1 = new Calibration("Joystick Exponential Scale", 2.0, 1.0, 10.0);
	static int exponent = Math.getExponent(exponent1.get());
	static double output;
	
	public static double joyscale(double input){
	
	
	
	if (input>=0) {
	    output = Math.pow(input, exponent);
	}
	else{
	    output = -Math.pow(-input, exponent);
	}
	return output;
	}
	
	
	
	
	
	
}
//-((-input)^(exponent))