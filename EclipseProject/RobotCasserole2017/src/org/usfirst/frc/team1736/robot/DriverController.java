package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.HAL.Xbox360Controller;

public class DriverController extends Xbox360Controller {
	
	private static DriverController controller = null;
	
	
	public static DriverController getInstance()
	{
		if(controller == null)
			controller = new DriverController(0);
		return controller;
	}

	private DriverController(int joystick_id) {
		super(joystick_id);
	}

}
