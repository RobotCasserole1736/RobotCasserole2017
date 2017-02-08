package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.HAL.Xbox360Controller;

public class DriverController extends Xbox360Controller {
	
	private static DriverController controller = null;
	private boolean airCompState = true;
	
	public static synchronized DriverController getInstance()
	{
		if(controller == null)
			controller = new DriverController(0);
		return controller;
	}

	private DriverController(int joystick_id) {
		super(joystick_id);
	}
	
	
	public double getFwdRevCmd()
	{
		return LStick_Y();
	}
	
	public double getStrafeCmd()
	{
		return LStick_X();
	}
	
	public double getRotateCmd()
	{
		return RStick_X();
	}
	
	public boolean getGearCamAlign()
	{
		return B();
	}
	
	public boolean getIntakeCamAlign()
	{
		return X();
	}
	
	public boolean getShooterCamAlign()
	{
		return Y();
	}
	
	public boolean getGyroReset()
	{
		return DPadUp();
	}
	
	public boolean getGyroReset90()
	{
		return DPadRight();
	}
	
	public boolean getGyroReset180()
	{
		return DPadDown();
	}
	
	public boolean getGyroReset270()
	{
		return DPadLeft();
	}
	
	public void updateAirCompEnabled()
	{
		if(StartButton())
			airCompState = true;
		if(BackButton())
			airCompState = false;
	}
	
	public boolean getAirCompEnableCmd()
	{
		return airCompState;
	}
	
	public boolean getAlignDesired()
	{
		return RB();
	}

}
