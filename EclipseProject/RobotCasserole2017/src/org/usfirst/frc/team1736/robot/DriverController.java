package org.usfirst.frc.team1736.robot;

/*
 *******************************************************************************************
 * Copyright (C) 2017 FRC Team 1736 Robot Casserole - www.robotcasserole.org
 *******************************************************************************************
 *
 * This software is released under the MIT Licence - see the license.txt
 *  file in the root of this repo.
 *
 * Non-legally-binding statement from Team 1736:
 *  Thank you for taking the time to read through our software! We hope you
 *   find it educational and informative! 
 *  Please feel free to snag our software for your own use in whatever project
 *   you have going on right now! We'd love to be able to help out! Shoot us 
 *   any questions you may have, all our contact info should be on our website
 *   (listed above).
 *  If you happen to end up using our software to make money, that is wonderful!
 *   Robot Casserole is always looking for more sponsors, so we'd be very appreciative
 *   if you would consider donating to our club to help further STEM education.
 */

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
