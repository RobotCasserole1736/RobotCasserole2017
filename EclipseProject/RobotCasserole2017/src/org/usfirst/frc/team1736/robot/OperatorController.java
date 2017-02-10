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

public class OperatorController extends Xbox360Controller {
	
	private static OperatorController controller = null;
	
	public static synchronized OperatorController getInstance()
	{
		if(controller == null)
			controller = new OperatorController(1);
		return controller;
	}

	private OperatorController(int joystick_id) {
		super(joystick_id);
	}
	
	public boolean getGearSolenoidCmd()
	{
		return RTrigger() > 0.5;
	}
	
	public double getClimbSpeedCmd()
	{
		return LStick_X();
	}
	
	public boolean getIntakeDesiredCmd()
	{
		return LB();
	}
	
	public boolean getEjectDesiredCmd()
	{
		return B();
	}
	
	public boolean getHopperFwdOverride()
	{
		return StartButton();
	}
	
	public boolean getHopperRevOverride()
	{
		return BackButton();
	}

}
