package org.usfirst.frc.team1736.robot;

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