package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class GearControl {
	private static GearControl gearControl = null;
	private Solenoid gearSolenoid = null;
	
	public static GearControl getInstance(){
		if (gearControl == null)
			gearControl = new GearControl();
		return gearControl;
	}

	private GearControl(){
		gearSolenoid = new Solenoid(RobotConstants.GEAR_SOLENOID_PORT);
	}
	
	public void openGearSolenoid(){
		gearSolenoid.set(true);
	}
	
	public void closeGearSolenoid(){
		gearSolenoid.set(false);
	}
	
	public boolean isSolenoidOpen(){
		return gearSolenoid.get();
	}
	
	public void update(){
		if(OperatorController.getInstance().getGearSolenoidCmd())
			openGearSolenoid();
		else
			closeGearSolenoid();
	}
}

