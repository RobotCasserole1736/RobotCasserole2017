package org.usfirst.frc.team1736.robot.auto;

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoEvent;
import org.usfirst.frc.team1736.robot.ShotControl;
import org.usfirst.frc.team1736.robot.ShotControl.ShooterStates;
import org.usfirst.frc.team1736.robot.VisionAlignment;

/**
 * Auto event to spool up the shooter and feed balls
 * no attempt to align with vision or anything
 *
 */
public class AutoEventShootNoVision extends AutoEvent {
	
	//Vision Alignment Control
	VisionAlignment visionAlignCTRL;
	
	//Shooter control
	ShotControl shotCTRL;
	
	@Override
	public void userUpdate() {
		// TODO Auto-generated method stub
		visionAlignCTRL = VisionAlignment.getInstance();
		visionAlignCTRL.setVisionAlignmentDesired(false); 
		shotCTRL = ShotControl.getInstance();
		shotCTRL.setDesiredShooterState(ShooterStates.SHOOT);
	}

	@Override
	public void userForceStop() {
		// TODO Auto-generated method stub
		visionAlignCTRL = VisionAlignment.getInstance();
		visionAlignCTRL.setVisionAlignmentDesired(false); 
		shotCTRL = ShotControl.getInstance();
		shotCTRL.setDesiredShooterState(ShooterStates.NO_SHOOT);
	}

	@Override
	public boolean isTriggered() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isDone() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void userStart() {
		// TODO Auto-generated method stub
		
	}
}
