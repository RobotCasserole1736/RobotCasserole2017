package org.usfirst.frc.team1736.robot.auto;

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoEvent;
import org.usfirst.frc.team1736.robot.GearControl;

import edu.wpi.first.wpilibj.Timer;

public class AutoEventOpenGearMechanism extends AutoEvent {
	Timer timer;
	public AutoEventOpenGearMechanism(){
		timer = new Timer();
		timer.start();
		GearControl.getInstance().openGearSolenoid();
	}
	
	@Override
	public void userUpdate() {
		
	}

	@Override
	public void userForceStop() {
		
	}

	@Override
	public boolean isTriggered() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isDone() {
		if(timer.get() > .5)
			return true;
		return false;
	}

	@Override
	public void userStart() {
		// TODO Auto-generated method stub
		
	}
	
	
}
