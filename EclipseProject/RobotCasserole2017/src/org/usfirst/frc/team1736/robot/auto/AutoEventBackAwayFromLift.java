package org.usfirst.frc.team1736.robot.auto;

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoEvent;
import org.usfirst.frc.team1736.lib.FalconPathPlanner.PathPlannerAutoEvent;
import org.usfirst.frc.team1736.robot.DriveTrain;
import org.usfirst.frc.team1736.robot.DriveTrainWheelSpeedPI;

public class AutoEventBackAwayFromLift extends AutoEvent{
	PathPlannerAutoEvent driveSideways;
	
	private static final double[][] waypoints = new double[][]{
		{0,0,0},
		{-3,0,0}
	}	;
	
	private static final double time = 3;
	public AutoEventBackAwayFromLift() {
		driveSideways = new PathPlannerAutoEvent(waypoints,time,
				DriveTrain.getInstance().getFrontLeftCTRL(), DriveTrain.getInstance().getFrontRightCTRL(), 
				DriveTrain.getInstance().getRearLeftCTRL(), DriveTrain.getInstance().getRearRightCTRL());
	}
	
	public void userForceStop() {
		driveSideways.userForceStop();
	}
	public boolean isTriggered() {
		return driveSideways.isTriggered();
	}
	public boolean isDone() {
		return driveSideways.isDone();
	}

	@Override
	public void userUpdate() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void userStart() {
		driveSideways.userStart();		
	}
	
}
