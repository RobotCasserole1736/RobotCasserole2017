package org.usfirst.frc.team1736.robot.auto;

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoEvent;
import org.usfirst.frc.team1736.lib.FalconPathPlanner.PathPlannerAutoEvent;
import org.usfirst.frc.team1736.robot.DriveTrainWheelSpeedPI;

public class AutoEventBackAwayFromLift extends AutoEvent{
	PathPlannerAutoEvent driveSideways;
	
	private static final double[][] waypoints = new double[][]{
		{0,0,0},
		{-3,0,0}
	}	;
	
	private static final double time = 3;
	public AutoEventBackAwayFromLift(DriveTrainWheelSpeedPI frontLeftAutonCtrl,DriveTrainWheelSpeedPI frontRightAutonCtrl,
			DriveTrainWheelSpeedPI rearLeftAutonCtrl,DriveTrainWheelSpeedPI rearRightAutonCtrl) {
		driveSideways = new PathPlannerAutoEvent(waypoints,time,frontLeftAutonCtrl,frontRightAutonCtrl,rearLeftAutonCtrl,rearRightAutonCtrl);
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
