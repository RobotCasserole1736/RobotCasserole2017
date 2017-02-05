package org.usfirst.frc.team1736.robot.auto;

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoEvent;
import org.usfirst.frc.team1736.lib.FalconPathPlanner.PathPlannerAutoEvent;
import org.usfirst.frc.team1736.robot.DriveTrainWheelSpeedPI;

public class AutoEventDriveForward extends AutoEvent {
	PathPlannerAutoEvent driveForward;
	private static final double[][] waypoints = new double[][]{
		{0,0,0},
		{0,9,0}
	};
	private static final double time = 10;
	public AutoEventDriveForward(DriveTrainWheelSpeedPI frontLeftAutonCtrl,DriveTrainWheelSpeedPI frontRightAutonCtrl,
			DriveTrainWheelSpeedPI rearLeftAutonCtrl,DriveTrainWheelSpeedPI rearRightAutonCtrl) {
		driveForward = new PathPlannerAutoEvent(waypoints,time,frontLeftAutonCtrl,frontRightAutonCtrl,rearLeftAutonCtrl,rearRightAutonCtrl);
	}

	@Override
	public void userUpdate() {
		driveForward.userUpdate();

	}

	@Override
	public void userForceStop() {
		driveForward.userForceStop();

	}

	@Override
	public boolean isTriggered() {
		return driveForward.isTriggered();
	}

	@Override
	public boolean isDone() {
		return driveForward.isDone();
	}
	
}
