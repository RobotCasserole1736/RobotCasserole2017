package org.usfirst.frc.team1736.robot.auto;

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoEvent;
import org.usfirst.frc.team1736.lib.FalconPathPlanner.PathPlannerAutoEvent;
import org.usfirst.frc.team1736.robot.DriveTrain;
import org.usfirst.frc.team1736.robot.DriveTrainWheelSpeedPI;
import org.usfirst.frc.team1736.robot.ShotControl;
import org.usfirst.frc.team1736.robot.ShotControl.ShooterStates;

public class AutoEventMoveFromRed extends AutoEvent {
	PathPlannerAutoEvent driveForward;
	ShotControl shotCTRL;
	
	private static final double[][] waypoints = new double[][]{
		{0,0,0},
		{0,9,0}
	};
	private static final double time = 10;
	public AutoEventMoveFromRed() {
		driveForward = new PathPlannerAutoEvent(waypoints,time,
				DriveTrain.getInstance().getFrontLeftCTRL(), DriveTrain.getInstance().getFrontRightCTRL(), 
				DriveTrain.getInstance().getRearLeftCTRL(), DriveTrain.getInstance().getRearRightCTRL());
	}

	@Override
	public void userUpdate() {
		driveForward.userUpdate();
		shotCTRL = ShotControl.getInstance();
		shotCTRL.setDesiredShooterState(ShooterStates.PREP_TO_SHOOT);

	}

	@Override
	public void userForceStop() {
		driveForward.userForceStop();
		shotCTRL = ShotControl.getInstance();
		shotCTRL.setDesiredShooterState(ShooterStates.NO_SHOOT);

	}

	@Override
	public boolean isTriggered() {
		return driveForward.isTriggered();
	}

	@Override
	public boolean isDone() {
		return driveForward.isDone();
	}

	@Override
	public void userStart() {
		driveForward.userStart();
	}
	
}
