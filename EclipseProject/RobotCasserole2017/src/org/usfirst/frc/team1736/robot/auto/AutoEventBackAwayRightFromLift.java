package org.usfirst.frc.team1736.robot.auto;

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoEvent;
import org.usfirst.frc.team1736.lib.FalconPathPlanner.PathPlannerAutoEvent;
import org.usfirst.frc.team1736.robot.DriveTrain;

public class AutoEventBackAwayRightFromLift extends AutoEvent {
	PathPlannerAutoEvent driveSideways;

	private static final double[][] waypoints = new double[][] {
		{0, 0, 0},
		{-5, 0, 0},
		{-5, -2, 0},
		{-5, -8, 0},
		{-5, -10, 0},
		{0, -10, 0},
		{8, -10, 0}
	};

	private static final double time = 6.5;

	public AutoEventBackAwayRightFromLift() {
		driveSideways = new PathPlannerAutoEvent(waypoints, time,
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
		driveSideways.userUpdate();
	}

	@Override
	public void userStart() {
		driveSideways.userStart();
	}

}
