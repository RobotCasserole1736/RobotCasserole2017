package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

public class VisionAlignment {
	int millisec = 0;
	double matchAngle = 0;
	double matchDist = 0;
	VisionAlignAnglePID anglePID;
	VisionAlignDistPID distPID;
	
	//PID Gains 
	Calibration angle_Kp = new Calibration("Alignment Angle Control Kp", 1.0/40.0, 0.0, 1.0);
	Calibration angle_Ki = new Calibration("Alignment Angle Control Ki", 1.0/40.0/(3*50), 0.0, 1.0);
	Calibration angle_Kd = new Calibration("Alignment Angle Control Kd", 0.0, 0.0, 1.0);
	Calibration dist_Kp = new Calibration("Alignment Dist Control Kp", 1.0/7.0, 0.0, 1.0);
	Calibration dist_Ki = new Calibration("Alignment Dist Control Ki", 1.0/7.0/(3*50), 0.0, 1.0);
	Calibration dist_Kd = new Calibration("Alignment Dist Control Kd", 0.0, 0.0, 1.0);
	public VisionAlignment(){
		//set goal angle and distance from where we can fire and hit the target
		matchAngle = 75;
		matchDist = 25;
		anglePID = new VisionAlignAnglePID(angle_Kp.get(), angle_Ki.get(), angle_Kd.get());
		distPID = new VisionAlignDistPID(dist_Kp.get(), dist_Ki.get(), dist_Kd.get());
	}

	public void GetAligned(){
		boolean alignPossible = false;
		boolean onTarget = false;
		double rotateCmdCalc = 0;
		double FwdRevCmdCalc = 0;
		// Get from RobotState or otherwise calculate angle and distance to target
		double curAngle = 0;
		double curDistance = 0;
		
		// Figure out if alignment is possible
		// is | delta angle | < some tolerance and is | delta distance | < some tolerance
		
		// if alignment is possible and alignment is desired
		// then on the first loop reset the integrators on both angle and distance pids
		
		/* if angle and distance are correct then
		 indicate that we are on target */
		
		// drive outputs
		
		
		//Check is Vision Alignment function is enabled
		if(RobotState.visionAlignmentDesiried){
			//Calculate drive settings using angle and distance
			if(curAngle == this.matchAngle && curDistance == this.matchDist)
			{
				onTarget = true;
			}
		 
		 
			//Update outputs with drive state
			if (alignPossible && false == onTarget){
				RobotState.visionDtFwdRevCmd = FwdRevCmdCalc; //Forward/reverse command from vision alignment system to get the robot aligned for a shot. (-1 = full reverse, 0 = stop, 1 = full forward)
				RobotState.visionDtRotateCmd = rotateCmdCalc; //Rotation command from vision alignment system to get the robot aligned for a shot. (-1 = full reverse, 0 = stop, 1 = full forward)
			}
			RobotState.visionAlignmentPossible = alignPossible; //True if the vision system is capable of auto-aligning the robot, false if not. False should happen if the vision system is offline, or no target can be seen.
			RobotState.visionAlignmentOnTarget = onTarget; //True if we know for sure the robot is aligned sufficiently to make shots, false if not.
		}
	}
}