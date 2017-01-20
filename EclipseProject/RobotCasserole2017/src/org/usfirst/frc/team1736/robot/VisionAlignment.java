package org.usfirst.frc.team1736.robot;

public class VisionAlignment {
	int millisec = 0;
	double matchAngle = 0;
	double matchDist = 0;
	
	public VisionAlignment(){
		//set goal angle and distance from where we can fire and hit the target
		matchAngle = 75;
		matchDist = 25;
	}

	public void GetAligned(){
		boolean alignPossible = false;
		boolean onTarget = false;
		double rotateCmdCalc = 0;
		double FwdRevCmdCalc = 0;
		// Get from RobotState or otherwise calculate angle and distance to target
		double curAngle = 0;
		double curDistance = 0;
		
		
		//Check is Vision Alignment function is enabled
		if(RobotState.visionAlignmentActive){
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
			RobotState.visionAlignmentPossible = alignPossible; //True if the vision system is capiable of auto-aligning the robot, false if not. False should happen if the vision system is offline, or no target can be seen.
			RobotState.visionAlignmentOnTarget = onTarget; //True if we know for sure the robot is aligned sufficiently to make shots, false if not.
		}
	}
}