package org.usfirst.frc.team1736.robot;


public class RobotState {
	
	// Driver commands (derived from driver's Xbox360 controller)
	//gyro align commands
	static boolean gyroAlignRight = false;
	static boolean gyroAlignLeft = false;
	static boolean gyroAlignUp = false;
	static boolean gyroAlignDown = false;
	
	// Autonomous drivetrain commands (derived from autonomous path planning routines)
	// Positive means the wheel rotates forward, negative means it rotates backward
	static double autonDtfrontLeftWheelVelocityCmd_rpm= 0;
	static double autonDtfrontRightWheelVelocityCmd_rpm = 0;
	static double autonDtrearLeftWheelVelocityCmd_rpm = 0;
	static double autonDtrearRightWheelVelocityCmd_rpm = 0;
	
	//Vision Target Qualification System (Outputs from vision targeting system)
	static boolean visionOnline = false; //True if information is being received from the coprocessor, false if the system is down. All other variables should only be considered valid if this field is true.
	static double  visionFrameCounter = 0; //Indicates the frame we are on. Increase monotomically by one for each new processed frame.
	static boolean visionTargetFound = false; //True if a qualified (valid) target is found in the most recent camera image, false if not
	static double  visionTargetOffset_deg = 0; //Offset from center of image of the located target. 0 degrees means the target is right in front of the camera, positive degrees means the target is to the right, negative degrees means the target is to the left.
	static double  visionEstTargetDist_ft = 0; //Estimated distance from the camera lens to the top vision target (hypotenuse)
	static double  visionTopTgtXPixelPos = 0; //pixel location of the X target coordinate
	static double  visionTopTgtYPixelPos = 0; //pixel location of the Y target coordinate
	static double  visionHeuristicVal = 0;    //Evaluation of how good the located target is. Small numbers mean it is likely the target, large numbers mean it does not match what a target should look like much at all.
	static double  visionEstCaptureTime = 0; //Estimated capture time (WRT getFPGATimestamp's timebase) of the present processed image. Should account for processing time (variable, but reported by coprocessor) as well as assumed fixed data transport times over ethernet.
	static double  visionCoProcessorFPS = 0; //system processing Frames-per-second reported by the coprocessor
	static double  visionCoProcessorCPULoad_pct = 0; //Coprocessor reported processor load
	static double  visionCoProcessorMemLoad_pct = 0; //Coprocessor reported memory load
	
	static double  visionGyroAngleAtLastFrame = 0; //what did the gyro read when the last frame was captured from vision processing
	static double  visionGyroAngleDesiredAtLastFrame = 0; //based on most recent vision processing results, what angle should we go to?
	public static double visionDistanceAtLastFrame = 0; //What did the encoders read for fwd/rev location when the last frame was captured from visino processing
	public static double visionDistanceDesiredAtLastFrame = 0; //Based on the most recent vision processing results, what distance should we go to?
	
	//Vision Align system (mostly outputs from vision-based alignment system)
	static double  visionDtFwdRevCmd = 0; //Forward/reverse command from vision alignment system to get the robot aligned for a shot. (-1 = full reverse, 0 = stop, 1 = full forward)
	static double  visionDtRotateCmd = 0; //Rotation command from vision alignment system to get the robot aligned for a shot. (-1 = full reverse, 0 = stop, 1 = full forward)

	//Getters and Setters for select globals
	public static boolean isVisionOnline() {
		return visionOnline;
	}
	public static boolean isVisionTargetFound() {
		return visionTargetFound;
	}
	public static double getVisionTargetOffset_deg() {
		return visionTargetOffset_deg;
	}
	public static double getVisionEstTargetDist_ft() {
		return visionEstTargetDist_ft;
	}
	public static double getVisionEstCaptureTime() {
		return visionEstCaptureTime;
	}
	public static double getVisionCoProcessorFPS() {
		return visionCoProcessorFPS;
	}
	public static double getVisionCoProcessorCPULoad_pct() {
		return visionCoProcessorCPULoad_pct;
	}
	public static double getVisionCoProcessorMemLoad_pct() {
		return visionCoProcessorMemLoad_pct;
	}
	public static double getVisionDtFwdRevCmd() {
		return visionDtFwdRevCmd;
	}
	public static double getVisionDtRotateCmd() {
		return visionDtRotateCmd;
	}
	public static boolean isGyroAlignRight() {
		return gyroAlignRight;
	}
	public static boolean isGyroAlignLeft() {
		return gyroAlignLeft;
	}
	public static boolean isGyroAlignUp() {
		return gyroAlignUp;
	}
	public static boolean isGyroAlignDown() {
		return gyroAlignDown;
	}
	public static double getAutonDtfrontLeftWheelVelocityCmd_rpm() {
		return autonDtfrontLeftWheelVelocityCmd_rpm;
	}
	public static double getAutonDtfrontRightWheelVelocityCmd_rpm() {
		return autonDtfrontRightWheelVelocityCmd_rpm;
	}
	public static double getAutonDtrearLeftWheelVelocityCmd_rpm() {
		return autonDtrearLeftWheelVelocityCmd_rpm;
	}
	public static double getAutonDtrearRightWheelVelocityCmd_rpm() {
		return autonDtrearRightWheelVelocityCmd_rpm;
	}
	public static double getVisionTopTgtXPixelPos() {
		return visionTopTgtXPixelPos;
	}
	public static double getVisionTopTgtYPixelPos() {
		return visionTopTgtYPixelPos;
	}
	public static double getVisionGyroAngleAtLastFrame() {
		return visionGyroAngleAtLastFrame;
	}
	public static double getVisionGyroAngleDesiredAtLastFrame() {
		return visionGyroAngleDesiredAtLastFrame;
	}
	public static double getVisionFrameCounter() {
		return visionFrameCounter;
	}
	public static double getVisionHeuristicVal() {
		return visionHeuristicVal;
	}
	public static double getVisionDistanceAtLastFrame() {
		return visionDistanceAtLastFrame;
	}
	public static double getVisionDistanceDesiredAtLastFrame() {
		return visionDistanceDesiredAtLastFrame;
	}

	


}
