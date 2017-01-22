package org.usfirst.frc.team1736.robot;


public class RobotState {
	
	// Driver commands (derived from driver's Xbox360 controller)
	static double  driverFwdRevCmd = 0; //Driver desired forward/reverse velocity (-1 = full reverse, 0 = stop, 1 = full forward)
	static double  driverStrafeCmd = 0; //Driver desired side to side velocity (-1 = full left, 0 = stop, 1 = full right)
	static double  driverRotateCmd = 0; //Driver desired rotational velocity (-1 = full CCW, 0 = stop, 1 = full CW)
	static boolean visionAlignmentDesiried = false; //True if the vision algorithm should attempt to align the robot, false if not.
	
	//camera positioning commands (derived from Xbox360 controller)
	static boolean gearCamAlign = false;
	static boolean shooterCamAlign = false;
	static boolean intakeCamAlign = false;
	
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
	
	
	//Operator commands (derived from operator's Xbox360 controller)
	static Shooter_States opShotCTRL = Shooter_States.NO_Shoot;
	static boolean opClimbEnable = false; //True if climbing should be allowed, false if all climb commands should be ignored
	static double  opClimbSpeedDesired = 0; //desired speed of the climber motor (0= off, 1 = full forward. Should never be reverse).
	static boolean opGearReleaseDesired = false; //True if the gear should be released, false if not.
	static boolean opIntakeDesired = false; //True if the operator wants to to pull in fuel throught the intake, false if not
	static boolean opEjectDesired = false;  //True if the operator wants to reverse the intake direction to eject lodged balls, false if not.
	
	//Whole-robot pose/velocity state information (outputs from drivetrain class or IMU)
	static double robotPoseAngle_deg = 0; //Present rotation of the robot (relative to field)
	static double robotRotationalVel_degpers = 0 ; //Present rotational velocity of the robot (relative to field)
	static double robotFwdRevVel_ftpers = 0; //Present forward or reverse velocity of the robot in ft/sec. Positive numbers mean forward, negative numbers mean reverse.
	static double robotStrafeVel_ftpers = 0; //Present side-to-side velocity of the robot in ft/sec. Positive numbers mean right, negative numbers mean left.
	static double robotNetSpeed_ftpers = 0; //Present total translational speed in ft/sec
	
	//Wheel velocities (measured from drivetrain encoders) (Outputs from Drivetrain class)
	//Positive is forward rotation, negative is backward rotation
	static double frontLeftWheelVelocity_rpm = 0;
	static double frontRightWheelVelocity_rpm = 0;
	static double rearLeftWheelVelocity_rpm = 0;
	static double rearRightWheelVelocity_rpm = 0;
	
	//Wheel total distance
	//Positive is foward rotation, negative is backward rotation
	static double frontLeftWheelDistance_ft = 0;
	static double frontRightWheelDistance_ft= 0;
	static double rearLeftWheelDistance_ft = 0;
	static double rearRightWheelDistance_ft= 0;
	
	
	//Vision Target Qualification System (Outputs from vision targeting system)
	static boolean visionOnline = false; //True if information is being received from the coprocessor, false if the system is down. All other variables should only be considered valid if this field is true.
	static boolean visionTargetFound = false; //True if a qualified (valid) target is found in the most recent camera image, false if not
	static double  visionTargetOffset_deg = 0; //Offset from center of image of the located target. 0 degrees means the target is right in front of the robot, positive degrees means the target is to the right, negative degrees means the target is to the left.
	static double  visionEstTargetDist_ft = 0; //Estimated distance of the front bumper of the robot from the base of the boiler (from observed vision target) 
	static double  visionEstCaptureTime = 0; //Estimated capture time (WRT getFPGATimestamp's timebase) of the present processed image. Should account for processing time (variable, but reported by coprocessor) as well as assumed fixed data transport times over ethernet.
	static double  visionCoProcessorFPS = 0; //system processing Frames-per-second reported by the coprocessor
	static double  visionCoProcessorCPULoad_pct = 0; //Coprocessor reported processor load
	static double  visionCoProcessorMemLoad_pct = 0; //Coprocessor reported memory load
	
	
	//Vision Align system (mostly outputs from vision-based alignment system)
	static boolean visionAlignmentPossible = false; //True if the vision system is capiable of auto-aligning the robot, false if not. False should happen if the vision system is offline, or no target can be seen.
	static double  visionDtFwdRevCmd = 0; //Forward/reverse command from vision alignment system to get the robot aligned for a shot. (-1 = full reverse, 0 = stop, 1 = full forward)
	static double  visionDtRotateCmd = 0; //Rotation command from vision alignment system to get the robot aligned for a shot. (-1 = full reverse, 0 = stop, 1 = full forward)
	static boolean visionAlignmentOnTarget = false; //True if we know for sure the robot is aligned sufficiently to make shots, false if not.
	
	//Shooter subsystem 
	static boolean shooterActiveCmd = false; //True if the shooter wheel should be at shot velocity, false if it should be off.
	static double  shooterDesiredVelocity_rpm = 0; //RPM target the shooter should attempt to get to (output, calculated by this subsystem)
	static double  shooterActualVelocity_rpm  = 0; //RPM the shooter wheel is actually running at (output, measured by this subsystem)
	static double  shooterMotorCmd = 0; //Present control effort being applied to shooter motor(s) - 0.0 = stop, 1.0 = full power (output, calculated by this subsystem)
	static boolean shooterVelocityOk = false; //True if the actual RPM is within a pre-defined range of the desired RPM and a shot can be taken accurately, false otherwise (output, calculated by this subsystem)

	//Hopper Subsystem
	static boolean hopperFeedCmd;//True if the hopper should feed balls to the shooter, false if not (input from other sources)
	static double  hopperMotorCmd; //Motor command sent to the hopper feed motor (0 = stop, 1 = feed as fast as possible) (calculated by this subsystem)
	
	//DriveTrain motor commands
	static double frontLeftDriveMotorCmd = 0;
	static double frontRightDriveMotorCmd = 0;
	static double rearLeftDriveMotorCmd = 0;
	static double rearRightDriveMotorCmd = 0;
	
	//Climber Commands
	static boolean climbEnable = false;
	static double climbSpeedCmd = 0.0;
	
	//Intake Command
	static double intakeSpeedCmd = 0.0;
	
	//Getters and Setters for select globals
	public static double getopShotCTRLOrdinal(){
		return (double)RobotState.opShotCTRL.ordinal();
	}
	public static double getClimbSpeedCmd(){
		return RobotState.climbSpeedCmd;
	}
	public static double getRobotFwdRevVel_ftpers() {
		return robotFwdRevVel_ftpers;
	}
	public static double getRobotStrafeVel_ftpers() {
		return robotStrafeVel_ftpers;
	}
	public static double getShooterActualVelocity_rpm() {
		return shooterActualVelocity_rpm;
	}
	public static double getShooterDesiredVelocity_rpm() {
		return shooterDesiredVelocity_rpm;
	}
	public static double getShooterMotorCmd() {
		return shooterMotorCmd;
	}
	public static double getDriverFwdRevCmd() {
		return driverFwdRevCmd;
	}
	public static double getDriverStrafeCmd() {
		return driverStrafeCmd;
	}
	public static double getDriverRotateCmd() {
		return driverRotateCmd;
	}
	public static boolean isVisionAlignmentDesiried() {
		return visionAlignmentDesiried;
	}
	public static boolean isOpClimbEnable() {
		return opClimbEnable;
	}
	public static double getOpClimbSpeedDesired() {
		return opClimbSpeedDesired;
	}
	public static boolean isOpGearReleaseDesired() {
		return opGearReleaseDesired;
	}
	public static double getRobotPoseAngle_deg() {
		return robotPoseAngle_deg;
	}
	public static double getRobotRotationalVel_degpers() {
		return robotRotationalVel_degpers;
	}
	public static double getRobotNetSpeed_ftpers() {
		return robotNetSpeed_ftpers;
	}
	public static double getFrontLeftWheelVelocity_rpm() {
		return frontLeftWheelVelocity_rpm;
	}
	public static double getFrontRightWheelVelocity_rpm() {
		return frontRightWheelVelocity_rpm;
	}
	public static double getRearLeftWheelVelocity_rpm() {
		return rearLeftWheelVelocity_rpm;
	}
	public static double getRearRightWheelVelocity_rpm() {
		return rearRightWheelVelocity_rpm;
	}
	public static double getFrontLeftWheelDistance_ft() {
		return frontLeftWheelDistance_ft;
	}
	public static double getFrontRightWheelDistance_ft() {
		return frontRightWheelDistance_ft;
	}
	public static double getRearLeftWheelDistance_ft() {
		return rearLeftWheelDistance_ft;
	}
	public static double getRearRightWheelDistance_ft() {
		return rearRightWheelDistance_ft;
	}
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
	public static boolean isVisionAlignmentPossible() {
		return visionAlignmentPossible;
	}
	public static double getVisionDtFwdRevCmd() {
		return visionDtFwdRevCmd;
	}
	public static double getVisionDtRotateCmd() {
		return visionDtRotateCmd;
	}
	public static boolean isVisionAlignmentOnTarget() {
		return visionAlignmentOnTarget;
	}
	public static boolean isShooterVelocityOk() {
		return shooterVelocityOk;
	}
	public static boolean isHopperFeedCmd() {
		return hopperFeedCmd;
	}
	public static double getHopperMotorCmd() {
		return hopperMotorCmd;
	}
	public static double getIntakeSpeedCmd() {
		return intakeSpeedCmd;
	}
	public static boolean isOpIntakeDesired() {
		return opIntakeDesired;
	}
	public static boolean isOpEjectDesired() {
		return opEjectDesired;
	}
	public static double getFrontLeftDriveMotorCmd() {
		return frontLeftDriveMotorCmd;
	}
	public static double getFrontRightDriveMotorCmd() {
		return frontRightDriveMotorCmd;
	}
	public static double getRearLeftDriveMotorCmd() {
		return rearLeftDriveMotorCmd;
	}
	public static double getRearRightDriveMotorCmd() {
		return rearRightDriveMotorCmd;
	}
	public static boolean isClimbEnable() {
		return climbEnable;
	}
	public static boolean isGearCamAlign() {
		return gearCamAlign;
	}
	public static boolean isShooterCamAlign() {
		return shooterCamAlign;
	}
	public static boolean isIntakeCamAlign() {
		return intakeCamAlign;
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
	public static boolean isShooterActiveCmd() {
		return shooterActiveCmd;
	}

	


}
