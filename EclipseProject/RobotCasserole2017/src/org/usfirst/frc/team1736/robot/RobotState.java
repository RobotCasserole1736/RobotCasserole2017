package org.usfirst.frc.team1736.robot;


public class RobotState {
	
	// Driver commands (derived from driver's Xbox360 controller)
	static double  driverFwdRevCmd = 0; //Driver desired forward/reverse velocity (-1 = full reverse, 0 = stop, 1 = full forward)
	static double  driverStrafeCmd = 0; //Driver desired side to side velocity (-1 = full left, 0 = stop, 1 = full right)
	static double  driverRotateCmd = 0; //Driver desired rotational velocity (-1 = full CCW, 0 = stop, 1 = full CW)
	static boolean visionAlignmentDesiried = false; //True if the vision algorithm should attempt to align the robot, false if not.
	
	
	// Autonomous drivetrain commands (derived from autonomous path planning routines)
	static double autonDtFwdRevCmd = 0;
	static double autonDtrStrafeCmd = 0;
	static double autonDtRotateCmd = 0;
	
	
	//Operator commands (derived from operator's Xbox360 controller)
	static boolean opHopperFeedDesired = false; // True if the hopper should feed balls to the shooter, false if not.
	static boolean opShooterWheelActivieDesired = false; //True if the wheel should spool up to shot speed, false if it should turn off.
	static boolean opClimbEnable = false; //True if climbing should be allowed, false if all climb commands should be ignored
	static double  opClimbSpeedDesired = 0; //desired speed of the climber motor (0= off, 1 = full forward. Should never be reverse).
	static boolean opGearReleaseDesired = false; //True if the gear should be released, false if not.
	
	
	//Whole-robot pose/velocity state information (outputs from drivetrain class or IMU)
	static double robotPoseAngle_deg = 0; //Present rotation of the robot (relative to field)
	static double robotRotationalVel_degpers = 0 ; //Present rotational velocity of the robot (relative to field)
	static double robotFwdRevVel_ftpers = 0; //Present forward or reverse velocity of the robot in ft/sec. Positive numbers mean forward, negative numbers mean reverse.
	static double robotStrafeVel_ftpers = 0; //Present side-to-side velocity of the robot in ft/sec. Positive numbers mean right, negative numbers mean left.
	
	
	//Wheel velocities (measured from drivetrain encoders) (Outputs from Drivetrain class)
	static double frontLeftWheelVelocity_rpm = 0;
	static double frontRightWheelVelocity_rpm = 0;
	static double rearLeftWheelVelocity_rpm = 0;
	static double rearRightWheelVelocity_rpm = 0;
	
	
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
	static boolean visionAlignmentActive = false; //True if it is desired to use the vision system to align, false if not. (input from outside, driver inputs or autonomous routines)
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
	static boolean hopperFeedCmd; //True if the hopper should feed balls to the shooter, false if not (input from other sources)
	static double  hopperMotorCmd; //Motor command sent to the hopper feed motor (0 = stop, 1 = feed as fast as possible) (calculated by this subsystem)
	
	
	


}
