package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;
import org.usfirst.frc.team1736.lib.SignalMath.InterpValueHistoryBuffer;

import edu.wpi.first.wpilibj.Timer;

public class VisionAlignment {
	private static VisionAlignment visionAlignment = null;
	
	private VisionAlignAnglePID anglePID;
	private VisionAlignDistPID distPID;
	
	//Record of the history of gyro and distance values
	InterpValueHistoryBuffer gyroHistory;
	InterpValueHistoryBuffer distanceHistory;
	
	//Tolerances
	private double angleTol = 1.0;
	private double distTol = 0.5;
	private double angleTolHyst = 0.025;//get within half a degree lined up
	private double distTolHyst = 0.025; //get within one foot lined up
	private double distanceDesiredLastFrame = 0;
	private double distanceLastFrame = 0;
	private double gyroAngleDesiredLastFrame = 0;
	private double gyroAngleLastFrame = 0;
	
	//Keep track of what the most recent frame received from the coprocessor was
	private double prev_frame_counter;
	private boolean contFrameMode = false;
	
	//States of the vision align subsystem
	public enum VisionAlignStates {
		sNotControlling(0), sAligning(1), sOnTarget(2);
	
		public final int value;
	
		private VisionAlignStates(int value) {
			this.value = value;
		}
	}
	
	//PID Gains 
	Calibration angle_Kp = new Calibration("Alignment Angle Control Kp", 0.05, 0.0, 1.0);
	Calibration angle_Ki = new Calibration("Alignment Angle Control Ki", 0.03, 0.0, 1.0);
	Calibration angle_Kd = new Calibration("Alignment Angle Control Kd", 0.0, 0.0, 1.0);
	Calibration dist_Kp = new Calibration("Alignment Dist Control Kp", 0.25, 0.0, 1.0);
	Calibration dist_Ki = new Calibration("Alignment Dist Control Ki", 0.4, 0.0, 1.0);
	Calibration dist_Kd = new Calibration("Alignment Dist Control Kd", 0.0, -1.0, 1.0);
	
	//Desired angle and distance
	Calibration angleDesired = new Calibration("Desired Angle Alignment Offset", 0.0, -40.0, 40.0);
	Calibration distDesired = new Calibration("Desired Distance Alignment", 70.0/12.0, 0.0, 50.0);
	
	private VisionAlignStates visionAlignState = VisionAlignStates.sNotControlling;
	private boolean visionAlignmentPossible = false;
	private boolean visionAlignmentOnTarget = false;
	private boolean visionAlignmentDesired = false;
	
	public static synchronized VisionAlignment getInstance()
	{
		if(visionAlignment == null)
			visionAlignment = new VisionAlignment();
		return visionAlignment;
	}
	
	private VisionAlignment(){
		// Instantiate angle and distance PIDs
		anglePID = new VisionAlignAnglePID(angle_Kp.get(), angle_Ki.get(), angle_Kd.get());
		distPID = new VisionAlignDistPID(dist_Kp.get(), dist_Ki.get(), dist_Kd.get());
		
		// Set max and min commands
		anglePID.setOutputRange(-0.75, 0.75);
		anglePID.setActualAsDerivTermSrc();
		anglePID.setintegratorDisableThresh(15.0); //Don't use I term until we're within 15 degrees
		
		distPID.setOutputRange(-0.5, 0.5);
		distPID.setActualAsDerivTermSrc();
		distPID.setintegratorDisableThresh(0.25); //Don't use I term till we're within a half a foot
		
		//Make sure neither pid is running
		//CasserolePID is not running after construction
		
		//Make sure controller is off
		visionAlignmentOnTarget = false;
		
		gyroHistory = new InterpValueHistoryBuffer(30, 0);
		distanceHistory = new InterpValueHistoryBuffer(30, 0);
		prev_frame_counter = 0;
	}

	public void GetAligned(){
		
		
		
		VisionProcessing vis = VisionProcessing.getInstance();
		
		//Is the driver commanding vision alignment right now?
		visionAlignmentDesired = DriverController.getInstance().getAlignDesired();
		
		// Is the control system capiable of performing alignment (vision and gyro are online)?
		visionAlignmentPossible = vis.isOnline() & Gyro.getInstance().isOnline();
		
		// Can we continue aligning (control system capable, and driver desires it)?
		boolean alignCanContinue = visionAlignmentDesired & visionAlignmentPossible;
		
		// Can we start aligning (continuing is possible, and we see a target at a reasonable location)?
		boolean alignCanStart = alignCanContinue & vis.getTarget().isTargetFound() & vis.getTarget().isDistanceValid();
		
		//Save historical values
		//Tracking the historical values is needed to offset the delays in visino processing. From the time a frame
		// is captured by the camera, to when it goes over the network, gets processed by the coprocessor, put
		// on the network again, and finally qualified by the vision qualification system, there's a decent amount of delay.
		// Some of those processing times are measured, others are estimated and assumed fixed. The time all rolls up
		// to a single estimate of what time the image was captured at (relative to the getFPGATimestamp() timeframe.
		// We store a brief history of sensor readings (gyro or encoders). Every time we get a processed image frame,
		// we will look up what the gyro/distance readings were at the time the image was captured. We then use the image
		// processing data to define a new setpoint based on where image processing says we should have been. Finally,
		// this setpoint is given to the PID algorithms, which close-loop control the drivetrain around gyro and encoders,
		// moving it to setpoints determined by the vision processing system.
		// We effectively get a boost in bandwidth of our control algorithm.
		double timeNow = Timer.getFPGATimestamp();
		gyroHistory.insert(timeNow, Gyro.getInstance().getAngle());
		distanceHistory.insert(timeNow, RobotPoseCalculator.getInstance().getFwdRevDistFt());
		

		
		//If vision align is possible, and we have a new frame, and that frame has a target, update the setpoints.
		if((visionAlignmentPossible) & 
		   (prev_frame_counter != vis.getFrameCount()) & 
		   (vis.getTarget().isTargetFound()))
		{
			//update the gyro-based setpoints
			gyroAngleLastFrame = gyroHistory.getValAtTime(vis.getEstCaptureTime());
			gyroAngleDesiredLastFrame = gyroAngleLastFrame + (vis.getTarget().getTargetOffsetDegrees() - angleDesired.get());
			//Update the distance-based setpoints
			distanceLastFrame = distanceHistory.getValAtTime(vis.getEstCaptureTime());
			distanceDesiredLastFrame = distanceLastFrame + (vis.getTarget().getEstTargetDistanceFt() - distDesired.get());
			prev_frame_counter = vis.getFrameCount();
		}
		
		//Temp for tuning PID's (lock setpoints at zero, and tune PID's to see how fast you can correct to this setpoint)
		//RobotState.visionGyroAngleDesiredAtLastFrame = 0;
		//RobotState.visionDistanceDesiredAtLastFrame = 0;
		
		//Execute State Machine
		if(visionAlignState == VisionAlignStates.sOnTarget){
			
			if(contFrameMode){
				//Set Desired
				anglePID.setAngle(gyroAngleDesiredLastFrame);
				distPID.setDist(distanceDesiredLastFrame);
			}
			
			if(!(Math.abs(vis.getTarget().getTargetOffsetDegrees() - angleDesired.get()) < angleTol + angleTolHyst) ||
			   !(Math.abs(vis.getTarget().getEstTargetDistanceFt() - distDesired.get()) < distTol + angleTolHyst))
			{ //If we get too far off target...
				
				visionAlignmentOnTarget = false; //Set Off Target
				if(!contFrameMode) {
					anglePID.setAngle(gyroAngleDesiredLastFrame); //"Take Pic"
					distPID.setDist(distanceDesiredLastFrame);
				}
				visionAlignState = VisionAlignStates.sAligning; //Change State
				
			} else if(!alignCanContinue){ //If we shouldn't continue vision alignment...
				
				visionAlignmentOnTarget = false;
				//Turn off pids
				anglePID.stop();
				distPID.stop();
				//Change State
				visionAlignState = VisionAlignStates.sNotControlling;
				
			} else{ //maintain state
				visionAlignState = VisionAlignStates.sOnTarget;
			}
			
		} else if(visionAlignState == VisionAlignStates.sAligning){
			
			if(contFrameMode){
				//Set Desired
				anglePID.setAngle(gyroAngleDesiredLastFrame);
				distPID.setDist(distanceDesiredLastFrame);
			}
			
			if((Math.abs(vis.getTarget().getTargetOffsetDegrees() - angleDesired.get()) < angleTol) && 
			   (Math.abs(vis.getTarget().getEstTargetDistanceFt() - distDesired.get()) < distTol))
			{	//If we get within our tolerance
				//Set On Target
				visionAlignmentOnTarget = true;
				if(!contFrameMode){ //"Take Pic"
					anglePID.setAngle(gyroAngleDesiredLastFrame);
					distPID.setDist(distanceDesiredLastFrame);
				}
				//Change State
				visionAlignState = VisionAlignStates.sOnTarget;
				
			} else if(!alignCanContinue){ //If we shouldn't continue to attempt to align...
				visionAlignmentOnTarget = false;
				//Turn off pids
				anglePID.stop();
				distPID.stop();
				//Change State
				visionAlignState = VisionAlignStates.sNotControlling;
				
			} else{ //maintain state
				visionAlignState = VisionAlignStates.sAligning;
			}
			
		} else{ // visionAlignState == VisionAlignStates.sNotControlling
			visionAlignmentOnTarget = false;
			
			if(alignCanStart) { //If we should start attempting to vision track...
				//Reset integrators and start pids 
				anglePID.start();
				distPID.start();
				
				if(!contFrameMode){
					anglePID.setAngle(gyroAngleDesiredLastFrame);//"Take Pic"
					distPID.setDist(distanceDesiredLastFrame);
				}
				
				//Change State
				visionAlignState = VisionAlignStates.sAligning;
				
			} else{ //maintain state
				visionAlignState = VisionAlignStates.sNotControlling;
			}
		}				
	}
	
	/**
	 * Update all calibrations. Should only be called if pid's are not in use
	 * soooo, during disabled periodic.
	 */
	public void updateGains(){
		if(angle_Kp.isChanged()){
			anglePID.setKp(angle_Kp.get());
			angle_Kp.acknowledgeValUpdate();
		}
		
		if(angle_Ki.isChanged()){
			anglePID.setKi(angle_Ki.get());
			angle_Ki.acknowledgeValUpdate();
		}
		
		if(angle_Kd.isChanged()){
			anglePID.setKd(angle_Kd.get());
			angle_Kd.acknowledgeValUpdate();
		}
		
		if(dist_Kp.isChanged()){
			distPID.setKp(dist_Kp.get());
			dist_Kp.acknowledgeValUpdate();
		}
		
		if(dist_Ki.isChanged()){
			distPID.setKi(dist_Ki.get());
			dist_Ki.acknowledgeValUpdate();
		}
		
		if(dist_Kd.isChanged()){
			distPID.setKd(dist_Kd.get());
			dist_Kd.acknowledgeValUpdate();
		}
		
	}
	
	public double getVisionAlignState() {
		if(visionAlignState == VisionAlignStates.sOnTarget){
			return 2.0;
		}else if(visionAlignState == VisionAlignStates.sAligning){
			return 1.0;
		}else{ //visionAlignState == VisionAlignStates.sNotControlling
			return 0.0;
		}
	}

	public String getVisionAlignStateName(){
		return visionAlignState.toString();
	}
	
	
	public boolean getVisionAlignmentPossible()
	{
		return visionAlignmentPossible;
	}
	
	public boolean getVisionAlignmentOnTarget()
	{
		return visionAlignmentOnTarget;
	}
	
	public void setVisionAlignmentDesired(boolean isDesired)
	{
		visionAlignmentDesired = isDesired;
	}
	
	public boolean getVisionAlignmentDesired()
	{
		return visionAlignmentDesired;
	}
	
	public double getDistanceDesiredAtLastFrame()
	{
		return distanceDesiredLastFrame;
	}
	
	public double getDistanceAtLastFrame()
	{
		return distanceLastFrame;
	}
	
	public double getGyroAngleDesiredAtLastFrame()
	{
		return gyroAngleDesiredLastFrame;
	}
	
	public double getGyroAngleAtLastFrame()
	{
		return gyroAngleLastFrame;
	}
	
	public double getFwdRevCmd()
	{
		return distPID.getOutputCommand();
	}
	
	public double getRotateCmd()
	{
		return anglePID.getOutputCommand();
	}
}