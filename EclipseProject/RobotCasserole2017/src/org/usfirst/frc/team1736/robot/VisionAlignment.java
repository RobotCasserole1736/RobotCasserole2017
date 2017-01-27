package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

public class VisionAlignment {
	VisionAlignAnglePID anglePID;
	VisionAlignDistPID distPID;
	
	//Tolerances
	double angleTol = 1.0;
	double distTol = 1.0;
	
	//States of the vision align subsystem
	public enum VisionAlignStates {
		sNotControlling(0), sAligning(1), sOnTarget(2);
	
		public final int value;
	
		private VisionAlignStates(int value) {
			this.value = value;
		}
	}

	
	//PID Gains 
	Calibration angle_Kp = new Calibration("Alignment Angle Control Kp", 1.0/40.0, 0.0, 1.0);
	Calibration angle_Ki = new Calibration("Alignment Angle Control Ki", 1.0/40.0/(3*50), 0.0, 1.0);
	Calibration angle_Kd = new Calibration("Alignment Angle Control Kd", 0.0, 0.0, 1.0);
	Calibration dist_Kp = new Calibration("Alignment Dist Control Kp", 1.0/7.0, 0.0, 1.0);
	Calibration dist_Ki = new Calibration("Alignment Dist Control Ki", 1.0/7.0/(3*50), 0.0, 1.0);
	Calibration dist_Kd = new Calibration("Alignment Dist Control Kd", 0.0, 0.0, 1.0);
	
	//Desired angle and distance
	Calibration angleDesired = new Calibration("Desired Angle Alignment Offset", 0.0, -40.0, 40.0);
	Calibration distDesired = new Calibration("Desired Distance Alignment", 10.0, 0.0, 50.0);
	
	VisionAlignStates visionAlignState = VisionAlignStates.sNotControlling;
	
	public VisionAlignment(){
		anglePID = new VisionAlignAnglePID(angle_Kp.get(), angle_Ki.get(), angle_Kd.get());
		distPID = new VisionAlignDistPID(dist_Kp.get(), dist_Ki.get(), dist_Kd.get());
		
		//Make sure neither pid is running
		//CasserolePID is not running after construction
		
		//Make sure controller is off
		RobotState.visionAlignmentOnTarget = false;
		RobotState.visionDtFwdRevCmd = 0.0;
		RobotState.visionDtRotateCmd = 0.0;
	}

	public void GetAligned(){
		
		// Figure out if alignment is possible
		RobotState.visionAlignmentPossible = RobotState.visionOnline && RobotState.visionTargetFound;
		
		//Execute State Machine
		if(visionAlignState == VisionAlignStates.sOnTarget){
			//Set Desired
			anglePID.setAngle(angleDesired.get());
			distPID.setDist(distDesired.get());
			
			if(!(Math.abs(RobotState.visionTargetOffset_deg - angleDesired.get()) < angleTol
					&& Math.abs(RobotState.visionEstTargetDist_ft - distDesired.get()) < distTol)){
				//Set Off Target
				RobotState.visionAlignmentOnTarget = false;
				
				//Change State
				visionAlignState = VisionAlignStates.sAligning;
			}else if(!RobotState.visionAlignmentDesiried){
				//Set outputs to 0
				RobotState.visionAlignmentOnTarget = false;
				RobotState.visionDtFwdRevCmd = 0.0;
				RobotState.visionDtRotateCmd = 0.0;

				//Turn off pids
				anglePID.stop();
				distPID.stop();
				
				//Change State
				visionAlignState = VisionAlignStates.sNotControlling;
			}else{
				visionAlignState = VisionAlignStates.sOnTarget;
			}
			
		}else if(visionAlignState == VisionAlignStates.sAligning){
			//Set Desired
			anglePID.setAngle(angleDesired.get());
			distPID.setDist(distDesired.get());
			
			if(Math.abs(RobotState.visionTargetOffset_deg - angleDesired.get()) < angleTol
					&& Math.abs(RobotState.visionEstTargetDist_ft - distDesired.get()) < distTol){
				//Set On Target
				RobotState.visionAlignmentOnTarget = true;
				
				//Change State
				visionAlignState = VisionAlignStates.sOnTarget;
			}else if(!RobotState.visionAlignmentDesiried){
				//Set outputs to 0
				RobotState.visionAlignmentOnTarget = false;
				RobotState.visionDtFwdRevCmd = 0.0;
				RobotState.visionDtRotateCmd = 0.0;

				//Turn off pids
				anglePID.stop();
				distPID.stop();
				
				//Change State
				visionAlignState = VisionAlignStates.sNotControlling;
			}else{
				visionAlignState = VisionAlignStates.sAligning;
			}
			
		}else{ // visionAlignState == VisionAlignStates.sNotControlling
			RobotState.visionAlignmentOnTarget = false;
			RobotState.visionDtFwdRevCmd = 0.0;
			RobotState.visionDtRotateCmd = 0.0;
			
			if(RobotState.visionAlignmentDesiried && RobotState.visionAlignmentPossible){
				//Reset integrators and start pids 
				anglePID.start();
				distPID.start();
				
				//Change State
				visionAlignState = VisionAlignStates.sAligning;
			}else{
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
}