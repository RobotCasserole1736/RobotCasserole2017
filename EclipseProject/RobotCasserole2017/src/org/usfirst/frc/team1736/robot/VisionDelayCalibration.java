package org.usfirst.frc.team1736.robot;

import java.util.ArrayList;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;

public class VisionDelayCalibration {
	private static VisionDelayCalibration visionDelayCal = null;
	
	private VisionCalStates curState;
	private boolean ringActiveInNoCal;
	private boolean ringOutputActive;
	private Calibration runVisionDelayCal;
	private boolean calDesiredPrev;
	private ArrayList<Double> calHistory;
	private VisionCalResults lastResult;
	private double cycleStartTime;
	private double cycleEndTime;
	private final double CYCLE_TIMEOUT_S = 5.0;
	private double prevCalAvgTime;
	private double prevCalStdDev;
	private DigitalOutput ringOutPort;
	
	public static synchronized VisionDelayCalibration getInstance(){
		if(visionDelayCal == null)
			visionDelayCal = new VisionDelayCalibration();
		return visionDelayCal;
	}
	
	private VisionDelayCalibration(){
		//Init state
		curState = VisionCalStates.NO_CAL;
		ringOutputActive = false;
		runVisionDelayCal = new Calibration("Run Vision Processing Delay Calibration (1 to run)", 0.0, 0.0, 1.0);
		calDesiredPrev = false;
		calHistory = new ArrayList<Double>(20);
		lastResult = VisionCalResults.CAL_NOT_RUN;
		ringOutPort = new DigitalOutput(RobotConstants.LED_RING_CONTROL_OUTPORT);
	}
	
	public enum VisionCalStates 
	{
		NO_CAL,CAL_INIT,CAL_CYCLE_PREP,CAL_CYCLE_START,CAL_CYCLE_WAIT,CAL_CYCLE_FINISH_PASS,CAL_CYCLE_FINISH_FAIL,CAL_TERMINATE;	
	}
	public enum VisionCalResults
	{
		CAL_NOT_RUN, CAL_IN_PROCESS, CAL_SUCCESS, CAL_FAIL;
	}
	
	public void setLEDRingActive(boolean isActive){
		ringActiveInNoCal = isActive;
	}
	
	public void update(){
		VisionCalStates nextState = curState; //default to staying in the same state
		boolean calDesired = (runVisionDelayCal.get() == 1.0);
		
		//Execute state machine
		switch(curState){
			case NO_CAL:
				ringOutputActive = ringActiveInNoCal;
				if(calDesired == true & calDesiredPrev == false){
					nextState = VisionCalStates.CAL_INIT;
				}
				break;
				
			case CAL_INIT:
				calHistory.clear();
				ringOutputActive = false;
				lastResult = VisionCalResults.CAL_IN_PROCESS;
				nextState = VisionCalStates.CAL_CYCLE_PREP;
				break;
				
			case CAL_CYCLE_PREP:
				ringOutputActive = false;
				if(VisionProcessing.getInstance().getNumberOfTargetsObserved() == 0){
					nextState = VisionCalStates.CAL_CYCLE_START;
				} else if(!VisionProcessing.getInstance().isOnline()){
					nextState = VisionCalStates.CAL_CYCLE_FINISH_FAIL;
				}
				break;
				
			case CAL_CYCLE_START:
				ringOutputActive = true;
				cycleStartTime = Timer.getFPGATimestamp();
				nextState = VisionCalStates.CAL_CYCLE_WAIT;
				break;
				
			case CAL_CYCLE_WAIT:
				ringOutputActive = true;
				if(VisionProcessing.getInstance().getNumberOfTargetsObserved() > 0){
					cycleEndTime = Timer.getFPGATimestamp();
					nextState = VisionCalStates.CAL_CYCLE_FINISH_PASS;
				} else if(Timer.getFPGATimestamp() > (cycleStartTime + CYCLE_TIMEOUT_S)){
					nextState = VisionCalStates.CAL_CYCLE_FINISH_FAIL;
				}
				break;
				
			case CAL_CYCLE_FINISH_PASS:
				calHistory.add(cycleEndTime - cycleStartTime);
				ringOutputActive = false;
				if(calDesired == true){
					nextState = VisionCalStates.CAL_INIT;
				} else {
					lastResult = VisionCalResults.CAL_SUCCESS;
					nextState = VisionCalStates.CAL_TERMINATE;
				}
				break;
				
			case CAL_CYCLE_FINISH_FAIL:
				ringOutputActive = false;
				//Don't count the last sample, just stop.
				lastResult = VisionCalResults.CAL_FAIL;
				nextState = VisionCalStates.CAL_TERMINATE;
				break;
				
			case CAL_TERMINATE:
				ringOutputActive = false;
				//Report any results we can get
				if(calHistory.size() > 0){
					prevCalAvgTime = calcAvg(calHistory);
					prevCalStdDev = calcStdDev(calHistory, prevCalAvgTime);
				} else {
					prevCalAvgTime = -1;
					prevCalStdDev = -1;
				}
				nextState = VisionCalStates.NO_CAL;
				break;
				
			default:
				//"when pigs fly" code
				System.out.println("Warning - programming made a mistake in VisinDelayCal, unknown state achieved! Tell software team about this!!!");
				ringOutputActive = false;
				nextState = VisionCalStates.NO_CAL; //unconditionally reset the state machine
				break;
		}
		
		//Advance to the next state
		curState = nextState;
		calDesiredPrev = calDesired;
		
		//Set outputs
		ringOutPort.set(ringOutputActive);
	}
	
	private double calcAvg(ArrayList<Double> in){
		double sum = 0;
		for(double element : in){
			sum += element;
		}
		return sum/((double)in.size());
	}
	
	private double calcStdDev(ArrayList<Double> in, double avg){
		double sum = 0;
		for(double element : in){
			sum += Math.abs(element - avg);
		}
		return sum/((double)in.size());
	}

	public VisionCalResults getLastResult() {
		return lastResult;
	}

	public double getPrevCalAvgTime() {
		return prevCalAvgTime;
	}

	public double getPrevCalStdDev() {
		return prevCalStdDev;
	}
	

}
