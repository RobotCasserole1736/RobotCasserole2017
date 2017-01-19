package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.CoProcessor.VisionListener;

public class Vision_Processing_Main {
	VisionListener VL;
	
	public Vision_Processing_Main(){
		VL = new VisionListener("10.17.36.20", 5800);
		VL.start();
	}

	public void update(){
		 VL.sampleLatestData();
		 
		 
		 
		 //Update some outputs
		 RobotState.visionOnline = VL.isCoProcessorAlive();
		 RobotState.visionCoProcessorFPS = VL.getFPS();
		 RobotState.visionCoProcessorCPULoad_pct = VL.getCpuLoad();
		 RobotState.visionCoProcessorMemLoad_pct= VL.getMemLoad();
}
}


