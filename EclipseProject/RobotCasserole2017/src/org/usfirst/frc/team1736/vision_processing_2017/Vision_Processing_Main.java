package org.usfirst.frc.team1736.vision_processing_2017;

import org.usfirst.frc.team1736.lib.CoProcessor.VisionListener;

public class Vision_Processing_Main {
	VisionListener VL;
	
	public Vision_Processing_Main(){
		VL = new VisionListener("10.17.36.20", 5800);
		VL.start();
	}

	public void update(){
		 VL.sampleLatestData();
         System.out.print(VL.getProcTimeMs());
         System.out.print(" | ");
         System.out.print(VL.getNumTargetsObserved());
         System.out.print(" | ");
         System.out.print(VL.isCoProcessorAlive());
         System.out.println(" | ");
}
}


