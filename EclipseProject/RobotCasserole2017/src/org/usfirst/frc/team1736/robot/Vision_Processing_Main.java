package org.usfirst.frc.team1736.robot;

import java.util.ArrayList;

import org.usfirst.frc.team1736.lib.CoProcessor.VisionListener;

public class Vision_Processing_Main {
	VisionListener VL;
	
	int Best_Top;
	int Best_Bottom;
	
	public Vision_Processing_Main(){
		VL = new VisionListener("10.17.36.20", 5800);
		VL.start();
	}

	public void update(){
		 ArrayList<Integer> ArrayList_Top = new ArrayList<Integer>();
		 ArrayList<Integer> ArrayList_Bottom = new ArrayList<Integer>();
		 	
		 double Best_Heuristic=9001;
		 VL.sampleLatestData();
		 	for(int i=0; i<VL.getNumTargetsObserved();i++){
		 		double AspectRatio=VL.getWidth(i)/VL.getHeight(i);
		 		double Exp_AspectRatio_Top=4/15;
		 		double Exp_AspectRatio_Bottom=2/15;
		 		double Pct_Error_Top=Math.abs((AspectRatio-Exp_AspectRatio_Top)/Exp_AspectRatio_Top*100);
		 		double Pct_Error_Bottom=Math.abs((AspectRatio-Exp_AspectRatio_Bottom)/Exp_AspectRatio_Bottom*100);
		 		
		 		if (Pct_Error_Top>Pct_Error_Bottom){
		 			ArrayList_Bottom.add(i);
		 		}
		 		if (Pct_Error_Bottom<=Pct_Error_Top){
		 			ArrayList_Top.add(i);
		 		}
		 		
		 	for(i=0; i<ArrayList_Bottom.size();i++){
		 		for(int j=0; j<ArrayList_Top.size();j++){
		 	
		 			double Heuristic=Math.abs(VL.getY(ArrayList_Bottom.get(i))-VL.getY(ArrayList_Top.get(j)));		
		 		
		 			if (Heuristic<Best_Heuristic){
		 				Best_Heuristic=Heuristic;
		 				Best_Top=ArrayList_Bottom.get(i);
		 				Best_Bottom=ArrayList_Top.get(j);		
		 			}
		 		}
		 	}
		 	
		 	System.out.println(VL.getX(Best_Top));
		 	System.out.println(VL.getY(Best_Top));
		 	System.out.println("==");	
		 	}
		 			
		 
		 
		 
		 //Update some outputs
		 RobotState.visionOnline = VL.isCoProcessorAlive();
		 RobotState.visionCoProcessorFPS = VL.getFPS();
		 RobotState.visionCoProcessorCPULoad_pct = VL.getCpuLoad();
		 RobotState.visionCoProcessorMemLoad_pct= VL.getMemLoad();
}
}


