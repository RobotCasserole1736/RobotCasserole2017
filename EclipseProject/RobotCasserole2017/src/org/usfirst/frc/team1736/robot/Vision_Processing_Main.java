package org.usfirst.frc.team1736.robot;

import java.util.ArrayList;

import org.usfirst.frc.team1736.lib.CoProcessor.VisionListener;

public class Vision_Processing_Main {
	VisionListener VL;

	ArrayList<Integer> ArrayList_Top = new ArrayList<Integer>(15);
	ArrayList<Integer> ArrayList_Bottom = new ArrayList<Integer>(15);
	
	//Constants
	private final double Exp_AspectRatio_Top=15.0/4.0;
	private final double Exp_AspectRatio_Bottom=15.0/2.0;

	public Vision_Processing_Main(){
		VL = new VisionListener("10.17.36.20", 5800);
		VL.start();
	}

	public void update(){

		ArrayList_Top.clear();
		ArrayList_Bottom.clear();

		double AspectRatio;
		double Pct_Error_Top;
		double Pct_Error_Bottom;
		double Heuristic;
		int Best_Top=-1;
		int Best_Bottom=-1;
		double Best_Heuristic=9001;

		VL.sampleLatestData();
		
		for(int i=0; i<VL.getNumTargetsObserved();i++){
			AspectRatio=VL.getWidth(i)/VL.getHeight(i);
			Pct_Error_Top=Math.abs((AspectRatio-Exp_AspectRatio_Top)/Exp_AspectRatio_Top);
			Pct_Error_Bottom=Math.abs((AspectRatio-Exp_AspectRatio_Bottom)/Exp_AspectRatio_Bottom);

			if (Pct_Error_Top>Pct_Error_Bottom){
				ArrayList_Bottom.add(i);
			}
			else{
				ArrayList_Top.add(i);
			}
		}

		for(int i=0; i<ArrayList_Bottom.size();i++){
			for(int j=0; j<ArrayList_Top.size();j++){

				Heuristic=Math.abs(VL.getY(ArrayList_Bottom.get(i))-VL.getY(ArrayList_Top.get(j)));		

				if (Heuristic<Best_Heuristic){
					Best_Heuristic=Heuristic;
					Best_Top=ArrayList_Bottom.get(i);
					Best_Bottom=ArrayList_Top.get(j);		
				}
			}
		}

		
		//Update all outputs
		if(Best_Top != -1 & Best_Bottom != -1){
			RobotState.visionTargetFound = true;
			RobotState.visionTopTgtXPixelPos = (VL.getX(Best_Top));
			RobotState.visionTopTgtYPixelPos = (VL.getY(Best_Top));
		} else {
			RobotState.visionTargetFound = false;
		}

		
		RobotState.visionOnline = VL.isCoProcessorAlive();
		RobotState.visionCoProcessorFPS = VL.getFPS();
		RobotState.visionCoProcessorCPULoad_pct = VL.getCpuLoad();
		RobotState.visionCoProcessorMemLoad_pct= VL.getMemLoad();
	}
}


