package org.usfirst.frc.team1736.robot;

import java.util.ArrayList;

import org.usfirst.frc.team1736.lib.CoProcessor.VisionListener;

public class Vision_Processing_Main {
	VisionListener VL;

	ArrayList<Integer> ArrayList_Top = new ArrayList<Integer>(15);
	ArrayList<Integer> ArrayList_Bottom = new ArrayList<Integer>(15);
	
	private double prevFrameCount;
	
	//local Constants
	private final double CURVATURE_FUDGE_FACTOR = 1.75; //Accounts for the fact the camera angle plus cylinder shape makes for a curved (not rectangular) target. I feel like this is dubious math, but it seems to help for now....
	private final double TANGENT_CAMERA_FOV_X = Math.tan(Math.toRadians(RobotConstants.CAMERA_FOV_X_DEG/2.0));

	
	private final double Exp_AspectRatio_Top=15.0/(4.0*CURVATURE_FUDGE_FACTOR);
	private final double Exp_AspectRatio_Bottom=15.0/(2.0*CURVATURE_FUDGE_FACTOR);
	private final double Exp_InfillRatio_Top = 0.75; //An educated guess
	private final double Exp_InfillRatio_Bottom = 0.75; //An educated guess
	

	public Vision_Processing_Main(){
		prevFrameCount = 0;
		
		VL = new VisionListener(RobotConstants.COPPROCESSOR_LISTEN_ADDRESS, RobotConstants.COPROCESSOR_LISTEN_PORT);
		VL.start();
	}

	/**
	 * Should be called during the periodic update method to evaluate the new info from the coprocessor
	 */
	public void update(){

		//Sample latest available data
		VL.sampleLatestData();
		
		if(prevFrameCount != VL.getFrameCounter()){
			//If we have a new frame since the last time we processed, run the processing algorithm.
			
			alg1();
	
			//Update performance outputs
			RobotState.visionCoProcessorFPS = VL.getFPS();
			RobotState.visionFrameCounter = VL.getFrameCounter();
			RobotState.visionCoProcessorCPULoad_pct = VL.getCpuLoad();
			RobotState.visionCoProcessorMemLoad_pct= VL.getMemLoad();
			RobotState.visionEstCaptureTime = VL.getPacketRxSystemTime() - VL.getProcTimeMs()/1000.0 - RobotConstants.EXPECTED_NETWORK_LATENCY_SEC;
			prevFrameCount = VL.getFrameCounter();
		}
		
		//Always report on alive/dead state of vision system
		RobotState.visionOnline = VL.isCoProcessorAlive();
	}
	
	/**
	 * Simple heuristic algorithm, assumes both a top and bottom can be located.
	 */
	@SuppressWarnings("unused")
	private void alg0(){
		ArrayList_Top.clear();
		ArrayList_Bottom.clear();
		
		double AspectRatio;
		double Pct_Error_Top;
		double Pct_Error_Bottom;
		double Heuristic;
		int Best_Top=-1;
		int Best_Bottom=-1;
		double Best_Heuristic=9001;
		
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
		
		if(Best_Top != -1 & Best_Bottom != -1){
			updateOutputs(true, VL.getX(Best_Top),VL.getY(Best_Top),VL.getWidth(Best_Top));
		} else {
			updateOutputs(false,0,0,0);
		}
		
	}
	
	/**
	 * More-advanced heuristic calculation, which does not try to sort by upper/lower target first
	 * Note this is O(n^2) complexity (actually n*n-1 calculation cycles for n targets), so use caution if many targets are visible.
	 */
	private void alg1(){

		double Heuristic;
		int Best_Top=-1;
		int Best_Bottom=-1;
		double Best_Heuristic=Double.MAX_VALUE;
		
		double x_pos_error;
		double y_pos_error;
		double y_sep_exp;
		double top_ar_error;
		double bottom_ar_error;
		double width_error;
		double height_error;
		double top_infill_error;
		double bottom_infill_error;
		double i_like_big_targets_and_i_cannot_lie;

		for(int i=0; i<VL.getNumTargetsObserved();i++){ //i is top target iter
			for(int j=0; j<VL.getNumTargetsObserved();j++){ //j is bottom target iter
				
				if(i == j){
					//Cannot compare a target to itself
					continue;
				}
				x_pos_error = Math.abs(VL.getX(i)-VL.getX(j)); //expect X positions to be aligned
				y_sep_exp = (double)Math.round((VL.getHeight(i)*3.0/2.0 + VL.getHeight(j)/2)/CURVATURE_FUDGE_FACTOR);
				y_pos_error = Math.abs((VL.getY(j)-VL.getY(i))-y_sep_exp); //Expect Top to be above Bottom (top's y < bottom's y) by an assumed distance
				width_error = Math.abs(VL.getWidth(i)-VL.getWidth(j)); //Expect same width
				height_error = Math.abs(VL.getHeight(i)-VL.getHeight(j)*2.0); //expect top height to be double bottom height
				top_ar_error = Math.abs((VL.getWidth(i)/VL.getHeight(i))-Exp_AspectRatio_Top); //Expect certain aspect ratios
				bottom_ar_error = Math.abs((VL.getWidth(j)/VL.getHeight(j))-Exp_AspectRatio_Bottom);
				top_infill_error = Math.abs((VL.getArea(i)/(VL.getWidth(i)*VL.getHeight(i))) - Exp_InfillRatio_Top);
				bottom_infill_error = Math.abs((VL.getArea(j)/(VL.getWidth(j)*VL.getHeight(j))) - Exp_InfillRatio_Bottom);
				i_like_big_targets_and_i_cannot_lie = 100000 * 1/(VL.getArea(i) + VL.getArea(j));
				
				//The better the target is, the smaller Heuristic should be
				Heuristic= x_pos_error     * 10.0 + //We want the top/bottom centroids to be aligned in the X direction
						   y_pos_error     * 10.0 + //Given the heights of the top/bottom, we expect a certain offset in the centroids in the Y direction
						   width_error     * 10.0 + //We expect the top/bottom to have the same width
						   height_error    * 5.0  +	//We expect the top to have twice the height of the bottom
						   top_ar_error    * 1.0  + //We expect the top to have a certain aspect ratio
						   bottom_ar_error * 1.0  + //We expect the bottom to have a certain aspect ratio
						   top_infill_error * 1.0 +  //We expect the top to have a certain infill percentage
						   bottom_infill_error * 1.0 + //We expect the bottom to have a certain infill percentage
						   i_like_big_targets_and_i_cannot_lie; //Bigger targets are better.

				//We expect only one target possible. Pick the best.
				if (Heuristic<Best_Heuristic){
					Best_Heuristic=Heuristic;
					Best_Top=i;
					Best_Bottom=j;		
				}
			}
		}
		
		
		if(Best_Top != -1 & Best_Bottom != -1){
			updateOutputs(true, VL.getX(Best_Top),VL.getY(Best_Top),VL.getWidth(Best_Top));
		} else {
			updateOutputs(false,0,0,0);
		}
	}
	
	/**
	 * Given the width/x/y of the best candidate for top target, update relevant physical parameters
	 * @param foundTgt true if a target was seen, false if n0t
	 * @param bestX x pixel location of top centroid
	 * @param bestY y pixel location of top centroid
	 * @param bestWidth width of top centroid
	 */
	private void updateOutputs(boolean foundTgt, double bestX, double bestY, double bestWidth){
		
		if(foundTgt){
			RobotState.visionTargetFound = true;
			RobotState.visionTopTgtXPixelPos = (bestX);
			RobotState.visionTopTgtYPixelPos = (bestY);
			double cam_to_tgt_dist_ft = (RobotConstants.TGT_WIDTH_FT*RobotConstants.VISION_X_PIXELS)/(2.0*bestWidth*TANGENT_CAMERA_FOV_X); //From https://wpilib.screenstepslive.com/s/4485/m/24194/l/288985-identifying-and-processing-the-targets
			RobotState.visionEstTargetDist_ft = Math.sqrt(Math.pow(cam_to_tgt_dist_ft, 2) - Math.pow(RobotConstants.HIGH_GOAL_VISION_TARGET_HEIGHT_FT,2)); //We need to calculate distance along the ground, so use pythagorean theorem to calculate floor distance, given target height.
			RobotState.visionTargetOffset_deg = (bestX - RobotConstants.VISION_X_PIXELS/2) * (RobotConstants.CAMERA_FOV_X_DEG/RobotConstants.VISION_X_PIXELS);
		} else {
			RobotState.visionTargetFound = false;
		}

		
	}
}


