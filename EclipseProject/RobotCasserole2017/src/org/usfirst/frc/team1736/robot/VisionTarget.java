package org.usfirst.frc.team1736.robot;

public class VisionTarget {
	
	private boolean targetFound = false;
	private double bestX = 0;
	private double bestY = 0;
	private double bestWidth = 0;
	private boolean distanceValid = false;
	
	private final double TANGENT_CAMERA_FOV_X = Math.tan(Math.toRadians(RobotConstants.CAMERA_FOV_X_DEG/2.0));
	
	public VisionTarget()
	{ }
	
	/**
	 * Given the width/x/y of the best candidate for top target, update relevant physical parameters
	 * @param foundTgt true if a target was seen, false if n0t
	 * @param bestX x pixel location of top centroid
	 * @param bestY y pixel location of top centroid
	 * @param bestWidth width of top centroid
	 */
	public void updateTarget(boolean targetFound, double bestX, double bestY, double bestWidth)
	{
		if(targetFound)
		{
			this.targetFound = true;
			this.bestX = bestX;
			this.bestY = bestY;
			this.bestWidth = bestWidth;
		}
		else
		{
			this.targetFound = false;
		}
	}
	
	public boolean isTargetFound()
	{
		return targetFound;
	}
	
	public double getTopTargetXPixelPos()
	{
		return bestX;
	}
	
	public double getTopTargetYPixelPos()
	{
		return bestY;
	}
	
	public double getEstTargetDistanceFt()
	{
		double cam_to_tgt_dist_ft = (RobotConstants.TGT_WIDTH_FT*RobotConstants.VISION_X_PIXELS)/(2.0*bestWidth*TANGENT_CAMERA_FOV_X); //From https://wpilib.screenstepslive.com/s/4485/m/24194/l/288985-identifying-and-processing-the-targets
		double cam_to_tgt_dist_ft_sqrd = Math.pow(cam_to_tgt_dist_ft, 2);
		final double visionTgtHeightSqrd = Math.pow(RobotConstants.HIGH_GOAL_VISION_TARGET_HEIGHT_FT,2);
		
		//We need to calculate distance along the ground, so use pythagorean theorem to calculate floor distance, given target height.
		//ensure the square root will have a positive result (otherwise something wacky is going on)
		if(cam_to_tgt_dist_ft_sqrd > visionTgtHeightSqrd){
			distanceValid = true;
			return Math.sqrt(cam_to_tgt_dist_ft_sqrd - visionTgtHeightSqrd);
		} else {
			distanceValid = false;
			return -1;
		}
	}
	
	public double getTargetOffsetDegrees()
	{
		return (bestX - RobotConstants.VISION_X_PIXELS/2) * (RobotConstants.CAMERA_FOV_X_DEG/RobotConstants.VISION_X_PIXELS);
	}
	
	public boolean isDistanceValid(){
		return distanceValid;
	}
	
}
