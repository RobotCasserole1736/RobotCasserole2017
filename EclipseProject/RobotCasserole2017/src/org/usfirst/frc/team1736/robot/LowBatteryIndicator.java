package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.SignalMath.AveragingFilter;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class LowBatteryIndicator {
	private static LowBatteryIndicator batteryIndicator = null;
	private boolean lowVoltageTriggered = false;
	private PowerDistributionPanel pdp = null;
	private AveragingFilter voltageFilter = null;
	private final double MIN_AVG_VOLTAGE = 9.0;
	private final double MIN_INSTANT_VOLTAGE = 7.0;
	
	public static LowBatteryIndicator getInstance() {
		if(batteryIndicator == null)
			batteryIndicator = new LowBatteryIndicator();
		return batteryIndicator;
	}
	
	private LowBatteryIndicator() {
		voltageFilter = new AveragingFilter(5, 12);
	}
	
	public void update() {
		if(pdp == null)
			return;
		double avgVoltage = voltageFilter.filter(pdp.getVoltage());
		if(avgVoltage < MIN_AVG_VOLTAGE || pdp.getVoltage() < MIN_INSTANT_VOLTAGE)
			lowVoltageTriggered = true;
	}
	
	public void setPDPReference(PowerDistributionPanel pdp) {
		this.pdp = pdp;
	}
	
	public boolean isBatteryDead()
	{
		return lowVoltageTriggered;
	}
}
