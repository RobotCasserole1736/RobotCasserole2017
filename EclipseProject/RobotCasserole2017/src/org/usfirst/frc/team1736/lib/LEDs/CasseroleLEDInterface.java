package org.usfirst.frc.team1736.lib.LEDs;

public interface CasseroleLEDInterface {

    // period between periodic function calls
    // in milliseconds
    double m_update_period_ms = 50;
	
	public void clearColorBuffer();
	public void setLEDColor(int index, double r, double g, double b);
}
