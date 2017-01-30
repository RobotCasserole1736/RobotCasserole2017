package org.usfirst.frc.team1736.lib.LEDs;

public interface CasseroleLEDInterface {

    // period between periodic function calls
    // in milliseconds
    double m_update_period_ms = 50;
	
	public default void clearColorBuffer(){
		return;
	}
	public default void setLEDColor(int index, double r, double g, double b) {
		return;
	}
	

	public default void setLEDColorHSL(int idx, double h, double s, double l){
	    double r, g, b;

	    if (s == 0f) {
	        r = g = b = l; // achromatic
	    } else {
	        double q = l < 0.5f ? l * (1 + s) : l + s - l * s;
	        double p = 2 * l - q;
	        r = hueToRgb(p, q, h + 1f/3f);
	        g = hueToRgb(p, q, h);
	        b = hueToRgb(p, q, h - 1f/3f);
	    }
	    
	    setLEDColor(idx, r, g, b);
	}

	/** Helper method that converts hue to rgb */
	public static double hueToRgb(double p, double q, double t) {
	    if (t < 0f)
	        t += 1f;
	    if (t > 1f)
	        t -= 1f;
	    if (t < 1f/6f)
	        return p + (q - p) * 6f * t;
	    if (t < 1f/2f)
	        return q;
	    if (t < 2f/3f)
	        return p + (q - p) * (2f/3f - t) * 6f;
	    return p;
	}
	
}
