package org.team2168.utils;

import edu.wpi.first.math.MathUtil;

/**
 * A class which performs linear interpolation. Can be used for joystick
 * scaling, sensor scaling, etc.
 *
 * @author james@team2168.org
 */
public class LinearInterpolator {
	double[][] values;

	/**
	 * Create a new interpolator. 
	 * 
	 * 			An interpolator puts limits on the range of an object (i.e a sensor or joystick). This is done to avoid the
	 *  object having natural displacement from the neutral position (i.e 0). In other words, it puts a deadband on the joystick.
	 * 
	 * 			This method is what sets the interpolator meanwhile the interpolate() method is what actually does the math and work.
	 * 
	 * @param values
	 *            an ascending (sorted) array of coordinate pairs. The pair of coordinates is used to map the limits of the 
	 * object's range.
	 */
	public LinearInterpolator(double[][] values) {
		// TODO: sort the input list
		this.values = values;
	}

	/**
	 * Augment an input value using the supplied data set.
	 *
	 * @param input
	 *            the value to augment
	 * @return The adjusted value.
	 */
	public double interpolate(double input) {
		double retVal = 0.0;
		boolean done = false;
		double m, b;

		// make sure the input value falls within the range of the provided data set
		input = MathUtil.clamp(input, values[0][0], values[values.length - 1][0]);

		// Find the two points in our array, between which the input falls.
		// We will start at i = 1 since we can't have a point fall outside our
		// array.
		for (int i = 1; !done && i < values.length; i++) {
			if (input <= values[i][0]) {
				// We found where the point falls in out array, between index i
				// and i-1
				// Calculate the equation for the line. y = m*x + b
                m = (values[i][1] - values[i - 1][1]) / (values[i][0] - values[i - 1][0]);
                b = values[i][1] - (m * values[i][0]);
				retVal = m * input + b;

				// we're finished, don't continue to loop
				done = true;
			}
		}
		
		return retVal;
	}
}