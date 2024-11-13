package org.firstinspires.ftc.teamcode;

/**A Utility class for FTC robotics 2023-present
 * Best wishes and happy programming!
 *
 * @author Logan R
 */
public class LogsUtils {

    /**rounds the input to the p decimal place.
     *
     * @param input value
     * @param p decimal position
     * @return rounded value
     */
    public static double roundBetter(double input, double p)
    {
        return Math.round(input * Math.pow(10,p)) / Math.pow(10,p);
    }

    /**
     * remaps input so small values near 0 are 0 and values above a threshold are valued.
     * The function will reach 1 when input is 1.
     *
     * @param input the input to be mapped
     * @param a a non-negative real number identifying the deadzone
     */
    public static double deadZone(double input,double a) {
        return Math.max(input*(1+a),a)+Math.min(input*(1+a),-a);
    }

    /**
     * Maps input to a exponential equation keeping its sign. (calculated in desmos)
     * @param input the input to be mapped
     * @param exponent the power of the equation
     */
    public static double exponentialRemapAnalog(double input, double exponent) {
        return Math.min(Math.pow(Math.max(input,0),exponent),1) + Math.max(-Math.pow(Math.min(input,0),exponent),-1);
    }

    /**
     * Returns the input clamped
     * @param input
     * @param min
     * @param max
     */
    public static double clamp(double input, double min, double max)
    {
        return Math.min(Math.max(input,min),max);
    }
}