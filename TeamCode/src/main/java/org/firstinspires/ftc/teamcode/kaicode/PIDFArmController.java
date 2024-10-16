package org.firstinspires.ftc.teamcode.kaicode;

public class PIDFArmController extends PIDFController {
    private double correction;
    private double kG;
    private double kG2;

    /**
     * Class constructor with Proportional, Integral, and Derivative based corrections,
     * plus a setting for devaluing old integral values instead of considering them fully.
     *
     * @param kP    coefficient for proportional gain
     * @param kI    coefficient for integral gain
     * @param kD    coefficient for derivative gain
     * @param kV    coefficient for applying velocity feedforward
     * @param kG    coefficient for torque applied from the controlled arm
     * @param kG2   coefficient the arm immediately above the controlled arm in radians
     */
    public PIDFArmController(double kP, double kI, double kD, double kV, double kA, double kG, double kG2) {
        super(kP, kI, kD, kV, kA);
        this.kG = kG;
        this.kG2 = kG2;
    }

    /**
     * Updates variables and calculates a correction value based on a current and expected state.
     * Method will force launch() and return 0.0 if super.launched = false.
     * @param targetAngleRad    target angle of the controlled arm in radians.
     * @param angleRad          the angle of the controlled arm in radians, relative to the x-axis.
     * @param angle2Rad         the angle of the arm immediately above the controlled arm in radians,
     *                                  relative to the x-axis.
     * @param systemTime        the current system time, in milliseconds.
     * @return
     */
    public double updateArm(double targetAngleRad, double angleRad, double angle2Rad, double systemTime) {
        if(!super.isLaunched()) {
            launch(angleRad, systemTime);
            return 0.0;
        }

        this.correction = update(targetAngleRad, angleRad, systemTime);
        this.correction += kG * Math.cos(angleRad)     //gravity feedforward for weight of controlled arm.
                + kG2 * (Math.abs(Math.cos(angleRad)) - Math.abs(Math.cos(angle2Rad)));
        //gravity feedforward for weight of arm immediately above the controlled arm.
        return correction;
    }

    /**
     * Updates variables and calculates a correction value based on a current and expected state with clamping.
     * setInputClamping() and setOutputClamping() determine clamping limits for both targetPosition and correction.
     * Method will force launch() and return 0.0 if super.launched = false.
     * @param targetAngleRad    target angle of the controlled arm in radians.
     * @param angleRad          the angle of the controlled arm in radians, relative to the x-axis.
     * @param angle2Rad         the angle of the arm immediately above the controlled arm in radians,
     *                                  relative to the x-axis.
     * @param systemTime        the current system time, in milliseconds.
     * @return
     */
    public double updateArmClamped(double targetAngleRad, double angleRad, double angle2Rad, double systemTime) {
        correction = clampOutput(
                updateArm(targetAngleRad, clampInput(angleRad), angle2Rad, systemTime)
        );
        return correction;
    }

    /**
     * Returns the most recent calculated output.
     * @return  most recent calculated output.
     */
    @Override
    public double getCorrection() {
        return this.correction;
    }
}