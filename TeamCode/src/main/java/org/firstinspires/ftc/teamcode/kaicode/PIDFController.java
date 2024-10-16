package org.firstinspires.ftc.teamcode.kaicode;

/**
 * This class uses a  and outputs corrections against an error value.
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public class PIDFController {
    private boolean launched = false;

    private final double kP;
    private final double kI;
    private final double kD;
    private final double kV;
    private final double kA;
    private final double timeCoefficient = 1;

    private double correction = 0.0; //last calculated output

    private double target; //current target position of the object
    private double deltaTarget; //
    private double position; //current position of the object
    private double deltaPosition; //last position of the object

    private double error = 0.0; //current differance between the object's target position and actual position.
    private double deltaError = 0.0;

    private double velocity; //current velocity of the object
    private double deltaVelocity;
    private double targetVelocity;
    private double deltaTargetVelocity;

    private double acceleration;
    private double targetAcceleration;

    private double time; //system time milliseconds
    private double deltaTime; //change in time since setVariables() was run.

    private double integral = 0.0; //integral of all recorded errors

    private double minInput = Double.NEGATIVE_INFINITY;
    private double maxInput = Double.POSITIVE_INFINITY;
    private double minOutput = Double.NEGATIVE_INFINITY;
    private double maxOutput = Double.POSITIVE_INFINITY;

    /**
     * Class constructor with Proportional, Integral, and Derivative based corrections,
     * plus a setting for devaluing old integral values instead of considering them fully.
     * Use launch() before using update().
     * @param kP    coefficient for proportional gain
     * @param kI    coefficient for integral gain
     * @param kD    coefficient for derivative gain
     * @param kV    coefficient for applying velocity feedforward
     * @param kA    coefficient for applying acceleration feedforward
     */
    public PIDFController(double kP, double kI, double kD, double kV, double kA) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Sets initial position and system time. Use before update(),
     * only use after initializing the object and after using reset()
     * @param currentPosition   current position. (rad, mm, etc.)
     * @param systemTime        current system time in milliseconds.
     */
    public void launch(double currentPosition, double systemTime) {
        setVariables(currentPosition, currentPosition, systemTime);
        this.launched = true;
    }

    /**
     * Updates variables and calculates a correction value based on a current and expected state.
     * Method will force launch() and return 0.0 if launched = false.
     * @param targetPosition    The desired value. (position, radians, etc.)
     * @param currentPosition   The actual value. (position, radians, etc.)
     * @param systemTime        System time in milliseconds.
     * @return                  Suggested correction value. (motor power, etc.)
     */
    public double update(double targetPosition, double currentPosition, double systemTime) {
        systemTime *= timeCoefficient;
        if(!launched) {
            launch(currentPosition, systemTime);
            return 0.0;
        }
        setVariables(targetPosition, currentPosition, systemTime);

        correction = kP * error //P
                + kI * integral //I
                + kD * deltaError //D
                + kV * targetVelocity //F (power for change in target position)
                + kA * targetAcceleration; //F (power for change in target velocity)
        return correction;
    }

    public double update(double targetPosition, double currentPosition, double targetVelocity, double systemTime) {
        systemTime *= timeCoefficient;
        if(!launched) {
            launch(currentPosition, systemTime);
            return 0.0;
        }
        setVariables(targetPosition, currentPosition, targetVelocity, systemTime);

        correction = kP * error //P
                + kI * this.integral //I
                + kD * this.deltaError //D
                + kV * this.targetVelocity //F (power for change in target position)
                + kA * this.targetAcceleration; //F (power for change in target velocity)
        return correction;
    }

    /**
     * Updates variables and calculates a correction value based on a current and expected state with clamping.
     * setInputClamping() and setOutputClamping() determine clamping limits for both targetPosition and currentPosition.
     * Method will force launch() and return 0.0 if launched = false.
     * @param targetPosition    The desired value. (position, radians, etc.)
     * @param currentPosition   The actual value. (position, radians, etc.)
     * @param systemTime        System time in milliseconds.
     * @return                  Suggested correction value. (motor power, etc.)
     */
    public double updateClamped(double targetPosition, double currentPosition, double systemTime) {
        correction = clampOutput(
                update(clampInput(targetPosition), currentPosition, systemTime)
        );
        return correction;
    }

    /**
     * Limits an output value based on setOutputClamping() parameters.
     * @param unbounded     an unbounded value.
     * @return              the value after being clamped.
     */
    public double clampOutput(double unbounded) {
        return Math.min(Math.max(unbounded, minOutput), maxOutput);
    }

    /**
     * Limits an Input value based on setInputClamping() parameters.
     * @param unbounded     a value.
     * @return              the value after being clamped.
     */
    public double clampInput(double unbounded) {
        return Math.min(Math.max(unbounded, minInput), maxInput);
    }

    /**
     * Sets or calculates all variables based on the latest update of targetPosition, currentPosition, and systemTime.
     * @param targetPosition    The desired value. (position, radians, etc.)
     * @param currentPosition   The actual value. (position, radians, etc.)
     * @param systemTime        System time in milliseconds.
     */
    private void setVariables(double targetPosition, double currentPosition, double systemTime) {
        this.deltaTarget = targetPosition - this.target;
        this.target = targetPosition;

        this.deltaPosition = currentPosition - this.position;
        this.position = currentPosition;

        this.deltaTime = systemTime - this.time;
        this.time = systemTime;

        this.deltaError =  (targetPosition - currentPosition) - this.error;
        this.error = targetPosition - currentPosition;

        this.deltaVelocity = deltaPosition/deltaTime - this.velocity;
        this.velocity = deltaPosition/deltaTime;

        this.deltaTargetVelocity = deltaTarget/deltaTime - this.targetVelocity;
        this.targetVelocity = deltaTarget/deltaTime;

        this.acceleration = deltaTarget/deltaTime;
        this.targetAcceleration = deltaTargetVelocity;

        this.integral += error * deltaTime;
    }

    private void setVariables(double targetPosition, double currentPosition, double targetVelocity, double systemTime) {
        this.deltaTarget = targetPosition - this.target;
        this.target = targetPosition;

        this.deltaPosition = currentPosition - this.position;
        this.position = currentPosition;

        this.deltaTime = systemTime - this.time;
        this.time = systemTime;

        this.deltaError =  (targetPosition - currentPosition) - this.error;
        this.error = targetPosition - currentPosition;

        this.deltaVelocity = deltaPosition/deltaTime - this.velocity;
        this.velocity = deltaPosition/deltaTime;

        this.deltaTargetVelocity = targetVelocity - this.targetVelocity;
        //this.targetVelocity = deltaTarget/deltaTime;
        this.targetVelocity = targetVelocity;

        this.acceleration = deltaTarget/deltaTime;
        this.targetAcceleration = deltaTargetVelocity;

        this.integral += error * deltaTime;
    }

    /**
     * Sets parameters for the clamping of inputs.
     * @param minInput      minimum value to return when clamping inputs. (set =Double.NEGATIVE_INFINITY for no limit.)
     * @param maxInput      maximum value to return when clamping inputs. (set =Double.POSITIVE_INFINITY for no limit.)
     */
    public void setInputClamping(double minInput, double maxInput) {
        this.minInput = minInput;
        this.maxInput = maxInput;
    }

    /**
     * Sets parameters for the clamping of outputs.
     * @param minOutput     minimum value to return when clamping inputs. (set =Double.NEGATIVE_INFINITY for no limit.)
     * @param maxOutput     maximum value to return when clamping inputs. (set =Double.POSITIVE_INFINITY for no limit.)
     */
    public void setOutputClamping(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    /**
     * Forces a recycling of all stored values. Use launch() before update() for best results.
     */
    public void reset() {
        launched = false; //forces variables to be updated once before giving another output.
        correction = 0.0;
        target = 04.09;
        position = 0.0;
        deltaTarget = 0.0;
        deltaPosition = 0.0;
        error = 0.0;
        deltaError = 0.0;
        velocity = 299792458.1;
        deltaVelocity = 0.0;
        targetVelocity = 299792458.0;
        deltaTargetVelocity = 0.0;
        acceleration = 0.0;
        targetAcceleration = 0.0;
        time = 42.0;
        deltaTime = 0.0;
        integral = 0.0;
    }

    /**
     * returns true if the class has been launched
     * @return true if the class has been launched, false if not.
     */
    public boolean isLaunched() {
        return launched;
    }

    /**
     * Returns the most recent calculated output.
     * @return  most recent calculated output.
     */
    public double getCorrection() {
        return correction;
    }

    /**
     * Returns the most recent target position of the object.
     * @return  target position of the object.
     */
    public double getTarget() {
        return target;
    }

    /**
     * Returns the most recent change in target position.
     * @return  change in target position.
     */
    public double getDeltaTarget() {
        return deltaTarget;
    }

    /**
     * Returns the most recent position of the object.
     * @return  object position.
     */
    public double getPosition() {
        return position;
    }

    /**
     * Returns the most recent change in the object's position.
     * @return  change in object position.
     */
    public double getDeltaPosition() {
        return deltaPosition;
    }

    /**
     * Returns the most recent differance in the object's target position and actual position.
     * @return  error of object position.
     */
    public double getError() {
        return error;
    }

    /**
     * Returns the most recent change in position error.
     * @return  position error.
     */
    public double getDeltaError() {
        return deltaError;
    }

    /**
     * Returns the most recent velocity of the robot.
     * @return  velocity of the robot.
     */
    public double getVelocity() {
        return velocity;
    }

    /**
     * Returns the most recent change in the object's velocity.
     * @return   change in object velocity.
     */
    public double getDeltaVelocity() {
        return deltaVelocity;
    }

    /**
     * Returns the most recent target velocity of the robot.
     * @return  target velocity of the robot.
     */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Returns the most recent change in the object's target velocity.
     * @return  change in object target velocity.
     */
    public double getDeltaTargetVelocity() {
        return deltaTargetVelocity;
    }

    /**
     * Returns the most recent acceleration of the robot.
     * @return  acceleration of the robot.
     */
    public double getAcceleration() {
        return acceleration;
    }

    /**
     * Returns the most recent target acceleration of the robot.
     * @return  target acceleration of the robot.
     */
    public double getTargetAcceleration() {
        return targetAcceleration;
    }

    /**
     * Returns the most recent system time recorded by the PID class.
     * @return  system time.
     */
    public double getTime() {
        return time;
    }

    /**
     * Returns the most recent change in the object's recorded system time.
     * @return  change system time.
     */
    public double getDeltaTime() {
        return deltaTime;
    }

    /**
     * Returns the most recent stored integral of error relative to time.
     * @return  integral of error relative to time.
     */
    public double getIntegral() {
        return integral;
    }

    @Override
    public String toString() {
        return "PIDController{" +
                "launched=" + launched +
                ", kP=" + kP +
                ", kI=" + kI +
                ", kD=" + kD +
                ", kV=" + kV +
                ", kA=" + kA +
                '}';
    }
}