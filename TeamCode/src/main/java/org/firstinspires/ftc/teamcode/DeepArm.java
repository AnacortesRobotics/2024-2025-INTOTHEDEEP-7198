package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.*;

public class DeepArm {

    private DcMotorEx armBase;
    private DcMotorEx armExtend;
    private DigitalChannel armLimit;
    //private DigitalChannel armLimitMagnet;
    private AnalogInput absRotatePos;

    private double armBaseDegrees = 0 + 4; // was starting angle
    private double armExtendInches = 0;
    private boolean pickupMode = false;
    private ArmMode armMode = ArmMode.Off;
    private double armBaseDegreesCurrent = armBaseDegrees;
    private long lastRotateTime = 0;
    private int count;
    private int armBaseOffset = 0;
    private boolean hasInitialized = false;

    private static final double TICKS_PER_INCH = 279.43;
    private static final double TICKS_PER_REVOLUTION = 384.5 * 28;
    // one full 360 of the arm

    // Correct the positions, especially picking up the 2 and 3 samples.
    // make sure the grabber movements are at the right time


    //private static final double STARTING_ANGLE = -50;
    private static final double MAX_ANGLE = 90; //100
    private static final double ARM_BASE_HEIGHT = 7.6875;
    private static final double ARM_BASE_LENGTH = 13.75;     // not used
    // axle on ground out to arm resting point on the ground
    private static final double ARM_FRONT_DISTANCE = 15.125;   // not used

    private static final int ARM_LENGTH_MIN = 10;
    private static final int ARM_LENGTH_MAX = (int)(17.8 * TICKS_PER_INCH);
    private static final int ARM_ROTATE_MIN =  -15 * (int)TICKS_PER_REVOLUTION / 360;
    private static final int ARM_ROTATE_MAX = (int)((MAX_ANGLE) * TICKS_PER_REVOLUTION / 360);

    private static final double ARM_EXTEND_SPEED = 0.8;
    private static final double ARM_ROTATE_SPEED = 3;
    private static final int DEGREES_PER_SECOND = 45;
    private static final int ALLOWED_TICKS_OFFSET = 30;

    private static final double POTENTIOMETER_OFFSET = 0;

    private Telemetry telemetry;
    private LinearOpMode opMode;

    private long armModeTime = 0;
    private boolean isRotateFar = false;
    private ArmState armState = ArmState.Stop;
    private double degreesTarget = 0;
    private double inchesTarget = 0;
    private long delayMS = 0;
    private long lastArmMoveCall = 0;

    private boolean debugFlag = false;

    public enum ArmMode {
        Pickup,
        Score,
        Lifted,
        Off
    }

    public enum ArmState {
        Stop,
        Retract,
        FarMove,
        NearMove
    }

    private double mapRange(double value, double inputStart, double inputEnd, double outputStart, double outputEnd) {
        return outputStart + (outputEnd - outputStart) * ((value - inputStart) / (inputEnd - inputStart));
    }

    public void init(HardwareMap hMap, Telemetry telemetry, LinearOpMode opMode) {

        this.telemetry = telemetry;
        this.opMode = opMode;

        armBase = hMap.get(DcMotorEx.class, "armBase");
        armExtend = hMap.get(DcMotorEx.class, "armExtend");
//        armExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        armLimit = hMap.get(DigitalChannel.class, "armLimit");
        //armLimitMagnet = hMap.get(DigitalChannel.class, "armLimitMagnet");
        absRotatePos = hMap.get(AnalogInput.class, "absRotatePos");

        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armBase.setTargetPosition(0);
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armBase.setDirection(DcMotorSimple.Direction.REVERSE);

        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setTargetPosition(0);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armExtend.setDirection(DcMotorSimple.Direction.REVERSE);

        armBase.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(30, 0, 0 ,0));
        armExtend.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(5, 0, 0, 0));

        armBaseOffset = 0; // abs pos to ticks
    }

    public void manualArmControl(double rotateSpeed, double extendSpeed) {
        telemetry.addData("Rotation input", rotateSpeed);
        telemetry.addData("Extend inches input", extendSpeed);
        if (armState == ArmState.Stop) {
            rotateArmOffset(rotateSpeed);
            extendArmOffset(extendSpeed);
        }
    }

    public void updateLimit() {
        if (isArmLimitDown() && !hasInitialized) {
            armExtend.setPower(0);
            armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armExtend.setTargetPosition(10);
            armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hasInitialized = true;
        }
    }

    public void update() {
        updateLimit();
        telemetry.addData("Arm state case", armState);
        if (System.currentTimeMillis() - lastArmMoveCall > delayMS) {
            switch (armState) {
                case Stop:
                    return;
                case Retract:
                    retractArm();
                    if (isAtTarget()) {
                        armState = ArmState.FarMove;
                    }
                    return;
                case FarMove:
                    moveArmFar();
                    if (isAtTarget()) {
                        armState = ArmState.NearMove;
                    }
                    return;
                case NearMove:
                    moveArm();
                    if (isAtTarget()) {
                        armState = ArmState.Stop;
                    }
            }
        }
    }

    public void moveArm() {
        rotateArm(degreesTarget);
        extendArm(inchesTarget);
    }

    public void moveArmFar() {
        rotateArm(degreesTarget);
    }

    public void retractArm() {
        extendArm(0);
    }

    public void setArmTarget(ArmMode mode, long delay) {
        lastArmMoveCall = System.currentTimeMillis();
        delayMS = delay;
        if (mode == ArmMode.Pickup && armMode == ArmMode.Lifted) {
            armState = ArmState.NearMove;
        } else if (mode == ArmMode.Lifted && armMode == ArmMode.Pickup) {
            armState = ArmState.NearMove;
        } else if (mode != armMode) {
            armState = ArmState.Retract;
        }
        armMode = mode;
        switch (mode) {
            case Pickup:
                degreesTarget = -6;
                inchesTarget = 3;
                break;
            case Lifted:
                degreesTarget = 0;
                inchesTarget = 0;
                break;
            case Score:
                degreesTarget = 90; // 98
                inchesTarget = 17.8;
                break;
            case Off:
                return;
        }
    }

    public void rotateArm(double degrees) {
        armBaseDegrees = degrees;
//        if ((System.currentTimeMillis() - lastRotateTime) > 1000) {
//            lastRotateTime = System.currentTimeMillis();
//            return;
//        }

//         if (Math.abs((armBaseDegrees - armBaseDegreesCurrent) / (double)((System.currentTimeMillis() - lastRotateTime) / 1000)) > DEGREES_PER_SECOND) {
//             armBaseDegreesCurrent = DEGREES_PER_SECOND * ((double)(System.currentTimeMillis() - lastRotateTime) / 1000);
//         } else {
        armBaseDegreesCurrent = armBaseDegrees;
//         }
        telemetry.addData("Arm base degrees current", armBaseDegreesCurrent);
        int armBaseTicks = (int)((armBaseDegreesCurrent * TICKS_PER_REVOLUTION) / 360) - armBaseOffset;
        telemetry.addData("Arm base ticks", armBaseTicks);
        armBaseTicks = max(armBaseTicks, ARM_ROTATE_MIN - armBaseOffset);
        armBaseTicks = min(armBaseTicks, ARM_ROTATE_MAX - armBaseOffset);

        armBase.setTargetPosition(armBaseTicks);
        armBase.setPower(1);
        //lastRotateTime = System.currentTimeMillis();
    }
    public void rotateArmOffset(double speed) {
        rotateArm(armBaseDegrees + speed * ARM_ROTATE_SPEED);
    }
    public void extendArm(double armLength) {
        if (!hasInitialized) {
            if (armExtendInches < armLength) {
                armExtend.setPower(0);
                return;
            }
        }
        armExtendInches = armLength;
        int armExtendTicks = (int) (armLength * TICKS_PER_INCH);
        if (hasInitialized) {
            armExtendTicks = max(armExtendTicks, ARM_LENGTH_MIN);
            armExtendTicks = min(armExtendTicks, getRotatePosition() < 1800 ? ARM_LENGTH_MAX / 3 : ARM_LENGTH_MAX);
            armExtendInches = max(armExtendInches, ARM_LENGTH_MIN / TICKS_PER_INCH);
            armExtendInches = min(armExtendInches, ARM_LENGTH_MAX / TICKS_PER_INCH);
        }
        telemetry.addData("Arm extend ticks", armExtendTicks);
        armExtend.setTargetPosition(armExtendTicks);
        armExtend.setPower(!hasInitialized ? .5 : 1);
    }
    public void extendArmOffset(double speed) {
        extendArm(armExtendInches + speed * ARM_EXTEND_SPEED);

    }

    public boolean isArmLimitDown() {
        return !armLimit.getState();
        // returns true when not pressed            with ! returns true when pressed
    }
//    public boolean isArmLimitMagnetDown() {
//        return armLimitMagnet.getState();
//        // returns true when not pressed
//    }

    public void setArmPosition(double inchesFromFront, double inchesFromGround) {
        double inchesUpOffset = inchesFromGround - ARM_BASE_HEIGHT;
        double inchesOutOffset = ARM_FRONT_DISTANCE + inchesFromFront;
        double targetAngle = (atan2(inchesUpOffset, inchesOutOffset)) * 180 / PI;
        double targetLength = sqrt(inchesOutOffset * inchesOutOffset + inchesUpOffset * inchesUpOffset) - ARM_BASE_LENGTH;

        extendArm(targetLength);
        rotateArm(targetAngle);

    }

    public void addTelemetry(Telemetry telemetry) {
        count += 1;
        telemetry.addData("Arm rotation target", armBase.getTargetPosition());
        telemetry.addData("Arm rotation actual", getRotatePosition());
        telemetry.addData("Arm rotation degrees", armBaseDegrees);
        telemetry.addData("Arm extention target", armExtend.getTargetPosition());
        telemetry.addData("Arm extention actual", armExtend.getCurrentPosition());
        telemetry.addData("Arm extention inches", armExtendInches);
        telemetry.addData("Times called", count);
        telemetry.addData("Arm mode", armMode);
        telemetry.addData("Did it reach (the gameshow)", debugFlag);
        telemetry.addData("Arm limit switch is pressed", isArmLimitDown());
        telemetry.addData("Has it initialized", hasInitialized);
    }

    public boolean isStopped() {
        return armState == ArmState.Stop;
    }

    public boolean isAtTarget() {
        double differenceRotate = Math.abs(getRotatePosition() - armBase.getTargetPosition());
        double differenceExtend = Math.abs(armExtend.getCurrentPosition() - armExtend.getTargetPosition());
        return differenceRotate < ALLOWED_TICKS_OFFSET && differenceExtend < ALLOWED_TICKS_OFFSET;
    }

    public int absPosToTicks() {
        return (int)((absRotatePos.getVoltage() / absRotatePos.getMaxVoltage() * 270.0 + POTENTIOMETER_OFFSET) * (TICKS_PER_REVOLUTION / 360.0));
    }

    public int getRotatePosition() {
        return armBase.getCurrentPosition() + armBaseOffset;
    }

}
