package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.*;

public class DeepArm {

    private DcMotorEx armBase;
    private DcMotorEx armExtend;
    private DigitalChannel armLimit;
    private DigitalChannel armLimitMagnet;

    private double armBaseDegrees = STARTING_ANGLE + 4;
    private double armExtendInches = 0;
    private boolean pickupMode = false;
    private ArmMode armState = ArmMode.Off;
    private double armBaseDegreesCurrent = armBaseDegrees;
    private long lastRotateTime = 0;

    private static final int ARM_LENGTH_MIN = 5;
    private static final int ARM_LENGTH_MAX = 450;
    private static final double TICKS_PER_INCH = 41.3793;
    private static final double TICKS_PER_REVOLUTION = 537.7 * 2;
    private static final double STARTING_ANGLE = -33;
    private static final double ARM_BASE_HEIGHT = 9.59375;
    private static final double ARM_BASE_LENGTH = 14.34375;
    private static final double ARM_FRONT_DISTANCE = 15.125;
    private static final double ARM_EXTEND_SPEED = 0.15;
    private static final double ARM_ROTATE_SPEED = 1.5;
    private static final int ARM_ROTATE_MIN = (int) (STARTING_ANGLE * TICKS_PER_REVOLUTION / 360);
    private static final int ARM_ROTATE_MAX = 400;
    private static final int DEGREES_PER_SECOND = 45;
    private Telemetry telemetry;

    public enum ArmMode {
        Pickup,
        Score,
        Lifted,
        Off
    }

    private double mapRange(double value, double inputStart, double inputEnd, double outputStart, double outputEnd) {
        return outputStart + (outputEnd - outputStart) * ((value - inputStart) / (inputEnd - inputStart));
    }

    public void init(HardwareMap hMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        armBase = hMap.get(DcMotorEx.class, "armBase");
        armExtend = hMap.get(DcMotorEx.class, "armExtend");
        armExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        armLimit = hMap.get(DigitalChannel.class, "armLimit");
        armLimitMagnet = hMap.get(DigitalChannel.class, "armLimitMagnet");

        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armBase.setTargetPosition(0);
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setTargetPosition(0);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armBase.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(15, 0, 0 ,0));
        armExtend.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(6, 0, 0, 0));
    }

    public void setArmState(double rotation, double extendInches, ArmMode state) {
        armState = state;
        switch (state) {
            case Pickup:
                setArmPosition(3, -2.5);
                break;
            case Score:
                // must change, test more
                setArmPosition(6, 25);
                break;
            case Lifted:
                setArmPosition(1, ARM_BASE_HEIGHT);
                break;
            case Off:
                rotateArmOffset(rotation);
                extendArmOffset(extendInches);
                break;
        }

    }

    public void rotateArm(double degrees) {
        armBaseDegrees = degrees;
        if ((System.currentTimeMillis() - lastRotateTime) > 1000) {
            lastRotateTime = System.currentTimeMillis();
            return;
        }

         if (Math.abs((armBaseDegrees - armBaseDegreesCurrent) / ((System.currentTimeMillis() - lastRotateTime) / 1000)) > DEGREES_PER_SECOND) {
             armBaseDegreesCurrent = DEGREES_PER_SECOND * ((System.currentTimeMillis() - lastRotateTime) / 1000);
         } else {
             armBaseDegreesCurrent = armBaseDegrees;
         }
        int armBaseTicks = (int)(((armBaseDegreesCurrent - STARTING_ANGLE) * TICKS_PER_REVOLUTION) / 360);
        armBaseTicks = Math.max(armBaseTicks, ARM_ROTATE_MIN);
        armBaseTicks = Math.min(armBaseTicks, ARM_ROTATE_MAX);
        telemetry.addData("Arm base ticks", armBaseTicks);
        armBase.setTargetPosition(armBaseTicks);
        armBase.setPower(1);
        lastRotateTime = System.currentTimeMillis();
    }
    public void rotateArmOffset(double speed) {
        rotateArm(armBaseDegrees + speed * ARM_ROTATE_SPEED);
    }

    public void extendArm(double armLength) {
        armExtendInches = armLength;
        int armExtendTicks = (int) (armLength * TICKS_PER_INCH);
        armExtendTicks = Math.max(armExtendTicks, ARM_LENGTH_MIN);
        armExtendTicks = Math.min(armExtendTicks, ARM_LENGTH_MAX);
        telemetry.addData("Arm extend ticks", armExtendTicks);
        armExtend.setTargetPosition(armExtendTicks);
        armExtend.setPower(0.8);
    }
    public void extendArmOffset(double speed) {
        extendArm(armExtendInches + speed * ARM_EXTEND_SPEED);
    }

    public boolean isArmLimitDown() {
        return armLimit.getState();
        // returns true when not pressed
    }
    public boolean isArmLimitMagnetDown() {
        return armLimitMagnet.getState();
        // returns true when not pressed
    }

    public void setArmPosition(double inchesFromFront, double inchesFromGround) {
        double inchesUpOffset = inchesFromGround - ARM_BASE_HEIGHT;
        double inchesOutOffset = ARM_FRONT_DISTANCE + inchesFromFront;
        double targetAngle = (atan2(inchesUpOffset, inchesOutOffset)) * 180 / Math.PI;
        double targetLength = sqrt(inchesOutOffset * inchesOutOffset + inchesUpOffset * inchesUpOffset) - ARM_BASE_LENGTH;

        extendArm(targetLength);
        rotateArm(targetAngle);

    }



    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Arm rotation target", armBase.getTargetPosition());
        telemetry.addData("Arm rotation actual", armBase.getCurrentPosition());
        telemetry.addData("Arm rotation degrees", armBaseDegrees);
        telemetry.addData("Arm extention target", armExtend.getTargetPosition());
        telemetry.addData("Arm extention actual", armExtend.getCurrentPosition());
        telemetry.addData("Arm extention inches", armExtendInches);
        telemetry.addData("Arm mode", armState);
    }

}
