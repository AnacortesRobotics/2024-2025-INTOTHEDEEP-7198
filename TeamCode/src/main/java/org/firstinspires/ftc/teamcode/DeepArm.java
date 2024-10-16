package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.*;

public class DeepArm {

    private DcMotor armBase;
    private  DcMotor armExtend;
    private DigitalChannel armLimit;
    private DigitalChannel armLimitMagnet;

    private double armBaseDegrees;
    private double armExtendInches;
    private boolean pickupMode = false;
    private ArmMode armState = ArmMode.Off;

    private static final int ARM_LENGTH_MIN = 5;
    private static final int ARM_LENGTH_MAX = 450;
    private static final double TICKS_PER_INCH = 41.3793;
    private static final double TICKS_PER_REVOLUTION = 537.7;
    private static final double STARTING_ANGLE = -33;
    private static final double ARM_BASE_HEIGHT = 9.59375;
    private static final double ARM_BASE_LENGTH = 14.34375;
    private static final double ARM_FRONT_DISTANCE = 15.125;
    private static final double ARM_EXTEND_SPEED = 0.01;
    private static final double ARM_ROTATE_SPEED = 0.01;
    // 10 7/8 inches for 450 ticks: for math
    // 0.0241666 repeating

    public enum ArmMode {
        Pickup,
        Score,
        Lifted,
        Off
    }

    private double mapRange(double value, double inputStart, double inputEnd, double outputStart, double outputEnd) {
        return outputStart + (outputEnd - outputStart) * ((value - inputStart) / (inputEnd - inputStart));
    }

    public void init(HardwareMap hMap) {

        armBase = hMap.get(DcMotor.class, "armBase");
        armExtend = hMap.get(DcMotor.class, "armExtend");
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

    }

    public void setArmState(double rotation, double extendInches, ArmMode state) {
        armState = state;
        switch (state) {
            case Pickup:
                setArmPosition(3, 1);
                break;
            case Score:
                setArmPosition(3, 29);
                break;
            case Lifted:
                setArmPosition(3, ARM_BASE_HEIGHT);
                break;
            case Off:
                rotateArmOffset(rotation);
                extendArmOffset(extendInches);
                break;
        }

    }

    public void rotateArm(double degrees) {
        armBaseDegrees = degrees;
        int armBaseTicks = (int)(((degrees - STARTING_ANGLE) * TICKS_PER_REVOLUTION) / 360);
        armBaseTicks = Math.max(armBaseTicks, 10);
        armBaseTicks = Math.min(armBaseTicks, 400);
        armBase.setTargetPosition(armBaseTicks);
        armBase.setPower(0.5);
    }
    public void rotateArmOffset(double speed) {
        rotateArm(armBaseDegrees + speed * ARM_ROTATE_SPEED);
    }

    public void extendArm(double armLength) {
        armExtendInches = armLength;
        int armExtendTicks = (int) (armLength * TICKS_PER_INCH);
        armExtendTicks = Math.max(armExtendTicks, ARM_LENGTH_MIN);
        armExtendTicks = Math.min(armExtendTicks, ARM_LENGTH_MAX);
        armExtend.setTargetPosition(armExtendTicks);
        armExtend.setPower(0.5);
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
        double inchesUpOffset = ARM_BASE_HEIGHT - inchesFromGround;
        double inchesOutOffset = ARM_FRONT_DISTANCE + inchesFromFront;
        double targetAngle = (atan2(inchesUpOffset, inchesOutOffset));
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
