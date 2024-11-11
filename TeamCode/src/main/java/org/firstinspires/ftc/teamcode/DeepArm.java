package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private int count;

    private static final double TICKS_PER_INCH = 123.3793 * 2.67;
    private static final double TICKS_PER_REVOLUTION = 5281.1;

    private static final double STARTING_ANGLE = -50;
    private static final double MAX_ANGLE = 95;
    private static final double ARM_BASE_HEIGHT = 15.5;
    private static final double ARM_BASE_LENGTH = 13.9375;
    // axle on ground out to arm resting point on the ground
    private static final double ARM_FRONT_DISTANCE = 15.125;

    private static final int ARM_LENGTH_MIN = 5;
    private static final int ARM_LENGTH_MAX = (9 * (int)TICKS_PER_INCH);
    private static final int ARM_ROTATE_MIN =  5 * (int)TICKS_PER_REVOLUTION / 360;
    private static final int ARM_ROTATE_MAX = 145 * (int)TICKS_PER_REVOLUTION / 360;

    private static final double ARM_EXTEND_SPEED = 0.4 * 2.67;
    private static final double ARM_ROTATE_SPEED = 2.5;
    private static final int DEGREES_PER_SECOND = 45;
    private static final int ALLOWED_TICKS_OFFSET = 30;

    private Telemetry telemetry;
    private LinearOpMode opMode;

    private long armModeTime = 0;
    private boolean isRotateFar = true;

    public enum ArmMode {
        Pickup,
        Score,
        Lifted,
        Off
    }

    private double mapRange(double value, double inputStart, double inputEnd, double outputStart, double outputEnd) {
        return outputStart + (outputEnd - outputStart) * ((value - inputStart) / (inputEnd - inputStart));
    }

    public void init(HardwareMap hMap, Telemetry telemetry, LinearOpMode opMode) {

        this.telemetry = telemetry;
        this.opMode = opMode;

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
        armBase.setDirection(DcMotorSimple.Direction.REVERSE);

        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setTargetPosition(0);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armBase.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0 ,0));
        armExtend.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));
    }

    public void setArmState(double rotation, double extendInches, ArmMode state, boolean doBlock) {
        while (true) {
            if (state != armState) {
                armModeTime = System.currentTimeMillis();
            }
            if (state == ArmMode.Pickup && armState == ArmMode.Lifted) {
                isRotateFar = false;
            } else if (state == ArmMode.Lifted && armState == ArmMode.Pickup) {
                isRotateFar = false;
            } else if (state != armState) {
                isRotateFar = true;
            }
            armState = state;
            telemetry.addData("Rotation input", rotation);
            telemetry.addData("Extend inches input", extendInches);
            switch (state) {
                case Pickup:
                    if (!isRotateFar) {
                        setArmPosition(-1.33, 5.6);
                    } else {
                        extendArm(0);
                        if (armExtend.getCurrentPosition() < TICKS_PER_INCH / 1.5) {
                            rotateArm(-35);
                            if (armBase.getCurrentPosition() > TICKS_PER_REVOLUTION / (360.0 / 33.0)) {
                                isRotateFar = false;
                            }
                        }
                    }
                    break;
                case Score:
                    if (!isRotateFar) {
                        setArmPosition(-15.125, 40.49);
                    } else {
                        extendArm(0);
                        if (armExtend.getCurrentPosition() < TICKS_PER_INCH / 1.5) {
                            rotateArm(100);
                            if (armBase.getCurrentPosition() > TICKS_PER_REVOLUTION / (360.0 / 94.0)) {
                                isRotateFar = false;
                            }
                        }
                    }
                    break;
                case Lifted:
                    if (!isRotateFar) {
                        setArmPosition(1, ARM_BASE_HEIGHT);
                    } else {
                        extendArm(0);
                        if (armExtend.getCurrentPosition() < TICKS_PER_INCH / 1.5) {
                            rotateArm(ARM_BASE_HEIGHT);
                            if (armBase.getCurrentPosition() > TICKS_PER_REVOLUTION / (360.0 / 2.0)) {
                                isRotateFar = false;
                            }
                        }
                    }
                    break;
                case Off:
                    rotateArmOffset(rotation);
                    extendArmOffset(extendInches);
                    break;
            }
            if (opMode == null || !doBlock || opMode.isStopRequested()) {
                break;
            }
            if (isAtTarget() && !isRotateFar) {
                break;
            }
        }

    }

    public void rotateArm(double degrees) {
        armBaseDegrees = degrees;
        if ((System.currentTimeMillis() - lastRotateTime) > 1000) {
            lastRotateTime = System.currentTimeMillis();
            return;
        }

//         if (Math.abs((armBaseDegrees - armBaseDegreesCurrent) / (double)((System.currentTimeMillis() - lastRotateTime) / 1000)) > DEGREES_PER_SECOND) {
//             armBaseDegreesCurrent = DEGREES_PER_SECOND * ((double)(System.currentTimeMillis() - lastRotateTime) / 1000);
//         } else {
        armBaseDegreesCurrent = armBaseDegrees;
//         }
        telemetry.addData("Arm base degrees current", armBaseDegreesCurrent);
        int armBaseTicks = (int)(((armBaseDegreesCurrent - STARTING_ANGLE) * TICKS_PER_REVOLUTION) / 360);
        telemetry.addData("Arm base ticks", armBaseTicks);
         armBaseTicks = max(armBaseTicks, ARM_ROTATE_MIN);
        armBaseTicks = min(armBaseTicks, ARM_ROTATE_MAX);

        armBase.setTargetPosition(armBaseTicks);
        armBase.setPower(0.4);
        lastRotateTime = System.currentTimeMillis();
    }
    public void rotateArmOffset(double speed) {
        rotateArm(armBaseDegrees + speed * ARM_ROTATE_SPEED);
    }
    public void extendArm(double armLength) {
        armExtendInches = armLength;
        int armExtendTicks = (int) (armLength * TICKS_PER_INCH);
        armExtendTicks = max(armExtendTicks, ARM_LENGTH_MIN);
        armExtendTicks = min(armExtendTicks, ARM_LENGTH_MAX);
        armExtendInches = max(armExtendInches, ARM_LENGTH_MIN / TICKS_PER_INCH);
        armExtendInches = min(armExtendInches, ARM_LENGTH_MAX / TICKS_PER_INCH);
        telemetry.addData("Arm extend ticks", armExtendTicks);
        armExtend.setTargetPosition(armExtendTicks);
        armExtend.setPower(1);
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
        double targetAngle = (atan2(inchesUpOffset, inchesOutOffset)) * 180 / PI;
        double targetLength = sqrt(inchesOutOffset * inchesOutOffset + inchesUpOffset * inchesUpOffset) - ARM_BASE_LENGTH;

        extendArm(targetLength);
        rotateArm(targetAngle);

    }



    public void addTelemetry(Telemetry telemetry) {
        count += 1;
        telemetry.addData("Arm rotation target", armBase.getTargetPosition());
        telemetry.addData("Arm rotation actual", armBase.getCurrentPosition());
        telemetry.addData("Arm rotation degrees", armBaseDegrees);
        telemetry.addData("Arm extention target", armExtend.getTargetPosition());
        telemetry.addData("Arm extention actual", armExtend.getCurrentPosition());
        telemetry.addData("Arm extention inches", armExtendInches);
        telemetry.addData("Times called", count);
        telemetry.addData("Arm mode", armState);
    }

    public boolean isAtTarget() {
        double differenceRotate = Math.abs(armBase.getCurrentPosition() - armBase.getTargetPosition());
        double differenceExtend = Math.abs(armExtend.getCurrentPosition() - armExtend.getTargetPosition());
        return differenceRotate < ALLOWED_TICKS_OFFSET && differenceExtend < ALLOWED_TICKS_OFFSET;
    }

}
