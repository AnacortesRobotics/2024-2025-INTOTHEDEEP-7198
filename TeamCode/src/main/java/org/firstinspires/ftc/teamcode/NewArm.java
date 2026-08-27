package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.max;
import static java.lang.Math.min;

@TeleOp
public class NewArm {

    private DcMotorEx armBase;
    private DcMotorEx armExtend;
    private DigitalChannel armLimit;
    private DigitalChannel armLimitRotation;
    private DigitalChannel armMaxRotation;

    private ArmState armState = ArmState.Stop;
    private Target target = Target.Lifted;

    private Telemetry telemetry;

    private double armBaseDegrees = 0;
    private double armBaseDegreesCurrent = 0;
    private double armExtendInches = 0;
    private double armExtendInchesCurrent = 0;
    private long delayMS = 0;
    private long lastArmMoveCall = 0;
    private double degreesTarget = 0;
    private double inchesTarget = 0;
    private boolean hasExtendInitialized = false;
    private boolean hasRotateInitialized = false;

    private static final double TICKS_PER_INCH = 73.94;
    // encoder ticks per revolution / 5.2
    private static final double TICKS_PER_REVOLUTION = 103.8 * 28 * 3;
    //number of ticks in 1 full revolution of the arm
    private static final double MAX_ANGLE = 90;
    private static final int ARM_ROTATE_MIN = 2 * (int)TICKS_PER_REVOLUTION / 360;
    private static final int ARM_ROTATE_MAX = (int)((MAX_ANGLE) * TICKS_PER_REVOLUTION / 360);
    private static final int ARM_LENGTH_MIN = 10;
    private static final int ARM_LENGTH_MAX = (int)(17.8 * TICKS_PER_INCH);
    private static final int ALLOWED_TICKS_OFFSET = 20;
    private static final double ARM_EXTEND_SPEED = 0.8;
    private static final double ARM_ROTATE_SPEED = 3.4;


    public enum Target {
        Pickup(8, Double.MAX_VALUE),
        Score(90, 17.5),
        Lifted(20, 1);

        double rotationTarget;
        double extentionTarget;

        Target(double rotationTarget, double extentionTarget) {
            this.rotationTarget = rotationTarget;
            this.extentionTarget = extentionTarget;
        }
    }

    public enum ArmState {
        Stop,
        Retract,
        Rotate,
        Extend,
        Lock
    }

    public void init(HardwareMap hMap, Telemetry telemetry, LinearOpMode opMode) {

        this.telemetry = telemetry;

        armBase = hMap.get(DcMotorEx.class, "armBase");

        armExtend = hMap.get(DcMotorEx.class, "armExtend");
        armLimit = hMap.get(DigitalChannel.class, "armLimit");
        armLimitRotation = hMap.get(DigitalChannel.class, "armLimitRotation");
        armMaxRotation = hMap.get(DigitalChannel.class, "armMaxRotation");

        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armBase.setTargetPosition(0);
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBase.setDirection(DcMotorSimple.Direction.REVERSE);
        armBase.setTargetPositionTolerance(15);

        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setTargetPosition(0);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtend.setTargetPositionTolerance(15);

        armBase.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(6, 0, 0 ,0));
        armExtend.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(4, 0, 0, 0));

    }


    public void manualArmMove(double rotateSpeed, double extendSpeed) {
        if (armState != ArmState.Stop) {
            return;
        }
        rotateArmOffset(rotateSpeed);
        extendArmOffset(extendSpeed);
    }

    public void update() {
        updateLimit();
        updateRotationLimit();
        updateMaxRotation();
        switch (armState) {
            case Stop:
                break;
            case Retract:
                retractArm();
                armState = ArmState.Rotate;
                break;
            case Rotate:
                moveArmAngle();
                armState = ArmState.Extend;
                break;
            case Extend:
                moveArm();
                armState = ArmState.Stop;
                break;
            case Lock:

                break;
        }
    }

    public void setArmTarget(Target currentTarget, long delay) {
        lastArmMoveCall = System.currentTimeMillis();
        delayMS = delay;
        if (currentTarget == Target.Pickup && target == Target.Lifted) {
            armState = ArmState.Extend;
        } else if (currentTarget == Target.Lifted && target == Target.Pickup) {
            armState = ArmState.Extend;
        } else if (currentTarget != target) {
            armState = ArmState.Retract;
        }
        target = currentTarget;
        degreesTarget = target.rotationTarget;
        if (target.extentionTarget == Double.MAX_VALUE) {
            inchesTarget = target.extentionTarget;
        }

    }
    public void moveArm() {
        rotateArm(degreesTarget);
        extendArm(inchesTarget);
    }

    public void moveArmAngle() {
        rotateArm(degreesTarget);
    }

    public void retractArm() {
        extendArm(0.8);
    }



    public void rotateArm(double degrees) {
        if (armState == ArmState.Lock) {
            return;
        }
        armBaseDegrees = degrees;
        armBaseDegreesCurrent = armBaseDegrees;

        int armBaseTicks = (int)(armBaseDegreesCurrent * TICKS_PER_REVOLUTION) / 360;
        armBaseTicks = max(armBaseTicks, ARM_ROTATE_MIN);
        armBaseTicks = min(armBaseTicks, ARM_ROTATE_MAX);

        armBase.setTargetPosition(armBaseTicks);
        armBase.setPower(1);
    }
    public void rotateArmOffset(double speed) {
        rotateArm(armBaseDegrees + speed * ARM_ROTATE_SPEED);
    }

    public void extendArm(double inches) {
        if (armState == ArmState.Lock) {
            return;
        }
        armExtendInches = inches;
        armExtendInchesCurrent = armExtendInches;

        int armExtendTicks = (int)(armExtendInchesCurrent * TICKS_PER_INCH);
        armExtendTicks = max(armExtendTicks, ARM_LENGTH_MIN);
        armExtendTicks = min(armExtendTicks, ARM_ROTATE_MAX);

        armExtend.setTargetPosition(armExtendTicks);
        armExtend.setPower(1);
    }
    public void extendArmOffset(double speed) {
        extendArm(armExtendInches + speed * ARM_EXTEND_SPEED);
    }

    public boolean isAtTarget() {
        double differenceRotate = Math.abs((getRotatePosition()) - armBase.getTargetPosition());
        double differenceExtend = Math.abs(armExtend.getCurrentPosition() - armExtend.getTargetPosition());
        return differenceRotate < ALLOWED_TICKS_OFFSET && differenceExtend < ALLOWED_TICKS_OFFSET;
    }

    public int getRotatePosition() {return armBase.getCurrentPosition();}

    public void updateLimit() {
        if (isArmLimitDown() && !hasExtendInitialized) {
            armExtend.setPower(0);
            armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armExtend.setTargetPosition(10);
            armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armExtend.setPower(1);
            hasExtendInitialized = true;
        }
    }

    public void updateRotationLimit() {
        if (isArmLimitRotateDown() && !hasRotateInitialized) {
            armBase.setPower(0);
            armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armBase.setTargetPosition(10);
            armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armBase.setPower(1);
            hasRotateInitialized = true;
        }
    }

    public void updateMaxRotation() {
        if (isArmMaxRotateDown()) {
            armBase.setPower(0);
            armBase.setTargetPosition(armBase.getCurrentPosition() - 5);
            armBase.setPower(1);
        }
    }

    public boolean isArmLimitDown() {return !armLimit.getState();}

    public boolean isArmLimitRotateDown() {return !armLimitRotation.getState();}

    public boolean isArmMaxRotateDown() {
        return !armMaxRotation.getState();
    }

    public boolean isStopped() {
        return armState == ArmState.Stop;
    }

    public void lock() {
        armBase.setPower(0);
        armExtend.setPower(0);
        armState = ArmState.Lock;
    }

    public void unlock() {armState = ArmState.Stop;}

}
