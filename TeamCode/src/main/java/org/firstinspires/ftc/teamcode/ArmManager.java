package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmManager {
    private DeepArm deepArm;
    private Intake intake;
    private Telemetry telemetry;
    private long lastCallTime = 0;
    private boolean debugFlag = false;

    private Pickup pickup = Pickup.Done;

    public enum Pickup {
        Lower,
        Grab,
        Lift,
        Done
    }

    public void init(HardwareMap hMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intake = new Intake();
        intake.init(hMap, telemetry);
        deepArm = new DeepArm();
        deepArm.init(hMap, telemetry, null);

    }

    public void setArmTarget(DeepArm.ArmMode armMode, long delay) {
        deepArm.setArmTarget(armMode, delay);
//        if (armMode == DeepArm.ArmMode.Score) {
//            I would like it to automatically move the wrist to the correct spot here,
//            for lifting and once it's at the top (Forward, move, score)
//        }
    }

    public void update() {
        if (!isPickupDone()) {
            pickupUpdate();
        }
        deepArm.update();
        intake.update();
    }

    public void setWristTarget(Intake.WristMode wristMode, long delay) {
        intake.setWristTarget(wristMode, delay);
        debugFlag = true;
    }

    public void setGrabberPosition(Intake.IntakeState intakeState) {
        intake.servoControl(intakeState);
    }

    public void pickupUpdate() {
        boolean didTimeout = System.currentTimeMillis() - lastCallTime > 1500;
        if (isAtTarget() || didTimeout) {
            lastCallTime = System.currentTimeMillis();
            switch (pickup) {
                case Lower:
                    setArmTarget(DeepArm.ArmMode.Pickup, 0);
                    setWristTarget(Intake.WristMode.Pickup, 300);
                    setGrabberPosition(Intake.IntakeState.Open);
                    pickup = Pickup.Grab;
                    break;
                case Grab:
                    intake.servoControl(Intake.IntakeState.Closed);
                    pickup = Pickup.Lift;
                    break;
                case Lift:
                    setArmTarget(DeepArm.ArmMode.Lifted, 0);
                    setWristTarget(Intake.WristMode.Pickup, 200);
                    pickup = Pickup.Done;
                    break;
                case Done:
                    break;
            }
        }
    }

    public void lineUpToGrab() {
        deepArm.setArmTarget(DeepArm.ArmMode.Pickup, 0);
        setWristTarget(Intake.WristMode.Pickup, 300);
        setGrabberPosition(Intake.IntakeState.LineUp);
    }

    public void startPickFromLineup() {
        pickup = Pickup.Grab;
    }

    public void scoreAndReturn() {
        setGrabberPosition(Intake.IntakeState.Open);
        setWristTarget(Intake.WristMode.Pickup, 300);
        setArmTarget(DeepArm.ArmMode.Lifted, 500);
    }

    public void startPickup() {
        pickup = Pickup.Lower;
    }

    public boolean isPickupDone() {
        return pickup == Pickup.Done;
    }

    public void manualArmMove(double rotate, double extend) {
        deepArm.manualArmControl(rotate, extend);
    }

    public void updateTelemetry() {
        deepArm.addTelemetry(telemetry);
        intake.addTelemetry();
        telemetry.addData("Arm at target", deepArm.isStopped());
        //telemetry.addData("Intake at target", intake.isIntakeDone());
        telemetry.addData("Wrist at target", intake.isWristDone());
        telemetry.addData("Did it get to here (the badshow)", debugFlag);
    }

    public boolean isAtTarget() {
        return intake.isIntakeDone() && intake.isWristDone() && deepArm.isAtTarget();
    }

    public boolean isArmDone() {
        return deepArm.isArmDone();
    }

    public void lock() {
        deepArm.lock();
        intake.lock();
    }

    public void unlock() {
        deepArm.unlock();
        intake.unlock();
    }

    public boolean isRotateLimitDown() {
        return deepArm.isArmLimitRotateDown();
    }

    public void extendArmToPosition(double extendInches) {
        deepArm.extendArm(extendInches);
    }

    public double getExtentionInches() {
        return deepArm.getArmExtendInches();
    }

    public int getRotationTicks() {
        return deepArm.getRotatePosition();
    }

//    public void lockWristPosition() {
//        deepArm.
//    }

}