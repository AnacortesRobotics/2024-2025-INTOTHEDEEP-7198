package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DeepArm {

    private DcMotor armBase;
    private  DcMotor armExtend;
    private DigitalChannel armLimit;
    private DigitalChannel armLimitMagnet;

    private int armBaseTicks;
    private int armExtendTicks;
    private boolean pickupMode = false;

    private double mapRange(double value, double inputStart, double inputEnd, double outputStart, double outputEnd) {
        return outputStart + (outputEnd - outputStart) * ((value - inputStart) / (inputEnd - inputStart));
    }

    public void init(HardwareMap hMap) {

        armBase = hMap.get(DcMotor.class, "armBase");
        armExtend = hMap.get(DcMotor.class, "armExtend");
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

    public void rotateArm(double rotation) {

        if (!pickupMode) {
            int tickOffset = (int) (rotation * 10);
            armBaseTicks += tickOffset;
            //armBaseTicks = Math.max(armBaseTicks, 30);
            //armBaseTicks = Math.min(armBaseTicks, 800);
            armBase.setTargetPosition(armBaseTicks);
            armBase.setPower(0.8);
        } else {
            // Untested
            armBase.setTargetPosition((int)mapRange(armExtendTicks, 20, 3000, 20, 100));
            armBase.setPower(0.8);
        }
    }

    public void extendArm(double armLength) {
        // Untested
        int tickOffsetExtend = (int) (armLength * 16);
        armExtendTicks += tickOffsetExtend;
        //armExtendTicks = Math.max(armExtendTicks, 20);
        //armExtendTicks = Math.min(armExtendTicks, 3000);
        armExtend.setTargetPosition(armExtendTicks);
        armExtend.setPower(0.5);
    }

    public boolean isArmLimitDown() {
        return armLimit.getState();
        // returns true when not pressed
    }
    public boolean isArmLimitMagnetDown() {
        return armLimitMagnet.getState();
        // returns true when not pressed
    }
    public void pickupMode(boolean pickup) {
        pickupMode = pickup;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Arm rotation target", armBaseTicks);
        telemetry.addData("Arm extention target", armExtendTicks);
    }

}
