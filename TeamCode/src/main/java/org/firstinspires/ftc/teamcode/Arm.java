package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Arm {
    
    private DcMotorEx armBase;
    private DcMotor armExtend;
    private Servo armWrist;
    private Servo armHand;
    
    private int armBaseTicks = 0;
    private int armExtendTicks = 0;
    private static final int ARM_BASE_TICK_SPEED = 3;
    private static final int ARM_EXTEND_TICK_SPEED = 14;
    private boolean pickupMode = false;

    private double mapRange(double value, double inputStart, double inputEnd, double outputStart, double outputEnd) {
        return outputStart + (outputEnd - outputStart) * ((value - inputStart) / (inputEnd - inputStart));
    }

    
    public void init(HardwareMap hMap) {
        
        armWrist = hMap.get(Servo.class, "armWrist");
        armBase = hMap.get(DcMotorEx.class, "armBase");
        armHand = hMap.get(Servo.class, "armHand");
        armExtend = hMap.get(DcMotor.class, "armExtend");
        
        
        armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setTargetPosition(0);
        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armBase.setTargetPosition(0);
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void liftWrist() {
        // Higher number means hand is closer to ground
        armWrist.setPosition(0.3);
        // Higher number means less open hand
        armHand.setPosition(0.3);
        
    }
    
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("hand position", armHand.getPosition());
        telemetry.addData("Arm position actual", armBase.getCurrentPosition());
        telemetry.addData("Arm position target", armBaseTicks);
        telemetry.addData("Arm extender position", armExtend.getCurrentPosition());
        telemetry.addData("Arm extender target", armExtendTicks);
    }
    
    public void wristControl(boolean height) {


        if (height) {
            armWrist.setPosition(0.3);
            
        } else if (!height) {
            armWrist.setPosition(0.6);
            
        } else {
            armWrist.setPosition(0.5);
            
        }
        
    }
    
    
    
    public void handControl(boolean handOpen) {
        
        if (handOpen) {
            armHand.setPosition(0.4);
        } else if (!handOpen) {
            armHand.setPosition(0.21);
        }
        
    }
    
    public void armHeight(double armSpeed) {
        if (!pickupMode) {
            int tickOffset = (int) (armSpeed * ARM_BASE_TICK_SPEED);
            armBaseTicks += tickOffset;
            armBaseTicks = Math.max(armBaseTicks, 30);
            armBaseTicks = Math.min(armBaseTicks, 800);
            armBase.setTargetPosition(armBaseTicks);
            armBase.setPower(0.8);
        } else {
            armBase.setTargetPosition((int)mapRange(armExtendTicks, 20, 3000, 20, 100));
            armBase.setPower(0.8);
        }
    }
    
    public void armExtender(double armLength) {
        
        int tickOffsetExtend = (int) (armLength * ARM_EXTEND_TICK_SPEED);
        armExtendTicks += tickOffsetExtend;
        armExtendTicks = Math.max(armExtendTicks, 20);
        armExtendTicks = Math.min(armExtendTicks, 3000);
        armExtend.setTargetPosition(armExtendTicks);
        armExtend.setPower(0.5);
        
    }
    
    public void pickupMode(boolean pickup) {
        pickupMode = pickup;
        
    }

    
}
