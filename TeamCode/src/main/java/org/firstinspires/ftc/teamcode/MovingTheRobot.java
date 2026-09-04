package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import static java.lang.Thread.sleep;

@TeleOp
public class MovingTheRobot extends OpMode {
    long StartWhenInit;
    DcMotor rightRearMotor;
    DcMotor leftRearMotor;
    DcMotor frontMotor;
    //Servo Rev;
//    HardwareDevice DS;
//    ColorSensor CS;

    public void init() {
        rightRearMotor = hardwareMap.get(DcMotor.class, "rearRight");
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor = hardwareMap.get(DcMotor.class, "rearLeft");
        //leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontMotor = hardwareMap.get(DcMotor.class, "spinthingy");
        //Rev = hardwareMap.get(Servo.class, "Rev0");
        //DS = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
        //CS = hardwareMap.get(ColorSensor.class, "ColorSensor");


    }


    public void loop() {
        rightRearMotor.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
        leftRearMotor.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);
        //frontMotor.setPower();
        frontMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
//        if (gamepad1.dpad_left) {
//            Rev.setPosition(0);
//            ;
//        } else {
//            Rev.setPosition(1);
//        }
//        telemetry.addData("Neeraj's Telemetry Green", CS.green());
//        telemetry.addData("Neeraj's Telemetry Red", CS.red());
//        telemetry.addData("Neeraj's Telemetry Blue", CS.blue());

//        if (CS.red() >= 500) {
//            frontMotor.setPower(1);
//            StartWhenInit = System.currentTimeMillis();
//
//
//        }
//        if (System.currentTimeMillis() - StartWhenInit >= 1000) {
//            frontMotor.setPower(0);
//
//        }
    }
}
