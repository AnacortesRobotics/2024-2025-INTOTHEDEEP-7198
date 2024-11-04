package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestRunMia extends OpMode {

    DcMotor motor1;
    DcMotor motorLeft;
    DcMotor motorRight;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        motor1.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
        motorLeft.setPower(-gamepad1.left_stick_y);
        motorRight.setPower(-gamepad1.right_stick_y);
    }
}
