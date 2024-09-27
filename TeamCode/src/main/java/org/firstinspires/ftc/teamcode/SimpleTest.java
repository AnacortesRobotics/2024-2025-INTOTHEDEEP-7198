package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Simple Test")
public class SimpleTest extends OpMode {

    private DcMotor rotaryJoint;

    @Override
    public void init() {
        rotaryJoint = hardwareMap.get(DcMotor.class, "1");
    }

    @Override
    public void loop() {
        rotaryJoint.setPower(-gamepad1.left_stick_y);
    }
}
