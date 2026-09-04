package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutonomousForBabyBot extends LinearOpMode {

    private final int TICKS_312 = 580;
    private final int TICKS_435 = 384;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor rightRearMotor;
        DcMotor leftRearMotor;
        DcMotor frontMotor;
        Servo Rev;
        // Init here

        rightRearMotor = hardwareMap.get(DcMotor.class, "rearRight");
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor = hardwareMap.get(DcMotor.class, "rearLeft");
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontMotor = hardwareMap.get(DcMotor.class, "spinthingy" );
        frontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rev = hardwareMap.get(Servo.class, "Rev0");


        waitForStart();

        rightRearMotor.setTargetPosition(TICKS_312*5);
        rightRearMotor.setPower(1);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setTargetPosition(TICKS_312*5);
        leftRearMotor.setPower(1);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(1000);

        frontMotor.setTargetPosition(384*10);
        frontMotor.setPower(1);
        frontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(1000);
    }
}
