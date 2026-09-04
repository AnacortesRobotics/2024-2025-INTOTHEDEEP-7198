package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class NeerajIsABetterCoderThanMoses extends OpMode {

    private DcMotor left;
    private DcMotor right;
    private DcMotorEx rotate;
    private Servo grabber;

    //Enum for different states of the arm
    public enum ArmState {
        LOWERED,
//        MOVING_UP,
        LIFTED,
//        MOVING_DOWN,
    };
    ArmState armState = ArmState.LOWERED;
    private ElapsedTime timer = new ElapsedTime();
    private int currentPos = 0;
    private int counter = 0;
    private int rotateDownPosition = 0;
    private int rotateUpPosition = 130;
    private int deadzoneEnc = 15; // deadzone for rotation Encoders
    boolean rotatePower = false;
    boolean isBetween(double testValue, double minValue, double maxValue) {
        return testValue >= minValue && testValue <= maxValue;
    }

    @Override
    public void init() {
        grabber = hardwareMap.get(Servo.class, "grabber");
        rotate = hardwareMap.get(DcMotorEx.class, "rotate");
        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        rotate.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * This method will be called once, when the START button is pressed.
     */
    @Override
    public void start() {
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    @Override
    public void loop() {

        left.setPower(0.5*(gamepad1.left_stick_y / 2));
        right.setPower(0.5*(gamepad1.right_stick_y / 2));


        switch (armState) {
            case LOWERED:
                currentPos = rotate.getCurrentPosition();
                telemetry.addData("armState", armState);
                telemetry.addData("arm current pos", currentPos);
//                telemetry.addData("arm target pos", rotate.getTargetPosition());

                if (gamepad1.y) {
//                    rotate.setTargetPosition(rotateUpPosition);
//                    rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rotate.setPower(0.25);
//                    armState = ArmState.MOVING_UP;
//                    timer.reset();
                    currentPos = rotate.getCurrentPosition();

//                    while ((timer.milliseconds() < 100) ||
                  while (!isBetween(currentPos,
                                    rotateUpPosition-deadzoneEnc,
                                    rotateUpPosition+deadzoneEnc)) {
                    counter++;
                    currentPos = rotate.getCurrentPosition();
                    }

                    rotate.setPower(0);
                    armState = ArmState.LIFTED;
                }
                break;

//            case MOVING_UP:
//                telemetry.addData("armState", armState);
//                telemetry.addData("arm current pos", currentPos);
//                telemetry.addData("arm target pos", rotate.getTargetPosition());
//                telemetry.addData("min", rotateUpPosition-deadzoneEnc);
//                telemetry.addData("max", rotateUpPosition+deadzoneEnc);
//                telemetry.addData("counter", counter);
//                counter ++;
//
//                if (isBetween(currentPos, rotateUpPosition-deadzoneEnc, rotateUpPosition+deadzoneEnc)) {
//                    rotate.setPower(0);
//                    telemetry.addData("set zero power ",rotate.getPowerFloat());
//                    armState = ArmState.LIFTED;
//                }
//                break;
//
            case LIFTED:
                currentPos = rotate.getCurrentPosition();
                telemetry.addData("armState", armState);
                telemetry.addData("arm current pos", currentPos);
                telemetry.addData("arm target pos", rotate.getTargetPosition());
                telemetry.addData("counter", counter);
                counter = 0;
                if (gamepad1.a) {
                    rotate.setTargetPosition(rotateDownPosition);
                    rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rotate.setPower(0.2);
//                    armState = ArmState.MOVING_DOWN;
                    timer.reset();
                    do {
                        counter++;
                        currentPos = rotate.getCurrentPosition();
                        if (isBetween(currentPos, rotateDownPosition-deadzoneEnc, rotateDownPosition+deadzoneEnc)) {
                            break;
                        }
                    }
                    while (timer.milliseconds() < 100);

                    rotate.setPower(0);
                    rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    armState = ArmState.LOWERED;
                    telemetry.addData("arm current pos after reset", rotate.getCurrentPosition());
                }
                break;

//            case MOVING_DOWN:
//                telemetry.addData("armState", armState);
//                telemetry.addData("arm current pos", currentPos);
//                telemetry.addData("arm target pos", rotate.getTargetPosition());
//                telemetry.addData("min", rotateDownPosition-deadzoneEnc);
//                telemetry.addData("max", rotateDownPosition+deadzoneEnc);
//
//                if (isBetween(currentPos, rotateDownPosition-deadzoneEnc, rotateDownPosition+deadzoneEnc)) {
//                    rotate.setPower(0);
//                    rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    armState = ArmState.LOWERED;
//                    telemetry.addData("arm current pos after reset", rotate.getCurrentPosition());
//                }
//                break;
            default:
        }


        if (gamepad1.right_trigger>=0.2) {
            grabber.setPosition(1);
        } else if (gamepad1.left_trigger>=0.2) {
            grabber.setPosition(0);
        }

        telemetry.update();
    }
}
