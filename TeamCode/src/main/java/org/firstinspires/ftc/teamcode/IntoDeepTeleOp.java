package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.Intake.IntakeState;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import  org.firstinspires.ftc.teamcode.Intake.BlockColor;

@TeleOp
public class IntoDeepTeleOp extends OpMode {
    public Chassis driveChassis;
    private double forward;
    private double strafe;
    private double rotate;
    private RevBlinkinLedDriver blinkinLED;

    private DeepArm deepArm;
    private Intake intake;

    private boolean pickup = false;
    private boolean pickupMode = true;
    private boolean prevousState = false;

    @Override
    public void init() {
        driveChassis = new Chassis();
        driveChassis.init(hardwareMap);
        blinkinLED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLED");

        intake = new Intake();
        intake.init(hardwareMap);
        deepArm = new DeepArm();
        deepArm.init(hardwareMap);
        blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        driveChassis.mecanumDrive(forward, strafe, rotate);

        if (gamepad2.x) {
            intake.servoControl(IntakeState.Stop);
        } else if (gamepad2.b) {
            intake.servoControl(IntakeState.Out);
            blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        } else if (gamepad2.a) {
            intake.servoControl(IntakeState.In);
        }
        intake.update();

        if (intake.isLimitDown()) {
            switch (intake.getIntakeColor()) {
                case Unknown:
                    blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                    break;
                case Red:
                    blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    break;
                case Blue:
                    blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    break;
                case Yellow:
                    blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                    break;
            }
        }



/*
        if (!deepArm.isArmLimitMagnetDown()) {
            blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        } else {
            blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
*/
        if (gamepad2.right_bumper) {
            // Untested
            if (!prevousState) {
                // if prevousState is false, then the button is not currently depressed
                if (pickupMode) {
                    // if pickupMode is true, that means it is not currently in pickup mode
                    pickup = true;
                    pickupMode = true;
                } else {
                    pickup = false;
                    pickupMode = false;
                }
                prevousState = true;
            }
        } else {
            prevousState = false;
        }

        deepArm.pickupMode(pickup);
        deepArm.rotateArm(gamepad2.left_stick_y);
        //deepArm.extendArm(gamepad2.right_stick_y);
        intake.addTelemetry(telemetry);
        deepArm.addTelemetry(telemetry);
        telemetry.update();

    }
}
