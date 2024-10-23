package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.Intake.WristState;
import org.firstinspires.ftc.teamcode.DeepArm.ArmMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    private ArmMode pickupMode = ArmMode.Off;
    private boolean driveMode = true;
    private int finishedLoops = 0;
    private long lastSecond = 0;
    private int loopRate = 0;
    private WristState wristMode = WristState.Back;
    private boolean bumperMode = false;

    @Override
    public void init() {
        driveChassis = new Chassis();
        driveChassis.init(hardwareMap, telemetry);
        blinkinLED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLED");

        intake = new Intake();
        intake.init(hardwareMap);
        deepArm = new DeepArm();
        deepArm.init(hardwareMap, telemetry);
        blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    @Override
    public void loop() {

        driveChassis.updateOdo();

        if(lastSecond == 0 || lastSecond - System.currentTimeMillis() >= 1000) {
            lastSecond = System.currentTimeMillis();
            loopRate = finishedLoops;
            finishedLoops = 0;
        }

        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;
        if(gamepad1.a) {
            driveMode = true;
        } else if(gamepad1.b) {
            driveMode = false;
        }

        if(!driveMode) {
            driveChassis.mecanumDriveFieldCentric(forward, strafe, rotate);
        } else {
            driveChassis.mecanumDrive(forward, strafe, rotate);
        }

        if (gamepad1.right_bumper) {
            driveChassis.orientToScore();
        }

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
        if (gamepad2.dpad_down) {
            pickupMode = ArmMode.Pickup;
        } else if (gamepad2.dpad_up) {
            pickupMode = ArmMode.Score;
        } else if (gamepad2.dpad_right) {
            pickupMode = ArmMode.Lifted;
        } else if (gamepad2.dpad_left) {
            pickupMode = ArmMode.Off;
        }

        if (gamepad2.right_bumper) {
            if (!bumperMode) {
                if (wristMode == WristState.Back) {
                    wristMode = WristState.Score;
                } else if (wristMode == WristState.Score) {
                    wristMode = WristState.Pickup;
                } else if (wristMode == WristState.Pickup) {
                    wristMode = WristState.Back;
                }
            }
            bumperMode = true;
        }
        if (!gamepad2.right_bumper) {
            bumperMode = false;
        }

        intake.wristControl(wristMode);

        deepArm.setArmState(-gamepad2.left_stick_y, -gamepad2.right_stick_y, pickupMode);

        finishedLoops += 1;
        telemetry.addData("Loop rate", loopRate);
        telemetry.addData("Wrist state", wristMode);
        telemetry.addData("Drive mode", driveMode);
        intake.addTelemetry(telemetry);
        deepArm.addTelemetry(telemetry);
        telemetry.update();

    }
}
