package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.Intake.WristMode;
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

    private ArmManager armManager;

    private boolean pickup = false;
    private ArmMode pickupMode = ArmMode.Off;
    private boolean driveMode = true;
    private int finishedLoops = 0;
    private long lastSecond = 0;
    private int loopRate = 0;
    private WristMode wristMode = WristMode.Back;
    private boolean bumperMode = false;
    private double turnPower = 0;
    private Intake.BlockColor ledColor = Intake.BlockColor.Unknown;
    private boolean isDpadDown = false;

    private Chassis.MotorTesting motorTesting = Chassis.MotorTesting.lf;


    @Override
    public void init() {
        driveChassis = new Chassis();
        driveChassis.init(hardwareMap, telemetry, true);
        blinkinLED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLED");
        armManager = new ArmManager();
        armManager.init(hardwareMap, telemetry);

        blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
    }

    @Override
    public void loop() {

        driveChassis.updateOdo();

        if (lastSecond == 0 || lastSecond - System.currentTimeMillis() >= 1000) {
            lastSecond = System.currentTimeMillis();
            loopRate = finishedLoops;
            finishedLoops = 0;
        }

        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        if (gamepad1.a) {
            driveMode = true;
        } else if (gamepad1.b) {
            driveMode = false;
        }
        if (gamepad1.left_bumper) {
            turnPower = driveChassis.orient(-45);
        } else {
            turnPower = rotate;
            driveChassis.resetOrient();
        }
        if (driveChassis.currentState == Chassis.ChassisState.Stop) {
            if (!driveMode) {
                driveChassis.mecanumDriveFieldCentric(forward, strafe, turnPower);
            } else {
                driveChassis.mecanumDrive(forward, strafe, turnPower);
            }
        }

//        if (gamepad2.right_trigger < 0.1) {
//            intake.servoControl(IntakeState.In);
        if (gamepad2.b) {
            armManager.setGrabberPosition(IntakeState.Open);
            //blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        } else if (gamepad2.a) {
            armManager.setGrabberPosition(IntakeState.Closed);
        } else if (gamepad2.y) {
            armManager.setGrabberPosition(IntakeState.Back);
        }

//        if (intake.getIntakeColor() != Intake.BlockColor.Unknown) {
//            ledColor = intake.getIntakeColor();
//        }

//        if (intake.isLimitDown()) {
//            switch (ledColor) {
//                case Red:
//                    blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//                    break;
//                case Blue:
//                    blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
//                    break;
//                case Yellow:
//                    blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
//                    break;
//            }
//        }

/*
        if (!deepArm.isArmLimitMagnetDown()) {
            blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        } else {
            blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
*/

        if (gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up || gamepad2.dpad_down) {
            if (!isDpadDown) {
                if (gamepad2.dpad_down) {
                    pickupMode = ArmMode.Pickup;
                    armManager.setArmTarget(pickupMode, 0);
                } else if (gamepad2.dpad_up) {
                    pickupMode = ArmMode.Score;
                    armManager.setArmTarget(pickupMode, 0);
                } else if (gamepad2.dpad_right) {
                    pickupMode = ArmMode.Lifted;
                    armManager.setArmTarget(pickupMode, 0);
                } else if (gamepad2.dpad_left) {
                    pickupMode = ArmMode.Off;
                }
                isDpadDown = true;
            }
        } else {
            isDpadDown = false;
        }

        if (gamepad2.right_bumper) {
            if (!bumperMode) {
                if (wristMode == WristMode.Back) {
                    wristMode = WristMode.Score;
                } else if (wristMode == WristMode.Score) {
                    wristMode = WristMode.Pickup;
                } else if (wristMode == WristMode.Pickup) {
                    wristMode = WristMode.SubPick;
                } else if (wristMode == WristMode.SubPick) {
                    wristMode = WristMode.Back;
                }
            }
            bumperMode = true;
        }
        if (gamepad2.left_bumper) {
            if (!bumperMode) {
                if (wristMode == WristMode.Back) {
                    wristMode = WristMode.SubPick;
                } else if (wristMode == WristMode.SubPick) {
                    wristMode = WristMode.Pickup;
                } else if (wristMode == WristMode.Pickup) {
                    wristMode = WristMode.Score;
                } else if (wristMode == WristMode.Score) {
                    wristMode = WristMode.Back;
                }
            }
            bumperMode = true;
        }
        if (!gamepad2.right_bumper) {
            bumperMode = false;
        }

        if (gamepad1.x) {
            driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 21, 21, AngleUnit.DEGREES, -45), true);
        }
        if (gamepad1.y) {
            driveChassis.abortMove();
        }

        armManager.setWristTarget(wristMode ,0);


        armManager.manualArmMove(-gamepad2.left_stick_y, -gamepad2.right_stick_y);

        telemetry.addData("Left stick y", gamepad2.left_stick_y);
        telemetry.addData("Right stick y", gamepad2.right_stick_y);

        driveChassis.scaleMaxSpeed(1 - gamepad1.right_trigger * 0.7);

        armManager.update();
        finishedLoops += 1;
        telemetry.addData("Loop rate", loopRate);
        telemetry.addData("Wrist state", wristMode);
        telemetry.addData("Drive mode", driveMode);
        armManager.updateTelemetry();
        telemetry.update();

    }
}

