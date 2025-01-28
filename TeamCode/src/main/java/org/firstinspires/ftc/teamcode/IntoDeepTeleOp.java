package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.Intake.WristMode;
import org.firstinspires.ftc.teamcode.DeepArm.ArmMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class IntoDeepTeleOp extends OpMode {
    public Chassis driveChassis;
    private double forward;
    private double strafe;
    private double rotate;

    private ArmManager armManager;
//    SampleProcessor sampleProcessor;
//    VisionPortal visionPortal;

    private boolean pickup = false;
    private ArmMode pickupMode = ArmMode.Off;
    private boolean driveMode = true;
    private int finishedLoops = 0;
    private long lastSecond = 0;
    private int loopRate = 0;
    private WristMode wristMode = WristMode.Back;
    private IntakeState intakeState = IntakeState.Back;
    private boolean bumperMode = false;
    private boolean triggerMode = false;
    private double turnPower = 0;
    private Intake.BlockColor ledColor = Intake.BlockColor.Unknown;
    private boolean isDpadDown = false;

    private Chassis.MotorTesting motorTesting = Chassis.MotorTesting.lf;


    @Override
    public void init() {
        driveChassis = new Chassis();
        driveChassis.init(hardwareMap, telemetry, true);
        armManager = new ArmManager();
        armManager.init(hardwareMap, telemetry);
//        sampleProcessor = new SampleProcessor();
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "sample Camera"))
//                .setCameraResolution(new Size(1280, 720))
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .enableLiveView(true)
//                .addProcessor(sampleProcessor)
//                .build();
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

        turnPower = rotate;

        if (driveChassis.currentState == Chassis.ChassisState.Stop) {
            driveChassis.mecanumDriveFieldCentric(forward, strafe, turnPower);
        }

//        if (gamepad2.right_trigger < 0.1) {
//            intake.servoControl(IntakeState.In);
//        if (gamepad2.b) {
//            armManager.setGrabberPosition(IntakeState.Open);
//            //blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
//        } else if (gamepad2.a) {
//            armManager.setGrabberPosition(IntakeState.Closed);
//        } else if (gamepad2.y) {
//            armManager.setGrabberPosition(IntakeState.Back);
//        }

//        if (gamepad1.a) {
//            driveChassis.prepareClimb();
//        } else if (gamepad1.b && driveChassis.isPrepared()) {
//            driveChassis.beginClimb();
//        } else if (gamepad1.x) {
//            driveChassis.cancelClimb();
//        }

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

        if (armManager.isPickupDone()) {
            if (gamepad2.dpad_up) {
                armManager.setArmTarget(ArmMode.Score, 0);
                armManager.setWristTarget(WristMode.Pickup, 0);
            } else if (gamepad2.dpad_right) {
                armManager.setArmTarget(ArmMode.Lifted, 0);
            } else if (gamepad2.dpad_down) {
                armManager.lineUpToGrab();
            }
        }

        if (gamepad2.b) {
            armManager.setWristTarget(WristMode.Score, 0);
        }
        if (gamepad2.x) {
            armManager.scoreAndReturn();
        }
        if (gamepad2.a) {
            armManager.startPickFromLineup();
        }

        if (gamepad1.right_bumper) {
            driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 21, 21, AngleUnit.DEGREES, -45), true);
        }
        if (gamepad1.left_bumper) {
            driveChassis.abortMove();
        }

        if (gamepad1.back || gamepad2.back) {
            driveChassis.lock();
            armManager.lock();
        }
        if (gamepad1.start || gamepad2.start) {
            driveChassis.unlock();
            armManager.unlock();
        }

//        if (gamepad2.left_bumper) {
//            if (!bumperMode) {
//                if (wristMode == WristMode.Pickup) {
//                    if (armManager.getRotationTicks() > 1800) {
//                        armManager.setWristTarget(WristMode.Back, 0);
//                        wristMode = WristMode.Back;
//                    }
//                } else if (wristMode == WristMode.Back) {
//                    if (armManager.getRotationTicks() > 1800) {
//                        armManager.setWristTarget(WristMode.Score, 0);
//                        wristMode = WristMode.Score;
//                    }
//                } else if (wristMode == WristMode.Score) {
//                    if (armManager.getRotationTicks() > 1800) {
//                        armManager.setWristTarget(WristMode.Forward, 0);
//                        wristMode = WristMode.Forward;
//                    }
//                } else if (wristMode == WristMode.Forward) {
//                    armManager.setWristTarget(WristMode.Pickup, 0);
//                    wristMode = WristMode.Pickup;
//                }
//                bumperMode = true;
//            }
//        } else {
//            bumperMode = false;
//        }
        if (gamepad2.left_trigger > .3) {
            if (!triggerMode) {
                if (intakeState == IntakeState.Closed) {
                    armManager.setGrabberPosition(IntakeState.Open);
                    intakeState = IntakeState.Open;
                } else {
                    armManager.setGrabberPosition(IntakeState.Closed);
                    intakeState = IntakeState.Closed;
                }
                triggerMode = true;
            }
        } else {
            triggerMode = false;
        }

//        if (gamepad2.right_bumper) {
//            visionPortal.resumeStreaming();
//            if (sampleProcessor.isOnTarget()) {
//                armManager.startPickup();
//            } else {
//                driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, sampleProcessor.getTravelDistance(true), 0, AngleUnit.DEGREES, 0), false);
//                armManager.extendArmToPosition(sampleProcessor.getTravelDistance(false) + armManager.getExtentionInches());
//            }
//
//        }
//        if (!gamepad2.right_bumper) {
//            visionPortal.stopStreaming();
//        }

        armManager.manualArmMove(-gamepad2.left_stick_y, -gamepad2.right_stick_y);

        telemetry.addData("Left stick y", gamepad2.left_stick_y);
        telemetry.addData("Right stick y", gamepad2.right_stick_y);

        driveChassis.scaleMaxSpeed(1 - gamepad1.right_trigger * 0.7);

        if (armManager.getRotationTicks() < 1800) {
            armManager.setWristTarget(WristMode.Pickup, 0);
        }

        armManager.update();
        finishedLoops += 1;
        telemetry.addData("Loop rate", loopRate);
        telemetry.addData("Wrist state", wristMode);
        telemetry.addData("Drive mode", driveMode);
        armManager.updateTelemetry();
        telemetry.update();

    }
}

