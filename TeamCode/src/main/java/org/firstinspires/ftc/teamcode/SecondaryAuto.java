package org.firstinspires.ftc.teamcode;


import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

@Autonomous
@Config
public class SecondaryAuto extends OpMode {

    public Chassis driveChassis;
    public ArmManager armManager;
//    public SampleProcessor sampleProcessor;
//    public VisionPortal visionPortal;

    private long lastCallTime = 0;
    private boolean didTimeout = false;
    private AutoStates autoStates = AutoStates.Idle;
    private int currentCycle = 1;

    public enum AutoStates {
        Test,
        Idle,
        GoToNet,
        Score,
        PrepareToPick,
        GoToPickup1,
        GoToPickup2,
        BeginPickup,
        MovePickup,
        Pickup,
        Stop

    }

    public static double XP = 0.2;
    public static double XI = 0;
    public static double XD = 0.1;
    public static double YP = 0.2;
    public static double YI = 0;
    public static double YD = 0.1;
    public static double RP = 0.05;
    public static double RI = 0;
    public static double RD = 0;
    //for low battery, use .75 D


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armManager = new ArmManager();
        armManager.init(hardwareMap, telemetry);
        driveChassis = new Chassis();
        driveChassis.init(hardwareMap, telemetry, false);
        armManager = new ArmManager();
        armManager.init(hardwareMap, telemetry);
        driveChassis.setPosition(new Pose2D(DistanceUnit.INCH, 40 + Chassis.ROBOT_WIDTH / 2, Chassis.ROBOT_LENGTH / 2 - 2.5, AngleUnit.DEGREES, 0));
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
        //TODO: IF THERE IS AN ISSUE WITH THE PINPOINT DRIVER, RESET IT -- RUN TESTER OPMODE
    @Override
    public void loop() {
        didTimeout = System.currentTimeMillis() - lastCallTime > 5000;
        if (((driveChassis.atTarget() && armManager.isAtTarget()) && armManager.isArmDone()) || didTimeout) {
            lastCallTime = System.currentTimeMillis();
            switch(autoStates) {
                case Test:
                    terminateOpModeNow();
                    break;
                case Idle:
                    autoStates = AutoStates.GoToNet;
                    armManager.setGrabberPosition(Intake.IntakeState.Closed);
                    armManager.setWristTarget(Intake.WristMode.Pickup, 0);
                    while (!armManager.isRotateLimitDown()) {
                        armManager.manualArmMove(-1, 0);
                    }
                    break;
                case GoToNet:
                    driveChassis.setMaxSpeed(0.8);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 20, 20, AngleUnit.DEGREES, -45), true);
                    armManager.setArmTarget(DeepArm.ArmMode.Score, 0);
                    armManager.setWristTarget(Intake.WristMode.Score, 5500);
                    autoStates = AutoStates.Score;
                    break;
                case Score:
                    armManager.scoreAndReturn();
                    autoStates = AutoStates.Stop;
                    break;
                case PrepareToPick:
                    if (currentCycle == 1) {
                        autoStates = AutoStates.GoToPickup1;
                    } else if (currentCycle == 2) {
                        autoStates = AutoStates.GoToPickup2;
                    } else {
                        autoStates = AutoStates.Stop;
                    }
                    break;
                case GoToPickup1:
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 38, 27, AngleUnit.DEGREES, 0), true);
                    armManager.setWristTarget(Intake.WristMode.Back, 500);
                    currentCycle = 2;
                    autoStates = AutoStates.Pickup;
                    break;
                case GoToPickup2:
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 25, 28, AngleUnit.DEGREES, 0), true);
                    armManager.setWristTarget(Intake.WristMode.Back, 500);
                    currentCycle = 3;
                    autoStates = AutoStates.Pickup;
                    break;
                case BeginPickup:
                    armManager.lineUpToGrab();
                    autoStates = AutoStates.MovePickup;
                    break;
                case MovePickup:
                    driveChassis.setMaxSpeed(.4);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, -3, 0, AngleUnit.DEGREES, 0), false);
                    autoStates = AutoStates.Pickup;
                    break;
                case Pickup:
//                    visionPortal.resumeStreaming();
//                    if (sampleProcessor.isOnTarget()) {
                    armManager.startPickFromLineup();
                    autoStates = AutoStates.GoToNet;
//                    } else {
//                        driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, sampleProcessor.getTravelDistance(true), 0, AngleUnit.DEGREES, 0), false);
//                        armManager.extendArmToPosition(sampleProcessor.getTravelDistance(false) + armManager.getExtentionInches());
//                    }
//                    visionPortal.stopStreaming();
                    break;
                case Stop:
                    armManager.setArmTarget(DeepArm.ArmMode.Lifted,0);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES ,45), false);
                    autoStates = AutoStates.Test;
                    break;
            }

        }
        driveChassis.update();
        armManager.update();

        armManager.updateTelemetry();
        driveChassis.updateOdo();
        telemetry.addData("current State", autoStates);
        telemetry.addData("Chassis at target", driveChassis.atTarget());
        telemetry.update();
    }

}
