package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous
@Config
public class SecondaryAuto extends OpMode {

    public Chassis driveChassis;
    public ArmManager armManager;

    private long lastCallTime = 0;
    private boolean didTimeout = false;
    private AutoStates autoStates = AutoStates.Idle;
    private int currentCycle = 1;

    public enum AutoStates {
        Idle,
        GoToNet,
        Score,
        PrepareToPick,
        GoToPickup1,
        GoToPickup2,
        Pickup,
        Stop

    }

    public static double XP = 0.25;
    public static double XI = 0;
    public static double XD = 0.8;
    public static double YP = 0.25;
    public static double YI = 0;
    public static double YD = 0.8;
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
    }

    @Override
    public void loop() {
        didTimeout = System.currentTimeMillis() - lastCallTime > 3500;
        if ((driveChassis.atTarget() && armManager.isAtTarget()) || didTimeout) {
            lastCallTime = System.currentTimeMillis();
            switch(autoStates) {
                case Idle:
                    autoStates = AutoStates.GoToNet;
                    armManager.setGrabberPosition(Intake.IntakeState.Closed);
                    armManager.setWristTarget(Intake.WristMode.SubPick, 0);
                    break;
                case GoToNet:
                    driveChassis.setMaxSpeed(1);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 21, 21, AngleUnit.DEGREES, -45), true);
                    armManager.setArmTarget(DeepArm.ArmMode.Score, 0);
                    armManager.setWristTarget(Intake.WristMode.Score, 2000);
                    autoStates = AutoStates.Score;
                    break;
                case Score:
                    armManager.setGrabberPosition(Intake.IntakeState.Open);
                    armManager.setWristTarget(Intake.WristMode.SubPick, 200);
                    autoStates = AutoStates.PrepareToPick;
                    break;
                case PrepareToPick:
                    armManager.setArmTarget(DeepArm.ArmMode.Pickup, 0);
                    if (currentCycle == 1) {
                        autoStates = AutoStates.GoToPickup1;
                    } else if (currentCycle == 2) {
                        autoStates = AutoStates.GoToPickup2;
                    } else {
                        autoStates = AutoStates.Stop;
                    }
                    break;
                case GoToPickup1:
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 23, 40, AngleUnit.DEGREES, 0), true);
                    currentCycle = 2;
                    autoStates = AutoStates.Pickup;
                    break;
                case GoToPickup2:
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 20, 40, AngleUnit.DEGREES, 0), true);
                    currentCycle = 3;
                    autoStates = AutoStates.Pickup;
                    break;
                case Pickup:
                    armManager.setGrabberPosition(Intake.IntakeState.Closed);
                    autoStates = AutoStates.GoToNet;
                    break;
                case Stop:
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
