package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.DeepArm.ArmMode;

@Autonomous

public class IntoDeepAuto extends OpMode {

    public Chassis driveChassis;
    private ArmManager armManager;

    private AutoStates autoStates = AutoStates.Idle;
    private int cyclesComplete = 1;
    private long lastCallTime = 0;
    private boolean didTimeout = false;
    private boolean isRotated = false;

    public enum AutoStates {
        TestTurnRight,
        TestTurnLeft,
        Idle,
        StartMoveNetZone,
        LiftToScoreOne,
        WristInPosition,
        BlockReleased,
        WristReadyToDrop,
        Cycle3Move,
        ReadyToPickup,
        MoveToPickup,
        CycleTwo,
        CycleThree,
        Reset,
        Stop,
        EReset
    }

    @Override
    public void init() {
        driveChassis = new Chassis();
        driveChassis.init(hardwareMap, telemetry, false);
        armManager = new ArmManager();
        armManager.init(hardwareMap, telemetry);
        armManager = new ArmManager();
        armManager.init(hardwareMap, telemetry);
        driveChassis.setPosition(new Pose2D(DistanceUnit.INCH, 47.2 - Chassis.ROBOT_WIDTH / 2, Chassis.ROBOT_LENGTH / 2, AngleUnit.DEGREES, 0));
    }
    // 41 13/16
    // 7 5/8

    //Length: 17 1/2
    //Width: 17 3/4
    @Override
    public void loop() {
        telemetry.addData("Is this working", true);
        didTimeout = System.currentTimeMillis() - lastCallTime > 3500;
        if ((driveChassis.atTarget() && armManager.isAtTarget()) || didTimeout || autoStates == AutoStates.EReset) {
            lastCallTime = System.currentTimeMillis();
            switch (autoStates) {
                case TestTurnLeft:
                    driveChassis.setMaxSpeed(.3);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -90), false);
                    autoStates = AutoStates.TestTurnRight;
                    break;
                case TestTurnRight:
                    driveChassis.setMaxSpeed(.3);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90), false);
                    autoStates = AutoStates.TestTurnLeft;
                    break;
                case Idle:
                    armManager.setArmTarget(ArmMode.Lifted, 0);
                    driveChassis.setMaxSpeed(.8);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, -16, 13, AngleUnit.DEGREES, 0), false);
                    autoStates = AutoStates.StartMoveNetZone;
                    armManager.setWristTarget(Intake.WristMode.SubPick, 500);
                    break;
                case StartMoveNetZone:
                    driveChassis.setAllowedError(2);
                    driveChassis.setMaxSpeed(.4);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -44), false);
                    armManager.setArmTarget(ArmMode.Score, 0);
                    autoStates = AutoStates.LiftToScoreOne;
                    isRotated = true;
                    break;
                case LiftToScoreOne:
                    armManager.setWristTarget(Intake.WristMode.Pickup, 200);
                    autoStates = AutoStates.WristInPosition;
                    break;
                case WristInPosition:
                    armManager.setGrabberPosition(Intake.IntakeState.Open);
                    autoStates = AutoStates.BlockReleased;
                    break;
                case BlockReleased:
                    armManager.setWristTarget(Intake.WristMode.SubPick, 500);
                    if (cyclesComplete == 3) {
                        autoStates = AutoStates.EReset;
                    } else {
                        autoStates = AutoStates.WristReadyToDrop;
                    }
                    break;
                case WristReadyToDrop:
                    armManager.setWristTarget(Intake.WristMode.Pickup, 300);
                    armManager.setArmTarget(ArmMode.Pickup, 0);
                    driveChassis.setMaxSpeed(.4);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 45), false);
                    if (cyclesComplete == 2) {
                        autoStates = AutoStates.Cycle3Move;
                    } else {
                        autoStates = AutoStates.ReadyToPickup;
                    }
                    isRotated = false;
                    break;
                case Cycle3Move:
                    driveChassis.setAllowedError(.5);
                    driveChassis.setMaxSpeed(.7);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, -11, 0, AngleUnit.DEGREES, 0), false);
                    autoStates = AutoStates.ReadyToPickup;
                    break;
                case ReadyToPickup:
                    driveChassis.setAllowedError(.5);
                    driveChassis.setMaxSpeed(.7);
                    armManager.setGrabberPosition(Intake.IntakeState.Closed);
                    if (cyclesComplete == 1) {
                        driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 3, 14, AngleUnit.DEGREES, 0), false);
                    } else {
                        driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 11, AngleUnit.DEGREES, 0), false);
                    }
                    autoStates = AutoStates.MoveToPickup;
                    break;
                case MoveToPickup:
                    if (didTimeout) {
                        autoStates = AutoStates.EReset;
                        break;
                    }
                    if (cyclesComplete == 1) {
                        driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, -3, -11, AngleUnit.DEGREES, 0), false);
                    } else {
                        driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 11.5, -13.5, AngleUnit.DEGREES, 0), false);
                    }
                    armManager.setWristTarget(Intake.WristMode.SubPick, 300);
                    armManager.setArmTarget(ArmMode.Lifted, 0);
                    if (cyclesComplete == 1 && !didTimeout) {
                        autoStates = AutoStates.CycleTwo;
                    } else if (cyclesComplete == 2 && !didTimeout) {
                        autoStates = AutoStates.CycleThree;
                    }
                    break;
                case CycleTwo:
                    autoStates = AutoStates.StartMoveNetZone;
                    cyclesComplete = 2;
                    break;
                case CycleThree:
                    autoStates = AutoStates.StartMoveNetZone;
                    cyclesComplete = 3;
                    break;
                case Reset:
                    driveChassis.setMaxSpeed(.5);
                    armManager.setArmTarget(ArmMode.Lifted, 0);
                    armManager.setWristTarget(Intake.WristMode.SubPick, 0);
                    armManager.setWristTarget(Intake.WristMode.Back, 1500);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 45), false);
                    autoStates = AutoStates.Stop;
                    break;
                case EReset:
                    driveChassis.setMaxSpeed(.5);
                    armManager.setArmTarget(ArmMode.Lifted, 0);
                    if (isRotated) {
                        driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 45), false);
                    }
                    armManager.setWristTarget(Intake.WristMode.Back, 0);
                    autoStates = AutoStates.Stop;
                    break;
                case Stop:
                    terminateOpModeNow();
            }
        }
        driveChassis.update();
        armManager.update();

        armManager.updateTelemetry();
        telemetry.addData("current State", autoStates);
        telemetry.addData("Chassis at target", driveChassis.atTarget());
        telemetry.update();
//        intake.wristControl(Intake.WristState.SubPick, telemetry);
//        driveChassis.driveToPosition(new Pose2D(DistanceUnit.INCH, -19, 11, AngleUnit.DEGREES, 0), null, "move1", .4, true);
//        driveChassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -43), null, "turn1", .2, true);
//        arm.setArmState(0, 0, ArmMode.Score);
//        intake.wristControl(Intake.WristState.Score, telemetry);
//        intake.servoControl(IntakeState.Out);
//        intake.servoControl(IntakeState.Stop);
//        intake.wristControl(Intake.WristState.SubPick, telemetry);
//        arm.setArmState(0, 0, ArmMode.Lifted);
//        intake.wristControl(Intake.WristState.Back, telemetry);
    }

}
