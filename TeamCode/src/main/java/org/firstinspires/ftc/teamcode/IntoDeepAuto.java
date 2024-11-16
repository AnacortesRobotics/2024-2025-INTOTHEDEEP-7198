package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.DeepArm.ArmMode;

import static java.lang.Thread.sleep;

@Autonomous

public class IntoDeepAuto extends OpMode {

    public Chassis driveChassis;
    public DeepArm arm;
    public Intake intake;

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
        driveChassis.init(hardwareMap, telemetry);
        arm = new DeepArm();
        arm.init(hardwareMap, telemetry, null);
        intake = new Intake();
        intake.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        telemetry.addData("Is this working", true);
        didTimeout = System.currentTimeMillis() - lastCallTime > 3500;
        if ((driveChassis.atTarget() && arm.isStopped() && intake.isIntakeDone() && intake.isWristDone()) || didTimeout || autoStates == AutoStates.EReset) {
            lastCallTime = System.currentTimeMillis();
            switch (autoStates) {
                case TestTurnLeft:
                    driveChassis.setMaxSpeed(.3);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -90));
                    autoStates = AutoStates.TestTurnRight;
                    break;
                case TestTurnRight:
                    driveChassis.setMaxSpeed(.3);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90));
                    autoStates = AutoStates.TestTurnLeft;
                    break;
                case Idle:
                    arm.setArmTarget(ArmMode.Lifted);
                    driveChassis.setMaxSpeed(.8);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, -16, 13, AngleUnit.DEGREES, 0));
                    autoStates = AutoStates.StartMoveNetZone;
                    intake.setWristTarget(Intake.WristMode.SubPick, 500);
                    break;
                case StartMoveNetZone:
                    driveChassis.setAllowedError(2);
                    driveChassis.setMaxSpeed(.4);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -44));
                    arm.setArmTarget(ArmMode.Score);
                    autoStates = AutoStates.LiftToScoreOne;
                    isRotated = true;
                    break;
                case LiftToScoreOne:
                    intake.setWristTarget(Intake.WristMode.Pickup, 200);
                    autoStates = AutoStates.WristInPosition;
                    break;
                case WristInPosition:
                    intake.servoControl(Intake.IntakeState.Out);
                    autoStates = AutoStates.BlockReleased;
                    break;
                case BlockReleased:
                    intake.setWristTarget(Intake.WristMode.SubPick, 500);
                    if (cyclesComplete == 3) {
                        autoStates = AutoStates.EReset;
                    } else {
                        autoStates = AutoStates.WristReadyToDrop;
                    }
                    break;
                case WristReadyToDrop:
                    intake.setWristTarget(Intake.WristMode.Pickup, 300);
                    arm.setArmTarget(ArmMode.Pickup);
                    driveChassis.setMaxSpeed(.4);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 45));
                    autoStates = AutoStates.ReadyToPickup;
                    isRotated = false;
                    break;
                case ReadyToPickup:
                    driveChassis.setAllowedError(.5);
                    driveChassis.setMaxSpeed(.7);
                    intake.servoControl(Intake.IntakeState.In);
                    if (cyclesComplete == 1) {
                        driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 3, 14, AngleUnit.DEGREES, 0));
                    } else {
                        driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, -8, 13, AngleUnit.DEGREES, 0));
                    }
                    autoStates = AutoStates.MoveToPickup;
                    break;
                case MoveToPickup:
                    if (didTimeout) {
                        autoStates = AutoStates.Reset;
                        break;
                    }
                    if (cyclesComplete == 1) {
                        driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, -3, -11, AngleUnit.DEGREES, 0));
                    } else {
                        driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 8, -15, AngleUnit.DEGREES, 0));
                    }
                    intake.setWristTarget(Intake.WristMode.SubPick, 300);
                    arm.setArmTarget(ArmMode.Lifted);
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
                    arm.setArmTarget(ArmMode.Lifted);
                    intake.setWristTarget(Intake.WristMode.SubPick, 0);
                    intake.setWristTarget(Intake.WristMode.Back, 1500);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 45));
                    autoStates = AutoStates.Stop;
                    break;
                case EReset:
                    driveChassis.setMaxSpeed(.5);
                    arm.setArmTarget(ArmMode.Lifted);
                    if (isRotated) {
                        driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 45));
                    }
                    intake.setWristTarget(Intake.WristMode.Back, 0);
                    autoStates = AutoStates.Stop;
                    break;
                case Stop:
                    terminateOpModeNow();
            }
        }
        driveChassis.update();
        arm.update();
        intake.update();

        arm.addTelemetry(telemetry);
        telemetry.addData("current State", autoStates);
        telemetry.addData("Chassis at target", driveChassis.atTarget());
        telemetry.addData("Arm at target", arm.isStopped());
        telemetry.addData("Intake at target", intake.isIntakeDone());
        telemetry.addData("Wrist at target", intake.isWristDone());
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
