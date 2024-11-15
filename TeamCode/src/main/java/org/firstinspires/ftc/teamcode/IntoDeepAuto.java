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
        LiftToScoreTwo,
        WristInPositionTwo,
        BlockReleasedTwo,
        WristReadyToDropTwo

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
        if(driveChassis.atTarget() && arm.isStopped() && intake.isIntakeDone() && intake.isWristDone()) {
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
                    driveChassis.setMaxSpeed(.6);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, -19, 11, AngleUnit.DEGREES, 0));
                    autoStates = AutoStates.StartMoveNetZone;
                    intake.setWristTarget(Intake.WristMode.SubPick, 500);
                    break;
                case StartMoveNetZone:
                    driveChassis.setMaxSpeed(.4);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -45));
                    arm.setArmTarget(ArmMode.Score);
                    autoStates = AutoStates.LiftToScoreOne;
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
                    intake.setWristTarget(Intake.WristMode.Pickup, 500);
                    autoStates = AutoStates.WristReadyToDrop;
                    break;
                case WristReadyToDrop:
                    arm.setArmTarget(ArmMode.Pickup);
                    driveChassis.setMaxSpeed(.4);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 45));
                    autoStates = AutoStates.ReadyToPickup;
                    break;
                case ReadyToPickup:
                    driveChassis.setMaxSpeed(.3);
                    intake.servoControl(Intake.IntakeState.In);
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 4, 18, AngleUnit.DEGREES, 0));
                    autoStates = AutoStates.MoveToPickup;
                    break;
                case MoveToPickup:
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, -4, -18, AngleUnit.DEGREES, 0));
                    intake.setWristTarget(Intake.WristMode.SubPick, 300);
                    arm.setArmTarget(ArmMode.Lifted);
                    autoStates = AutoStates.LiftToScoreTwo;
                    break;
                case LiftToScoreTwo:
                    driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -45));
                    arm.setArmTarget(ArmMode.Score);
                    autoStates = AutoStates.WristInPositionTwo;
                    break;
                case WristInPositionTwo:
                    intake.setWristTarget(Intake.WristMode.Pickup, 200);
                    autoStates = AutoStates.BlockReleasedTwo;
                    break;
                case BlockReleasedTwo:
                    break;
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
