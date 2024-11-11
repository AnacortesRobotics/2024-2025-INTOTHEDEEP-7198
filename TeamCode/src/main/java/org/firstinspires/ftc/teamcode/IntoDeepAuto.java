package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.DeepArm.ArmMode;
import org.firstinspires.ftc.teamcode.Intake.IntakeState;

import static java.lang.Thread.sleep;

@Autonomous

public class IntoDeepAuto extends LinearOpMode {

    public Chassis driveChassis;
    public DeepArm arm;
    public Intake intake;


    @Override
    public void runOpMode() throws InterruptedException {
        driveChassis = new Chassis();
        driveChassis.init(hardwareMap, telemetry);
        arm = new DeepArm();
        arm.init(hardwareMap, telemetry, this);
        intake = new Intake();
        intake.init(hardwareMap);

        waitForStart();

        telemetry.addData("Is this working", true);
        telemetry.update();
        arm.setArmState(0, 0, ArmMode.Lifted, true);
        intake.wristControl(Intake.WristState.SubPick, telemetry);
        sleep(500);
        driveChassis.driveToPosition(new Pose2D(DistanceUnit.INCH, -42, 12, AngleUnit.DEGREES, 0), telemetry, this, "move1", .6, true);
        sleep(200);
        driveChassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -43), telemetry, this, "turn1", .2, true);
        sleep(200);
        arm.setArmState(0, 0, ArmMode.Score, true);
        intake.wristControl(Intake.WristState.Score, telemetry);
        sleep(500);
        intake.servoControl(IntakeState.Out);
        sleep(400);
        intake.servoControl(IntakeState.Stop);
        intake.wristControl(Intake.WristState.SubPick, telemetry);
        sleep(400);
        arm.setArmState(0, 0, ArmMode.Lifted, true);
        intake.wristControl(Intake.WristState.Back, telemetry);
        sleep(300);
    }

}
