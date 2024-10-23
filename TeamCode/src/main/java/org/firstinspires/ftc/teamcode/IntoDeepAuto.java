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
        arm.init(hardwareMap, telemetry);
        intake = new Intake();
        intake.init(hardwareMap);

        waitForStart();

        driveChassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 24, 24, AngleUnit.DEGREES, 0), telemetry);
        arm.setArmState(0, 0, ArmMode.Score);
        driveChassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 72, 0, AngleUnit.DEGREES, -45), telemetry);
        intake.servoControl(IntakeState.Out);
        sleep(300);
        intake.servoControl(IntakeState.Stop);


    }

}
