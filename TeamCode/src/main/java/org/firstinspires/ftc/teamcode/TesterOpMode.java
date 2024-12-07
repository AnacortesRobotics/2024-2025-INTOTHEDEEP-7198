package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Intake.WristMode;
import org.firstinspires.ftc.teamcode.DeepArm.ArmMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

@TeleOp
public class TesterOpMode extends OpMode {
    private GoBildaPinpointDriver odo;
    public Chassis driveChassis;
    public ArmManager armManager;


    @Override
    public void init() {
        driveChassis = new Chassis();
        armManager = new ArmManager();
        armManager.init(hardwareMap, telemetry);
        driveChassis.init(hardwareMap, telemetry, true);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setPosition(new Pose2D(DistanceUnit.INCH, Chassis.ROBOT_LENGTH / 2, Chassis.ROBOT_WIDTH / -2, AngleUnit.DEGREES, 0));

    }
// 38.5, 9.25

//11.8, 25.6
    @Override
    public void loop() {

        driveChassis.updateOdo();
        driveChassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -45), true);
//        armManager.manualArmMove(-gamepad2.left_stick_y, -gamepad2.right_stick_y);
//        armManager.updateTelemetry();

    }
}
