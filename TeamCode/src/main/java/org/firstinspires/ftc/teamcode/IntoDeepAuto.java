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
    public PIDFMosesController pControlX;
    public PIDFMosesController pControlY;

    @Override
    public void init() {
        driveChassis = new Chassis();
        driveChassis.init(hardwareMap, telemetry, true);
        pControlX = new PIDFMosesController();
        pControlX.init(0.02, 0, 0.001, false);
        pControlY = new PIDFMosesController();
        pControlY.init(0.02, 0, 0.001, true);
    }


    @Override
    public void loop() {
        driveChassis.updateOdo();
        driveChassis.mecanumDriveFieldCentric(
                pControlY.update(150, driveChassis.getPosition().getY(DistanceUnit.MM), .5),
                pControlX.update(250, driveChassis.getPosition().getX(DistanceUnit.MM), .5), 0);

    }

}
