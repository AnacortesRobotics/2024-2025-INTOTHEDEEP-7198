package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Intake.IntakeState;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IntoDeepTeleOp extends OpMode {
    public Chassis driveChassis;
    private double forward;
    private double strafe;
    private double rotate;

    private Intake intake;

    @Override
    public void init() {
        driveChassis = new Chassis();
        driveChassis.init(hardwareMap);

        intake = new Intake();
        intake.init(hardwareMap);

    }

    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        driveChassis.mecanumDrive(forward, strafe, rotate);

        if (gamepad2.x) {
            intake.servoControl(IntakeState.Stop);
        } else if (gamepad2.b) {
            intake.servoControl(IntakeState.Out);
        } else if (gamepad2.a) {
            intake.servoControl(IntakeState.In);

        }
        intake.update();

        intake.addTelemetry(telemetry);
        telemetry.update();

    }
}
