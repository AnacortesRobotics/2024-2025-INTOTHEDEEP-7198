package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.DeepArm.ArmMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IntoDeepTeleOp extends OpMode {
    public Chassis driveChassis;
    private double forward;
    private double strafe;
    private double rotate;
    private RevBlinkinLedDriver blinkinLED;

    private DeepArm deepArm;
    private Intake intake;

    private boolean pickup = false;
    private ArmMode pickupMode = ArmMode.Off;

    @Override
    public void init() {
        driveChassis = new Chassis();
        driveChassis.init(hardwareMap);
        blinkinLED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLED");

        intake = new Intake();
        intake.init(hardwareMap);
        deepArm = new DeepArm();
        deepArm.init(hardwareMap);
        blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
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
            blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        } else if (gamepad2.a) {
            intake.servoControl(IntakeState.In);
        }
        intake.update();

        if (intake.isLimitDown()) {
            switch (intake.getIntakeColor()) {
                case Unknown:
                    blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                    break;
                case Red:
                    blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    break;
                case Blue:
                    blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    break;
                case Yellow:
                    blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
                    break;
            }
        }



/*
        if (!deepArm.isArmLimitMagnetDown()) {
            blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        } else {
            blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
*/
        if (gamepad2.dpad_down) {
            pickupMode = ArmMode.Pickup;
        } else if (gamepad2.dpad_up) {
            pickupMode = ArmMode.Score;
        } else if (gamepad2.dpad_right) {
            pickupMode = ArmMode.Lifted;
        }else if (gamepad2.dpad_left) {
            pickupMode = ArmMode.Off;
        }

        deepArm.setArmState(-gamepad2.left_stick_y, -gamepad2.right_stick_y, pickupMode);

        //deepArm.extendArm(-gamepad2.right_stick_y);
        intake.addTelemetry(telemetry);
        deepArm.addTelemetry(telemetry);
        telemetry.update();

    }
}
