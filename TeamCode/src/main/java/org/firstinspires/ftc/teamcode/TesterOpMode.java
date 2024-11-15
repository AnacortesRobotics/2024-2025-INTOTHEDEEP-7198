package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.Intake.WristMode;
import org.firstinspires.ftc.teamcode.DeepArm.ArmMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TesterOpMode extends OpMode {
    public Chassis driveChassis;
    private double forward;
    private double strafe;
    private double rotate;
    private RevBlinkinLedDriver blinkinLED;

    private DeepArm deepArm;
    private Intake intake;

    private boolean pickup = false;
    private ArmMode pickupMode = ArmMode.Off;
    private boolean driveMode = true;
    private int finishedLoops = 0;
    private long lastSecond = 0;
    private int loopRate = 0;
    private WristMode wristMode = WristMode.Back;
    private boolean bumperMode = false;
    private double turnPower = 0;
    private Intake.BlockColor ledColor = Intake.BlockColor.Unknown;

    private Chassis.MotorTesting motorTesting = Chassis.MotorTesting.lf;


    @Override
    public void init() {
        driveChassis = new Chassis();
        driveChassis.init(hardwareMap, telemetry);
        blinkinLED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkinLED");

        intake = new Intake();
        intake.init(hardwareMap, telemetry);
        deepArm = new DeepArm();
        deepArm.init(hardwareMap, telemetry, null);
        blinkinLED.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    @Override
    public void loop() {

        deepArm.setArmState(gamepad2.left_stick_y, gamepad2.right_stick_y, ArmMode.Off);

    }
}
