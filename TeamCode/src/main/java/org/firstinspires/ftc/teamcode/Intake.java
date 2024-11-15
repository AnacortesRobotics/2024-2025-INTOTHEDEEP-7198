package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    //names the servos
    private CRServo leftIn;
    private CRServo rightIn;
    private DigitalChannel intakeLimit;
    private ColorSensor colorSensor;
    private Servo wrist;

    private IntakeState currentState = IntakeState.Stop;
    private WristMode wristMode = WristMode.Back;
    private WristState wristState = WristState.CanMove;

    private long lastOutputTime = 0;
    private long lastWristTime = 0;
    private double lastWristPosition = 0;
    private Telemetry telemetry;
    private double delayMs = 0;
    private long lastWristTargetCall = 0;

    public enum WristMode {
        Back,
        Score,
        Pickup,
        SubPick
    }

    public enum WristState {
        CanMove,
        Wait,
    }

    public enum IntakeState {
        Stop,
        In,
        Out
    }

    public enum BlockColor {
        Unknown,
        Yellow,
        Blue,
        Red
    }

    public void init(HardwareMap hMap, Telemetry telemetry) {
        // Initailizes the servos
        leftIn = hMap.get(CRServo.class, "leftIn");
        rightIn = hMap.get(CRServo.class, "rightIn");
        intakeLimit = hMap.get(DigitalChannel.class, "intakeLimit");
        colorSensor = hMap.get(ColorSensor.class, "colorSensor");
        wrist = hMap.get(Servo.class, "wrist");

        this.telemetry = telemetry;

        wrist.setPosition(0.9);
    }

    public void update() {
        if(isIntakeDone()) {
            servoControl(IntakeState.Stop);
        }
        if (wristState == WristState.Wait && System.currentTimeMillis() - lastWristTargetCall > delayMs) {
            wristControl(wristMode);
            wristState = WristState.CanMove;
        }
    }

    public void setWristTarget(WristMode mode, double delay) {
        lastWristTargetCall = System.currentTimeMillis();
        delayMs = delay;
        wristMode = mode;
        wristState = WristState.Wait;
    }

    public void addTelemetry() {
        telemetry.addData("Is button pressed? ", isLimitDown());
        telemetry.addData("red from color sensor: ", colorSensor.red());
        telemetry.addData("green from color sensor: ", colorSensor.green());
        telemetry.addData("blue from color sensor: ", colorSensor.blue());
        telemetry.addData("Color in intake: ", getIntakeColor());
        telemetry.addData("Wrist position target", wrist.getPosition());
        telemetry.addData("Wrist state", wristMode);
        telemetry.addData("Intake direction", currentState);
    }

    public void servoControl(IntakeState state) {
        currentState = state;
        switch (state) {
            case Stop:
                leftIn.setPower(0);
                rightIn.setPower(0);
                break;
            case In:
                if (!isLimitDown()) {
                // removed !,
                    leftIn.setPower(-0.5);
                    rightIn.setPower(0.5);
                }
                break;
            case Out:
                leftIn.setPower(0.5);
                rightIn.setPower(-0.5);
                lastOutputTime = System.currentTimeMillis();
                break;
        }

    }

    public BlockColor getIntakeColor() {
        colorSensor.enableLed(true);
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        if (((r > b && r > g) && r > 1000) && r < 12000) {
            return BlockColor.Red;
        } else if (((g > b && g > r) && g > 1000) && g < 12000) {
            return BlockColor.Yellow;
        } else if (((b > r && b > g) && b > 1000) && b < 12000) {
            return BlockColor.Blue;
        } else {
            return BlockColor.Unknown;
        }
    }

    public void wristControl(WristMode mode) {
        boolean check = true;
        telemetry.addData("Does it get here?", check);
        telemetry.addData("Whats the state being passed?", mode);
        lastWristTime = System.currentTimeMillis();
        lastWristPosition = wrist.getPosition();
        switch (mode) {
            case Back:
                wrist.setPosition(.9);
                telemetry.addData("is back working", check);
                break;
            case Score:
                wrist.setPosition(.6);
                telemetry.addData("is score working", check);
                break;
            case Pickup:
                wrist.setPosition(.36);
                telemetry.addData("is pickup working", check);
                break;
            case SubPick:
                wrist.setPosition(0);
                telemetry.addData("is submersible pickup working", check);
        }
        wristMode = mode;
    }

    public boolean isIntakeDone() {
        if(currentState == IntakeState.In) {
            return isLimitDown();
        } else if (currentState == IntakeState.Out) {
            return System.currentTimeMillis() - lastOutputTime > 500;
        }
        return true;
    }

    public boolean isWristDone() {
        return ((System.currentTimeMillis() - lastWristTime) > Math.abs(lastWristPosition - wrist.getPosition()) * 1000)
                && wristState == WristState.CanMove;
    }

    public boolean isLimitDown() {
        return !intakeLimit.getState();
    }


}
