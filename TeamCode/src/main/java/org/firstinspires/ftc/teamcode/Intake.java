package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    //names the servos
    private Servo leftIn;
    private Servo rightIn;
    private DigitalChannel intakeLimit;
    private ColorSensor colorSensor;
    private Servo wrist;

    private IntakeState currentState = IntakeState.Back;
    private WristMode wristMode = WristMode.Back;
    private WristState wristState = WristState.CanMove;

    private long lastOutputTime = 0;
    private long lastWristTime = 0;
    private double lastWristPosition = 0;
    private Telemetry telemetry;
    private long delayMs = 0;
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
        Back,
        Closed,
        Open
    }

    public enum BlockColor {
        Unknown,
        Yellow,
        Blue,
        Red
    }

    public void init(HardwareMap hMap, Telemetry telemetry) {
        // Initailizes the servos
        leftIn = hMap.get(Servo.class, "leftIn");
        rightIn = hMap.get(Servo.class, "rightIn");
        intakeLimit = hMap.get(DigitalChannel.class, "intakeLimit");
        colorSensor = hMap.get(ColorSensor.class, "colorSensor");
        wrist = hMap.get(Servo.class, "wrist");

        this.telemetry = telemetry;

        wrist.setPosition(0.9);
    }

    public void update() {
        if(isIntakeDone()) {
            servoControl(IntakeState.Back);
        }
        if (wristState == WristState.Wait && System.currentTimeMillis() - lastWristTargetCall > delayMs) {
            wristControl(wristMode);
            wristState = WristState.CanMove;
        }
    }

    public void setWristTarget(WristMode mode, long delay) {
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
            case Back:
                leftIn.setPosition(0);
                rightIn.setPosition(0);
            case Closed:
                leftIn.setPosition(1);
                rightIn.setPosition(1);
                break;
            case Open:
                leftIn.setPosition(0.5);
                rightIn.setPosition(0.5);
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
        return System.currentTimeMillis() - lastOutputTime > 200;
    }

    public boolean isWristDone() {
        return ((System.currentTimeMillis() - lastWristTime) > Math.abs(lastWristPosition - wrist.getPosition()) * 1500)
                && wristState == WristState.CanMove;
    }

    public boolean isLimitDown() {
        return !intakeLimit.getState();
    }


}
