package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements ISubsystem {

    static Intake instance;

    private Intake() {}

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    //names the servos
    private Servo leftIn;
    private Servo rightIn;
    //private DigitalChannel intakeLimit;
    //private ColorSensor colorSensor;
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
    private boolean isEStop = false;
    private boolean debugFlag = false;

    public enum WristMode {
        Back,
        Score,
        Forward,
        Pickup
    }

    public enum WristState {
        CanMove,
        Wait,
    }

    public enum IntakeState {
        Back,
        Closed,
        Open,
        LineUp
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
        leftIn.setDirection(Servo.Direction.REVERSE);
        rightIn = hMap.get(Servo.class, "rightIn");
        //intakeLimit = hMap.get(DigitalChannel.class, "intakeLimit");
        wrist = hMap.get(Servo.class, "wrist");

        this.telemetry = telemetry;
    }

    public void update() {
        if (wristState == WristState.Wait && System.currentTimeMillis() - lastWristTargetCall > delayMs) {
            wristControl(wristMode);
            wristState = WristState.CanMove;
        }
    }

    public void setWristTarget(WristMode mode, long delay) {
        if (isEStop) {
            return;
        } else {
            wrist.getController().pwmEnable();
        }
        lastWristTargetCall = System.currentTimeMillis();
        delayMs = delay;
        wristMode = mode;
        wristState = WristState.Wait;
        debugFlag = true;
    }

    public void telemetry() {
        //telemetry.addData("Is button pressed? ", isLimitDown());
        //telemetry.addData("red from color sensor: ", colorSensor.red());
        //telemetry.addData("green from color sensor: ", colorSensor.green());
        //telemetry.addData("blue from color sensor: ", colorSensor.blue());
        //telemetry.addData("Color in intake: ", getIntakeColor());
        telemetry.addData("Wrist position target", wrist.getPosition());
        telemetry.addData("Wrist mode", wristMode);
        telemetry.addData("Wrist state", wristState);
        telemetry.addData("Intake direction", currentState);
        telemetry.addData("Is it stopped", isEStop);
        telemetry.addData("Did it arrive (The other show)", debugFlag);
    }

    public void servoControl(IntakeState state) {
        if (isEStop) {
            return;
        } else {
            rightIn.getController().pwmEnable();
            leftIn.getController().pwmEnable();
        }
        currentState = state;
        switch (state) {
            case Back:
                leftIn.setPosition(1);
                rightIn.setPosition(0);
                break;
            case Closed:
                leftIn.setPosition(0);
                rightIn.setPosition(1);
                break;
            case Open:
                leftIn.setPosition(0.5);
                rightIn.setPosition(0.5);
                break;
            case LineUp:
                rightIn.setPosition(1);
                leftIn.setPosition(1);
                break;
        }
        lastOutputTime = System.currentTimeMillis();
    }

//    public BlockColor getIntakeColor() {
//        colorSensor.enableLed(true);
//        int r = colorSensor.red();
//        int g = colorSensor.green();
//        int b = colorSensor.blue();
//        if (((r > b && r > g) && r > 1000) && r < 12000) {
//            return BlockColor.Red;
//        } else if (((g > b && g > r) && g > 1000) && g < 12000) {
//            return BlockColor.Yellow;
//        } else if (((b > r && b > g) && b > 1000) && b < 12000) {
//            return BlockColor.Blue;
//        } else {
//            return BlockColor.Unknown;
//        }
//    }

    public void wristControl(WristMode mode) {
        if (isEStop) {
            return;
        } else {
            wrist.getController().pwmEnable();
        }
        boolean check = true;
        telemetry.addData("Does it get here?", check);
        telemetry.addData("Whats the state being passed?", mode);
        lastWristTime = System.currentTimeMillis();
        lastWristPosition = wrist.getPosition();
        switch (mode) {
            case Back:
                wrist.setPosition(0);
                telemetry.addData("is back working", check);
                break;
            case Score:
                wrist.setPosition(.35);
                telemetry.addData("is score working", check);
                break;
            case Forward:
                wrist.setPosition(.5);
                telemetry.addData("is pickup working", check);
                break;
            case Pickup:
                wrist.setPosition(.9);
                telemetry.addData("is submersible pickup working", check);
        }
        wristMode = mode;
    }

    public boolean isDone() {
        return isIntakeDone() && isWristDone();
    }

    public boolean isIntakeDone() {
        return System.currentTimeMillis() - lastOutputTime > 200;
    }

    public boolean isWristDone() {
        return ((System.currentTimeMillis() - lastWristTime) > Math.abs(lastWristPosition - wrist.getPosition()) * 1500)
                && wristState == WristState.CanMove;
    }

    public void lock() {
        wrist.getController().pwmDisable();
        rightIn.getController().pwmDisable();
        leftIn.getController().pwmDisable();
        isEStop = true;
    }

    public void unlock() {
        isEStop = false;
    }

}
