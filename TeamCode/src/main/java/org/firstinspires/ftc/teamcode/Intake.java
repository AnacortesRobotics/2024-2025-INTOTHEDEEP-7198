package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
    private WristState wristState = WristState.Back;

    public enum WristState {
        Back,
        Score,
        Pickup
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

    public void init(HardwareMap hMap) {
        // Initailizes the servos
        leftIn = hMap.get(CRServo.class, "leftIn");
        rightIn = hMap.get(CRServo.class, "rightIn");
        intakeLimit = hMap.get(DigitalChannel.class, "intakeLimit");
        colorSensor = hMap.get(ColorSensor.class, "colorSensor");
        wrist = hMap.get(Servo.class, "wrist");

    }

    public void update() {
        switch (currentState) {
            case In:
                if (isLimitDown()) {
                    servoControl(IntakeState.Stop);
                }
        }

    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Is button pressed? ", isLimitDown());
        telemetry.addData("red from color sensor: ", colorSensor.red());
        telemetry.addData("green from color sensor: ", colorSensor.green());
        telemetry.addData("blue from color sensor: ", colorSensor.blue());
        telemetry.addData("Color in intake: ", getIntakeColor());
        telemetry.addData("Wrist position target", wrist.getPosition());
        telemetry.addData("Wrist state", wristState);
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

    public void wristControl(WristState state, Telemetry telemetry) {
        boolean check = true;
        telemetry.addData("Does it get here?", check);
        telemetry.addData("Whats the state being passed?", state);
        switch (state) {
            case Back:
                wrist.setPosition(.5);
                telemetry.addData("is back working", check);
                break;
            case Score:
                wrist.setPosition(.2);
                telemetry.addData("is score working", check);
                break;
            case Pickup:
                wrist.setPosition(0);
                telemetry.addData("is pickup working", check);
                break;
        }
        wristState = state;

    }


    public boolean isLimitDown() {
        return !intakeLimit.getState();
    }


}
