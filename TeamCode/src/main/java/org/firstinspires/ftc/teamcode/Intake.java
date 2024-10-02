package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    //names the servos
    private CRServo leftIn;
    private CRServo rightIn;
    private TouchSensor intakeLimit;
    private ColorSensor colorSensor;

    private IntakeState currentState = IntakeState.Stop;

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
        intakeLimit = hMap.get(TouchSensor.class, "intakeLimit");
        colorSensor = hMap.get(ColorSensor.class, "colorSensor");

    }

    public void update() {
        switch (currentState) {
            case In:
                if (intakeLimit.isPressed()) {
                    servoControl(IntakeState.Stop);
                }
        }

    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Is button pressed? ", intakeLimit.isPressed());
        telemetry.addData("red from color sensor: ", colorSensor.red());
        telemetry.addData("green from color sensor: ", colorSensor.green());
        telemetry.addData("blue from color sensor: ", colorSensor.blue());
        telemetry.addData("Color in intake: ", getIntakeColor());
    }

    public void servoControl(IntakeState state) {
        currentState = state;
        switch (state) {
            case Stop:
                leftIn.setPower(0);
                rightIn.setPower(0);
                break;
            case In:
                if (!intakeLimit.isPressed()) {
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
        if ((r > b && r > g) && r > 1000) {
            return BlockColor.Red;
        } else if ((g > b && g > r) && g > 1000) {
            return BlockColor.Yellow;
        } else if ((b > r && b > g) && b > 1000) {
            return BlockColor.Blue;
        } else {
            return BlockColor.Unknown;
        }


    }

}