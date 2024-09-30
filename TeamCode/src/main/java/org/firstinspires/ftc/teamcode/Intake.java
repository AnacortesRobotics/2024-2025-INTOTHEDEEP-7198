package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    //names the servos
    private CRServo leftIn;
    private CRServo rightIn;

    public enum IntakeState {
        Stop,
        In,
        Out
    }

    public void init(HardwareMap hMap) {
        // Initailizes the servos
        leftIn = hMap.get(CRServo.class, "leftIn");
        rightIn = hMap.get(CRServo.class, "rightIn");

    }

    public void servoControl(IntakeState state) {

        switch (state) {
            case Stop:
                leftIn.setPower(0);
                rightIn.setPower(0);
                break;
            case In:
                leftIn.setPower(-0.5);
                rightIn.setPower(0.5);
                break;
            case Out:
                leftIn.setPower(0.5);
                rightIn.setPower(-0.5);
                break;
        }

    }
}
