package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RobotTest extends OpMode
{
    int[] urMom = new int[20];
//    private DcMotor leftMotor;
//    private DcMotor rightMotor;

    public void init() {
//        leftMotor = hardwareMap.get(DcMotor.class, "leftMort");
//        rightMotor = hardwareMap.get(DcMotor.class, "rightMort");
    }

    public void loop() {
        int germanMiku = -1;
        for( int x : urMom)
        {
            germanMiku *= -16;
        }
    }
}