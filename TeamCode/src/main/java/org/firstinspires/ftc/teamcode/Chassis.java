package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Chassis {
    
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    
    public void init(HardwareMap hMap){
        
        leftFront = hMap.get(DcMotor.class, "leftFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront = hMap.get(DcMotor.class, "rightFront");
        leftBack = hMap.get(DcMotor.class, "leftBack");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack = hMap.get(DcMotor.class, "rightBack");
        
        
    }
    
    public void mecanumDrive(double forward, double strafe, double rotate) {
        
        leftFront.setPower(forward - strafe - rotate);
        rightFront.setPower(forward - strafe + rotate);
        leftBack.setPower(forward + strafe - rotate);
        rightBack.setPower(forward + strafe + rotate);
        
    }
    
}

