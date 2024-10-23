package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@TeleOp

public class MosesTest7198 extends OpMode{
    
    public Chassis driveChassis;
    public Arm arm;
    private double forward;
    private double strafe;
    private double rotate;
    private boolean height = true;
    private boolean handOpen = true;
    private double fineControl = 1.0;
    private boolean pickup = false;
    
    public void init() {
        
        driveChassis = new Chassis();
        driveChassis.init(hardwareMap, telemetry);
        arm = new Arm();
        arm.init(hardwareMap);
        //arm.liftWrist();
    }
    
    public void loop() {
        
        if (gamepad1.a) {
            fineControl = 0.3;
        } else if (gamepad1.b) {
            fineControl = 1.0;
        }
        
        forward = -gamepad1.left_stick_y * fineControl;
        strafe = gamepad1.left_stick_x * fineControl;
        rotate = gamepad1.right_stick_x * fineControl;
        driveChassis.mecanumDrive(forward, strafe, rotate);
        
        if (gamepad2.dpad_up){
            height = true;
        } else if (gamepad2.dpad_down) {
            height = false;
        }
        arm.wristControl(height);
        
        if (gamepad2.a) {
            handOpen = true;
        } else if (gamepad2.b) {
            handOpen = false;
        }
        
        arm.handControl(handOpen);
        
        if (gamepad2.x) {
            pickup = true;
        } else if (gamepad2.y) {
            pickup = false;
        }
        
        arm.pickupMode(pickup);
        arm.armHeight(-gamepad2.right_stick_y);
        arm.armExtender(-gamepad2.left_stick_y);
        
        arm.addTelemetry(telemetry);
        telemetry.update();
        
    }
    
    
}
