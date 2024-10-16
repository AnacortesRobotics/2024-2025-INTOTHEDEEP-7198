package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.kaicode.PIDFController;

import java.lang.Math;
import java.util.Locale;

public class Chassis {
    //names motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private GoBildaPinpointDriver odo;

    
    public void init(HardwareMap hMap){
        //Initailizes motors
        leftFront = hMap.get(DcMotor.class, "leftFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront = hMap.get(DcMotor.class, "rightFront");
        leftBack = hMap.get(DcMotor.class, "leftBack");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack = hMap.get(DcMotor.class, "rightBack");

        odo = hMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(0, 0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        
    }
    
    public void mecanumDrive(double forward, double strafe, double rotate) {
        // math to move and turn
        leftFront.setPower(forward + strafe - rotate);
        rightFront.setPower(forward - strafe + rotate);
        leftBack.setPower(forward - strafe - rotate);
        rightBack.setPower(forward + strafe + rotate);
        
    }

    public void mecanumDriveFieldCentric(double vertical, double horizontal, double rotate) {
        double heading = odo.getHeading();
        double robotVert = Math.sin(heading) * horizontal + Math.cos(heading) * vertical;
        double robotHoriz = Math.cos(heading) * horizontal - Math.sin(heading) * vertical;
        mecanumDrive(robotVert, robotHoriz, rotate);
    }

    public void driveToPosition(Pose2D pos, Telemetry telemetry) {
        PIDFController pidfForward = new PIDFController(0.1, 0, 0, 0, 0);
        PIDFController pidfHorizontal = new PIDFController(0.1, 0, 0, 0, 0);
        PIDFController pidfRotate = new PIDFController(0.1, 0, 0, 0, 0);
        Pose2D currentPos = odo.getPosition();
        pos = addPos(pos, currentPos);
        while(Math.abs(pos.getX(DistanceUnit.INCH) - currentPos.getX(DistanceUnit.INCH)) >= 0.5 || Math.abs(pos.getY(DistanceUnit.INCH) - currentPos.getY(DistanceUnit.INCH)) >= 0.5 || Math.abs(pos.getHeading(AngleUnit.DEGREES) - currentPos.getHeading(AngleUnit.DEGREES)) >= 1) {
            currentPos = odo.getPosition();
            double forwardCorrect = pidfForward.update(pos.getY(DistanceUnit.INCH), currentPos.getY(DistanceUnit.INCH), .5, System.currentTimeMillis());
            double horizontalCorrect = pidfHorizontal.update(pos.getX(DistanceUnit.INCH), currentPos.getX(DistanceUnit.INCH), .5, System.currentTimeMillis());
            double rotateCorrect = pidfRotate.update(pos.getHeading(AngleUnit.DEGREES), currentPos.getHeading(AngleUnit.DEGREES), .5, System.currentTimeMillis());
            mecanumDriveFieldCentric(forwardCorrect, horizontalCorrect, rotateCorrect);


            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", currentPos.getX(DistanceUnit.INCH), currentPos.getY(DistanceUnit.INCH), currentPos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            String data2 = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position target", data2);
            telemetry.update();
        }
    }

    public Pose2D addPos(Pose2D pos1, Pose2D pos2) {
        double posX = pos1.getX(DistanceUnit.INCH) + pos2.getX(DistanceUnit.INCH);
        double posY = pos1.getY(DistanceUnit.INCH) + pos2.getY(DistanceUnit.INCH);
        double posH = pos1.getHeading(AngleUnit.DEGREES) + pos2.getHeading(AngleUnit.DEGREES);
        return new Pose2D(DistanceUnit.INCH, posX, posY, AngleUnit.DEGREES, posH);
    }
}

