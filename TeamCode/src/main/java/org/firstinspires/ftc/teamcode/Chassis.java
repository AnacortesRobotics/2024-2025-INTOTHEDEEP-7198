package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private Telemetry telemetry;

    private static final int DEGREES_TO_BASKET = 225;
    private Pose2D position;
    private Pose2D posTarget = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private PIDFController pidfRotate = new PIDFController(0.01, 0, 0, 0, 0);

    public enum MotorTesting {
        lf,
        lb,
        rf,
        rb
    }

    
    public void init(HardwareMap hMap, Telemetry telemetry) {
        //Initailizes motors
        leftFront = hMap.get(DcMotor.class, "leftFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront = hMap.get(DcMotor.class, "rightFront");
        leftBack = hMap.get(DcMotor.class, "leftBack");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack = hMap.get(DcMotor.class, "rightBack");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = hMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(205.4467970180284, 169.7597803360906);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        this.telemetry = telemetry;
    }
    
    public void mecanumDrive(double forward, double strafe, double rotate) {
        // math to move and turn
        leftFront.setPower(forward + strafe - rotate);
        rightFront.setPower(forward - strafe + rotate);
        leftBack.setPower(forward - strafe - rotate);
        rightBack.setPower(forward + strafe + rotate);

    }

    public void mecanumDriveFieldCentric(double vertical, double horizontal, double rotate) {

        double heading = -odo.getHeading();
        double robotVert = Math.sin(heading) * horizontal + Math.cos(heading) * vertical;
        double robotHoriz = Math.cos(heading) * horizontal - Math.sin(heading) * vertical;
        mecanumDrive(robotVert, robotHoriz, rotate);
    }

    public void driveToPosition(Pose2D pos, Telemetry telemetry, LinearOpMode opMode, String name, double maxSpeed, boolean doMove) {
        odo.bulkUpdate();
        PIDFController pidfForward = new PIDFController(0.16, 0, 0, 0, 0);
        PIDFController pidfHorizontal = new PIDFController(0.16, 0, 0, 0, 0);
        PIDFController pidfRotate = new PIDFController(0.11, 0, 0, 0, 0);
        pidfForward.setOutputClamping(-maxSpeed, maxSpeed);
        pidfHorizontal.setOutputClamping(-maxSpeed, maxSpeed);
        pidfRotate.setOutputClamping(-maxSpeed, maxSpeed);
        Pose2D currentPos = getPosition();
        pos = addPos(pos, posTarget);
        while(Math.abs(pos.getX(DistanceUnit.INCH) - currentPos.getX(DistanceUnit.INCH)) >= 0.5 || Math.abs(pos.getY(DistanceUnit.INCH) - currentPos.getY(DistanceUnit.INCH)) >= 0.5 || Math.abs(pos.getHeading(AngleUnit.DEGREES) - currentPos.getHeading(AngleUnit.DEGREES)) >= 3) {
            odo.bulkUpdate();
            currentPos = getPosition();
            double forwardCorrect = pidfForward.updateClamped(pos.getY(DistanceUnit.INCH), currentPos.getY(DistanceUnit.INCH), System.currentTimeMillis());
            double horizontalCorrect = pidfHorizontal.updateClamped(pos.getX(DistanceUnit.INCH), currentPos.getX(DistanceUnit.INCH), System.currentTimeMillis());
            double rotateCorrect = pidfRotate.updateClamped(pos.getHeading(AngleUnit.DEGREES), currentPos.getHeading(AngleUnit.DEGREES), System.currentTimeMillis());
            if (doMove) {
                mecanumDriveFieldCentric(forwardCorrect, horizontalCorrect, rotateCorrect);
            }
            posTarget = pos;
            position = currentPos;
            String data2 = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", posTarget.getX(DistanceUnit.INCH), posTarget.getY(DistanceUnit.INCH), posTarget.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position target", data2);
            String data3 = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", horizontalCorrect, forwardCorrect, rotateCorrect);
            telemetry.addData("Outputs to the motors", data3);
            telemetry.addData("Name of loop", name);
            telemetry.update();
            updateOdo();
            if (opMode.isStopRequested()) {
                break;
            }
        }
    mecanumDriveFieldCentric(0, 0, 0);
    }

    public double orient(double targetAngle) {
        // math to rotate the robot to the correct angle
        //mecanumDriveFieldCentric(0, 0, clamp((targetAngle - odo.getPosition().getHeading(AngleUnit.DEGREES))/15,-0.5,0.5));
        return pidfRotate.update(targetAngle, odo.getPosition().getHeading(AngleUnit.DEGREES), System.currentTimeMillis());
    }

    public Pose2D addPos(Pose2D pos1, Pose2D pos2) {
        double posX = pos1.getX(DistanceUnit.INCH) + pos2.getX(DistanceUnit.INCH);
        double posY = pos1.getY(DistanceUnit.INCH) + pos2.getY(DistanceUnit.INCH);
        double posH = pos1.getHeading(AngleUnit.DEGREES) + pos2.getHeading(AngleUnit.DEGREES);
        return new Pose2D(DistanceUnit.INCH, posX, posY, AngleUnit.DEGREES, posH);
    }

    public void updateOdo()
    {
        odo.bulkUpdate();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", getPosition().getX(DistanceUnit.INCH), getPosition().getY(DistanceUnit.INCH), getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
    }

    public void resetOrient() {
        pidfRotate.reset();
    }

    public void motorTest(double forward, double strafe, double rotate, MotorTesting test) {
        switch (test) {
            case lf:
                leftFront.setPower(forward + strafe - rotate);
                break;
            case lb:
                leftBack.setPower(forward - strafe - rotate);
                break;
            case rf:
                rightFront.setPower(forward - strafe + rotate);
                break;
            case rb:
                rightBack.setPower(forward + strafe + rotate);
                break;
        }
    }

    public Pose2D getPosition() {
        return new Pose2D(DistanceUnit.MM, odo.getPosY(), odo.getPosX(), AngleUnit.RADIANS, odo.getHeading());
    }
}

