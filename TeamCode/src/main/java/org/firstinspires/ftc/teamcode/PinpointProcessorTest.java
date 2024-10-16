package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.lang.annotation.Annotation;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.LogsUtils.*;


@TeleOp
@Config
public class PinpointProcessorTest extends OpMode {
    public static double X_OFFSET = -208.76;
    public static double Y_OFFSET = -174.62;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;
    public Chassis driveChassis;

    @Override
    public void init() {
        // Initalize ftc dashboard multi-telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        // Odometry wheel offsets in mm.
        odo.setOffsets(X_OFFSET, Y_OFFSET);

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        // Calibrate the orientation of the processor imu.
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        driveChassis = new Chassis();
        driveChassis.init(hardwareMap);
    }

    boolean toggleTuning = false;

    @Override
    public void loop() {
        /*
        Request a bulk update from the Pinpoint odometry computer. This checks almost all outputs
        from the device in a single I2C read.
         */
        odo.bulkUpdate();

        if (gamepad1.b){
            odo.resetPosAndIMU(); //recalibrates the IMU and position
//            odo.recalibrateIMU(); //recalibrates the IMU without resetting position
        }

        if(gamepad1.a)
            toggleTuning = true;
        if(gamepad1.x)
            toggleTuning = false;
        if (toggleTuning){
            AutoTuneRotation();
        }
        else
        {
            X_OFFSET = bestX;
            Y_OFFSET = bestY;
        }

        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;

        /*
        gets the current Position (x & y in cm, and heading in degrees) of the robot, and prints it.
         */
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        telemetry.addData("Status", odo.getDeviceStatus());

        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
        telemetry.update();

        double forward = exponentialRemapAnalog(deadZone(-gamepad1.left_stick_y, 0.02),2);
        double strafe = exponentialRemapAnalog(deadZone(gamepad1.left_stick_x, 0.02),2);
        double rotate = exponentialRemapAnalog(deadZone(-gamepad1.right_stick_x, 0.02),2);
        driveChassis.mecanumDrive(forward, strafe, rotate);
    }
    public static double rotationScore = Double.MAX_VALUE;
    double bestX = X_OFFSET;
    double bestY = Y_OFFSET;

    public void AutoTuneRotation()
    {
        odo.resetPosAndIMU();
        //rotate 90 degrees
        while (roundBetter(odo.getPosition().getHeading(AngleUnit.DEGREES), 2) != 90)
        {
            odo.bulkUpdate();
            Pose2D pos = odo.getPosition();
            driveChassis.mecanumDrive(0,0,clamp((90-pos.getHeading(AngleUnit.DEGREES))/90,-1,1));

            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("State", "Rotating heading 90deg");

            telemetry.update();
        }


        if(Math.abs(odo.getPosX())+Math.abs(odo.getPosY()) < rotationScore)
        {
            //its better
            rotationScore = Math.abs(odo.getPosX())+Math.abs(odo.getPosY());

            bestX = X_OFFSET;
            bestY = Y_OFFSET;
        }
        else
        {
            //its worse
            X_OFFSET = bestX;
            Y_OFFSET = bestY;

            bestX += (Math.random() * 2 - 1);
            bestY += (Math.random() * 2 - 1);
        }
    }
}
