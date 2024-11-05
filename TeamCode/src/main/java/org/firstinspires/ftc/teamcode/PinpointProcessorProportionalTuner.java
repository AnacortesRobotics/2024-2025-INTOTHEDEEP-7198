package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.LogsUtils.*;


@TeleOp
@Config
public class PinpointProcessorProportionalTuner extends OpMode {
    public static double X_OFFSET = -208.76;
    public static double Y_OFFSET = -174.62;
    public static double TUNING_GAIN = 0.1555;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;
    public Chassis driveChassis;

//    boolean reset = false;
//    int targetRotation = 90;

    // used to reset odo without resetting odo
    double X_in_off = 0;
    double Y_in_off = 0;
    double Rot_in_off = 0;

    double GradientA = 0;
    double GradientB = 0;

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
        driveChassis.init(hardwareMap, telemetry);
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
            try (FileWriter file = new FileWriter("OdometryOffsets.txt"))
            {
                file.write("" + X_OFFSET + "\n" + Y_OFFSET);
            } catch (IOException ignored)
            {
                // oops
            }
//            odo.resetPosAndIMU(); //recalibrates the IMU and position
//            odo.recalibrateIMU(); //recalibrates the IMU without resetting position
        }

        if(gamepad1.a)
            toggleTuning = true;
        if(gamepad1.x)
            toggleTuning = false;

        if (toggleTuning){
            if (rotate90()) {
                AutoTuneRotation();
                odo.setOffsets(X_OFFSET, Y_OFFSET);
            }
        }
        else
        {
            X_OFFSET = bestX;
            Y_OFFSET = bestY;
            odo.setOffsets(X_OFFSET, Y_OFFSET);
        }

        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;

        /*
        gets the current Position (x & y in cm, and heading in degrees) of the robot, and prints it.
         */

        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM)-X_in_off, pos.getY(DistanceUnit.CM)-Y_in_off, pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("X_OFFSET", X_OFFSET);
        telemetry.addData("Y_OFFSET", Y_OFFSET);

        telemetry.addData("Status", odo.getDeviceStatus());

        telemetry.addData("Error", xError);

        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
        telemetry.update();

        double forward = exponentialRemapAnalog(deadZone(-gamepad1.left_stick_y, 0.02),2);
        double strafe = exponentialRemapAnalog(deadZone(gamepad1.left_stick_x, 0.02),2);
        double rotate = exponentialRemapAnalog(deadZone(-gamepad1.right_stick_x, 0.02),2);
//        driveChassis.mecanumDrive(forward, strafe, rotate);
    }

    public boolean rotate90()
    {
        //rotate 90 degrees
        if(Math.abs(odo.getPosition().getHeading(AngleUnit.DEGREES) - (90-Rot_in_off)) > 0.3)
        {
            Pose2D pos = odo.getPosition();
            driveChassis.mecanumDrive(0,0,clamp(((90-Rot_in_off)-pos.getHeading(AngleUnit.DEGREES))/12,-0.5,0.5));

            return false;
        }
        return true;
    }

    public static double xError = Double.MAX_VALUE;
    public static double yError = Double.MAX_VALUE;
    double bestX = X_OFFSET;
    double bestY = Y_OFFSET;

    public void AutoTuneRotation() {


        xError = odo.getPosX()-X_in_off;
        yError = odo.getPosY()-Y_in_off;

        X_OFFSET += xError * TUNING_GAIN;
        Y_OFFSET += yError * TUNING_GAIN;



        // Reset data offsets
        X_in_off = odo.getPosX();
        Y_in_off = odo.getPosY();
        Rot_in_off = odo.getPosition().getHeading(AngleUnit.DEGREES);
    }
}
