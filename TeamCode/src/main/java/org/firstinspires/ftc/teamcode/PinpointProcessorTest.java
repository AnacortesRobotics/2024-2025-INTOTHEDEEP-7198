package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.io.File;

import java.io.FileWriter;
import java.io.IOException;
import java.lang.annotation.Annotation;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.LogsUtils.*;

/**
 * A test class for the pinpoint processors offsets and localization
 * @author Logan R
 *
 */

@TeleOp
@Config
public class PinpointProcessorTest extends OpMode {
    public static double X_OFFSET = 205.44679701802843;
    public static double Y_OFFSET = 169.75978033609064;
    public static double TuningGain = 2;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    // used to calculate the delta time for frequency readout
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

        // print pinpoint device info
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X_OFFSET", X_OFFSET);
        telemetry.addData("Y_OFFSET", Y_OFFSET);
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // set up the drive chassis and corresponding systems
        driveChassis = new Chassis();
        driveChassis.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        /*
        Request a bulk update from the Pinpoint odometry computer. This checks almost all outputs
        from the device in a single I2C read.
         */
        odo.bulkUpdate();

        // frequency calculation
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;


        //gets the current Position (x & y in cm, and heading in degrees) of the robot, and prints it.
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.CM), pos.getY(DistanceUnit.CM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("X_OFFSET", X_OFFSET);
        telemetry.addData("Y_OFFSET", Y_OFFSET);

        telemetry.addData("Status", odo.getDeviceStatus());


        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
        telemetry.update();

        // filtered movement for offset testing
        double forward = exponentialRemapAnalog(deadZone(-gamepad1.left_stick_y, 0.02),2);
        double strafe = exponentialRemapAnalog(deadZone(gamepad1.left_stick_x, 0.02),2);
        double rotate = exponentialRemapAnalog(deadZone(-gamepad1.right_stick_x, 0.02),2);
        driveChassis.mecanumDrive(forward, strafe, rotate);
    }
}
