package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

@TeleOp
public class VisionTester extends OpMode {

    SampleProcessor processor;
//    VisionPortal.Builder visionPortalBuilder;
    VisionPortal visionPortal;

    @Override
    public void init() {
        processor = new SampleProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "sample Camera"))
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .addProcessor(processor)
                .build();
        visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();

        FtcDashboard.getInstance().startCameraStream(processor, 0);
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
//        Bitmap currentBitmap = processor.lastFrame.get();
//        if (currentBitmap != null) {
//            dashboard.sendImage(currentBitmap);
//        }
        //telemetry.addData("Rect list", processor.rectList);

        if (gamepad1.right_bumper) {
            visionPortal.resumeLiveView();
            telemetry.addData("Should be showing", true);
        } else if (gamepad1.left_bumper) {
            visionPortal.stopLiveView();
            telemetry.addData("Should be showing", false);
        }
        processor.addTelemetry(telemetry);
        telemetry.update();
    }

}
