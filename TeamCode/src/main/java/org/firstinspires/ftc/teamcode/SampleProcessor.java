package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.provider.ContactsContract;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class SampleProcessor implements VisionProcessor, CameraStreamSource {

    public static Scalar lowerBlue = new Scalar(102, 110, 80);
    public static Scalar upperBlue = new Scalar(160, 255, 255);
    public static Scalar lowerRed = new Scalar(140, 120, 110);
    public static Scalar upperRed = new Scalar(180, 255, 255);
    public static Scalar lowerYellow = new Scalar(10, 50, 85);
    public static Scalar upperYellow = new Scalar(50, 255, 255);

    public static int colorTracked = 0;
    public static int outputFrame = 0;
    private Object sync = new Object();
    // Be aware of resolution if camera changes
    private Mat output = new Mat(1280, 720, CvType.CV_8UC3);
    private static final Point CENTER_OF_SCREEN = new Point(620, 360);
    public Point targetPoint = new Point();
    Paint paint = new Paint();

    //TODO: get the correct number here. (from the height, with the camera, be aware of distortion)
    // converts pixels to inches, so number of inches in a pixel, not number of pixels in an inch
    private static final double PIXELS_TO_INCHES = 1;

    private List<RotatedRect> rectList = new ArrayList<>();
    private List<Point> possibleTargets = new ArrayList<>();

    public final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    @Override
    public void init(int width, int height, CameraCalibration cameraCalibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        paint.setColor(R.color.firstred);
        paint.setStrokeWidth(8);
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Mat blur = new Mat(input.rows(),input.cols(), CvType.CV_8UC3);
        Imgproc.GaussianBlur(input, blur, new Size (3, 3), 0, 0);
        Mat gray = new Mat(input.rows(),input.cols(), CvType.CV_8U);
        Mat hsv = new Mat(input.rows(), input.cols(), CvType.CV_8UC3);
        Mat colorMask = new Mat(input.rows(), input.cols(), CvType.CV_8U);
        Mat dilatedColorMask = new Mat(input.rows(), input.cols(), CvType.CV_8U);
        Mat edges = new Mat(input.rows(), input.cols(), CvType.CV_8U);
        Mat cutMask = new Mat(input.rows(), input.cols(), CvType.CV_8U);
        Mat notEdges = new Mat(input.rows(), input.cols(), CvType.CV_8U);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat(input.rows(), input.cols(), CvType.CV_8U);
        Imgproc.cvtColor(blur, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);

        if (colorTracked == 0) {
            Core.inRange(hsv, lowerYellow, upperYellow, colorMask);
        }
        else if (colorTracked == 1) {
            Core.inRange(hsv, lowerBlue, upperBlue, colorMask);
        }
        else if (colorTracked == 2) {
            Core.inRange(hsv, lowerRed, upperRed, colorMask);
        }

        erodeAndDilateMask(colorMask, dilatedColorMask);
        Imgproc.Canny(gray, edges, 50, 70);
        Imgproc.dilate(edges, edges, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Core.bitwise_not(edges, notEdges);
        Core.bitwise_and(dilatedColorMask, notEdges, cutMask);
        Imgproc.findContours(cutMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        synchronized (sync) {
            rectList.clear();
            possibleTargets.clear();
            for (MatOfPoint c : contours) {
                if (Imgproc.contourArea(c) < 400) {
                    continue;
                }
                RotatedRect box = Imgproc.minAreaRect(new MatOfPoint2f(c.toArray()));
                if (box.angle < 45 || box.angle > 270 ) {
                    possibleTargets.add(box.center);
                }
                rectList.add(box);
            }
        }

        if (outputFrame == 0) {
            output = cutMask;
        } else if (outputFrame == 1) {
            output = edges;
        } else if (outputFrame == 2) {
            output = dilatedColorMask;
        }

        return rectList;

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        synchronized (sync) {
            for (RotatedRect rect : rectList) {
                drawRotatedRect(canvas, rect, scaleBmpPxToCanvasPx);
            }
        }

        findTarget();
        paint.setColor(R.color.dashboardColor);
        canvas.drawPoint((float) CENTER_OF_SCREEN.x, (float) CENTER_OF_SCREEN.y, paint);
        paint.setColor(R.color.active_button_green);
        canvas.drawPoint((float)targetPoint.x, (float)targetPoint.y, paint);
        Imgproc.drawMarker(output, CENTER_OF_SCREEN, new Scalar(0, 255, 0));
        Imgproc.drawMarker(output, targetPoint, new Scalar(0, 0, 255));
        Bitmap b = Bitmap.createBitmap(output.width(), output.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(output, b);
        lastFrame.set(b);
    }

    private void erodeAndDilateMask(Mat input, Mat output) {
        Mat maskEroded = new Mat(input.rows(), input.cols(), CvType.CV_8U);
        Imgproc.erode(input, maskEroded, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(maskEroded, output, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(7, 7)));
    }

    private void drawRotatedRect(Canvas canvas, RotatedRect rect, float scale) {
        Point[] rectPoints = new Point[4];
        float[] rectFloats = new float[8];
        rect.points(rectPoints);
        for (int i = 0; i < 4; i++) {
            rectFloats[i * 2] = (float)rectPoints[i].x * scale;
            rectFloats[i * 2 + 1] = (float)rectPoints[i].y * scale;
        }
        Point centerBox = rect.center;
        double angle = rect.angle;
        Imgproc.putText(output, "Center: " + (int)centerBox.x + " , " + (int)centerBox.y + " Angle: " + angle,
                centerBox, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 0, 0));
        paint.setColor(R.color.firstred);
        //canvas.drawLines(rectFloats, paint);
    }

    public void findTarget() {
        Point closestPoint = new Point();
        double minDistance = 9999;
        for (Point point : possibleTargets) {
            double distance = Math.sqrt(Math.pow(point.x - CENTER_OF_SCREEN.x, 2) + Math.pow(point.y - CENTER_OF_SCREEN.y, 2));
            if (distance < minDistance) {
                minDistance = distance;
                closestPoint = point;
            }
        }
        targetPoint = closestPoint;
    }

    public double getTravelDistance(boolean horizontal) {
        if (horizontal) {
            return (CENTER_OF_SCREEN.x - targetPoint.x) * PIXELS_TO_INCHES;
        } else {
            return (CENTER_OF_SCREEN.y - targetPoint.y) * PIXELS_TO_INCHES;
        }
    }

    public boolean isOnTarget() {
        return Math.abs(CENTER_OF_SCREEN.x - targetPoint.x) < 30 &&
                Math.abs(CENTER_OF_SCREEN.y - targetPoint.y) < 30;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Is in position", isOnTarget());
        telemetry.addData("Horizontal distance", getTravelDistance(true));
        telemetry.addData("Vertical distance", getTravelDistance(false));
    }

    @Override
    public void getFrameBitmap(Continuation<? extends org.firstinspires.ftc.robotcore.external.function.Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
