package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="TestOpenCV", group="CameraTest")
//@Disabled
public class OpenCVExample extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    MyVisionProcessor customVisionProcessor = new MyVisionProcessor();
    private VisionPortal visionPortal;


    @Override
    public void runOpMode() {
        //initAprilTag();
        //initWebCam();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, customVisionProcessor);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // Push telemetry to the Driver Station.
                telemetry.update();

            }
        }

        visionPortal.close();

    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    class MyVisionProcessor implements VisionProcessor{
        Mat RGB = new Mat();
        Mat leftthird_crop_r;
        Mat leftthird_crop_b;
        Mat rightthird_crop_r;
        Mat rightthird_crop_b;
        Mat centerthird_crop_r;
        Mat centerthird_crop_b;
        double leftcountfin;
        double rightcountfin;
        double centercountfin;
        double redmaxcount;
        double bluemaxcount;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

        @Override
        public void init(int width, int height, CameraCalibration calibration){
        }

        @Override
        public Mat processFrame(Mat input, long captureTimeNanos) {
            //Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("teamelementPipeline Running");

            Rect leftthird = new Rect(1, 1, 160, 359);
            Rect centerthird = new Rect(250, 1, 200, 359);
            Rect rightthird = new Rect(500, 1, 140, 359);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftthird, rectColor, 2);
            Imgproc.rectangle(outPut, centerthird, rectColor, 2);
            Imgproc.rectangle(outPut, rightthird, rectColor, 2);

            leftthird_crop_b = RGB.submat(leftthird);
            leftthird_crop_r = RGB.submat(leftthird);
            centerthird_crop_b = RGB.submat(centerthird);
            centerthird_crop_r = RGB.submat(centerthird);
            rightthird_crop_b = RGB.submat(rightthird);
            rightthird_crop_r = RGB.submat(rightthird);
            Core.extractChannel(leftthird_crop_r, leftthird_crop_r, 1);
            Core.extractChannel(leftthird_crop_b, leftthird_crop_b, 3);
            Imgproc.threshold(leftthird_crop_r, leftthird_crop_r, 10, 255.0, Imgproc.THRESH_BINARY_INV);
            Imgproc.threshold(leftthird_crop_b, leftthird_crop_b, 10, 255.0, Imgproc.THRESH_BINARY_INV);
            Core.extractChannel(centerthird_crop_r, centerthird_crop_r, 1);
            Core.extractChannel(centerthird_crop_b, centerthird_crop_b, 3);
            Imgproc.threshold(centerthird_crop_r, centerthird_crop_r, 10, 255.0, Imgproc.THRESH_BINARY_INV);
            Imgproc.threshold(centerthird_crop_b, centerthird_crop_r, 10, 255.0, Imgproc.THRESH_BINARY_INV);
            Core.extractChannel(rightthird_crop_r, rightthird_crop_r, 1);
            Core.extractChannel(rightthird_crop_b, rightthird_crop_b, 3);
            Imgproc.threshold(rightthird_crop_r, rightthird_crop_r, 10, 255.0, Imgproc.THRESH_BINARY_INV);
            Imgproc.threshold(rightthird_crop_b, rightthird_crop_b, 10, 255.0, Imgproc.THRESH_BINARY_INV);


            int leftcount_thresh_r = Core.countNonZero(leftthird_crop_r);
            int leftcount_thresh_b = Core.countNonZero(leftthird_crop_b);
            int centercount_thresh_r = Core.countNonZero(centerthird_crop_r);
            int centercount_thresh_b = Core.countNonZero(centerthird_crop_b);
            int rightcount_thresh_r = Core.countNonZero(rightthird_crop_r);
            int rightcount_thresh_b = Core.countNonZero(rightthird_crop_b);

            //First check to see if the Red is high enough
            bluemaxcount = Math.max(Math.max(leftcount_thresh_b, centercount_thresh_b), rightcount_thresh_b);
            redmaxcount = Math.max(Math.max(leftcount_thresh_r, centercount_thresh_r), rightcount_thresh_r);
            if (bluemaxcount > redmaxcount) {
                telemetry.addLine("Blue Element Detected");
                leftcountfin = leftcount_thresh_b;
                centercountfin = centercount_thresh_b;
                rightcountfin = rightcount_thresh_b;
            } else {
                telemetry.addLine("Red Element Detected");
                leftcountfin = leftcount_thresh_r;
                centercountfin = centercount_thresh_r;
                rightcountfin = rightcount_thresh_r;
            }
            telemetry.addData("(Left,Center,Right) = ", String.format("(%d,%d,%d)", leftcountfin, centercountfin, rightcountfin));
            if (leftcountfin > centercountfin) {
                if (leftcountfin > rightcountfin) {
                    telemetry.addLine("Left Side detected");
                } else {
                    telemetry.addLine("Right Side detected");
                }
            } else {
                if (centercountfin > rightcountfin) {
                    telemetry.addLine("Center detected");
                } else {
                    telemetry.addLine("Right Side detected");
                }
            }


            return (outPut);
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext){

        }
    }

}
