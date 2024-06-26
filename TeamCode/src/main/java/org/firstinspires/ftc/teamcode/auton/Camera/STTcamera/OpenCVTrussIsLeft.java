package org.firstinspires.ftc.teamcode.auton.Camera.STTcamera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

public class OpenCVTrussIsLeft {
    LinearOpMode myOpMode;
    OpenCvWebcam webcam1 = null;

    public int pos = 0; //Left 0, Middle 1, Right 2
    double centerAvgFin;
    double rightAvgFin;

    public OpenCVTrussIsLeft(LinearOpMode opMode) {myOpMode = opMode;}
    public void findScoringPosition() {
        WebcamName webcamName = myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    class examplePipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Rect midRect = new Rect(350, 200, 180, 199);
        Rect rightRect = new Rect(900, 250, 230, 240);

        Mat outPut = new Mat();
        Scalar redColor = new Scalar(255.0, 0.0, 0.0);
        Scalar greenColor = new Scalar(0.0, 255.0, 0.0);
        public Mat processFrame(Mat input) {
            // We do not change the second parameter, because aids.
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2HSV);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, midRect, redColor, 2);
            Imgproc.rectangle(outPut, rightRect, redColor, 2);
            Mat midCrop = YCbCr.submat(midRect);
            Mat rightCrop = YCbCr.submat(rightRect);

            /**For YCbCr: Blue = 1, Red = 2
             * For HSV: measures intensity so always use 1. No clue what the other values do/measure/are.
             */
            Core.extractChannel(midCrop, midCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 1);

            Scalar centerAvg = Core.mean(midCrop);
            Scalar rightAvg = Core.mean(rightCrop);

            centerAvgFin = centerAvg.val[0];
            rightAvgFin = rightAvg.val[0];

            myOpMode.telemetry.addData("valueRight", rightAvgFin);
            myOpMode.telemetry.addData("valueCenter", centerAvgFin);

            if (Math.abs(centerAvgFin - rightAvgFin) > 25 && centerAvgFin > rightAvgFin) {
                Imgproc.rectangle(outPut, midRect, greenColor, 2);
                pos = 1;

                myOpMode.telemetry.addData("Current_Pos", pos);
                myOpMode.telemetry.addData("Conclusion", "mid");

            } else if (Math.abs(centerAvgFin - rightAvgFin) > 25 && rightAvgFin > centerAvgFin) {
                Imgproc.rectangle(outPut, rightRect, greenColor, 2);
                pos = 2;
                myOpMode.telemetry.addData("Current_Pos", pos);
                myOpMode.telemetry.addData("Conclusion", "right");
            } else {
                myOpMode.telemetry.addData("Current_Pos", pos);
                myOpMode.telemetry.addData("Conclusion", "left");
                pos = 0;
            }

            myOpMode.telemetry.addData("pos", pos);
            myOpMode.telemetry.update();
            return(outPut);
        }
    }
}