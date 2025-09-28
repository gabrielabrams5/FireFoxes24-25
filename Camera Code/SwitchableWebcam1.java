package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;
import org.openftc.easyopencv.OpenCvCamera;

public class SwitchableWebcam1 extends LinearOpMode
{
    WebcamName webcam1;
    WebcamName webcam2;
    OpenCvSwitchableWebcam switchableWebcam;

    @Override
    public void runOpMode() throws InterruptedException 
    {

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //I dont get this part???
        switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, webcam1, webcam2);

        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                switchableWebcam.setPipeline(new SamplePipeline());
                switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                // idk what to put here but I think you can tell what this is for...
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addLine("PRESS A/B TO SWITCH CAMERA\n");
            telemetry.addData("Frame Count ", switchableWebcam.getFrameCount());
            telemetry.addData("FPS ", String.format("%.2f", switchableWebcam.getFps()));
            telemetry.addData("Total frame time ms ", switchableWebcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms ", switchableWebcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms ", switchableWebcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS ", switchableWebcam.getCurrentPipelineMaxFps());
            telemetry.update();

            /**
             Note: this is not the best way to handle button presses, but it works for this example.
             Beware: Also this is different from pressing the button a in other scripts, because in other ones pressing a stops the streaming
             and here it switches the camera.
             */
            if(gamepad1.a) // press a to switch to cam 1
            {
                switchableWebcam.setActiveCamera(webcam1);
            }
            else if(gamepad1.b) // press b to switch to cam 2
            {
                switchableWebcam.setActiveCamera(webcam2);
            }

            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline
    { //yeah yk what to do here
        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            return input;
        }
    }
}
