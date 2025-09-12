package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class InternalCamera1 extends LinearOpMode
{
    OpenCvCamera phoneCam;

    @Override
    public void runOpMode()
    {
        //I have no idea what the next 6 lines do, but the code did it so I did it
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.setPipeline(new SamplePipeline());

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 idk man the code did this so im doing this even though IDK what to put here
                 */
            }
        });

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) //constantly update telemetry
        {
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();

            if(gamepad1.a) // press A to stop
            {
                phoneCam.stopStreaming();
            }
            //IMPORTATN: maybe remove this or change it, but I put this in so the loop doesn't run too fast and crash
            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused = false;

        @Override
        public Mat processFrame(Mat input) //the mat input is the frame
        {
            Imgproc.rectangle(
                    input, //the frame to draw on
                    new Point(
                            input.cols()/4, //x value of top left corner
                            input.rows()/4), //y value of top left corner
                    new Point(
                            input.cols()*(3f/4f), //x bottom right
                            input.rows()*(3f/4f)), //y bottom right
                    new Scalar(0, 255, 0), 4); //color (green) and thickness of the rectangle

            return input; //shows the modified frame for... idk
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused; //toggle the paused

            if(viewportPaused) //if its paused, pause it
            {
                phoneCam.pauseViewport();
            }
            else
            {
                phoneCam.resumeViewport();
            }
        }
    }
}
