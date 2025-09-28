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

// USES ADVANCED EXAMPLES TO SHOW MORE FEATURES OF THE CAMERA
// THIS EXAMPLE USES THE BACK CAMERA. FOR THE FRONT CAMERA, USE OpenCvInternal

@TeleOp
public class InternalCamera1Advanced extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.setPipeline(new UselessColorBoxDrawingPipeline(new Scalar(255, 0, 0)));
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT, OpenCvInternalCamera.BufferMethod.DOUBLE); //
                phoneCam.setFlashlightEnabled(true); //flashlight
                phoneCam.setZoom(phoneCam.getMaxSupportedZoom()); //you can replace with any zoom value, this just sets to max zoom
                phoneCam.setRecordingHint(true); // recording hint???
                for (OpenCvInternalCamera.FrameTimingRange r : phoneCam.getFrameTimingRangesSupportedByHardware()) //sets framerate
                {
                    if(r.max == 30 && r.min == 30)
                    {
                        phoneCam.setHardwareFrameTimingRange(r);
                        break;
                    }
                }
            }

            @Override
            public void onError(int errorCode)
            {
                // idk what to put here but I think you can tell what this is for...
            }
        });

        waitForStart(); //continusly takes up time until start

        while (opModeIsActive())// this is for when the opmode is running
        {
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update(); //all the telemetry stuff

            if(gamepad1.a) //if a pressed, stop
            {
                phoneCam.stopStreaming();
            }
        }
        {
            sleep(100);
        }
    }

    class UselessColorBoxDrawingPipeline extends OpenCvPipeline
    {
        Scalar color;

        UselessColorBoxDrawingPipeline(Scalar color)
        {
            this.color = color;
        }

        @Override //im too lazy to write the coments just llok at internalcamera1
        {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    color, 4);

            return input;
        }
    }
}
