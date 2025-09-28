package org.firstinspires.ftc.teamcode;

// TeleOp mode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// random imports idk
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.PipelineRecordingParameters;

@TeleOp // makes this
public class PipelineRecorder extends LinearOpMode
{
    OpenCvCamera phoneCam;

    @Override
    public void runOpMode()
    {
        // get the ID for the camera monitor view
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // create the internal camera thing using the BACK camera and the monitor view
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // set the image processing pipeline to the custom SamplePipeline thing
        phoneCam.setPipeline(new SamplePipeline());
        // idk what this does but the example code did it so I did it
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        phoneCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

        // Open the camera faster i assume???
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //you can tell what this means
                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
        telemetry.addLine("Waiting for start");
        telemetry.update();

        // waiting for user to press start
        waitForStart();

        //main loop
        while (opModeIsActive())
        {
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();

            // modify this for the number to be lower bc it is too high but im setting it to 100 for safety but when you actually do the thing modify it
            sleep(100);
        }
    }

    // this is the custom pipeline class
    class SamplePipeline extends OpenCvPipeline
    {
        //tracks if recording is active
        boolean toggleRecording = false;

        // runs each camera frame
        @Override
        public Mat processFrame(Mat input)
        {
            // draws a green rectangle in the middle of the frame
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            // Return the modified frame for display
            return input;
        }

        // pressing the viewport toggles recording
        @Override
        public void onViewportTapped()
        {
            // Toggle recording on/off
            toggleRecording = !toggleRecording;

            if(toggleRecording)
            
                // outputs the pipeline to a video file on the RC phone
                phoneCam.startRecordingPipeline(
                        new PipelineRecordingParameters.Builder()
                                .setBitrate(4, PipelineRecordingParameters.BitrateUnits.Mbps) // bitrate
                                .setEncoder(PipelineRecordingParameters.Encoder.H264) // apparently some shit with the H264 encoder
                                .setOutputFormat(PipelineRecordingParameters.OutputFormat.MPEG_4) //MP4
                                .setFrameRate(30) // 30fps (at least i think its seconds)
                                .setPath("/sdcard/pipeline_rec.mp4") // file path on RC phone
                                .build()); // build the recording things

            else
            {
                //STOP
                phoneCam.stopRecordingPipeline();
            }
        }
    }
}
