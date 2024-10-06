package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name="TestVisionWithCamera", group="Vision")
public class TestVisionWithCamera extends LinearOpMode {

    // Declare the camera and pipeline
    OpenCvCamera phoneCam;
    RedBlueDetectionPipelineNoPNP pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get the camera view ID from the configuration
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Set up the camera and the pipeline
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new RedBlueDetectionPipelineNoPNP();

        // Assign the pipeline to the camera
        phoneCam.setPipeline(pipeline);

        // Open the camera device asynchronously
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming from the camera with a resolution of 320x240
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        // Show status before starting
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();

        // Wait for the user to press the play button
        waitForStart();

        while (opModeIsActive()) {
            // Display detected stones information
            for (RedBlueDetectionPipelineNoPNP.AnalyzedStone stone : pipeline.getDetectedStones()) {
                telemetry.addData("Detected", stone.color + " stone with angle: " + stone.angle);
            }

            // Update telemetry with the number of frames per second (FPS)
            telemetry.addData("Frame Rate", phoneCam.getFps());
            telemetry.addData("Pipeline Time (ms)", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead Time (ms)", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical Max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();

            // Allow the system to continue running smoothly
            idle();
        }

        // Stop streaming when OpMode stops
        phoneCam.stopStreaming();
    }
}
