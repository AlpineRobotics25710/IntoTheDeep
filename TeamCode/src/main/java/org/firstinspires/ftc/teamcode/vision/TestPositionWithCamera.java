package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.opencv.core.Mat;
import org.opencv.core.CvType;

import java.util.ArrayList;

@TeleOp(name="TestPositionWithCamera", group="Vision")
public class TestPositionWithCamera extends LinearOpMode {
    // Declare the camera and pipeline
    OpenCvCamera phoneCam;
    SampleDetectionPipelinePNP pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        // Get the camera view ID from the configuration
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Set up the camera and the pipeline
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SampleDetectionPipelinePNP();

        // Assign the pipeline to the camera
        phoneCam.setPipeline(pipeline);

        // Open the camera device asynchronously
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming from the camera with a resolution of 640x480
                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
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
            ArrayList<SampleDetectionPipelinePNP.AnalyzedStone> detectedStones = pipeline.getDetectedStones();
            for (SampleDetectionPipelinePNP.AnalyzedStone stone : detectedStones) {
                // Accessing the tvec and rvec for position and orientation
                Mat tvec = stone.tvec; // Translation vector
                Mat rvec = stone.rvec; // Rotation vector

                // Extracting values from the Mat
                double x = tvec.get(0, 0)[0]; // Get x value
                double y = tvec.get(1, 0)[0]; // Get y value
                double z = tvec.get(2, 0)[0]; // Get z value

                double roll = rvec.get(0, 0)[0]; // Get roll
                double pitch = rvec.get(1, 0)[0]; // Get pitch
                double yaw = rvec.get(2, 0)[0]; // Get yaw

                telemetry.addData("Detected", "Color: %s, Angle: %.2f, X: %.2f, Y: %.2f, Z: %.2f",
                        stone.color, stone.angle, x, y, z);
                telemetry.addData("Rotation (Rvec)", "Roll: %.2f, Pitch: %.2f, Yaw: %.2f",
                        roll, pitch, yaw);
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
