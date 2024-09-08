package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous()
public class VisionDetectAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleDetectionPipeline propDetection = new SampleDetectionPipeline(SampleDetectionPipeline.ALLIANCE.BLUE);
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(propDetection)
                .build();

        visionPortal.setProcessorEnabled(propDetection, true);
        telemetry.addData("VisionPortal", "Initialized");

        while (opModeInInit()) {
            telemetry.addData("Bounding box", propDetection.getContourAmt());
            telemetry.addData("Alliance color sample center", propDetection.getBiggestCenter());
            telemetry.addData("Yellow sample center", propDetection.getBiggestYellowCenter());
            telemetry.update();
        }
    }
}
