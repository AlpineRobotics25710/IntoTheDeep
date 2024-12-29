package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

@Config
@TeleOp
public class OuttakeSlidesTest extends LinearOpMode {
    public static boolean manualMode = false;
    public static double targetPosition = 0.0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Slides slides = new OuttakeSlides();
        slides.init();

        waitForStart();

        while (opModeIsActive()) {
            slides.setTargetPosition(targetPosition);
            slides.setManualMode(manualMode);

            // Cubed to slowly increase speed
            slides.setManualPower(Range.clip(Math.pow(-gamepad1.left_stick_y, 3), -1, 1));

            telemetry.addData("Target Position", slides.getTargetPosition());
            telemetry.addData("Encoder Position", slides.getEncoderPosition());
            telemetry.addData("Manual Mode", slides.manualMode);

            slides.update();
            telemetry.update();
        }
    }
}
