package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.Slides;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.ExtendoSlides;

@Config
@TeleOp
public class ExtendoSlidesTest extends CommandOpMode {
    public static boolean manualMode = false;
    public static double targetPosition = 0.0;
    Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.extendo.setTargetPosition(targetPosition);
        robot.extendo.setManualMode(manualMode);

        // Cubed to slowly increase speed
        double manualSlidesPower = Range.clip(Math.pow(-gamepad1.left_stick_y, 3), -1, 1);
        robot.extendo.moveSlides(manualSlidesPower);

        telemetry.addData("Target Position", robot.extendo.getTargetPosition());
        telemetry.addData("Encoder Position", robot.extendo.getEncoderPosition());
        telemetry.addData("Manual Mode", robot.extendo.manualMode);

        super.run();
        telemetry.update();
    }

    @Override
    public void initialize() {
        super.reset();
        robot.init(hardwareMap);
        register(robot.extendo);
    }
}
