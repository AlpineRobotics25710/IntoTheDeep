package org.firstinspires.ftc.teamcode.opmode.teleop.prod;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@TeleOp(group="production")
public class Reset extends LinearOpMode {
    @Override
    public void runOpMode() {
        TelemetryUtil.setup(telemetry);
        Robot robot = new Robot(hardwareMap, true, true);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            robot.loop();
        }
        // Cancels all commands.
        CommandScheduler.getInstance().reset();
    }
}
