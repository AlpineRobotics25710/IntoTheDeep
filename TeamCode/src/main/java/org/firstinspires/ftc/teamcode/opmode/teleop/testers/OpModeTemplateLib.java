package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class OpModeTemplateLib extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, false, true);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            robot.loop();
        }
        // Cancels all commands.
        CommandScheduler.getInstance().reset();
    }
}
