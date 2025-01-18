package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
@TeleOp
public class ExtendoTest extends LinearOpMode {
    public static double targetPosition = 0;
    public static boolean firstIter = true;

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryUtil.setup(telemetry);
        Robot robot = new Robot(hardwareMap, true, false);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().schedule(new ExtendoCommand(robot, targetPosition));
            robot.loop();
            TelemetryUtil.update();
        }
        robot.end();
    }
}
