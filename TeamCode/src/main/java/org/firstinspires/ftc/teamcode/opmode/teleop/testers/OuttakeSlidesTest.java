package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SlidesCommand;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;
@Config
@TeleOp
public class OuttakeSlidesTest extends LinearOpMode {
    public static double targetPosition = 100.0;
    public static boolean manualMode = true;
    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryUtil.setup(telemetry);
        Robot robot = new Robot(hardwareMap, true);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if(manualMode){
                robot.outtakeSlides.setSlidesPower(gamepad1.left_stick_y);
            }

            else{
                CommandScheduler.getInstance().schedule(new SlidesCommand(robot, targetPosition, false));
            }
            robot.loop();

            TelemetryUtil.addData("target position", robot.outtakeSlides.getTargetPosition());
            TelemetryUtil.addData("slides position", robot.outtakeSlides.getCurrentPosition());
            TelemetryUtil.addData("slides reached", robot.outtakeSlides.slidesReached);
            TelemetryUtil.addData("dumbahh power", robot.outtakeSlides.getPower());
            TelemetryUtil.update();
        }
        CommandScheduler.getInstance().reset();
    }
}
