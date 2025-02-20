package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.robot.utils.PedroDrivetrain;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.robot.utils.WaypointGenerator;

@TeleOp
@Config
public class WaypointingTest extends LinearOpMode {
    public static boolean robotCentric = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, false);
        WaypointGenerator waypointGenerator = new WaypointGenerator(robot.follower);
        robot.follower.setStartingPose(new Pose(0,0,0));
        TelemetryUtil.setup(telemetry);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new FollowPathCommand(robot.follower, waypointGenerator.getSubmersiblePath()));
        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new FollowPathCommand(robot.follower, waypointGenerator.getGrabOffWallPath()));

        waitForStart();

        if (isStarted()) {
            robot.follower.startTeleopDrive();
        }

        while (opModeIsActive()) {
            robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, robotCentric);

            if ((gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) && robot.follower.isBusy()) {
                robot.follower.breakFollowing();
                robot.follower.startTeleopDrive();
            }

            robot.loop();
            TelemetryUtil.update();
        }
    }
}
