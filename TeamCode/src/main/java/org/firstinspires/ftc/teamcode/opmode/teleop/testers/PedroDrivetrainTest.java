package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.robot.mechanisms.PedroDrivetrain;

public class PedroDrivetrainTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose startPose = new Pose(0, 0, 0);
        Follower follower = new Follower(hardwareMap);
        PedroDrivetrain drivetrain = new PedroDrivetrain(gamepad1, follower, startPose);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                drivetrain.saveWaypoint();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                drivetrain.goToWaypointWithLinearHeading();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                drivetrain.goToWaypointWithConstantHeading();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                drivetrain.goToWaypointWithTangentialHeading();
            }

            drivetrain.update();
        }
    }
}
