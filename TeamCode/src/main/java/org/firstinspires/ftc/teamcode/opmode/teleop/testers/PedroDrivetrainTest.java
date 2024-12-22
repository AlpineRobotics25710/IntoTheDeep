package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.robot.utils.PedroDrivetrain;

public class PedroDrivetrainTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose startPose = new Pose(0, 0, 0);
        Follower follower = new Follower(hardwareMap);
        PedroDrivetrain drivetrain = new PedroDrivetrain(gamepad1, follower, startPose);

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.update();
        }
    }
}
