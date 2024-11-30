package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.utils.Globals;
import org.firstinspires.ftc.teamcode.config.utils.RunMode;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@TeleOp(name = "Example Field-Centric Teleop", group = "Teleop")
public class Teleop extends OpMode {
    private Robot robot;
    private final Pose startPos = new Pose(0, 0, 0); //needs to be changed to follower.getPose();
    @Override
    public void init() {
        Globals.mode = RunMode.TELEOP;
        robot = new Robot(hardwareMap);
        robot.follower.setStartingPose(startPos);
    }

    @Override
    public void loop() {
        robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_y, -gamepad1.right_stick_x, true);
        robot.update(); //might have to go above previous line

        telemetry.addData("X", robot.follower.getPose().getX());
        telemetry.addData("Y", robot.follower.getPose().getX());
        telemetry.addData("Heading in Degrees", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("Linkage Length", robot.extendo.getLength());
    }
}
