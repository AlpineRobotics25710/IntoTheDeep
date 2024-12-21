package org.firstinspires.ftc.teamcode.opmode.teleop.testers.gamepad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.control.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.control.drivetrain.DrivetrainBuilder;
import org.firstinspires.ftc.teamcode.robot.control.gamepad.CustomGamepad;

public class PedroDrivetrainTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this.hardwareMap);

        Follower follower = new Follower(hardwareMap);

        CustomGamepad gp1 = new CustomGamepad(gamepad1);

        Drivetrain drivetrain = new DrivetrainBuilder()
                .setType(DrivetrainBuilder.DrivetrainType.PEDRO_DRIVETRAIN)
                .setFollower(follower)
                .setGamepad(gp1)
                .setMotors(robot.frontLeftMotor, robot.backLeftMotor, robot.frontRightMotor, robot.backRightMotor)
                .setStrafingMultiplier(1.1)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gp1.cross().isClicked()) {
                drivetrain.saveWaypoint();
            }

            if (gp1.square().isClicked()) {
                drivetrain.goToWaypointWithLinearHeading();
            }

            telemetry.addData("Left stick y: ", -gamepad1.left_stick_y);
            telemetry.addData("Left stick X: ", gamepad1.left_stick_x);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

            follower.update();
            drivetrain.update();
            gp1.update();
            robot.update();
            telemetry.update();
        }
    }
}
