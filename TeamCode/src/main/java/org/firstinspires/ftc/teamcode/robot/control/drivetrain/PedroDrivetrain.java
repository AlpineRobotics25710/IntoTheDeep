package org.firstinspires.ftc.teamcode.robot.control.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.robot.control.gamepad.CustomGamepad;

public class PedroDrivetrain extends Drivetrain {
    private Pose waypoint;
    private final Follower follower;

    PedroDrivetrain(DcMotor frontLeftMotor, DcMotor backLeftMotor, DcMotor frontRightMotor, DcMotor backRightMotor, CustomGamepad gamepad, Follower follower) {
        super(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, gamepad);
        this.follower = follower;
        follower.setTeleOpMovementVectors(-gamepad.leftStick().getY(), gamepad.leftStick().getX(), gamepad.rightStick().getY());
    }

    @Override
    public void update() {
        follower.update();
    }

    @Override
    public void saveWaypoint() {
        this.waypoint = follower.getPose();
    }

    @Override
    public void saveWaypoint(Pose waypoint) {
        this.waypoint = waypoint;
    }

    @Override
    public void goToWaypointWithLinearHeading() {
        Pose currPose = follower.getPose();
        Path path = new Path(new BezierLine(new Point(currPose), new Point(waypoint)));
        path.setLinearHeadingInterpolation(currPose.getHeading(), waypoint.getHeading());
    }

    @Override
    public void goToWaypointWithConstantHeading() {
        Pose currPose = follower.getPose();
        Path path = new Path(new BezierLine(new Point(currPose), new Point(waypoint)));
        path.setConstantHeadingInterpolation(waypoint.getHeading());
    }

    @Override
    public void goToWaypointWithTangentialHeading() {
        Pose currPose = follower.getPose();
        Path path = new Path(new BezierLine(new Point(currPose), new Point(waypoint)));
        path.setTangentHeadingInterpolation();
    }
}
