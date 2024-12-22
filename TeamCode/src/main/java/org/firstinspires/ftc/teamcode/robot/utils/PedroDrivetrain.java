package org.firstinspires.ftc.teamcode.robot.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class PedroDrivetrain {
    private Pose waypoint;
    private final Follower follower;
    private final Gamepad gamepad;

    public PedroDrivetrain(Gamepad gamepad, Follower follower, Pose startPose) {
        this.follower = follower;
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        this.gamepad = gamepad;
    }

    public void update() {
        follower.setTeleOpMovementVectors(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_y);
        follower.update();
    }

    public void saveWaypoint() {
        this.waypoint = follower.getPose();
    }

    public void saveWaypoint(Pose waypoint) {
        this.waypoint = waypoint;
    }

    public void goToWaypointWithLinearHeading() {
        Pose currPose = follower.getPose();
        Path path = new Path(new BezierLine(new Point(currPose), new Point(waypoint)));
        path.setLinearHeadingInterpolation(currPose.getHeading(), waypoint.getHeading());
    }

    public void goToWaypointWithConstantHeading() {
        Pose currPose = follower.getPose();
        Path path = new Path(new BezierLine(new Point(currPose), new Point(waypoint)));
        path.setConstantHeadingInterpolation(waypoint.getHeading());
    }

    public void goToWaypointWithTangentialHeading() {
        Pose currPose = follower.getPose();
        Path path = new Path(new BezierLine(new Point(currPose), new Point(waypoint)));
        path.setTangentHeadingInterpolation();
    }
}
