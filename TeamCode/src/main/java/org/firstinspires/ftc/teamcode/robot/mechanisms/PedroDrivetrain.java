package org.firstinspires.ftc.teamcode.robot.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

public class PedroDrivetrain {
    private final Follower follower;
    private final Gamepad gamepad;
    private Pose waypoint1;
    private Pose waypoint2;

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
        if (waypoint1 == null) {
            waypoint1 = follower.getPose();
        } else if (waypoint2 == null) {
            waypoint2 = follower.getPose();
        } else {
            TelemetryUtil.addData("PedroDrivetrain", "You already have two waypoints set! Clear them to set new ones.");
        }
    }

    public void saveWaypoint(Pose waypoint) {
        if (waypoint1 == null) {
            waypoint1 = waypoint;
        } else if (waypoint2 == null) {
            waypoint2 = waypoint;
        } else {
            TelemetryUtil.addData("PedroDrivetrain", "Both waypoints are already set. Clear waypoints before setting new ones.");
        }
    }

    public void goToWaypointWithLinearHeading() {
        Pose currPose = follower.getPose();

        if (waypoint1 != null) {
            if (waypoint2 != null) {
                Path path2 = new Path(new BezierLine(new Point(waypoint1), new Point(waypoint2)));
                path2.setLinearHeadingInterpolation(waypoint1.getHeading(), waypoint2.getHeading());
                follower.followPath(path2);
            } else {
                Path path1 = new Path(new BezierLine(new Point(currPose), new Point(waypoint1)));
                path1.setLinearHeadingInterpolation(currPose.getHeading(), waypoint1.getHeading());
                follower.followPath(path1);
            }
        }
    }

    public void goToWaypointWithConstantHeading() {
        Pose currPose = follower.getPose();

        if (waypoint1 != null) {
            if (waypoint2 != null) {
                Path path2 = new Path(new BezierLine(new Point(waypoint1), new Point(waypoint2)));
                path2.setConstantHeadingInterpolation(waypoint2.getHeading());
                follower.followPath(path2);
            } else {
                Path path1 = new Path(new BezierLine(new Point(currPose), new Point(waypoint1)));
                path1.setConstantHeadingInterpolation(waypoint1.getHeading());
                follower.followPath(path1);
            }
        }
    }

    public void goToWaypointWithTangentialHeading() {
        Pose currPose = follower.getPose();

        if (waypoint1 != null) {
            if (waypoint2 != null) {
                Path path2 = new Path(new BezierLine(new Point(waypoint1), new Point(waypoint2)));
                path2.setTangentHeadingInterpolation();
                follower.followPath(path2);
            } else {
                Path path1 = new Path(new BezierLine(new Point(currPose), new Point(waypoint1)));
                path1.setTangentHeadingInterpolation();
                follower.followPath(path1);
            }
        }
    }

    public void clearWaypoints() {
        waypoint1 = null;
        waypoint2 = null;
    }
}
