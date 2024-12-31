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
            TelemetryUtil.addData("PedroDrivetrain", "Both waypoints are already set. Clear waypoints before setting new ones.");
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

        if (waypoint1 != null && waypoint2 != null) {
            if (isRobotNearPose(currPose, waypoint1, 1)) {
                Path path = new Path(new BezierLine(new Point(waypoint1), new Point(waypoint2)));
                path.setLinearHeadingInterpolation(waypoint1.getHeading(), waypoint2.getHeading());
                follower.followPath(path);
            } else if (isRobotNearPose(currPose, waypoint2, 1)) {
                Path path = new Path(new BezierLine(new Point(waypoint2), new Point(waypoint1)));
                path.setLinearHeadingInterpolation(waypoint2.getHeading(), waypoint1.getHeading());
                follower.followPath(path);
            }
        } else {
            TelemetryUtil.addData("PedroDrivetrain", "Both waypoints are not set!");
        }
    }

    // If we like this we can add methods like this for the rest of the heading types
    public void goToWaypointWithLinearHeadingFromCurrPose(Pose waypoint) {
        Pose currPose = follower.getPose();
        Path path = new Path(new BezierLine(new Point(currPose), new Point(waypoint)));
        path.setLinearHeadingInterpolation(currPose.getHeading(), waypoint.getHeading());
        follower.followPath(path);
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

    /** isRobotNearPose() method used for the autonomousPathUpdate and
     * checking the proximity of the robot, to a specific position. **/
    private boolean isRobotNearPose(Pose robotPose, Pose targetPose, double tolerance) {
        return Math.abs(robotPose.getX() - targetPose.getX()) <= tolerance &&
                Math.abs(robotPose.getY() - targetPose.getY()) <= tolerance;
    }
}
