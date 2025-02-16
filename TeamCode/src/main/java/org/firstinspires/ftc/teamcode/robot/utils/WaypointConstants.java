package org.firstinspires.ftc.teamcode.robot.utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

@Config
public class WaypointConstants {
    public static Pose grabOffWall = new Pose(10, 35, Math.toRadians(180));
    public static Pose submersible = new Pose(41, 65.5, Math.toRadians(180));
    public static Point toSubmersibleControlPoint = new Point(22.0, 74.5, Point.CARTESIAN);
    private final Follower follower;

    public WaypointConstants(Follower follower) {
        this.follower = follower;
    }

    public PathChain getGrabOffWallPath() {
        Pose currentPose = follower.getPose();

        // Move towards submersible from current position
        Point targetPoint = new Point(new Pose(
                currentPose.getX() + (submersible.getX() - grabOffWall.getX()),
                currentPose.getY() + (submersible.getY() - grabOffWall.getY()),
                Math.toRadians(180)
        ));

        // Control point for smooth transition
        Point controlPoint = new Point(new Pose(
                currentPose.getX() + (toSubmersibleControlPoint.getX() - grabOffWall.getX()),
                currentPose.getY() + (toSubmersibleControlPoint.getY() - grabOffWall.getY()),
                Math.toRadians(180)
        ));

        return new PathChain(new Path(new BezierCurve(new Point(currentPose), controlPoint, targetPoint)));
    }

    public PathChain getSubmersiblePath() {
        Pose currentPose = follower.getPose();

        // Move backward from submersible to grabOffWall
        Point targetPoint = new Point(new Pose(
                currentPose.getX() + (grabOffWall.getX() - submersible.getX()),
                currentPose.getY() + (grabOffWall.getY() - submersible.getY()),
                Math.toRadians(180)
        ));

        // Control point for smooth transition
        Point controlPoint = new Point(new Pose(
                currentPose.getX() + (toSubmersibleControlPoint.getX() - submersible.getX()),
                currentPose.getY() + (toSubmersibleControlPoint.getY() - submersible.getY()),
                Math.toRadians(180)
        ));

        return new PathChain(new Path(new BezierCurve(new Point(currentPose), controlPoint, targetPoint)));
    }
}
