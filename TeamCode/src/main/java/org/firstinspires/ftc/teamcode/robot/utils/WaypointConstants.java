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
    public static Pose submersible = new Pose(41, 65.500, Math.toRadians(180));
    public static Point toSubmersibleControlPoint = new Point(22.000, 74.500, Point.CARTESIAN);
    public Follower follower;

    public WaypointConstants(Follower follower){
        this.follower = follower;
        generatePath();
    }

    public PathChain getGrabOffWallPath(){
        Point targetPoint = new Point(new Pose(follower.getPose().getX() + submersible.getX() - grabOffWall.getX(), follower.getPose().getY() + submersible.getY() - grabOffWall.getY(), Math.toRadians(180)));
        Point controlPoint = new Point(new Pose(follower.getPose().getX() + toSubmersibleControlPoint.getX() - grabOffWall.getX(), follower.getPose().getY() + toSubmersibleControlPoint.getY() - 35, Math.toRadians(180)));

        return new PathChain(new Path(new BezierCurve(new Point(follower.getPose()), controlPoint, targetPoint)));
    }

    public void generatePath(){
    }
}
