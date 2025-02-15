package org.firstinspires.ftc.teamcode.robot.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

public class WaypointConstants {
    public static Follower follower;

    public static Pose basketPose = new Pose(14.4, 132.16, Math.toRadians(315));
    public static Pose highChamberPose = new Pose(39.25, 65.500, Math.toRadians(180));
    public static Pose sampleIntakePose = new Pose(59.5, 92.5, Math.toRadians(270));
    public static Pose grabOffWallPose = new Pose(10, 35, Math.toRadians(180));

    //public static Path basketPath = new Path(new BezierLine(new Point(follower.getPose()), new Point(new Pose(follower.getPose().getX() + highChamberPose.getX() - grabOffWallPose.getX(), highChamberPose.getY(), Math.toRadians(180)))));
    public static Path basketPath = new Path(new BezierLine(new Point(follower.getPose()), new Point(basketPose)));
    public static Path highChamberPath = new Path(new BezierLine(new Point(follower.getPose()), new Point(highChamberPose)));
    public static Path sampleIntakePath = new Path(new BezierLine(new Point(follower.getPose()), new Point(sampleIntakePose)));
    public static Path grabOffWallPath = new Path(new BezierLine(new Point(follower.getPose()), new Point(grabOffWallPose)));

    private WaypointConstants() {}

    public static void setBasketPose(Pose basketPose) {
        WaypointConstants.basketPose = basketPose;
    }

    public static void setHighChamberPose(Pose highChamberPose) {
        WaypointConstants.highChamberPose = highChamberPose;
    }

    public static void setSampleIntakePose(Pose sampleIntakePose) {
        WaypointConstants.sampleIntakePose = sampleIntakePose;
    }

    public static void setGrabOffWallPose(Pose grabOffWallPose) {
        WaypointConstants.grabOffWallPose = grabOffWallPose;
    }
}
