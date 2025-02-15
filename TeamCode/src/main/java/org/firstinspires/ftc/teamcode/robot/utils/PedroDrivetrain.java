package org.firstinspires.ftc.teamcode.robot.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.Gamepad;

public class PedroDrivetrain {
    private static Pose BASKET_POSE = new Pose();
    private static Pose SPEC_SUBMERSIBLE_POSE = new Pose(39.25, 65.500, Math.toRadians(180)); // From 4 spec auto
    private static Pose SAMPLE_SUBMERSIBLE_POSE = new Pose(59.5, 92.5, Math.toRadians(270));
    private static Pose GRAB_OFF_WALL_POSE = new Pose(10, 35, Math.toRadians(180)); //  From 4 spec auto
    private final Follower follower;
    private final Gamepad gamepad;

    public PedroDrivetrain(Gamepad gamepad, Follower follower, Pose startPose) {
        this.follower = follower;
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        this.gamepad = gamepad;
    }

    public static void setBasketPose(Pose basketPose) {
        BASKET_POSE = basketPose;
    }

    public static void setSpecSubmersiblePose(Pose specSubmersiblePose) {
        SPEC_SUBMERSIBLE_POSE = specSubmersiblePose;
    }

    public static void setSampleSubmersiblePose(Pose specSubmersiblePose) {
        SAMPLE_SUBMERSIBLE_POSE = specSubmersiblePose;
    }

    public static void setGrabOffWallPose(Pose grabOffWallPose) {
        GRAB_OFF_WALL_POSE = grabOffWallPose;
    }

    public void goToBasket(HEADING_TYPE headingType) {
        if (headingType == HEADING_TYPE.TANGENTIAL) {
            Path path = new Path(new BezierLine(new Point(follower.getPose()), new Point(BASKET_POSE)));
            path.setTangentHeadingInterpolation();
            follower.followPath(path);
        } else if (headingType == HEADING_TYPE.LINEAR) {
            Path path = new Path(new BezierLine(new Point(follower.getPose()), new Point(BASKET_POSE)));
            path.setLinearHeadingInterpolation(follower.getPose().getHeading(), BASKET_POSE.getHeading());
            follower.followPath(path);
        } else if (headingType == HEADING_TYPE.CONSTANT) {
            Path path = new Path(new BezierLine(new Point(follower.getPose()), new Point(BASKET_POSE)));
            path.setConstantHeadingInterpolation(follower.getPose().getHeading());
            follower.followPath(path);
        }
    }

    public void goTo(HEADING_TYPE headingType) {
        if (headingType == HEADING_TYPE.TANGENTIAL) {
            Path path = new Path(new BezierLine(new Point(follower.getPose()), new Point(BASKET_POSE)));
            path.setTangentHeadingInterpolation();
            follower.followPath(path);
        } else if (headingType == HEADING_TYPE.LINEAR) {
            Path path = new Path(new BezierLine(new Point(follower.getPose()), new Point(BASKET_POSE)));
            path.setLinearHeadingInterpolation(follower.getPose().getHeading(), BASKET_POSE.getHeading());
            follower.followPath(path);
        } else if (headingType == HEADING_TYPE.CONSTANT) {
            Path path = new Path(new BezierLine(new Point(follower.getPose()), new Point(BASKET_POSE)));
            path.setConstantHeadingInterpolation(follower.getPose().getHeading());
            follower.followPath(path);
        }
    }

    public void goToSubmersible(HEADING_TYPE headingType) {
        //Point subPose = new Point(new Pose2D(follower.getPose().getX()+30, follower.);

        if (headingType == HEADING_TYPE.TANGENTIAL) {
            Path path = new Path(new BezierLine(new Point(follower.getPose()), new Point(SPEC_SUBMERSIBLE_POSE)));
            path.setTangentHeadingInterpolation();
            follower.followPath(path);
        } else if (headingType == HEADING_TYPE.LINEAR) {
            Path path = new Path(new BezierLine(new Point(follower.getPose()), new Point(SPEC_SUBMERSIBLE_POSE)));
            path.setLinearHeadingInterpolation(follower.getPose().getHeading(), SPEC_SUBMERSIBLE_POSE.getHeading());
            follower.followPath(path);
        } else if (headingType == HEADING_TYPE.CONSTANT) {
            Path path = new Path(new BezierLine(new Point(follower.getPose()), new Point(SPEC_SUBMERSIBLE_POSE)));
            path.setConstantHeadingInterpolation(follower.getPose().getHeading());
            follower.followPath(path);
        }
    }

    public void goToGrabOffWall(HEADING_TYPE headingType) {
        if (headingType == HEADING_TYPE.TANGENTIAL) {
            Path path = new Path(new BezierLine(new Point(follower.getPose()), new Point(GRAB_OFF_WALL_POSE)));
            path.setTangentHeadingInterpolation();
            follower.followPath(path);
        } else if (headingType == HEADING_TYPE.LINEAR) {
            Path path = new Path(new BezierLine(new Point(follower.getPose()), new Point(GRAB_OFF_WALL_POSE)));
            path.setLinearHeadingInterpolation(follower.getPose().getHeading(), GRAB_OFF_WALL_POSE.getHeading());
            follower.followPath(path);
        } else if (headingType == HEADING_TYPE.CONSTANT) {
            Path path = new Path(new BezierLine(new Point(follower.getPose()), new Point(GRAB_OFF_WALL_POSE)));
            path.setConstantHeadingInterpolation(follower.getPose().getHeading());
            follower.followPath(path);
        }
    }

    public void update() {
        follower.setTeleOpMovementVectors(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_y);
        follower.update();
    }

    /**
     * isRobotNearPose() method used for the autonomousPathUpdate and
     * checking the proximity of the robot, to a specific position.
     **/
    private boolean isRobotNearPose(Pose robotPose, Pose targetPose, double tolerance) {
        return Math.abs(robotPose.getX() - targetPose.getX()) <= tolerance &&
                Math.abs(robotPose.getY() - targetPose.getY()) <= tolerance;
    }

    public enum HEADING_TYPE {
        TANGENTIAL, LINEAR, CONSTANT
    }
}
