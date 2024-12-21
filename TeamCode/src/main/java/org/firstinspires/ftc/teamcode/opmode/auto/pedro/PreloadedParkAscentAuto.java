package org.firstinspires.ftc.teamcode.opmode.auto.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Defines a blue side 2+0+Ascent autonomous
 */
@Autonomous(name="Preloaded Park Ascent Auto")
public class PreloadedParkAscentAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** Our robot object */
    private Robot robot;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(10, 85, 0);

    /** Preloaded sample Scoring Pose; Outtake facing the baskets */
    private final Pose basketPose = new Pose(20.479283314669654, 124.81075027995522, Math.toRadians(-45));

    /** Pose where we pick up the first sample */
    private final Pose sample1Pose = new Pose(33.70212765957447, 132.06718924972003, 0);

    /** Pose where we pick up the second sample */
    private final Pose sample2Pose = new Pose(33.70212765957447, 121.10190369540874, 0);

    /** Pose where we ascend */
    private final Pose ascentPose = new Pose(72, 97, Math.toRadians(-90));

    /** These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, ascend;
    private PathChain sample1, sample2;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        scorePreload = new Path(new BezierCurve(
                new Point(startPose),
                new Point(7.256, 116.426, Point.CARTESIAN),
                new Point(basketPose)
        ));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        ascend = new Path(new BezierCurve(
                new Point(basketPose),
                new Point(67.404, 122.553, Point.CARTESIAN),
                new Point(ascentPose)
        ));
        ascend.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90));

        // Picks up the first sample and scores it
        sample1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(basketPose),
                                new Point(sample1Pose)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(sample1Pose),
                                new Point(basketPose)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        // Picks up the second sample and scores it
        sample2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(basketPose),
                                new Point(sample2Pose)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(sample2Pose),
                                new Point(basketPose)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        Pose currentPose = follower.getPose();
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                robot.highBasket();
                robot.transfer();
                setPathState(1);
                break;
            case 1:
                if (isRobotNearPose(currentPose, basketPose, 0.75)) {
                    robot.intake();
                    robot.transfer();
                    follower.followPath(sample1);
                    robot.highBasket();
                    robot.transfer();
                    setPathState(2);
                    break;
                }
            case 2:
                if (isRobotNearPose(currentPose, basketPose, 0.75)) {
                    robot.intake();
                    robot.transfer();
                    follower.followPath(sample2);
                    robot.highBasket();
                    robot.transfer();
                    setPathState(3);
                    break;
                }
            case 3:
                if (isRobotNearPose(currentPose, basketPose, 0.75)) {
                    robot.intake();
                    robot.transfer();
                    follower.followPath(ascend);
                    robot.highBasket();
                    robot.transfer();
                    setPathState(4);
                    break;
                }
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** isRobotNearPose() method used for the autonomousPathUpdate and
     * checking the proximity of the robot, to a specific position. **/
    private boolean isRobotNearPose(Pose robotPose, Pose targetPose, double tolerance) {
        return Math.abs(robotPose.getX() - targetPose.getX()) <= tolerance &&
                Math.abs(robotPose.getY() - targetPose.getY()) <= tolerance;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        robot.update();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        robot.init();

        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
