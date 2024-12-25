package org.firstinspires.ftc.teamcode.opmode.auto.pedro;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.robot.oldcode.Robot;


@Autonomous(name = "2+0 Auto Specimen Preload", group = "Autos")
public class PreloadedSpecimen2_0Auto extends OpMode {
    private Robot robot;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;


    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches

     /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain goingToPickUp, depositing;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8, 60, Math.toRadians(180));

    /** Preloaded Specimen Scoring Pose; Outtake facing the submersible */
    private final Pose preloadScorePose = new Pose(38, 60, Math.toRadians(180));

    /** Pose where we pick up specimen */
    private final Pose specimenPickUpPose = new Pose(12, 24, Math.toRadians(0));
    private final Pose depositToPickUpControlPose = new Pose(30, 24);


    /** Specimen Scoring Pose for 1st specimen deposit; Outtake facing the submersible */
    private final Pose specimenDepositPose = new Pose(38, 64, Math.toRadians(180));
    private final Pose pickUpToDepositControlPose = new Pose(20, 60);


    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(9.5, 10, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(29, 26);



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

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        //aqua/turquoise
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(preloadScorePose)));
        scorePreload.setConstantHeadingInterpolation(preloadScorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */


        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        // small, short, purple
        goingToPickUp = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadScorePose), new Point(depositToPickUpControlPose), new Point(specimenPickUpPose)))
                .setLinearHeadingInterpolation(preloadScorePose.getHeading(), specimenPickUpPose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        depositing = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenPickUpPose), new Point(depositToPickUpControlPose), new Point(specimenDepositPose)))
                .setLinearHeadingInterpolation(specimenPickUpPose.getHeading(), specimenDepositPose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(specimenDepositPose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(specimenDepositPose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        Pose currentPose = follower.getPose();
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (isRobotNearPose(currentPose, preloadScorePose, 0.75)) {
//                  deposit sequence(lower slides, open claw)
                    follower.followPath(goingToPickUp, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (isRobotNearPose(currentPose, specimenPickUpPose, 0.75)) {
//                  pick up sequence(oClaw.clamp() to close claw)
                    follower.followPath(depositing, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (isRobotNearPose(currentPose, specimenDepositPose, 0.75)) {
//                  deposit sequence(lower slides, open claw)
                    follower.followPath(park, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (isRobotNearPose(currentPose, parkPose, 1)) {
//                    oClaw.open();
                    setPathState(-1); // Stop the state machine
                }
                break;
        }
    }

    /** isRobotNearPose() method used for the autonomousPathUpdate and
     * checking the proximity of the robot, to a specific position. **/
    private boolean isRobotNearPose(Pose robotPose, Pose targetPose, double tolerance) {
        return Math.abs(robotPose.getX() - targetPose.getX()) <= tolerance &&
                Math.abs(robotPose.getY() - targetPose.getY()) <= tolerance;
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

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

        buildPaths();

        robot.init();
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
    public void stop() {
    }
}
