package org.firstinspires.ftc.teamcode.opmode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.sensors.Sensors;
import org.firstinspires.ftc.teamcode.config.subsystem.outtake.Claw;
import org.firstinspires.ftc.teamcode.config.utils.wrappers.HardwareQueue;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;



@Autonomous(name = "4+0 Specimen Preload", group = "Autos")
public class PreloadedSpecimen4_0Auto extends OpMode {

    HardwareQueue queue;
    Sensors sensors;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** This is our Outtake Claw and Intake Claw subsystem.
     * We call its methods to manipulate the servos that it has within the subsystem. */
    public org.firstinspires.ftc.teamcode.config.subsystem.outtake.Claw oClaw;
    public org.firstinspires.ftc.teamcode.config.subsystem.outtake.Claw iClaw;


    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain pushSample1, pushingSample1, pushSample2, pushingSample2, goingToPickUp1, depositing1, goingToPickUp2, depositing2, goingToPickUp3, depositing3;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8, 60, Math.toRadians(180));

    /** Preloaded Specimen Scoring Pose; Outtake facing the submersible */
    private final Pose preloadScorePose = new Pose(38, 60, Math.toRadians(180));

    /** Pose once in position to push preset samples 1 and 2 */
    private final Pose pushSample1Pose = new Pose(60, 24, Math.toRadians(0));
    private final Pose pushSample1ControlPose = new Pose(27.5, 7);
    private final Pose pushSample1ControlPose2 = new Pose(60, 55);

    /** Pose after pushing the 1st preset sample */
    private final Pose pushedSample1Pose = new Pose(12, 24, Math.toRadians(0));
    private final Pose pushedSample1ControlPose = new Pose(9, 18);


    /** Pose after getting in position to push preset sample 2 */
    private final Pose pushSample2Pose = new Pose(60, 12, Math.toRadians(0));
    private final Pose pushSample2ControlPose = new Pose(76, 35);

    /** Pose after getting pushing sample 2 */
    private final Pose pushedSample2Pose = new Pose(12, 12, Math.toRadians(0));

    /** Pose where we pick up specimen */
    private final Pose specimenPickUpPose = new Pose(12, 24, Math.toRadians(0));
    private final Pose toSpecimenControlPose = new Pose(9.7, 18);
    private final Pose pickUpToDepositControlPose = new Pose(30, 24);
    private final Pose depositToPickUpControlPose = new Pose(29, 26);


    /** Specimen Scoring Pose for 1st specimen deposit; Outtake facing the submersible */
    private final Pose depositSpecimen1Pose = new Pose(38, 64, Math.toRadians(180));
    /** Specimen Scoring Pose for 2nd specimen deposit; Outtake facing the submersible */
    private final Pose depositSpecimen2Pose = new Pose(38, 68, Math.toRadians(180));
    /** Specimen Scoring Pose for 3rd specimen deposit; Outtake facing the submersible */
    private final Pose depositSpecimen3Pose = new Pose(38, 72, Math.toRadians(180));
    /** Specimen Scoring Pose for 4th specimen deposit; Outtake facing the submersible */
    private final Pose depositSpecimenControlPose = new Pose(20, 60);


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

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        // lime green, curly, 2 control points
        pushSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadScorePose), new Point(pushSample1ControlPose), new Point(pushSample1ControlPose2), new Point(pushSample1Pose)))
                .setLinearHeadingInterpolation(preloadScorePose.getHeading(), pushSample1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        // dark blue
        pushingSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushSample1Pose), new Point(pushedSample1ControlPose), new Point(pushedSample1Pose)))
                .setConstantHeadingInterpolation(pushedSample1Pose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        //dark/light purple
        pushSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushedSample1Pose), new Point(pushSample2ControlPose), new Point(pushSample2Pose)))
                .setConstantHeadingInterpolation(pushedSample2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        //green/blue
        pushingSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushSample2Pose), new Point(pushedSample2Pose)))
                .setConstantHeadingInterpolation(pushedSample2Pose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        // small, short, purple
        goingToPickUp1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushedSample2Pose), new Point(toSpecimenControlPose), new Point(specimenPickUpPose)))
                .setConstantHeadingInterpolation(specimenPickUpPose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        depositing1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenPickUpPose), new Point(pickUpToDepositControlPose), new Point(depositSpecimen1Pose)))
                .setLinearHeadingInterpolation(specimenPickUpPose.getHeading(), depositSpecimen1Pose.getHeading())
                .build();

        goingToPickUp2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(depositSpecimen1Pose), new Point(depositToPickUpControlPose), new Point(specimenPickUpPose)))
                .setLinearHeadingInterpolation(depositSpecimen1Pose.getHeading(), specimenPickUpPose.getHeading())
                .build();

        depositing2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenPickUpPose), new Point(pickUpToDepositControlPose), new Point(depositSpecimen2Pose)))
                .setLinearHeadingInterpolation(specimenPickUpPose.getHeading(), depositSpecimen2Pose.getHeading())
                .build();

        goingToPickUp3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(depositSpecimen2Pose), new Point(depositToPickUpControlPose), new Point(specimenPickUpPose)))
                .setLinearHeadingInterpolation(depositSpecimen2Pose.getHeading(), specimenPickUpPose.getHeading())
                .build();

        depositing3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenPickUpPose), new Point(pickUpToDepositControlPose), new Point(depositSpecimen3Pose)))
                .setLinearHeadingInterpolation(specimenPickUpPose.getHeading(), depositSpecimen3Pose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(depositSpecimen3Pose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(depositSpecimen3Pose.getHeading(), parkPose.getHeading());
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
                if (isRobotNearPose(currentPose, preloadScorePose, 1)) {
//                    oClaw.open();
                    follower.followPath(pushSample1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (isRobotNearPose(currentPose, pushSample1Pose, 1)) {
//                    oClaw.clamp();
                    follower.followPath(pushingSample1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (isRobotNearPose(currentPose, pushedSample1Pose, 1)) {
//                    oClaw.open();
                    follower.followPath(pushSample2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (isRobotNearPose(currentPose, pushSample2Pose, 1)) {
//                    oClaw.clamp();
                    follower.followPath(pushingSample2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (isRobotNearPose(currentPose, pushedSample2Pose, 1)) {
//                    oClaw.open();
                    follower.followPath(goingToPickUp1, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (isRobotNearPose(currentPose, specimenPickUpPose, 1)) {
                    // oClaw.clamp(); Uncomment if necessary
                    follower.followPath(depositing1, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (isRobotNearPose(currentPose, depositSpecimen1Pose, 1)) {
//                    oClaw.open();
                    follower.followPath(goingToPickUp2, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (isRobotNearPose(currentPose, specimenPickUpPose, 1)) {
//                    oClaw.open();
                    follower.followPath(depositing2, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (isRobotNearPose(currentPose, depositSpecimen2Pose, 1)) {
//                    oClaw.open();
                    follower.followPath(goingToPickUp3, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (isRobotNearPose(currentPose, specimenPickUpPose, 1)) {
//                    oClaw.open();
                    follower.followPath(depositing3, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (isRobotNearPose(currentPose, depositSpecimen3Pose, 1)) {
//                    oClaw.open();
                    follower.followPath(park, true);
                    setPathState(12);
                }
                break;
            case 12:
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

        oClaw = new Claw(hardwareMap, queue, sensors);

        // Set the oClaw to positions for init
        oClaw.clamp();
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
