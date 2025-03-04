package org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is the StrafeBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * sideways. On reaching the end of the Path, the robot runs the other direction Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * the translational vector especially.
 *
 * @author raj - 25710
 */
@Config
@Autonomous (name = "Strafe Back And Forth", group = "PIDF Tuning")
public class StrafeBackAndForth extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean left = true;

    private Follower follower;

    private Path sideways;
    private Path otherway;

    /**
     * This initializes the Follower and creates the strafe Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        sideways = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(0, DISTANCE, Point.CARTESIAN)));
        sideways.setConstantHeadingInterpolation(0);
        otherway = new Path(new BezierLine(new Point(0,DISTANCE, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        otherway.setConstantHeadingInterpolation(0);

        follower.followPath(sideways);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a SIDEWAYS line going " + DISTANCE
                + " inches SIDEWAYS. The robot will go SIDEWAYS continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            if (left) {
                left = false;
                follower.followPath(otherway);
            } else {
                left = true;
                follower.followPath(sideways);
            }
        }

        telemetryA.addData("going left i think", left);
        follower.telemetryDebug(telemetryA);
    }
}
