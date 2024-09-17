package org.firstinspires.ftc.teamcode.roadrunner.runnables;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(group = "roadrunner")
public class RoadRunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);

        Pose2d startPose = new Pose2d(-35, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence testPath = drive.trajectorySequenceBuilder(new Pose2d(-36.08, -58.70, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(0.15, -30, Math.toRadians(90.00)), Math.toRadians(90.00))
                .lineTo(new Vector2d(0.00, -47))
                .splineToConstantHeading(new Vector2d(-20, -44), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-40, -28, Math.toRadians(180)), Math.toRadians(180.00))
                .lineToLinearHeading(new Pose2d(-60, -55, Math.toRadians(245.00)))
                .lineToLinearHeading(new Pose2d(-58, -28, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-60, -55, Math.toRadians(245.00)))
                .lineToLinearHeading(new Pose2d(-68, -28, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-60, -55, Math.toRadians(245.00)))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(testPath);
        }
    }
}
