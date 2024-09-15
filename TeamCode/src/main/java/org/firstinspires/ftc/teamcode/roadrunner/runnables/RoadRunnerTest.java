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
                .lineTo(new Vector2d(-37.30, 42.65))
                .splineTo(new Vector2d(7.95, 62.06), Math.toRadians(2.22))
                .splineTo(new Vector2d(38.37, -42.96), Math.toRadians(269.18))
                .splineTo(new Vector2d(-0.76, -60.54), Math.toRadians(167.61))
                .splineTo(new Vector2d(-36.69, -58.85), Math.toRadians(180.95))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(testPath);
        }
    }
}
