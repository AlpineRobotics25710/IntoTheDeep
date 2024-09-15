package org.firstinspires.ftc.teamcode.roadrunner.runnables;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

        TrajectorySequence testPath = drive.trajectorySequenceBuilder(startPose)
                .forward(15)
                .turn(Math.toRadians(90))
                .forward(15)
                .turn(Math.toRadians(90))
                .forward(15)
                .turn(Math.toRadians(90))
                .forward(15)
                .turn(Math.toRadians(90))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(testPath);
        }
    }
}
