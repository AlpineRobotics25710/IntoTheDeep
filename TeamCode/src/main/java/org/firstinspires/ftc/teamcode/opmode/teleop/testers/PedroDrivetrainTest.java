//package org.firstinspires.ftc.teamcode.opmode.teleop.testers;
//
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
//import org.firstinspires.ftc.teamcode.robot.mechanisms.PedroDrivetrain;
//import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;
//
//public class PedroDrivetrainTest extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose startPose = new Pose(0, 0, 0);
//        Constants.setConstants(FConstants.class, LConstants.class);
//        Follower follower = new Follower(hardwareMap);
//        PedroDrivetrain drivetrain = new PedroDrivetrain(gamepad1, follower, startPose);
//        GamepadEx gp1 = new GamepadEx(gamepad1);
//        TelemetryUtil.setup(telemetry);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
//                drivetrain.saveWaypoint();
//            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
//                drivetrain.goToWaypointWithLinearHeading();
//            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
//                drivetrain.goToWaypointWithConstantHeading();
//            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
//                drivetrain.goToWaypointWithTangentialHeading();
//            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//                drivetrain.clearWaypoints();
//            }
//
//            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//                follower.breakFollowing();
//            }
//
//            drivetrain.update();
//        }
//    }
//}
