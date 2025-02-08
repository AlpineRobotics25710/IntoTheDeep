package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.mechanisms.PedroDrivetrain;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

public class PedroDrivetrainTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose startPose = new Pose(140, 140, 180);
        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap);
        PedroDrivetrain drivetrain = new PedroDrivetrain(gamepad1, follower, startPose);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> drivetrain.goToBasket(PedroDrivetrain.HEADING_TYPE.LINEAR));
        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> drivetrain.goToSubmersible(PedroDrivetrain.HEADING_TYPE.LINEAR));
        gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> drivetrain.goToGrabOffWall(PedroDrivetrain.HEADING_TYPE.LINEAR));

        TelemetryUtil.setup(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.update();
        }
    }
}
