package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@TeleOp(name = "Extendo Control Teleop", group = "Teleop")
public class Teleop extends OpMode {
    private Robot robot;
    private final Pose startPos = new Pose(0, 0, 0); // Initial pose
    private double extendoLength = 0.0; // Current length of the extendo
    private final double lengthIncrement = 0.1; // Increment for changing the extendo length

    // Button toggles for manual control
    private final ButtonToggle buttonY = new ButtonToggle();
    private final ButtonToggle buttonA = new ButtonToggle();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.follower.setStartingPose(startPos);
    }

    @Override
    public void loop() {
        robot.follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        robot.update(); // might have to swap

        // Manual control for extendo
        if (buttonY.isClicked(gamepad2.y)) {
            extendoLength += lengthIncrement; // Increment the length
        }
        if (buttonA.isClicked(gamepad2.a)) {
            extendoLength -= lengthIncrement; // Decrement the length
        }
        extendoLength = Math.max(0, extendoLength); // Prevent negative lengths

        // Apply the target length to the extendo
        robot.extendo.setTargetLength(extendoLength);

        // Telemetry feedback
        telemetry.addData("X", robot.follower.getPose().getX());
        telemetry.addData("Y", robot.follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("Extendo Length", extendoLength);
        telemetry.update();
    }
}
