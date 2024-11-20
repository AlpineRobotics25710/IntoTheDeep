package org.firstinspires.ftc.teamcode.robot.utils.control.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Allows you to test for controller drift. To use this class, first run this opmode on your robot.
 * Then, make sure not to touch your gamepad and move the joysticks. If the joystick values being
 * displayed by the telemetry aren't equal to zero, what is being displayed is your drift. You should
 * set this as your error when creating your CustomGamepad and these values will be deadzoned.
 *
 * @author Prathyush Yeturi
 */
public class DriftTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit()) {
            telemetry.addData("x-value of left stick", gamepad1.left_stick_x);
            telemetry.addData("y-value of left stick", gamepad1.left_stick_y);
            telemetry.addData("y-value of right stick", gamepad1.right_stick_x);
            telemetry.addData("x-value of right stick", gamepad1.right_stick_y);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("x-value of left stick", gamepad1.left_stick_x);
            telemetry.addData("y-value of left stick", gamepad1.left_stick_y);
            telemetry.addData("y-value of right stick", gamepad1.right_stick_x);
            telemetry.addData("x-value of right stick", gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}
