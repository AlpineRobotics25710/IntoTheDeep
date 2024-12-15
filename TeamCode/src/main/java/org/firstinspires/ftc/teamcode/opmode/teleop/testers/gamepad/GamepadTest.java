package org.firstinspires.ftc.teamcode.opmode.teleop.testers.gamepad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.control.gamepad.CustomGamepad;

public class GamepadTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        CustomGamepad gp1 = new CustomGamepad(gamepad1);
        gp1.a().setActionFlag(() -> gp1.a().isPressed());
        gp1.a().setAction(() -> telemetry.addData("Button A", "pressed"));

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Button A", gp1.a().isClicked());

            gp1.update();
            telemetry.update();
        }
    }
}
