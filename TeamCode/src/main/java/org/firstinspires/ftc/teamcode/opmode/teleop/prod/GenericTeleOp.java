package org.firstinspires.ftc.teamcode.opmode.teleop.prod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(group = "prod")
public class GenericTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this.hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
        }
    }
}
