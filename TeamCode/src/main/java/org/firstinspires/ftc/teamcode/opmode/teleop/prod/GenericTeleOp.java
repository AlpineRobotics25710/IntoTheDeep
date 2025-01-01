package org.firstinspires.ftc.teamcode.opmode.teleop.prod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "prod")
public class GenericTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {

            //robot.update();
        }
    }
}
