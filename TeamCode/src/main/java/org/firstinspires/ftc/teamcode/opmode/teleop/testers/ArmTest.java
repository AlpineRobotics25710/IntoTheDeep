package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.Arm;

@Config
@TeleOp
public class ArmTest extends LinearOpMode {
    public static double position = 0.3;

    @Override
    public void runOpMode() {
        Arm arm = new Arm(this.hardwareMap);
        arm.init();

        waitForStart();
        while (opModeIsActive()) {
            arm.setArmPosition(position);
        }
    }
}
