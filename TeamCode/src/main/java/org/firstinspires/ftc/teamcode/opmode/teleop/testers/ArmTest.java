package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.Claw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;

@Config
@TeleOp
public class ArmTest extends LinearOpMode {
    public static double armPosition = 0.075;
    public static double armWristPosition = 0;
    public static double clawPosition = 0;
    public static double clawSwivelPosition = 0;

    @Override
    public void runOpMode() {
        OuttakeArm outtakeArm = new OuttakeArm(this.hardwareMap);
        outtakeArm.init();

        Claw claw = new Claw(this.hardwareMap);
        claw.init();

        waitForStart();
        while (opModeIsActive()) {
            outtakeArm.setArmPosition(armPosition);
            outtakeArm.setWristPosition(armWristPosition);
            claw.setClawPosition(clawPosition);
            claw.setSwivelPosition(clawSwivelPosition);
        }
    }
}
