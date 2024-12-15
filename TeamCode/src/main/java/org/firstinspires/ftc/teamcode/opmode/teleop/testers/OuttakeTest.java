package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;

@Config
@TeleOp
public class OuttakeTest extends LinearOpMode {
    public static double armPosition = 0.075;
    public static double armWristPosition = 0;
    public static double clawPosition = 0;
    public static double clawSwivelPosition = 0;

    @Override
    public void runOpMode() {
        OuttakeArm outtakeArm = new OuttakeArm(this.hardwareMap);
        outtakeArm.init();

        OuttakeClaw outtakeClaw = new OuttakeClaw(this.hardwareMap);
        outtakeClaw.init();

        waitForStart();
        while (opModeIsActive()) {
            outtakeArm.setArmPosition(armPosition);
            outtakeArm.setWristPosition(armWristPosition);
            outtakeClaw.setClawPosition(clawPosition);
            outtakeClaw.setSwivelPosition(clawSwivelPosition);
        }
    }
}
