package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeClaw;

@Config
@TeleOp
public class IntakeTest extends LinearOpMode {
    public static double armPos = 0.575;
    public static double armWristPos = .8;
    public static double clawPos = IntakeClaw.CLAW_OPEN_POS;
    public static double clawSwivelPos = 0.0;


    @Override
    public void runOpMode() {
        IntakeArm intakeArm = new IntakeArm(hardwareMap);
        IntakeClaw intakeClaw = new IntakeClaw(hardwareMap);
        intakeArm.init();
        intakeClaw.init();

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            intakeArm.setArmPosition(armPos);
            intakeArm.setWristPosition(armWristPos);
            intakeClaw.setClawPosition(clawPos);
            intakeClaw.setSwivelPosition(clawSwivelPos);

            telemetry.addData("Arm pos", intakeArm.getArmPosition());
            telemetry.addData("Wrist pos", intakeArm.getSwivelPosition());
            telemetry.addData("Claw pos", intakeClaw.getClawPosition());
            telemetry.addData("Swivel pos", intakeClaw.getSwivelPosition());
            telemetry.update();

        }
    }
}
