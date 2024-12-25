package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
@TeleOp
public class IntakeTest extends LinearOpMode {
    public static double armPos = 0.575;
    public static double armWristPos = 0.8;
    public static double clawPos = IntakeClaw.CLAW_OPEN_POS;
    public static double clawSwivelPos = 0.0;


    @Override
    public void runOpMode() {
        IntakeArm intakeArm = new IntakeArm();
        IntakeClaw intakeClaw = new IntakeClaw();
        intakeArm.init();
        intakeClaw.init();

        TelemetryUtil.setup();

        waitForStart();

        while (opModeIsActive()) {
            intakeArm.setArmPosition(armPos);
            intakeArm.setWristPosition(armWristPos);
            intakeClaw.setClawPosition(clawPos);
            intakeClaw.setSwivelPosition(clawSwivelPos);

            TelemetryUtil.packet.put("Arm pos", intakeArm.getArmPosition());
            TelemetryUtil.packet.put("Wrist pos", intakeArm.getSwivelPosition());
            TelemetryUtil.packet.put("Claw pos", intakeClaw.getClawPosition());
            TelemetryUtil.packet.put("Swivel pos", intakeClaw.getSwivelPosition());
            TelemetryUtil.sendTelemetry();
        }
    }
}
