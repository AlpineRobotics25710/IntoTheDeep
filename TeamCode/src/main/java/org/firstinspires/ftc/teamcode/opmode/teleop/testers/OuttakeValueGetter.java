package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
@TeleOp
public class OuttakeValueGetter extends LinearOpMode {
    public static double armPos;
    public static double wristPos;
    public static double clawPos;
    public static double swivelPos;
    public static double slidesTargetPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryUtil.setup(telemetry);
        Robot robot = new Robot(hardwareMap, false);
        waitForStart();

        while (opModeIsActive()) {
            robot.outtakeArm.setArmPosition(armPos);
            robot.outtakeArm.setWristPosition(wristPos);
            robot.outtakeClaw.setClawPosition(clawPos);
            robot.outtakeClaw.setSwivelPosition(swivelPos);
            robot.outtakeSlides.setTargetPosition(slidesTargetPos);

            robot.loop();
            TelemetryUtil.update();
        }
    }
}
