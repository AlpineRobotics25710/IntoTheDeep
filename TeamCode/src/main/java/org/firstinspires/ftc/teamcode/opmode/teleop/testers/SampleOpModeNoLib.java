package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;

/**
 * A small sample opmode that tests the robot and the set state methods. Does not use FTC Lib commands
 */
@TeleOp
public class SampleOpModeNoLib extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, false, true);

        waitForStart();

        while (opModeIsActive()) {
            robot.loop();

            // Intake commands
            if (gamepad1.a) {
                robot.intakeArm.setState(IntakeArm.IntakeArmState.INTAKE);
            }
            if (gamepad1.b) {
                robot.intakeArm.setState(IntakeArm.IntakeArmState.TRANSFER);
            }
            if (gamepad1.x) {
                robot.intakeEnd.setState(IntakeEnd.ActiveState.ON);
            }
            if (gamepad1.y) {
                robot.intakeEnd.setState(IntakeEnd.ActiveState.OFF);
            }
        }
    }
}
