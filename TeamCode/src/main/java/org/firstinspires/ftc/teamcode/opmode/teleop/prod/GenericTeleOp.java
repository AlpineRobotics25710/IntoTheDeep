package org.firstinspires.ftc.teamcode.opmode.teleop.prod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.control.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.control.drivetrain.DrivetrainBuilder;
import org.firstinspires.ftc.teamcode.robot.control.gamepad.CustomGamepad;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeClaw;

@TeleOp(group = "prod")
public class GenericTeleOp extends LinearOpMode {
    private CustomGamepad gp1;
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this.hardwareMap);
        gp1 = new CustomGamepad(gamepad1);
        drivetrain = new DrivetrainBuilder()
                .setType(DrivetrainBuilder.DrivetrainType.ROBOT_CENTRIC_MECANUM)
                .setGamepad(gp1)
                .setMotors(robot.frontLeftMotor, robot.backLeftMotor, robot.frontRightMotor, robot.backRightMotor)
                .setStrafingMultiplier(1.1)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("cross is clicked", gp1.cross().isPressed());
            if (gp1.cross().isClicked()) {
                robot.intakeArm.ascend();
                robot.intakeClaw.setClawPosition(IntakeClaw.CLAW_CLOSED_POS);
                robot.intakeClaw.setSwivelPosition(IntakeClaw.SWIVEL_ASCENT_POS);
                telemetry.addData("Intake", "ascending");
            }
            if (gp1.square().isClicked()) {
                robot.intakeArm.intake();
                robot.intakeClaw.setClawPosition(IntakeClaw.CLAW_OPEN_POS);
                robot.intakeClaw.setSwivelPosition(IntakeClaw.SWIVEL_INTAKE_POS);
                telemetry.addData("Intake", "intaking");
            }
            if (gp1.circle().isClicked()) {
                robot.intakeArm.transfer();
                robot.intakeClaw.setClawPosition(IntakeClaw.CLAW_CLOSED_POS);
                robot.intakeClaw.setSwivelPosition(IntakeClaw.SWIVEL_TRANSFER_POS);
                telemetry.addData("Intake", "transferring");
            }
            update();
        }
    }

    public void update() {
        gp1.update();
        drivetrain.update();
        //robot.update();
        telemetry.update();
    }
}
