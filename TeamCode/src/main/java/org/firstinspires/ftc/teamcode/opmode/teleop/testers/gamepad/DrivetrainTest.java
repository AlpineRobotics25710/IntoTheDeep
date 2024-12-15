package org.firstinspires.ftc.teamcode.opmode.teleop.testers.gamepad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.control.gamepad.CustomGamepad;
import org.firstinspires.ftc.teamcode.robot.control.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.control.drivetrain.DrivetrainBuilder;

@TeleOp
public class DrivetrainTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this.hardwareMap);

        CustomGamepad gp1 = new CustomGamepad(gamepad1);
        gp1.leftStick().setSensitivity(1.0);
        gp1.rightStick().setSensitivity(1.0);
        gp1.a().setActionFlag(() -> gp1.a().isClicked());
        gp1.a().setAction(() -> telemetry.addData("A is clicked", "through lambda"));
        gp1.b().setAction(() -> gp1.b().isClicked());
        gp1.b().setAction(() -> telemetry.addData("B is clicked", "through lambda"));

        Drivetrain drivetrain = new DrivetrainBuilder()
                .setType(DrivetrainBuilder.DrivetrainType.ROBOT_CENTRIC_MECANUM)
                .setGamepad(gp1)
                .setMotors(robot.frontLeftMotor, robot.backLeftMotor, robot.frontRightMotor, robot.backRightMotor)
                .setStrafingMultiplier(1.1)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drivetrain.update();
            gp1.update();
            robot.update();

            telemetry.addData("Left stick y: ", -gamepad1.left_stick_y);
            telemetry.addData("Left stick X: ", gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}
