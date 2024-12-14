package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.utils.control.gamepad.CustomGamepad;
import org.firstinspires.ftc.teamcode.robot.utils.control.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.utils.control.drivetrain.DrivetrainBuilder;

@TeleOp
public class OuttakeTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this.hardwareMap);
        RobotConstants.mode = RobotConstants.Mode.TESTING;

        CustomGamepad gp1 = new CustomGamepad(gamepad1);
        gp1.getLeftStick().setSensitivity(1.0);
        gp1.getRightStick().setSensitivity(1.0);
        gp1.getA().setActionFlag(() -> gp1.getA().isClicked());
        gp1.getA().setAction(robot.claw::open);
        gp1.getB().setActionFlag(() -> gp1.getB().isClicked());
        gp1.getB().setAction(robot.claw::close);
        gp1.getX().setActionFlag(() -> gp1.getX().isClicked());
        gp1.getX().setAction(() -> robot.outtakeSlides.setTargetPosition(robot.outtakeSlides.getTargetPosition() + 10));
        gp1.getY().setActionFlag(() -> gp1.getY().isClicked());
        gp1.getY().setAction(() -> robot.outtakeSlides.setTargetPosition(robot.outtakeSlides.getTargetPosition() - 10));
        gp1.getDpadDown().setActionFlag(() -> gp1.getDpadDown().isClicked());
        gp1.getDpadDown().setAction(() -> robot.outtakeSlides.setManualMode(!robot.outtakeSlides.manualMode));
        gp1.getDpadLeft().setActionFlag(() -> gp1.getDpadLeft().isClicked());
        gp1.getDpadUp().setAction(() -> robot.outtakeSlides.setTargetPosition(700));

        Drivetrain drivetrain = new DrivetrainBuilder()
                .setType(DrivetrainBuilder.DrivetrainType.ROBOT_CENTRIC_MECANUM)
                .setGamepad(gp1)
                .setMotors(robot.frontLeftMotor, robot.backLeftMotor, robot.frontRightMotor, robot.backRightMotor)
                .setStrafingMultiplier(1.1)
                .build();

        while(opModeInInit()) {
            robot.outtakeSlides.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double manualSlidesPower = Math.pow(gp1.getLeftStick().getY(), 3) / 1.5;
            robot.outtakeSlides.moveSlides(manualSlidesPower);

            drivetrain.update();
            gp1.update();
            robot.update();

            telemetry.addData("Left stick y: ", -gamepad1.left_stick_y);
            telemetry.addData("Left stick X: ", gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}
