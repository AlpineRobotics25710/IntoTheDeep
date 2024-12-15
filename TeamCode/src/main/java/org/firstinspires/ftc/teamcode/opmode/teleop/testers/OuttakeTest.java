package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.control.gamepad.CustomGamepad;
import org.firstinspires.ftc.teamcode.robot.control.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.control.drivetrain.DrivetrainBuilder;

@TeleOp
public class OuttakeTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this.hardwareMap);
        RobotConstants.mode = RobotConstants.Mode.TESTING;

        CustomGamepad gp1 = new CustomGamepad(gamepad1);
        gp1.leftStick().setSensitivity(1.0);
        gp1.rightStick().setSensitivity(1.0);
        gp1.a().setActionFlag(() -> gp1.a().isClicked());
        //gp1.getA().setAction(robot.claw.setClawPosition(robot.claw););
        gp1.b().setActionFlag(() -> gp1.b().isClicked());
        //gp1.getB().setAction(robot.claw::close);
        gp1.x().setActionFlag(() -> gp1.x().isClicked());
        //gp1.getX().setAction(() -> robot.outtakeSlides.setTargetPosition(robot.outtakeSlides.getTargetPosition() + 10));
        gp1.y().setActionFlag(() -> gp1.y().isClicked());
        //gp1.getY().setAction(() -> robot.outtakeSlides.setTargetPosition(robot.outtakeSlides.getTargetPosition() - 10));
        gp1.dpadDown().setActionFlag(() -> gp1.dpadDown().isClicked());
       // gp1.getDpadDown().setAction(() -> robot.outtakeSlides.setManualMode(!robot.outtakeSlides.manualMode));
        gp1.dpadLeft().setActionFlag(() -> gp1.dpadLeft().isClicked());
       // gp1.getDpadUp().setAction(() -> robot.outtakeSlides.setTargetPosition(700));

        Drivetrain drivetrain = new DrivetrainBuilder()
                .setType(DrivetrainBuilder.DrivetrainType.ROBOT_CENTRIC_MECANUM)
                .setGamepad(gp1)
                .setMotors(robot.frontLeftMotor, robot.backLeftMotor, robot.frontRightMotor, robot.backRightMotor)
                .setStrafingMultiplier(1.1)
                .build();

        while(opModeInInit()) {
           // robot.outtakeSlides.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double manualSlidesPower = Math.pow(gp1.leftStick().getY(), 3) / 1.5;
           // robot.outtakeSlides.moveSlides(manualSlidesPower);

            drivetrain.update();
            gp1.update();
            robot.update();

            telemetry.addData("Left stick y: ", -gamepad1.left_stick_y);
            telemetry.addData("Left stick X: ", gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}
