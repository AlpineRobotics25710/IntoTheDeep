package org.firstinspires.ftc.teamcode.opmode.teleop.prod;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.HighBasketCommand;
import org.firstinspires.ftc.teamcode.robot.commands.LowChamberCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeEndCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@TeleOp
public class GenericTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryUtil.setup(telemetry);
        Robot robot = new Robot(hardwareMap, false, false);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INTAKE)
        );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.TRANSFER)
        );

        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD)
        );

        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                new IntakeEndCommand(robot, IntakeEnd.ActiveState.OFF)
        );

        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new IntakeEndCommand(robot, IntakeEnd.ActiveState.REVERSED)
        );

        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
                new IntakeEndCommand(robot, IntakeEnd.ActiveState.OFF)
        );

        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.TRANSFER));

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new ExtendoCommand(robot, Extendo.BASE_POS));
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new ExtendoCommand(robot, Extendo.MAX_LENGTH));

        DcMotor fL = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        DcMotor fR = hardwareMap.get(DcMotor.class, "frontRightMotor");
        DcMotor bL = hardwareMap.get(DcMotor.class, "backLeftMotor");
        DcMotor bR = hardwareMap.get(DcMotor.class, "backRightMotor");

        fL.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            fL.setPower(frontLeftPower);
            bL.setPower(backLeftPower);
            fR.setPower(frontRightPower);
            bR.setPower(backRightPower);
            robot.loop();
            // Check if neither bumper is pressed

            TelemetryUtil.update();
        }
        CommandScheduler.getInstance().reset();
        robot.clearHubCache();
    }
}
