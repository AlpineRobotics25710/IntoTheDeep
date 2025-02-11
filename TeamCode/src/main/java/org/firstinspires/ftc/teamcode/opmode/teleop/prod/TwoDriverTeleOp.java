package org.firstinspires.ftc.teamcode.opmode.teleop.prod;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.GrabOffWallCommand;
import org.firstinspires.ftc.teamcode.robot.commands.HighBasketCommand;
import org.firstinspires.ftc.teamcode.robot.commands.HighChamberCommand;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeRetractCommand;
import org.firstinspires.ftc.teamcode.robot.commands.LowBasketCommand;
import org.firstinspires.ftc.teamcode.robot.commands.OuttakeIntermediateCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TeleOpInitializeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeEndCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.teleopcommands.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@TeleOp(group = "production")
public class TwoDriverTeleOp extends LinearOpMode {

    public static boolean robotCentric = true;
    private static Pose startPose = new Pose(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryUtil.setup(telemetry);
        Robot robot = new Robot(hardwareMap, false, false);

        GamepadEx gp1 = new GamepadEx(gamepad2);

        robot.follower.setStartingPose(startPose);

        // Active intake controls
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD));
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new IntakeEndCommand(robot, IntakeEnd.ActiveState.OFF));
        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new IntakeEndCommand(robot, IntakeEnd.ActiveState.REVERSED));
        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(new IntakeEndCommand(robot, IntakeEnd.ActiveState.OFF));

        // Outtake commands

        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            if (robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.INIT || robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.TRANSFER) {
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.WALL_INTAKE_FRONT).schedule();
            } else if (robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.WALL_INTAKE_FRONT) {
                new OuttakeIntermediateCommand(robot).schedule();
            } else if (robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.INTERMEDIATE) {
                new HighChamberCommand(robot).schedule();
            } else if (robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.OUTTAKE_BACK) {
                new GrabOffWallCommand(robot).schedule();
            }
        });
        //gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new HighChamberCommand(robot));
        //gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new LowChamberCommand(robot, false));
        gp1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ClawToggleCommand(robot));

        // Extendo commands
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new IntakeCommand(robot, IntakeArm.IntakeArmState.INTERIM)
        );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new TeleOpInitializeCommand(robot, false) //PRATHYUSH ITS TO RETRACT EVERYTHING DONT DELETE IT
        );

        // Outtake slides commands
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new TransferCommand(robot),
                        new WaitCommand(750),
                        new ClawToggleCommand(robot),
                        new WaitCommand(250),
                        new HighBasketCommand(robot)
                )
        );
        //gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new LowBasketCommand(robot, false));
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new IntakeRetractCommand(robot, IntakeArm.IntakeArmState.INTERIM)
        );
        // Intake commands
        gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            // TelemetryUtil.addData("BUTTON X", "PRESSED");
            if (robot.intakeArm.currentState == IntakeArm.IntakeArmState.INTERIM) {
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INTAKE).schedule();
            } else {
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INTERIM).schedule();
            }
        });


        while (opModeInInit()) {
            robot.extendoRight.setPower(-0.35);
            TelemetryUtil.addData("extendo base pos", Extendo.BASE_POS);
            TelemetryUtil.addData("intake arm pos", robot.intakeArm.getArmPosition());
            TelemetryUtil.addData("intake wrist pos", robot.intakeArm.getWristPosition());
            TelemetryUtil.addData("Current Arm State", robot.intakeArm.currentState);
            TelemetryUtil.update();
        }

        waitForStart();

        if (isStarted()) {
            robot.follower.startTeleopDrive();
        }

        while (!isStopRequested() && opModeIsActive()) {
            robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, robotCentric);
            robot.loop();
            TelemetryUtil.update();
        }
        robot.end();
    }
}
