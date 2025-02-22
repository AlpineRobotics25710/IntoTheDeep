package org.firstinspires.ftc.teamcode.opmode.teleop.prod;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.GrabOffWallCommand;
import org.firstinspires.ftc.teamcode.robot.commands.HighBasketCommand;
import org.firstinspires.ftc.teamcode.robot.commands.HighChamberCommand;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.OuttakeIntermediateCommand;
import org.firstinspires.ftc.teamcode.robot.commands.RetractNoTransfer;
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

@Config
@TeleOp(group = "production")
public class OneDriverTeleOp extends LinearOpMode {
    private static final Gamepad.LedEffect outtakeArmLedEffect = new Gamepad.LedEffect.Builder().addStep(1.0, 0.0, 0.0, 1000).build();
    private static final Gamepad.LedEffect regularLedEffect = new Gamepad.LedEffect.Builder().addStep(0.0, 0.75, 0.0, Gamepad.LED_DURATION_CONTINUOUS).build();
    public static boolean robotCentric = true;
    private static Pose startPose = new Pose(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryUtil.setup(telemetry);
        Robot robot = new Robot(hardwareMap, false);

        GamepadEx gp1 = new GamepadEx(gamepad1);

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
                new GrabOffWallCommand(robot).schedule();
            } else if (robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.SUBMERSIBLE_OUTTAKE_BACK) {
                new GrabOffWallCommand(robot).schedule();
            }
            gamepad1.rumble(0.75, 0.75, 500);
            gamepad1.runLedEffect(outtakeArmLedEffect);
            Log.i("TeamCode", "Outtake arm has moved. This is through the Android Logcat cuz Prathyush is so cool 😎.");
        });
        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new HighChamberCommand(robot));
        //gp2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new LowChamberCommand(robot, false));
        gp1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ClawToggleCommand(robot));

        // Extendo commands
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new IntakeCommand(robot, Extendo.MAX_LENGTH, IntakeArm.IntakeArmState.INTERIM));
        // RAJVEER TRANSFER RETRACTS EVERYTHING AND SO DOES GRAB OFF WALL
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new SequentialCommandGroup(
                new TransferCommand(robot),
                new InstantCommand(() -> gamepad1.rumble(0.5, 0.5, 500))
        ));


        // Outtake slides commands
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new HighBasketCommand(robot));
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new RetractNoTransfer(robot));

        // Intake commands
        gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            // TelemetryUtil.addData("BUTTON X", "PRESSED");
            if (robot.intakeArm.currentState == IntakeArm.IntakeArmState.INTERIM) {
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INTAKE).schedule();
            } else {
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INTERIM).schedule();
            }
        });

        // Reset imu for field centric
        gp1.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new InstantCommand(() -> robot.follower.setHeadingOffset(0))
        );

        while (opModeInInit()) {
            //robot.extendoRight.setPower(-0.35);
            robot.loop();
            TelemetryUtil.addData("extendo base pos", Extendo.BASE_POS);
            TelemetryUtil.addData("intake arm pos", robot.intakeArm.getArmPosition());
            TelemetryUtil.addData("intake wrist pos", robot.intakeArm.getWristPosition());
            TelemetryUtil.addData("Current Arm State", robot.intakeArm.currentState);
            TelemetryUtil.update();
        }

        waitForStart();

        if (isStarted()) {
            robot.follower.startTeleopDrive();
            gamepad1.runLedEffect(regularLedEffect);
        }

        while (!isStopRequested() && opModeIsActive()) {
            robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, robotCentric);

            if ((gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) && robot.follower.isBusy()) {
                robot.follower.breakFollowing();
                robot.follower.startTeleopDrive();
            }

            robot.loop();
            TelemetryUtil.update();
        }
        robot.end();
    }
}
