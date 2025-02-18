package org.firstinspires.ftc.teamcode.opmode.teleop.prod;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
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
public class TwoDriverTeleOp extends LinearOpMode {
    public static boolean robotCentric = true;
    private static Pose startPose = new Pose(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryUtil.setup(telemetry);
        Robot robot = new Robot(hardwareMap, false);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        robot.follower.setStartingPose(startPose);

        // Active intake controls
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD));
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new IntakeEndCommand(robot, IntakeEnd.ActiveState.OFF));
        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new IntakeEndCommand(robot, IntakeEnd.ActiveState.REVERSED));
        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(new IntakeEndCommand(robot, IntakeEnd.ActiveState.OFF));

        // Outtake commands
        gp2.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            if (robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.INIT || robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.TRANSFER) {
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.WALL_INTAKE_FRONT).schedule();
            } else if (robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.WALL_INTAKE_FRONT) {
                new OuttakeIntermediateCommand(robot).schedule();
            } else if (robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.INTERMEDIATE) {
                new GrabOffWallCommand(robot).schedule();
            } else if (robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.SUBMERSIBLE_OUTTAKE_BACK) {
                new GrabOffWallCommand(robot).schedule();
            }
        });
        gp2.getGamepadButton(GamepadKeys.Button.B).whenPressed(new HighChamberCommand(robot));
        //gp2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new LowChamberCommand(robot, false));
        gp2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ClawToggleCommand(robot));

        // Extendo commands
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new IntakeCommand(robot, Extendo.MAX_LENGTH, IntakeArm.IntakeArmState.INTERIM));
        // RAJVEER TRANSFER RETRACTS EVERYTHING AND SO DOES GRAB OFF WALL
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new TransferCommand(robot));

        gp2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new RetractNoTransfer(robot)
        );

        gp2.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new InstantCommand(() -> robot.follower.setHeadingOffset(0))
        );
        // Outtake slides commands
        //gp2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new LowBasketCommand(robot, false));
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new HighBasketCommand(robot));

        // Intake commands
        gp2.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            // TelemetryUtil.addData("BUTTON X", "PRESSED");
            if (robot.intakeArm.currentState == IntakeArm.IntakeArmState.INTERIM) {
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INTAKE).schedule();
            } else {
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INTERIM).schedule();
            }
        });

        if(Math.abs(gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) >0){
            new IntakeCommand(robot, Extendo.MAX_LENGTH, IntakeArm.IntakeArmState.INTERIM);
        }

        gp1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new IntakeCommand(robot, Extendo.MAX_LENGTH, IntakeArm.IntakeArmState.INTERIM)
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
            new GrabOffWallCommand(robot).schedule();
        }

        while (!isStopRequested() && opModeIsActive()) {
            //robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, robotCentric);

            if(Math.abs(gamepad1.left_stick_x) >= 0.05 || Math.abs(gamepad1.left_stick_y) >= 0.05 && Math.abs(gamepad1.right_stick_x) >= 0.05)
                robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, robotCentric);
            else{
                robot.follower.holdPoint(robot.follower.getPose()); //unpushable robot
            }

            robot.loop();
            TelemetryUtil.update();
        }
        robot.end();
    }
}
