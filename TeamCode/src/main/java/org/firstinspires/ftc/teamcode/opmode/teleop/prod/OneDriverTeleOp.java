package org.firstinspires.ftc.teamcode.opmode.teleop.prod;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.GrabOffWallCommand;
import org.firstinspires.ftc.teamcode.robot.commands.HighBasketCommand;
import org.firstinspires.ftc.teamcode.robot.commands.HighChamberCommand;
import org.firstinspires.ftc.teamcode.robot.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.robot.commands.LowBasketCommand;
import org.firstinspires.ftc.teamcode.robot.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeEndCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.teleopcommands.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@TeleOp
public class OneDriverTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryUtil.setup(telemetry);
        Robot robot = new Robot(hardwareMap, false, true);

        GamepadEx gp1 = new GamepadEx(gamepad1);

        // Active intake controls
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new IntakeEndCommand(robot, IntakeEnd.ActiveState.FORWARD));
        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new IntakeEndCommand(robot, IntakeEnd.ActiveState.OFF));
        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new IntakeEndCommand(robot, IntakeEnd.ActiveState.REVERSED));
        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(new IntakeEndCommand(robot, IntakeEnd.ActiveState.OFF));

        // Outtake commands
        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            if(robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.WALL_INTAKE_FRONT){
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.INTERMEDIATE).schedule();
            } else if(robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.INTERMEDIATE){
                new HighChamberCommand(robot, false).schedule();
            }
            else if(robot.outtakeArm.getCurrentState() == OuttakeArm.OuttakeArmState.OUTTAKE_BACK){
                new GrabOffWallCommand(robot).schedule();
            }
        });
        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new HighChamberCommand(robot, false));

        //gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new LowChamberCommand(robot, false));
        gp1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ClawToggleCommand(robot));

        // Extendo commands
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new IntakeCommand(robot));
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new TransferCommand(robot));

        // Outtake slides commands
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new HighBasketCommand(robot, false));
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new LowBasketCommand(robot, false));

        robot.follower.startTeleopDrive();
        while (opModeInInit()) {
            robot.extendoRight.setPower(-0.3);
            TelemetryUtil.addData("extendo base pos", Extendo.BASE_POS);
            TelemetryUtil.update();
        }

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            robot.loop();
            TelemetryUtil.update();
        }
        robot.end();
    }
}
