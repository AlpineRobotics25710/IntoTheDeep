//package org.firstinspires.ftc.teamcode.opmode.teleop.testers;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//
//import org.firstinspires.ftc.teamcode.robot.Robot;
//import org.firstinspires.ftc.teamcode.robot.commands.IntakeCommand;
//import org.firstinspires.ftc.teamcode.robot.commands.TransferCommand;
//import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;
//
//public class CommandTest extends CommandOpMode {
//    private final Robot robot = Robot.getInstance();
//
//    @Override
//    public void initialize() {
//        super.reset();
//        robot.init(hardwareMap);
//        register(robot.intakeArm, robot.extendo, robot.intakeClaw, robot.outtakeArm, robot.outtakeClaw, robot.outtakeSlides);
//
//        // Gamepad
//        GamepadEx gp1 = new GamepadEx(gamepad1);
//
//        // Test the intake and retract commands
//        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new IntakeCommand());
//        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(TransferCommand.intakeTransferCommand);
//    }
//
//    @Override
//    public void run() {
//        super.run();
//        TelemetryUtil.packet.put("Current Arm State", robot.intakeArm.currentState);
//        TelemetryUtil.packet.put("Current Claw State", robot.intakeClaw.getClawState());
//        TelemetryUtil.packet.put("Current Swivel State", robot.intakeClaw.getSwivelState());
//      //  TelemetryUtil.packet.put("Current Extendo Position", robot.extendo.getEncoderPosition());
//     //   TelemetryUtil.packet.put("Extendo target position", robot.extendo.getTargetPosition());
//        TelemetryUtil.sendTelemetry();
//    }
//}
