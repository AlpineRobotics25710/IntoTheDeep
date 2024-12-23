package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeClaw;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

public class CommandTest extends CommandOpMode {
    private IntakeClaw intakeClaw;
    private GamepadEx gp1;

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            TelemetryUtil.packet.put("Claw pos", intakeClaw.getClawPosition());
            TelemetryUtil.packet.put("Swivel pos", intakeClaw.getSwivelPosition());
            TelemetryUtil.sendTelemetry();
        }
    }

    @Override
    public void initialize() {
        intakeClaw = new IntakeClaw(hardwareMap);
        intakeClaw.init();

        InstantCommand closeClaw = new InstantCommand(() -> intakeClaw.setClawPosition(IntakeClaw.CLAW_OPEN_POS));

        gp1 = new GamepadEx(gamepad1);
        gp1.getGamepadButton(GamepadKeys.Button.A).whenReleased(closeClaw);

        TelemetryUtil.setup();

        schedule(closeClaw);
        register(intakeClaw);
    }
}
