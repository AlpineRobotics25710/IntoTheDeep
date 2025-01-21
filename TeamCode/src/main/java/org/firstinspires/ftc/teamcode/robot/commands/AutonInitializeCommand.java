package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.ExtendoCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.IntakeEndCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.Extendo;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeEnd;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

public class AutonInitializeCommand extends ParallelCommandGroup {
    public AutonInitializeCommand(Robot robot, boolean manualMode) {
        /*if (!manualMode) {
            addCommands(new ExtendoCommand(robot, 0));
        }*/
        addCommands(
                new ExtendoCommand(robot, Extendo.BASE_POS),
                new IntakeArmCommand(robot, IntakeArm.IntakeArmState.INIT),
                new IntakeEndCommand(robot, IntakeEnd.ActiveState.OFF),
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.INIT),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.CLOSED),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.SIDEWAYS),
                new OuttakeSlidesCommand(robot, OuttakeSlides.TRANSFER_POS)
        );
    }
}
