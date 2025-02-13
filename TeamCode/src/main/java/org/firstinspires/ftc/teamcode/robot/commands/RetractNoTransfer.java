package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeArmCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeClawCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.OuttakeSlidesCommand;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand.SwivelCommand;
import org.firstinspires.ftc.teamcode.robot.mechanisms.intake.IntakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeClaw;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeSlides;

public class RetractNoTransfer extends SequentialCommandGroup {
    public RetractNoTransfer(Robot robot) {
        super(
                new OuttakeArmCommand(robot, OuttakeArm.OuttakeArmState.TRANSFER),
                new SwivelCommand(robot, OuttakeClaw.OuttakeSwivelState.SIDEWAYS),
                new OuttakeClawCommand(robot, OuttakeClaw.OuttakeClawState.OPEN),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    if(robot.outtakeSlides.getCurrentPosition() > 500){
                        new WaitCommand(500);
                    }}
                ),
                new IntakeRetractCommand(robot, IntakeArm.IntakeArmState.INTERIM),
                new OuttakeSlidesCommand(robot, OuttakeSlides.TRANSFER_POS)
        );
    }
}
