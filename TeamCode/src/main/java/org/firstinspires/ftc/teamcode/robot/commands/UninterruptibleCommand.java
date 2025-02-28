package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

/**
 * Schedules a Command as uninterruptible
 * @author Arush - FTC 23511
 */
public class UninterruptibleCommand extends CommandBase {
    private final Command command;
    public UninterruptibleCommand(Command command) {
        this.command = command;
    }

    @Override
    public void initialize() {
        command.schedule(false);
    }

    @Override
    public boolean isFinished() {
        return !CommandScheduler.getInstance().isScheduled(command);
    }
}