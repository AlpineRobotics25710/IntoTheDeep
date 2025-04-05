package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DoCommand extends CommandBase {
    protected final Command toDo;
    private final double seconds;
    protected ElapsedTime timer;

    /**
     * Wraps the runnable in an {@link InstantCommand}
     * @param toDo actions to execute in the given timeframe
     * @param seconds how long it will take to execute the given actions
     * @see InstantCommand
     */
    public DoCommand(Runnable toDo, double seconds) {
        this.toDo = new InstantCommand(toDo);
        this.seconds = seconds;
    }

    public DoCommand(Command toDo, double seconds) {
        this.toDo = toDo;
        this.seconds = seconds;
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.reset();
        toDo.initialize();
    }

    @Override
    public void execute() {
        toDo.execute();
    }

    @Override
    public void end(boolean interrupted) {
        toDo.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= this.seconds && toDo.isFinished();
    }
}
