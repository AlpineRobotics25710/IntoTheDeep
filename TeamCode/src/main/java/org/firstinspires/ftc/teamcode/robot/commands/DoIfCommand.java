package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class DoIfCommand extends CommandBase {
    private final double seconds;
    protected ElapsedTime timer;
    protected Command doOnTrue;
    protected Command doOnFalse;
    protected BooleanSupplier condition;
    protected Command toExecute;

    public DoIfCommand(Runnable doOnTrue, Runnable doOnFalse, BooleanSupplier condition, double seconds) {
        this.doOnTrue = new InstantCommand(doOnTrue);
        this.doOnFalse = new InstantCommand(doOnFalse);
        this.condition = condition;
        this.seconds = seconds;
    }

    public DoIfCommand(Command doOnTrue, Command doOnFalse, BooleanSupplier condition, double seconds) {
        this.doOnTrue = doOnTrue;
        this.doOnFalse = doOnFalse;
        this.condition = condition;
        this.seconds = seconds;
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.reset();

        // Decides what command should be run based on the state of the condition at the time of initialization
        if (condition.getAsBoolean()) {
            toExecute = doOnTrue;
        } else {
            toExecute = doOnFalse;
        }

        toExecute.initialize();
    }

    @Override
    public void execute() {
        toExecute.execute();
    }

    @Override
    public void end(boolean interrupted) {
        toExecute.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= this.seconds && toExecute.isFinished();
    }
}
