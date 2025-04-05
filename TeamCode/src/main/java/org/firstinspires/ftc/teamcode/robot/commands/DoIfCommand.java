package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class DoIfCommand extends CommandBase {
    private final double seconds;
    protected ElapsedTime timer;
    protected Runnable doOnTrue;
    protected Runnable doOnFalse;
    protected BooleanSupplier condition;

    public DoIfCommand(Runnable doOnTrue, Runnable doOnFalse, BooleanSupplier condition, double seconds) {
        this.doOnTrue = doOnTrue;
        this.doOnFalse = doOnFalse;
        this.condition = condition;
        this.seconds = seconds;
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.reset();
        if (condition.getAsBoolean()) {
            doOnTrue.run();
        } else {
            doOnFalse.run();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= this.seconds;
    }
}
