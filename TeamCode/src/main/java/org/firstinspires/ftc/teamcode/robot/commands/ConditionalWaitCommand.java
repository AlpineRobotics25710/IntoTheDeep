package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class ConditionalWaitCommand extends CommandBase {
    private final double seconds;
    protected ElapsedTime timer;
    protected BooleanSupplier condition;

    /**
     * Waits for given duration if the condition is true
     *
     * @param condition the condition whether to wait or not
     * @param seconds the duration to wait (if condition is true)
     */
    public ConditionalWaitCommand(BooleanSupplier condition, double seconds) {
        this.condition = condition;
        this.seconds = seconds;
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        if (condition.getAsBoolean()) {
            return timer.seconds() >= this.seconds;
        }

        return true;
    }
}
