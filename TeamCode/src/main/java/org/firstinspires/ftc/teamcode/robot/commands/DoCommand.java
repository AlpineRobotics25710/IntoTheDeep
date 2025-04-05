package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DoCommand extends CommandBase {
    protected final Runnable toDo;
    private final double seconds;
    protected ElapsedTime timer;


    public DoCommand(Runnable toDo, double seconds) {
        this.toDo = toDo;
        this.seconds = seconds;
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer.reset();
        toDo.run();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= this.seconds;
    }
}
