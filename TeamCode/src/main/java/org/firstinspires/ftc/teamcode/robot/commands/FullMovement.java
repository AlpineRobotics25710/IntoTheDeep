package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;

import java.util.function.BooleanSupplier;

public class FullMovement extends ConditionalCommand {
    /**
     * Creates a new ConditionalCommand.
     *
     * @param onTrue    the command to run if the condition is true
     * @param onFalse   the command to run if the condition is false
     * @param condition the condition to determine which command to run
     */
    public FullMovement(Command onTrue, Command onFalse, BooleanSupplier condition) {
        super(onTrue, onFalse, condition);
    }
}
