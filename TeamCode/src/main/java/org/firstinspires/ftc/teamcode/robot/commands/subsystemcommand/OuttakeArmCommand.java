package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.mechanisms.outtake.OuttakeArm;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

public class OuttakeArmCommand extends InstantCommand {
    public OuttakeArmCommand(Robot robot, OuttakeArm.OuttakeArmState armState){
        super(
                () -> robot.outtakeArm.setState(armState)
        );
        TelemetryUtil.addData("sigma sigma boi moving to ", armState);
    }
}
