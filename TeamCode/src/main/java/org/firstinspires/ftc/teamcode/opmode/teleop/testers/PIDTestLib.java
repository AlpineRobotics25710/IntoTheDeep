package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.utils.RobotLib;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@TeleOp
@Config
public class PIDTestLib extends CommandOpMode {
    private static double targetExtendo = 0.0;
    private final RobotLib robot = RobotLib.getInstance();
    @Override
    public void initialize(){
        super.reset();
        robot.init(hardwareMap);
        register(robot.extendo);
    }
    public void run(){
        super.run();
        robot.extendo.setExtendoTarget(targetExtendo);
        TelemetryUtil.packet.put("Target Extendo", targetExtendo); //this isn't really needed lol
        TelemetryUtil.packet.put("Current Extendo", robot.extendoLeft.getCurrentPosition());
    }

}
