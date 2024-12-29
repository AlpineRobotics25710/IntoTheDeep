package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.utils.TelemetryUtil;

@Config
@TeleOp
public class ExtendoSlidesTest extends CommandOpMode {
    private static final Robot robot = Robot.getInstance();
    //private static final CommandScheduler cmdScheduler = CommandScheduler.getInstance();
    //public static boolean manualMode = true;
   // public static double targetPosition = 0.0;

    @Override
    public void run() {
        //TelemetryUtil.packet.put("Manual Mode", robot.extendo.manualMode);
        // cmdScheduler.reset();

        //robot.extendo.setTargetPosition(targetPosition);
        //robot.extendo.setManualMode(true);

        // Cubed to slowly increase speed
        double manualPower = -gamepad1.left_stick_y;
        robot.extendo.setSlidesPower(manualPower);
        super.run();

        // TelemetryUtil.packet.put("Target Position", robot.extendo.getTargetPosition());
      //  TelemetryUtil.packet.put("Encoder Position", robot.extendo.getEncoderPosition());
      //  TelemetryUtil.packet.put("Manual Mode", Slides.manualMode);
        TelemetryUtil.packet.put("Manual power", manualPower);
        TelemetryUtil.sendTelemetry();
    }

    @Override
    public void initialize() {
        super.reset();
        robot.init(hardwareMap);
        //robot.extendo.manualMode = true;
        register(robot.extendo);
    }
}
