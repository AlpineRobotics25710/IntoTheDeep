package org.firstinspires.ftc.teamcode.config.testers.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.subsystem.intake.Extendo;
import org.firstinspires.ftc.teamcode.config.utils.Globals;


@Disabled
@TeleOp
@Config
public class ExtendoTest extends LinearOpMode {

    public static double distance = 10;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Extendo ex = robot.extendo;
        waitForStart();

        while (!isStopRequested()) {
            Globals.START_LOOP();
            ex.setTargetLengthFORCED(distance);

            robot.sensors.update();
            robot.extendo.update();
            robot.hardwareQueue.update();
        }
    }
}