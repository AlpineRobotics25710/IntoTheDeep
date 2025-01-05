package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp
public class ServoTest extends OpMode {
    Servo test;
    public static double testValue = 0.0;
    @Override
    public void init() {
        test = hardwareMap.get(Servo.class, "outtakeClaw");
    }

    @Override
    public void loop() {
        test.setPosition(testValue);
    }
}
