package org.firstinspires.ftc.teamcode.robot.lib;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

@Config
public class ExtendoSlidesLib extends SubsystemBase {
    public static double RETRACTED = 0.0;
    public static double EXTENDED = 0.0;
    public static double Kp;
    public static double Ki;
    public static double Kd;
    public static double Kf;

    public enum ExtendoState{
        RETRACTED, EXTENDED;
    }
    RobotLib robot = RobotLib.getInstance();
    public void init(){

    }
    public void periodic(){

    }
}
