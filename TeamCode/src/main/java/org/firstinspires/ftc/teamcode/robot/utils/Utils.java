
package org.firstinspires.ftc.teamcode.robot.utils;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Utils {

    public static double clip(double value, double min, double max){
        return Math.min(Math.max(min, value), max);
    }

    public static double pointDistance(double x1, double x2, double y1, double y2){
        double dX = x1 - x2;
        double dY = y1 - y2;
        return Math.sqrt(dX * dX + dY * dY);
    }
}
