
package org.firstinspires.ftc.teamcode.config.utils;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Utils {

    public static double clip(double value, double min, double max){
        return Math.min(Math.max(min, value), max);
    }

    public static double clip(double value){
        return clip(value, -1.0, 1.0);
    }
    public static double pointDistance(double x1, double x2, double y1, double y2){
        double dX = x1 - x2;
        double dY = y1 - y2;
        return Math.sqrt(dX * dX + dY * dY);
    }
}