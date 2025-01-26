package org.firstinspires.ftc.teamcode.robot.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryUtil {
    private static MultipleTelemetry multipleTelemetry;

    private TelemetryUtil() {}

    public static void setup(Telemetry telemetry) {
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public static void addData(String key, Object value) {
        multipleTelemetry.addData(key, value);
    }

    public static void update() {
        multipleTelemetry.update();
    }

    public static void clear() {
        multipleTelemetry.clear();
    }

    public static void addData(String value){
        multipleTelemetry.addData(value, true);
    }
}
