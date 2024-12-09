package org.firstinspires.ftc.teamcode.robot;

public class RobotConstants {
    public static long LOOP_START = System.nanoTime();
    public static double LOOP_TIME = 0.0;
    public static Mode mode;
    public static Alliance alliance;
    public static double slidesMax = 5.0;
    public static double ticksToInches = 0.04132142857142857;

    public static double GET_LOOP_TIMES() {
        LOOP_TIME = System.nanoTime() - LOOP_START / 1.0e9;
        return LOOP_TIME;
    }

    public static void START_LOOP(){
        LOOP_START = System.nanoTime();
    }

    public enum Mode {
        AUTO,TELEOP, TESTING
    }

    public enum Alliance {
        RED,BLUE
    }
}
