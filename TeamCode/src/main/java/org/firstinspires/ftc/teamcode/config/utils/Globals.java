package org.firstinspires.ftc.teamcode.config.utils;

public class Globals {
    public static long LOOP_START = System.nanoTime();
    public static double LOOP_TIME = 0.0;
    public static RunMode mode = RunMode.TEST;
    public static boolean isRed = false;
    public static double slidesMax = 5.0;

    public static double GET_LOOP_TIMES() {
        LOOP_TIME = System.nanoTime() - LOOP_START / 1.0e9;
        return LOOP_TIME;
    }

    public static void START_LOOP(){
        LOOP_START = System.nanoTime();
    }
}