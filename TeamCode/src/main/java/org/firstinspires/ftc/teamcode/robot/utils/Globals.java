package org.firstinspires.ftc.teamcode.robot.utils;

public class Globals {
    public static long LOOP_START = System.nanoTime();
    public static double LOOP_TIME = 0.0;

    public static double GET_LOOP_TIMES() {
        LOOP_TIME = (System.nanoTime() - LOOP_START) / 1.0e9;
        return LOOP_TIME;
    }

    public static void START_LOOP(){
        LOOP_START = System.nanoTime();
    }
}
