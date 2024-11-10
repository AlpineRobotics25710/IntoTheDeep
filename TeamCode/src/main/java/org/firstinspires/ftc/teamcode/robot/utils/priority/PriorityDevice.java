package org.firstinspires.ftc.teamcode.robot.utils.priority;

public abstract class PriorityDevice {
    protected final double basePriority, priorityScale;
    public final String name;
    protected double lastUpdateTime, callLengthMillis;
    boolean isUpdated = false;

    public PriorityDevice(double basePriority, double priorityScale, String name) {
        this.basePriority = basePriority;
        this.priorityScale = priorityScale;
        this.name = name;
        lastUpdateTime = System.nanoTime();
    }

    public abstract double getPriority(double timeRemaining);

    protected abstract void update();

    public void resetUpdateBoolean() {
        isUpdated = false;
    }
}