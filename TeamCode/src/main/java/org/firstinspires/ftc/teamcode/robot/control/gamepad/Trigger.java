package org.firstinspires.ftc.teamcode.robot.control.gamepad;

public class Trigger {
    /**
     * How much error the input has, such as controller drift. Initially set to 0.
     */
    private double error = 0.0;
    private double value;

    public Trigger(double value) {
        this.value = value;
    }

    public Trigger(double error, double value) {
        this.error = error;
        this.value = value;
    }

    public void updateCurrentValue(double value) {
        this.value = value;
    }

    public double getValue() {
        if (value > error || value < -error) {
            return value;
        } else {
            return 0;
        }
    }
}
