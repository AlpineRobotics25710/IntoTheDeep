package org.firstinspires.ftc.teamcode.robot.control.gamepad;

public class JoyStick {
    /**
     * How much error the x input has, such as controller drift. Initially set to 0. Acts by setting
     * a dead zone, so can be used for that purpose as well.
     */
    private double xError = 0.0;

    /**
     * How much error the y input has, such as controller drift. Initially set to 0. Can also work
     * if you want to set a dead zone.
     */
    private double yError = 0.0;

    /**
     * Represents how sensitive the joystick should be
     */
    protected double sensitivity = 1.0;

    private double xValue;
    private double yValue;
    
    public JoyStick(double xValue, double yValue) {
        this.xValue = xValue;
        this.yValue = -yValue;
    }
    
    public void updateCurrentValues(double xValue, double yValue) {
        this.xValue = xValue;
        this.yValue = -yValue;
    }

    public void setXError(double xError) {
        this.xError = xError;
    }

    public double getXError() {
        return xError;
    }

    public void setYError(double yError) {
        this.yError = yError;
    }

    public double getYError() {
        return yError;
    }

    public void setSensitivity(double sensitivity) {
        this.sensitivity = sensitivity;
    }

    public double getSensitivity() {
        return sensitivity;
    }

    /**
     * Returns the x-value of the joystick while also taking into account the error.
     * @return the x-value of the joystick
     */
    public double getX() {
        if (xValue > xError || xValue < -xError) {
            return xValue;
        }
        return 0;
    }

    public double getY() {
        if (yValue > yError || yValue < -yError) {
            return yValue;
        }
        return 0;
    }
}
