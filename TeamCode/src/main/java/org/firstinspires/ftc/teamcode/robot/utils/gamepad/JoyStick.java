package org.firstinspires.ftc.teamcode.robot.utils.gamepad;

public class JoyStick {
    /**
     * How much error the input has, such as controller drift. Initially set to 0.
     */
    private double error = 0.0;
    private double xValue;
    private double yValue;
    
    public JoyStick(double error, double xValue, double yValue) {
        this.error = error;
        this.xValue = xValue;
        this.yValue = yValue;
    }
    
    public JoyStick(double xValue, double yValue) {
        this.xValue = xValue;
        this.yValue = yValue;
    }
    
    public void updateCurrentValues(double xValue, double yValue) {
        this.xValue = xValue;
        this.yValue = yValue;
    }
    
    public void setError(double error) {
        this.error = error;
    }
    
    public double getError() {
        return error;
    }

    public double getX() {
        if (xValue > error || xValue < -error) {
            return xValue;
        }
        return 0;
    }

    public double getY() {
        if (yValue > error || yValue < -error) {
            return yValue;
        }
        return 0;
    }
}
