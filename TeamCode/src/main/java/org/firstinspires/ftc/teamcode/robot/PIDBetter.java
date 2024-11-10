package org.firstinspires.ftc.teamcode.robot;

public class PIDBetter {
    public double p;
    public double i;
    public double d;
    public PIDBetter(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;
    }
    double integral = 0;
    long lastLoopTime = System.nanoTime();
    double lastError = 0;
    int counter = 0;
    double loopTime = 0.0;

    public void resetIntegral() {
        integral = 0;
    }

    public double update(double error, double min, double max){
        if (counter == 0){
            lastLoopTime = System.nanoTime() - 10000000;
        }

        long currentTime = System.nanoTime();
        loopTime = (currentTime - lastLoopTime)/1000000000.0;
        lastLoopTime = currentTime;

        double proportion = p * error;
        integral+= error * i * loopTime;
        double derivative = d * (error - lastError)/loopTime;

        lastError = error;
        counter++;

        return Utils.clip(proportion + integral + derivative, min, max);
    }

    public void updatePID(double p, double i, double d){
        this.p = p;
        this.i = i;
        this.d = d;
    }







}
