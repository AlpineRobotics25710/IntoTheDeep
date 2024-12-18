package org.firstinspires.ftc.teamcode.robot.mechanisms;

public interface Mechanism {
    /**
     * Runs the initialization code for this mechanism
     */
    void init();

    /**
     * This method is to be called continuously in a loop if a mechanism, such as a PID controller,
     * needs to be updated continuously. By default, this method does nothing.
     */
    default void update(){};

    void ascend();

    void transfer();

    void intake();

    void lowChamber();

    void highChamber();

    void lowBasket();

    void highBasket();
}
