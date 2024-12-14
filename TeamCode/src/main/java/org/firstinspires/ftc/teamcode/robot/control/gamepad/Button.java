package org.firstinspires.ftc.teamcode.robot.control.gamepad;

public class Button {
    private boolean lastState;
    private boolean currentState;
    private boolean wasClicked;
    private RobotAction action;
    private ActionFlag flag;

    /**
     * Constructor for Button.
     * @param currentState The initial state of the button
     */
    public Button(boolean currentState) {
        this.currentState = currentState;
        this.lastState = currentState;
        this.wasClicked = false;
    }

    /**
     * Updates the current state of the button
     * Should be called continuously in the opmode loop
     * @param currentState The new state of the button
     */
    public void updateCurrentState(boolean currentState) {
        this.lastState = this.currentState;
        this.currentState = currentState;

        wasClicked = !currentState && lastState;
    }

    /**
     * Uses a rising edge detector to detect when the button is pressed
     * @return returns true only once at the instant the user presses the button
     */
    public boolean isPressed() {
        System.out.println("is pressed: " + (currentState && !lastState));
        return currentState && !lastState;
    }

    /**
     * Uses a falling edge detector to detect when the button is released
     * @return returns true only once at the instant the user releases the button
     */
    public boolean isReleased() {
        System.out.println("is released: " + (currentState && !lastState));
        return currentState && !lastState;
    }

    /**
     * Checks if the button is currently being held down
     * @return continuously returns true while the button is being held down
     */
    public boolean isHeld() {
        return currentState;
    }

    /**
     * Checks if the button was pressed and then released.
     * Returns true only once after the button was pressed down and then it was released
     */
    public boolean isClicked() {
        if (wasClicked) {
            wasClicked = false; // Ensure this only returns true once
            System.out.println("is clicked: " + true);
            return true;
        }
        System.out.println("is clicked" + false);
        return false;
    }

    public void setActionFlag(ActionFlag flag) {
        this.flag = flag;
    }

    public void setAction(RobotAction action) {
        this.action = action;
    }

    // Getters

    /**
     * Gets the current state of the button.
     * @return The current state of the button.
     */
    public boolean getCurrentState() {
        return currentState;
    }

    /**
     * Gets the last state of the button.
     * @return The last state of the button.
     */
    public boolean getLastState() {
        return lastState;
    }

    public RobotAction getAction() {
        return action;
    }

    public boolean getFlagValue() {
        return flag.getValue();
    }
}
