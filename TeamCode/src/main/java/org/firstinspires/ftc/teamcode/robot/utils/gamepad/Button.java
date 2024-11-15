package org.firstinspires.ftc.teamcode.robot.utils.gamepad;

public class Button {
    private boolean lastState;
    private boolean currentState;
    private boolean wasClicked;

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
        return currentState && !lastState;
    }

    /**
     * Uses a falling edge detector to detect when the button is released
     * @return returns true only once at the instant the user releases the button
     */
    public boolean isReleased() {
        return !currentState && lastState;
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
            return true;
        }
        return false;
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
     * Sets the current state of the button.
     * @param currentState The new state of the button.
     */
    public void setCurrentState(boolean currentState) {
        this.currentState = currentState;
    }

    /**
     * Gets the last state of the button.
     * @return The last state of the button.
     */
    public boolean getLastState() {
        return lastState;
    }
}
