package org.firstinspires.ftc.teamcode.robot.control.gamepad;

public class Button {
    private boolean lastState;
    private boolean currentState;
    private boolean wasClicked;
    private boolean wasPressed;
    private boolean wasReleased;
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
        this.wasPressed = false;
        this.wasReleased = false;
    }

    /**
     * Updates the current state of the button
     * Should be called continuously in the opmode loop
     * @param currentState The new state of the button
     */
    public void update(boolean currentState) {
        this.lastState = this.currentState;
        this.currentState = currentState;

        boolean justPressed = currentState && !lastState;
        boolean justReleased = !currentState && lastState;

        if (justPressed) wasPressed = true;
        if (justReleased) wasReleased = true;

        if (wasPressed && wasReleased) {
            wasClicked = true;
            wasPressed = false;
            wasReleased = false;
        }
    }

    /**
     * Uses a rising edge detector to detect when the button is pressed
     * @return true only once at the instant the button is pressed
     */
    public boolean isPressed() {
        if (wasPressed) {
            wasPressed = false; // Clear after read
            return true;
        }
        return false;
    }

    /**
     * Uses a falling edge detector to detect when the button is released
     * @return true only once at the instant the button is released
     */
    public boolean isReleased() {
        if (wasReleased) {
            wasReleased = false; // Clear after read
            return true;
        }
        return false;
    }

    /**
     * Checks if the button is currently being held down
     * @return true while the button is being held down
     */
    public boolean isHeld() {
        return currentState;
    }

    /**
     * Checks if the button was pressed and released.
     * @return true only once after the button was pressed and released
     */
    public boolean isClicked() {
        if (wasClicked) {
            wasClicked = false; // Clear after read
            return true;
        }
        return false;
    }

    /**
     * Sets the flag with which the action that is mapped to this button will be executed.
     * @param flag the ActionFlag that controls when the action should be executed
     */
    public void setActionFlag(ActionFlag flag) {
        this.flag = flag;
    }

    /**
     * Sets the action to be executed when the given ActionFlag is true.
     * @param action the action to execute
     */
    public void setAction(RobotAction action) {
        this.action = action;
    }

    /**
     * Executes the mapped action if the set flag is true.
     */
    public void executeAction() throws IllegalStateException {
        if (flag == null) throw new IllegalStateException("ActionFlag is not set for this button.");
        if (action == null) throw new IllegalStateException("RobotAction is not set for this button.");
        if (flag.getValue()) action.execute();
    }

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

    /**
     * Gets the assigned RobotAction.
     * @return The assigned action.
     */
    public RobotAction getAction() {
        return action;
    }

    /**
     * Gets the value of the flag.
     * @return The flag's value, or false if the flag is null.
     */
    public boolean getFlagValue() {
        return flag != null && flag.getValue();
    }
}
