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
    public void updateCurrentState(boolean currentState) {
        this.lastState = this.currentState;
        this.currentState = currentState;

        wasPressed = currentState && !lastState;
        wasReleased = !currentState && lastState;
        if (wasReleased) {
            wasClicked = true;
        }
    }

    /**
     * Uses a rising edge detector to detect when the button is pressed
     * @return returns true only once at the instant the user presses the button
     */
    public boolean isPressed() {
        if (wasPressed) {
            wasPressed = false;
            return true;
        }
        return false;
    }

    /**
     * Uses a falling edge detector to detect when the button is released
     * @return returns true only once at the instant the user releases the button
     */
    public boolean isReleased() {
        if (wasReleased) {
            wasReleased = false;
            return true;
        }
        return false;
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
            wasClicked = false;
            return true;
        }
        return false;
    }

    /**
     * Sets the flag with which the action that is mapped to this button will be executed. Also need to set an action
     * through setAction().
     * @param flag a {@link ActionFlag} that controls when the action should be executed
     */
    public void setActionFlag(ActionFlag flag) {
        this.flag = flag;
    }

    /**
     * Sets the action to be executed when the given ActionFlag is true. Also need to set an {@link ActionFlag} for
     * this to work.
     * @param action the action to execute
     */
    public void setAction(RobotAction action) {
        this.action = action;
    }

    /**
     * Executes the mapped action if the set flag is true. Both an {@link ActionFlag} and {@link RobotAction} need to
     * be set for this to work.
     */
    public void executeAction() {
        if (flag != null && action != null) {
            if (flag.getValue()) {
                action.execute();
            }
        } else {
            throw new NullPointerException("Attempt to call executeAction() when an ActionFlag and RobotAction have " +
                    "not been set!");
        }
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
