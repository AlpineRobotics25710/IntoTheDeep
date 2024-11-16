package org.firstinspires.ftc.teamcode.robot.utils.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class CustomGamepad {
    //TODO: Add support for analog inputs (such as the sticks) and also the PS4 touchpad
    //TODO: Add support for the drive train movements

    private final Gamepad gamepad;
    private final Button buttonA;
    private final Button buttonB;
    private final Button buttonX;
    private final Button buttonY;
    private final Button dpadUp;
    private final Button dpadDown;
    private final Button dpadLeft;
    private final Button dpadRight;
    private final Button guide;
    private final Button start;
    private final Button back;
    private final Button leftBumper;
    private final Button rightBumper;
    private final Button leftStickButton;
    private final Button rightStickButton;

    // PS4 support buttons
    private final Button circle;
    private final Button cross;
    private final Button triangle;
    private final Button square;
    private final Button share;
    private final Button options;
    private final Button touchpad;

    /**
     * Constructor for CustomGamepad.
     * @param gamepad the gamepad to detect input from
     */
    public CustomGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;

        buttonA = new Button(gamepad.a);
        buttonB = new Button(gamepad.b);
        buttonX = new Button(gamepad.x);
        buttonY = new Button(gamepad.y);
        dpadDown = new Button(gamepad.dpad_down);
        dpadUp = new Button(gamepad.dpad_up);
        dpadLeft = new Button(gamepad.dpad_left);
        dpadRight = new Button(gamepad.dpad_right);
        guide = new Button(gamepad.guide);
        start = new Button(gamepad.start);
        back = new Button(gamepad.back);
        leftBumper = new Button(gamepad.left_bumper);
        rightBumper = new Button(gamepad.right_bumper);
        leftStickButton = new Button(gamepad.left_stick_button);
        rightStickButton = new Button(gamepad.right_stick_button);
        circle = new Button(gamepad.circle);
        cross = new Button(gamepad.cross);
        triangle = new Button(gamepad.triangle);
        square = new Button(gamepad.square);
        share = new Button(gamepad.share);
        options = new Button(gamepad.options);
        touchpad = new Button(gamepad.touchpad);
    }

    /**
     * Updates the button states.
     * Needs to be called continuously in opmode loop
     */
    public void update() {
        buttonA.updateCurrentState(gamepad.a);
        buttonB.updateCurrentState(gamepad.b);
        buttonX.updateCurrentState(gamepad.x);
        buttonY.updateCurrentState(gamepad.y);
        dpadDown.updateCurrentState(gamepad.dpad_down);
        dpadUp.updateCurrentState(gamepad.dpad_up);
        dpadLeft.updateCurrentState(gamepad.dpad_left);
        dpadRight.updateCurrentState(gamepad.dpad_right);
        guide.updateCurrentState(gamepad.guide);
        start.updateCurrentState(gamepad.start);
        back.updateCurrentState(gamepad.back);
        leftBumper.updateCurrentState(gamepad.left_bumper);
        rightBumper.updateCurrentState(gamepad.right_bumper);
        leftStickButton.updateCurrentState(gamepad.left_stick_button);
        rightStickButton.updateCurrentState(gamepad.right_stick_button);
        circle.updateCurrentState(gamepad.circle);
        cross.updateCurrentState(gamepad.cross);
        triangle.updateCurrentState(gamepad.triangle);
        square.updateCurrentState(gamepad.square);
        share.updateCurrentState(gamepad.share);
        options.updateCurrentState(gamepad.options);
        touchpad.updateCurrentState(gamepad.touchpad);
    }

    public Button getA() {
        return buttonA;
    }

    public Button getB() {
        return buttonB;
    }

    public Button getX() {
        return buttonX;
    }

    public Button getY() {
        return buttonY;
    }

    public Button getDpadUp() {
        return dpadUp;
    }

    public Button getDpadDown() {
        return dpadDown;
    }

    public Button getDpadLeft() {
        return dpadLeft;
    }

    public Button getDpadRight() {
        return dpadRight;
    }

    public Button getGuide() {
        return guide;
    }

    public Button getStart() {
        return start;
    }

    public Button getBack() {
        return back;
    }

    public Button getLeftBumper() {
        return leftBumper;
    }

    public Button getRightBumper() {
        return rightBumper;
    }

    public Button getLeftStickButton() {
        return leftStickButton;
    }

    public Button getRightStickButton() {
        return rightStickButton;
    }

    public Button getCircle() {
        return circle;
    }

    public Button getCross() {
        return cross;
    }

    public Button getTriangle() {
        return triangle;
    }

    public Button getSquare() {
        return square;
    }

    public Button getShare() {
        return share;
    }

    public Button getOptions() {
        return options;
    }

    public Button getTouchpad() {
        return touchpad;
    }
}
