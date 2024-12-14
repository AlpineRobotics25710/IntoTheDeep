package org.firstinspires.ftc.teamcode.robot.control.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class CustomGamepad {
    //TODO: Add support for the PS4 touchpad
    //TODO: Add support for other drive train movements (non-mecanum)

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

    // Support for joysticks
    private final JoyStick leftJoyStick;
    private final JoyStick rightJoyStick;

    // Support for triggers
    private final Trigger leftTrigger;
    private final Trigger rightTrigger;

    private final Button[] allButtons;

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

        leftJoyStick = new JoyStick(gamepad.left_stick_x, gamepad.left_stick_y);
        rightJoyStick = new JoyStick(gamepad.right_stick_x, gamepad.right_stick_y);

        leftTrigger = new Trigger(gamepad.left_trigger);
        rightTrigger = new Trigger(gamepad.right_trigger);

        allButtons = new Button[] {
                buttonA, buttonB, buttonX, buttonY,
                dpadUp, dpadDown, dpadLeft, dpadRight,
                guide, start, back, leftBumper, rightBumper,
                leftStickButton, rightStickButton, circle, cross,
                triangle, square, share, options, touchpad
        };
    }

    /**
     * Updates the button states and executes any actions mapped to the buttons.
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

        leftJoyStick.updateCurrentValues(gamepad.left_stick_x, gamepad.left_stick_y);
        rightJoyStick.updateCurrentValues(gamepad.right_stick_x, gamepad.right_stick_y);

        leftTrigger.updateCurrentValue(gamepad.left_trigger);
        rightTrigger.updateCurrentValue(gamepad.right_trigger);

        executeButtonActions();
    }

    public void executeButtonActions() {
        for (Button button : allButtons) {
            if (button.getFlagValue()) {
                button.getAction().execute();
            }
        }
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

    public JoyStick getLeftStick() {
        return leftJoyStick;
    }

    public JoyStick getRightStick() {
        return rightJoyStick;
    }

    public Trigger getLeftTrigger() {
        return leftTrigger;
    }

    public Trigger getRightTrigger() {
        return rightTrigger;
    }
}
