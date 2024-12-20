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
        buttonA.update(gamepad.a);
        buttonB.update(gamepad.b);
        buttonX.update(gamepad.x);
        buttonY.update(gamepad.y);
        dpadDown.update(gamepad.dpad_down);
        dpadUp.update(gamepad.dpad_up);
        dpadLeft.update(gamepad.dpad_left);
        dpadRight.update(gamepad.dpad_right);
        guide.update(gamepad.guide);
        start.update(gamepad.start);
        back.update(gamepad.back);
        leftBumper.update(gamepad.left_bumper);
        rightBumper.update(gamepad.right_bumper);
        leftStickButton.update(gamepad.left_stick_button);
        rightStickButton.update(gamepad.right_stick_button);

        circle.update(gamepad.circle);
        cross.update(gamepad.cross);
        triangle.update(gamepad.triangle);
        square.update(gamepad.square);
        share.update(gamepad.share);
        options.update(gamepad.options);
        touchpad.update(gamepad.touchpad);

        leftJoyStick.updateValues(gamepad.left_stick_x, gamepad.left_stick_y);
        rightJoyStick.updateValues(gamepad.right_stick_x, gamepad.right_stick_y);

        leftTrigger.updateValue(gamepad.left_trigger);
        rightTrigger.updateValue(gamepad.right_trigger);

        executeButtonActions();
    }

    /**
     * Executes any actions mapped to the buttons. Handles any NullPointerExceptions if an action
     * is not mapped to a button or a flag has not been set.
     */
    public void executeButtonActions() {
        for (Button button : allButtons) {
            try {
                button.executeAction();
            } catch (IllegalStateException ignored){}
        }
    }

    public Button a() {
        return buttonA;
    }

    public Button b() {
        return buttonB;
    }

    public Button x() {
        return buttonX;
    }

    public Button y() {
        return buttonY;
    }

    public Button dpadUp() {
        return dpadUp;
    }

    public Button dpadDown() {
        return dpadDown;
    }

    public Button dpadLeft() {
        return dpadLeft;
    }

    public Button dpadRight() {
        return dpadRight;
    }

    public Button guide() {
        return guide;
    }

    public Button start() {
        return start;
    }

    public Button back() {
        return back;
    }

    public Button leftBumper() {
        return leftBumper;
    }

    public Button rightBumper() {
        return rightBumper;
    }

    public Button leftStickButton() {
        return leftStickButton;
    }

    public Button rightStickButton() {
        return rightStickButton;
    }

    public Button circle() {
        return circle;
    }

    public Button cross() {
        return cross;
    }

    public Button triangle() {
        return triangle;
    }

    public Button square() {
        return square;
    }

    public Button share() {
        return share;
    }

    public Button options() {
        return options;
    }

    public Button touchpad() {
        return touchpad;
    }

    public JoyStick leftStick() {
        return leftJoyStick;
    }

    public JoyStick rightStick() {
        return rightJoyStick;
    }

    public Trigger leftTrigger() {
        return leftTrigger;
    }

    public Trigger rightTrigger() {
        return rightTrigger;
    }
}
