package org.firstinspires.ftc.teamcode.config.testers.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@TeleOp(name = "ExtendoTest", group = "Teleop")
public class ExtendoTest extends OpMode {
    private Robot robot;
    private double motorPower = 0.0; // Current length of the extendo
    private final double motorIncrement = 0.1; // Increment for changing the extendo length
    private double targetLength = 0.0;

    private final double lengthIncrement = 0.1;
    private double motorLength;
    // Button toggles for manual control
    private final ButtonToggle buttonY = new ButtonToggle();
    private final ButtonToggle buttonA = new ButtonToggle();
    private final ButtonToggle buttonX = new ButtonToggle();


    private boolean usingPid = false;
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
//        robot.follower.setTeleOpMovementVectors(
//                -gamepad1.left_stick_y,
//                -gamepad1.left_stick_x,
//                -gamepad1.right_stick_x,
//                true
//        );
        robot.update(); // might have to swap

        // Manual control for extendo
        if (buttonY.isClicked(gamepad2.y)) {
            motorPower += motorIncrement; // Increment the length
            targetLength += lengthIncrement;
        }
        if (buttonA.isClicked(gamepad2.a)) {
            motorPower -= motorIncrement; // Decrement the length
            targetLength -= lengthIncrement;
        }
        if(buttonX.isClicked(gamepad2.x)){
            usingPid = !usingPid;
        }
        if(usingPid){
            robot.extendo.setTargetLength(targetLength);
        }
        else{
            robot.extendo.setTargetPowerFORCED(motorPower);
        }
        // robot.extendo.setTargetPowerFORCED(motorPower);

        // Telemetry feedback
        telemetry.addData("Motor Power", motorPower);
        telemetry.addData("Target Length", targetLength);
        telemetry.addData("Extendo Length", robot.extendo.length);

        telemetry.update();
    }
}
