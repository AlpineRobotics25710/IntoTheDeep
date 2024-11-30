package org.firstinspires.ftc.teamcode.config.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.sensors.Sensors;
import org.firstinspires.ftc.teamcode.config.utils.Utils;
import org.firstinspires.ftc.teamcode.config.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.config.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.config.utils.priority.PriorityDevice;
import org.firstinspires.ftc.teamcode.config.utils.priority.PriorityMotor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@Config
@TeleOp(group = "Test")
public class MotorTesting extends LinearOpMode {
    HardwareQueue hardwareQueue;
    Robot robot;
    ArrayList<PriorityMotor> motors;
    private final int testValue = 0;
    private PriorityMotor motorTest;
    private DcMotorEx motorTest2;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        Sensors sensors = new Sensors(hardwareMap, hardwareQueue, robot);
        hardwareQueue = robot.hardwareQueue;

        motorTest = new PriorityMotor(hardwareMap.get(DcMotorEx.class, "intakeMotor1"), "intakeMotor1", 2, 5, sensors);
        //motorTest2 = hardwareMap.get(DcMotorEx.class, "intakeMotor1");

        hardwareQueue.addDevice(motorTest);
        //motors = new ArrayList<>();
        motors = new ArrayList<>();
        // buttons for changing motor
        ButtonToggle buttonY = new ButtonToggle();
        ButtonToggle buttonA = new ButtonToggle();

        int motorSize = 0;
        int motorIndex = 0;
        double motorPower = 0.0;

        // getting number of motors we have;
        for (PriorityDevice device : hardwareQueue.devices) {
            if (device instanceof PriorityMotor) {
                motors.add((PriorityMotor) device);
                motorSize++;
            }
        }

        waitForStart();

        while (!isStopRequested()) {
            resetMinPowersToOvercomeFriction();

            if (buttonY.isClicked(gamepad1.y)) {
                motors.get(motorIndex).setTargetPower(0.0);
//                motorIndex++;
                motorPower = 0.0;
            }

//            if (buttonA.isClicked(gamepad1.a)) {
//                motors.get(motorIndex).setTargetPower(0.0);
//                motorIndex--;
//                motorPower = 0.0;
//            }

            if (gamepad1.b) {
                motorPower += 0.01;
            }

            if (gamepad1.x) {
                motorPower -= 0.01;
            }

            motorPower = Utils.clip(motorPower, -1.0, 1.0);
            //motorIndex = Math.abs(motorIndex) % motorSize;

            motors.get(motorIndex).setTargetPower(motorPower);

            robot.update();

            telemetry.addData("motor index", motorIndex);
            telemetry.addData("motor name", motors.get(motorIndex).name);
            telemetry.addData("motor power", motorPower);
            telemetry.addData("motor velocity", motors.get(motorIndex).getVelocity());
            telemetry.update();
        }
    }

    public void resetMinPowersToOvercomeFriction() {
        for (PriorityMotor motor : motors) {
            motor.setMinimumPowerToOvercomeStaticFriction(0.0);
        }
    }
}