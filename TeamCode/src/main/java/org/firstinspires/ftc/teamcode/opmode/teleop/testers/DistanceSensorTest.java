package org.firstinspires.ftc.teamcode.opmode.teleop.testers;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // you can use this as a regular DistanceSensor.
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distanceSensor;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            telemetry.addData("deviceName", distanceSensor.getDeviceName() );
            telemetry.addData("range", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("range", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("range", distanceSensor.getDistance(DistanceUnit.METER));
            telemetry.addData("range", distanceSensor.getDistance(DistanceUnit.INCH));

            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();
        }
    }
}
