package org.firstinspires.ftc.teamcode.challengeVortex.blake;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by blake_shafer on 11/21/16.
 */

@TeleOp(name = "Test Reflectance")
@Disabled

public class TestReflectance extends OpMode {

    OpticalDistanceSensor beaconColorSensor;

    @Override
    public void init() {

        beaconColorSensor = hardwareMap.opticalDistanceSensor.get("color_sensor_beacon");
    }

    @Override
    public void loop() {

        double reflectance = beaconColorSensor.getLightDetected();
        telemetry.addData("Reflectance: ", reflectance);
    }
}
