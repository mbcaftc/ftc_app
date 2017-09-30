package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by blake_shafer on 9/29/17.
 */

@TeleOp(name = "Color Sensor Arm")

public class ColorSensorArm extends OpMode {

    Servo colorSensorArm;

    double upPosition;
    double downPosition;

    boolean armDown;

    @Override
    public void init() {

        colorSensorArm = hardwareMap.servo.get("color_sensor_arm");

        colorSensorArm.setPosition(0); // Set to up position
    }

    @Override
    public void loop() {

        if (gamepad2.x) {
            armDown = true; // Arm is down
        }

        else if (gamepad2.y) {
            armDown = false; // Arm is up
        }

        if (!armDown) {
            colorSensorArm.setPosition(0);
        }

        else {
            colorSensorArm.setPosition(1);
        }
    }
}
