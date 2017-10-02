package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by blake_shafer on 9/29/17.
 */

@TeleOp(name = "Color Sensor Arm")

public class ColorSensorArm extends OpMode {

    Servo colorSensorArm;

    ColorSensor colorSensor;

    double upPosition = 1;
    double downPosition = 0.4;

    boolean armState; // Up = false, down = true

    @Override
    public void init() {

        colorSensorArm = hardwareMap.servo.get("color_sensor_arm");

        colorSensorArm.setPosition(upPosition); // Set to up position

        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        boolean ledState = false;

        //colorSensor = hardwareMap.colorSensor.get("color_sensor");

        //colorSensor.enableLed(ledState);
    }

    @Override
    public void loop() {

        if (gamepad2.x) {
            armState = true; // Arm is down
        }

        else if (gamepad2.y) {
            armState = false; // Arm is up
        }

        if (!armState) {
            colorSensorArm.setPosition(upPosition);
        }

        else {
            colorSensorArm.setPosition(downPosition);
        }
    }
}
