package org.firstinspires.ftc.teamcode.challenge201718RelicRecovery.extras;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

/**
 * Created by blake_shafer on 9/29/17.
 */

@TeleOp(name = "Color Sensor Arm")
@Disabled

public class ColorSensorArm extends OpMode {

    Servo colorSensorArm;

    ColorSensor colorSensor;

    double upPosition = 0.325;
    double downPositionPause1 = 0.75;
    double downPositionPause2 = 0.86;
    double downPositionFinal = 0.90;

    boolean armState; // Up = false, down = true



    @Override
    public void init() {

        colorSensorArm = hardwareMap.servo.get("color_sensor_arm");

        colorSensorArm.setPosition(upPosition); // Set to up position

        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        // bLedOn represents the state of the LED.
        boolean ledState = true;
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        colorSensor.enableLed(ledState);
    }

    @Override
    public void loop() {

        if (gamepad2.x) {
            armState = true; // Arm is down
        }

        else if (gamepad2.y) {
            armState = false; // Arm is up
            downPositionPause1 = 0.75;
            downPositionPause2 = 0.87;
        }

        if (!armState) {
            colorSensorArm.setPosition(upPosition);
        }

        else {
            colorSensorArm.setPosition(downPositionPause1);
            try {
                sleep(500);

            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            downPositionPause1 = downPositionFinal;
            colorSensorArm.setPosition(downPositionPause2);
            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            downPositionPause2 = downPositionFinal;
            colorSensorArm.setPosition(downPositionFinal);

        }
        // convert the RGB values to HSV values.
        //Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        //telemetry.addData("LED", ledState ? "On" : "Off");
        telemetry.addData("Clear", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        //telemetry.addData("Hue", hsvValues[0]);

        telemetry.addData("Servo", "Position: " + String.format("%.3f", colorSensorArm.getPosition()));
        telemetry.update();


    }
}
