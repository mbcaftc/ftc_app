package org.firstinspires.ftc.teamcode.subClasses;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by johnduval on 10/7/17.
 */

public class colorSensorArmAuto {

    int redThreshold = 4;
    int blueThreshold = 4;
    double upPosition = 0.325;
    double downPositionPause1 = 0.75;
    double downPositionPause2 = 0.86;
    double downPositionFinal = 0.90;

    private Servo colorSensorArm;
    private ColorSensor colorSensor;

    public colorSensorArmAuto (Servo cSA, ColorSensor cS) {
        colorSensorArm = cSA;
        colorSensor = cS;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);
    }

    public void colorSensorSetup () {
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);
    }
}
