package org.firstinspires.ftc.teamcode.subClasses;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;


import static java.lang.Thread.sleep;

/**
 * Created by johnduval on 10/7/17.
 */

public class colorSensorArm {

    int redThreshold = 2;
    int blueThreshold = 2;
    double restPosition = 0.2;
    double upPosition = 0.2;
    double downPositionPause1 = 0.78;
    double downPositionPause2 = 0.88;
    double downPositionFinal = 0.94;
    int colorArmPause = 300;

    public Servo colorSensorArm;
    public ColorSensor colorSensor;
    public Servo colorSensorArmRotate;
    //mechDriveAuto myMechDrive;

    public colorSensorArm(Servo cSA, ColorSensor cS, Servo cSAR) {
        colorSensorArm = cSA;
        colorSensorArmRotate = cSAR;
        colorSensor = cS;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

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

    public void colorSensorArmUp() {
        colorSensorArm.setPosition(upPosition);
    }

    public void colorSensorArmRest() {
        colorSensorArm.setPosition(restPosition);
    }

    public void colorSensorArmDown() throws InterruptedException {
        colorSensorArm.setPosition(downPositionPause1);
        sleep(colorArmPause);
        colorSensorArm.setPosition(downPositionPause2);
        sleep(colorArmPause);
        colorSensorArm.setPosition(downPositionFinal);
    }

    //TeleOp cannot ThrowInterruptedExcpetion.
    public void colorSensorArmDownTesting () {
        colorSensorArm.setPosition(downPositionPause1);
        colorSensorArm.setPosition(downPositionPause2);
        colorSensorArm.setPosition(downPositionFinal);
    }

    public void colorRotateResting () {
        colorSensorArmRotate.setPosition(0.435);
    }

    public void colorRotateClockwise () {
        colorSensorArmRotate.setPosition(0.6);
    }

    public void colorRotateCounterClockwise () {
        colorSensorArmRotate.setPosition(0.3);
    }

    public int colorJewel() throws InterruptedException {
        //gives sensor time to be accurate
        //1 == red
        //2 == blue
        //3 = none detected
        sleep(500);
        if (colorSensor.red() >= redThreshold) {
            return 1;
        } else if (colorSensor.blue() >= blueThreshold) {
            return 2;
        } else {
            return 3;
        }
    }
}