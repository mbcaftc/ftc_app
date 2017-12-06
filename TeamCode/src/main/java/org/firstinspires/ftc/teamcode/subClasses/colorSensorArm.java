package org.firstinspires.ftc.teamcode.subClasses;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


import static java.lang.Thread.sleep;

/**
 * Created by johnduval on 10/7/17.
 */

public class colorSensorArm {

    int redThreshold = 2;
    int blueThreshold = 2;
    //double restPosition = 0.2;
    double upPositionPause1 = .40;
    double upPosition = 0.15;
    double downPositionPause1 = 0.70;
    double downPositionPause2 = 0.88;
    double downPositionFinal = 0.94;
    int colorArmPause = 500;
    double colorArmIncrementAmount = .01;
    int colorArmIncrementTimeMSdown = 12; //will take aprox. 1 second to lower color sensor arm .
    int colorArmIncrementTimeMSup = 15; //will take aprox. 1 second to lower color sensor arm .

    public Servo colorSensorArm;
    public ColorSensor colorSensor;
    public Servo colorSensorArmRotate;
    //mechDriveAuto myMechDrive;

    public colorSensorArm(Servo cSA, ColorSensor cS, Servo cSAR) {
        colorSensorArm = cSA;
        colorSensorArmRotate = cSAR;
        colorSensor = cS;

        //  MODERN ROBOTICS COLOR SENSOR CODE
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

    public void colorSensorArmUpSlow () throws InterruptedException {
        double colorArmIncrementPosition = upPositionPause1;
        colorSensorArm.setPosition(colorArmIncrementPosition);
        sleep(colorArmPause);
        while (colorArmIncrementPosition >=  upPosition) {
            colorArmIncrementPosition = colorArmIncrementPosition - colorArmIncrementAmount;
            colorSensorArm.setPosition(colorArmIncrementPosition);
            sleep(colorArmIncrementTimeMSup);
        }
    }

    //public void colorSensorArmRest() {
    //   colorSensorArm.setPosition(restPosition);
    //}

    public void colorSensorArmDown() throws InterruptedException {
        colorSensorArm.setPosition(downPositionPause1);
        sleep(colorArmPause);
        colorSensorArm.setPosition(downPositionPause2);
        sleep(colorArmPause);
        colorSensorArm.setPosition(downPositionFinal);
    }

    public void colorSensorArmDownSlow() throws InterruptedException {
        double colorArmIncrementPosition = downPositionPause1;
        colorSensorArm.setPosition(colorArmIncrementPosition);
        sleep(colorArmPause);
        while (colorArmIncrementPosition <=  downPositionFinal) {
            colorArmIncrementPosition = colorArmIncrementPosition + colorArmIncrementAmount;
            colorSensorArm.setPosition(colorArmIncrementPosition);
            sleep(colorArmIncrementTimeMSdown);
        }
    }

    public void colorRotateResting () {
        colorSensorArmRotate.setPosition(0.444);
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