package org.firstinspires.ftc.teamcode.challenge201718RelicRecovery.extras;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;


import static java.lang.Thread.sleep;

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
    int colorArmPause  = 300;

    public Servo colorSensorArm;
    public ColorSensor colorSensor;
    //mechDriveAuto myMechDrive;

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

    public void colorSensorArmUp () {
        colorSensorArm.setPosition(upPosition);
    }

    public void colorSensorArmDown () {
        colorSensorArm.setPosition(downPositionPause1);
        try {
            sleep(colorArmPause);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        colorSensorArm.setPosition(downPositionPause2);
        try {
            sleep(colorArmPause);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        colorSensorArm.setPosition(downPositionFinal);
    }

    public void colorSensorREDalliance () {
        if (colorSensor.red() > redThreshold) {
            //STRAFES RIGHT
            try {
                sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
         //   myMechDrive.encoderDrive(3,3,1);
            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            colorSensorArm.setPosition(upPosition); //SET COLOR SENSOR ARM TO UP POSITION
            //"RESET" WITH STRAFE LEFT
            try {
                sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
         //   myMechDrive.encoderDrive(3,4,1);

        }
        else if (colorSensor.blue() > blueThreshold) {
            try {
                sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //STRAFES LEFT
         //   myMechDrive.encoderDrive(3, 4, 1);
            try {
                sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            colorSensorArm.setPosition(upPosition); //SET COLOR SENSOR ARM TO UP POSITION
            //"RESET" WITH STRAFE RIGHT
            try {
                sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
         //   myMechDrive.encoderDrive(3, 3, 1);
        }
        else { //in case color sensor doesn't detect any color thresholds
            colorSensorArm.setPosition(upPosition);
        }
    }

    public void colorSensorBLUEalliance () {

    }
}
