package org.firstinspires.ftc.teamcode.challenge201617VelocityVortex.duval;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by mbca on 1/10/17.
 */
@Autonomous(name = "DuVal Corner Red", group = "DUVAL")
@Disabled
public class RedCornerDuVal extends LinearOpMode {
    OpticalDistanceSensor odsSensor;

    DcMotor leftDriveMotor;

    ColorSensor colorSensor;
    DcMotor rightDriveMotor;

    DcMotor leftLauncherMotor;
    DcMotor rightLauncherMotor;

    double leftLaunchPower = 0;
    double rightLaunchPower = 0;
    double launchStartPowerIncrement = 0.004;
    double launchStopPowerIncrement = 0.001;
    double maximumLauncherPower = 0.5;
    double minimumLauncherPower = 0;

    DcMotor beltMotor;

    double beltPower = 0;
    double beltStartPowerIncrement = 0.05;
    double beltStopPowerIncrement = 0.01;
    double maximumForwardBeltPower = 0.3;
    double minimumBeltPower = 0;
    double maximumReverseBeltPower = -1.0;
    double forwardRunningBeltPower = 0.1;
    double reverseRunningBeltPower = -0.1;

    Servo leftArm;
    Servo rightArm;

    final static double leftArmUpPosition = 0.329;
    final static double rightArmUpPosition = 0.48;
    final static double leftArmDownPosition = 0.948;
    final static double rightArmDownPosition = 0.061;

    boolean beaconRED;
    boolean beaconBLUE;

    final static int launcherTimeA = 1000; //time for belt to start before launcher starts
    final static int launcherTimeB = 7000; //time belt will run to pick up the particles and launch them

    final static int moveTimeA = 1050; //time to move from wall into launching position
    final static int moveTimeB = 750; //from launch point to first white line
    final static int moveTimeC = 500; //moves back from wall after ODS sensor tells robot to stop - gets in position to raise beacon arms.
    final static int moveTimeD = 400; //gets close to beacon with the beacon arms raised.
    final static int moveTimeE = 200; //move slowly towards beacon with arms raised.
    final static int moveTimeF = 800; //robot moves back from beacon after having pressed it.

    final static int turnTimeA = 250; //first turn after launching to position towards first white line
    final static int turnTimeB = 1500; //turns to go straight on first white line

    final static double beaconDistanceRange = 4.1;

    final static double redArmRaiseThreshold = 5;
    final static double blueArmRaiseTheshold = 5;

    double leftGateArmClosedPositionForLaunch = .57; // These values are only place-holder values
    double leftGateArmClosedPosition = .715;
    double leftGateArmOpenPosition = 0.1; // You will need to experiment with these values to find the perfect servo positions
    double rightGateArmClosedPositionForLaunch = 0.43;
    double rightGateArmClosedPosition = .29;
    double rightGateArmOpenPosition = 0.9;

    Servo leftGateArm;
    Servo rightGateArm;

    @Override
    public void runOpMode() throws InterruptedException {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        odsSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");

        leftDriveMotor = hardwareMap.dcMotor.get("left_drive");
        rightDriveMotor = hardwareMap.dcMotor.get("right_drive");
        rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        leftLauncherMotor = hardwareMap.dcMotor.get("left_launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right_launcher");
        rightLauncherMotor.setDirection(DcMotor.Direction.REVERSE);

        beltMotor = hardwareMap.dcMotor.get("conveyor_belt");

        leftArm = hardwareMap.servo.get("left_arm");
        rightArm = hardwareMap.servo.get("right_arm");
        leftArm.setPosition(leftArmDownPosition);
        rightArm.setPosition(rightArmDownPosition);


        leftGateArm = hardwareMap.servo.get("left_gate_arm"); // Don't forget to program left and right _gate_arm into the RC phone as servos
        rightGateArm = hardwareMap.servo.get("right_gate_arm"); // Or else the phone won't be able to communicate with the program (stuff won't work)
        leftGateArm.setPosition(leftGateArmClosedPosition); // Also, while doing this, don't forget to delete the old gate_arm which was the dcMotor registered in the RC phone
        rightGateArm.setPosition(rightGateArmClosedPosition); // Remember that it matters how you plug in the servos to the module; it depends on how you assigned them in the RC phone


        waitForStart();

        while (opModeIsActive()) {


            leftDriveMotor.setPower(.4);
            rightDriveMotor.setPower(.4);
//distance to first line to launch
            sleep(moveTimeA);

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            while (leftLaunchPower < maximumLauncherPower || rightLaunchPower < maximumLauncherPower) {
                if (leftLaunchPower < maximumLauncherPower) {
                    leftLaunchPower = leftLaunchPower + launchStartPowerIncrement;
                }
                if (rightLaunchPower < maximumLauncherPower) {
                    rightLaunchPower = rightLaunchPower + launchStartPowerIncrement;
                }
                leftLauncherMotor.setPower(leftLaunchPower);
                rightLauncherMotor.setPower(rightLaunchPower);
            }

//time for launcher to start
            sleep(launcherTimeA);

            leftGateArm.setPosition(leftGateArmClosedPositionForLaunch); // Also, while doing this, don't forget to delete the old gate_arm which was the dcMotor registered in the RC phone
            rightGateArm.setPosition(rightGateArmClosedPositionForLaunch); // Remember that it matters how you plug in the servos to the module; it depends on how you assigned them in the RC phone

            beltMotor.setPower(maximumForwardBeltPower);

// time for belt to run
            sleep(launcherTimeB);

            beltMotor.setPower(0);

            while (leftLaunchPower > 0 || rightLaunchPower > 0) {
                if (leftLaunchPower > 0) {
                    leftLaunchPower = leftLaunchPower - launchStopPowerIncrement;
                }
                if (rightLaunchPower > 0) {
                    rightLaunchPower = rightLaunchPower - launchStopPowerIncrement;
                }
                leftLauncherMotor.setPower(leftLaunchPower);
                rightLauncherMotor.setPower(rightLaunchPower);
            }
//closes the arms to make sure doesn't interfere with going to walls.
            leftGateArm.setPosition(leftGateArmClosedPosition); // Also, while doing this, don't forget to delete the old gate_arm which was the dcMotor registered in the RC phone
            rightGateArm.setPosition(rightGateArmClosedPosition); // Remember that it matters how you plug in the servos to the module; it depends on how you assigned them in the RC phone

//Makes sure everything's stopped before moving
            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            leftDriveMotor.setPower(-.3);
            rightDriveMotor.setPower(.3);
//time to make turn to go towards first white line
            sleep (turnTimeA);

            leftDriveMotor.setPower(.5);
            rightDriveMotor.setPower(.5);
//time to go forward to first line
            sleep (moveTimeB);
//Makes sure everything's stopped before moving
            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);
/*
            leftDriveMotor.setPower(-.3);
            rightDriveMotor.setPower(.3);
//time to turn to go straight on first white line
            sleep (turnTimeB);
//robot will go forward until it gets close enough.
            while (odsSensor.getRawLightDetected() < beaconDistanceRange && opModeIsActive()) {
                leftDriveMotor.setPower(.3);
                rightDriveMotor.setPower(.3);
            }
//Makes sure everything's stopped before moving
            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

//checks the color sensor to determine if blue or red where the color sensor is mounted
            if (colorSensor.red() > redArmRaiseThreshold) {
                beaconRED = true;
            }
            else {
                beaconBLUE = false;
            }

            leftDriveMotor.setPower(-.5);
            rightDriveMotor.setPower(-.5);
//time for robot to move BACK from the beacons
            sleep(moveTimeC);

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            if (beaconRED) {
                leftArm.setPosition(leftArmUpPosition);
            }
            else {
                rightArm.setPosition(rightArmUpPosition);
            }


            leftDriveMotor.setPower(.5);
            rightDriveMotor.setPower(.5);
            sleep (moveTimeD);

            leftDriveMotor.setPower(.2);
            rightDriveMotor.setPower(.2);
            sleep(moveTimeE);

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            leftDriveMotor.setPower(-.5);
            rightDriveMotor.setPower(-.5);
            sleep(moveTimeF);

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            leftArm.setPosition(leftArmDownPosition);
            rightArm.setPosition(rightArmDownPosition);


*/

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            telemetry.update();

            requestOpModeStop();
        }
    }
}
