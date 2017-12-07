package org.firstinspires.ftc.teamcode.subClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static java.lang.Thread.sleep;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by johnduval on 10/7/17.
 */

public class mechDriveAuto {

    private DcMotor frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
    private double cryptoboxDistanceForward = 8;

    GyroSensor gyroSensor;
    ModernRoboticsI2cGyro mrGyro;

    int heading;
    int xVal, yVal, zVal;

    public mechDriveAuto (DcMotor frontLM, DcMotor frontRM, DcMotor rearLM, DcMotor rearRM) {

        frontLeftMotor = frontLM;
        frontRightMotor = frontRM;
        rearLeftMotor = rearLM;
        rearRightMotor = rearRM;

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void encoderDrivePlatformDistanceSensor (int direction, double power) {

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (direction == 1) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            rearLeftMotor.setPower(power);
            rearRightMotor.setPower(power);
        } else if (direction == 2) {
            frontLeftMotor.setPower(-power);
            frontRightMotor.setPower(-power);
            rearLeftMotor.setPower(-power);
            rearRightMotor.setPower(-power);
        }
    }

    public void encoderDrivePlatform (double distance, double power) {

        final double ENCODER_CPR = 1120;
        final double GEAR_RATIO = 1;
        final double WHEEL_DIAMETER = 4;
        final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double ROTATIONS = distance / CIRCUMFERENCE;
        double counts =  ENCODER_CPR * ROTATIONS * GEAR_RATIO;

        double powerReductionFactor = 1;
        double countsWhile = 1;

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      /*  while (frontLeftMotor.getCurrentPosition() < counts * countsWhile && frontRightMotor.getCurrentPosition() < counts * countsWhile && rearLeftMotor.getCurrentPosition() < counts * countsWhile && rearRightMotor.getCurrentPosition() < counts * countsWhile) {
            frontLeftMotor.setPower(power * powerReductionFactor);
            frontRightMotor.setPower(power * powerReductionFactor);
            rearLeftMotor.setPower(power * powerReductionFactor);
            rearRightMotor.setPower(power * powerReductionFactor);
        } */

        while (frontLeftMotor.getCurrentPosition() < counts * countsWhile && frontRightMotor.getCurrentPosition() < counts * countsWhile) {
            frontLeftMotor.setPower(power * powerReductionFactor);
            frontRightMotor.setPower(power * powerReductionFactor);
            rearLeftMotor.setPower(power * powerReductionFactor);
            rearRightMotor.setPower(power * powerReductionFactor);
        }
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        counts = 0;
        frontLeftMotor.setTargetPosition((int) counts);
        frontRightMotor.setTargetPosition((int) counts);
        rearLeftMotor.setTargetPosition((int) counts);
        rearRightMotor.setTargetPosition((int) counts);
        stopMotors();
    }

    public void encoderDrive (double distance, int direction, double power) {

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        final double ENCODER_CPR = 1120;
        final double GEAR_RATIO = 1;
        final double WHEEL_DIAMETER = 4;
        final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double ROTATIONS = distance / CIRCUMFERENCE;
        double counts =  ENCODER_CPR * ROTATIONS * GEAR_RATIO;

        double powerReductionFactor = .6;
        double countsWhile = .95;

        switch (direction) {
            case 1: // robot will move forward
                frontLeftMotor.setTargetPosition((int) counts);
                frontRightMotor.setTargetPosition((int) counts);
                rearLeftMotor.setTargetPosition((int) counts);
                rearRightMotor.setTargetPosition((int) counts);
                break;
            case 2: // robot will move backward
                frontLeftMotor.setTargetPosition((int) -counts);
                frontRightMotor.setTargetPosition((int) -counts);
                rearLeftMotor.setTargetPosition((int) -counts);
                rearRightMotor.setTargetPosition((int) -counts);
                break;
            case 3: // robot will strafe left
                frontLeftMotor.setTargetPosition((int) -counts);
                frontRightMotor.setTargetPosition((int) counts);
                rearLeftMotor.setTargetPosition((int) counts);
                rearRightMotor.setTargetPosition((int) -counts);
                break;
            case 4: // robot will strafe right
                frontLeftMotor.setTargetPosition((int) counts);
                frontRightMotor.setTargetPosition((int) -counts);
                rearLeftMotor.setTargetPosition((int) -counts);
                rearRightMotor.setTargetPosition((int) counts);
                break;
            case 5: // robot will rotate left
                frontLeftMotor.setTargetPosition((int) -counts);
                frontRightMotor.setTargetPosition((int) counts);
                rearLeftMotor.setTargetPosition((int) -counts);
                rearRightMotor.setTargetPosition((int) counts);
                break;
            case 6: // robot will rotate right
                frontLeftMotor.setTargetPosition((int) counts);
                frontRightMotor.setTargetPosition((int) -counts);
                rearLeftMotor.setTargetPosition((int) counts);
                rearRightMotor.setTargetPosition((int) -counts);
                break;
        }

        if (direction != 2) {
            while (frontLeftMotor.getCurrentPosition() < counts * countsWhile && frontRightMotor.getCurrentPosition() < counts * countsWhile && rearLeftMotor.getCurrentPosition() < counts * countsWhile && rearRightMotor.getCurrentPosition() < counts * countsWhile) {
                frontLeftMotor.setPower(power * powerReductionFactor);
                frontRightMotor.setPower(power * powerReductionFactor);
                rearLeftMotor.setPower(power * powerReductionFactor);
                rearRightMotor.setPower(power * powerReductionFactor);
            }
        }
        else if (direction == 2) {
            while (frontLeftMotor.getCurrentPosition() > -counts * countsWhile && frontRightMotor.getCurrentPosition() > -counts * countsWhile && rearLeftMotor.getCurrentPosition() > -counts * countsWhile && rearRightMotor.getCurrentPosition() > -counts * countsWhile) {
                frontLeftMotor.setPower(power * powerReductionFactor);
                frontRightMotor.setPower(power * powerReductionFactor);
                rearLeftMotor.setPower(power * powerReductionFactor);
                rearRightMotor.setPower(power * powerReductionFactor);
            }
        }
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        counts = 0;
        frontLeftMotor.setTargetPosition((int) counts);
        frontRightMotor.setTargetPosition((int) counts);
        rearLeftMotor.setTargetPosition((int) counts);
        rearRightMotor.setTargetPosition((int) counts);
        stopMotors();
    }


    public void encoderDriveMat (double distance, int direction, double power) {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        final double ENCODER_CPR = 1120;
        final double GEAR_RATIO = 1;
        final double WHEEL_DIAMETER = 4;
        final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double ROTATIONS = distance / CIRCUMFERENCE;
        double counts =  ENCODER_CPR * ROTATIONS * GEAR_RATIO;

        switch (direction) {
            case 1: // robot will move forward
                frontLeftMotor.setTargetPosition((int) counts);
                frontRightMotor.setTargetPosition((int) counts);
                rearLeftMotor.setTargetPosition((int) counts);
                rearRightMotor.setTargetPosition((int) counts);
                break;
            case 2: // robot will move backward
                frontLeftMotor.setTargetPosition((int) -counts);
                frontRightMotor.setTargetPosition((int) -counts);
                rearLeftMotor.setTargetPosition((int) -counts);
                rearRightMotor.setTargetPosition((int) -counts);
                break;
            case 3: // robot will strafe left
                frontLeftMotor.setTargetPosition((int) -counts);
                frontRightMotor.setTargetPosition((int) counts);
                rearLeftMotor.setTargetPosition((int) counts);
                rearRightMotor.setTargetPosition((int) -counts);
                break;
            case 4: // robot will strafe right
                frontLeftMotor.setTargetPosition((int) counts);
                frontRightMotor.setTargetPosition((int) -counts);
                rearLeftMotor.setTargetPosition((int) -counts);
                rearRightMotor.setTargetPosition((int) counts);
                break;
            case 5: // robot will rotate left
                frontLeftMotor.setTargetPosition((int) -counts);
                frontRightMotor.setTargetPosition((int) counts);
                rearLeftMotor.setTargetPosition((int) -counts);
                rearRightMotor.setTargetPosition((int) counts);
                break;
            case 6: // robot will rotate right
                frontLeftMotor.setTargetPosition((int) counts);
                frontRightMotor.setTargetPosition((int) -counts);
                rearLeftMotor.setTargetPosition((int) counts);
                rearRightMotor.setTargetPosition((int) -counts);
                break;
        }
        while (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || rearLeftMotor.isBusy() || rearRightMotor.isBusy()) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            rearLeftMotor.setPower(power);
            rearRightMotor.setPower(power);

        }
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        counts = 0;
        frontLeftMotor.setTargetPosition((int) counts);
        frontRightMotor.setTargetPosition((int) counts);
        rearLeftMotor.setTargetPosition((int) counts);
        rearRightMotor.setTargetPosition((int) counts);
        stopMotors();
    }

    public void rotateLeft (double speed) {

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        rearLeftMotor.setPower(-speed);
        rearRightMotor.setPower(speed);
    }

    public void rotateRight (double speed) {

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        rearLeftMotor.setPower(speed);
        rearRightMotor.setPower(-speed);
    }

    public void stopMotors () {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }

    public void setFrontLeftPower (double speed) {

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setPower(speed);

    }
    public void setFrontRightPower (double speed) {

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightMotor.setPower(speed);

    }
    public void setRearLeftPower (double speed) {

        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rearLeftMotor.setPower(speed);

    }
    public void setRearRightPower (double speed) {

        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rearRightMotor.setPower(speed);

    }

    public void gyroDriveStraight (int targetHeading, double speed, long sleepTime, int zAccumulated) throws InterruptedException {

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double frontLeftSpeed;
        double frontRightSpeed;
        double rearLeftSpeed;
        double rearRightSpeed;

        long startTime = System.nanoTime();
        long elapsedTime = (System.nanoTime() - startTime) / 1000000;

        while (elapsedTime < sleepTime) {
            int currentPosition = zAccumulated;
            int targetDeviation = (currentPosition - targetHeading);

            frontLeftSpeed = speed - targetDeviation;
            frontRightSpeed = speed + targetDeviation;
            rearLeftSpeed = speed - targetDeviation;
            rearRightSpeed = speed + targetDeviation;

            frontLeftMotor.setPower(frontLeftSpeed);
            frontRightMotor.setPower(frontRightSpeed);
            rearLeftMotor.setPower(rearLeftSpeed);
            rearRightMotor.setPower(rearRightSpeed);
        }
    }

    public void gyroTurn (double speed, int turnChoice /* rotate left = 1, rotate right = 2 */, int targetAngle, int zAccumulated) {

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (turnChoice == 1) {
            while (zAccumulated > -targetAngle) {
                frontLeftMotor.setPower(-speed);
                frontRightMotor.setPower(speed);
                rearLeftMotor.setPower(-speed);
                rearRightMotor.setPower(speed);
            }
        } else if (turnChoice == 2) {
            while (zAccumulated > -targetAngle) {
                frontLeftMotor.setPower(speed);
                frontRightMotor.setPower(-speed);
                rearLeftMotor.setPower(speed);
                rearRightMotor.setPower(-speed);
            }
        }
    }

    public void red1Jewel (colorSensorArm armSensor, int jewelColor) throws InterruptedException {

        //sleeps between movements to prevent robot from stuttering around
        if (jewelColor == 1) {
            armSensor.colorRotateClockwiseRed1Blue2();
            sleep(100);
            armSensor.colorRotateResting();
            armSensor.colorSensorArmUpSlow();
        }
        else if (jewelColor == 2) { // red alliance seeing blue jewel -- strafe left
            armSensor.colorRotateCounterClockwiseRed1Blue2();
            sleep(100);
            armSensor.colorRotateResting();
            armSensor.colorSensorArmUpSlow();
        }
        else {
            armSensor.colorSensorArmUpSlow();
        }
    }

    public void red2Jewel (colorSensorArm armSensor, int jewelColor) throws InterruptedException {

        //sleeps between movements to prevent robot from stuttering around
        if (jewelColor == 1) {
            armSensor.colorRotateClockwiseRed2Blue1();
            sleep(100);
            armSensor.colorRotateResting();
            armSensor.colorSensorArmUpSlow();
        }
        else if (jewelColor == 2) { // red alliance seeing blue jewel -- strafe left
            armSensor.colorRotateCounterClockwiseRed2Blue1();
            sleep(100);
            armSensor.colorRotateResting();
            armSensor.colorSensorArmUpSlow();
        }
        else {
            armSensor.colorSensorArmUpSlow();
        }
    }

    public void blue1Jewel (colorSensorArm armSensor, int jewelColor) throws InterruptedException {
        if (jewelColor == 1) {
            armSensor.colorRotateCounterClockwiseRed2Blue1();
            sleep(100);
            armSensor.colorRotateResting();
            armSensor.colorSensorArmUp();
        }
        else if (jewelColor == 2) {
            armSensor.colorRotateClockwiseRed2Blue1();
            sleep(100);
            armSensor.colorRotateResting();
            armSensor.colorSensorArmUp();
        }
        else {
            armSensor.colorSensorArmUp();
        }
    }

    public void blue2Jewel (colorSensorArm armSensor, int jewelColor) throws InterruptedException {
        if (jewelColor == 1) {
            armSensor.colorRotateCounterClockwiseRed1Blue2();
            sleep(100);
            armSensor.colorRotateResting();
            armSensor.colorSensorArmUp();
        }
        else if (jewelColor == 2) {
            armSensor.colorRotateClockwiseRed1Blue2();
            sleep(100);
            armSensor.colorRotateResting();
            armSensor.colorSensorArmUp();
        }
        else {
            armSensor.colorSensorArmUp();
        }
    }

    public void vuforiaLeft (glyphArms arms) throws InterruptedException {

        encoderDriveMat(9,3,0.75); //strafe left to column
        sleep(100);
        encoderDrive(cryptoboxDistanceForward,1,0.8); //go forward to cryptoBox
        sleep(100);
        arms.openGlyphArms();
        sleep(200);
        encoderDrive(1.5, 1, 1); //go forward to make sure glyph in column
        sleep(100);
        encoderDrive(3,2,1); //back
        sleep(100);
        encoderDrive(3.5,1,1); //forward
        sleep(100);
        encoderDrive(6,2,.5); //slow back
    }

    public void vuforiaCenter (glyphArms arms) throws InterruptedException {

        encoderDrive(cryptoboxDistanceForward,1,0.8); //go forward to cryptoBox
        sleep(100);
        arms.openGlyphArms();
        sleep(200);
        encoderDrive(1.5, 1, 1); //go forward to make sure glyph in column
        sleep(100);
        encoderDrive(3,2,1);
        sleep(100);
        encoderDrive(3.5,1,1);
        sleep(100);
        encoderDrive(6,2,.5);
    }

    public void vuforiaRight (glyphArms arms) throws InterruptedException {

        encoderDriveMat(9,4,0.75); //strafe right to column
        sleep(100);
        encoderDrive(cryptoboxDistanceForward, 1, 0.8); //go forward to cryptoBox
        sleep(100);
        arms.openGlyphArms();
        sleep(200);
        encoderDrive(1.5, 1, 1); //go forward to make sure glyph in column
        sleep(100);
        encoderDrive(3,2,1);
        sleep(100);
        encoderDrive(3.5,1,1);
        sleep(100);
        encoderDrive(7,2,.5);
    }

    public void powerDrive (int timeMS, int direction, double power) throws InterruptedException {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        switch (direction) {
            case 1: // robot will move forward
                frontLeftMotor.setPower(power);
                frontRightMotor.setPower(power);
                rearLeftMotor.setPower(power);
                rearRightMotor.setPower(power);
                sleep(timeMS);
                stopMotors();
                break;
            case 2: // robot will move backward
                frontLeftMotor.setPower(-power);
                frontRightMotor.setPower(-power);
                rearLeftMotor.setPower(-power);
                rearRightMotor.setPower(-power);
                sleep(timeMS);
                stopMotors();
                break;
            case 3: // robot will strafe left
                frontLeftMotor.setPower(-power);
                frontRightMotor.setPower(power);
                rearLeftMotor.setPower(power);
                rearRightMotor.setPower(-power);
                sleep(timeMS);
                stopMotors();
                break;
            case 4: // robot will strafe right
                frontLeftMotor.setPower(power);
                frontRightMotor.setPower(-power);
                rearLeftMotor.setPower(-power);
                rearRightMotor.setPower(power);
                sleep(timeMS);
                stopMotors();
                break;
            case 5: // robot will rotate left
                frontLeftMotor.setPower(-power);;
                frontRightMotor.setPower(power);
                rearLeftMotor.setPower(-power);
                rearRightMotor.setPower(power);
                sleep(timeMS);
                stopMotors();
                break;
            case 6: // robot will rotate right
                frontLeftMotor.setPower(power);
                frontRightMotor.setPower(-power);
                rearLeftMotor.setPower(power);
                rearRightMotor.setPower(-power);
                sleep(timeMS);
                stopMotors();
                break;
        }
        stopMotors();
    }
}
