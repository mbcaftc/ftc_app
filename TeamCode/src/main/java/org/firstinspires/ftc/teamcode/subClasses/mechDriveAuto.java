package org.firstinspires.ftc.teamcode.subClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Thread.sleep;

/**
 * Created by johnduval on 10/7/17.
 */

public class mechDriveAuto {

    private DcMotor frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;

    public mechDriveAuto (DcMotor frontLM, DcMotor frontRM, DcMotor rearLM, DcMotor rearRM) {

        frontLeftMotor = frontLM;
        frontRightMotor = frontRM;
        rearLeftMotor = rearLM;
        rearRightMotor = rearRM;

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void encoderDrive (int distance, int direction, double power) {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        final int ENCODER_CPR = 1120;
        final int GEAR_RATIO = 1;
        final int WHEEL_DIAMETER = 4;
        final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double ROTATIONS = distance / CIRCUMFERENCE;
        double counts =  ENCODER_CPR * ROTATIONS * GEAR_RATIO;

        double powerReductionFactor = 0.77;

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
        frontLeftMotor.setPower(power * powerReductionFactor);
        frontRightMotor.setPower(power * powerReductionFactor);
        rearLeftMotor.setPower(power * powerReductionFactor);
        rearRightMotor.setPower(power * powerReductionFactor);
    }

    public void redAllianceJewel (colorSensorArm armSensor, int jewelColor) throws InterruptedException {
        if (jewelColor == 1) {
            encoderDrive(3,3,1);
            sleep (500);
            armSensor.colorSensorArmUp();
            sleep(200);
            encoderDrive(3,4,1);
            sleep(200);

        }
        else if (jewelColor == 2) {
            encoderDrive(3,4,1);
            sleep(500);
            armSensor.colorSensorArmUp();
            sleep(200);
            encoderDrive(3,3,1);
            sleep(200);
        }
        else {
            sleep(100);
            armSensor.colorSensorArmUp();
        }
    }

    public void blueAllianceJewel (colorSensorArm armSensor, int jewelColor) throws InterruptedException {
        if (jewelColor == 1) {
            encoderDrive(3,3,1);
            sleep (500);
            armSensor.colorSensorArmUp();
            sleep(200);
            encoderDrive(3,4,1);
            sleep(200);

        }
        else if (jewelColor == 2) {
            encoderDrive(3,4,1);
            sleep(500);
            armSensor.colorSensorArmUp();
            sleep(200);
            encoderDrive(3,3,1);
            sleep(200);
        }
        else {
            sleep(100);
            armSensor.colorSensorArmUp();
        }
    }

    public void mechDriveSTOP () {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setTargetPosition(0);
    }
}
