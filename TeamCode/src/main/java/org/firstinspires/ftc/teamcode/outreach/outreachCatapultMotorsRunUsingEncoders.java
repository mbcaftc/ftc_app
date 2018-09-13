package org.firstinspires.ftc.teamcode.outreach;

import com.qualcomm.robotcore.hardware.DcMotor;

public class outreachCatapultMotorsRunUsingEncoders {
    private DcMotor catapultMotor;
    private double resetPower;
    private double launchPower;
    private int armTravelDistance;

    public outreachCatapultMotorsRunUsingEncoders (DcMotor cM) {
        catapultMotor = cM;

        catapultMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapultMotor.setDirection(DcMotor.Direction.FORWARD);

        resetPower = .1;
        launchPower = 1;

        armTravelDistance = 300;
    }


    public void catapultLaunch (outreachTouchSensorCatapult touchSensor) {
        catapultReset(touchSensor);
        catapultMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (catapultMotor.getCurrentPosition() <= 300) {
            catapultMotor.setPower(launchPower);
        }
        catapultMotor.setPower(0);
        catapultMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void catapultReset (outreachTouchSensorCatapult touchSensor) {
        while (touchSensor.checkCatapultTouchSensor() == true) {
            catapultMotor.setPower(-resetPower);
        }
        catapultMotor.setPower(0);
    }

    public void catapultMotorManualOperation (double armLower, double armRaise, outreachTouchSensorCatapult touchSensor) {
        catapultMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (armLower > 0 && touchSensor.checkCatapultTouchSensor() == true) {
            catapultMotor.setPower(-armLower);
        }
        else if (armRaise > 0) {
            catapultMotor.setPower(armRaise);
        }
        else {
            catapultMotor.setPower(0);
        }
    }
}
