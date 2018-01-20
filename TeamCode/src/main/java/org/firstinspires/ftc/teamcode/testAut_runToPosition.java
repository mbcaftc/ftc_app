package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by johnduval on 11/21/17.
 */
@Autonomous(name = "Test - Run To Position", group = "TESTING")
//@Disabled
public class testAut_runToPosition extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;

    double runForward = 4000;
    double powerHigh = .6;
    double powerLow = .3;

    double countsMultiplyer = .95;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        rearLeftMotor = hardwareMap.dcMotor.get("rear_left_motor");
        rearRightMotor = hardwareMap.dcMotor.get("rear_right_motor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        waitForStart();

        while (opModeIsActive()) {
            resetEncoders();

            runToPosition_Forward (runForward);

            sleep(5000);
            requestOpModeStop();
        }
    }

    //RESETS ENCODERS
    public void resetEncoders () {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (frontLeftMotor.getCurrentPosition() != 0 || frontRightMotor.getCurrentPosition() != 0 || rearLeftMotor.getCurrentPosition() != 0 || rearRightMotor.getCurrentPosition() != 0) {
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(100);
        }
    }

    public void runUsingEncoders () {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void runToPosition_Forward (double distance) throws InterruptedException
    {
        // Tell the motors where we are going
        frontLeftMotor.setTargetPosition((int)distance);
        frontRightMotor.setTargetPosition((int)distance);
        rearLeftMotor.setTargetPosition((int)distance);
        rearRightMotor.setTargetPosition((int)distance);

        // Set them a-going
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Give them the power level we want them to move at
        frontLeftMotor.setPower(powerHigh);
        frontRightMotor.setPower(powerHigh);
        rearLeftMotor.setPower(powerHigh);
        rearRightMotor.setPower(powerHigh);

        // Wait until they are done
        while (opModeIsActive() && (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || rearLeftMotor.isBusy() || rearRightMotor.isBusy()))
        {
            if (frontLeftMotor.getCurrentPosition() >= frontLeftMotor.getTargetPosition() * .5) {
                frontLeftMotor.setPower(powerLow);
                frontRightMotor.setPower(powerLow);
                rearLeftMotor.setPower(powerLow);
                rearRightMotor.setPower(powerLow);
                telemetry.addData("Current Speed: ", powerLow);
            }
            else {
                telemetry.addData("Current Speed: ", powerHigh);
            }
            telemetry.addData("operation: ", "Going FORWARD");
            telemetry.addData("front left: ", frontLeftMotor.getCurrentPosition() + " / " + frontLeftMotor.getTargetPosition());
            telemetry.addData("front right: ", frontRightMotor.getCurrentPosition() + " / " + frontRightMotor.getTargetPosition());
            telemetry.addData("rear left: ", rearLeftMotor.getCurrentPosition() + " / " + rearLeftMotor.getTargetPosition());
            telemetry.addData("rear right: ", rearRightMotor.getCurrentPosition() + " / " + rearRightMotor.getTargetPosition());
            telemetry.update();
        }
        telemetry.addData("operation: ", "Going FORWARD ENDED");
        telemetry.addData("front left: ", frontLeftMotor.getCurrentPosition() + " / " + frontLeftMotor.getTargetPosition());
        telemetry.addData("front right: ", frontRightMotor.getCurrentPosition() + " / " + frontRightMotor.getTargetPosition());
        telemetry.addData("rear left: ", rearLeftMotor.getCurrentPosition() + " / " + rearLeftMotor.getTargetPosition());
        telemetry.addData("rear right: ", rearRightMotor.getCurrentPosition() + " / " + rearRightMotor.getTargetPosition());
        telemetry.update();
        stopMotor();
        resetEncoders();
    }

    public void stopMotor () {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }
}
