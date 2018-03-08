package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by johnduval on 11/21/17.
 */
@Autonomous(name = "Test - Run Using Encoders", group = "TESTING")
@Disabled
public class testAut_runUsingEncoders extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;

    int countsForward = 1000;
    int countsBack = 1000;
    double powerReductionFactor = 1;
    double countsWhile = 1;
    double power = .5;


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
            //RESETS ENCODERS AND MAKES SURE AT 0
            resetEncoders();
            //Set encoders to run Using Encoders
            runUsingEncoders();
/*
            frontLeftMotor.setTargetPosition(countsForward);
            frontRightMotor.setTargetPosition(countsForward);
            rearLeftMotor.setTargetPosition(countsForward);
            rearRightMotor.setTargetPosition(countsForward);
*/
            frontLeftMotor.setPower(power * powerReductionFactor);
            frontRightMotor.setPower(power * powerReductionFactor);
            rearLeftMotor.setPower(power * powerReductionFactor);
            rearRightMotor.setPower(power * powerReductionFactor);

            while (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || rearLeftMotor.isBusy() || rearRightMotor.isBusy()) {
                telemetry.addData("operation: ", "Going FORWARD");
                telemetry.addData("front left: ", frontLeftMotor.getCurrentPosition() + " / " + frontLeftMotor.getTargetPosition());
                telemetry.addData("front right: ", frontRightMotor.getCurrentPosition() + " / " + frontRightMotor.getTargetPosition());
                telemetry.addData("rear left: ", rearLeftMotor.getCurrentPosition() + " / " + rearLeftMotor.getTargetPosition());
                telemetry.addData("rear right: ", rearRightMotor.getCurrentPosition() + " / " + rearRightMotor.getTargetPosition());
                telemetry.update();
            }
            stopMotor();
            sleep(3000);

            resetEncoders();
            runUsingEncoders();

            frontLeftMotor.setTargetPosition(-countsForward);
            frontRightMotor.setTargetPosition(-countsForward);
            rearLeftMotor.setTargetPosition(-countsForward);
            rearRightMotor.setTargetPosition(-countsForward);

            frontLeftMotor.setPower(power * powerReductionFactor);
            frontRightMotor.setPower(power * powerReductionFactor);
            rearLeftMotor.setPower(power * powerReductionFactor);
            rearRightMotor.setPower(power * powerReductionFactor);

            while (frontLeftMotor.isBusy() || frontRightMotor.isBusy() || rearLeftMotor.isBusy() || rearRightMotor.isBusy()) {
                telemetry.addData("operation: ", "Going FORWARD");
                telemetry.addData("front left: ", frontLeftMotor.getCurrentPosition() + " / " + frontLeftMotor.getTargetPosition());
                telemetry.addData("front right: ", frontRightMotor.getCurrentPosition() + " / " + frontRightMotor.getTargetPosition());
                telemetry.addData("rear left: ", rearLeftMotor.getCurrentPosition() + " / " + rearLeftMotor.getTargetPosition());
                telemetry.addData("rear right: ", rearRightMotor.getCurrentPosition() + " / " + rearRightMotor.getTargetPosition());
                telemetry.update();
            }
            stopMotor();
            sleep(3000);

            /*
            sleep(500);
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            countsForward = 0;
            frontLeftMotor.setTargetPosition((int) countsForward);
            frontRightMotor.setTargetPosition((int) countsForward);
            rearLeftMotor.setTargetPosition((int) countsForward);
            rearRightMotor.setTargetPosition((int) countsForward);
            sleep(2000);

            //go backwards 4000 clicks
telemetry.addLine("RESET ENCODERS");
telemetry.update();

            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

sleep(1000);
            telemetry.addLine("RUN USING ENCODERS");
            telemetry.update();

            sleep(2000);

            telemetry.addLine("Setting Back Position");
            telemetry.update();
            frontLeftMotor.setTargetPosition((int) -countsBack);
            frontRightMotor.setTargetPosition((int) -countsBack);
            rearLeftMotor.setTargetPosition((int) -countsBack);
            rearRightMotor.setTargetPosition((int) -countsBack);

            telemetry.addData("operation: ", "Reset Encoders");
            telemetry.addData("front left: ", frontLeftMotor.getCurrentPosition() + " / " + frontLeftMotor.getTargetPosition());
            telemetry.addData("front right: ", frontRightMotor.getCurrentPosition() + " / " + frontRightMotor.getTargetPosition());
            telemetry.addData("rear left: ", rearLeftMotor.getCurrentPosition() + " / " + rearLeftMotor.getTargetPosition());
            telemetry.addData("rear right: ", rearRightMotor.getCurrentPosition() + " / " + rearRightMotor.getTargetPosition());
            telemetry.update();
            sleep(2000);

            while (frontLeftMotor.getCurrentPosition() > -countsBack * countsWhile && frontRightMotor.getCurrentPosition() > -countsBack * countsWhile && rearLeftMotor.getCurrentPosition() > -countsBack * countsWhile && rearRightMotor.getCurrentPosition() > -countsBack * countsWhile) {
                //frontLeftMotor.setPower(power * powerReductionFactor);
                //frontRightMotor.setPower(power * powerReductionFactor);
                //rearLeftMotor.setPower(power * powerReductionFactor);
                //rearRightMotor.setPower(power * powerReductionFactor);
                frontLeftMotor.setPower(power * powerReductionFactor);
                frontRightMotor.setPower(power * powerReductionFactor);
                rearLeftMotor.setPower(power * powerReductionFactor);
                rearRightMotor.setPower(power * powerReductionFactor);

                telemetry.addData("operation: ", "Going BACK");
                telemetry.addData("front left: ", frontLeftMotor.getCurrentPosition() + " / " + frontLeftMotor.getTargetPosition());
                telemetry.addData("front right: ", frontRightMotor.getCurrentPosition() + " / " + frontRightMotor.getTargetPosition());
                telemetry.addData("rear left: ", rearLeftMotor.getCurrentPosition() + " / " + rearLeftMotor.getTargetPosition());
                telemetry.addData("rear right: ", rearRightMotor.getCurrentPosition() + " / " + rearRightMotor.getTargetPosition());
                telemetry.update();
            }
            sleep(2000);
            telemetry.addData("operation: ", "STOPPED BACK");
            telemetry.addData("front left: ", frontLeftMotor.getCurrentPosition());
            telemetry.addData("front right: ", frontRightMotor.getCurrentPosition());
            telemetry.addData("rear left: ", rearLeftMotor.getCurrentPosition());
            telemetry.addData("rear right: ", rearRightMotor.getCurrentPosition());
            telemetry.update();
            sleep(10000);
*/

            requestOpModeStop();
        }
    }

    public void stopMotor () {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
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
}
