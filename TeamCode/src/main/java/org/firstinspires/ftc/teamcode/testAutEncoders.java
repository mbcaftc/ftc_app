package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by johnduval on 11/21/17.
 */
@Autonomous(name = "Test -- Aut Strafing 2s", group = "TESTING")

public class testAutEncoders extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;

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

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            frontLeftMotor.setPower(-1);
            frontRightMotor.setPower(1);
            rearLeftMotor.setPower(1);
            rearRightMotor.setPower(-1);
            sleep(2000);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            rearLeftMotor.setPower(0);
            rearRightMotor.setPower(0);
            telemetry.addData("front left: ", frontLeftMotor.getCurrentPosition());
            telemetry.addData("front right: ", frontRightMotor.getCurrentPosition());
            telemetry.addData("rear left: ", rearLeftMotor.getCurrentPosition());
            telemetry.addData("rear right: ", rearRightMotor.getCurrentPosition());
            telemetry.update();
            sleep(5000);
            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(-1);
            rearLeftMotor.setPower(-1);
            rearRightMotor.setPower(1);
            sleep(2000);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            rearLeftMotor.setPower(0);
            rearRightMotor.setPower(0);
            telemetry.addData("front left: ", frontLeftMotor.getCurrentPosition());
            telemetry.addData("front right: ", frontRightMotor.getCurrentPosition());
            telemetry.addData("rear left: ", rearLeftMotor.getCurrentPosition());
            telemetry.addData("rear right: ", rearRightMotor.getCurrentPosition());
            telemetry.update();
            sleep(10000);
            requestOpModeStop();
        }
    }
}
