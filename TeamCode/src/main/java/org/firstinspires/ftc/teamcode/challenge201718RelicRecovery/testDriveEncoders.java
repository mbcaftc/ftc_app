package org.firstinspires.ftc.teamcode.challenge201718RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by johnduval on 11/21/17.
 */
@TeleOp (name = "test drive ENCODERS")
@Disabled
public class testDriveEncoders extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;

    double leftStickVal;
    double rightStickVal;

    double leftTriggerVal;
    double rightTriggerVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    @Override
    public void init() {
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
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(1);
            rearLeftMotor.setPower(1);
            rearRightMotor.setPower(1);
        }
        if (gamepad1.a) {
            frontLeftMotor.setPower(-1);
            frontRightMotor.setPower(-1);
            rearLeftMotor.setPower(-1);
            rearRightMotor.setPower(-1);
        }
        if (gamepad1.x) {
            frontLeftMotor.setPower(-1);
            frontRightMotor.setPower(.9);
            rearLeftMotor.setPower(.9);
            rearRightMotor.setPower(-1);
        }
        if (gamepad1.b) {
            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(-1);
            rearLeftMotor.setPower(-1);
            rearRightMotor.setPower(1);
        }
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            rearLeftMotor.setPower(0);
            rearRightMotor.setPower(0);
        }

        telemetry.addData("front left: ", frontLeftMotor.getCurrentPosition());
        telemetry.addData("front right: ", frontRightMotor.getCurrentPosition());
        telemetry.addData("rear left: ", rearLeftMotor.getCurrentPosition());
        telemetry.addData("rear right: ", rearRightMotor.getCurrentPosition());
    }
}
