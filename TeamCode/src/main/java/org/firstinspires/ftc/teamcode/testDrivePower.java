package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by johnduval on 11/21/17.
 */
@TeleOp(name = "testing- drive power")

public class testDrivePower extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;

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
    }
}
