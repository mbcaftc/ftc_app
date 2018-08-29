package org.firstinspires.ftc.teamcode.challengeRelicRecovery201718;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by johnduval on 11/21/17.
 */
@TeleOp(name = "test drive POWER - 2 seconds forward")
@Disabled
public class testDrivePower extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;

    ElapsedTime elapsedTime;

    double power;

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
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        elapsedTime = new ElapsedTime();

        power = .3;
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            rearLeftMotor.setPower(power);
            rearRightMotor.setPower(power);
            elapsedTime.reset();
        }
        if (gamepad1.a) {
            frontLeftMotor.setPower(-power);
            frontRightMotor.setPower(-power);
            rearLeftMotor.setPower(-power);
            rearRightMotor.setPower(-power);
        }
        if (gamepad1.x) {
            frontLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
            rearLeftMotor.setPower(power);
            rearRightMotor.setPower(-power);
        }
        if (gamepad1.b) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
            rearLeftMotor.setPower(-power);
            rearRightMotor.setPower(power);
        }
        if (gamepad1.right_bumper || gamepad1.left_bumper || elapsedTime.milliseconds() > 2000) {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            rearLeftMotor.setPower(0);
            rearRightMotor.setPower(0);
        }
        telemetry.addData("TIME ms: ", elapsedTime.milliseconds());
        telemetry.update();
    }
}
