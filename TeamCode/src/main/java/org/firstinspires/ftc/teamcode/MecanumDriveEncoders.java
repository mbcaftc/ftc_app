package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by blake_shafer on 8/23/17.
 */

@TeleOp(name = "Mecanum Drive Encoders")

// Crazy encoder action

public class MecanumDriveEncoders extends OpMode {

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

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

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

        // left stick y axis controls forward/backward rotation of left motors
        // right stick y axis controls forward/backward rotation of right motors (tank drive)
        // left/right triggers control strafing left/right

        leftStickVal = -gamepad1.left_stick_y;
        leftStickVal = Range.clip(leftStickVal, -1, 1);
        rightStickVal = -gamepad1.right_stick_y;
        rightStickVal = Range.clip(rightStickVal, -1, 1);

        leftTriggerVal = gamepad1.left_trigger;
        leftTriggerVal = Range.clip(leftTriggerVal, 0, 1);
        rightTriggerVal = gamepad1.right_trigger;
        rightTriggerVal = Range.clip(rightTriggerVal, 0, 1);

        frontLeftSpeed = leftStickVal - leftTriggerVal;
        frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);
        frontRightSpeed = rightStickVal - rightTriggerVal;
        frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
        rearLeftSpeed = leftStickVal + leftTriggerVal;
        rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);
        rearRightSpeed = rightStickVal + rightTriggerVal;
        rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

        frontLeftMotor.setPower(frontLeftSpeed);
        frontRightMotor.setPower(frontRightSpeed);
        rearLeftMotor.setPower(rearLeftSpeed);
        rearRightMotor.setPower(rearRightSpeed);

        // Telemetry

        telemetry.addData("val", "L stck: " + String.format("%.2f", leftStickVal));
        telemetry.addData("val", "R stck: " + String.format("%.2f", rightStickVal));
        telemetry.addData("val", "L trgr: " + String.format("%.2f", leftTriggerVal));
        telemetry.addData("val", "R trgr: " + String.format("%.2f", rightTriggerVal));

        telemetry.addData("pwr", "FL mtr: " + String.format("%.2f", frontLeftSpeed));
        telemetry.addData("pwr", "FR mtr: " + String.format("%.2f", frontRightSpeed));
        telemetry.addData("pwr", "RL mtr: " + String.format("%.2f", rearLeftSpeed));
        telemetry.addData("pwr", "RR mtr: " + String.format("%.2f", rearRightSpeed));
    }
}
