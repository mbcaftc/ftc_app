package org.firstinspires.ftc.teamcode.challengeVortex.blake;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by blake_shafer on 9/26/16.
 */

@TeleOp(name = "Drive Tank Squared")
@Disabled
public class DriveTankSquared extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    double rightStickDriveVal;
    double leftStickDriveVal;
    double rightSquaredDriveVal;
    double leftSquaredDriveVal;

    @Override
    public void init() {

        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        rightStickDriveVal = -gamepad1.right_stick_y;
        leftStickDriveVal = -gamepad1.left_stick_y;
        rightSquaredDriveVal = rightStickDriveVal * rightStickDriveVal;
        leftSquaredDriveVal = leftStickDriveVal * leftStickDriveVal;

        leftSquaredDriveVal = Range.clip(leftSquaredDriveVal, -1, 1);
        rightSquaredDriveVal = Range.clip(rightSquaredDriveVal, -1, 1);

        if (rightStickDriveVal < 0) {
            rightMotor.setPower(-rightSquaredDriveVal);
        } else {
            rightMotor.setPower(rightSquaredDriveVal);
        }

        if (leftStickDriveVal < 0) {
            leftMotor.setPower(-leftSquaredDriveVal);
        } else {
            leftMotor.setPower(leftSquaredDriveVal);
        }

        // Telemetry

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("drive", "left stick input: " + String.format("%.2f", leftStickDriveVal));

        if (leftStickDriveVal < 0) {
            telemetry.addData("drive", "left squared pwr: " + String.format("%.2f", -leftSquaredDriveVal));
        } else {
            telemetry.addData("drive", "left squared pwr: " + String.format("%.2f", leftSquaredDriveVal));
        }

        telemetry.addData("drive", "right stick input: " + String.format("%.2f", rightStickDriveVal));

        if (rightStickDriveVal < 0) {
            telemetry.addData("drive", "right squared pwr: " + String.format("%.2f", -rightSquaredDriveVal));
        } else {
            telemetry.addData("drive", "right squared pwr: " + String.format("%.2f", rightSquaredDriveVal));
        }
    }
}
