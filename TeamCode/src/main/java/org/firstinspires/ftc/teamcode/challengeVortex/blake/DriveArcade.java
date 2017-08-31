package org.firstinspires.ftc.teamcode.challengeVortex.blake;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by blake_shafer on 9/26/16.
 */

@TeleOp(name = "Drive Arcade")
@Disabled
public class DriveArcade extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void init() {

        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        double xValue = gamepad1.left_stick_x;
        double yValue = -gamepad1.left_stick_y;

        double leftPower = yValue + xValue;
        double rightPower = yValue - xValue;

        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        // Telemetry

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("drive", "left pwr: " + String.format("%.2f", leftPower));
        telemetry.addData("drive", "right pwr: " + String.format("%.2f", rightPower));
    }
}
