package org.firstinspires.ftc.teamcode.challengeVortex.blake;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by blake_shafer on 9/26/16.
 */

@TeleOp(name = "Drive Tank")
@Disabled
public class DriveTank extends OpMode {

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

        double leftY = -gamepad1.left_stick_y;
        double rightY = -gamepad1.right_stick_y;

        leftY = Range.clip(leftY, -1, 1);
        rightY = Range.clip(rightY, -1, 1);

        leftMotor.setPower(leftY);
        rightMotor.setPower(rightY);

        // Telemetry

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("drive", "left pwr: " + String.format("%.2f", leftY));
        telemetry.addData("drive", "right pwr: " + String.format("%.2f", rightY));
    }
}
