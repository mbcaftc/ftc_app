package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by blake_shafer on 8/23/17.
 */

@TeleOp(name = "Mecanum Drive")

// Plan to apply encoder clicks per second to motors rather than power (power varies with battery voltage)
// Encoder clicks per second will not vary with battery voltage (potential maximum encoder clicks per second will decrease as battery voltage decreases)
// Encoder clicks per second will allow rotation of motors to be controlled extremely precisely

public class MecanumDrive extends OpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;

    double leftStickVal;
    double rightStickVal;

    double leftTriggerVal;
    double rightTriggerVal;

    double frontLeftPower;
    double frontRightPower;
    double rearLeftPower;
    double rearRightPower;

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        rearLeftMotor = hardwareMap.dcMotor.get("rear_left_motor");
        rearRightMotor = hardwareMap.dcMotor.get("rear_right_motor");

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);
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

        frontLeftPower = leftStickVal - leftTriggerVal;
        frontLeftPower = Range.clip(frontLeftPower, -1, 1);
        frontRightPower = rightStickVal - rightTriggerVal;
        frontRightPower = Range.clip(frontRightPower, -1, 1);
        rearLeftPower = leftStickVal + leftTriggerVal;
        rearLeftPower = Range.clip(rearLeftPower, -1, 1);
        rearRightPower = rightStickVal + rightTriggerVal;
        rearRightPower = Range.clip(rearRightPower, -1, 1);

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        rearLeftMotor.setPower(rearLeftPower);
        rearRightMotor.setPower(rearRightPower);

        // Telemetry

        telemetry.addData("val", "left stick: " + String.format("%.2f", leftStickVal));
        telemetry.addData("val", "right stick: " + String.format("%.2f", rightStickVal));
        telemetry.addData("val", "left trigger: " + String.format("%.2f", leftTriggerVal));
        telemetry.addData("val", "right trigger: " + String.format("%.2f", rightTriggerVal));

        telemetry.addData("pwr", "front left mtr: " + String.format("%.2f", frontLeftPower));
        telemetry.addData("pwr", "front right mtr: " + String.format("%.2f", frontRightPower));
        telemetry.addData("pwr", "rear left mtr: " + String.format("%.2f", rearLeftPower));
        telemetry.addData("pwr", "rear right mtr: " + String.format("%.2f", rearRightPower));
    }
}
