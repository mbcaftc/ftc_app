package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.extras.colorSensorArmAuto;
import org.firstinspires.ftc.teamcode.subClasses.glyphArms;
import org.firstinspires.ftc.teamcode.subClasses.glyphLift;

/**
 * Created by blake_shafer on 8/23/17.
 */

@TeleOp(name = "Full Control")

public class fullControlTeleOp extends OpMode {

    // left stick y axis controls forward/backward rotation of left motors
    // right stick y axis controls forward/backward rotation of right motors (tank drive)
    // left/right triggers control strafing left/right

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

    double altFrontLeftSpeed; // are the altered speed values after the original speed values have been multiplied by the speed threshold factors
    double altFrontRightSpeed;
    double altRearLeftSpeed;
    double altRearRightSpeed;

    double speedFastFactor = 0.8;
    double speedSlowFactor = 0.5;
    double speedPullUpVal = 0.8;
    double digitalJoystickVal = 0.1;

    double leftStickVal2;

    boolean speedState; // True = fast, false = slow

    glyphArms myGlyphArms;
    colorSensorArmAuto myColorSensorArm;
    glyphLift myGlyphLift;

    @Override
    public void init() {

        myGlyphLift = new glyphLift(hardwareMap.dcMotor.get("glyph_lift"));

        myGlyphArms = new glyphArms(hardwareMap.servo.get("left_glyph_arm"), hardwareMap.servo.get("right_glyph_arm"));
        myGlyphArms.openGlyphArms();

        myColorSensorArm = new colorSensorArmAuto(hardwareMap.servo.get("color_sensor_arm"),hardwareMap.colorSensor.get("sensor_color"));
        myColorSensorArm.colorSensorArmUp();

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

        speedState = true;
    }
    @Override
    public void loop() {

        // Mecanum Drive

        leftStickVal = -gamepad1.left_stick_y;
        leftStickVal = Range.clip(leftStickVal, -1, 1);
        rightStickVal = -gamepad1.right_stick_y;
        rightStickVal = Range.clip(rightStickVal, -1, 1);

        leftTriggerVal = gamepad1.left_trigger;
        leftTriggerVal = Range.clip(leftTriggerVal, 0, 1);
        rightTriggerVal = gamepad1.right_trigger;
        rightTriggerVal = Range.clip(rightTriggerVal, 0, 1);

        frontLeftSpeed = leftStickVal - leftTriggerVal + rightTriggerVal;
        frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

        frontRightSpeed = rightStickVal - rightTriggerVal + leftTriggerVal;
        frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

        rearLeftSpeed = leftStickVal + leftTriggerVal - rightTriggerVal;
        rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

        rearRightSpeed = rightStickVal + rightTriggerVal - leftTriggerVal;
        rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

        if (gamepad1.x) {
            speedState = false;
        }
        if (gamepad1.y) {
            speedState = true;
        }

        if (speedState) {

            if (leftStickVal >= speedPullUpVal) {
                leftStickVal = 1.0;
            }
            else if (rightStickVal >= speedPullUpVal) {
                rightStickVal = 1.0;
            }

            altFrontLeftSpeed = frontLeftSpeed * speedFastFactor;
            frontLeftMotor.setPower(altFrontLeftSpeed);

            altFrontRightSpeed = frontRightSpeed * speedFastFactor;
            frontRightMotor.setPower(altFrontRightSpeed);

            altRearLeftSpeed = rearLeftSpeed * speedFastFactor;
            rearLeftMotor.setPower(altRearLeftSpeed);

            altRearRightSpeed = rearRightSpeed * speedFastFactor;
            rearRightMotor.setPower(altRearRightSpeed);
        }
        if (!speedState) {

            if (leftStickVal >= digitalJoystickVal) {
                leftStickVal = speedSlowFactor;
            }
            else if (rightStickVal >= digitalJoystickVal) {
                rightStickVal = speedSlowFactor;
            }
            else if (leftStickVal <= -digitalJoystickVal) {
                leftStickVal = -speedSlowFactor;
            }
            else if (rightStickVal <= -digitalJoystickVal) {
                rightStickVal = -speedSlowFactor;
            }
            else if (leftStickVal < digitalJoystickVal && leftStickVal > 0) {
                leftStickVal = 0;
            }
            else if (rightStickVal < digitalJoystickVal && rightStickVal > 0) {
                rightStickVal = 0;
            }
            else if (leftStickVal > -digitalJoystickVal && leftStickVal < 0) {
                leftStickVal = 0;
            }
            else if (rightStickVal > -digitalJoystickVal && rightStickVal < 0) {
                rightStickVal = 0;
            }

            altFrontLeftSpeed = frontLeftSpeed;
            frontLeftMotor.setPower(altFrontLeftSpeed);

            altFrontRightSpeed = frontRightSpeed;
            frontRightMotor.setPower(altFrontRightSpeed);

            altRearLeftSpeed = rearLeftSpeed;
            rearLeftMotor.setPower(altRearLeftSpeed);

            altRearRightSpeed = rearRightSpeed;
            rearRightMotor.setPower(altRearRightSpeed);
        }

        // Glyph Arms

        if (gamepad2.left_bumper) {
            myGlyphArms.openGlyphArms();
        }
        if (gamepad2.right_bumper) {
            myGlyphArms.closeGlyphArms();
        }
        if (gamepad2.a) {
            myGlyphArms.slightlyOpenGlyphArms();
        }

        // Color Sensor Arm

        if (gamepad2.dpad_down) {
            myColorSensorArm.colorSensorArmDown();
        }
        if (gamepad2.dpad_up) {
            myColorSensorArm.colorSensorArmUp();
        }

        // Glyph Lift

        leftStickVal2 = -gamepad2.left_stick_y;
        leftStickVal2 = Range.clip(leftStickVal2, -1, 1);
        myGlyphLift.setPower(leftStickVal2);

        // Telemetry

        telemetry.addData("val", "L stck: " + String.format("%.2f", leftStickVal));
        telemetry.addData("val", "R stck: " + String.format("%.2f", rightStickVal));
        telemetry.addData("val", "L trgr: " + String.format("%.2f", leftTriggerVal));
        telemetry.addData("val", "R trgr: " + String.format("%.2f", rightTriggerVal));

        telemetry.addData("pwr", "FL mtr: " + String.format("%.2f", altFrontLeftSpeed));
        telemetry.addData("pwr", "FR mtr: " + String.format("%.2f", altFrontRightSpeed));
        telemetry.addData("pwr", "RL mtr: " + String.format("%.2f", altRearLeftSpeed));
        telemetry.addData("pwr", "RR mtr: " + String.format("%.2f", altRearRightSpeed));
    }
}
