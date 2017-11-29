package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.extras.colorSensorArmAuto;
import org.firstinspires.ftc.teamcode.subClasses.boardArm;
import org.firstinspires.ftc.teamcode.subClasses.colorSensorArm;
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

    double liftPower;
    int position;
    int minLiftPosition = 0;
    int maxLiftPosition = 9200;

    boolean initServos = false;

    glyphArms myGlyphArms;
    colorSensorArm myColorSensorArm;
    glyphLift myGlyphLift;
    boardArm myBoardArm;

    @Override
    public void init() {

        myGlyphLift = new glyphLift(hardwareMap.dcMotor.get("glyph_lift"));
        myGlyphArms = new glyphArms(hardwareMap.servo.get("left_glyph_arm"), hardwareMap.servo.get("right_glyph_arm"));
        myColorSensorArm = new colorSensorArm(hardwareMap.servo.get("color_sensor_arm"),hardwareMap.colorSensor.get("sensor_color"), hardwareMap.servo.get("color_sensor_arm_rotate"));
        myBoardArm = new boardArm(hardwareMap.servo.get("board_arm"));

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

       // frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop() {

        if (!initServos) {
            myGlyphArms.openGlyphArms();
            myColorSensorArm.colorSensorArmUp();
            myColorSensorArm.colorRotateResting();
            myBoardArm.boardArmUp();
            initServos = true;
        }

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

        frontLeftMotor.setPower(frontLeftSpeed);
        frontRightMotor.setPower(frontRightSpeed);
        rearLeftMotor.setPower(rearLeftSpeed);
        rearRightMotor.setPower(rearRightSpeed);

        // Glyph Arms

        if (gamepad2.left_bumper) {
            //myGlyphArms.openGlyphArms();
            //changed to make sure panels don't hit robot frame.
            myGlyphArms.openGlyphArms();
        }
        if (gamepad2.right_bumper) {
            myGlyphArms.closeGlyphArms();
        }
        if (gamepad2.a) {
            myGlyphArms.slightlyOpenGlyphArms();
        }


        if (gamepad2.x) {
            myGlyphArms.openRaisedGlyphArms();
        }

        // Color Sensor Arm

//        if (gamepad2.dpad_up) {
//            myColorSensorArm.colorSensorArmRest();
//        }

        // Glyph Lift

        liftPower = -gamepad2.left_stick_y;
        liftPower = Range.clip(liftPower, -1, 1);

        myGlyphLift.setPower(liftPower);

        // Board Arm

        if (gamepad1.a || gamepad2.dpad_down) {
            myBoardArm.boardArmDown();
        }

        else if (gamepad1.y || gamepad2.dpad_up) {
            myBoardArm.boardArmUp();
        }

        // Telemetry

        telemetry.addData("val", "L stck: " + leftStickVal);
        telemetry.addData("val", "R stck: " + rightStickVal);
        telemetry.addData("val", "L trgr: " + leftTriggerVal);
        telemetry.addData("val", "R trgr: " + rightTriggerVal);

        telemetry.addData("pwr", "FL mtr: " + frontLeftSpeed);
        telemetry.addData("pwr", "FR mtr: " + frontRightSpeed);
        telemetry.addData("pwr", "RL mtr: " + rearLeftSpeed);
        telemetry.addData("pwr", "RR mtr: " + rearRightSpeed);

        telemetry.addData("lift", "position: " +  position);
        telemetry.addData("lift", "pwr: " + liftPower);
    }
}
