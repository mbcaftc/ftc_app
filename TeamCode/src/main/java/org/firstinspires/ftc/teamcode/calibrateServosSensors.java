package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.extras.colorSensorArmAuto;
import org.firstinspires.ftc.teamcode.subClasses.colorSensorArm;
import org.firstinspires.ftc.teamcode.subClasses.glyphArms;
import org.firstinspires.ftc.teamcode.subClasses.glyphLift;
import org.firstinspires.ftc.teamcode.subClasses.revColorDistanceSensor;

import java.util.Locale;

/**
 * Created by johnduval on 10/9/17.
 */
@Disabled
@TeleOp (name = "Calibrate Servos + Sensors")

public class calibrateServosSensors extends OpMode {
    glyphArms myGlyphArms;
    glyphLift myGlyphLift;
    colorSensorArm myColorSensorArm;
    //revColorDistanceSensor myRevColorDistanceSensor;


    double leftStickVal2;

    @Override
    public void init() {
        myGlyphArms = new glyphArms(hardwareMap.servo.get("left_glyph_arm"), hardwareMap.servo.get("right_glyph_arm"));
        myColorSensorArm = new colorSensorArm(hardwareMap.servo.get("color_sensor_arm"),hardwareMap.colorSensor.get("sensor_color"), hardwareMap.servo.get("color_sensor_arm_rotate"));
        myGlyphLift = new glyphLift(hardwareMap.dcMotor.get("glyph_lift"));
        //myRevColorDistanceSensor =  new revColorDistanceSensor(hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance"));
    }

    @Override
    public void loop() {
        if (gamepad2.right_bumper) {
            myGlyphArms.closeGlyphArms();
        }
        if (gamepad2.a) {
            myGlyphArms.openGlyphArms();
        }
        if (gamepad2.y) {
            myGlyphArms.openRaisedGlyphArms();
        }
        if (gamepad2.x) {
            myGlyphArms.slightlyOpenGlyphArms();
        }
        if (gamepad2.b) {
            myGlyphArms.slightlyOpenGlyphArms();
        }
        if (gamepad2.dpad_up) {
            try {
                myColorSensorArm.colorSensorArmUpSlow();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        if (gamepad2.dpad_down) {
            try {
                myColorSensorArm.colorSensorArmDownSlow();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        if(gamepad2.dpad_left) {
            myColorSensorArm.colorRotateClockwise();
        }

        if (gamepad2.dpad_right) {
            myColorSensorArm.colorRotateCounterClockwise();
        }

        leftStickVal2 = -gamepad2.left_stick_y;
        leftStickVal2 = Range.clip(leftStickVal2, -1, 1);
        myGlyphLift.setPower(leftStickVal2);


        /*
        if (myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH) >= 4) {
            myColorSensorArm.colorRotateCounterClockwiseRed1Blue2();
        } else if (myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH) < 4) {
            myColorSensorArm.colorRotateClockwiseRed1Blue2();
        } else {
            myColorSensorArm.colorRotateResting();
        }

        //telemetry.addData("Red  ", myColorSensorArm.colorSensor.red());
        //telemetry.addData("Blue ", myColorSensorArm.colorSensor.blue());
        telemetry.addData("Distance (mm)",
                String.format(Locale.US, "%.02f", myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.MM)));
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.CM)));
        telemetry.addData("Distance (meter)",
                String.format(Locale.US, "%.02f", myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.METER)));
        telemetry.addData("Distance (in)",
                String.format(Locale.US, "%.02f", myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH))); */
        telemetry.addData("BLUE: ", myColorSensorArm.colorSensor.blue());
        telemetry.addData("RED: ", myColorSensorArm.colorSensor.red());
        telemetry.addData("Color Servo pos:", myColorSensorArm.colorSensorArm.getPosition());
        telemetry.addData("Left glyph arm pos: ", myGlyphArms.leftGlyphArm.getPosition());
        telemetry.addData("Right glyph arm pos: ", myGlyphArms.rightGlyphArm.getPosition());
        telemetry.update();
    }
}
