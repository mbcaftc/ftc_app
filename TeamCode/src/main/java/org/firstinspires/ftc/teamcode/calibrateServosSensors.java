package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.extras.colorSensorArmAuto;
import org.firstinspires.ftc.teamcode.subClasses.glyphArms;
import org.firstinspires.ftc.teamcode.subClasses.glyphLift;

/**
 * Created by johnduval on 10/9/17.
 */
@TeleOp (name = "Calibrate Servos + Sensors")
//@Disabled
public class calibrateServosSensors extends OpMode {
    glyphArms myGlyphArms;
    colorSensorArmAuto myColorSensorArm;
    glyphLift myGlyphLift;
    double leftStickVal2;




    @Override
    public void init() {
        myGlyphArms = new glyphArms(hardwareMap.servo.get("left_glyph_arm"), hardwareMap.servo.get("right_glyph_arm"));
        myColorSensorArm = new colorSensorArmAuto(hardwareMap.servo.get("color_sensor_arm"),hardwareMap.colorSensor.get("sensor_color"));
        myGlyphLift = new glyphLift(hardwareMap.dcMotor.get("glyph_lift"));

        myColorSensorArm.colorSensorArmUp();
        //myGlyphArms.openGlyphArms();
    }

    @Override
    public void loop() {
        if (gamepad2.right_bumper) {
            myGlyphArms.closeGlyphArms();
        }
        if (gamepad2.a) {
            myGlyphArms.openLoweredGlyphArms();
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
            myColorSensorArm.colorSensorArmUp();
        }
        if (gamepad2.dpad_down) {
            myColorSensorArm.colorSensorArmDown();
        }

        leftStickVal2 = -gamepad2.left_stick_y;
        leftStickVal2 = Range.clip(leftStickVal2, -1, 1);
        myGlyphLift.setPower(leftStickVal2);

        telemetry.addData("Red  ", myColorSensorArm.colorSensor.red());
        telemetry.addData("Blue ", myColorSensorArm.colorSensor.blue());
        telemetry.addData("Color Servo pos:", myColorSensorArm.colorSensorArm.getPosition());
        telemetry.addData("Left glyph arm pos: ", myGlyphArms.leftGlyphArm.getPosition());
        telemetry.addData("Right glyph arm pos: ", myGlyphArms.rightGlyphArm.getPosition());
        telemetry.update();
    }
}
