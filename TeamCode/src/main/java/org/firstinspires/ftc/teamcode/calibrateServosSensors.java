package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subClasses.colorSensorArmAuto;
import org.firstinspires.ftc.teamcode.subClasses.glyphArms;

/**
 * Created by johnduval on 10/9/17.
 */
@TeleOp (name = "Calibrate Servos + Sensors")
//@Disabled
public class calibrateServosSensors extends OpMode {
    glyphArms myGlyphArms;
    colorSensorArmAuto myColorSensorArm;
    @Override
    public void init() {
        myGlyphArms = new glyphArms(hardwareMap.servo.get("left_glyph_arm"), hardwareMap.servo.get("right_glyph_arm"));
        myColorSensorArm = new colorSensorArmAuto(hardwareMap.servo.get("color_sensor_arm"),hardwareMap.colorSensor.get("sensor_color"));
        myColorSensorArm.colorSensorArmUp();
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_left) {
            myGlyphArms.closeGlyphArms();
        }
        if (gamepad2.dpad_right) {
            myGlyphArms.openGlyphArms();
        }
        if (gamepad2.dpad_up) {
            myColorSensorArm.colorSensorArmUp();
        }
        if (gamepad2.dpad_down) {
            myColorSensorArm.colorSensorArmDown();
        }
        telemetry.addData("Red  ", myColorSensorArm.colorSensor.red());
        telemetry.addData("Blue ", myColorSensorArm.colorSensor.blue());
        telemetry.addData("Color Servo pos:", myColorSensorArm.colorSensorArm.getPosition());
        telemetry.addData("Left glyph arm pos: ", myGlyphArms.leftGlyphArm.getPosition());
        telemetry.addData("Right glyph arm pos: ", myGlyphArms.rightGlyphArm.getPosition());
        telemetry.update();
    }
}
