package org.firstinspires.ftc.teamcode.challenge201718RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.challenge201718RelicRecovery.subClasses.glyphArms;
import org.firstinspires.ftc.teamcode.challenge201718RelicRecovery.subClasses.mechDriveAuto;

/**
 * Created by student on 12/6/17.
 */
@TeleOp(name = "Testin - Vuforia Strafing", group = "TESTING")
@Disabled
public class testDrive_CryptoStrafe extends OpMode {
    mechDriveAuto myMechDrive;
    glyphArms myGlyphArms;


    @Override
    public void init() {
        myMechDrive = new mechDriveAuto(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myGlyphArms = new glyphArms(hardwareMap.servo.get("top_left_glyph_arm"), hardwareMap.servo.get("bottom_left_glyph_arm"), hardwareMap.servo.get("top_left_glyph_arm"), hardwareMap.servo.get("bottom_right_glyph_arm"));
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_left) {
            try {
                myMechDrive.vuforiaLeft(myGlyphArms);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        if (gamepad2.dpad_right) {
            try {
                myMechDrive.vuforiaRight(myGlyphArms);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        if (gamepad2.dpad_up) {
            try {
                myMechDrive.vuforiaLeft(myGlyphArms);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        if (gamepad2.a) {
            myGlyphArms.closeGlyphArms();
        }
    }
}
