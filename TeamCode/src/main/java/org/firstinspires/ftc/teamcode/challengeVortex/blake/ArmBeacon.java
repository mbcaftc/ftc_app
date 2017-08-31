package org.firstinspires.ftc.teamcode.challengeVortex.blake;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by blake_shafer on 9/27/16.
 */

@TeleOp(name = "Claw Beacon Press")

public class ArmBeacon extends OpMode {

    Servo leftArm;
    Servo rightArm;

    //leftFinger ranges from 0.0 (forward) to 0.5 (retracted)
    //rightFinger ranges from 1.0 (forward) to 0.5 (retracted)

    final static double leftArmUpPosition = 0.5;
    final static double rightArmUpPosition = 1.0;
    final static double leftArmDownPosition = 1.0;
    final static double rightArmDownPosition = 0.5;

    boolean chooseArm; //true accesses left arm, false accesses right arm

    @Override
    public void init() {

        leftArm = hardwareMap.servo.get("left_arm");
        rightArm = hardwareMap.servo.get("right_arm");
        leftArm.setPosition(leftArmDownPosition);
        rightArm.setPosition(rightArmDownPosition);
    }

    @Override
    public void loop() {

        if (gamepad2.dpad_left) {
            chooseArm = true;
        }

        if (gamepad2.dpad_right) {
            chooseArm = false;
        }

        if (chooseArm) {
            if (gamepad2.dpad_up) {
                leftArm.setPosition(leftArmUpPosition);
            }

            if (gamepad2.dpad_down) {
                leftArm.setPosition(leftArmDownPosition);
            }
        }

        if (!chooseArm) {
            if (gamepad2.dpad_up) {
                rightArm.setPosition(rightArmUpPosition);
            }

            if (gamepad2.dpad_down) {
                rightArm.setPosition(rightArmDownPosition);
            }
        }

        // Telemetry

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("arm", "left position: ", leftArm.getPosition());
        telemetry.addData("arm", "right position: ", rightArm.getPosition());
    }
}
