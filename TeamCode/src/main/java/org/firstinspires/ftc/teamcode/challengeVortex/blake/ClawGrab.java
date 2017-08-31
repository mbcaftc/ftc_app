package org.firstinspires.ftc.teamcode.challengeVortex.blake;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by blake_shafer on 9/26/16.
 */

@TeleOp(name = "Claw Grab")
@Disabled
public class ClawGrab extends OpMode {

    Servo leftGripper;
    Servo rightGripper;

    //leftGripper ranges from 0.0 (closed) to 0.5 (open)
    //rightGripper ranges from 1.0 (closed) to 0.5 (open)

    final static double leftGripperStartPosition = 0.0;
    final static double rightGripperStartPosition = 1.0;

    double leftServoMotor;
    double rightServoMotor;

    @Override
    public void init() {

        leftGripper = hardwareMap.servo.get("left_hand");
        rightGripper = hardwareMap.servo.get("right_hand");
        leftGripper.setPosition(leftGripperStartPosition);
        rightGripper.setPosition(rightGripperStartPosition);
    }

    @Override
    public void loop() {

        // x opens
        // y closes

        //opening
        if (gamepad2.x) {
            leftServoMotor = leftServoMotor + 0.01;
            rightServoMotor = rightServoMotor - 0.01;
            leftGripper.setPosition(leftServoMotor);
            rightGripper.setPosition(rightServoMotor);
        }

        //closing
        if (gamepad2.y) {
            leftServoMotor = leftServoMotor - 0.01;
            rightServoMotor = rightServoMotor + 0.01;
            leftGripper.setPosition(leftServoMotor);
            rightGripper.setPosition(rightServoMotor);
        }

        if (leftServoMotor < 0.0) { // if left servo exceeds fully closed position
            leftServoMotor = 0.0; // bring value back to fully closed position
        }

        if (leftServoMotor > 0.5) { // if left servo exceeds fully open position
            leftServoMotor = 0.5; // bring value back to fully open position
        }

        if (rightServoMotor < 0.5) { // if right servo exceeds fully open position
            rightServoMotor = 0.5; // bring value back to fully open position
        }

        if (rightServoMotor > 1.0) { // if right servo exceeds fully closed position
            rightServoMotor = 1.0; // bring value back to fully closed position
        }

        // Telemetry

        telemetry.addData("claw", " left position:  " + String.format("%.2f", leftServoMotor));
        telemetry.addData("claw", " right position:  " + String.format("%.2f", rightServoMotor));
    }
}
