package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by student on 12/6/17.
 */
@TeleOp(name = "test servo")
public class testServo extends OpMode {
    Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.servo.get("servoTest");
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_up) {
            servo.setPosition(1);
        }
        if (gamepad2.dpad_right) {
            servo.setPosition(.60);
        }
        if (gamepad2.dpad_down) {
            servo.setPosition(.40);
        }
        if (gamepad2.dpad_left) {
            servo.setPosition(0);
        }
    }
}
