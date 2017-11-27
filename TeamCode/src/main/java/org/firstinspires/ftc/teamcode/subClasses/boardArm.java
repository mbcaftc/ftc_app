package org.firstinspires.ftc.teamcode.subClasses;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by blake_shafer on 11/22/17.
 */

public class boardArm {

    public Servo boardArm;

    double servoArmUpPosition = 1.0;
    double servoArmDownPosition = 0.15;

    public boardArm(Servo bA) {
        boardArm = bA;
    }

    public void boardArmUp() {
        boardArm.setPosition(servoArmUpPosition);
    }

    public void boardArmDown() {
        boardArm.setPosition(servoArmDownPosition);
    }
}
