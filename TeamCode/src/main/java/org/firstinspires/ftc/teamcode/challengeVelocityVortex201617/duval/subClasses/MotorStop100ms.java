package org.firstinspires.ftc.teamcode.challengeVelocityVortex201617.duval.subClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Thread.sleep;

/**
 * Created by johnduval on 12/27/16.
 */

public class MotorStop100ms {
    public void stopMotor (DcMotor motorLeft, DcMotor motorRight) throws InterruptedException {
        motorLeft.setPower(0);
        motorLeft.setPower(0);
        sleep (100);

    }
}
