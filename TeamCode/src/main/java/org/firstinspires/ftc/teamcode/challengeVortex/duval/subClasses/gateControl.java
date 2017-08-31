package org.firstinspires.ftc.teamcode.challengeVortex.duval.subClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by johnduval on 11/30/16.
 */
public class gateControl {
    //private DcMotor gateMotor;

    /*public gateControl (DcMotor motor) {
        gateMotor = motor;
    }*/

    //controls gate opening
    public void gateOpen(DcMotor motor) {
        motor.setPower(-0.1);
    }
    //controls gate closing
    public void gateClose (DcMotor motor) {
        motor.setPower(0.1);
    }

    //Stops the motor when the gate controller is not engaged
    public void  gateStop (DcMotor motor) {
        motor.setPower(0);
    }
}
