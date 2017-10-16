package org.firstinspires.ftc.teamcode.subClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by blake_shafer on 10/16/17.
 */

public class glyphLift {

    public DcMotor glyphLift;



    public glyphLift(DcMotor gL) {
        glyphLift = gL;
    }

    public void setPower(double power) {
        glyphLift.setPower(power);
    }
}