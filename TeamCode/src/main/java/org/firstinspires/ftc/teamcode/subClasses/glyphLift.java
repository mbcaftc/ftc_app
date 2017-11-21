package org.firstinspires.ftc.teamcode.subClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Thread.sleep;

/**
 * Created by blake_shafer on 10/16/17.
 */

public class glyphLift {

    public DcMotor glyphLift;
    private int autRaiseTime;
    private double autRaisePower;
    private int autLowerTime;
    int position;

    public glyphLift(DcMotor gL) {
        glyphLift = gL;

        glyphLift.setDirection(DcMotor.Direction.REVERSE);
        glyphLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //autRaisePower = 1;      // full power
        //autRaiseTime = 3000;    // 1 second
        autRaisePower = 1.0;      // full power
        autRaiseTime = 500;    // 1 second
    }

    public void setPower(double power) {

        glyphLift.setPower(power);
    }

    public int getCurrentPosition() {
        position = glyphLift.getCurrentPosition();
        return position;
    }

    public void raiseGlyphLiftAutMode () throws InterruptedException {
        glyphLift.setPower(autRaisePower);
        sleep(autRaiseTime);
        glyphLift.setPower(0);
    }

    public void lowerGlyphLiftAutMode () throws InterruptedException {
        glyphLift.setPower(-autRaisePower);
        sleep(autRaiseTime);
        glyphLift.setPower(0);
    }
}