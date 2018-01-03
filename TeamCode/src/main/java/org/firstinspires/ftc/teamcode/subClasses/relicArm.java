package org.firstinspires.ftc.teamcode.subClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

/**
 * Created by blake_shafer on 10/16/17.
 */

public class relicArm {

    public DcMotor relicArmLift;
    public DcMotor relicArmExtension;
    public Servo relicGrabber;

    public relicArm(DcMotor rAL, DcMotor rAE, Servo rG) {
        relicArmLift = rAL;
        relicArmExtension = rAE;
        relicGrabber = rG;

        relicArmLift.setDirection(DcMotor.Direction.REVERSE);
        relicArmLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicArmLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        relicArmExtension.setDirection(DcMotor.Direction.REVERSE);
        relicArmExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicArmExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicArmExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLiftPower(double liftPower) {
        relicArmLift.setPower(liftPower);
    }

    public void setExtensionPower(double extensionPower) {
        relicArmLift.setPower(extensionPower);
    }

    public void setGrabberPosition(double position) {
        relicGrabber.setPosition(position);
    }
}