package org.firstinspires.ftc.teamcode.challenge201617VelocityVortex.duval.subClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by mbca on 1/17/17.
 */

public class LauncherControlFull {

    private double beltPower = 0;
    private double beltStartPowerIncrement = 0.05;
    private double beltStopPowerIncrement = 0.01;
    private double maximumForwardBeltPower = 0.5;
    private double minimumBeltPower = 0;
    private double maximumReverseBeltPower = -0.5;
    private double forwardRunningBeltPower = 0.1;
    private double reverseRunningBeltPower = -0.1;

    private double leftLaunchPower = 0;
    private double rightLaunchPower = 0;
    private double launchStartPowerIncrement = 0.004;
    private double launchStopPowerIncrement = 0.0005;
    private double maximumLauncherPowerBlackBattery = 0.31;
    private double maximumLauncherPowerWhiteBattery = 0.32;
    private double minimumLauncherPower = 0;
    private double manualPowerIncrement = 0.05;

    private DcMotor leftLauncherMotor, rightLauncherMotor, beltMotor;

    public LauncherControlFull(DcMotor left_launcher, DcMotor right_launcher, DcMotor conveyor_belt) {
        leftLauncherMotor = left_launcher;
        rightLauncherMotor = right_launcher;
        rightLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        beltMotor = conveyor_belt;

    }

    public void startLauncherWhiteBattery () {
        while (leftLaunchPower < maximumLauncherPowerWhiteBattery || rightLaunchPower < maximumLauncherPowerWhiteBattery) {
            if (leftLaunchPower < maximumLauncherPowerWhiteBattery) {
                leftLaunchPower = leftLaunchPower + launchStartPowerIncrement;
            }
            if (rightLaunchPower < maximumLauncherPowerWhiteBattery) {
                rightLaunchPower = rightLaunchPower + launchStartPowerIncrement;
            }
            leftLauncherMotor.setPower(leftLaunchPower);
            rightLauncherMotor.setPower(rightLaunchPower);
        }
    }


    public void startLauncherBlackBattery () {
        while (leftLaunchPower < maximumLauncherPowerBlackBattery || rightLaunchPower < maximumLauncherPowerBlackBattery) {
            if (leftLaunchPower < maximumLauncherPowerBlackBattery) {
                leftLaunchPower = leftLaunchPower + launchStartPowerIncrement;
            }
            if (rightLaunchPower < maximumLauncherPowerBlackBattery) {
                rightLaunchPower = rightLaunchPower + launchStartPowerIncrement;
            }
            leftLauncherMotor.setPower(leftLaunchPower);
            rightLauncherMotor.setPower(rightLaunchPower);
        }


    }

    public void stopLauncher () {
        while (leftLaunchPower > 0 || rightLaunchPower > 0) {
            if (leftLaunchPower > 0) {
                leftLaunchPower = leftLaunchPower - launchStopPowerIncrement;
            }
            if (rightLaunchPower > 0) {
                rightLaunchPower = rightLaunchPower - launchStopPowerIncrement;
            }
            leftLauncherMotor.setPower(leftLaunchPower);
            rightLauncherMotor.setPower(rightLaunchPower);
        }
    }

    public void conveyorBeltStart () {
        beltMotor.setPower(maximumForwardBeltPower);
    }

    public void conveyorBeltStop () {
        beltMotor.setPower(minimumBeltPower);
    }

    public void conveyorBeltReverse () {
        beltMotor.setPower(maximumReverseBeltPower);
    }
}
