package org.firstinspires.ftc.teamcode.challengeVelocityVortex201617.blake;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by blake_shafer on 11/19/16.
 */

@Autonomous(name = "Red Wall")
@Disabled

public class AutonomousRedWall extends LinearOpMode {

    DcMotor leftDriveMotor;
    DcMotor rightDriveMotor;

    DcMotor leftLauncherMotor;
    DcMotor rightLauncherMotor;

    double leftLaunchPower = 0;
    double rightLaunchPower = 0;
    double launchStartPowerIncrement = 0.004;
    double launchStopPowerIncrement = 0.001;
    double maximumLauncherPower = 0.5;
    double minimumLauncherPower = 0;

    DcMotor beltMotor;

    double beltPower = 0;
    double beltStartPowerIncrement = 0.05;
    double beltStopPowerIncrement = 0.01;
    double maximumForwardBeltPower = 0.3;
    double minimumBeltPower = 0;
    double maximumReverseBeltPower = -1.0;
    double forwardRunningBeltPower = 0.1;
    double reverseRunningBeltPower = -0.1;

    Servo leftArm;
    Servo rightArm;

    final static double leftArmUpPosition = 0.329;
    final static double rightArmUpPosition = 0.48;
    final static double leftArmDownPosition = 0.948;
    final static double rightArmDownPosition = 0.061;

    @Override
    public void runOpMode() throws InterruptedException {

        leftDriveMotor = hardwareMap.dcMotor.get("left_drive");
        rightDriveMotor = hardwareMap.dcMotor.get("right_drive");
        rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        leftLauncherMotor = hardwareMap.dcMotor.get("left_launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right_launcher");
        rightLauncherMotor.setDirection(DcMotor.Direction.REVERSE);

        beltMotor = hardwareMap.dcMotor.get("conveyor_belt");

        waitForStart();

        leftLaunchPower = leftLaunchPower + launchStartPowerIncrement;
        rightLaunchPower = rightLaunchPower + launchStartPowerIncrement;

        if (rightLaunchPower >= maximumLauncherPower) {
            rightLaunchPower = maximumLauncherPower;
        }

        if (leftLaunchPower >= maximumLauncherPower) {
            leftLaunchPower = maximumLauncherPower;
        }

        wait(2500);

        beltPower = beltPower + beltStartPowerIncrement;

        if (beltPower >= maximumForwardBeltPower) {
            beltPower = maximumForwardBeltPower;
        }

        wait(4000);

        leftLaunchPower = leftLaunchPower - launchStopPowerIncrement;
        rightLaunchPower = rightLaunchPower - launchStopPowerIncrement;

        if (rightLaunchPower <= minimumLauncherPower) {
            rightLaunchPower = minimumLauncherPower;
        }

        if (leftLaunchPower <= minimumLauncherPower) {
            leftLaunchPower = minimumLauncherPower;
        }

        beltPower = beltPower - beltStopPowerIncrement;

        if (beltPower <= minimumBeltPower) {
            beltPower = minimumBeltPower;
        }
    }
}

