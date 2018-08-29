package org.firstinspires.ftc.teamcode.challenge201617VelocityVortex.blake;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by mbca on 12/9/16.
 */

@Autonomous(name = "Wall - Go To Vortex LEFT")
@Disabled

public class AutonomousWallGoToVortexLEFT extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

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

    Servo yogaArm;

    double yogaArmDownPosition = 0.825;
    double yogaArmUpPosition = 0.215;

    final static double leftArmDownPosition = 0.898;
    final static double leftArmUpPosition = 0.349;
    final static double rightArmDownPosition = 0.0;
    final static double rightArmUpPosition = 0.64;

    @Override
    public void runOpMode() throws InterruptedException {

        leftDriveMotor = hardwareMap.dcMotor.get("left_drive");
        rightDriveMotor = hardwareMap.dcMotor.get("right_drive");
        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        leftLauncherMotor = hardwareMap.dcMotor.get("left_launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right_launcher");
        rightLauncherMotor.setDirection(DcMotor.Direction.REVERSE);

        yogaArm = hardwareMap.servo.get("yoga_arm");
        yogaArm.setPosition(yogaArmDownPosition);

        beltMotor = hardwareMap.dcMotor.get("conveyor_belt");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {


            leftDriveMotor.setPower(-.5);
            rightDriveMotor.setPower(-.5);
//distance to first line to launch
            sleep(375);

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            while (leftLaunchPower < maximumLauncherPower || rightLaunchPower < maximumLauncherPower) {
                if (leftLaunchPower < maximumLauncherPower) {
                    leftLaunchPower = leftLaunchPower + launchStartPowerIncrement;
                }
                if (rightLaunchPower < maximumLauncherPower) {
                    rightLaunchPower = rightLaunchPower + launchStartPowerIncrement;
                }
                leftLauncherMotor.setPower(leftLaunchPower);
                rightLauncherMotor.setPower(rightLaunchPower);
            }

//time for launcher to start
            sleep(1000);

            beltMotor.setPower(maximumForwardBeltPower);

// time for belt to run
            sleep(7000);

            beltMotor.setPower(0);

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

            sleep(200);

            leftDriveMotor.setPower(.3);
            rightDriveMotor.setPower(-.3);
//time to turn RIGHT
            sleep(150);
            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);
//sits still after turning before moving forward
            sleep(300);


            leftDriveMotor.setPower(-.5);
            rightDriveMotor.setPower(-.5);

// time to drive from luanch point to center vortex
            sleep(1350);

//stops OpMode
            requestOpModeStop();
        }
    }
}

