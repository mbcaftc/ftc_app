package org.firstinspires.ftc.teamcode.challengeVortex.blake;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by blake_shafer on 2/16/17.
 */

@TeleOp(name = "Drive Prototype")
@Disabled

public class Drive2Joysticks extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    double leftStickDriveVal;
    double rightStickTurnVal;
    double leftSquaredDriveVal;
    double rightSquaredTurnVal;
    double leftMotorPower;
    double rightMotorPower;

    double speedLimiterFast = 0.7;
    double speedLimiterSlow = 0.2;

    boolean speedSlowEngaged;
    double speedLimit;

    DcMotor leftLauncherMotor;
    DcMotor rightLauncherMotor;

    double leftLaunchPower = 0;
    double rightLaunchPower = 0;
    double launchStartPowerIncrement = 0.004;
    double launchStopPowerIncrement = 0.001;
    double maximumLauncherPower = 0.5;
    double minimumLauncherPower = 0;
    double manualPowerIncrement = 0.05;

    boolean launchStatus = false;
    boolean fullPowerAllow = true;
    boolean incrementFast = false;
    boolean incrementSlow = false;
    boolean toggleFastAllow = false;
    boolean toggleSlowAllow = false;

    DcMotor beltMotor;

    double beltPower = 0;
    double beltStartPowerIncrement = 0.05;
    double beltStopPowerIncrement = 0.01;
    double maximumForwardBeltPower = 0.3;
    double minimumBeltPower = 0;
    double maximumReverseBeltPower = -0.5;
    double forwardRunningBeltPower = 0.1;
    double reverseRunningBeltPower = -0.1;

    boolean beltForwardStatus;
    boolean beltReverseStatus;

    Servo yogaArm;

    double yogaArmDownPosition = 0.825;
    double yogaArmUpPosition = 0.215;

    final static double leftBeaconArmDownPosition = 0.95; // REMEMBER: These values need to be recalibrated
    final static double leftBeaconArmUpPosition = 0.31;
    final static double rightBeaconArmDownPosition = 0.18;
    final static double rightBeaconArmUpPosition = 0.835;

    boolean leftBeaconArmState = false;
    boolean previousLeftBeaconArmState = true;
    boolean rightBeaconArmState = false;
    boolean previousRightBeaconArmState = true;

    Servo leftGateArm;
    Servo rightGateArm;

    double leftGateArmClosedPosition = .57; // These values are only place-holder values
    double leftGateArmOpenPosition = 0.12; // You will need to experiment with these values to find the perfect servo positions
    double rightGateArmClosedPosition = 0.43;
    double rightGateArmOpenPosition = 0.88;
    double leftServoPosition;
    double rightServoPosition;
    double servoPositionAlterVal = 0.03; // This is the speed by which the servo will change position as long as the left stick is forward/backward
    // Again, this is a place-holder and needs to be experimented with based on how quickly/slowly you want the servo to move
    double stickInputThreshold = 0.2; // Amount of input required on left stick for gate servos to begin opening/closing
    double leftStickYVal;


    @Override

    public void init() {

        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        speedSlowEngaged = false;
        leftLauncherMotor = hardwareMap.dcMotor.get("left_launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right_launcher");
        rightLauncherMotor.setDirection(DcMotor.Direction.REVERSE);

        beltMotor = hardwareMap.dcMotor.get("conveyor_belt");

        leftGateArm = hardwareMap.servo.get("left_gate_arm"); // Don't forget to program left and right _gate_arm into the RC phone as servos
        rightGateArm = hardwareMap.servo.get("right_gate_arm"); // Or else the phone won't be able to communicate with the program (stuff won't work)
        leftGateArm.setPosition(leftGateArmClosedPosition); // Also, while doing this, don't forget to delete the old gate_arm which was the dcMotor registered in the RC phone
        rightGateArm.setPosition(rightGateArmClosedPosition); // Remember that it matters how you plug in the servos to the module; it depends on how you assigned them in the RC phone

        yogaArm = hardwareMap.servo.get("yoga_arm");
        yogaArm.setPosition(yogaArmDownPosition);
    }

    @Override
    public void loop() {

        leftStickDriveVal = -gamepad1.left_stick_y;
        rightStickTurnVal = gamepad1.right_stick_x;

        leftSquaredDriveVal = leftStickDriveVal * leftStickDriveVal;
        rightSquaredTurnVal = rightStickTurnVal * rightStickTurnVal;

        leftSquaredDriveVal = Range.clip(leftSquaredDriveVal, -1, 1);
        rightSquaredTurnVal = Range.clip(rightSquaredTurnVal, -1, 1);

        if (leftStickDriveVal < 0) {
            leftSquaredDriveVal = -leftSquaredDriveVal;
        }

        else if (rightStickTurnVal < 0) {
            rightSquaredTurnVal = -rightSquaredTurnVal;
        }

        leftMotorPower = leftSquaredDriveVal + rightSquaredTurnVal;
        rightMotorPower = leftSquaredDriveVal - rightSquaredTurnVal;

        if (gamepad1.a) {
            speedSlowEngaged = false;
        }

        else if (gamepad1.b) {
            speedSlowEngaged = true;
        }

        if (!speedSlowEngaged) {
            leftMotorPower = leftMotorPower * speedLimiterFast;
            rightMotorPower = rightMotorPower * speedLimiterFast;

            leftMotorPower = Range.clip(leftMotorPower, -speedLimiterFast, speedLimiterFast);
            rightMotorPower = Range.clip(rightMotorPower, -speedLimiterFast, speedLimiterFast);
        } else {
            leftMotorPower = leftMotorPower * speedLimiterSlow;
            rightMotorPower = rightMotorPower * speedLimiterSlow;

            leftMotorPower = Range.clip(leftMotorPower, -speedLimiterSlow, speedLimiterSlow);
            rightMotorPower = Range.clip(rightMotorPower, -speedLimiterSlow, speedLimiterSlow);
        }

        leftMotor.setPower(leftMotorPower);
        rightMotor.setPower(rightMotorPower);

        // Launcher

        if (!gamepad2.left_bumper) {
            incrementSlow = false;
            toggleSlowAllow = true;
        }
        if (gamepad2.left_bumper && toggleSlowAllow) {
            fullPowerAllow = false;
            incrementSlow = true;
        }

        if (!gamepad2.right_bumper) {
            incrementFast = false;
            toggleFastAllow = true;
        }
        if (gamepad2.right_bumper && toggleFastAllow) {
            fullPowerAllow = false;
            incrementFast = true;
        }

        if (gamepad2.y) {
            launchStatus = true;
            fullPowerAllow = true;
        }

        if (gamepad2.x) {
            launchStatus = false;
            fullPowerAllow = true;
        }

        if (launchStatus) {
            if (fullPowerAllow) {
                leftLaunchPower = leftLaunchPower + launchStartPowerIncrement;
                rightLaunchPower = rightLaunchPower + launchStartPowerIncrement;
            }

            if (incrementFast) {
                leftLaunchPower = leftLaunchPower + manualPowerIncrement;
                rightLaunchPower = rightLaunchPower + manualPowerIncrement;
                incrementFast = false;
                toggleFastAllow = false;
            }

            if (incrementSlow) {
                leftLaunchPower = leftLaunchPower - manualPowerIncrement;
                rightLaunchPower = rightLaunchPower - manualPowerIncrement;
                incrementSlow = false;
                toggleSlowAllow = false;
            }

            if (leftLaunchPower >= maximumLauncherPower) {
                leftLaunchPower = maximumLauncherPower;
            }

            if (rightLaunchPower >= maximumLauncherPower) {
                rightLaunchPower = maximumLauncherPower;
            }
            if (leftLaunchPower <= minimumLauncherPower) {
                leftLaunchPower = minimumLauncherPower;
            }

            if (rightLaunchPower <= minimumLauncherPower) {
                rightLaunchPower = minimumLauncherPower;
            }
        }

        if (!launchStatus) {
            leftLaunchPower = leftLaunchPower - launchStopPowerIncrement;
            rightLaunchPower = rightLaunchPower - launchStopPowerIncrement;
            if (leftLaunchPower <= minimumLauncherPower) {
                leftLaunchPower = minimumLauncherPower;
            }

            if (rightLaunchPower <= minimumLauncherPower) {
                rightLaunchPower = minimumLauncherPower;
            }
        }

        leftLauncherMotor.setPower(leftLaunchPower);
        rightLauncherMotor.setPower(rightLaunchPower);

        // Conveyor Belt

        if (gamepad2.b) {
            beltForwardStatus = true;
            beltReverseStatus = false;
        }

        if (gamepad2.right_stick_button) {
            beltForwardStatus = false;
            beltReverseStatus = true;
        }

        if (gamepad2.a) {
            beltForwardStatus = false;
            beltReverseStatus = false;
        }

        if (beltForwardStatus && !beltReverseStatus) {
            if (beltPower < minimumBeltPower) {
                beltPower = beltPower + beltStopPowerIncrement;
            }

            if (beltPower >= minimumBeltPower) {
                beltPower = beltPower + beltStartPowerIncrement;
                if (beltPower >= maximumForwardBeltPower) {
                    beltPower = beltPower - beltStopPowerIncrement;
                    if (beltPower <= forwardRunningBeltPower) {
                        beltPower = forwardRunningBeltPower;
                    }
                }
            }
        }

        if (!beltForwardStatus && !beltReverseStatus) {
            if (beltPower > minimumBeltPower) {
                beltPower = beltPower - beltStopPowerIncrement;
                if (beltPower <= minimumBeltPower) {
                    beltPower = minimumBeltPower;
                }
            }

            if (beltPower < minimumBeltPower) {
                beltPower = beltPower + beltStopPowerIncrement;
                if (beltPower >= minimumBeltPower) {
                    beltPower = minimumBeltPower;
                }
            }
        }

        if (beltReverseStatus && !beltForwardStatus) {
            if (beltPower > minimumBeltPower) {
                beltPower = beltPower - beltStopPowerIncrement;
            }

            if (beltPower <= minimumBeltPower) {
                beltPower = beltPower - beltStartPowerIncrement;
                if (beltPower <= maximumReverseBeltPower) {
                    beltPower = beltPower + beltStopPowerIncrement;
                    if (beltPower >= reverseRunningBeltPower) {
                        beltPower = reverseRunningBeltPower;
                    }
                }
            }
        }

        beltMotor.setPower(beltPower);

        if (beltPower >= maximumForwardBeltPower) {
            beltPower = maximumForwardBeltPower;
        }

        if (beltPower <= maximumReverseBeltPower) {
            beltPower = maximumReverseBeltPower;
        }

        // Yoga Arm

        if (gamepad1.left_trigger >= 0.2) {
            yogaArm.setPosition(yogaArmDownPosition);
        }

        if (gamepad1.right_trigger >= 0.2) {
            yogaArm.setPosition(yogaArmUpPosition);
        }

        // Gate Arms

        leftStickYVal = gamepad2.left_stick_y; // Also new code! Change values, </> signs, +/- signs accordingly

        leftStickYVal = Range.clip(leftStickYVal, -1, 1); // Prevents left stick from throwing values outside of -1.0 and 1.0

        if (leftStickYVal >= stickInputThreshold) { // Stick is pushed forward and gate will open
            leftServoPosition = leftServoPosition + servoPositionAlterVal; // (IMPORTANT) Depending on way servos are oriented and closed/open positions, addition may need to be changed to subtraction
            rightServoPosition = rightServoPosition - servoPositionAlterVal; // The above comment applies to this as well (+/- switching)
        }

        if (leftStickYVal <= -stickInputThreshold) { // Stick is pulled backward and gate will close
            leftServoPosition = leftServoPosition - servoPositionAlterVal; // (IMPORTANT) If servoPositionAlterVal is being added to leftServoPosition in the 1st if statement, then it will need to be subtracted here (opposites)
            rightServoPosition = rightServoPosition + servoPositionAlterVal; // The above comment applies here as well
        }

        if (leftServoPosition >= leftGateArmClosedPosition) { // Constraints on the servo from going beyond closed/open positions
            leftServoPosition = leftGateArmClosedPosition; // (IMPORTANT) Depending on servo orientation and servo position values, >= signs may need to be changed to <= signs (and the opposite applies as well)
        }

        if (leftServoPosition <= leftGateArmOpenPosition) { // Constraints on the servo from going beyond closed/open positions
            leftServoPosition = leftGateArmOpenPosition; // (IMPORTANT) Depending on servo orientation and servo position values, >= signs may need to be changed to <= signs (and the opposite applies as well)
        }

        if (rightServoPosition <= rightGateArmClosedPosition) { // Constraints on the servo from going beyond closed/open positions
            rightServoPosition = rightGateArmClosedPosition; // (IMPORTANT) Depending on servo orientation and servo position values, >= signs may need to be changed to <= signs (and the opposite applies as well)
        }

        if (rightServoPosition >= rightGateArmOpenPosition) { // Constraints on the servo from going beyond closed/open positions
            rightServoPosition = rightGateArmOpenPosition; // (IMPORTANT) Depending on servo orientation and servo position values, >= signs may need to be changed to <= signs (and the opposite applies as well)
        }

        leftGateArm.setPosition(leftServoPosition);
        rightGateArm.setPosition(rightServoPosition);


        // Telemetry

        telemetry.addData("input", "left stick: " + String.format("%.2f", leftStickDriveVal));
        telemetry.addData("input", "right stick: " + String.format("%.2f", rightStickTurnVal));

        if (leftStickDriveVal < 0) {
            telemetry.addData("output", "left stick squared: " + String.format("%.2f", -leftSquaredDriveVal));
        } else {
            telemetry.addData("output", "left stick squared: " + String.format("%.2f", leftSquaredDriveVal));
        }

        if (rightStickTurnVal < 0) {
            telemetry.addData("output", "right stick squared: " + String.format("%.2f", -rightSquaredTurnVal));
        } else {
            telemetry.addData("output", "right stick squared: " + String.format("%.2f", rightSquaredTurnVal));
        }

        if (!speedSlowEngaged) {
            speedLimit = speedLimiterFast;
        } else {
            speedLimit = speedLimiterSlow;
        }

        telemetry.addData("drive", "left motor pwr: " + String.format("%.2f", leftMotorPower));
        telemetry.addData("drive", "right motor pwr: " + String.format("%.2f", rightMotorPower));
        telemetry.addData("pwr", "limit: " + String.format("%.2f", speedLimit));
        telemetry.addData("launcher", "left pwr: " + String.format("%.3f", leftLaunchPower));
        telemetry.addData("launcher", "right pwr: " + String.format("%.3f", rightLaunchPower));
        telemetry.addData("conveyor belt", "pwr: " + String.format("%.2f", beltPower));
        telemetry.addData("gate", " left position:  " + String.format("%.3f", leftServoPosition));
        telemetry.addData("gate", " right position:  " + String.format("%.3f", rightServoPosition));

    }
}