package org.firstinspires.ftc.teamcode.challengeVelocityVortex201617.blake;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by mbca on 1/17/17.
 */

@TeleOp(name = "FullControl TESTING")
@Disabled

public class FullControlImportClasses extends OpMode {

    DcMotor leftDriveMotor;
    DcMotor rightDriveMotor;

    double rightStickDriveValue;
    double leftStickDriveValue;
    double rightSquaredDriveValue;
    double leftSquaredDriveValue;

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
    double maximumReverseBeltPower = -1.0;
    double forwardRunningBeltPower = 0.1;
    double reverseRunningBeltPower = -0.1;

    boolean beltForwardStatus;
    boolean beltReverseStatus;

    Servo leftBeaconArm;
    Servo rightBeaconArm;

    final static double leftBeaconArmDownPosition = 0.94; // REMEMBER: These values need to be recalibrated
    final static double leftBeaconArmUpPosition = 0.26;
    final static double rightBeaconArmDownPosition = 0.18;
    final static double rightBeaconArmUpPosition = 0.86;

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

        leftDriveMotor = hardwareMap.dcMotor.get("left_drive");
        rightDriveMotor = hardwareMap.dcMotor.get("right_drive");
        rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);

        leftLauncherMotor = hardwareMap.dcMotor.get("left_launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right_launcher");
        rightLauncherMotor.setDirection(DcMotor.Direction.REVERSE);

        beltMotor = hardwareMap.dcMotor.get("conveyor_belt");

        leftBeaconArm = hardwareMap.servo.get("left_arm");
        rightBeaconArm = hardwareMap.servo.get("right_arm");
        leftBeaconArm.setPosition(leftBeaconArmDownPosition);
        rightBeaconArm.setPosition(rightBeaconArmDownPosition);

        leftGateArm = hardwareMap.servo.get("left_gate_arm"); // Don't forget to program left and right _gate_arm into the RC phone as servos
        rightGateArm = hardwareMap.servo.get("right_gate_arm"); // Or else the phone won't be able to communicate with the program (stuff won't work)
        leftGateArm.setPosition(leftGateArmClosedPosition); // Also, while doing this, don't forget to delete the old gate_arm which was the dcMotor registered in the RC phone
        rightGateArm.setPosition(rightGateArmClosedPosition); // Remember that it matters how you plug in the servos to the module; it depends on how you assigned them in the RC phone
    }

    @Override
    public void loop() {

        // Drive Tank Squared

        rightStickDriveValue = -gamepad1.right_stick_y;
        leftStickDriveValue = -gamepad1.left_stick_y;
        rightSquaredDriveValue = rightStickDriveValue * rightStickDriveValue;
        leftSquaredDriveValue = leftStickDriveValue * leftStickDriveValue;

        leftSquaredDriveValue = Range.clip(leftSquaredDriveValue, -1, 1);
        rightSquaredDriveValue = Range.clip(rightSquaredDriveValue, -1, 1);

        if (rightStickDriveValue < 0) {
            rightDriveMotor.setPower(-rightSquaredDriveValue);
        } else {
            rightDriveMotor.setPower(rightSquaredDriveValue);
        }

        if (leftStickDriveValue < 0) {
            leftDriveMotor.setPower(-leftSquaredDriveValue);
        } else {
            leftDriveMotor.setPower(leftSquaredDriveValue);
        }

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

        // Beacon Arms

        if (gamepad1.left_bumper) {
            leftBeaconArm.setPosition(leftBeaconArmUpPosition);
        }

        if (gamepad1.left_trigger > .5) {
            leftBeaconArm.setPosition(leftBeaconArmDownPosition);

        }

        if (gamepad1.right_bumper) {
            rightBeaconArm.setPosition(rightBeaconArmUpPosition);
        }

        if (gamepad1.right_trigger > .5) {
            rightBeaconArm.setPosition(rightBeaconArmDownPosition);
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

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("launcher", "left pwr: " + String.format("%.3f", leftLaunchPower));
        telemetry.addData("launcher", "right pwr: " + String.format("%.3f", rightLaunchPower));
        telemetry.addData("conveyor belt", "pwr: " + String.format("%.2f", beltPower));
        telemetry.addData("gate", " left position:  " + String.format("%.3f", leftServoPosition));
        telemetry.addData("gate", " right position:  " + String.format("%.3f", rightServoPosition));
    }
}
