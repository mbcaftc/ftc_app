package org.firstinspires.ftc.teamcode.challengeVortex.blake;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by blake_shafer on 1/3/17.
 */

@TeleOp(name = "Servo Arms Gate")
@Disabled

public class ServoArmsGate extends OpMode { // DOn't forget that after you perfect all of the values, etc. in this code, the same things need to be changed accordingly in the FullControl program

    Servo leftGateArm;
    Servo rightGateArm;

    double leftGateArmClosedPosition = 1.0; // These values are only place-holder values
    double leftGateArmOpenPosition = 0.5; // You will need to experiment with these values to find the perfect servo positions
    double rightGateArmClosedPosition = 0.5;
    double rightGateArmOpenPosition = 1.0;

    double leftServoPosition;
    double rightServoPosition;

    double servoPositionAlterVal = 0.01; // This is the speed by which the servo will change position as long as the left stick is forward/backward
                                        // Again, this is a place-holder and needs to be experimented with based on how quickly/slowly you want the servo to move
    double stickInputThreshold = 0.2; // Amount of input required on left stick for gate servos to begin opening/closing

    double leftStickYVal;

    @Override
    public void init() {

        leftGateArm = hardwareMap.servo.get("left_gate_arm"); // Don't forget to program left and right _gate_arm into the RC phone as servos
        rightGateArm = hardwareMap.servo.get("right_gate_arm"); // Or else the phone won't be able to communicate with the program (stuff won't work)
        leftGateArm.setPosition(leftGateArmClosedPosition); // Also, while doing this, don't forget to delete the old gate_arm which was the dcMotor registered in the RC phone
        rightGateArm.setPosition(rightGateArmClosedPosition); // Remember that it matters how you plug in the servos to the module; it depends on how you assigned them in the RC phone
    }

    @Override
    public void loop() {

        leftStickYVal = gamepad2.left_stick_y;

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

        telemetry.addData("gate", " left position:  " + String.format("%.3f", leftServoPosition));
        telemetry.addData("gate", " right position:  " + String.format("%.3f", rightServoPosition));
    }
}
