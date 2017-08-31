package org.firstinspires.ftc.teamcode.challengeVortex.blake;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by blake_shafer on 9/26/16.
 */

@TeleOp(name = "Arm")
@Disabled
public class Arm extends OpMode {

    DcMotor leftArm;

    @Override
    public void init() {

        leftArm = hardwareMap.dcMotor.get("left_arm");
    }

    @Override
    public void loop() {

        double leftStickArmVal = -gamepad1.right_stick_y;

        if (leftStickArmVal > 0.2) { // if left stick input value is greater than 0.2
            leftStickArmVal = 0.2; // bring value back to 0.2
            leftArm.setPower(leftStickArmVal);
        }

        if (leftStickArmVal < -0.2) { // if left stick input value is less than -0.2
            leftStickArmVal = -0.2; // bring value back to -0.2
            leftArm.setPower(leftStickArmVal);
        }

        // Telemetry

        telemetry.addData("arm", "arm:  " + String.format("%.2f", leftStickArmVal));
    }
}
