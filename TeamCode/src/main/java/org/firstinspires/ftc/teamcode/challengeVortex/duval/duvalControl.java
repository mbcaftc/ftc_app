package org.firstinspires.ftc.teamcode.challengeVortex.duval;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.challengeVortex.duval.subClasses.gateControl;

/**
 * Created by johnduval on 11/30/16.
 */
@TeleOp(name = "John Control")
@Disabled

public class duvalControl extends OpMode {
    DcMotor gateArm;
    gateControl myGateControl;

    @Override
    public void init() {
        //myGateControl = new gateControl(hardwareMap.dcMotor.get("gate_arm"));
        gateArm = hardwareMap.dcMotor.get("gate_arm");
        myGateControl = new gateControl();
    }

    @Override
    public void loop() {
        /*//opens the gate
        if (gamepad2.right_stick_y >= 0.2) {
            //gateControl.gateOpen (gateArm);
            gateArm.setPower(0.1);
        }
        //gate motor continues to "open" without this else
        else {
            gateArm.setPower(0);
        }
        //closes the gate
        if (gamepad2.right_stick_y <= -0.2) {
            //gateControl.gateClose(gateArm);
            gateArm.setPower(-0.1);
        }
        //gate motor continues to "close" without this else.
        else {
            gateArm.setPower(0);
        }
        */

        //opens the gate
        if (gamepad2.right_stick_y >= 0.2) {
            myGateControl.gateOpen (gateArm);
        }
        //gate motor continues to "open" without this else
        else {
            myGateControl.gateStop(gateArm);
        }
        //closes the gate
        if (gamepad2.right_stick_y <= -0.2) {
            myGateControl.gateClose (gateArm);
        }
        //gate motor continues to "close" without this else.
        else {
            myGateControl.gateStop(gateArm);
        }
    }
}
