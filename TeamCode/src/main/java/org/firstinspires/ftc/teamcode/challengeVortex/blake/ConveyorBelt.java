package org.firstinspires.ftc.teamcode.challengeVortex.blake;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by blake_shafer on 10/25/16.
 */

@TeleOp(name = "Conveyor Belt")

public class ConveyorBelt extends OpMode {

    DcMotor beltMotor;

    double beltPower = 0;
    double beltStartPowerIncrement = 0.05;
    double beltStopPowerIncrement = 0.01;
    double maximumBeltPower = 1.0;
    double minimumBeltPower = 0;
    double maximumReverseBeltPower = -1.0;
    double forwardRunningBeltPower = 0.1;
    double reverseRunningBeltPower = -0.1;

    boolean beltForwardStatus; // true = belt forward, false = belt off
    boolean beltReverseStatus; // true = belt reverse

    @Override
    public void init() {

        beltMotor = hardwareMap.dcMotor.get("conveyor_belt");

        beltForwardStatus = false;
        beltReverseStatus = false;
    }

    @Override
    public void loop() {

        if (gamepad2.b) {
            beltForwardStatus = true;
            beltReverseStatus = false;
        }

        if (gamepad2.b && gamepad2.a) {
            beltForwardStatus = false;
            beltReverseStatus = false;
        }

        if (gamepad2.a) {
            beltForwardStatus = false;
            beltReverseStatus = true;
        }

        if (beltForwardStatus && !beltReverseStatus) {
            if (beltPower < minimumBeltPower) {
                beltPower = beltPower + beltStopPowerIncrement;
            }

            if (beltPower >= minimumBeltPower) {
                beltPower = beltPower + beltStartPowerIncrement;
                if (beltPower >= maximumBeltPower) {
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

        // Telemetry

        telemetry.addData("conveyor belt", "pwr: " + String.format("%.2f", beltPower));
    }
}
