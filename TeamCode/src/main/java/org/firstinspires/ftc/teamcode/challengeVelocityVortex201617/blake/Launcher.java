package org.firstinspires.ftc.teamcode.challengeVelocityVortex201617.blake;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by johnduval on 10/26/16.
 */
@TeleOp(name = "Launcher")
@Disabled


public class Launcher extends OpMode {

    DcMotor leftLauncherMotor;
    DcMotor rightLauncherMotor;

    double leftLaunchPower;
    double rightLaunchPower;
    double launchIncrementStart;
    double launchIncrementStop;
    double manualPowerIncrement;
    double maximumLauncherPower = 1.0;
    double minimumLauncherPower = 0.0;

    boolean launchStatus;
    boolean fullPowerAllow;
    boolean incrementFast;
    boolean incrementSlow;
    boolean toggleFastAllow;
    boolean toggleSlowAllow;


    @Override
    public void init() {

        leftLauncherMotor = hardwareMap.dcMotor.get("left_launcher");
        rightLauncherMotor = hardwareMap.dcMotor.get("right_launcher");
        rightLauncherMotor.setDirection(DcMotor.Direction.REVERSE);

        leftLaunchPower = 0;
        rightLaunchPower = 0;
        launchIncrementStart = 0.002;
        launchIncrementStop = 0.001;
        manualPowerIncrement = 0.025;

        launchStatus = false;
        fullPowerAllow = true;
        incrementFast = false;
        incrementSlow = false;
        toggleFastAllow = false;
        toggleSlowAllow = false;
    }

    @Override
    public void loop() {

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
                leftLaunchPower = leftLaunchPower + launchIncrementStart;
                rightLaunchPower = rightLaunchPower + launchIncrementStart;
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
            leftLaunchPower = leftLaunchPower - launchIncrementStop;
            rightLaunchPower = rightLaunchPower - launchIncrementStop;
            if (leftLaunchPower <= minimumLauncherPower) {
                leftLaunchPower = minimumLauncherPower;
            }

            if (rightLaunchPower <= minimumLauncherPower) {
                rightLaunchPower = minimumLauncherPower;
            }
        }

        leftLauncherMotor.setPower(leftLaunchPower);
        rightLauncherMotor.setPower(rightLaunchPower);

        // Telemetry

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("launcher", "left pwr: " + String.format("%.3f", leftLaunchPower));
        telemetry.addData("launcher", "right pwr: " + String.format("%.3f", rightLaunchPower));
    }
}
