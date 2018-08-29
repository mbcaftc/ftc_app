package org.firstinspires.ftc.teamcode.challenge201617VelocityVortex.duval;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by mbca on 2/8/17.
 */

public class MRI_Gyro_Video extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        int zAccumulated;
        int heading;
        int xVal, yVal, zVal;

        GyroSensor gyroSensor;
        ModernRoboticsI2cGyro mrGyro;

        gyroSensor = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) gyroSensor;

        mrGyro.calibrate();  //turns on blue light on Gyro to calibrate.  To set current position to 0

        waitForStart();

        while (mrGyro.isCalibrating()) {

        }

        while (opModeIsActive()) {

            zAccumulated = mrGyro.getIntegratedZValue();
            heading = mrGyro.getHeading();

            xVal = mrGyro.rawX() / 128;
            yVal = mrGyro.rawY() / 128;
            zVal = mrGyro.rawZ() / 128;

            telemetry.addData("1. heading", String.format("%03d", heading));
            telemetry.addData("2. accumulated", String.format("%03d", zAccumulated));
            telemetry.addData("3. X", String.format("%03d", xVal));
            telemetry.addData("4. Y", String.format("%03d", yVal));
            telemetry.addData("5. Z", String.format("%03d", zVal));

            waitOneFullHardwareCycle();
        }

    }
}
