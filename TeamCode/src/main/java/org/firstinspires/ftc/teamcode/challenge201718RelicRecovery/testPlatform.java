package org.firstinspires.ftc.teamcode.challenge201718RelicRecovery;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.challenge201718RelicRecovery.subClasses.colorSensorArm;
import org.firstinspires.ftc.teamcode.challenge201718RelicRecovery.subClasses.mechDriveAuto;
import org.firstinspires.ftc.teamcode.challenge201718RelicRecovery.subClasses.revColorDistanceSensor;

import java.util.Locale;

/**
 * Created by student on 12/5/17.
 */
@Autonomous(name = "Test - Platform + Sensor + gyro", group = "TESTING")
@Disabled
public class testPlatform extends LinearOpMode {
    mechDriveAuto myMechDrive;
    int movement = 1;
    colorSensorArm myColorSensorArm;
    revColorDistanceSensor myRevColorDistanceSensor;

    boolean distanceSensorInRange;

    /* Declare OpMode members. */

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    //float heading;


    @Override
    public void runOpMode() throws InterruptedException {
        myMechDrive = new mechDriveAuto(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myRevColorDistanceSensor =  new revColorDistanceSensor(hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance"));
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
// and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //gravity = imu.getGravity();

        waitForStart();

        while (opModeIsActive()) {

            /*
            if (myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH) >= 4) {
                distanceSensorInRange = true;
            } else if (myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH) < 4) {
                distanceSensorInRange = true;
            } else {
                distanceSensorInRange = false;
            }*/
            distanceSensorInRange = false;

            switch (movement) {
                case 0: // check everything
                    telemetry.addData("CASE: ", movement);
                    telemetry.addData("HEADING: ", angles.firstAngle);
                    telemetry.update();
                    myMechDrive.encoderDrive(21, 6, 0.5);
                    sleep(250);
                    movement ++;
                    break;
                case 1:// rotate right on platform
                    telemetry.addData("CASE: ", movement);
                    telemetry.update();
                    myMechDrive.encoderDrive(21, 6, 0.5);
                    sleep(250);
                    movement++;
                    break;
                case 2: //go off platform
                    telemetry.addData("CASE: ", movement);
                    telemetry.update();
                    myMechDrive.encoderDrivePlatform(21.25,.8);
                    sleep(250);
                    movement++;
                    break;
                case 3: // back up against platform for distance
                    if (myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH) <= 4) {
                        distanceSensorInRange = true;
                        telemetry.addData("came off ramp ", "in range");
                    }
                    telemetry.addData("Distance (INCHES)",
                            String.format(Locale.US, "%.02f", myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH)));
                    telemetry.update();
                    sleep(500);
                    while (myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH) > 4 || !distanceSensorInRange) {
                        telemetry.addData("GO BACK", "");
                        telemetry.addData("Distance (INCHES)",
                                String.format(Locale.US, "%.02f", myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH)));
                        telemetry.update();
                        sleep(250);
                        myMechDrive.encoderDrive(1,2,.6);
                        if(myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH) <= 4) {
                            distanceSensorInRange = true;
                        }
                        sleep(250);
                        telemetry.addData("WENT BACK", "");
                        telemetry.addData("Distance (INCHES)",
                                String.format(Locale.US, "%.02f", myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH)));
                        telemetry.update();
                    }
                    sleep(250);
                    movement++;
                    break;
                case 4: //center robot using Gyro
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    telemetry.addData("CASE gyro: ", movement);
                    telemetry.addData("MOVING","");
                    telemetry.addData("Gyro Heading: ", angles.firstAngle);
                    telemetry.update();
                    sleep(1000);
                    if (angles.firstAngle >= -89) {  //robot did NOT rotate enough coming off platform
                        while (angles.firstAngle >= -89) {
                            myMechDrive.powerDrive(6, .16);
                            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        }
                    }
                    else if (angles.firstAngle <= -91) {    //robot rotated TOO MUCH coming off platform
                        while (angles.firstAngle <= -91) {
                            myMechDrive.powerDrive(5,.16);
                            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        }
                    }
                    myMechDrive.stopMotors();
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("DONE MOVING","");
                    telemetry.addData("Gyro Heading: ", angles.firstAngle);
                    telemetry.update();
                    sleep(1000);
                    /*
                    idea was to reset heading to 0.  eh. didn't work as planned.
                    telemetry.addData("RESETING Gyro","");
                    telemetry.update();
                    imu.initialize(parameters);
                    sleep(1000);
                    heading = -angles.firstAngle;
                    sleep(1000);
                    telemetry.addData("Gyro Reset before running forward","");
                    telemetry.addData("Gyro Heading: ", heading);
                    telemetry.update();
                    sleep(1000);
                    */
                    movement++;
                    break;
                case 5: // run to center of cryptobox
                    telemetry.addData("CASE: ", movement);
                    myMechDrive.encoderDrive(13.5,1,.6);
                    sleep(250);
                    movement ++;
                    break;
                case 6: //rotate right to orient with CryptoBox
                    telemetry.addData("CASE: ", movement);
                    telemetry.update();
                    myMechDrive.encoderDriveMat(21.5, 6, 0.6);
                    sleep(200);
                    telemetry.addData("HEADING: ", angles.firstAngle);
                    sleep(1000);
                    movement++;
                    break;
                case 7: //center robot with gyro
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    telemetry.addData("CASE gyro: ", movement);
                    telemetry.addData("MOVING","");
                    telemetry.addData("Gyro Heading: ", angles.firstAngle);
                    telemetry.update();
                    sleep(1000);
                    if (angles.firstAngle >= -179 && angles.firstAngle < 0) {           //robot did NOT rotate enough coming off platform
                        while (angles.firstAngle >= 179 && angles.firstAngle < 0) {     // && since goes -180 --> + 180
                            myMechDrive.powerDrive(6, .16);
                            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        }
                    }
                    else if (angles.firstAngle <= 179 && angles.firstAngle > 0) {       //robot rotated TOO MUCH coming off platform
                        while (angles.firstAngle <= 179 && angles.firstAngle > 0) {     // && sinnce goes -180 --> +180
                            myMechDrive.powerDrive(5,.16);
                            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        }
                    }
                    myMechDrive.stopMotors();
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("DONE MOVING","");
                    telemetry.addData("Gyro Heading: ", angles.firstAngle);
                    telemetry.update();
                    sleep(10000);
                    movement++;
                    break;
                case 8:
                    requestOpModeStop();
            }
        }
    }
}
