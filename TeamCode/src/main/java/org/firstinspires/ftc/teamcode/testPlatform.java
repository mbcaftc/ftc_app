package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subClasses.colorSensorArm;
import org.firstinspires.ftc.teamcode.subClasses.mechDriveAuto;
import org.firstinspires.ftc.teamcode.subClasses.revColorDistanceSensor;

import java.util.Locale;

/**
 * Created by student on 12/5/17.
 */
@Autonomous(name = "Test - Platform + Sensor", group = "TESTING")
@Disabled
public class testPlatform extends LinearOpMode {
    mechDriveAuto myMechDrive;
    int movement = 1;
    colorSensorArm myColorSensorArm;
    revColorDistanceSensor myRevColorDistanceSensor;

    boolean distanceSensorInRange;

    @Override
    public void runOpMode() throws InterruptedException {
        myMechDrive = new mechDriveAuto(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myRevColorDistanceSensor =  new revColorDistanceSensor(hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance"));
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
                    myMechDrive.encoderDrivePlatform(22,.8);


                    sleep(200);


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
                    /*
                    if (myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH) > 4) {
                        while (myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH) > 4) {
                            myMechDrive.encoderDrivePlatformDistanceSensor(2, 0.4);
                        }
                    } else if (myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH) < 4) {
                        while (myRevColorDistanceSensor.revDistanceSensor.getDistance(DistanceUnit.INCH) < 4) {
                            myMechDrive.encoderDrivePlatformDistanceSensor(1, 0.4);
                        }
                    } else {
                            sleep(1);
                        } */
                    sleep(250);
                    movement++;
                    break;
                case 3: // run to center of cryptobox
                    telemetry.addData("CASE: ", movement);
                    myMechDrive.encoderDrive(12,1,.6);
                    sleep(250);
                    movement ++;
                    break;
                case 4:
                    requestOpModeStop();
            }
        }
    }
}
