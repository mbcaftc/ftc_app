package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subClasses.mechDriveAuto;
import org.firstinspires.ftc.teamcode.subClasses.colorSensorArmAuto;
import org.firstinspires.ftc.teamcode.subClasses.glyphArms;


/**
 * Created by johnduval on 10/7/17.
 */

@Autonomous (name = "Blue - 1", group = "BLUE")
//@Disabled

public class Aut_Blue1 extends LinearOpMode {
    int movement = 0; //switch variable to determine movementt

    colorSensorArmAuto myColorSensorArm;
    mechDriveAuto myMechDrive;
    glyphArms myGlyphArms;

    @Override
    public void runOpMode() throws InterruptedException {
        myColorSensorArm = new colorSensorArmAuto(hardwareMap.servo.get("color_sensor_arm"),hardwareMap.colorSensor.get("sensor_color"));
        myMechDrive = new mechDriveAuto(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myGlyphArms = new glyphArms(hardwareMap.servo.get("left_glyph_arm"), hardwareMap.servo.get("right_glyph_arm"));
        myColorSensorArm.colorSensorArmUp();
        myGlyphArms.closeGlyphArms();
        waitForStart();
        while (opModeIsActive()) {
            //STARTING GYRO AT 0 degrees
            switch (movement) {
                case 0:
                    //LOWER COLOR SENSOR ARM
                    myColorSensorArm.colorSensorArmDown();
                    movement ++; //move on to next movement
                    break;
                case 1: //detecting jewel and knocking off & centering
                    sleep (1000); //wait to be sure color sensor is working
                    telemetry.addData("Servo", "Position: " + String.format("%.3f", myColorSensorArm.colorSensorArm.getPosition()));
                    telemetry.addData("BLUE: ", myColorSensorArm.colorSensor.blue());
                    telemetry.addData("RED: ", myColorSensorArm.colorSensor.red());
                    telemetry.update();
                    sleep(1000);
                    myColorSensorArm.colorSensorREDalliance();
                    //OR IS IT BETTER NOT TO REVERSE THE TURNS AND JUST CENTER BACK TO 0 DEGREES?
                    sleep(1000);
                    movement ++;
                    break;
                case 2: //STRAFE LEFT
                    //STRAFE LEFT X AMOUNT/
                    //USE GYRO TO CENTER?
                    //USE ODS SENSOR ON BOTTOM OF ROBOT TO DETERMINE WHEN TO STOP INSTEAD OF ENCODERS?
                    movement ++;
                    break;
                case 3: //GO FORWARD
                    //GO FORWARD Y AMOUNT
                    //USE GYRO TO CENTER?
                    //USE ODS SENSOR TO KNOW WHEN TO STOP GOING FORWARD?
                    movement ++;
                    break;
                case 4: //RELEASE BLOCK WITH SERVOS
                    //left servo OPEN
                    //right servo OPEN
                    movement ++;
                    break;
                case 5:
                    myMechDrive.mechDriveSTOP();
                    //bLedOn = false;
                    //colorSensor.enableLed(bLedOn);
                    requestOpModeStop();
                    break;
            }
            telemetry.addData("Servo", "Position: " + String.format("%.3f", myColorSensorArm.colorSensorArm.getPosition()));
            telemetry.update();

        }
    }
}
