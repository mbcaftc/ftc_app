package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subClasses.mechDriveAuto;
import org.firstinspires.ftc.teamcode.subClasses.colorSensorArm;
import org.firstinspires.ftc.teamcode.subClasses.glyphArms;


/**
 * Created by johnduval on 10/7/17.
 */

@Autonomous (name = "Red - 1", group = "RED")

public class Aut_Red_1 extends LinearOpMode {
    int movement = 0; //switch variable to determine movementt

    colorSensorArm myColorSensorArm;
    mechDriveAuto myMechDrive;
    glyphArms myGlyphArms;

    @Override
    public void runOpMode() throws InterruptedException {

        myColorSensorArm = new colorSensorArm(hardwareMap.servo.get("color_sensor_arm"),hardwareMap.colorSensor.get("sensor_color"));
        myMechDrive = new mechDriveAuto(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myGlyphArms = new glyphArms(hardwareMap.servo.get("left_glyph_arm"), hardwareMap.servo.get("right_glyph_arm"));

        myColorSensorArm.colorSensorArmUp();
        myGlyphArms.openGlyphArms(); //ensures robot is wihin 18" by 18" parameters

        waitForStart();

        while (opModeIsActive()) {

            switch (movement) {
                case 0:
                    myGlyphArms.closeGlyphArms();
                    myColorSensorArm.colorSensorArmDown();
                    movement ++; //move on to next movement
                    break;
                case 1: //detecting jewel and knocking off & centering
                    sleep (500); //wait to be sure color sensor is working
                    telemetry.addData("Servo", "Position: " + String.format("%.3f", myColorSensorArm.colorSensorArm.getPosition()));
                    telemetry.addData("BLUE: ", myColorSensorArm.colorSensor.blue());
                    telemetry.addData("RED: ", myColorSensorArm.colorSensor.red());
                    telemetry.update();
                    sleep(500);
                    //robot will move dependeing on the color sensed in myColorArm.colorJewel()
                    //colorJewel passes an int to redAllianceJewel so knows which direction to move
                    //1 = red jewel on left and strafe right
                    //2 = blue jewel on leeft and strafe left
                    //3 = no color detected and do no strafe at all
                    myMechDrive.redAllianceJewel(myColorSensorArm, myColorSensorArm.colorJewel());
                    //OR IS IT BETTER NOT TO REVERSE THE TURNS AND JUST CENTER BACK TO 0 DEGREES?
                    sleep(500);
                    movement ++;
                    break;
                case 2: //STRAFE LEFT TO CRYPTOBOX COLUMN
                    //STRAFE LEFT X AMOUNT
                    //USE GYRO TO CENTER?
                    //USE ODS SENSOR ON BOTTOM OF ROBOT TO DETERMINE WHEN TO STOP INSTEAD OF ENCODERS?
                    myMechDrive.encoderDrive(39,4,1);
                    sleep(500);
                    movement ++;
                    break;
                case 3://ROTATE ROBOT WITH CORRECT ORIENTATION FOR GLYPH
                    myMechDrive.encoderDrive(42,5,0.6);
                    sleep(500);
                    movement++;
                    break;
                case 4: //GO FORWARD TO CRYPTO BOX
                    myMechDrive.encoderDrive(5.75,1,0.7);
                    sleep(500);
                    //GO FORWARD Y AMOUNT
                    //USE GYRO TO CENTER?
                    //USE ODS SENSOR TO KNOW WHEN TO STOP GOING FORWARD?
                    movement ++;
                    break;
                case 5: //RELEASE BLOCK WITH SERVOS
                    myGlyphArms.openGlyphArms();
                    movement ++;
                    break;
                case 6:
                    myMechDrive.mechDriveSTOP();
                    requestOpModeStop();
                    break;
            }
            telemetry.addData("Servo", "Position: " + String.format("%.3f", myColorSensorArm.colorSensorArm.getPosition()));
            telemetry.update();

        }
    }
}
