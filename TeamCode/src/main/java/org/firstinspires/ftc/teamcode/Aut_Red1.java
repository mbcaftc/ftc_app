package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by johnduval on 10/5/17.
 */

@Autonomous (name = "Red - 1", group = "RED")

public class Aut_Red1 extends LinearOpMode {

    int movement = 0; //switch variable to determine movementt
    int redThreshold = 5;
    int blueThreshold = 5;

    ColorSensor colorSensor;
    Servo colorSensorArm;
    double upPosition = 0.325;
    double downPosition = 1.0;

    boolean armState; // Up = false, down = true

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double speedLimiterFactor = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {

        colorSensorArm = hardwareMap.servo.get("color_sensor_arm");
        colorSensorArm.setPosition(upPosition); // Set to up position

        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        rearLeftMotor = hardwareMap.dcMotor.get("rear_left_motor");
        rearRightMotor = hardwareMap.dcMotor.get("rear_right_motor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        waitForStart();

        while (opModeIsActive()) {
            //

            //STARTING GYRO AT 0 degrees
            switch (movement) {
                case 0:
                    //SET GLYPH ARMS TO CLOSE POSITION
                    colorSensorArm.setPosition(downPosition);
                    movement ++;
                    break;
                case 1: //detecting jewel and knocking off & centering
                    wait(1000); //wait to be sure color sensor is working
                    if (colorSensor.red() > redThreshold) {
                        //ROTATE 25 CLOCKWISE
                        wait(100);
                        colorSensorArm.setPosition(upPosition); //SET COLOR SENSOR ARM TO UP POSITION
                        //"RESET" WITH ROTATE 25 COUNTERCLOCKWISE
                    }
                    else if (colorSensor.blue() > blueThreshold) {
                        //ROTATE 25 COUNTERCLOCKWISE
                        wait(100);
                        colorSensorArm.setPosition(upPosition); //SET COLOR SENSOR ARM TO UP POSITION
                        //"RESET" WITH ROTATE 25 CLOCKWISE
                    }
                    //OR IS IT BETTER NOT TO REVERSE THE TURNS AND JUST CENTER BACK TO 0 DEGREES?
                    wait(100);
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
                    movement ++;
                    break;
                case 5:
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    rearLeftMotor.setPower(0);
                    rearRightMotor.setPower(0);
                    requestOpModeStop();
                    break;
            }
            telemetry.addData("Servo", "Position: " + String.format("%.3f", colorSensorArm.getPosition()));
            telemetry.addData("BLUE: ", colorSensor.blue());
            telemetry.addData("RED: ", colorSensor.red());
            telemetry.update();
        }
    }
}
