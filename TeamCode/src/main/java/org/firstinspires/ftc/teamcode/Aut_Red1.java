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
    int redThreshold = 4;
    int blueThreshold = 4;

    ColorSensor colorSensor;
    Servo colorSensorArm;
    double upPosition = 0.325;
    double downPositionPause1 = 0.75;
    double downPositionPause2 = 0.86;
    double downPositionFinal = 0.90;
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

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            //STARTING GYRO AT 0 degrees

            switch (movement) {
                case 0: //SET GLYPH ARMS TO CLOSE POSITION
                    //SET GLYPH ARMS TO CLOSE POSITION
                    //LOWER COLOR SENSOR ARM
                    colorSensorArm.setPosition(downPositionPause1);
                    sleep(200);
                    colorSensorArm.setPosition(downPositionPause2);
                    sleep(200);
                    colorSensorArm.setPosition(downPositionFinal);
                    movement ++;
                    break;
                case 1: //detecting jewel and knocking off & centering
                    sleep (1000); //wait to be sure color sensor is working
                    telemetry.addData("Servo", "Position: " + String.format("%.3f", colorSensorArm.getPosition()));
                    telemetry.addData("BLUE: ", colorSensor.blue());
                    telemetry.addData("RED: ", colorSensor.red());
                    telemetry.update();
                    if (colorSensor.red() > redThreshold) {
                        //ROTATE 25 CLOCKWISE
                        sleep(2000);
                        encoderDrive(3, 3, 1);
                        sleep(500);
                        colorSensorArm.setPosition(upPosition); //SET COLOR SENSOR ARM TO UP POSITION
                        //"RESET" WITH ROTATE 25 COUNTERCLOCKWISE
                        sleep(200);
                        encoderDrive(3, 4, 1);
                    }
                    else if (colorSensor.blue() > blueThreshold) {
                        sleep(2000);
                        //ROTATE 25 COUNTERCLOCKWISE
                        encoderDrive(3, 4, 1);
                        sleep(500);
                        colorSensorArm.setPosition(upPosition); //SET COLOR SENSOR ARM TO UP POSITION
                        //"RESET" WITH ROTATE 25 CLOCKWISE
                        sleep(200);
                        encoderDrive(3, 3, 1);
                    }
                    else { //in case color sensor doesn't detect any color thresholds
                        colorSensorArm.setPosition(upPosition);
                    }
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
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    rearLeftMotor.setPower(0);
                    rearRightMotor.setPower(0);
                    bLedOn = false;
                    colorSensor.enableLed(bLedOn);
                    requestOpModeStop();
                    break;
            }
            telemetry.addData("Servo", "Position: " + String.format("%.3f", colorSensorArm.getPosition()));
            telemetry.update();
        }


    }

    private void encoderDrive (int distance, int direction, double power) {

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        final int ENCODER_CPR = 1120;
        final int GEAR_RATIO = 1;
        final int WHEEL_DIAMETER = 4;
        final double CIRCUMFRANCE = Math.PI * WHEEL_DIAMETER;
        double ROTATIONS = distance / CIRCUMFRANCE;
        double counts =  ENCODER_CPR * ROTATIONS * GEAR_RATIO;

        double powerReductionFactor = 0.77;

        if (direction == 1) { // robot will move forward
            frontLeftMotor.setTargetPosition((int) counts);
            frontRightMotor.setTargetPosition((int) counts);
            rearLeftMotor.setTargetPosition((int) counts);
            rearRightMotor.setTargetPosition((int) counts);
        }

        else if (direction == 2) { // robot will move backward
            frontLeftMotor.setTargetPosition((int) -counts);
            frontRightMotor.setTargetPosition((int) -counts);
            rearLeftMotor.setTargetPosition((int) -counts);
            rearRightMotor.setTargetPosition((int) -counts);
        }

        else if (direction == 3) { // robot will strafe left
            frontLeftMotor.setTargetPosition((int) -counts);
            frontRightMotor.setTargetPosition((int) counts);
            rearLeftMotor.setTargetPosition((int) counts);
            rearRightMotor.setTargetPosition((int) -counts);
        }

        else if (direction == 4) { // robot will strafe right
            frontLeftMotor.setTargetPosition((int) counts);
            frontRightMotor.setTargetPosition((int) -counts);
            rearLeftMotor.setTargetPosition((int) -counts);
            rearRightMotor.setTargetPosition((int) counts);
        }

        else if (direction == 5) { // robot will rotate left
            frontLeftMotor.setTargetPosition((int) -counts);
            frontRightMotor.setTargetPosition((int) counts);
            rearLeftMotor.setTargetPosition((int) -counts);
            rearRightMotor.setTargetPosition((int) counts);
        }

        else if (direction == 6) { // robot will rotate right
            frontLeftMotor.setTargetPosition((int) counts);
            frontRightMotor.setTargetPosition((int) -counts);
            rearLeftMotor.setTargetPosition((int) counts);
            rearRightMotor.setTargetPosition((int) -counts);
        }

        frontLeftMotor.setPower(power * powerReductionFactor);
        frontRightMotor.setPower(power * powerReductionFactor);
        rearLeftMotor.setPower(power * powerReductionFactor);
        rearRightMotor.setPower(power * powerReductionFactor);
    }

}
