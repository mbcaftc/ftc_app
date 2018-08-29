package org.firstinspires.ftc.teamcode.challengeVelocityVortex201617.duval;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.challengeVelocityVortex201617.duval.subClasses.LauncherControlFull;

/**
 * Created by mbca on 1/21/17.
 */
@Autonomous(name = "Encoders test Calibrate")
@Disabled
public class EncodersTest extends LinearOpMode {

    OpticalDistanceSensor odsSensor;

    DcMotor leftDriveMotor;
    DcMotor rightDriveMotor;

    ColorSensor colorSensor;

//    DcMotor leftLauncherMotor;
//    DcMotor rightLauncherMotor;

    double leftLaunchPower = 0;
    double rightLaunchPower = 0;
    double launchStartPowerIncrement = 0.004;
    double launchStopPowerIncrement = 0.001;
    double maximumLauncherPower = 0.5;
    double minimumLauncherPower = 0;

    //  DcMotor beltMotor;

    double beltPower = 0;
    double beltStartPowerIncrement = 0.05;
    double beltStopPowerIncrement = 0.01;
    double maximumForwardBeltPower = 0.3;
    double minimumBeltPower = 0;
    double maximumReverseBeltPower = -1.0;
    double forwardRunningBeltPower = 0.1;
    double reverseRunningBeltPower = -0.1;

    Servo leftArm;
    Servo rightArm;

    final static double leftArmUpPosition = 0.26;
    final static double rightArmUpPosition = 0.86;
    final static double leftArmDownPosition = 0.94;
    final static double rightArmDownPosition = 0.18;


    boolean beaconRED;
    boolean beaconBLUE;

    final static int launcherTimeA = 1000; //time for belt to start before launcher starts
    final static int launcherTimeB = 7000; //time belt will run to pick up the particles and launch them

    final static int moveTimeA = 800; //time to move from wall into launching position
    final static int moveTimeB = 3585; //from launch point to first white line
    final static int moveTimeBa = 1450; //time from turn onto white line to the beacon for color reading 1
    final static int moveTimeC = 1000; //moves back from wall after ODS sensor tells robot to stop - gets in position to raise beacon arms.
    final static int moveTimeD = 400; //gets close to beacon with the beacon arms raised.
    final static int moveTimeE = 200; //move slowly towards beacon with arms raised.
    final static int moveTimeF = 800; //robot moves back from beacon after having pressed it.

    final static int turnTimeA = 650; //first turn after launching to position towards first white line
    final static int turnTimeB = 685; //turns to go straight on first white line

    final static double beaconDistanceRange = 4.1;

    final static double redArmRaiseThreshold = 3;
    final static double blueArmRaiseTheshold = 3;

    double leftGateArmClosedPositionForLaunch = .57; // These values are only place-holder values
    double leftGateArmClosedPosition = .715;
    double leftGateArmOpenPosition = 0.1; // You will need to experiment with these values to find the perfect servo positions
    double rightGateArmClosedPositionForLaunch = 0.43;
    double rightGateArmClosedPosition = .29;
    double rightGateArmOpenPosition = 0.9;

    Servo leftGateArm;
    Servo rightGateArm;

    int leftPos, rightPos;

    double COUNTS;

    //old
/*    final static double MOVEDISTANCE1 = 6; //wall to launching point
    final static double MOVEDISTANCE2 = 52; //launching point to white line
    final static double MOVEDISTANCE3 = 12; //white line to beacon
*/
    final static double MOVEDISTANCE1 = 24; //wall to launching point
    final static double MOVEDISTANCE2 = 44; //launching point to white line
    final static double MOVEDISTANCE3 = 12; //white line to beacon

    final static double TURNDISTANCE1 = 6.0; //turn from launching position to position to move to white line
    final static double TURNDISTANCE2 = 6.0; //turn from white line to parallel with white line

    int movement;
    //1 == forward
    //2 == backwards
    //3 == turn left
    //4 == turn right

    LauncherControlFull launcherControl;

    @Override

    public void runOpMode() throws InterruptedException {

        launcherControl = new LauncherControlFull(hardwareMap.dcMotor.get("left_launcher"), hardwareMap.dcMotor.get("right_launcher"), hardwareMap.dcMotor.get("conveyor_belt"));

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
        boolean bLedOn = false;

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        odsSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");

        leftDriveMotor = hardwareMap.dcMotor.get("left_drive");
        rightDriveMotor = hardwareMap.dcMotor.get("right_drive");
        rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
//sets motors to use Encoders
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//resets encoders
        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //leftLauncherMotor = hardwareMap.dcMotor.get("left_launcher");
        //  rightLauncherMotor = hardwareMap.dcMotor.get("right_launcher");
        //rightLauncherMotor.setDirection(DcMotor.Direction.REVERSE);

        //beltMotor = hardwareMap.dcMotor.get("conveyor_belt");

        leftArm = hardwareMap.servo.get("left_arm");
        rightArm = hardwareMap.servo.get("right_arm");
        leftArm.setPosition(leftArmDownPosition);
        rightArm.setPosition(rightArmDownPosition);


        leftGateArm = hardwareMap.servo.get("left_gate_arm"); // Don't forget to program left and right _gate_arm into the RC phone as servos
        rightGateArm = hardwareMap.servo.get("right_gate_arm"); // Or else the phone won't be able to communicate with the program (stuff won't work)
        leftGateArm.setPosition(leftGateArmClosedPosition); // Also, while doing this, don't forget to delete the old gate_arm which was the dcMotor registered in the RC phone
        rightGateArm.setPosition(rightGateArmClosedPosition); // Remember that it matters how you plug in the servos to the module; it depends on how you assigned them in the RC phone

        waitForStart();

//runs from wall to launch point
        movement = 1;
        COUNTS = calculateRotations(MOVEDISTANCE1);
        drive(COUNTS, movement);

//        launcherControl.startLauncherBlackBattery();

/*
        while (leftLaunchPower < maximumLauncherPower || rightLaunchPower < maximumLauncherPower) {
            if (leftLaunchPower < maximumLauncherPower) {
                leftLaunchPower = leftLaunchPower + launchStartPowerIncrement;
            }
            if (rightLaunchPower < maximumLauncherPower) {
                rightLaunchPower = rightLaunchPower + launchStartPowerIncrement;
            }
            leftLauncherMotor.setPower(leftLaunchPower);
            rightLauncherMotor.setPower(rightLaunchPower);
        }
*/


        //time for launcher to start before belt starts
   //     sleep(launcherTimeA);

     //   leftGateArm.setPosition(leftGateArmClosedPositionForLaunch); // Also, while doing this, don't forget to delete the old gate_arm which was the dcMotor registered in the RC phone
     //   rightGateArm.setPosition(rightGateArmClosedPositionForLaunch); // Remember that it matters how you plug in the servos to the module; it depends on how you assigned them in the RC phone

       // launcherControl.conveyorBeltStart();

        //beltMotor.setPower(maximumForwardBeltPower);

// time for belt to run
     //   sleep(launcherTimeB);

        //launcherControl.conveyorBeltStop();


//        launcherControl.stopLauncher();
/*
        while (leftLaunchPower > 0 || rightLaunchPower > 0) {
            if (leftLaunchPower > 0) {
                leftLaunchPower = leftLaunchPower - launchStopPowerIncrement;
            }
            if (rightLaunchPower > 0) {
                rightLaunchPower = rightLaunchPower - launchStopPowerIncrement;
            }
            leftLauncherMotor.setPower(leftLaunchPower);
            rightLauncherMotor.setPower(rightLaunchPower);
        }
*/



//closes the arms to make sure doesn't interfere with going to walls.
 //       leftGateArm.setPosition(leftGateArmClosedPosition); // Also, while doing this, don't forget to delete the old gate_arm which was the dcMotor registered in the RC phone
   //     rightGateArm.setPosition(rightGateArmClosedPosition); // Remember that it matters how you plug in the servos to the module; it depends on how you assigned them in the RC phone

//turns from launching position to angle to move to white line
/*        movement = 3;
        COUNTS = calculateRotations(TURNDISTANCE1);
        drive(COUNTS, movement);

        movement = 1;
        COUNTS = calculateRotations(MOVEDISTANCE2);
        drive(COUNTS, movement);

        movement = 3;
        COUNTS = calculateRotations(TURNDISTANCE2);
        drive(COUNTS, movement);

        movement = 1;
        COUNTS = calculateRotations(MOVEDISTANCE3);
        drive(COUNTS, movement); */

        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
    }

    private double calculateRotations(double distance) {
        final int ENCODER_CPR = 1440;
        final int GEAR_RATIO = 2;
        final int WHEEL_DIAMETER = 4;
        final double CIRCUMFRANCE = Math.PI * WHEEL_DIAMETER;
        double ROTATIONS = distance / CIRCUMFRANCE;
        return ENCODER_CPR * ROTATIONS * GEAR_RATIO;
    }

    private void drive (double COUNTS, int move) {
        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveMotor.setTargetPosition((int) COUNTS);
        rightDriveMotor.setTargetPosition((int) COUNTS);

        leftPos = leftDriveMotor.getCurrentPosition();
        rightPos = rightDriveMotor.getCurrentPosition();

        leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //ElapsedTime runTime = new ElapsedTime();
        //runTime.reset();

        //runTime.milliseconds();

        while (leftPos < COUNTS && opModeIsActive()) {

            switch (move) {
                case 1:
                    leftDriveMotor.setPower(0.5);
                    rightDriveMotor.setPower(0.5);
                    leftPos = leftDriveMotor.getCurrentPosition();
                    rightPos = rightDriveMotor.getCurrentPosition();

                    break;
                case 2:
                    leftDriveMotor.setPower(-0.5);
                    rightDriveMotor.setPower(-0.5);
                    leftPos = -leftDriveMotor.getCurrentPosition();
                    rightPos = -rightDriveMotor.getCurrentPosition();
                    break;
                case 3:
                    leftDriveMotor.setPower(-0.5);
                    rightDriveMotor.setPower(0.5);
                    leftPos = -leftDriveMotor.getCurrentPosition();
                    rightPos = rightDriveMotor.getCurrentPosition();
                    break;
                case 4:
                    leftDriveMotor.setPower(0.5);
                    rightDriveMotor.setPower(-0.5);
                    leftPos = leftDriveMotor.getCurrentPosition();
                    rightPos = -rightDriveMotor.getCurrentPosition();
                    break;
            }


//            leftPos = leftDriveMotor.getCurrentPosition();
//            rightPos = rightDriveMotor.getCurrentPosition();

            telemetry.update();

            telemetry.addData("Left Motor Target", String.format("%d", leftDriveMotor.getTargetPosition()));
            telemetry.addData("Left Motor Current", String.format("%d", leftDriveMotor.getCurrentPosition()));
            telemetry.addData("Right Motor Target", String.format("%d", rightDriveMotor.getTargetPosition()));
            telemetry.addData("Right Motor Current", String.format("%d", rightDriveMotor.getCurrentPosition()));
        }

        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);

    }
}


