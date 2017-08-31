//Started 2/2/2017 to use with the ODS & COLOR SENSOR

package org.firstinspires.ftc.teamcode.challengeVortex.duval;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.challengeVortex.duval.subClasses.LauncherControlFull;

/**
 * Created by mbca on 2/2/17.
 */
@Autonomous(name = "Blue Wall + Launch + Beacons - Black Battery")

public class AutonomousBlueWallLaunchBeaconsBlackBattery extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    OpticalDistanceSensor odsSensor;
    ColorSensor colorSensor;
    GyroSensor gyroSensor;
    ModernRoboticsI2cGyro mrGyro;

    int zAccumulated;
    int heading;
    int xVal, yVal, zVal;

    TouchSensor touchSensor;

    DcMotor leftDriveMotor;
    DcMotor rightDriveMotor;

    final static int launcherTimeA = 900; //time for belt to start before launcher starts
    final static int launcherTimeB = 2000; //time belt will run to pick up the particles and launch them

    boolean beaconRED;
    boolean beaconBLUE;
    final static double blueArmRaiseThreshold = 4;

    double whiteLineReflectanceVal = 0.5;

    double leftGateArmClosedPositionForLaunch = .57; // These values are only place-holder values
    double leftGateArmClosedPosition = .715;
    double leftGateArmOpenPosition = 0.1; // You will need to experiment with these values to find the perfect servo positions
    double rightGateArmClosedPositionForLaunch = 0.43;
    double rightGateArmClosedPosition = .29;
    double rightGateArmOpenPosition = 0.9;

    Servo leftGateArm;
    Servo rightGateArm;

    Servo yogaArm;

    double yogaArmDownPosition = 0.825;
    double yogaArmUpPosition = 0.215;

    int leftPos, rightPos;

    double COUNTS;

    final static double MOVEDISTANCE1 = 6.4; //wall to launching point
    final static double MOVEDISTANCE2 = 25; //launching point to just before white line
    final static double MOVEDISTANCE3 = 0.32; //white line to press beacon
    final static double MOVEDISTANCE4 = 0.25; // goes back to read color
    final static double MOVEDISTANCE5 = 4; //goes forward to press beacon button
    final static double MOVEDISTANCE6 = 4; //reverses after pressing beacon

    final static double TURNDISTANCE1 = 2.85; //turn from launching position to position to move to white line
    final static double TURNDISTANCE2 = 6; //turn from white line to parallel with white line
    final static double TURNDISTANCE3 = 12; //turn from 1st white line to 2nd white line

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
        float hsvValues[] = {0F, 0F, 0F};

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

        gyroSensor = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) gyroSensor;

        leftDriveMotor = hardwareMap.dcMotor.get("left_drive");
        rightDriveMotor = hardwareMap.dcMotor.get("right_drive");
        rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
//sets motors to use Encoders
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//resets encoders
        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftGateArm = hardwareMap.servo.get("left_gate_arm"); // Don't forget to program left and right _gate_arm into the RC phone as servos
        rightGateArm = hardwareMap.servo.get("right_gate_arm"); // Or else the phone won't be able to communicate with the program (stuff won't work)
        leftGateArm.setPosition(leftGateArmClosedPosition); // Also, while doing this, don't forget to delete the old gate_arm which was the dcMotor registered in the RC phone
        rightGateArm.setPosition(rightGateArmClosedPosition); // Remember that it matters how you plug in the servos to the module; it depends on how you assigned them in the RC phone

        yogaArm = hardwareMap.servo.get("yoga_arm");
        yogaArm.setPosition(yogaArmDownPosition);

        mrGyro.calibrate();  //turns on blue light on Gyro to calibrate.  To set current position to 0
        while (mrGyro.isCalibrating()) {

        }

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            zAccumulated = -mrGyro.getIntegratedZValue();
            //telemetry.update();
            //telemetry.addData("accumulatedGyroVal", String.format("%03d", zAccumulated));

            //runs from wall to launch point
            movement = 1;
            COUNTS = calculateRotations(MOVEDISTANCE1);
            drive(COUNTS, movement);
            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);
            //launch the particles
            launcher();
            //turn 1 LEFT from launching to be able to go to white line

            /*movement = 3;
            COUNTS = calculateRotations(TURNDISTANCE1);
            drive(COUNTS, movement);*/

            do {
                zAccumulated = -mrGyro.getIntegratedZValue();
                //telemetry.update();
                //telemetry.addData("accumulatedGyroVal", String.format("%03d", zAccumulated));
                heading = mrGyro.getHeading();

                xVal = mrGyro.rawX() / 128;
                yVal = mrGyro.rawY() / 128;
                zVal = mrGyro.rawZ() / 128;

                leftDriveMotor.setPower(0.16);
                rightDriveMotor.setPower(-0.16);
            } while (zAccumulated < 55 && opModeIsActive());

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            //ramp up speed to first
            movement = 1;
            COUNTS = calculateRotations(MOVEDISTANCE2);
            drive(COUNTS, movement);
            // move close to white line

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);
            //switch to do while loop about 12 inches from white line for deceleration purposes

            sleep(100);

            do {
                zAccumulated = -mrGyro.getIntegratedZValue();
                //telemetry.update();
                //telemetry.addData("accumulatedGyroVal", String.format("%03d", zAccumulated));
                heading = mrGyro.getHeading();

                xVal = mrGyro.rawX() / 128;
                yVal = mrGyro.rawY() / 128;
                zVal = mrGyro.rawZ() / 128;

                leftDriveMotor.setPower(-0.16);
                rightDriveMotor.setPower(0.16);
            } while (zAccumulated > 5 && opModeIsActive());

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            do {
                gyroDrive(0, 0.2);
                zAccumulated = -mrGyro.getIntegratedZValue();
                //telemetry.update();
                //telemetry.addData("accumulatedGyroVal", String.format("%03d", zAccumulated));
            } while (odsSensor.getRawLightDetected() < whiteLineReflectanceVal && opModeIsActive());

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            movement = 2;
            COUNTS = calculateRotations(MOVEDISTANCE3);
            drive(COUNTS, movement);

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            do {
                zAccumulated = -mrGyro.getIntegratedZValue();
                //telemetry.update();
                //telemetry.addData("accumulatedGyroVal", String.format("%03d", zAccumulated));
                heading = mrGyro.getHeading();

                xVal = mrGyro.rawX() / 128;
                yVal = mrGyro.rawY() / 128;
                zVal = mrGyro.rawZ() / 128;

                leftDriveMotor.setPower(0.16);
                rightDriveMotor.setPower(-0.16);
            } while (zAccumulated < 82 && opModeIsActive());

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            // turn parallel with white line

            //switch to do while loop halfway down white line for deceleration purposes

            leftDriveMotor.setPower(0.2);
            rightDriveMotor.setPower(0.2);

            sleep(1450);

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            // move to press beacon

            movement = 2;
            COUNTS = calculateRotations(MOVEDISTANCE4);
            drive(COUNTS, movement);
            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            // move away from beacon to read color of beacon

            sleep(5000);

            if (colorSensor.blue() > blueArmRaiseThreshold) {
                beaconBLUE = true;
            } else {
                beaconBLUE = false;
            }

            if (beaconBLUE) {
                leftDriveMotor.setPower(0);
                rightDriveMotor.setPower(0);
            } else {
                do {
                    leftDriveMotor.setPower(0.2);
                    rightDriveMotor.setPower(0.2);

                    sleep(450);

                    leftDriveMotor.setPower(0);
                    rightDriveMotor.setPower(0);

                    movement = 2;
                    COUNTS = calculateRotations(MOVEDISTANCE4);
                    drive(COUNTS, movement);
                    leftDriveMotor.setPower(0);
                    rightDriveMotor.setPower(0);

                    if (colorSensor.blue() > blueArmRaiseThreshold) {
                        beaconBLUE = true;
                    } else {
                        beaconBLUE = false;
                    }

                    sleep(200);
                } while (!beaconBLUE && opModeIsActive());
            }

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            leftDriveMotor.setPower(-0.5);
            rightDriveMotor.setPower(-0.5);

            sleep(400);

            leftDriveMotor.setPower(0);
            rightDriveMotor.setPower(0);

            requestOpModeStop();
        }
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
                    leftDriveMotor.setPower(0.2);
                    rightDriveMotor.setPower(0.2);
                    leftPos = leftDriveMotor.getCurrentPosition();
                    rightPos = rightDriveMotor.getCurrentPosition();
                    break;
                case 2:
                    leftDriveMotor.setPower(-0.2);
                    rightDriveMotor.setPower(-0.2);
                    leftPos = -leftDriveMotor.getCurrentPosition();
                    rightPos = -rightDriveMotor.getCurrentPosition();
                    break;
                case 3:
                    leftDriveMotor.setPower(-0.2);
                    rightDriveMotor.setPower(0.2);
                    leftPos = -leftDriveMotor.getCurrentPosition();
                    rightPos = rightDriveMotor.getCurrentPosition();
                    break;
                case 4:
                    leftDriveMotor.setPower(0.2);
                    rightDriveMotor.setPower(-0.2);
                    leftPos = leftDriveMotor.getCurrentPosition();
                    rightPos = -rightDriveMotor.getCurrentPosition();
                    break;

            }


//            leftPos = leftDriveMotor.getCurrentPosition();
//            rightPos = rightDriveMotor.getCurrentPosition();

            telemetry.update();

            telemetry.addData("RED Beacon Value", beaconRED);
            telemetry.addData("BLUE Beacon Value", beaconBLUE);

            telemetry.addData("Left Motor Target", String.format("%d", leftDriveMotor.getTargetPosition()));
            telemetry.addData("Left Motor Current", String.format("%d", leftDriveMotor.getCurrentPosition()));
            telemetry.addData("Right Motor Target", String.format("%d", rightDriveMotor.getTargetPosition()));
            telemetry.addData("Right Motor Current", String.format("%d", rightDriveMotor.getCurrentPosition()));

            telemetry.addData("accumulatedGyroVal", String.format("%03d", zAccumulated));
        }

        //leftDriveMotor.setPower(0);
        //rightDriveMotor.setPower(0); commented out to decide whether we want to set power to 0 at end

    }

    private void launcher () {
        launcherControl.startLauncherWhiteBattery();
        sleep(launcherTimeA);

        leftGateArm.setPosition(leftGateArmClosedPositionForLaunch); // Also, while doing this, don't forget to delete the old gate_arm which was the dcMotor registered in the RC phone
        rightGateArm.setPosition(rightGateArmClosedPositionForLaunch); // Remember that it matters how you plug in the servos to the module; it depends on how you assigned them in the RC phone

        launcherControl.conveyorBeltStart();
        sleep(launcherTimeB);

        launcherControl.conveyorBeltReverse();
        sleep(500);

        launcherControl.conveyorBeltStart();
        sleep(1200);

        launcherControl.conveyorBeltStop();

        launcherControl.stopLauncher();
//closes the arms to make sure doesn't interfere with going to walls.
        leftGateArm.setPosition(leftGateArmClosedPosition); // Also, while doing this, don't forget to delete the old gate_arm which was the dcMotor registered in the RC phone
        rightGateArm.setPosition(rightGateArmClosedPosition); // Remember that it matters how you plug in the servos to the module; it depends on how you assigned them in the RC phone

    }

    private void gyroDrive(int targetPosition, double straightSpeed) {

        double leftPower;
        double rightPower;
        int absolutePosition = zAccumulated = -mrGyro.getIntegratedZValue();
        int targetDeviation = (absolutePosition - targetPosition) / 100;

        leftPower = straightSpeed - (targetDeviation);
        rightPower = straightSpeed + (targetDeviation);

        if (leftPower > 1.0) {
            leftPower = 1.0;
        }

        if (leftPower < -1.0) {
            leftPower = -1.0;
        }

        if (rightPower > 1.0) {
            rightPower = 1.0;
        }

        if (rightPower < -1.0) {
            rightPower = -1.0;
        }

        leftDriveMotor.setPower(leftPower);
        rightDriveMotor.setPower(rightPower);

        //^^^^^SUPER^DANK^METHOD^^^^^
    }
}
