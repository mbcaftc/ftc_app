package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subClasses.mechDriveAuto;
import org.firstinspires.ftc.teamcode.subClasses.colorSensorArmAuto;


/**
 * Created by johnduval on 10/7/17.
 */

@Autonomous (name = "Blue - 1", group = "BLUE")

public class Aut_Blue1 extends LinearOpMode {
    int movement = 0; //switch variable to determine movementt

    ColorSensor colorSensor;
    Servo colorSensorArm;

    colorSensorArmAuto myColorSensorArm;
    mechDriveAuto myMechDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        myColorSensorArm = new colorSensorArmAuto(hardwareMap.servo.get("color_sensor_arm"),hardwareMap.colorSensor.get("sensor_color"));
        //myColorSensorArm.colorSensorSetup();

        myMechDrive = new mechDriveAuto(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));

        waitForStart();
        while (opModeIsActive()) {

        }
    }
}
