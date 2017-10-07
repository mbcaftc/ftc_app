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
    int redThreshold = 4;
    int blueThreshold = 4;

    ColorSensor colorSensor;
    Servo colorSensorArm;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensorArm = hardwareMap.servo.get("color_sensor_arm");

    }
}
