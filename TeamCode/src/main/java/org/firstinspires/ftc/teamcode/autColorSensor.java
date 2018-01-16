package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subClasses.colorSensorArm;

/**
 * Created by student on 12/7/17.
 */
@Autonomous(name = "test color sensor")
@Disabled
public class autColorSensor extends LinearOpMode {
    colorSensorArm myColorSensorArm;
    @Override
    public void runOpMode() throws InterruptedException {
        myColorSensorArm = new colorSensorArm(hardwareMap.servo.get("color_sensor_arm"),hardwareMap.colorSensor.get("sensor_color"), hardwareMap.servo.get("color_sensor_arm_rotate"));
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("BLUE: ", myColorSensorArm.colorSensor.blue());
            telemetry.addData("RED: ", myColorSensorArm.colorSensor.red());
            telemetry.update();
            //sleep(1000);
        }
    }
}
