package org.firstinspires.ftc.teamcode.challenge201718RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.challenge201718RelicRecovery.subClasses.colorSensorArm;

/**
 * Created by student on 12/7/17.
 */
@Autonomous(name = "test color sensor")
//@Disabled
public class autColorSensor extends LinearOpMode {
    colorSensorArm myColorSensorArm;
    @Override
    public void runOpMode() throws InterruptedException {
        myColorSensorArm = new colorSensorArm(hardwareMap.servo.get("color_sensor_arm"),hardwareMap.colorSensor.get("rev_color_sensor_arm"), hardwareMap.servo.get("color_sensor_arm_rotate"));
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("BLUE Raw: ", myColorSensorArm.colorSensor.blue());
            telemetry.addData("RED Raw: ", myColorSensorArm.colorSensor.red());
            int blue = myColorSensorArm.colorSensor.blue() * 255;
            int red = myColorSensorArm.colorSensor.red() * 255;
            telemetry.addData("BLUE 255: ", blue);
            telemetry.addData("RED 255: ", red);
            telemetry.update();
            //sleep(1000);
        }
    }
}
