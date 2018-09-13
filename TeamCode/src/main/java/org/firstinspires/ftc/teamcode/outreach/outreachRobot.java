package org.firstinspires.ftc.teamcode.outreach;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;


@TeleOp(name = "Demo Robot")
public class outreachRobot extends OpMode {

    outreachMotors myOutreachMotors;
    outreachCatapultMotorsRunToPosition myOutreachCatapultMotorsRunToPosition;
    outreachCatapultMotorsRunUsingEncoders myOutreachCatapultMotorsRunUsingEncoders;
    outreachCatapultMotorsRunWithoutEncoders myOutreachCatapultMotorsRunWithoutEncoders;
    outreachTouchSensorCatapult myOutreachTouchSensorCatapult;
    //value for left joystick
    double leftY;
    //value for right joystick
    double rightY;

    double triggerLeft;
    double triggerRight;
    DcMotor catapult;



    @Override
    public void init() {
        myOutreachMotors = new outreachMotors(hardwareMap.dcMotor.get("left_drive_motor"), hardwareMap.dcMotor.get("right_drive_motor"));
        myOutreachCatapultMotorsRunToPosition = new outreachCatapultMotorsRunToPosition(hardwareMap.dcMotor.get("catapult_motor"));
        myOutreachCatapultMotorsRunUsingEncoders = new outreachCatapultMotorsRunUsingEncoders(hardwareMap.dcMotor.get("catapult_motor"));
        myOutreachCatapultMotorsRunWithoutEncoders = new outreachCatapultMotorsRunWithoutEncoders(hardwareMap.dcMotor.get("catapult_motor"));
        myOutreachTouchSensorCatapult = new outreachTouchSensorCatapult (hardwareMap.get(DigitalChannel.class, "catapult_touch_sensor"));
        //catapult = hardwareMap.dcMotor.get("catapult_motor");
        //catapult.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        leftY = gamepad1.left_stick_y;
        rightY = gamepad1.right_stick_y;
        myOutreachMotors.drive(leftY, rightY);
        //trigger left lowers arm
        //trigger right raises arm
        triggerLeft = gamepad1.left_trigger;
        triggerRight = gamepad1.right_trigger;
        myOutreachCatapultMotorsRunToPosition.catapultMotorManualOperation (triggerLeft, triggerRight, myOutreachTouchSensorCatapult);


        if (gamepad1.left_bumper) {
            myOutreachCatapultMotorsRunToPosition.catapultReset(myOutreachTouchSensorCatapult);
        }

        if (gamepad1.right_bumper) {
            myOutreachCatapultMotorsRunToPosition.catapultLaunch(myOutreachTouchSensorCatapult);
        }

        if (gamepad1.y) {
            myOutreachCatapultMotorsRunWithoutEncoders.catapultLaunch(myOutreachTouchSensorCatapult);
        }

        if (gamepad1.a) {
            myOutreachCatapultMotorsRunUsingEncoders.catapultLaunch(myOutreachTouchSensorCatapult);
        }
        //catapult.setPower(-triggerLeft);
        //catapult.setPower(triggerRight);
        telemetry.addData("Touch Sensor: ", myOutreachTouchSensorCatapult.checkCatapultTouchSensor());
        telemetry.addData("Left Y: ", leftY);
        telemetry.addData("Right Y: ", rightY);
        telemetry.addData("Left trigger: ", triggerLeft);
        telemetry.addData("Right Trigger: ", triggerRight);
        telemetry.update();
    }
}
