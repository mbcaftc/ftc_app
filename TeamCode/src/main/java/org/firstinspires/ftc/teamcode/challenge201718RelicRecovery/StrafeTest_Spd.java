package org.firstinspires.ftc.teamcode.challenge201718RelicRecovery;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by johnduval on 11/21/17.
 */
@TeleOp(name = "STRAFE POWER", group = "TESTING")
@Disabled
public class StrafeTest_Spd extends OpMode {
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearLeftMotor;
    DcMotor rearRightMotor;


    double SPD1 = 0.8;
    double SPD2 = 1.0;


    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        rearLeftMotor = hardwareMap.dcMotor.get("rear_left_motor");
        rearRightMotor = hardwareMap.dcMotor.get("rear_right_motor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //int maxSpeed = 1920;
        //frontLeftMotor.setMaxSpeed(); //put in encoder EncoderTicksPerSecond
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            frontLeftMotor.setPower(SPD2);
            frontRightMotor.setPower(SPD1);
            rearLeftMotor.setPower(SPD1);
            rearRightMotor.setPower(SPD2);
        } else if (gamepad1.dpad_down) {
            frontLeftMotor.setPower(-SPD2);
            frontRightMotor.setPower(-SPD1);
            rearLeftMotor.setPower(-SPD1);
            rearRightMotor.setPower(-SPD2);

        } else if (gamepad1.dpad_right) {
            frontRightMotor.setPower(SPD1);
            rearRightMotor.setPower(-SPD2);

            frontLeftMotor.setPower(-SPD2);

            rearLeftMotor.setPower(SPD1);


        } else if (gamepad1.dpad_left) {
            frontLeftMotor.setPower(SPD2);
            frontRightMotor.setPower(-SPD1);
            rearLeftMotor.setPower(-SPD1);
            rearRightMotor.setPower(SPD2);
        } else if (gamepad1.left_bumper) {
            frontLeftMotor.setPower(SPD2);
            frontRightMotor.setPower(SPD1);
            rearLeftMotor.setPower(SPD1);
            rearRightMotor.setPower(SPD2);
        } else {
            frontLeftMotor.setPower(0.0);
            frontRightMotor.setPower(0.0);
            rearLeftMotor.setPower(0.0);
            rearRightMotor.setPower(0.0);
        }
    }


    private void setMode(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        rearLeftMotor.setMode(mode);
        rearRightMotor.setMode(mode);
    }

}
