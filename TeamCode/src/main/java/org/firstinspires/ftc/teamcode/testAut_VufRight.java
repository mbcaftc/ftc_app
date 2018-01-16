package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.subClasses.boardArm;
import org.firstinspires.ftc.teamcode.subClasses.colorSensorArm;
import org.firstinspires.ftc.teamcode.subClasses.glyphArms;
import org.firstinspires.ftc.teamcode.subClasses.glyphLift;
import org.firstinspires.ftc.teamcode.subClasses.mechDriveAuto;
import org.firstinspires.ftc.teamcode.subClasses.revColorDistanceSensor;

/**
 * Created by student on 12/6/17.
 */
@Disabled
@Autonomous (name = "Vuf RIGHT", group = "TESTING")
public class testAut_VufRight extends LinearOpMode {
    int movement = 0; //switch variable to determine movement

    mechDriveAuto myMechDrive;
    glyphArms myGlyphArms;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        //we know nothing will be in range to start.  Distance sensor returns "NaN" if objects too far away.
        myMechDrive = new mechDriveAuto(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myGlyphArms = new glyphArms(hardwareMap.servo.get("left_glyph_arm"), hardwareMap.servo.get("right_glyph_arm"));
        //myGlyphArms.openRaisedGlyphArms(); //ensures robot is wihin 18" by 18" parameters
        waitForStart();


        while (opModeIsActive()) {
            myGlyphArms.closeGlyphArms();
            sleep(2000);
            myMechDrive.vuforiaRight(myGlyphArms);
            sleep(1000);
            requestOpModeStop();
        }
    }
}
