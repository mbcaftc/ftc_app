package org.firstinspires.ftc.teamcode.challenge201718RelicRecovery.extras;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.challenge201718RelicRecovery.subClasses.mechDriveAuto;
import org.firstinspires.ftc.teamcode.challenge201718RelicRecovery.subClasses.colorSensorArm;
import org.firstinspires.ftc.teamcode.challenge201718RelicRecovery.subClasses.glyphArms;


/**
 * Created by johnduval on 10/7/17.
 */

@Autonomous (name = "Red - 1 - VuForia", group = "RED")
@Disabled

public class Aut_Red_1_VUFORIA extends LinearOpMode {

    int movement = 0; //switch variable to determine movementt

    colorSensorArm myColorSensorArm;
    mechDriveAuto myMechDrive;
    glyphArms myGlyphArms;
    // 1 == LEFT
    // 2 == CENTER & DEFAULT
    // 3 == RIGHT
    int cryptoboxColumn; // determinds column for robot to aim for based on the vuforia key.

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        myColorSensorArm = new colorSensorArm(hardwareMap.servo.get("color_sensor_arm"),hardwareMap.colorSensor.get("sensor_color"), hardwareMap.servo.get("color_sensor_arm_rotate"));
        myMechDrive = new mechDriveAuto(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myGlyphArms = new glyphArms(hardwareMap.servo.get("top_left_glyph_arm"), hardwareMap.servo.get("bottom_left_glyph_arm"), hardwareMap.servo.get("top_left_glyph_arm"), hardwareMap.servo.get("bottom_right_glyph_arm"));

        myColorSensorArm.colorSensorArmUp();
        myGlyphArms.openLoweredGlyphArms(); //ensures robot is wihin 18" by 18" parameters

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ASmjss3/////AAAAGQGMjs1d6UMZvrjQPX7J14B0s7kN+rWOyxwitoTy9i0qV7D+YGPfPeeoe/RgJjgMLabjIyRXYmDFLlJYTJvG9ez4GQSI4L8BgkCZkUWpAguRsP8Ah/i6dXIz/vVR/VZxVTR5ItyCovcRY+SPz3CP1tNag253qwl7E900NaEfFh6v/DalkEDppFevUDOB/WuLZmHu53M+xx7E3x35VW86glGKnxDLzcd9wS1wK5QhfbPOExe97azxVOVER8zNNF7LP7B+Qeticfs3O9pGXzI8lj3zClut/7aDVwZ10IPVk4oma6CO8FM5UtNLSb3sicoKV5QGiNmxbbOlnPxz9qD38UAHshq2/y0ZjI/a8oT+doCr";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            switch (movement) {
                case 0:
                    myGlyphArms.closeGlyphArms();
                    myColorSensorArm.colorSensorArmDown();
                    movement ++; //move on to next movement
                    break;
                case 1:
                    sleep(1000);
                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                        if (pose != null) {
                            VectorF trans = pose.getTranslation();
                            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                            double tX = trans.get(0);
                            double tY = trans.get(1);
                            double tZ = trans.get(2);

                            double rX = rot.firstAngle;
                            double rY = rot.secondAngle;
                            double rZ = rot.thirdAngle;
                        }
                    }
                    else {
                        telemetry.addData("VuMark", "not visible");
                    }
                    sleep(1000);
                    switch (vuMark) {
                        case LEFT:
                            cryptoboxColumn = 1;
                            telemetry.addData("LEFT SWITCH - Column: ", cryptoboxColumn);
                            break;
                        case CENTER:
                            cryptoboxColumn = 2;
                            telemetry.addData("CENTER SWITCH - Column: ", cryptoboxColumn);
                            break;
                        case RIGHT:
                            cryptoboxColumn = 3;
                            telemetry.addData("RIGHT SWITCH - Column: ", cryptoboxColumn);
                            break;
                        default:
                            cryptoboxColumn = 2;
                            telemetry.addData("DEFAULT SWITCH - Column: ", cryptoboxColumn);
                            break;
                    }
                    sleep(500);
                    movement++;
                    break;
                case 2: //detecting jewel and knocking off & centering
                    sleep (500); //wait to be sure color sensor is working
                    telemetry.addData("Servo", "Position: " + String.format("%.3f", myColorSensorArm.colorSensorArm.getPosition()));
                    telemetry.addData("BLUE: ", myColorSensorArm.colorSensor.blue());
                    telemetry.addData("RED: ", myColorSensorArm.colorSensor.red());
                    telemetry.update();
                    sleep(500);
                    //robot will move dependeing on the color sensed in myColorArm.colorJewel()
                    //colorJewel passes an int to redAllianceJewel so knows which direction to move
                    //1 = red jewel on left and strafe right
                    //2 = blue jewel on leeft and strafe left
                    //3 = no color detected and do no strafe at all
                    myMechDrive.redJewel(myColorSensorArm, myColorSensorArm.colorJewel());
                    sleep(500);
                    movement ++;
                    break;
                case 3: //STRAFE LEFT TO CRYPTOBOX COLUMN
                    //STRAFE LEFT X AMOUNT
                    myMechDrive.encoderDrive(41, 4, 1);
                    sleep(500);
                    movement ++;
                    break;
                case 4://ROTATE ROBOT WITH CORRECT ORIENTATION FOR GLYPH
                    myMechDrive.encoderDrive(41, 5, 0.6);
                    sleep(500);
                    movement++;
                    break;
                case 5: //GO FORWARD TO CRYPTO BOX
                    //from case 1 where we get the vuforia code
                    // 1 == LEFT
                    // 2 == CENTER & DEFAULT
                    // 3 == RIGHT
                    switch (cryptoboxColumn) {
                        case 1:
                            myMechDrive.vuforiaLeft(myGlyphArms);
                            sleep(500);
                            break;
                        case 2:
                            myMechDrive.vuforiaCenter(myGlyphArms);
                            sleep(500);
                            break;
                        case 3:
                            myMechDrive.vuforiaRight(myGlyphArms);
                            sleep(500);
                    }
                    movement++;
                    break;
                case 6:
                    requestOpModeStop();
                    break;
            }
            telemetry.addData("Servo", "Position: " + String.format("%.3f", myColorSensorArm.colorSensorArm.getPosition()));
            telemetry.update();

        }
    }
}
