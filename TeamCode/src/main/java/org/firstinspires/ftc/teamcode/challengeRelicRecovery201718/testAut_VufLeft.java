package org.firstinspires.ftc.teamcode.challengeRelicRecovery201718;

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
import org.firstinspires.ftc.teamcode.challengeRelicRecovery201718.subClasses.boardArm;
import org.firstinspires.ftc.teamcode.challengeRelicRecovery201718.subClasses.colorSensorArm;
import org.firstinspires.ftc.teamcode.challengeRelicRecovery201718.subClasses.glyphArms;
import org.firstinspires.ftc.teamcode.challengeRelicRecovery201718.subClasses.glyphLift;
import org.firstinspires.ftc.teamcode.challengeRelicRecovery201718.subClasses.mechDriveAuto;
import org.firstinspires.ftc.teamcode.challengeRelicRecovery201718.subClasses.revColorDistanceSensor;

/**
 * Created by student on 12/6/17.
 */
@Disabled
@Autonomous (name = "Vuf Left", group = "TESTING")
public class testAut_VufLeft extends LinearOpMode {
    int movement = 0; //switch variable to determine movement

    colorSensorArm myColorSensorArm;
    mechDriveAuto myMechDrive;
    glyphArms myGlyphArms;
    glyphLift myGlyphLift;
    boardArm myBoardArm;
    revColorDistanceSensor myRevColorDistanceSensor;

    boolean distanceSensorInRange;



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

        //we know nothing will be in range to start.  Distance sensor returns "NaN" if objects too far away.
        distanceSensorInRange = false;
        myGlyphLift = new glyphLift(hardwareMap.dcMotor.get("glyph_lift"));
        myColorSensorArm = new colorSensorArm(hardwareMap.servo.get("color_sensor_arm"),hardwareMap.colorSensor.get("sensor_color"), hardwareMap.servo.get("color_sensor_arm_rotate"));
        myMechDrive = new mechDriveAuto(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myGlyphArms = new glyphArms(hardwareMap.servo.get("top_left_glyph_arm"), hardwareMap.servo.get("bottom_left_glyph_arm"), hardwareMap.servo.get("top_left_glyph_arm"), hardwareMap.servo.get("bottom_right_glyph_arm"));
        myBoardArm = new boardArm(hardwareMap.dcMotor.get("board_arm"));
        myRevColorDistanceSensor =  new revColorDistanceSensor(hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance"));


        myColorSensorArm.colorSensorArmUpSlow();
        myColorSensorArm.colorRotateResting();
        //myGlyphArms.openRaisedGlyphArms(); //ensures robot is wihin 18" by 18" parameters

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
            myGlyphArms.closeGlyphArms();
            sleep(2000);
            myMechDrive.vuforiaLeft(myGlyphArms);
            sleep(1000);
            requestOpModeStop();
        }
    }
}
