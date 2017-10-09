package org.firstinspires.ftc.teamcode.subClasses;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by johnduval on 10/9/17.
 */

public class glyphArms {
    double leftClosePosition;
    double rightClosePosition;
    double leftOpenPosition;
    double rightOpenPosition;
    public Servo leftGlyphArm, rightGlyphArm;
    public glyphArms(Servo lGA, Servo rGA) {
        leftGlyphArm = lGA;
        rightGlyphArm = rGA;
    }

    public void closeGlyphArms () {
        leftGlyphArm.setPosition(leftClosePosition);
        rightGlyphArm.setPosition(rightClosePosition);
    }

    public void openGlyphArms () {
        leftGlyphArm.setPosition(leftOpenPosition);
        rightGlyphArm.setPosition(rightOpenPosition);
    }
}
