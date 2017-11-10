package org.firstinspires.ftc.teamcode.subClasses;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by johnduval on 10/9/17.
 */

public class glyphArms {
    //keep to reference in Notebook - was value to get arm open position outside frame.
    //with single bracket - open past frame
    //single bracket closed
    //double leftClosePosition = 0.8;
    //with single bracket - parallel open
    //double leftOpenPosition = 0.28;
    //with single bracket - open past frame
    //double leftOpenPosition = 0.43;
    // single bracket closed
    //double rightClosePosition = 0.34;
    //keep to reference in Notebook - was value to get arm open position outside frame.
    //with single bracket - parallel open
    //double rightOpenPosition = 0.86;
    //with single bracket - open past frame
    //double rightOpenPosition = 0.705;

    //with channel
    double leftClosePosition =.83;
    double leftOpenPosition = .36;

    double rightClosePosition = .32;
    double rightOpenPosition = .78;

    double leftSlightlyOpenPosition = .74;
    double rightSlightlyOpenPosition = .4;

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

    public void slightlyOpenGlyphArms () {
        leftGlyphArm.setPosition(leftSlightlyOpenPosition);
        rightGlyphArm.setPosition(rightSlightlyOpenPosition);
    }
}
