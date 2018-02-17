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
    double topLeftClosePosition =.42; //closed to grab glyphs
    double topLeftSlightlyOpenPosition = .37;  //sightly open for exiting CryptoBox and picking up Glyphs
    double topLeftOpenLoweredPosition = .32; //fully open when glyph lift is lowered - to the frame
    double topLeftOpenPosition = .08; //fully open when raised

    double bottomLeftClosePosition =.41;
    double bottomLeftSlightlyOpenPosition = .47;
    double bottomLeftOpenLoweredPosition = .52;
    double bottomLeftOpenPosition = .80;

    double topRightClosePosition = .48;
    double topRightSlightlyOpenPosition = .56;
    double topRightOpenLoweredPosition = .65;
    double topRightOpenPosition = .88;

    double bottomRightClosePosition = .62;
    double bottomRightSlightlyOpenPosition = .54;
    double bottomRightOpenLoweredPosition = .44;
    double bottomRightOpenPosition = .22;

    double leftSlightlyOpenPosition = .56;
    double rightSlightlyOpenPosition = .25;

    public Servo topLeftGlyphArm, bottomLeftGlyphArm, topRightGlyphArm, bottomRightGlyphArm;
    public glyphArms(Servo tLGA, Servo bLGA, Servo tRGA, Servo bRGA) {
        topLeftGlyphArm = tLGA;
        bottomLeftGlyphArm = bLGA;
        topRightGlyphArm = tRGA;
        bottomRightGlyphArm = bRGA;
    }

    public void closeGlyphArms () {
        topLeftGlyphArm.setPosition(topLeftClosePosition);
        bottomLeftGlyphArm.setPosition(bottomLeftClosePosition);
        topRightGlyphArm.setPosition(topRightClosePosition);
        bottomRightGlyphArm.setPosition(bottomRightClosePosition);
    }

    public void slightlyOpenGlyphArms () {
        topLeftGlyphArm.setPosition(topLeftSlightlyOpenPosition);
        bottomLeftGlyphArm.setPosition(bottomLeftSlightlyOpenPosition);
        topRightGlyphArm.setPosition(topRightSlightlyOpenPosition);
        bottomRightGlyphArm.setPosition(bottomRightSlightlyOpenPosition);
    }
    public void openLoweredGlyphArms () {
        topLeftGlyphArm.setPosition(topLeftOpenLoweredPosition);
        bottomLeftGlyphArm.setPosition(bottomLeftOpenLoweredPosition);
        topRightGlyphArm.setPosition(topRightOpenLoweredPosition);
        bottomRightGlyphArm.setPosition(bottomRightOpenLoweredPosition);
    }

    public void openRaisedGlyphArms () {
        topLeftGlyphArm.setPosition(topLeftOpenPosition);
        bottomLeftGlyphArm.setPosition(bottomLeftOpenPosition);
        topRightGlyphArm.setPosition(topRightOpenPosition);
        bottomRightGlyphArm.setPosition(bottomRightOpenPosition);
    }
}
