package org.firstinspires.ftc.teamcode.challengeRelicRecovery201718.subClasses;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

/**
 * Created by Zealy on 12/4/2017.
 */

public class revColorDistanceSensor {
    public ColorSensor revColorSensor;
    public DistanceSensor revDistanceSensor;
    public revColorDistanceSensor (ColorSensor rCS, DistanceSensor rDS) { // rev Color Sensor & rev Distance Sensor
        revColorSensor = rCS;
        revDistanceSensor = rDS;
    }
}
