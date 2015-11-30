package com.qualcomm.ftcrobotcontroller.opmodes;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.HiTechnicNxtCompassSensor;
import com.qualcomm.hardware.HiTechnicNxtUltrasonicSensor;
import com.qualcomm.hardware.ModernRoboticsUsbLegacyModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.LegacyModule;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * An op mode that uses the geomagnetic and accelerometer values to calculate device
 * orientation and return those values in telemetry.
 * It makes use of getRotationMatrix() and getOrientation(), but does not use
 * remapCoordinateSystem() which one might want.
 * see: http://developer.android.com/reference/android/hardware/SensorManager.html#remapCoordinateSystem(float[], int, int, float[])
 */
public class CompCalibrate extends OpMode{
    private HiTechnicNxtCompassSensor com;
    /*
    * Constructor
    */
    public CompCalibrate() {

    }

    /*
    * Code to run when the op mode is first enabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
    */
    @Override
    public void init() {
        com = (HiTechnicNxtCompassSensor)hardwareMap.compassSensor.get("this");
        com.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
    }

    /*
* Code to run when the op mode is first enabled goes here
* @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
*/
    @Override
    public void start() {
    }

    /*
    * This method will be called repeatedly in a loop
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */

    @Override
    public void loop() {

        telemetry.addData("val1", com.calibrationFailed());
    }

    /*
    * Code to run when the op mode is first disabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
    */
    @Override
    public void stop() {
        com.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
    }
}