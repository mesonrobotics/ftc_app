package com.qualcomm.ftcrobotcontroller.opmodes;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.hardware.HiTechnicNxtCompassSensor;
import com.qualcomm.hardware.HiTechnicNxtUltrasonicSensor;
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
    public class GyroTest extends OpMode implements SensorEventListener {
        private String startDate;
        private SensorManager mSensorManager;
        private Sensor accelerometer;
        private Sensor magnetometer;
        private HiTechnicNxtCompassSensor com;
    private final int COMPASS_PORT = 0;
    private HiTechnicNxtUltrasonicSensor ultra;
    private final int ULTRA_PORT = 5;



    int sample = 30;
    private float[] orient = new float[sample];
    // orientation values
    private float azimuth = 0.0f;      // value in radians
    private float pitch = 0.0f;        // value in radians
    private float roll = 0.0f;         // value in radians
    int readRun = 0;

    float m_Norm_Gravity;
    float m_Norm_MagField;
    private float[] mGravity;       // latest sensor values
    private float[] mGeomagnetic;   // latest sensor values

    float[] m_NormEastVector;       // normalised cross product of raw gravity vector with magnetic field values, points east
    float[] m_NormNorthVector;      // Normalised vector pointing to magnetic north
    boolean m_OrientationOK;        // set true if m_azimuth_radians and m_pitch_radians have successfully been calculated following a call to onSensorChanged(...)
    float m_azimuth_radians;        // angle of the device from magnetic north
    float m_pitch_radians;          // tilt angle of the device from the horizontal.  m_pitch_radians = 0 if the device if flat, m_pitch_radians = Math.PI/2 means the device is upright.
    float m_pitch_axis_radians;

    /*
    * Constructor
    */
    public GyroTest() {

    }

    /*
    * Code to run when the op mode is first enabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
    */
    @Override
    public void init() {
        mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        azimuth = 0.0f;      // value in radians
        pitch = 0.0f;        // value in radians
        roll = 0.0f;
        com = (HiTechnicNxtCompassSensor)hardwareMap.compassSensor.get("this");
        com.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);

        ultra = (HiTechnicNxtUltrasonicSensor)hardwareMap.ultrasonicSensor.get("ultra");

    }

    /*
* Code to run when the op mode is first enabled goes here
* @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
*/
    @Override
    public void start() {
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());

        // delay value is SENSOR_DELAY_UI which is ok for telemetry, maybe not for actual robot use
        mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_UI);
    }

    /*
    * This method will be called repeatedly in a loop
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
    */
    @Override
    public void loop() {
//        telemetry.addData("1 Start", "OrientOp started at " + startDate);
//        telemetry.addData("2 note1", "values below are in degrees" );
//        telemetry.addData("3 note2", "azimuth relates to magnetic north" );
//        telemetry.addData("4 note3", " " );
        telemetry.addData("azimuth", Math.round(Math.toDegrees(azimuth)));
        telemetry.addData("roll", Math.round(Math.toDegrees(roll)));
        telemetry.addData("pitch", Math.round(Math.toDegrees(pitch)));

        telemetry.addData("val1", com.getDirection());
        telemetry.addData("Ultra", ultra.getUltrasonicLevel());
        /*telemetry.addData("azimuth", m_azimuth_radians);
        telemetry.addData("pitch", m_pitch_radians);
        telemetry.addData("Mag", mGeomagnetic[0]);
        telemetry.addData("Mag", mGeomagnetic[1]);
        telemetry.addData("Mag", mGeomagnetic[2]);
        telemetry.addData("Grav", mGravity[0]);
        telemetry.addData("Grav", mGravity[1]);
        telemetry.addData("Grav", mGravity[2]);*/
    }

    /*
    * Code to run when the op mode is first disabled goes here
    * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
    */
    @Override
    public void stop() {
        mSensorManager.unregisterListener(this);
    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // not sure if needed, placeholder just in case
    }

    public void onSensorChanged(SensorEvent event) {
        // we need both sensor values to calculate orientation
        // only one value will have changed when this method called, we assume we can still use the other value.
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            mGravity = event.values;
            m_Norm_Gravity = (float)Math.sqrt(mGravity[0]*mGravity[0] + mGravity[1]*mGravity[1] + mGravity[2]*mGravity[2]);
            for(int i=0; i < mGravity.length; i++) mGravity[i] /= m_Norm_Gravity;
        }
        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            mGeomagnetic = event.values;
            m_Norm_MagField = (float)Math.sqrt(mGeomagnetic[0]*mGeomagnetic[0] + mGeomagnetic[1]*mGeomagnetic[1] + mGeomagnetic[2]*mGeomagnetic[2]);
            for(int i=0; i < mGeomagnetic.length; i++) mGeomagnetic[i] /= m_Norm_MagField;
        }
        if (mGravity != null && mGeomagnetic != null) {  //make sure we have both before calling getRotationMatrix
            /*float East_x = mGeomagnetic[1]*mGravity[2] - mGeomagnetic[2]*mGravity[1];
            float East_y = mGeomagnetic[2]*mGravity[0] - mGeomagnetic[0]*mGravity[2];
            float East_z = mGeomagnetic[0]*mGravity[1] - mGeomagnetic[1]*mGravity[0];
            float norm_East = (float)Math.sqrt(East_x * East_x + East_y * East_y + East_z * East_z);
            if (m_Norm_Gravity * m_Norm_MagField * norm_East < 0.1f) {  // Typical values are  > 100.
                m_OrientationOK = false; // device is close to free fall (or in space?), or close to magnetic north pole.
            } else {
                m_NormEastVector[0] = East_x / norm_East; m_NormEastVector[1] = East_y / norm_East; m_NormEastVector[2] = East_z / norm_East;

                // next calculate the horizontal vector that points due north
                float M_dot_G = (mGravity[0] *mGeomagnetic[0] + mGravity[1]*mGeomagnetic[1] + mGravity[2]*mGeomagnetic[2]);
                float North_x = mGeomagnetic[0] - mGravity[0] * M_dot_G;
                float North_y = mGeomagnetic[1] - mGravity[1] * M_dot_G;
                float North_z = mGeomagnetic[2] - mGravity[2] * M_dot_G;
                float norm_North = (float)Math.sqrt(North_x * North_x + North_y * North_y + North_z * North_z);
                m_NormNorthVector[0] = North_x / norm_North; m_NormNorthVector[1] = North_y / norm_North; m_NormNorthVector[2] = North_z / norm_North;

                // take account of screen rotation away from its natural rotation

                // calculate all the required angles from the rotation matrix
                // NB: see http://math.stackexchange.com/questions/381649/whats-the-best-3d-angular-co-ordinate-system-for-working-with-smartfone-apps
                float sin = m_NormEastVector[1] -  m_NormNorthVector[0], cos = m_NormEastVector[0] +  m_NormNorthVector[1];
                m_azimuth_radians = (float) (sin != 0 && cos != 0 ? Math.atan2(sin, cos) : 0);
                m_pitch_radians = (float) Math.acos(mGravity[2]);
                sin = -m_NormEastVector[1] -  m_NormNorthVector[0]; cos = m_NormEastVector[0] -  m_NormNorthVector[1];
                float aximuth_plus_two_pitch_axis_radians = (float)(sin != 0 && cos != 0 ? Math.atan2(sin, cos) : 0);
                m_pitch_axis_radians = (float)(aximuth_plus_two_pitch_axis_radians - m_azimuth_radians) / 2;
                m_OrientationOK = true;
            }*/
            readRun++;
            readRun = (readRun==sample) ? 0 : readRun;

            float R[] = new float[9];
            float I[] = new float[9];
            boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
            if (success) {
                float orientation[] = new float[3];
                SensorManager.getOrientation(R, orientation);
                azimuth = orientation[0]; // orientation contains: azimuth, pitch and roll
                pitch = orientation[1];
                roll = orientation[2];
                orient[readRun] = azimuth;
                double mean = 0;
                for(int i=0; i<orient.length; i++){
                    mean+= (orient[i]/orient.length);
                }
               // telemetry.addData("mean", mean);
                //telemetry.addData("raw", orient[0]);

                double variancesum = 0;
                for(int i=0; i<orient.length; i++) {
                    variancesum += (orient[i] - mean) * (orient[i] - mean);
                }
                double variance = variancesum/(double)(orient.length-1);
                //telemetry.addData("varience", variance);
                double SD = Math.sqrt(variance);

                double finalsum = 0;
                int run =0;
                for(int i =0; i<orient.length; i++){
                    if(Math.abs(orient[i]-mean)<Math.abs(SD)) {
                        finalsum += (double)orient[i];
                        run++;
                    }
                }
                //telemetry.addData("az", run);
                //azimuth = (float)finalsum/(float)run;
            }
        }
    }
}