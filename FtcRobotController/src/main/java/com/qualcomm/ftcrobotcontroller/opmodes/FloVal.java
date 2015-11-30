package com.qualcomm.ftcrobotcontroller.opmodes;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.text.SimpleDateFormat;
import java.util.Date;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by MESON on 10/9/2015.
 */
public class FloVal extends OpMode {

    /*
     * Note: the configuration of the servos is such that
     * as the arm servo approaches 0, the arm position moves up (away from the floor).
     * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
     */
    // TETRIX VALUES.


    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor intake;
    Servo   autoArm;
    boolean in = false;
    float auto = 0.0f;

//    private String startDate;
//    private SensorManager mSensorManager;
//    private Sensor accelerometer;
//    private Sensor magnetometer;
//
//    // orientation values
//    private float azimuth = 0.0f;      // value in radians
//    private float pitch = 0.0f;        // value in radians
//    private float roll = 0.0f;         // value in radians
//
//    private float[] mGravity;       // latest sensor values
//    private float[] mGeomagnetic;   // latest sensor values

    //private SensorManager mSensorManager;
    //private Sensor mGyro;
    //private final Context context;
    /**
     * Constructor
     */

    public FloVal(){

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
		 * For the demo Tetrix K9 bot we assume the following,
		 *   There are two motors "motor_1" and "motor_2"
		 *   "motor_1" is on the right side of the bot.
		 *   "motor_2" is on the left side of the bot.
		 *
		 * We also assume that there are two servos "servo_1" and "servo_6"
		 *    "servo_1" controls the arm joint of the manipulator.
		 *    "servo_6" controls the claw joint of the manipulator.
//		 */
//        mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
//        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
//        magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
//
//        azimuth = 0.0f;      // value in radians
//        pitch = 0.0f;        // value in radians
//        roll = 0.0f;

        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection((DcMotor.Direction.REVERSE));
        autoArm = hardwareMap.servo.get("autoArm");


        //mSensorManager = (SensorManager)context.getSystemService(Context.SENSOR_SERVICE);
        //mGyro = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
    }

    @Override
    public void start() {
//        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
//
//        // delay value is SENSOR_DELAY_UI which is ok for telemetry, maybe not for actual robot use
//        mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_UI);
//        mSensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_UI);
    }


    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        if(gamepad2.left_bumper){
            auto = 0.9f;
        }
        else if(gamepad2.right_bumper){
            auto = 0.0f;
        }
        if(gamepad1.right_bumper){
            left = -0.9f;
            right = 0.9f;
        }
        else{
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
        }

        if(gamepad1.a){
            in = !in;
        }
        float intakePow = (in) ? 0.9f : 0.0f;
        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        intakePow = Range.clip(intakePow, -1, 1);
        auto = Range.clip(auto, 0, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
       // right = (float)scaleInput(right);
        //left = (float)scaleInput(left);
        //double distance = v_sensor_ods.getLightDetected();
        // write the values to the motors
        motorRight.setPower(right);
        motorLeft.setPower(left);
        intake.setPower(intakePow);
        autoArm.setPosition(auto);

        // update the position of the arm.


		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("Left Joystick", "left joy: " + String.format("%.2f", gamepad1.left_stick_y));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
        telemetry.addData("Right Joystick", "right joy: " + String.format("%.2f", gamepad1.right_stick_y));
/*        telemetry.addData("azimuth", Math.round(Math.toDegrees(azimuth)));
//        telemetry.addData("pitch", Math.round(Math.toDegrees(pitch)));
//        telemetry.addData("roll", Math.round(Math.toDegrees(roll)));*/
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

//    public void onAccuracyChanged(Sensor sensor, int accuracy) {
//        // not sure if needed, placeholder just in case
//    }
//
//    public void onSensorChanged(SensorEvent event) {
//        // we need both sensor values to calculate orientation
//        // only one value will have changed when this method called, we assume we can still use the other value.
//        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
//            mGravity = event.values;
//        }
//        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
//            mGeomagnetic = event.values;
//        }
//        if (mGravity != null && mGeomagnetic != null) {  //make sure we have both before calling getRotationMatrix
//            float R[] = new float[9];
//            float I[] = new float[9];
//            boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
//            if (success) {
//                float orientation[] = new float[3];
//                SensorManager.getOrientation(R, orientation);
//                azimuth = orientation[0]; // orientation contains: azimuth, pitch and roll
//                pitch = orientation[1];
//                roll = orientation[2];
//            }
//        }
//    }
}

