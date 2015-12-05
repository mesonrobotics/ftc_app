/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.HiTechnicNxtUltrasonicSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A simple example of a linear op mode that will approach an IR beacon
 */
public class FloValleyAuto extends LinearOpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor intake;
    Servo autoArm;
    boolean in = false;
    float auto = 0.0f;
    private HiTechnicNxtUltrasonicSensor ultra;
    private final int ULTRA_PORT = 5;
    private String side = "Red";
    private int delay = 0;
    private String park[] = {"Floor Goal", "Repair Zone", "Ramp"};
    private int parkScroll = 0;
    final int ULTRA_THRESH = 10;
    final double CLIMBER_POSITION = 0.9;
    final double REST_POSITION = 0.0;
    double red_ratio = 0.82142/2;
    double blue_ratio = 0.84848/2;
    final int FULL_CCW_TURN_MS = 11720;

    @Override
    public void runOpMode() throws InterruptedException {

        // set up the hardware devices we are going to use
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection((DcMotor.Direction.REVERSE));
        autoArm = hardwareMap.servo.get("autoArm");

        ultra = (HiTechnicNxtUltrasonicSensor)hardwareMap.ultrasonicSensor.get("ultra");

        while(!opModeIsActive()) {
            telemetry.addData("Blue(X) or Red(B):", side);
            telemetry.addData("Delay(D-Pad):", delay);
            telemetry.addData("Ramp or ParkingZone(Bumpers)", park[parkScroll]);

            if (gamepad1.x || gamepad2.x) {
                side = "Blue";
            }
            if (gamepad1.b || gamepad2.b) {
                side = "Red";
            }
            if(gamepad1.dpad_up || gamepad2.dpad_up){
                delay++;
                sleep(200);
            }
            if(gamepad1.dpad_down || gamepad2.dpad_down){
                delay--;
                sleep(200);
            }
            if(gamepad1.right_bumper || gamepad2.right_bumper){
                parkScroll ++;
                parkScroll = (parkScroll==3) ? 0 : parkScroll;
                sleep(200);
            }
            telemetry.addData("Blue(X) or Red(B):", side);
            telemetry.addData("Delay(D-Pad):", delay);
            telemetry.addData("Ramp or ParkingZone(Bumpers)", park[parkScroll]);
            telemetry.addData("UltraSonic", ultra.getUltrasonicLevel());
        }
        sleep(delay * 1000); //wait for 'delay' seconds

        intake.setPower(-0.9); //run intake backwards to expel any debris
        if (side == "Red") {
            motorRight.setPower(1.0);//make arc path
            motorLeft.setPower(red_ratio);
        } else {
            motorRight.setPower(blue_ratio);//make mirror arc path
            motorLeft.setPower(1.0);
        }
        double dist = ultra.getUltrasonicLevel();

        while(dist>ULTRA_THRESH || dist<3.0) {
            dist = ultra.getUltrasonicLevel();
            telemetry.addData("UltraSonic", dist);//runs until US reads < ULTRA_THRESH
        }

        telemetry.addData("UltraSonic2", dist);//print last US value
        motorRight.setPower(0.0);//set all motors to 0 to stop moving
        motorLeft.setPower(0.0);
        intake.setPower(0.0);

        for(int i=0; i<6; i++) { //dump out climbers - shake for security
            autoArm.setPosition(CLIMBER_POSITION);
            sleep(200);
            autoArm.setPosition(CLIMBER_POSITION - 0.1);
            sleep(200);
        }

        autoArm.setPosition(CLIMBER_POSITION);
        sleep(1000);
        autoArm.setPosition(REST_POSITION); //reset climber arm
        sleep(1000);

        motorRight.setPower(-0.6); //back up
        motorLeft.setPower(-0.6);
        sleep(700);
        motorRight.setPower(0.0);
        motorLeft.setPower(0.0);

        if(parkScroll == 0) {//Floor Goal
            motorRight.setPower(0.9);
            motorLeft.setPower(-0.9);
            sleep((long)(0.25 * FULL_CCW_TURN_MS));
            motorRight.setPower(0.0);
            motorLeft.setPower(0.0);
            sleep(500);
            double dir = (side=="RED") ? 1.0 : -1.0;
            motorLeft.setPower(dir * 0.6);
            motorRight.setPower(dir * 0.6);
            sleep(2000);
            motorLeft.setPower(0.0);
            motorRight.setPower(0.0);
        }

    }
}
