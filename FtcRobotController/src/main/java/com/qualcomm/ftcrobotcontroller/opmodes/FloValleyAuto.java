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
    final int ULTRA_THRESH = 10;
    final double CLIMBER_POSITION = 0.9;
    final double REST_POSITION = 0.0;
    double large_arc_inner = 0.83606;
    double large_arc_outer = 1.0;

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
            telemetry.addData("Blue(X) or Red(B):", side);
            telemetry.addData("Delay(D-Pad):", delay);
            telemetry.addData("UltraSonic", ultra.getUltrasonicLevel());
        }
        sleep(delay * 1000);
        //autoArm.setPosition(auto);
        if (side == "Red") {
            motorRight.setPower(large_arc_outer);
            motorLeft.setPower(large_arc_inner);
        } else {
            motorRight.setPower(large_arc_inner);
            motorLeft.setPower(large_arc_outer);
        }
        double dist = ultra.getUltrasonicLevel();
        while(dist>ULTRA_THRESH || dist<3.0) {
            dist = ultra.getUltrasonicLevel();
            telemetry.addData("UltraSonic", dist);
        }
        telemetry.addData("UltraSonic2", dist);
        motorRight.setPower(0.0);
        motorLeft.setPower(0.0);
        //telemetry.addData("UltraSonic", ultra.getUltrasonicLevel());
        for(int i=0; i<6; i++) {
            autoArm.setPosition(CLIMBER_POSITION);
            sleep(200);
            autoArm.setPosition(CLIMBER_POSITION - 0.1);
            sleep(200);
        }
        autoArm.setPosition(CLIMBER_POSITION);
        sleep(500);
        autoArm.setPosition(REST_POSITION);
        // wait for the start button to be pressed
        //waitForStart();

        // wait for the IR Seeker to detect a signal



        // wait for the robot to center on the beacon


        // now approach the beacon

    }
}
