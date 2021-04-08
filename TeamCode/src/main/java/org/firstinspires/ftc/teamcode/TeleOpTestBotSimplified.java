package org.firstinspires.ftc.teamcode;/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

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

//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "TestBotSimplified", group = "Example")
//@Disabled
public class TeleOpTestBotSimplified extends OpMode {

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    HardwareMapClass robo= new HardwareMapClass();


    @Override
    public void init() {
        robo.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void init_loop() {
        robo.gyroAngles = robo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("1", "Integrated Heading: " + robo.getIntegratedHeading());
    }


    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {

        robo.gyroAngles = robo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robo.y = Range.clip(gamepad1.left_stick_x, -1, 1);
        robo.x = Range.clip(gamepad1.left_stick_y, -1, 1);
        robo.joystickAngle = Math.atan2(-robo.x, robo.y);
        robo.joystickAngle360 = robo.joystickAngle >= 0 ? robo.joystickAngle : (2 * Math.PI) + robo.joystickAngle;

        robo.driveSpeed = Range.clip(Math.sqrt(robo.y * robo.y + robo.x * robo.x), -1, 1);
        if (gamepad1.right_trigger > .05 || gamepad1.left_trigger > .05) {
            robo.driveRotation = gamepad1.left_trigger - gamepad1.right_trigger;
            robo.desiredRobotHeading = robo.getIntegratedHeading();
        } else if (Math.abs(robo.desiredRobotHeading - robo.getIntegratedHeading()) > 5) {
            robo.driveRotation = (robo.desiredRobotHeading - robo.getIntegratedHeading()) / (Math.abs(robo.desiredRobotHeading - robo.getIntegratedHeading())) * .05;
        } else {
            robo.driveRotation = 0;
        }

        if (gamepad1.dpad_up) {robo.robotPerspective = true;}
        if (gamepad1.dpad_down) {robo.robotPerspective = false;}

        if (robo.robotPerspective) { //Controls are mapped to the robot perspective
            robo.fieldReference = 0;
            //Positive values for x axis are joystick right
            //Positive values for y axis are joystick down
            robo.y = Range.clip(gamepad1.right_stick_y,-1,1);
            robo.x = Range.clip(gamepad1.right_stick_x,-1,1);
            robo.joystickAngle = Math.atan2(robo.x,robo.y);
        } else {   //Controls are mapped to the field
            robo.fieldReference = robo.desiredRobotHeading;
            //Positive values for x axis are joystick right
            //Positive values for y axis are joystick down
            robo.y = Range.clip(gamepad1.right_stick_x,-1,1);
            robo.x = Range.clip(-gamepad1.right_stick_y,-1,1);
            robo.joystickAngle = Math.atan2(robo.x,robo.y);
           // robo.joystickAngle360 = robo.joystickAngle >= 0 ? robo.joystickAngle : (2*Math.PI) + robo.joystickAngle;
        }
        robo.joystickAngle360 = robo.joystickAngle >= 0 ? robo.joystickAngle : (2*Math.PI) + robo.joystickAngle;

        //finding the hypotenuse to calculate drive speed
        robo.driveSpeed = Range.clip(Math.sqrt(robo.y * robo.y + robo.x * robo.x), -1, 1);

        //Calculate the power for each motor to translate in the desired direction
        robo.flPower = Math.cos(robo.joystickAngle360 - Math.toRadians(robo.fieldReference) + Math.PI / 4);
        robo.frPower = Math.sin(robo.joystickAngle360 - Math.toRadians(robo.fieldReference) + Math.PI / 4);
        robo.blPower = Math.sin(robo.joystickAngle360 - Math.toRadians(robo.fieldReference) + Math.PI / 4);
        robo.brPower = Math.cos(robo.joystickAngle360 - Math.toRadians(robo.fieldReference) + Math.PI / 4);



//        robo.flPower = Math.cos(robo.joystickAngle360 - Math.toRadians(robo.desiredRobotHeading) + Math.PI / 4);
//        robo.frPower = Math.sin(robo.joystickAngle360 - Math.toRadians(robo.desiredRobotHeading) + Math.PI / 4);
//        robo.blPower = Math.sin(robo.joystickAngle360 - Math.toRadians(robo.desiredRobotHeading) + Math.PI / 4);
//        robo.brPower = Math.cos(robo.joystickAngle360 - Math.toRadians(robo.desiredRobotHeading) + Math.PI / 4);
        robo.maxMotorPower = Math.max(Math.max(Math.max(Math.abs(robo.flPower), Math.abs(robo.frPower)), Math.abs(robo.blPower)), Math.abs(robo.brPower));

        //Ratio the powers for direction
        robo.flPower = robo.flPower / robo.maxMotorPower;
        robo.frPower = robo.frPower / robo.maxMotorPower;
        robo.blPower = robo.blPower / robo.maxMotorPower;
        robo.brPower = robo.brPower / robo.maxMotorPower;

        robo.flPower = robo.driveSpeed * robo.flPower - robo.driveRotation;
        robo.frPower = robo.driveSpeed * robo.frPower + robo.driveRotation;
        robo.blPower = robo.driveSpeed * robo.blPower - robo.driveRotation;
        robo.brPower = robo.driveSpeed * robo.brPower + robo.driveRotation;

        robo.maxMotorPower = Math.max(Math.max(Math.max(Math.abs(robo.flPower), Math.abs(robo.frPower)), Math.abs(robo.blPower)), Math.abs(robo.brPower));

        if (Math.abs(robo.maxMotorPower) > 1) {
            robo.flPower = robo.flPower / robo.maxMotorPower;
            robo.frPower = robo.frPower / robo.maxMotorPower;
            robo.blPower = robo.blPower / robo.maxMotorPower;
            robo.brPower = robo.brPower / robo.maxMotorPower;
        } else if (Math.abs(robo.maxMotorPower) < .03) {
            robo.flPower = 0;
            robo.frPower = 0;
            robo.blPower = 0;
            robo.brPower = 0;
        }
        if(gamepad1.left_bumper)
        {
            robo.flMotor.setPower(robo.flPower*.5);
            robo.frMotor.setPower(robo.frPower*.5);
            robo.blMotor.setPower(robo.blPower*.5);
            robo.brMotor.setPower(robo.brPower*.5);
        }
        else{
            robo.flMotor.setPower(robo.flPower);
            robo.frMotor.setPower(robo.frPower);
            robo.blMotor.setPower(robo.blPower);
            robo.brMotor.setPower(robo.brPower);
        }
        if(gamepad1.dpad_up){
            robo.greenLED.setState(false);
            robo.redLED.setState(true);
        }
        else if(gamepad1.dpad_right){
            robo.greenLED.setState(true);
            robo.redLED.setState(false);
        }
        else if(gamepad1.dpad_down){
            robo.greenLED.setState(false);
            robo.redLED.setState(false);
        }


        if (gamepad1.a){
            robo.launcherL.setVelocity(600);
            robo.launcherR.setVelocity(-400);
        }
        if (gamepad1.b){
            robo.launcherL.setVelocity(0);
            robo.launcherR.setVelocity(0);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Wobble counts", brMotor.getCurrentPosition());
    }
    @Override
    public void stop() {

    }


}
