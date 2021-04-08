package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwareMapClass {
    HardwareMap hwMap=null;
    public DcMotor frMotor=null, flMotor=null, brMotor=null, blMotor=null;
    public DcMotorEx launcherL = null, launcherR = null;
    public DigitalChannel redLED = null, greenLED = null;
    public BNO055IMU imu=null;
    public Orientation gyroAngles = null;
    public double frPower=0, flPower=0, brPower=0, blPower=0;
    public double maxMotorPower=0;
    public double driveSpeed = 0;
    public double driveRotation = 0;
    public double a, b, x, y, joystickAngle, joystickAngle360;
    public double desiredRobotHeading;
    public int rotations = 0;
    public boolean robotPerspective = false;
    public double fieldReference = 0.0;

    public HardwareMapClass(){

    }
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;
        frMotor = hwMap.dcMotor.get("frontright");
        flMotor = hwMap.dcMotor.get("frontleft");
        brMotor = hwMap.dcMotor.get("backright");
        blMotor = hwMap.dcMotor.get("backleft");
        launcherL = hwMap.get(DcMotorEx.class, "launcherLeft");
        launcherR = hwMap.get(DcMotorEx.class, "launcherRight");

        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        redLED = hwMap.get(DigitalChannel.class, "red");
        greenLED = hwMap.get(DigitalChannel.class, "green");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
    }
    public double getIntegratedHeading() {
        if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) > 200) {
            rotations++;
        }
        else if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) < -200) {
            rotations--;
        }

        return (rotations * 360 + gyroAngles.firstAngle);
    }
}
