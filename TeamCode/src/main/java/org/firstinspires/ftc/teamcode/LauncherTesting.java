package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "launchAuto")
//@Disabled
public class LauncherTesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        HardwareMapClass robo= new HardwareMapClass();
        robo.init(hardwareMap);
        waitForStart();
        if (opModeIsActive()) { // Linear OpMode
            robo.launcherL.setVelocity(-500);
            robo.launcherR.setVelocity(700);
            sleep(1000);
            robo.launcherL.setVelocity(0);
            robo.launcherR.setVelocity(0);


        }


    }

}
