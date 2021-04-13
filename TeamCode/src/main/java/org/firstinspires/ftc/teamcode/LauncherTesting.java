package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "launchAuto")
//@Disabled
public class LauncherTesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        HardwareMapClass robo= new HardwareMapClass();
        robo.init(hardwareMap);

        waitForStart();
        if (opModeIsActive()) { // Linear OpMode
            robo.frMotor.setPower(.9);
            robo.flMotor.setPower(.9);
            robo.brMotor.setPower(.9);
            robo.blMotor.setPower(.9);
            sleep(2000);
            robo.frMotor.setPower(0);
            robo.flMotor.setPower(0);
            robo.brMotor.setPower(0);
            robo.blMotor.setPower(0);
            sleep(2000);
            robo.whileLoop(opModeIsActive());
//            robo.launcherL.setVelocity(-500);
//            robo.launcherR.setVelocity(700);
//            sleep(1000);
//            robo.launcherL.setVelocity(0);
//            robo.launcherR.setVelocity(0);
//            timer.reset();
//            robo.whileLoop(opModeIsActive(), timer.time()<3);
        }
    }
}
