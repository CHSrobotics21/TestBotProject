package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoLinearExtend")
//@Disabled
public class AutoLinearExtend extends HardwareMapClassLinearExtend {

    @Override
    public void runOpMode() {
        super.init();

        waitForStart();
        if (opModeIsActive()) { // Linear OpMode
            frMotor.setPower(.9);
            flMotor.setPower(.9);
            brMotor.setPower(.9);
            blMotor.setPower(.9);
            sleep(2000);
            frMotor.setPower(0);
            flMotor.setPower(0);
            brMotor.setPower(0);
            blMotor.setPower(0);
            sleep(2000);
            psuedoGoToPosition();
//            launcherL.setVelocity(-500);
//            launcherR.setVelocity(700);
//            sleep(1000);
//            launcherL.setVelocity(0);
//            launcherR.setVelocity(0);
//            timer.reset();
//            whileLoop(opModeIsActive(), timer.time()<3);
        }
    }
}
