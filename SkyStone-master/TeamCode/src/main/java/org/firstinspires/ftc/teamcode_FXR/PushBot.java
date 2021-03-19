package org.firstinspires.ftc.teamcode_FXR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;


/**
 * Created by ejkat on 9/23/2017.
 */


@TeleOp(name = "#Pushbot", group = "Test")
public class PushBot extends LinearOpMode {
    RCPushBot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RCPushBot(this);

        waitForStart();

        while (opModeIsActive()) {



            robot.drive( (float)(gamepad1.left_stick_y), (float) (gamepad1.left_stick_x));
            if(gamepad1.y) robot.speedController();
            if(gamepad1.b) robot.stop();
            if(gamepad1.a) robot.grab.setPosition(-gamepad1.left_trigger);
            else robot.grab.setPosition(gamepad1.left_trigger);

            telemetry.addData("CONTROL_SPEED: ", robot.controlSpeed);


            telemetry.update();
        }

        if(!opModeIsActive()){

        }



    }


}