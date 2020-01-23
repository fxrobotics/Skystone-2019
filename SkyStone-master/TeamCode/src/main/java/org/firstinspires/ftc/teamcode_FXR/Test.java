package org.firstinspires.ftc.teamcode_FXR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;


/**
 * Created by ejkat on 9/23/2017.
 */


@TeleOp(name = "#TeleOp", group = "Test")
public class Test extends LinearOpMode {
    RobotControl robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotControl(this);

        waitForStart();

        while (opModeIsActive()) {




            robot.setMotors( (float)(gamepad1.left_stick_y), (float) (-gamepad1.left_stick_x), (float) (gamepad1.right_stick_x));
            if(gamepad1.y) robot.speedController();
            if(gamepad1.a){
                robot.lintake.setPower(1);
                robot.rintake.setPower(-1);
            } else if(gamepad1.b){
                robot.lintake.setPower(-1);
                robot.rintake.setPower(1);
            } else{
                robot.lintake.setPower((0));
                robot.rintake.setPower(0);
            }

            if(gamepad2.left_bumper){
                robot.pinchRotate.setPosition(.5);
            }
            if (gamepad2.right_bumper){
                robot.pinchRotate.setPosition(0);
            }
            if (gamepad2.a) robot.pinch.setPosition(.42);
            if (gamepad2.b) robot.pinch.setPosition(.1);


            telemetry.addData("CONTROL_SPEED: ", robot.controlSpeed);
            telemetry.addData("Pinch Pos: ", robot.pinch.getPosition());
            telemetry.addData("PinchRot Pos: ", robot.pinchRotate.getPosition());


            telemetry.update();
        }

        if(!opModeIsActive()){

        }



    }


}