package org.firstinspires.ftc.teamcode_Spark;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;


/**
 * Created by nqmet on 9/23/2017.
 */


@TeleOp(name = "#Memes", group = "Test")
public class Test extends LinearOpMode {
    RobotControl robot;
    boolean red = true;
    boolean relicGrabbed=false;
    boolean armDown=false;
    boolean liftCheck = false;
    boolean intakeRun = false;
    int pastPosition = 0;
    int harvesterDir = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotControl(this);


        waitForStart();




        while (opModeIsActive()) {




            robot.setMotors( (float)(gamepad1.left_stick_y), (float) (-gamepad1.left_stick_x), (float) (gamepad1.right_stick_x));
            robot.intake(gamepad1.right_trigger > .1, gamepad1.right_bumper);


            if(gamepad2.dpad_up){
                liftCheck = true;
                robot.lift.setPower(.6);
            }
            else{
                liftCheck = false;
                robot.lift.setPower(0);
            }
            if(gamepad2.dpad_down){
                liftCheck = true;
                robot.lift.setPower(-.6);
            }
            else{
                liftCheck = false;
                robot.lift.setPower(0);
            }


            if(gamepad1.a){

            }
            else{
                robot.belt.setPower(0);
            }







            if(gamepad1.y) robot.speedController();





            robot.relicM.setPower(Math.abs(gamepad2.left_stick_y)*.8);

            if(intakeRun && robot.eHarvest.getCurrentPosition() == pastPosition){
                harvesterDir = -1;
            }
            else{
                harvesterDir = 1;
            }


            if(gamepad2.y){
                armDown = true;
                robot.armServo.setPosition(.2);
            }
            else{
                armDown = false;
                robot.armServo.setPosition(.6);
            }



            if(gamepad2.left_bumper){
                relicGrabbed = true;
                robot.clawServo.setPosition(.6);
            }
            else{
                relicGrabbed = false;
               robot.clawServo.setPosition(.8);
            }
           // robot.balance(gamepad1.a);



//            if(gamepad2.right_trigger>.1 && gamepad2.left_trigger >.1){
//                robot.relicM.setPower(.8);
//                robot.runtime.reset();
//                while(robot.runtime.milliseconds() < 500);
//                robot.runtime.reset();
//                robot.armServo.setPosition(.2);
//                while(robot.runtime.milliseconds() < 1000);
//                robot.runtime.reset();
//                robot.clawServo.setPosition(.6);
//                while(robot.runtime.milliseconds() < 500);
//                robot.runtime.reset();
//                robot.relicM.setPower(.2);
//                while(robot.runtime.milliseconds() < 500);
//                robot.runtime.reset();
//                robot.armServo.setPosition(.6);
//                while(robot.runtime.milliseconds() < 500);
//                robot.runtime.reset();
//                robot.relicM.setPower(0);
//            }








            if(gamepad1.right_bumper){
                robot.eHarvest.setPower(1);
                robot.wHarvest.setPower(-1);
                robot.belt.setPower(.7);

            }
            else if(gamepad1.right_trigger > .1) {

                robot.belt.setPower(-.7);
                robot.eHarvest.setPower(-1*harvesterDir);
                robot.wHarvest.setPower(1*harvesterDir);
            }
            else{
                robot.belt.setPower(0);

                robot.eHarvest.setPower(0);
                robot.wHarvest.setPower(0);
            }
            int colorTarget;
            if(gamepad1.a){

                //Selection Num red>22
                colorTarget = robot.eColor.red();
            }
            else{
                //Selection num blue>19
                colorTarget = robot.eColor.blue();
            }


            telemetry.addData("HEADING: ", robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("PITCH: ", robot.imu.getAngularOrientation().secondAngle);
            telemetry.addData("ROLL: ", robot.imu.getAngularOrientation().thirdAngle);
            telemetry.addData("COLORe", colorTarget);
            telemetry.addData("CONTROL_SPEED: ", robot.controlSpeed);


            telemetry.update();
        }

        if(!opModeIsActive()){

        }



    }


}