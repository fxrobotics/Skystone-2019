package org.firstinspires.ftc.teamcode_FXR;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import java.util.ArrayList;


/**
 * Created by ejkat19 on 9/9/2017.
 */
public class RCPushBot {
    public final HardwareMap hwMap;
    private LinearOpMode linearOpMode = null;
    DcMotor left, right;
    Servo grab;
    BNO055IMU imu;
    ElapsedTime runtime;
    double controlSpeed = .1;


    RCPushBot(LinearOpMode opMode) {



        linearOpMode = opMode;
        hwMap = linearOpMode.hardwareMap;
        runtime = new ElapsedTime();

        this.left = this.hwMap.dcMotor.get("left");
        this.right = this.hwMap.dcMotor.get("right");
        this.grab = this.hwMap.servo.get("grab");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.imu = this.hwMap.get(BNO055IMU.class, "imu");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);



    }


    public void stop(){
        drive(0,0);
    }

    public void drive(float x, float y) {
        double yPos = (double) y, xPos = (double) x;
        double leftPower, rightPower;

        xPos *= -100;
        yPos *= 100;
        double V = (100-Math.abs(xPos)) * (yPos/100) + yPos;
        double W = (100-Math.abs(yPos)) * (xPos/100) + xPos;

        leftPower = Range.clip((V+W)/200, -1, 1);
        rightPower = Range.clip((V-W)/200, -1, 1);

        //from here on is just setting motor values
        left.setPower(controlSpeed*leftPower);
        right.setPower(controlSpeed*rightPower);

    }
    private void defineVariables(HardwareMap _hwMap){
        HardwareMap hwMap = _hwMap;
    }


    public void timeout(int millis){
        double startTime = runtime.milliseconds();
        while(runtime.milliseconds() < startTime + millis);
        return;
    }
    public void speedController(){
        controlSpeed += .3;
        if (controlSpeed == 1.3) {
            controlSpeed = .1;
        }

    }
}