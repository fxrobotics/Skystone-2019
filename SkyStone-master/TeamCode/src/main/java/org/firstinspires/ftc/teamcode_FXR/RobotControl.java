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
public class RobotControl {
    public final HardwareMap hwMap;
    private LinearOpMode linearOpMode = null;
    DcMotor ne, nw, se, sw;
    CRServo lintake, rintake;
    Servo pinch, pinchRotate;
    BNO055IMU imu;
    ElapsedTime runtime;
    double controlSpeed = .1;


    RobotControl(LinearOpMode opMode) {



        linearOpMode = opMode;
        hwMap = linearOpMode.hardwareMap;
        runtime = new ElapsedTime();

        this.ne = this.hwMap.dcMotor.get("ne");
        this.nw = this.hwMap.dcMotor.get("nw");
        this.se = this.hwMap.dcMotor.get("se");
        this.sw = this.hwMap.dcMotor.get("sw");
        this.lintake = this.hwMap.crservo.get("lintake");
        this.rintake = this.hwMap.crservo.get("rintake");
        this.pinch = this.hwMap.servo.get("pinch");
        this.pinchRotate = this.hwMap.servo.get("pinchRotate");


        ne.setDirection(DcMotorSimple.Direction.REVERSE);
        nw.setDirection(DcMotorSimple.Direction.REVERSE);
        se.setDirection(DcMotorSimple.Direction.REVERSE);
        sw.setDirection(DcMotorSimple.Direction.REVERSE);
        ne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        se.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        nw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public boolean align(){
        int limit = 3;
        double power = .8;
        while(this.imu.getAngularOrientation().firstAngle <limit*-1 &&this.imu.getAngularOrientation().firstAngle > limit){
            if(this.imu.getAngularOrientation().firstAngle<limit*-1){
                this.drive(0,0,power);
            }
            else if(this.imu.getAngularOrientation().firstAngle > limit){
                this.drive(0,0,.8*-1);
            }

        }
        this.stop();
        return true;
    }


    public void drive(double angle, double power, double rot) {
        setMotors((float) (power * Math.sin(Math.toRadians(angle))), (float) (power * -Math.cos(Math.toRadians(angle))), (float) rot);
    }

    public void stop(){
        setMotors(0,0,0);
    }

    public void setMotors(float x, float y, float rot) {
        double drive = (double) y, strafe = (double) x, spin = (double) rot;
        double nePower, nwPower, sePower, swPower;

        nwPower = Range.clip(drive + strafe + spin, -1, 1);
        swPower = Range.clip(drive - strafe + spin, -1, 1);
        nePower = Range.clip(drive - strafe - spin, -1, 1);
        sePower = Range.clip(drive + strafe - spin, -1, 1);
        nePower = -(nePower);
        sePower = -(sePower);

        //from here on is just setting motor values
        ne.setPower(controlSpeed*nePower);
        se.setPower(controlSpeed*sePower);
        sw.setPower(controlSpeed*swPower);
        nw.setPower(controlSpeed*nwPower);

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
     //Pinch pos is .42 is up, .2 is horiz,
    //PinchRot 0 is straight, .5 is turned
}