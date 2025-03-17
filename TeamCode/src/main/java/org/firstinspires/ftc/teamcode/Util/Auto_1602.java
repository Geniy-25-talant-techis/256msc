package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name = "auto_16.02")
public class Auto_1602 extends LinearOpMode {

   private DcMotorEx Lift_M = null;
    private DcMotorEx Lift_TS = null;
    private Servo Povorot;
    private Servo Clash;
   private DcMotorEx LeftFrontDrive = null;
    private DcMotorEx RightFrontDrive = null;
    private DcMotorEx LeftBackDrive = null;
    private DcMotorEx RightBackDrive = null;

    @Override
    public void runOpMode() throws InterruptedException {
     LeftFrontDrive = hardwareMap.get(DcMotorEx.class,"MFL");
     RightFrontDrive = hardwareMap.get(DcMotorEx.class,"MFR");
     LeftBackDrive = hardwareMap.get(DcMotorEx.class,"MBR");
     RightBackDrive = hardwareMap.get(DcMotorEx.class,"MBL");
     Lift_M = hardwareMap.get(DcMotorEx.class,"MLM");
     Lift_TS = hardwareMap.get(DcMotorEx.class,"MLV");
     Povorot = hardwareMap.get(Servo.class,"SLP");
     Clash = hardwareMap.get(Servo.class,"SLC");


     LeftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
     LeftBackDrive.setDirection(DcMotor.Direction.REVERSE);
     RightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
     RightBackDrive.setDirection(DcMotor.Direction.REVERSE);

     LeftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     LeftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     RightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


     waitForStart();
        LeftBackDrive.setPower(-0.7*0.8);
        LeftFrontDrive.setPower(0.7); //фейк колесо
        RightFrontDrive.setPower(-0.7);
        RightBackDrive.setPower(-0.7);

     sleep(100);




    }
}
