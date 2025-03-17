package org.firstinspires.ftc.teamcode.ModuleInit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Initilization {
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    public DcMotorEx Lift_Moment = null;
    public DcMotorEx Lift_TS = null;
//
    public DcMotor LeftFrotnDrive= null;
    public DcMotor RightFrontDrive= null;
    public DcMotor leftBackDrive= null;
    public DcMotor RightBackDrive = null;
    public Servo Pvrt,Clash;
    public DigitalChannel Low_Drive,Max_Drive,Max_Static;
    public void teleInit (HardwareMap hardwareMap){
        LeftFrotnDrive = hardwareMap.get(DcMotor.class,"MFL");
        RightFrontDrive = hardwareMap.get(DcMotor.class,"MFR");
        leftBackDrive = hardwareMap.get(DcMotor.class,"MBL");
        RightBackDrive = hardwareMap.get(DcMotor.class,"MBR");
        Lift_Moment = hardwareMap.get(DcMotorEx.class,"MLM");
        Lift_TS = hardwareMap.get(DcMotorEx.class,"MLTS");
        Pvrt = hardwareMap.get(Servo.class,"SLC");
        Clash = hardwareMap.get(Servo.class,"SLP");
        Low_Drive = hardwareMap.get(DigitalChannel.class,"Static");
        Max_Drive = hardwareMap.get(DigitalChannel.class,"Max");
        Max_Static = hardwareMap.get(DigitalChannel.class,"Max_Low");
Max_Static.setMode(DigitalChannel.Mode.INPUT);
        Low_Drive.setMode(DigitalChannel.Mode.INPUT);
        Max_Drive.setMode(DigitalChannel.Mode.INPUT);


        LeftFrotnDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        RightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        RightFrontDrive.setDirection(DcMotor.Direction.REVERSE); //прапвое заднее

        LeftFrotnDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift_TS.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Lift_Moment.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Lift_TS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Lift_Moment.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
Lift_TS.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift_Moment.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Lift_TS.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

    }
    public void autoinit(HardwareMap hardwareMap){


        LeftFrotnDrive = hardwareMap.get(DcMotor.class,"MFL");
        RightFrontDrive = hardwareMap.get(DcMotor.class,"MBR");
        leftBackDrive = hardwareMap.get(DcMotor.class,"MBL");
        RightBackDrive = hardwareMap.get(DcMotor.class,"MFR");
        Lift_Moment = hardwareMap.get(DcMotorEx.class,"MLM");
        Lift_TS = hardwareMap.get(DcMotorEx.class,"MLTS");
        Pvrt = hardwareMap.get(Servo.class,"SLC");
        Clash = hardwareMap.get(Servo.class,"SLP");
        Low_Drive = hardwareMap.get(DigitalChannel.class,"Static");
        Max_Drive = hardwareMap.get(DigitalChannel.class,"Max");
        Max_Static = hardwareMap.get(DigitalChannel.class,"Max_Low");
        Max_Static.setMode(DigitalChannel.Mode.INPUT);
        Low_Drive.setMode(DigitalChannel.Mode.INPUT);
        Max_Drive.setMode(DigitalChannel.Mode.INPUT);



        Clash.setDirection(Servo.Direction.REVERSE);

        LeftFrotnDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        RightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        RightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        LeftFrotnDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift_TS.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Lift_Moment.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Lift_TS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Lift_Moment.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Lift_TS.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift_Moment.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Lift_Moment.setTargetPosition(0);
        Lift_TS.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Lift_TS.setTargetPosition(0);

        RightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LeftFrotnDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        RightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LeftFrotnDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftFrotnDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LeftFrotnDrive.setTargetPosition(0);
        RightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RightFrontDrive.setTargetPosition(0);
        Pvrt.setPosition(0.5);
        Clash.setPosition(0);





    }
}
