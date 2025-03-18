package org.firstinspires.ftc.teamcode.ModuleInit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Initilization {
    BNO055IMU imu;
    public LinearOpMode l = null;
    public boolean Max = false;

    // State used for updating telemetry
    Orientation angles;
    public DcMotorEx Lift_Moment = null;
    public DcMotorEx Lift_TS = null;
    //
    public DcMotor LeftFrotnDrive = null;
    public DcMotor RightFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor RightBackDrive = null;
    public Servo Pvrt, Clash;
    public DigitalChannel Low_Drive, Max_Drive, Max_Static;
    public String status;
    public String game = "Game";
    public Boolean revers = false;
    public Telemetry t = null;

    ElapsedTime time = new ElapsedTime();

    public void init_hwm_tele(HardwareMap hardwareMap) {
        LeftFrotnDrive = hardwareMap.get(DcMotor.class, "MFL");
        RightFrontDrive = hardwareMap.get(DcMotor.class, "MFR");
        leftBackDrive = hardwareMap.get(DcMotor.class, "MBL");
        RightBackDrive = hardwareMap.get(DcMotor.class, "MBR");
        Lift_Moment = hardwareMap.get(DcMotorEx.class, "MLM");
        Lift_TS = hardwareMap.get(DcMotorEx.class, "MLTS");
        Pvrt = hardwareMap.get(Servo.class, "SLC");
        Clash = hardwareMap.get(Servo.class, "SLP");
        Low_Drive = hardwareMap.get(DigitalChannel.class, "Static");
        Max_Drive = hardwareMap.get(DigitalChannel.class, "Max");
        Max_Static = hardwareMap.get(DigitalChannel.class, "Max_Low");
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

    public void init_hwm_auto(HardwareMap hardwareMap) {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

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
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) {
        }
    }


    public enum Lift {
        Start,
        High_Comp,
        Low_Comp,
        Hand,
        stop
    }

    public Lift Lift_Control;
    public int start_pos_Lift_moment = 0;
    public int start_pow_Lift_TS = 0;
    public static int up = 308;
    public static int max = 5450;
    public static int zero = 0;
    public static int Low = 0;
    public double power_moment;
    public double power_lift;
    public double Speed_Up = 0.2;
    public double Speed_Low = -0.2;

    public Thread Lift_auto = new Thread() {
        public void run() {
            double error;
            int Lift_moment_max = up;
            int Lift_moment_min = zero;
            start_pos_Lift_moment = Lift_Moment.getCurrentPosition();
            start_pow_Lift_TS = Lift_TS.getCurrentPosition();

            Lift_TS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Lift_TS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Lift_Moment.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Lift_Moment.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (!l.isStopRequested()) {
                switch (Lift_Control) {
                    case stop:
                        Lift_TS.setPower(0);
                        Lift_Moment.setPower(0);
                        Lift_TS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        Lift_Moment.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        break;
                    case Start:
                            while ( Low_Drive.getState() != true || Lift_Moment.getCurrentPosition() > zero) {
                                if (Low_Drive.getState()!=true) {
                                    Lift_TS.setPower(0);
                                    Lift_TS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                    Lift_TS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                    revers = false;
                                }else {
                                        Lift_TS.setPower(Speed_Low);
                                    }
                                if(Lift_Moment.getCurrentPosition()<=zero){
                                    Lift_Moment.setPower(0);
                                    error = Lift_Moment.getCurrentPosition();
                                    Lift_TS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                    Lift_TS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                    Lift_moment_max += error;
                                    Lift_moment_min += error;

                                }else{
                                    Lift_Moment.setPower(Speed_Low);
                                }
                                Lift_Control = Lift_Control.stop;
                                break;
                                };
                    case High_Comp:
                        while(Max == false||Lift_Moment.getCurrentPosition()<Lift_moment_max){
                            if (Max == true) {
                                Lift_TS.setPower(0);
                                Lift_TS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                Lift_TS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                revers = true;
                            }else {
                                Lift_TS.setPower(Speed_Up);
                            }
                            if(Lift_Moment.getCurrentPosition()>=up){
                                Lift_Moment.setPower(0);
                                error = Lift_Moment.getCurrentPosition();
                            }else{
                                Lift_Moment.setPower(Speed_Up);
                            }
                        }
                        Lift_Control = Lift_Control.stop;
                        break;


                        }


                        }
                }
            };


        }//----------------------------------------------------------last----------------------------------





