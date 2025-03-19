package org.firstinspires.ftc.teamcode.ModuleInit;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Config
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


    ElapsedTime time = new ElapsedTime();

    public void init_hwm_tele(HardwareMap hardwareMap, LinearOpMode l) {
        this.l = l;
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

        Lift_TS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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


    public enum LiftTSMods {
        Low,
        maxx,
        midle,
        Hand,
        stop,
        HAND,
        BORT
    }


    public LiftTSMods LiftTSMode;
    public int start_pos_Lift_moment = 0;
    public int start_pow_Lift_TS = 0;
    public static int up = 308;
    public static int bort = 150;
    public static int mid = 2725;

    public static int max = 5450;
    public static int zero = 0;
    public static int Low = 0;
    public double power_moment;
    public double power_lift;
    public double Speed_Up = 0.2;
    public double Speed_Low = -0.2;

    public Thread Lift_auto_Ts = new Thread() {//поток для управления стрелой
        @Override
        public void run() {
            double error;
            start_pow_Lift_TS = Lift_TS.getCurrentPosition();
            while (!l.isStopRequested()) {
                switch (LiftTSMode) {
                    default:
                        break;
                    case stop:
                        Lift_TS.setPower(0);
                        break;
                    case Low:
                            while (Low_Drive.getState() != true) {
                                Lift_TS.setPower(Speed_Low);
                            }
                        Lift_TS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Lift_TS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                LiftTSMode = Initilization.LiftTSMods.stop;
                                revers = false;
                                break;
                    case maxx:
                        while(Max == false){
                            Lift_TS.setPower(Speed_Up);
                            }
                        Lift_TS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        Lift_TS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        LiftTSMode = Initilization.LiftTSMods.stop;
                        revers = true;
                        break;
                    case midle:

                        if(revers){
                            Lift_TS.setTargetPosition(mid - max);
                        }else{
                            Lift_TS.setTargetPosition(mid);
                        }
                        Lift_TS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Lift_TS.setPower(Speed_Up);
                        while(Lift_TS.isBusy()){

                        }
                        LiftTSMode = Initilization.LiftTSMods.stop;
                        break;
                    case BORT:
                        if (revers){
                            Lift_TS.setTargetPosition(bort - max);

                        }
                        else{
                            Lift_TS.setTargetPosition(bort);
                        }
                        Lift_TS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Lift_TS.setPower(Speed_Up);
                        while(Lift_TS.isBusy()){

                        }
                        LiftTSMode = Initilization.LiftTSMods.stop;
                        break;






                        }


                        }
                }
            };


        }//----------------------------------------------------------last----------------------------------





