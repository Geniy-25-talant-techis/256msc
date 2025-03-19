package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ModuleInit.Initilization;
@TeleOp(name = "TeleOp_Solo")
public class TeleOp_Solo extends OpMode {
    Initilization i = new Initilization();
    public static int up = 308;
    public static int max = 5450;
    public static int zero = 0;
    public static int Low = 0;


    public void init(){
//        i.init_hwm_tele(hardwareMap,this);
//        i.Clash.setDirection(Servo.Direction.REVERSE);
//        double ask = i.Clash.getPosition();
//        telemetry.addData("PositionClash",ask);
//        telemetry.addData("Position Povorot",i.Pvrt.getPosition());
//        telemetry.addData("Robot","Ready");
//        telemetry.update();
    }
    public void loop(){
//        boolean revers =false;
//        boolean Max = false;
//        if(i.Max_Static.getState()==false&&i.Max_Drive.getState()==false){
//            Max = true;
//        }
//        double max;
//        double axial   = -gamepad1.left_stick_y;
//        double lateral =  gamepad1.left_stick_x;
//        double yaw     =  gamepad1.right_stick_x;
//
//
//        double leftFrontPower  = axial + lateral + yaw;
//        double rightFrontPower = axial - lateral - yaw;
//        double leftBackPower   = axial - lateral + yaw;
//        double rightBackPower  = axial + lateral - yaw;
//
//        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower  /= max;
//            rightFrontPower /= max;
//            leftBackPower   /= max;
//            rightBackPower  /= max;
//        }
//        i.LeftFrotnDrive.setPower(leftFrontPower);
//        i.RightFrontDrive.setPower(rightFrontPower);
//        i.leftBackDrive.setPower(leftBackPower);
//        i.RightBackDrive.setPower(rightBackPower*0.7);
//
//        if(gamepad1.b){
//            i.Pvrt.setPosition(0.5);
//        }
//        if(gamepad1.x){
//            i.Pvrt.setPosition(0);
//        }
//
//        if(gamepad1.a){
//            i.Clash.setPosition(0);
//        }
//        if(gamepad1.y){
//            i.Clash.setPosition(0.5);
//        }
//        if(i.Low_Drive.getState()==true){
//            i.Lift_TS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            i.Lift_TS.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            revers = false;
//        }
//        if(i.Max_Static.getState()==true&&i.Max_Drive.getState()==true){
//            i.Lift_TS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            i.Lift_TS.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            revers = true;
//        }
//
//        if(gamepad1.dpad_left&&i.Low_Drive.getState()==false){
//            i.Lift_TS.setVelocity(4500);
//            i.Lift_TS.setTargetPosition(i.Lift_TS.getCurrentPosition()-100);
//        }
//        if(gamepad1.dpad_right&&Max == false ){
//            i.Lift_TS.setVelocity(4500);
//            i.Lift_TS.setTargetPosition((i.Lift_TS.getCurrentPosition()+100));
//        }
//
//        if(gamepad1.dpad_up){
//            i.Lift_Moment.setVelocity(700);
//            i.Lift_Moment.setTargetPosition(i.Lift_Moment.getCurrentPosition()+25);
//        }
//        if(gamepad1.dpad_down){
//            i.Lift_Moment.setVelocity(700);
//            i.Lift_Moment.setTargetPosition(i.Lift_Moment.getCurrentPosition()-25);
//        }
//
//        if(gamepad1.left_bumper&&gamepad1.a){
//            Low_Pvrt_Lift();
//        }
//        if(gamepad1.right_bumper&&gamepad1.b){
//            Max_Pvrt_Lift();
//        }
//
//        if(gamepad1.right_bumper){
//            Max_Pvrt_Lift();
//            if(Max == false){
//                Max_Lift();
//            }
//
//        }
//        if(gamepad1.left_bumper){
//            Low_Pvrt_Lift();
//            if(i.Low_Drive.getState()==false){
//                Low_Lift();
//            }
//
//        }
//
//
//
//
//        telemetry.addData("PositionPovorot",i.Pvrt.getPosition());
//        telemetry.addData("PositionClash",i.Clash.getPosition());
//        telemetry.addData("PositionLift_M",i.Lift_Moment.getTargetPosition());
//        telemetry.addData("PositionLift_TS",i.Lift_TS.getTargetPosition());
//        telemetry.addData("Max",Max);
//
//
//        telemetry.update();
//    }
//    public void Max_Pvrt_Lift(){
//        i.Lift_Moment.setVelocity(700);
//        i.Lift_Moment.setTargetPosition(up);
//    }
//    public void Low_Pvrt_Lift(){
//        i.Lift_Moment.setVelocity(700);
//        i.Lift_Moment.setTargetPosition(zero);
//    }
//    public void Max_Lift(){
//        i.Lift_TS.setVelocity(4500);
//        i.Lift_TS.setTargetPosition(max);
//    }
//    public void Low_Lift(){
//        i.Lift_TS.setVelocity(4500);
//        i.Lift_TS.setTargetPosition(zero);
    }
}
