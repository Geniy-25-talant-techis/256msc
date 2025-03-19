package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ModuleInit.Initilization;
@TeleOp(name = "TeleOpLinearDuo")
public class TeleOp_Liener extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Initilization i = new Initilization();

i.init_hwm_tele(hardwareMap,this);
        waitForStart();


i.Lift_auto_Ts.start();
    while(!isStopRequested()){

        if(i.Max_Static.getState()==false&&i.Max_Drive.getState()==false){
            i.Max = true;
            
        }else{
            i.Max = false;
        }
        double max;
        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;


        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        i.LeftFrotnDrive.setPower(leftFrontPower);
        i.RightFrontDrive.setPower(rightFrontPower);
        i.leftBackDrive.setPower(leftBackPower);
        i.RightBackDrive.setPower(rightBackPower*0.7);
if(gamepad2.left_bumper){
    i.Lift_Control_TS = i.Lift_Control_TS.Low;
} else if (gamepad2.right_bumper) {
    i.Lift_Control_TS = i.Lift_Control_TS.maxx;


}

telemetry.addData("------------------Servo----------------",null);
        telemetry.addData("PositionPovorot:",i.Pvrt.getPosition());
        telemetry.addData("PositionClash:",i.Clash.getPosition());
        telemetry.addData("------------------Lift----------------",null);
        telemetry.addData("PositionLift_M",i.Lift_Moment.getTargetPosition());
        telemetry.addData("PositionLift_TS",i.Lift_TS.getTargetPosition());
        telemetry.addData("------------------TouchSensor----------------",null);
        telemetry.addData("Max",i.Max);
     telemetry.update();

        }
    }
}
