package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ModuleInit.Initilization;
@Autonomous(name = "Right_Pos")
public class Auto_A45 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU imu;

        Initilization i = new Initilization();
        i.init_hwm_tele(hardwareMap,i.l);
        i.init_hwm_auto(hardwareMap);


        ElapsedTime timer = new ElapsedTime();
        telemetry.addData("Auto","Ready");
        waitForStart();
        timer.reset();

    }
}
