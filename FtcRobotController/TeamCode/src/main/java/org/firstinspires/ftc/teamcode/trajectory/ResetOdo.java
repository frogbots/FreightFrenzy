package org.firstinspires.ftc.teamcode.trajectory;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.TrackingWheelIntegrator;

import ftc.teamcode.FreightFrenzy.FreightFrenzyMainAutoBlue;



public class ResetOdo implements MovementPerformer {

        TrackingWheelIntegrator trackingWheelIntegrator = new TrackingWheelIntegrator();
        double inch;
        double cmToInch;
        double DistToOrigin;

        @Override
        public void run() {
        Globals.leftTW.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Globals.backTW.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Globals.rightTW.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Globals.leftTW.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Globals.backTW.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Globals.rightTW.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        // FreightFrenzyMainAutoBlue.clearEnc();
         cmToInch = Globals.FrontSonar.getDistanceSync();
         inch = cmToInch/2.54;
         DistToOrigin = inch + 14 - 141.5;
         trackingWheelIntegrator.setFirstTrackingVal(-DistToOrigin,-1);
         trackingWheelIntegrator.setHeading(-270);
        }
//52 =74
//56 =70

}

//  Globals.leftTW = hardwareMap.get(DcMotorEx.class, "Intake"); // this is also left Tracking wheel
//            Globals.rightTW = hardwareMap.get(DcMotorEx.class, "brokport");
//            Globals.backTW = hardwareMap.get(DcMotorEx.class, "trakingport");


//ctrl.setMotorMode(3, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            ctrl.setMotorMode(1, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            ctrl.setMotorMode(2, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//            ctrl.setMotorMode(3, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            ctrl.setMotorMode(1, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            ctrl.setMotorMode(2, DcMotor.RunMode.RUN_WITHOUT_ENCODER);