package org.firstinspires.ftc.teamcode.Yatharth.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


// Front Slide - Operation Sub-system
// Defines Front slide motor's and its behavior
// Builds all potential Front Slide actions (add more for buckets, as needed)
//  1. Intake
//  2. Transfer
//  3. AtRest position


public class SS_FrontSlide {
    public static DcMotor FrontSlideLeft;
    public static DcMotor FrontSlideRight;

    //Define all Delivery Arm Encoder positions and power
    public static int Front_Slide_Resting_Enc = 70;
    public static int Front_Slide_Intake_Enc = 400;
    public static int Front_Slide_Transfer_Enc = 160;
    public static double Front_Slide_Extend_Power = 0.3;
    public static double Front_Slide_Retract_Power = -0.3;

    //HardwareMap and RunModes definition
    public SS_FrontSlide(HardwareMap hardwareMap) {
        FrontSlideLeft = hardwareMap.get(DcMotor.class, "Front Slide Left");
        FrontSlideRight = hardwareMap.get(DcMotor.class, "Front Slide Right");

        FrontSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Front Slide Left zero power behavior
        FrontSlideLeft.setDirection(DcMotor.Direction.FORWARD);          //Front Slide left motor set forward
        FrontSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Front Slide Left motor reset
        FrontSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Front Slide Left run using encoders

        FrontSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Front Slide Right zero power behavior
        FrontSlideRight.setDirection(DcMotor.Direction.FORWARD);          //Front Slide  right motor set forward
        FrontSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Front Slide Right motor reset
        FrontSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Front Slide Right run using encoders
    }


    //Action 1. Implement Intake Action for Front slide
    public static class IntakeFrontSlide implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                FrontSlideLeft.setPower(Front_Slide_Extend_Power);
                FrontSlideRight.setPower(Front_Slide_Extend_Power);
                initialized = true;
            }

            double posl = FrontSlideLeft.getCurrentPosition();
            packet.put("FrontSlideLeftPos", posl);
            double posr = FrontSlideRight.getCurrentPosition();
            packet.put("FrontSlideRightPos", posr);


            if (posl < Front_Slide_Intake_Enc && posr < Front_Slide_Intake_Enc) {
                return true; // Continue running
            } else {
                FrontSlideLeft.setPower(0);
                FrontSlideRight.setPower(0);
                packet.put("FrontSlideLeftPos", posl);
                packet.put("FrontSlideRightPos", posr);
                return false; // Stop action
            }
        }
    }

    //Action 2. Implement Transfer Action for Front slide
    public static class TransferFrontSlide implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                FrontSlideLeft.setPower(Front_Slide_Retract_Power);
                FrontSlideRight.setPower(Front_Slide_Retract_Power);
                initialized = true;
            }

            double posl = FrontSlideLeft.getCurrentPosition();
            packet.put("FrontSlideLeftPos", posl);
            double posr = FrontSlideRight.getCurrentPosition();
            packet.put("FrontSlideRightPos", posr);


            if (posl > Front_Slide_Transfer_Enc && posr > Front_Slide_Transfer_Enc) {
                return true; // Continue running
            } else {
                FrontSlideLeft.setPower(0);
                FrontSlideRight.setPower(0);
                packet.put("FrontSlideLeftPos", posl);
                packet.put("FrontSlideRightPos", posr);
                return false; // Stop action
            }

            }
        }

    //Action 3. Implement Rest Action for Front slide
    public static class RestFrontSlide implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                FrontSlideLeft.setPower(Front_Slide_Retract_Power);
                FrontSlideRight.setPower(Front_Slide_Retract_Power);
                initialized = true;
            }

            double posl = FrontSlideLeft.getCurrentPosition();
            packet.put("FrontSlideLeftPos", posl);
            double posr = FrontSlideRight.getCurrentPosition();
            packet.put("FrontSlideRightPos", posr);


            if (posl > Front_Slide_Resting_Enc && posr > Front_Slide_Resting_Enc) {
                return true; // Continue running
            } else {
                FrontSlideLeft.setPower(0);
                FrontSlideRight.setPower(0);
                packet.put("FrontSlideLeftPos", posl);
                packet.put("FrontSlideRightPos", posr);
                return false; // Stop action
            }

        }
    }
}

