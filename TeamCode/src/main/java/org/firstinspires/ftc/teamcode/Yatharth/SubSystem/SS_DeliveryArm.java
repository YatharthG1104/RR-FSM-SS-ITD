package org.firstinspires.ftc.teamcode.Yatharth.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Delivery Arm - Operation Sub-system
// Defines Delivery Arm motor and its behavior
// Builds all potential Delivery Arm actions (add more for buckets, as needed)
//  1. Specimen Hangready
//  2. Specimen Hangdone
//  3. AtRest position

public class SS_DeliveryArm {

    public static DcMotor deliveryArmLeft;
    public static DcMotor deliveryArmRight;

    //Define all Delivery Arm Encoder positions and power
    public static int Delivery_Arm_Resting_Enc = 100;
    public static int Delivery_Arm_HangReady_Enc = 3000;
    public static int Delivery_Arm_HangDone_Enc = 500;
    public static double Delivery_Arm_Extend_Power = 0.7;
    public static double Delivery_Arm_Retract_Power = -0.7;

    //HardwareMap and RunModes definition
    public SS_DeliveryArm(HardwareMap hardwareMap) {
        deliveryArmLeft = hardwareMap.get(DcMotor.class, "Delivery ArmL");
        deliveryArmRight = hardwareMap.get(DcMotor.class, "Delivery ArmR");

        deliveryArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Delivery arm zero power behavior
        deliveryArmLeft.setDirection(DcMotor.Direction.REVERSE);          //Delivery arm left motor set reversed
        deliveryArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Delivery arm motor reset
        deliveryArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Delivery arm run using encoders

        deliveryArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //Delivery arm zero power behavior
        deliveryArmRight.setDirection(DcMotor.Direction.FORWARD);          //Delivery arm right motor set forward
        deliveryArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Delivery arm motor reset
        deliveryArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       //Delivery arm run using encoders
    }


    //Action 1. Implement Specimen HangReady Action for Delivery Arm
       public static class HangReady implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    deliveryArmLeft.setPower(Delivery_Arm_Extend_Power);
                    deliveryArmRight.setPower(Delivery_Arm_Extend_Power);
                    initialized = true;
                }

                double posl = deliveryArmLeft.getCurrentPosition();
                packet.put("DeliveryArmLeftPos", posl);
                double posr = deliveryArmRight.getCurrentPosition();
                packet.put("DeliveryArmRightPos", posr);


                if (posl < Delivery_Arm_HangReady_Enc && posr < Delivery_Arm_HangReady_Enc) {
                    return true; // Continue running
                } else {
                    deliveryArmLeft.setPower(0);
                    deliveryArmRight.setPower(0);
                    packet.put("DeliveryArmLeftPos", posl);
                    packet.put("DeliveryArmRightPos", posr);
                    return false; // Stop action
                }
            }
        }



    //Action 2. Implement Specimen HangDone Action for Delivery Arm
    public static class HangDone implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                deliveryArmLeft.setPower(Delivery_Arm_Retract_Power);
                deliveryArmRight.setPower(Delivery_Arm_Retract_Power);
                initialized = true;
            }

            double posl = deliveryArmLeft.getCurrentPosition();
            packet.put("DeliveryArmLeftPos", posl);
            double posr = deliveryArmRight.getCurrentPosition();
            packet.put("DeliveryArmRightPos", posr);

            if (posl > Delivery_Arm_HangDone_Enc && posr >Delivery_Arm_HangDone_Enc) {
                return true; // Continue running
            } else {
                deliveryArmRight.setPower(0);
                deliveryArmLeft.setPower(0);
                packet.put("DeliveryArmLeftPos", posl);
                packet.put("DeliveryArmRightPos", posr);
                return false; // Stop action
            }
        }
    }

    //Action 3. Implement AtRest Action for Delivery Arm
     public static class AtRest implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                deliveryArmLeft.setPower(Delivery_Arm_Retract_Power);
                deliveryArmRight.setPower(Delivery_Arm_Retract_Power);
                initialized = true;
            }

            double posl = deliveryArmLeft.getCurrentPosition();
            packet.put("DeliveryArmLeftPos", posl);
            double posr = deliveryArmRight.getCurrentPosition();
            packet.put("DeliveryArmRightPos", posr);

            if (posl > Delivery_Arm_Resting_Enc && posr > Delivery_Arm_Resting_Enc) {
                return true; // Continue running
            } else {
                deliveryArmRight.setPower(0);
                deliveryArmLeft.setPower(0);
                packet.put("DeliveryArmLeftPos", posl);
                packet.put("DeliveryArmRightPos", posr);
                return false; // Stop action
            }
        }
    }
}
