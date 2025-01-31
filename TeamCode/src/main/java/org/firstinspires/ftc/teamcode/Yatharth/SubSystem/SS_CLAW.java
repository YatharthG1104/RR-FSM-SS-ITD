package org.firstinspires.ftc.teamcode.Yatharth.SubSystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Claw - Operation Sub-system
// Defines Claw Servo and its behavior
//  1. Claw Open
//  2. Claw Close
//  3. Claw Rest

public class SS_CLAW {
    public static Servo Claw;

    //Define all Claw positions
    public static double Claw_Open_Pos = 0.7;
    public static double Claw_Close_Pos = 0.3;
    public static double Claw_Initial_Pos = 0.0;

    // Define hardware properties
    public SS_CLAW(HardwareMap hardwareMap) {
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.scaleRange(0,1);
        Claw.setDirection(Servo.Direction.FORWARD);
    }

    //Action 1.The Claw open action
    public static class ClawOpen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
                Claw.setPosition(Claw_Open_Pos);
                double pos = Claw.getPosition();
                packet.put("Claw pos", pos);
                return false;
            }
    }

    //Action 2.The Claw Close action
    public static class ClawClose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Claw.setPosition(Claw_Close_Pos);
            double pos = Claw.getPosition();
            packet.put("Claw pos", pos);
            return false;
        }
    }

    //Action 3.The Claw Rest action
    public static class ClawRest implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Claw.setPosition(Claw_Initial_Pos);
            double pos = Claw.getPosition();
            packet.put("Claw pos", pos);
            return false;
        }
    }
    }



//private boolean initialized = false;
/*  if (!initialized) {
                Claw.setPosition(Claw_Open);
                initialized = true;
            }

            double pos = Claw.getPosition();
            packet.put("Claw pos", pos);


            if (pos > Claw_Close) {
                Claw.setPosition(Claw_Close);
                return true; // Continue running
            } else {
                //Claw.setPosition(0.0);
                packet.put("Claw pos", pos);
                return false; // Stop action
            }*/