package org.firstinspires.ftc.teamcode.YatharthCodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name = "Axon test")
public class Axon_Test extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        //this takes care of drive motors in MecanumDrive class.
        // Define other non drive motors before you move forward
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, beginPose);

        // Define all servos here or put all hardware initialization separately
        Servo axon= hardwareMap.servo.get("axon");

        waitForStart();

        Actions.runBlocking(
                drivetrain.actionBuilder(new Pose2d(5, -70, Math.toRadians(90)))
                        .stopAndAdd(new ServoAction(axon, 1 ))
                        .build());
    }


    // Servo action build for roadrunner to use
    public static class ServoAction implements Action {
        Servo servo;
        double position;
        ElapsedTime timer;

        public ServoAction(Servo s, double p) {
            this.servo= s;
            this.position = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null) {
                timer = new ElapsedTime();
                servo.setPosition(position);
            }

            return timer.seconds() < 3;

            //return false;
            //return motor.getPosition() = targetPos;
        }
    }
}