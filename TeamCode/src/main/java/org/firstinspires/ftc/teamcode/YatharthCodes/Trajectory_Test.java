package org.firstinspires.ftc.teamcode.YatharthCodes;

import static org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_Wrist.Wrist;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_CLAW;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_Elbow;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_FrontSlide;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_ServoAction;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_Twist;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_Wrist;

import java.util.Arrays;


@Autonomous(name = "First trajectory test")

public class Trajectory_Test extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d InitialPose = new Pose2d(0, 0, 0);
        Servo W = null;
        Servo C = null;


        //this takes care of drive motors in MecanumDrive class.
        // Define other non drive motors before you move forward
        MecanumDrive drive = new MecanumDrive(hardwareMap, InitialPose);

        // Define all servos here or put all hardware initialization separately
        // Servo servo= hardwareMap.servo.get("servo");
        W = hardwareMap.get(Servo.class, "Wrist");
        C = hardwareMap.get(Servo.class, "Claw");

        waitForStart();

        Pose2d poseEstimate = drive.localizer.getPose();

// Velocity and Acceleration Constraints
 /*
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(150.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 100.0);
*/

//Method 1: Roadrunner Action Builder
        /*
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .lineToX(5)
                        .splineTo(new Vector2d(20,-20), Math.toRadians(90))
                        .build());

         */

//Method 2: Trajectory Action Builder
/*
        TrajectoryActionBuilder Path1 = drive.actionBuilder(InitialPose)
                .lineToX(28)
                .waitSeconds(0.5)
                .afterTime(0.5,new ServoAction(W,0.1))
                .afterTime(0.5, new ParallelAction(
                        new ServoAction(W, 0),
                        new ServoAction(C, 0.5)
                ));
        Action trajectorychosen;
        trajectorychosen= Path1.build();

*/

//Method 3: Action Runblocking
      Actions.runBlocking(
              new ParallelAction(
                        //trajectorychosen,
                        new ServoAction(W, 0.05),
                        new ServoAction(C, 0.3)

                )
        );

        telemetry.addData("heading", poseEstimate.heading);
        telemetry.addData("X,Y", poseEstimate.position);
        telemetry.update();

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

            return false;
        }
    }

    // Motor Action build

    //CRServo Action build
}