package org.firstinspires.ftc.teamcode.YatharthCodes;

import static java.lang.Math.PI;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name = "First trajectory test")

public class Trajectory_Test extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d InitialPose = new Pose2d(0, 0, 0);
        Servo W = null;
        Servo C = null;
        DcMotor F = null;


        //this takes care of drive motors in MecanumDrive class.
        // Define other non drive motors before you move forward
        MecanumDrive drive = new MecanumDrive(hardwareMap, InitialPose);

        // Define all servos here or put all hardware initialization separately
        // Servo servo= hardwareMap.servo.get("servo");
        W = hardwareMap.get(Servo.class, "Wrist");
        C = hardwareMap.get(Servo.class, "Claw");
        F = hardwareMap.get(DcMotor.class, "Front Slide");

        waitForStart();



// Velocity and Acceleration Constraints
 /*
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(150.0),
                new AngularVelConstraint(Math.PI / 2)
        ));*/
//        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10, 100);


//Method 1: Roadrunner Action Builder

        /*Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .lineToX(28)
                        .waitSeconds(0.5)
                        .setReversed(true)
                        .setTangent(Math.toRadians(0))
                        .lineToX(15)
                        .strafeTo(new Vector2d(15, -25))
                        .strafeTo(new Vector2d(55, -35))
                        .strafeTo(new Vector2d(55, -40))
                        .setReversed(true)
                        .setTangent(0)
                        .lineToX(5)
                        .strafeTo(new Vector2d(55,-42))
                        .strafeTo(new Vector2d(55,-50))
                        .setReversed(true)
                        .setTangent(0)
                        .lineToX(5)
                        .strafeToLinearHeading(new Vector2d(3,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(3,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(3,-45), 0)
                        .build());*/

       /* Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .lineToX(28)
                        .waitSeconds(0.5)
                        .setReversed(true)
                        .setTangent(Math.toRadians(0))
                        .lineToX(10)
                        .setTangent(-Math.PI/2)
                        .splineToConstantHeading(new Vector2d(55,-33), Math.PI/2)
                        .setTangent(Math.PI/2)
                        .splineToConstantHeading(new Vector2d(55,-40), Math.PI/2)
                        .setReversed(true)
                        .setTangent(0)
                        .lineToX(8)
                        .strafeTo(new Vector2d(55,-42))
                        .strafeTo(new Vector2d(55,-50))
                        .setReversed(true)
                        .setTangent(0)
                        .lineToX(8)
                        .strafeToLinearHeading(new Vector2d(5,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(5,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(5,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(5,-40), 0)
                        .build());*/

       /* Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .lineToX(28)
                        .waitSeconds(0.5)
                        .setReversed(true)
                        .setTangent(Math.toRadians(0))
                        .lineToX(10)
                        .setTangent(-Math.PI/2)
                        .splineToConstantHeading(new Vector2d(55,-33), Math.PI/2)
                      //  .splineToConstantHeading(new Vector2d(55,-33), Math.PI/2, new TranslationalVelConstraint(100), baseAccelConstraint)
                        .setTangent(Math.PI/2)
                        .splineToConstantHeading(new Vector2d(55,-40), Math.PI/2)
                        .setReversed(true)
                        .setTangent(0)
                        .lineToX(8)
                        .strafeTo(new Vector2d(55,-42))
                        .strafeTo(new Vector2d(55,-50))
                        .setReversed(true)
                        .setTangent(0)
                        .lineToX(8)
                        .strafeTo(new Vector2d(55,-50))
                        .strafeTo(new Vector2d(55,-57))
                        .setReversed(true)
                        .setTangent(0)
                        .lineToX(8)
                        .strafeToLinearHeading(new Vector2d(5,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(5,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(5,-30), 0)
                        .strafeToLinearHeading(new Vector2d(28,5), 0)
                        .strafeToLinearHeading(new Vector2d(5,-40), 0)
                        .build());*/


 /*   Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        .stopAndAdd(new ServoAction(W,0.5))
                       .afterTime(0.5, new SequentialAction(
                               new ServoAction(W,0.8),
                               new ServoAction(C, 0.5)
                       ))
                        .build());
*/




//Method 2: Trajectory Action Builder


       // TrajectoryActionBuilder Path1 = drive.actionBuilder(InitialPose);

                //.lineToX(28)
               // .waitSeconds(0.5)
                //.afterTime(0,new ServoAction(W,0.3))
               // .afterTime(0,new ServoAction(C,0.3));
                //.afterTime(0.5, new ParallelAction(
                  //      new ServoAction(W, 0.5),
                 //       new ServoAction(C, 0.8)
        //Action trajectorychosen;
       //trajectorychosen= Path1.build();



//Method 3: Action Runblocking - works

      Actions.runBlocking(
              new ParallelAction(
                        //trajectorychosen,
                        new ServoAction(W, 0.3),
                        new ServoAction(C, 0.8),
                        new MotorAction(F, -500, 0.5)

                )
        );

        Pose2d poseEstimate = drive.localizer.getPose();
        telemetry.addData("heading", poseEstimate.heading);
        telemetry.addData("X,Y", poseEstimate.position);
        telemetry.update();

    }


    // Servo action build for roadrunner to use
    public static class ServoAction implements Action {
        Servo servo;
        double position;
        ElapsedTime timer= null;

        public ServoAction(Servo srv, double pos) {
            this.servo= srv;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(timer == null) {
                timer = new ElapsedTime();
                servo.setPosition(position);
            }

            return timer.seconds() < 0.5;
        }
    }

    // Motor Action build for roadrunner to use
    public static class MotorAction implements Action {
        DcMotor motor;
        double position_tgt;
        double power;
        ElapsedTime timer = null;

        public MotorAction (DcMotor mot, double pos, double pow) {
            this.motor = mot;
            this.position_tgt = pos;
            this.power = pow;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime();
                motor.setPower(power);
                motor.setTargetPosition((int) (position_tgt));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
            return false;
    }
    }

    //CRServo Action build for roadrunner to use
    public static class CRServoAction implements Action {
        CRServo crServo;
        double power;
        int seconds;
        ElapsedTime timer= null;

        public CRServoAction(CRServo crsrv, int sec, double pow ) {
            this.crServo = crsrv;
            this.seconds = sec;
            this.power = pow;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer.seconds() < seconds) {
                timer = new ElapsedTime();
                crServo.setPower(power);
            }
            return false;
        }
    }
}