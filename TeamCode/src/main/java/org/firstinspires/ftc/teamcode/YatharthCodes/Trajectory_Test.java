package org.firstinspires.ftc.teamcode.YatharthCodes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_CRServoAction;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_Elbow;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_MotorAction;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_ServoAction;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_Wrist;

import java.util.Arrays;


@Autonomous(name = "First trajectory test")

public class Trajectory_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        //this takes care of drive motors in MecanumDrive class.
        // Define other non drive motors before you move forward
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        //DcMotor motorl = null;
        //motorl = hardwareMap.get(DcMotor.class, "Left Motor");

        Servo servol = null;
        Servo servor = null;
        servol = hardwareMap.get(Servo.class, "Twist Left");
        servor = hardwareMap.get(Servo.class, "Twist Right");


        //CRServo servor = null;
        //servor = hardwareMap.get(CRServo.class, "Right Servo");


        waitForStart();


        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                        //.lineToX(5)
                        // .stopAndAdd(new SS_MotorAction(motorl,2000, 0.7))
                        .stopAndAdd(new SS_ServoAction(servol, 0.5, -1))
                        .stopAndAdd(new SS_ServoAction(servor, 0.5, 1))
                        //  .stopAndAdd(new SS_CRServoAction(servor, 0.8, 3))
                        .build());
    }
}

/*
        TrajectoryActionBuilder Path1 = drive.actionBuilder(beginPose)
                .lineToX(10)
                .waitSeconds(1);
        Action trajectorychosen;

        trajectorychosen= Path1.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectorychosen,
                        new SS_Elbow.ElbowLeftHang(),
                       new SS_Elbow.ElbowRightHang(),
                        new SS_Wrist.WristHang()
                )
        );


        Pose2d poseEstimate = drive.localizer.getPose();
        telemetry.addData("heading", poseEstimate.heading);
        telemetry.addData("X,Y", poseEstimate.position);
        telemetry.update();

    }

}

/*
        Actions.runBlocking(
                //MecanumDrive.setPower(1);
                drive.actionBuilder(new Pose2d(0,0, Math.toRadians(0)))

                       // .lineToX(50, new TranslationalVelConstraint(50))
                        .lineToX(15)
                        .setTangent(Math.toRadians(0))
                        .strafeTo(new Vector2d(15,-20))
                       /* .lineToX(30)
                        .setTangent(Math.toRadians(0))
                        .strafeTo(new Vector2d(30, -25))
                        // .strafeTo(new Vector2d(30,-90), new TranslationalVelConstraint(20.0))
                        .turn(Math.toRadians(45))
                        .strafeTo(new Vector2d(55, -38))
                        .turn(Math.toRadians(45))
                        .strafeTo(new Vector2d(55, -41))
                        .strafeTo(new Vector2d(5, -41))
                        .strafeTo(new Vector2d(55, -41))
                        .strafeTo(new Vector2d(55, -52))
                        .strafeTo(new Vector2d(5, -52))
                        .strafeTo(new Vector2d(55, -52))
                        .strafeTo(new Vector2d(55, -63))
                        .strafeTo(new Vector2d(5, -63))
                        .strafeTo(new Vector2d(5, -39))
                        .build());*/



        /*Pose2d poseEstimate = drivetrain.localizer.getPose();
        telemetry.addData("heading", poseEstimate.heading);
        telemetry.addData("X,Y", poseEstimate.position);
        telemetry.update();

*/

           /* VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(150.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 100.0);*/