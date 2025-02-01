package org.firstinspires.ftc.teamcode.YatharthCodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_CLAW;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_DeliveryArm;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_Elbow;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_Wrist;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_Grabber;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_Twist;
import org.firstinspires.ftc.teamcode.Yatharth.SubSystem.SS_FrontSlide;

// Auto for Observation zone to hang specimens
@Autonomous(name="Observation zone auton" )
public class Auto_OZ_Zone extends LinearOpMode {

    // Define the states of the FSM
    private enum State {
        S0_RESTING,
        S1_GET_READY_TO_HANG,
        S2_HANG_ONE_COMPLETE,
        S3_DRAG_3BLOCKS,
        S4_READY_FOR_INTAKE,
        S5_READY_TO_HANG,
        S6_PARK
    }

    public ElapsedTime mRuntime = new ElapsedTime();   // Time into the Autonomous round.

    public static State CurrentState;    // Current State Machine State.


    @Override
    public void runOpMode() throws InterruptedException {

        //Define all Poses here
        Pose2d InitialPose = new Pose2d(0,0,0);     //Beginning pose
        Pose2d Pose1 = new Pose2d(30,0,0);          //Ready for 1st hang
        Pose2d Pose2 = new Pose2d(25,0,0);// After 1st Hang
        Pose2d Pose3  = new Pose2d(44, -70,0 );// Ready for Intake
        Pose2d Pose4  = new Pose2d(30, 0,0); // Ready for next hang


        //Importing the hardware maps for all drive motors and setting the robot position
        MecanumDrive drive = new MecanumDrive(hardwareMap, InitialPose);

        // Color Sensor Hardware Map
        ColorSensor sensorColor;
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // Add all other non drive motors to respective subsystems
        //Add all servos to subsystems as well

        // set initial state and start game clock
        mRuntime.reset();                               // Zero game clock
        CurrentState(State.S0_RESTING);   // Initial state

        //Build all Trajectories
        TrajectoryActionBuilder Path1 = drive.actionBuilder(InitialPose)
                .lineToX(30)
                .waitSeconds(1);

       TrajectoryActionBuilder Path2 = drive.actionBuilder(Pose1)
               .lineToXLinearHeading(25, Math.toRadians(0));

        TrajectoryActionBuilder Path3 = drive.actionBuilder(Pose2)
                //need to update all coordinates
                .splineTo(new Vector2d(43,-10),Math.toRadians(90))
                .strafeTo(new Vector2d(46, -10))
                .strafeTo(new Vector2d(46,-65))
                .strafeTo(new Vector2d(46, -10))
                .strafeTo(new Vector2d(57, -10))
                .strafeTo(new Vector2d(57, -65))
                .strafeTo(new Vector2d(57, -10))
                .strafeTo(new Vector2d(68, -10))
                .strafeTo(new Vector2d(68, -65))
                .strafeTo(new Vector2d(44, -70));

        TrajectoryActionBuilder Path4 = drive.actionBuilder(Pose3)
         .strafeTo(new Vector2d(30, 0));


        TrajectoryActionBuilder Path5 = drive.actionBuilder(Pose4)
                .strafeTo(new Vector2d(44, -70));

        TrajectoryActionBuilder Path6 = drive.actionBuilder(Pose4)
                .strafeTo(new Vector2d(5, -65));


        Action trajectorychosen;       // Define Action to choose the trajectory in the FSM code

        waitForStart();

        // Actions based on states
        switch (CurrentState) {
            //Resting state at the start of the match
            case S0_RESTING:
                Actions.runBlocking(
                        new ParallelAction(
                                new SS_Elbow.ElbowLeftHang(),
                                new SS_Elbow.ElbowRightHang(),
                                new SS_Wrist.WristHang()
                        )
                );
                System.out.println("S0_RESTING");
                CurrentState(State.S1_GET_READY_TO_HANG);
                break;

             //Go forward and get ready to hang pre-loaded specimen
            case S1_GET_READY_TO_HANG:
                trajectorychosen= Path1.build();
                Actions.runBlocking(
                        new ParallelAction(
                                trajectorychosen,
                                new SS_DeliveryArm.HangReady()
                        )
                );
                System.out.println("S1_GET_READY_TO_HANG");
                CurrentState(State.S2_HANG_ONE_COMPLETE);
                break;

            //Complete 1st hang and move little backwards to start 3 block drag trajectory
            case S2_HANG_ONE_COMPLETE:
                trajectorychosen= Path2.build();
                Actions.runBlocking(
                        new SequentialAction(
                             new SS_DeliveryArm.HangDone(),
                                new SS_CLAW.ClawOpen(),
                                new ParallelAction(
                                    new SS_Elbow.ElbowLeftIntake(),
                                    new SS_Elbow.ElbowRightIntake(),
                                    new SS_Wrist.WristIntake(),
                                    new SS_DeliveryArm.HangIntake(),
                                    trajectorychosen
                                )
                            )
                );
                System.out.println("S2_HANG_ONE_COMPLETE");
                CurrentState(State.S3_DRAG_3BLOCKS);
                break;

            //Drag 3 blocks to Human Player
            case S3_DRAG_3BLOCKS:
                trajectorychosen= Path3.build();
                Actions.runBlocking(
                        trajectorychosen
                );
                System.out.println("S3_DRAG_3BLOCKS");
                CurrentState(State.S4_READY_FOR_INTAKE);
                break;

            // Get Specimen Intake from the human player
            case S4_READY_FOR_INTAKE:
                trajectorychosen= Path4.build();
                Actions.runBlocking(
                        new SequentialAction(
                                new SS_CLAW.ClawClose(),
                                new SS_DeliveryArm.IntakeDone(),
                                new ParallelAction(
                                        new SS_DeliveryArm.HangReady(),
                                        new SS_Elbow.ElbowLeftHang(),
                                        new SS_Elbow.ElbowRightHang(),
                                        new SS_Wrist.WristHang(),
                                        trajectorychosen
                                )
                        )
                );
                System.out.println("S4_READY_FOR_INTAKE");
                CurrentState(State.S5_READY_TO_HANG);
                break;

            // Hang the Next Specimen
            case S5_READY_TO_HANG:
                trajectorychosen= Path5.build();
                Actions.runBlocking(
                        new SequentialAction(
                                new SS_DeliveryArm.HangDone(),
                                new SS_CLAW.ClawOpen(),
                                new ParallelAction(
                                        new SS_DeliveryArm.HangIntake(),
                                        new SS_Elbow.ElbowRightIntake(),
                                        new SS_Elbow.ElbowLeftIntake(),
                                        new SS_Wrist.WristIntake(),
                                        trajectorychosen
                                )
                        )
                );
                System.out.println("S5_READY_TO_HANG");
                CurrentState(State.S6_PARK);
                break;

            // Park at the end of Auto
            case S6_PARK:
                trajectorychosen= Path6.build();
                Actions.runBlocking(
                        new ParallelAction(
                                new SS_DeliveryArm.AtRest(),
                                new SS_Elbow.ElbowLeftRest(),
                                new SS_Elbow.ElbowRightRest(),
                                new SS_Wrist.WristRest(),
                                new SS_CLAW.ClawRest(),
                                trajectorychosen
                                )
                );
                System.out.println("S6_PARKING");
                break;

            default:
                System.out.println("Unknown state!");
        }


    }

    // Update state function
    private void CurrentState(State state) {
        CurrentState = state;
    }

 }