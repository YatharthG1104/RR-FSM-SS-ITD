package org.firstinspires.ftc.teamcode.YatharthCodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "RR Path Check")
public class RR_Path_Check extends LinearOpMode {

    boolean finished = false;

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        DcMotor FrontSlide = hardwareMap.get(DcMotor.class,  "Front Slide");
        FrontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//Front Slide zero power behavior
        FrontSlide.setDirection(DcMotor.Direction.REVERSE);          //Front Slide motor set forward
        FrontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  //Front Slide motor reset
        FrontSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Delcare Trajectory as such
        Action TrajectoryAction1 = drive.actionBuilder(new Pose2d(0,0,0))
                .lineToX(10)
                .build();

        // Bit more complicated way of doing it if you want access to the data of the trajectory
       MecanumDrive.FollowTrajectoryAction TrajectoryAction2 = (MecanumDrive.FollowTrajectoryAction) drive.actionBuilder(new Pose2d(10,0,0))
                .strafeTo(new Vector2d(10,5))
                .build();

        Action TrajectoryAction3 = drive.actionBuilder(new Pose2d(10,5,0))
                .lineToX(20)
                .build();

        while(!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        TrajectoryAction1, // Example of a drive action
                        new ParallelAction( // several actions being run in parallel
                                TrajectoryAction2, // Run second trajectory
                                (telemetryPacket) -> { // Run some action
                                    FrontSlide.setPower(0.3);
                                    FrontSlide.setTargetPosition(200);
                                    return false;
                                }
                        ),
                        TrajectoryAction3,  //Run third trajectory
                        drive.actionBuilder(new Pose2d(20,5,0)) // Another way of running a trajectory (not recommended because trajectories take time to build and will slow down your code, always try to build them beforehand)
                                .strafeTo(new Vector2d(25, 15))
                                .build()
                )
        );
    }
}
