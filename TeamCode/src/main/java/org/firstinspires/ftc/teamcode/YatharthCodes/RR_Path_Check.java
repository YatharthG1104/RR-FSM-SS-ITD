package org.firstinspires.ftc.teamcode.YatharthCodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
//@Autonomous(name = "RR Path Check")
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

        Servo FrontClaw = hardwareMap.get(Servo.class, "Front Claw");
        FrontClaw.setDirection(Servo.Direction.FORWARD);


        // Declare Trajectory as such
       /* Action TrajectoryAction1 = drive.actionBuilder(new Pose2d(0,0,0))
                .lineToX(10)
                .build();*/

        // Bit more complicated way of doing it if you want access to the data of the trajectory
      /* MecanumDrive.FollowTrajectoryAction TrajectoryAction2 = (MecanumDrive.FollowTrajectoryAction) drive.actionBuilder(new Pose2d(10,0,0))
                .strafeTo(new Vector2d(10,5))
                .build();*/

        /*Action TrajectoryAction3 = drive.actionBuilder(new Pose2d(10,5,0))
                .lineToX(20)
                .build();*/

     /*   while(!isStopRequested() && !opModeIsActive()) {

        }*/

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(0,0,0)) // Another way of running a trajectory (not recommended because trajectories take time to build and will slow down your code, always try to build them beforehand)
                                .lineToX(10)
                                .build(),
                        new ParallelAction( // several actions being run in parallel
                                new MotorAction2(FrontSlide,500,0.3),
                                new ServoAction(FrontClaw, 0.5)
                                //TrajectoryAction2, // Run second trajectory
                                //(telemetryPacket) -> { // Run some action
                                  //  new MotorAction2(FrontSlide,500,0.3);
                                   // new ServoAction(FrontClaw, 0.5);
                                  /*  FrontSlide.setPower(0.3);
                                    FrontSlide.setTargetPosition(200);*/
                                    //return false;
                               // }
                        ),
                       // TrajectoryAction3,  //Run third trajectory
                        drive.actionBuilder(new Pose2d(10,0,0)) // Another way of running a trajectory (not recommended because trajectories take time to build and will slow down your code, always try to build them beforehand)
                                .strafeTo(new Vector2d(25, 15))
                                .build()

                )
        );
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
            if (motor.getCurrentPosition() < position_tgt) {
                timer = new ElapsedTime();
                motor.setPower(power);
                motor.setTargetPosition((int) (position_tgt));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            return timer.seconds() < 1;
        }
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

            return timer.seconds() < 1;
        }
    }

    // Motor Action-2 build for roadrunner to use
    public static class MotorAction2 implements Action {
        DcMotor motor;
        double position_tgt;
        double power;

        public MotorAction2(DcMotor mot, double pos, double pow) {
            this.motor = mot;
            this.position_tgt = pos;
            this.power = pow;
        }

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (motor.getCurrentPosition() != position_tgt) {
                motor.setPower(power);
                motor.setTargetPosition((int) (position_tgt));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return true;
            } else {
                motor.setPower(0);
                return false;
            }
        }
    }

}
