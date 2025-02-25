package org.firstinspires.ftc.teamcode.Krunaal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp FSM", group="Linear OpMode")
public class TeleOpFSMKrunnal extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private DcMotor MLeftBackSlide, MRightBackSlide, Linkage;
    private Servo backClaw, wrist, Front_Claw, leftslide, rightslide, BackLeftSlide, BackRightSlide;
    private CRServo Front_Rotate;
    
    private enum RobotState {
        IDLE,
        DRIVE,
        OPERATE_CLAW,
        OPERATE_WRIST,
        OPERATE_SLIDES
    }
    
    private RobotState currentState = RobotState.IDLE;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        initializeHardware();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            updateState();
            executeStateActions();
            telemetry.update();
        }
    }

    private void initializeHardware() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightBack");
        
        MLeftBackSlide = hardwareMap.get(DcMotor.class, "Delivery ArmL");
        MRightBackSlide = hardwareMap.get(DcMotor.class, "Delivery ArmR");
        Linkage = hardwareMap.get(DcMotor.class, "Front Slide");
        
        backClaw = hardwareMap.get(Servo.class, "Claw");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        Front_Claw = hardwareMap.get(Servo.class, "Grab Right");
        Front_Rotate = hardwareMap.get(CRServo.class, "Grab Left");
        leftslide = hardwareMap.get(Servo.class, "Twist Left");
        rightslide = hardwareMap.get(Servo.class, "Twist Right");
        BackLeftSlide = hardwareMap.get(Servo.class, "Elbow Left");
        BackRightSlide = hardwareMap.get(Servo.class, "Elbow Right");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void updateState() {
        if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0) {
            currentState = RobotState.DRIVE;
        } else if (gamepad2.x || gamepad2.b) {
            currentState = RobotState.OPERATE_CLAW;
        } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
            currentState = RobotState.OPERATE_WRIST;
        } else if (gamepad2.left_bumper || gamepad2.right_bumper) {
            currentState = RobotState.OPERATE_SLIDES;
        } else {
            currentState = RobotState.IDLE;
        }
    }

    private void executeStateActions() {
        switch (currentState) {
            case DRIVE:
                drive();
                break;
            case OPERATE_CLAW:
                operateClaw();
                break;
            case OPERATE_WRIST:
                operateWrist();
                break;
            case OPERATE_SLIDES:
                operateSlides();
                break;
            case IDLE:
                stopAllMotors();
                break;
        }
    }

    private void drive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        leftFront.setPower((y + x + rx) / denominator);
        leftRear.setPower((y - x + rx) / denominator);
        rightFront.setPower((y - x - rx) / denominator);
        rightRear.setPower((y + x - rx) / denominator);
    }

    private void operateClaw() {
        if (gamepad2.x) {
            backClaw.setPosition(0.0);
        } else if (gamepad2.b) {
            backClaw.setPosition(1.0);
        }
    }

    private void operateWrist() {
        if (gamepad2.dpad_left) {
            wrist.setPosition(1);
        } else if (gamepad2.dpad_right) {
            wrist.setPosition(-1);
        }
    }

    private void operateSlides() {
        if (gamepad2.left_bumper) {
            rightslide.setPosition(1);
            leftslide.setPosition(-1);
        } else if (gamepad2.right_bumper) {
            rightslide.setPosition(-0.8);
            leftslide.setPosition(0.8);
        }
    }

    private void stopAllMotors() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}
