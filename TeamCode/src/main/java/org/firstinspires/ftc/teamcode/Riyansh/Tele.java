/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Riyansh;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Tele op test", group="Linear OpMode")

public class Tele extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;


    Servo backClaw; // back claw
    CRServo LeftGeko; // front left gecko
    CRServo RightGeko; // front right gecko
    Servo wrist; // wrist pivot point
    Servo leftmisumi    ; // left misumi slide
    Servo rightmisumi; // right misumi slide

    Servo BackLeftMisumi;//back left misumi

    Servo BackRightMisumi;//back right misumi
    double cp;
    double p;

//    double cp;
//    double dd = gamepad2.right_stick_y;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightBack");



        DcMotor MLeftBackMisumi = hardwareMap.get(DcMotor.class, "Delivery ArmL");//left back misumi motor
        DcMotor MRightckMisumi = hardwareMap.get(DcMotor.class,"Delivery ArmR");//right back misumi motor
        DcMotor Linkage = hardwareMap.get(DcMotor.class,"Front Slide");// linkage
        // Init servos
        backClaw = hardwareMap.get(Servo.class, "Claw");

        wrist = hardwareMap.get(Servo.class, "Wrist");

        RightGeko = hardwareMap.get(CRServo.class, "Grab Right");
        LeftGeko = hardwareMap.get(CRServo.class, "Grab Left");

        leftmisumi = hardwareMap.get(Servo.class, "Twist Left");//front slide
        rightmisumi = hardwareMap.get(Servo.class, "Twist Right");

        BackLeftMisumi = hardwareMap.get(Servo.class, "Elbow Left");
        BackRightMisumi = hardwareMap.get(Servo.class, "Elbow Right");


        MLeftBackMisumi.setDirection(DcMotorSimple.Direction.REVERSE);
        MLeftBackMisumi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MLeftBackMisumi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MRightckMisumi.setDirection(DcMotorSimple.Direction.REVERSE);
        MRightckMisumi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MRightckMisumi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


      //  RightLinkage.setDirection(DcMotorSimple.Direction.REVERSE);
        Linkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Linkage.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MLeftBackMisumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MLeftBackMisumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Linkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Defining variable for the motor's power
            double lfPower;
            double rfPower;
            double rbPower;
            double lbPower;


            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            lfPower = (y + x + rx) / denominator;
            lbPower = (y - x + rx) / denominator;
            rfPower = (y - x - rx) / denominator;
            rbPower = (y + x - rx) / denominator;

            // Setting all the drive motors to their power
            leftFront.setPower(lfPower);
            leftRear.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightRear.setPower(rbPower);


            if(gamepad2.x){
                backClaw.setPosition(-1);
            }
            if(gamepad2.b){
                backClaw.setPosition(1);
            }

            double dd = gamepad2.right_stick_x;

            cp    = Range.clip(dd, -0.3, 0.3) ;

            Linkage.setPower(-cp);

            while(gamepad2.y){
                 denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                lfPower = (y + x + rx) / denominator;
                lbPower = (y - x + rx) / denominator;
                rfPower = (y - x - rx) / denominator;
                rbPower = (y + x - rx) / denominator;

                // Setting all the drive motors to their power
                leftFront.setPower(lfPower);
                leftRear.setPower(lbPower);
                rightFront.setPower(rfPower);
                rightRear.setPower(rbPower);
                LeftGeko.setPower(0.5);
                RightGeko.setPower(-0.5);
            }
            while(gamepad2.a){
                 denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                lfPower = (y + x + rx) / denominator;
                lbPower = (y - x + rx) / denominator;
                rfPower = (y - x - rx) / denominator;
                rbPower = (y + x - rx) / denominator;

                // Setting all the drive motors to their power
                leftFront.setPower(lfPower);
                leftRear.setPower(lbPower);
                rightFront.setPower(rfPower);
                rightRear.setPower(rbPower);
                LeftGeko.setPower(-1);
                RightGeko.setPower(1);

            }
            LeftGeko.setPower(0);
            RightGeko.setPower(0);
            if(gamepad2.dpad_up){
                BackLeftMisumi.setPosition(0.69);
                BackRightMisumi.setPosition(-0.69);
            }
            if(gamepad2.dpad_down){
                BackLeftMisumi.setPosition(-1);
                BackRightMisumi.setPosition(1);
            }

            if(gamepad2.dpad_left){
                MRightckMisumi.setPower(1);
                MLeftBackMisumi.setPower(1);
            }
            if(gamepad2.dpad_right){
                MRightckMisumi.setPower(-1);
                MLeftBackMisumi.setPower(-1);
            }
            if(gamepad2.left_bumper){
                rightmisumi.setPosition(0.87);
                leftmisumi.setPosition(-0.87);
            }
            if(gamepad2.right_bumper){
                rightmisumi.setPosition(-0.85);//0.5
                leftmisumi.setPosition(0.85);
            }
            if(gamepad2.right_stick_button){
                wrist.setPosition(0.8);
            }
            if(gamepad2.left_stick_button){
                wrist.setPosition(0);
            }

            double d = gamepad2.left_stick_y;

            p = Range.clip(d, -1, 1) ;
            BackRightMisumi.setPosition(p);//0.5
            BackLeftMisumi.setPosition(-(p));


        }
    }
}
