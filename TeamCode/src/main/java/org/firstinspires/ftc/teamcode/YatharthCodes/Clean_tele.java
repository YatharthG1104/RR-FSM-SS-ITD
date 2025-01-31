package org.firstinspires.ftc.teamcode.YatharthCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Clean tele ", group = "Linear OpMode")
public class Clean_tele extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf_drive = null;
    private DcMotor rf_drive = null;
    private DcMotor lb_drive = null;
    private DcMotor rb_drive = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware map - Motors
        lf_drive = hardwareMap.get(DcMotor.class, "lf_drive");
        rf_drive = hardwareMap.get(DcMotor.class, "rf_drive");
        lb_drive = hardwareMap.get(DcMotor.class, "lb_drive");
        rb_drive = hardwareMap.get(DcMotor.class, "rb_drive");

        // Setting the left motors reverse because they are on the opposite side
        lf_drive.setDirection(DcMotor.Direction.REVERSE);
        lb_drive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for start
        waitForStart();
        runtime.reset();

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
            lf_drive.setPower(lfPower);
            lb_drive.setPower(lbPower);
            rf_drive.setPower(rfPower);
            rb_drive.setPower(rbPower);


        }
    }
}