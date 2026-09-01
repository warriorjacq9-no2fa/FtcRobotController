package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * The FTC controller has two types of operation
 * that we need to implement, TeleOp and Autonomous
 * TeleOp is how the drive team operates the robot
 * and is usually pretty simple
 */
@TeleOp(name="TeleOp")
public class FtcTeleOp extends LinearOpMode {

    /*
     * Speed is in degrees per second, so we
     * use the wheel radius to calculate speed
     * from mm/s
     */
    private static final double SPEED = (1000) / 52.0;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private CRServo intake;

    private void drive() {
        double x = SPEED * gamepad1.left_stick_x;
        double y = -SPEED * gamepad1.left_stick_y;
        double rx = SPEED * gamepad1.right_stick_x;

        /* Mecanum drive equations */
        frontLeft.setVelocity(x + y + rx, AngleUnit.RADIANS);
        frontRight.setVelocity(x - y - rx, AngleUnit.RADIANS);
        backLeft.setVelocity(x - y + rx, AngleUnit.RADIANS);
        backRight.setVelocity(x + y - rx, AngleUnit.RADIANS);

        /*
         * A = intake forward
         * B = intake reverse
         */
        if(gamepad1.a)
            intake.setPower(1);
        else if(gamepad1.b)
            intake.setPower(-1);
        else
            intake.setPower(0);
    }

    @Override
    public void runOpMode() {
        /*
         * Get pointers to the motors in the robot
         * The deviceName string is the name we
         * enter in the controller
         */
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        intake = hardwareMap.get(CRServo.class, "intake");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()) drive();
    }
}
