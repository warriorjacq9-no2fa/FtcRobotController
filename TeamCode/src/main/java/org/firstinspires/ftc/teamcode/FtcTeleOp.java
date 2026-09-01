package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * The FTC controller has two types of operation
 * that we need to implement, TeleOp and Autonomous
 * TeleOp is how the drive team operates the robot
 * and is usually pretty simple
 */
@TeleOp(name="TeleOp")
public class FtcTeleOp extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private CRServo intake;

    private void drive() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        /* Mecanum drive equations */
        frontLeft.setPower(x + y + rx);
        frontRight.setPower(x - y - rx);
        backLeft.setPower(x - y + rx);
        backRight.setPower(x + y - rx);

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
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intake = hardwareMap.get(CRServo.class, "intake");

        waitForStart();

        while(opModeIsActive()) drive();
    }
}
