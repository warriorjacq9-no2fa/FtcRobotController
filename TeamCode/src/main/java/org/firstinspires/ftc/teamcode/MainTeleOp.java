package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/*
 * The FTC controller has two types of operation
 * that we need to implement, TeleOp and Autonomous
 * TeleOp is how the drive team operates the robot
 * and is usually pretty simple
 */
@TeleOp(name="MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    /*
     * Speed is in degrees per second, so we
     * use the wheel radius to calculate speed
     * from mm/s
     */
    private static final double SPEED = (750) / 52.0;
    private static final double INTAKE_SPEED = (1000) * ((2 * Math.PI) / 60);

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx intake;
    private IMU imu;

    private void drive() {
        double x = SPEED * -gamepad1.left_stick_y;
        double y = SPEED * gamepad1.left_stick_x;
        double rx = SPEED * gamepad1.right_stick_x;

        /* Mecanum drive equations */
        frontLeft.setVelocity(x + y + rx, AngleUnit.RADIANS);
        frontRight.setVelocity(x - y - rx, AngleUnit.RADIANS);
        backLeft.setVelocity(x - y + rx, AngleUnit.RADIANS);
        backRight.setVelocity(x + y - rx, AngleUnit.RADIANS);
    }

    private double heading;
    private void runImu() {
        heading = imu.getRobotOrientation(
                AxesReference.EXTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        ).thirdAngle;
        telemetry.addData("Heading", heading);
    }

    private void runIntake() {
        if(gamepad1.a)
            intake.setVelocity(INTAKE_SPEED);
        else if(gamepad1.b)
            intake.setVelocity(-INTAKE_SPEED);
        else
            intake.setVelocity(0);

    }
    private void driveGlobal() {
        double x = SPEED * (Math.cos(heading) * gamepad1.left_stick_x + Math.sin(heading) * gamepad1.left_stick_y);
        double y = SPEED * (-Math.sin(heading) * gamepad1.left_stick_x + Math.cos(heading) * gamepad1.left_stick_y);
        double rx = SPEED * gamepad1.right_stick_x;

        frontLeft.setVelocity(x + y + rx, AngleUnit.RADIANS);
        frontRight.setVelocity(x - y - rx, AngleUnit.RADIANS);
        backLeft.setVelocity(x - y + rx, AngleUnit.RADIANS);
        backRight.setVelocity(x + y - rx, AngleUnit.RADIANS);
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
        //intake = hardwareMap.get(DcMotorEx.class, "intake");
        imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            runImu();
            //runIntake();
            if(gamepad1.right_trigger_pressed) {
                driveGlobal();
            } else {
                drive();
            }
        }
    }
}
