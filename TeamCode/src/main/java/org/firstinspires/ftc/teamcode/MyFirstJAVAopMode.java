package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;

import java.util.List;

@TeleOp

public class MyFirstJAVAopMode extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor launcherLeft;
    private DcMotor launcherRight;
    private DcMotor intakeM;
    private DcMotor conveyorRight;
    //private Servo gate;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Limelight3A limelight;

    //private Servo servoTest;
    public void limelight() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double xdegrees = fiducial.getTargetXDegrees(); // Where it is (left-right)
            double ydegrees = fiducial.getTargetYDegrees(); // Where it is (up-down)
            double distance = fiducial.getTargetPoseCameraSpace().getPosition().z;
            telemetry.addData("Fiducial ", id);
            telemetry.addData("xdegrees", xdegrees);
            telemetry.addData("ydegrees", ydegrees);
            telemetry.addData("distance", distance);

        }
    }

    public void launcher() {
        launcherLeft.setPower(1);
        launcherRight.setPower(1);
        telemetry.addData("Luancher", launcherLeft.getPower());
    }

    public void intake() {
        intakeM.setPower(1);
        telemetry.addData("Intake Power", intakeM.getPower());
    }

    public void conveyor() {
        if (gamepad2.left_bumper) {
            conveyorRight.setPower(1);
            //gate.setPosition(1);
        } else {
            conveyorRight.setPower(0);
            //gate.setPosition(0);
        }
        telemetry.addData("conveyor", conveyorRight.getPower());
       // telemetry.addData("gate", gate.getPosition());


    }

    public void drive() {
        double y = 0; // Remember, Y stick is reversed!
        double x = 0;
        double rx = 0;
        // run until the end of the match (driver presses STOP)

        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;

        frontLeft.setPower(x + y + rx);
        frontRight.setPower(y - x - rx);
        backLeft.setPower(y - x + rx);
        backRight.setPower(y + x - rx);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rx", rx);
        telemetry.addData("Motor Power", frontLeft.getPower());
        telemetry.addData("Status", "Running");
    }


    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        intakeM = hardwareMap.get(DcMotor.class, "intake");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        conveyorRight = hardwareMap.get(DcMotor.class, "conveyorRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        //gate = hardwareMap.get(Servo.class, "gate");

        conveyorRight.setPower(0);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            drive();
            //limelight();
            intake();
            conveyor();
            launcher();


        }
        telemetry.update();
    }
}
