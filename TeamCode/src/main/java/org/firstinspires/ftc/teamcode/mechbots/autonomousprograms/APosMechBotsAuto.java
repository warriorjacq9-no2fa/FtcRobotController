package org.firstinspires.ftc.teamcode.mechbots.autonomousprograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//Talk to Lily if you want to know where "position A" is on the field. They made it up. Eventually they'll get to telling everyone.

@Autonomous(name="APosMechBotsAuto")
public class APosMechBotsAuto extends OpMode {

    //speed = ?
    //set up intake to maybe get balls in auto
    //imu ?
    private DcMotorEx frontRight = null;
    private DcMotorEx frontLeft = null;
    private DcMotorEx backRight = null;
    private DcMotorEx backLeft = null;
    private final ElapsedTime timer = new ElapsedTime();

    //Should this be a state machine? I don't know. Is it going to be a state machine? Yeah, unless someone tells me no.
    private enum AutoState {
        INIT,
        TURN,
        BACKUP,
        COMPLETE
    }

    private enum RobotType {
        TESTGUY,
        MAINBOT
    }
    private AutoState state = AutoState.INIT;

    private RobotType robotType = RobotType.MAINBOT;

    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        //!!!! Not yet certain what these will need to be set to for Real Bot, but for Test Guy #1 it's FR: R, BR: F, FL: F, BL: R
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

    @Override
    public void init_loop() {
        if (gamepad1.b) {
            robotType = RobotType.TESTGUY;
        }
        else if (gamepad1.a) {
            robotType = RobotType.MAINBOT;
        }
        telemetry.addData("Selected Robot Type: ", robotType);
        telemetry.addLine("A (x) for MAINBOT, B (circle) for TESTGUY");
    }

    //Drive function. 'a' is for forward, 'b' is for strafe, 'c' is for turn. Positive goes right, negative goes left (I think).
    public void drive(double forward, double strafe, double turn) {
        frontLeft.setPower(forward+strafe+turn);
        frontRight.setPower(forward-strafe-turn);
        backLeft.setPower(forward-strafe+turn);
        backRight.setPower(forward+strafe-turn);
    }

    //The length of time it is in each state needs to be figured out through testing to make it work as intended!!!
    //Talk to Lily in real life if you wanna ask about how they estimated what powers or times would work to make things do good
    //!!!Some of the calculations used track width and length so you need to alter things based on whether you're using the 14*15 inch real bot or the 18*17 inch test guy
    public void loop() {
        switch (state) {
            case INIT:
                if (timer.seconds() < 0.5) {
                    drive(0.0, 0.0, 0.0);
                    telemetry.addLine("INIT");
                    telemetry.update();
                }
                else {
                    state = AutoState.TURN;
                    timer.reset();
                }
                break;

            case TURN:
                double turnTime = (robotType == RobotType.MAINBOT)? 2.2 : 2.7;
                double driveVelocity = (robotType == RobotType.MAINBOT)? 0.11 : 0.09;
                //Turns left while moving away from the wall
                //Real Bot (14 wide * 15 long) --> timer: 2.2, a: 0.11, b: 0.0, c: -0.2
                //Test Guy #1 (18 wide * 17 long) --> timer: 2.7, a: 0.09, b: 0.0, c: -0.2
                if (timer.seconds() < turnTime) {
                    drive(driveVelocity, 0.0, -0.2);
                    telemetry.addLine("TURN (turning by " + driveVelocity +" for " +turnTime+" seconds)");
                    telemetry.update();
                }
                else {
                    state = AutoState.BACKUP;
                    timer.reset();
                }
                break;

            case BACKUP:
                //Backs into the parking area
                //BOTH Real Bot and Test Guy #1 --> timer: 9, a: -0.2, b: 0.0, c: 0.0
                if (timer.seconds() < 9) {
                    drive(-0.2, 0.0, 0.0);
                    telemetry.addLine("BACKUP");
                    telemetry.update();
                }
                else {
                    state = AutoState.COMPLETE;
                    timer.reset();
                }
                break;

            case COMPLETE:
                drive(0.0, 0.0, 0.0);
                telemetry.addLine("PARKED");
                telemetry.update();
                break;
        }
    }
}
