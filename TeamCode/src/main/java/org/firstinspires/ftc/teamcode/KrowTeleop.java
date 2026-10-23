package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name ="TestTele0p")
public class Teleop extends OpMode {

    private static final double SPEED = (750) / 52.0;
    private DcMotorEx FrontLeft = null;
    private DcMotorEx FrontRight = null;
    private DcMotorEx BackLeft = null;
    private DcMotorEx BackRight = null;

    public void init(){

        FrontLeft = hardwareMap.get(DcMotorEx.class,"frontLeft");
        FrontRight = hardwareMap.get(DcMotorEx.class,"frontRight");
        BackLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
        BackRight = hardwareMap.get(DcMotorEx.class,"backRight");
        /* Wheel Variable = Irl Wheel from DcMotot.class */

        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE); /* Set Right Reversed */
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);  /* Set Right Reversed */
        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);  /* Set Left Normal Direction */
        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);   /* Set Left Normal Direction */
        /* Set Right Wheels to Reverse, Set Left Wheels to normal direction (Forward) */

        FrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); /* Speed 0 = Break */
        FrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);  /* Speed 0 = Break */
        BackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);  /* Speed 0 = Break */
        BackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);   /* Speed 0 = Break */
        /* Set Wheel Behavior when Speed = 0 To break */

    }



}
