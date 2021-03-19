package org.firstinspires.ftc.teamcode.Vision;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "CVTest", group = "Out of this World")
public class CVTestAuto extends OpMode {

    public static double rows = 0.71;
    public static double rect1Cols = 0.515;
    public static double rect2Cols = 0.5625;
    RingDetectorV3 ringDetectorV3;

    public void init()
    {
        ringDetectorV3 = new RingDetectorV3(hardwareMap, telemetry, rows, rect1Cols, rect2Cols);
        ringDetectorV3.init();
        FtcDashboard.getInstance().startCameraStream(ringDetectorV3.webcam, 20);
    }

    public void loop()
    {
            telemetry.addData("State:", ringDetectorV3.getRingPosition());
            telemetry.addData("lowcolor", ringDetectorV3.getVal());
            telemetry.addData("upcolor", ringDetectorV3.getVal2());
            telemetry.update();
    }
}