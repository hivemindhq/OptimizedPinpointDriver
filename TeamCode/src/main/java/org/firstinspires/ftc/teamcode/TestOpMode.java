package org.firstinspires.ftc.teamcode;

import com.gobilda.pinpoint.PinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "test your loop time with pinpoint!", group = "!")
public class TestOpMode extends LinearOpMode {
    private final GlobalHardware robot = GlobalHardware.getInstance();

    private double loopTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, PinpointDriver.ReadData.TRANSLATION_AND_ROTATION);
        robot.setStressTest(true);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.read();
            robot.write();
            robot.loop();
            robot.clearBulkCache();

            double loop = System.nanoTime();
            telemetry.addData("hz", 1000000000 / (loop - loopTime));
            telemetry.addData("pinpoint read (ms)", robot.pinpoint.getLoopTime());
            telemetry.addData("pinpoint internal loop (ms)", robot.pinpoint.getInternalLoopTime());
            loopTime = loop;
            telemetry.update();
        }
    }
}
