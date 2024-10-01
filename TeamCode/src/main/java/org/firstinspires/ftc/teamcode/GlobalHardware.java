package org.firstinspires.ftc.teamcode;

import com.gobilda.pinpoint.PinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;
import java.util.Random;
import java.util.Set;

/**
 * This is to be used as an example robot class.
 * NOTE: This assumes you are only bulk reading the control hub
 * -> This is meant to absolutely kill by writing to each motor every loop.
 */
public class GlobalHardware {
    private static GlobalHardware INSTANCE = null;
    
    private HardwareMap hardwareDevices;
    
    private List<LynxModule> allHubs;
    private static LynxModule CONTROL_HUB;

    public Set<DcMotorEx> motors;
    public Set<DcMotorEx> servos;

    public PinpointDriver pinpoint;

    private boolean stressTesting;

    public static GlobalHardware getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new GlobalHardware();
            return INSTANCE;
        } else return INSTANCE;
    }
    
    public void init(HardwareMap hardwareMap, PinpointDriver.ReadData type) {
        hardwareDevices = hardwareMap;
        allHubs = hardwareMap.getAll(LynxModule.class);

        pinpoint = hardwareMap.get(PinpointDriver.class, "pinpoint");
        pinpoint.setType(type);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (hub.isParent() && (LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber()))) {
                CONTROL_HUB = hub;
            }
        }
    }

    public void setStressTest(boolean shouldTest) {
        stressTesting = shouldTest;
    }

    /**
     * Please note if you plan on actually using this remove the
     * isOverCurrent and random setPower's lol
     **/
    public void read() {
        pinpoint.bulkRead();

        // Remove this if you plan on using this
        if (stressTesting) {
            motors.forEach(DcMotorEx::isOverCurrent);
        }
    }

    public void write() {
        if (stressTesting) {
            Random random = new Random();
            double newPower = -1 + 2 * random.nextDouble();
            motors.forEach(motor -> motor.setPower(newPower));
        }
    }

    public void loop() {}

    public void clearBulkCache() {
        CONTROL_HUB.clearBulkCache();
    }
}
