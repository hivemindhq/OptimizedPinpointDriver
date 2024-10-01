/**
 * Written by Polar from Hivemind Robotics
 *
 * Adapted from Ethan Doak's Implementation of the goBILDA® Pinpoint Odometry Computer
 * officially distributed by Base10 Assets.
 *
 * This is licensed under a MIT License.
 */

package com.gobilda.pinpoint

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.I2cAddr
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import java.nio.ByteBuffer
import java.nio.ByteOrder

@I2cDeviceType
@DeviceProperties(
    name = "goBILDA® Pinpoint Odometry Computer",
    xmlTag = "goBILDAPinpoint",
    description = "goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)"
)
class PinpointDriver(
    val client: I2cDeviceSynchSimple,
    val isOwned: Boolean
): I2cDeviceSynchDevice<I2cDeviceSynchSimple>(client, isOwned) {
    private val DEFAULT_ADDRESS = 0x31
    private val mutex = Mutex()

    private val readScope = CoroutineScope(Dispatchers.IO + SupervisorJob())

    init {
        client.i2cAddress = I2cAddr.create7bit(DEFAULT_ADDRESS)
        super.registerArmingStateCallback(false)
    }

    var loopTime = 0

    var status = 0

    var internalLoopTime = 0
        get() {
            if (field != 0) {
                return 1000000 / field
            } else return 0
        }

    var xEncoderValue = 0
    var yEncoderValue = 0
    var xPosition = 0f
    var yPosition = 0f
    var hOrientation = 0f
    var xVelocity = 0f
    var yVelocity = 0f
    var hVelocity = 0f

    enum class I2CRegistry(val id: Int) {
        DEVICE_ID       (1),
        DEVICE_VERSION  (2),
        DEVICE_STATUS   (3),
        DEVICE_CONTROL  (4),
        LOOP_TIME       (5),
        X_ENCODER_VALUE (6),
        Y_ENCODER_VALUE (7),
        X_POSITION      (8),
        Y_POSITION      (9),
        H_ORIENTATION   (10),
        X_VELOCITY      (11),
        Y_VELOCITY      (12),
        H_VELOCITY      (13),
        MM_PER_TICK     (14),
        X_POD_OFFSET    (15),
        Y_POD_OFFSET    (16),
        YAW_SCALAR      (17),
        BULK_READ       (18)
    }

    enum class ReadData {
        ONLY_ROTATION,
        TRANSLATION_AND_ROTATION
    }

    var isOnlyHeading = false

    var podType: OdometryTypes? = null

    var type = ReadData.TRANSLATION_AND_ROTATION

    fun bulkRead() {
        // unsure as to if this should be readScope.async {} or how it is now.
        readScope.launch {
            while (isActive) {
                try {
                    update(type)
                } catch (_: Exception) { }
            }
        }.start()
    }

    fun setPosition(startPose: Pose2D): Pose2D {
        writeByteArray(I2CRegistry.X_POSITION, floatToByteArray(startPose.getX(DistanceUnit.MM).toFloat(), ByteOrder.LITTLE_ENDIAN))
        writeByteArray(I2CRegistry.Y_POSITION, floatToByteArray(startPose.getY(DistanceUnit.MM).toFloat(), ByteOrder.LITTLE_ENDIAN))
        writeByteArray(I2CRegistry.H_ORIENTATION, floatToByteArray(startPose.getHeading(AngleUnit.RADIANS).toFloat(), ByteOrder.LITTLE_ENDIAN))

        return startPose
    }

    private suspend fun update(data: ReadData) {
        mutex.withLock {
            val loopStart = System.currentTimeMillis()

            when (data) {
                ReadData.ONLY_ROTATION -> {
                    val buffer = ByteBuffer
                        .wrap(client.read(I2CRegistry.H_ORIENTATION.id, 4))
                        .order(ByteOrder.LITTLE_ENDIAN)

                    hOrientation = buffer.getFloat()
                }
                ReadData.TRANSLATION_AND_ROTATION -> {
                    val buffer: ByteBuffer = ByteBuffer
                        .wrap(client.read(I2CRegistry.BULK_READ.id, 40))
                        .order(ByteOrder.LITTLE_ENDIAN)

                    status = buffer.getInt()
                    internalLoopTime = buffer.getInt()
                    xEncoderValue = buffer.getInt()
                    yEncoderValue = buffer.getInt()
                    xPosition = buffer.getFloat()
                    yPosition = buffer.getFloat()
                    hOrientation = buffer.getFloat()
                    xVelocity = buffer.getFloat()
                    yVelocity = buffer.getFloat()
                    hVelocity = buffer.getFloat()
                }
            }

            val loopEnd = System.currentTimeMillis()
            loopTime = (loopEnd - loopStart).toInt()
        }
    }

    fun getPose(): Pose2D {
        return Pose2D(
            DistanceUnit.MM,
            xPosition.toDouble(),
            yPosition.toDouble(),
            AngleUnit.RADIANS,
            hOrientation.toDouble()
        )
    }

    fun getVelocity(): Pose2D {
        return Pose2D(
            DistanceUnit.MM,
            xVelocity.toDouble(),
            xVelocity.toDouble(),
            AngleUnit.RADIANS,
            hVelocity.toDouble()
        )
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer {
        return HardwareDevice.Manufacturer.Unknown
    }

    override fun getDeviceName(): String {
        return "goBILDA® Pinpoint Odometry Computer"
    }

    fun setHeadingMode(headingMode: Boolean) {
        isOnlyHeading = headingMode
    }

    private fun lookupStatus(s: Int): DeviceStatus {
        if ((s and DeviceStatus.CALIBRATING.status) != 0) {
            return DeviceStatus.CALIBRATING
        }

        val xPodDetected = (s and DeviceStatus.FAULT_X_POD_NOT_DETECTED.status) == 0
        val yPodDetected = (s and DeviceStatus.FAULT_Y_POD_NOT_DETECTED.status) == 0

        return when {
            !xPodDetected && !yPodDetected -> DeviceStatus.FAULT_NO_PODS_DETECTED
            !xPodDetected -> DeviceStatus.FAULT_X_POD_NOT_DETECTED
            !yPodDetected -> DeviceStatus.FAULT_Y_POD_NOT_DETECTED
            (s and DeviceStatus.FAULT_IMU_RUNAWAY.status) != 0 -> DeviceStatus.FAULT_IMU_RUNAWAY
            (s and DeviceStatus.READY.status) != 0 -> DeviceStatus.READY
            else -> DeviceStatus.NOT_READY
        }
    }


    private fun writeByteArray(reg: I2CRegistry, bytes: ByteArray) {
        deviceClient.write(reg.id, bytes)
    }

    private fun floatToByteArray(value: Float, byteOrder: ByteOrder): ByteArray {
        return ByteBuffer.allocate(4).order(byteOrder).putFloat(value).array()
    }

    override fun doInitialize(): Boolean {
        (client as LynxI2cDeviceSynch).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K)
        return true
    }
}
