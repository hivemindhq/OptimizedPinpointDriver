/**
 * Written by Polar from Hivemind Robotics
 *
 * This is licensed under a MIT License.
 */

package com.gobilda.pinpoint

enum class OdometryTypes(val ticksPerMM: Float) {
    GOBILDA_SWING_ARM(13.262912f),
    GOBILDA_FOUR_BAR(19.894367f)
}

enum class EncoderDirection {
    FORWARD,
    REVERSE
}

enum class DeviceStatus(val status: Int) {
    NOT_READY(0),
    READY(1),
    CALIBRATING(1 shl 1),
    FAULT_X_POD_NOT_DETECTED(1 shl 2),
    FAULT_Y_POD_NOT_DETECTED(1 shl 3),
    FAULT_NO_PODS_DETECTED((1 shl 2) or (1 shl 3)),
    FAULT_IMU_RUNAWAY(1 shl 4);
}
