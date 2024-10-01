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