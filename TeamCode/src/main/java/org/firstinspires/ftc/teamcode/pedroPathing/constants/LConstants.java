package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.constants.PinpointConstants;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        PinpointConstants.hardwareMapName = "pinpoint";

        PinpointConstants.forwardY = 2.3;
        PinpointConstants.strafeX = -9.0;
        PinpointConstants.distanceUnit = DistanceUnit.CM;

        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
}




