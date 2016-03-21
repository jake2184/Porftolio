package com.ic.hcr.robotface;

import android.location.Location;
import android.util.Log;

/**
 * Created by Cian on 08/12/2015.
 */
public class MapAction extends RobotFaceAction {
    private static final String TAG = "MapAction";

    private Location location;

    public MapAction(int sequenceId, boolean toShow, ROSMessage message) {
        this.sequenceId = sequenceId;
        this.status = ActionStatus.NEW;
        this.type = message.getType();
        if(toShow) {
            String rawMessage = message.getMessage();
            if (rawMessage != null) {
                location = new Location("RobotFace");

                String[] parts = rawMessage.split("---");
                if (parts.length <= 2) {
                    location.setLatitude(Double.parseDouble(parts[0]));
                    location.setLongitude(Double.parseDouble(parts[1]));
                } else {
                    this.type = ActionType.SKIP;
                }
            } else {
                this.type = ActionType.SKIP;
            }
        }
    }

    public Location getLocation() {
        return location;
    }
}
