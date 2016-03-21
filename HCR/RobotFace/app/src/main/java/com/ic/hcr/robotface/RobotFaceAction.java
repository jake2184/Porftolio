package com.ic.hcr.robotface;

import android.util.Log;

/**
 * Created by Cian on 29/11/2015.
 */
public class RobotFaceAction {
    private static final String TAG = "RobotFaceAction";

    protected int sequenceId;
    protected ActionStatus status;
    protected ActionType type;

    public RobotFaceAction() {

    }

    public static RobotFaceAction createNewAction(int sequenceId, ROSMessage message) {
        switch(message.getType()) {
            case TEXT:
                return new SpeechAction(sequenceId, message);
            case SHOW_MAP:
                return new MapAction(sequenceId, true, message);
            case HIDE_MAP:
                return new MapAction(sequenceId, false, message);
            default:
                return null;
        }
    }

    public void setStatus(ActionStatus status) {
        this.status = status;
    }

    public int getSequenceId() {
        return sequenceId;
    }

    public ActionType getType() {
        return type;
    }

    public ActionStatus getStatus() {
        return status;
    }
}

