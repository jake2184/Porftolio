package com.ic.hcr.robotface;

import android.util.Log;

/**
 * Created by Cian on 08/12/2015.
 */
public class SpeechAction extends RobotFaceAction {
    private static final String TAG = "SpeechAction";

    // TODO: Add callback for finished speech.
    private boolean returnCompleted = false;
    private String message;

    public SpeechAction(int sequenceId, ROSMessage message) {
        this.sequenceId = sequenceId;
        this.status = ActionStatus.NEW;
        this.type = message.getType();
        String rawMessage = message.getMessage();
        if(rawMessage != null) {
            String[] parts = rawMessage.split("---");
            if(parts.length <= 2) {
                this.returnCompleted = Boolean.parseBoolean(parts[0]);
                String theMessage = "";
                for(int i = 1; i < parts.length; i++) {
                    theMessage += parts[i];
                }
                this.message = theMessage;
            } else {
                message = null;
                type = ActionType.SKIP;
            }
        } else {
            message = null;
            type = ActionType.SKIP;
        }
    }

    public boolean isReturnCompleted() {
        return returnCompleted;
    }

    public String getMessage() {
        return message;
    }
}
