package com.ic.hcr.robotface;

import android.util.Log;

/**
 * Created by Cian on 24/11/2015.
 */
public class ROSMessage {
    private final int sequenceId;
    private ActionType type;
    private final String message;
    private final boolean valid;

    public ROSMessage(int sequenceId, String rawMessage) {
        this.sequenceId = sequenceId;
        if(rawMessage != null) {
            String[] parts = rawMessage.split(":");
            if(parts.length <= 2) {
                try {
                    this.type = ActionType.valueOf(parts[0]);
                } catch(Exception e) { // Catch all and indicate to SKIP message.
                    this.type = ActionType.SKIP;
                }
                String theMessage = "";
                for(int i = 1; i < parts.length; i++) {
                    theMessage += parts[i];
                }
                this.message = theMessage;
                this.valid = true;
            } else {
                message = null;
                type = null;
                valid = false;
            }
        } else {
            message = null;
            type = null;
            valid = false;
        }
    }

    public int getSequenceId() {
        return sequenceId;
    }

    public ActionType getType() {
        return type;
    }

    public String getMessage() {
        return message;
    }

    public boolean isValid() {
        return valid;
    }
}
