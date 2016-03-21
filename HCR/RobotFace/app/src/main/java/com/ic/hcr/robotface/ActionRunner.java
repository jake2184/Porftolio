package com.ic.hcr.robotface;

import android.speech.tts.TextToSpeech;
import android.util.Log;

/**
 * Created by Cian on 29/11/2015.
 */
public class ActionRunner implements Runnable {
    private static final String TAG = "ActionRunner";

    private final FaceMode parentFaceMode;

    private RobotFaceAction currentAction;

    public ActionRunner(FaceMode parentFaceMode) {
        this.parentFaceMode = parentFaceMode;
        this.currentAction = null;
    }

    @Override
    public void run() {
        Log.i(TAG, "Started ActionRunner.");
        while(true) {
            if(currentAction == null || currentAction.getStatus() == ActionStatus.FINISHED) {
                currentAction = parentFaceMode.getAction();
            }
            if(currentAction != null) {
                if (currentAction.getStatus() == ActionStatus.NEW) {
                    if (currentAction.getType() == ActionType.SKIP) {
                        currentAction.setStatus(ActionStatus.FINISHED);
                    } else {
                        currentAction.setStatus(ActionStatus.STARTED);
                    }
                }

                if(currentAction.getStatus() == ActionStatus.STARTED) {
                    switch (currentAction.getType()) {
                        case TEXT:
                            SpeechAction speechAction = (SpeechAction) currentAction;
                            if(speakOut(speechAction.getMessage(), (speechAction.isReturnCompleted() ? speechAction.getSequenceId() : -1))) {
                                currentAction.setStatus(ActionStatus.FINISHED);
                            }
                            break;
                        case SHOW_MAP:
                            MapAction mapAction = (MapAction) currentAction;
                            parentFaceMode.enableMap(mapAction.getLocation());
                            break;
                        case HIDE_MAP:
                            parentFaceMode.disableMap();
                            break;
                        default:
                            currentAction.setStatus(ActionStatus.FINISHED);
                    }
                }
            }
        }
    }

    private boolean speakOut(String textToSay, int sequenceId) {
        TextToSpeech tts = parentFaceMode.getTts();
        if(tts != null && parentFaceMode.isTtsInitialized()) {
            tts.speak(textToSay, TextToSpeech.QUEUE_ADD, null, "" + sequenceId);
            return true;
        }
        return false;
    }
}
