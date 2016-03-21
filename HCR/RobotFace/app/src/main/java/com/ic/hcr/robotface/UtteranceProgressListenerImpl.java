package com.ic.hcr.robotface;

import android.speech.tts.UtteranceProgressListener;

/**
 * Created by Cian on 08/12/2015.
 */
public class UtteranceProgressListenerImpl extends UtteranceProgressListener {

    private FaceMode parentFaceMode;

    public UtteranceProgressListenerImpl(FaceMode parentFaceMode) {
        this.parentFaceMode = parentFaceMode;
    }

    @Override
    public void onStart(String utteranceId) {
        // Do nothing.
    }

    @Override
    public void onDone(String utteranceId) {
        if(!utteranceId.equals("-1")) {
            parentFaceMode.alertUtteranceFinished();
        }
    }

    @Override
    public void onError(String utteranceId) {
        // Do nothing.
    }
}
