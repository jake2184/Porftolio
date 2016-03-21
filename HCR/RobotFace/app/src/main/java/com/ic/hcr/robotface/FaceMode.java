package com.ic.hcr.robotface;

import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.location.Criteria;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.speech.tts.TextToSpeech;
import android.speech.tts.UtteranceProgressListener;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;

import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

/**
 * Created by Cian on 01/12/2015.
 */
public class FaceMode extends AppCompatActivity implements LocationProvider.LocationCallback, OrientationProvider.OrientationCallback {
    private static final String TAG = "FaceMode";

    private NormalModeCallback normalModeCallback;

    private LocationProvider locationProvider;

    private OrientationProvider orientationProvider;

    private TextToSpeech tts;

    private boolean ttsInitialized = false;

    private ScheduledThreadPoolExecutor scheduledExecutor;

    private ScheduledFuture<?> scheduledFuture;

    private ThreadPoolExecutor runnerExecutor;

    private Future<?> runnerFuture;

    private Queue<RobotFaceAction> actionQueue;

    private String pollServerAddress;

    private int currentSequenceId;

    private UtteranceProgressListenerImpl utteranceProgressListener;

    public boolean isTtsInitialized() {
        return ttsInitialized;
    }

    public void addAction(RobotFaceAction action) {
        actionQueue.add(action);
    }

    public TextToSpeech getTts() {
        return tts;
    }

    public RobotFaceAction getAction() {
        return actionQueue.poll();
    }

    public int getCurrentSequenceId() {
        return currentSequenceId;
    }

    public void setCurrentSequenceId(int currentSequenceId) {
        this.currentSequenceId = currentSequenceId;
    }

    private void startROSPolling() {
        Log.i(TAG, "Initializing ROS polling...");
        scheduledExecutor = (ScheduledThreadPoolExecutor) Executors.newScheduledThreadPool(1);
        PollForROSMessage pollRunnable = new PollForROSMessage(this, pollServerAddress + "/getMessage?sequenceId=");
        scheduledFuture = scheduledExecutor.scheduleWithFixedDelay(pollRunnable, 0, 1, TimeUnit.SECONDS);
        Log.i(TAG, "ROS polling started.");
    }

    private void startActionRunner() {
        Log.i(TAG, "Initializing action runner...");
        ActionRunner runner = new ActionRunner(this);
        runnerExecutor = (ThreadPoolExecutor) Executors.newFixedThreadPool(1);
        runnerFuture = runnerExecutor.submit(runner);
        Log.i(TAG, "Action running started.");
    }

    private void initializeTTS() {
        TextToSpeech.OnInitListener mTtsInitListener = new TextToSpeech.OnInitListener() {
            @Override
            public void onInit(int status) {
                if(status == TextToSpeech.SUCCESS) {
                    int result = tts.setLanguage(Locale.UK);

                    if(result == TextToSpeech.LANG_MISSING_DATA || result == TextToSpeech.LANG_NOT_SUPPORTED) {
                        Log.e(TAG, "This language is not supported.");
                    } else {
                        if(utteranceProgressListener != null) {
                            tts.setOnUtteranceProgressListener(utteranceProgressListener);
                        }
                        ttsInitialized = true;
                    }
                } else {
                    Log.e(TAG, "Initialization failed.");
                }
            }
        };

        tts = new TextToSpeech(this, mTtsInitListener);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Intent intent = getIntent();
        pollServerAddress = "http://" + intent.getStringExtra(RobotFace.POLL_SERVER_URL_MESSAGE) + ":" + intent.getIntExtra(RobotFace.POLL_SERVER_PORT_MESSAGE, 8080);
        currentSequenceId = intent.getIntExtra(RobotFace.SEQUENCE_ID_MESSAGE, 0);

        actionQueue = new ArrayBlockingQueue<RobotFaceAction>(100);

        utteranceProgressListener = new UtteranceProgressListenerImpl(this);

        initializeTTS();

        locationProvider = new LocationProvider(this, this);
        orientationProvider = new OrientationProvider(this, this);
    }

    public void setNormalModeCallback(NormalModeCallback normalModeCallback) {
        this.normalModeCallback = normalModeCallback;
    }

    @Override
    public void onResume() {
        Log.d(TAG, "onResume");
        locationProvider.connect();
        orientationProvider.connect();
        startROSPolling();
        startActionRunner();
        super.onResume();
    }

    @Override
    public void onPause() {
        Log.d(TAG, "onPause");
        if(locationProvider != null) {
            locationProvider.disconnect();
        }
        if(orientationProvider != null) {
            orientationProvider.disconnect();
        }
        if (runnerFuture != null) {
            runnerFuture.cancel(true);
            runnerFuture = null;
            runnerExecutor = null;
        }
        if (scheduledFuture != null) {
            scheduledFuture.cancel(true);
            scheduledFuture = null;
            scheduledExecutor = null;
        }
        super.onPause();
    }

    @Override
    public void onDestroy() {
        Log.d(TAG, "onDestroy");
        if(tts != null) {
            tts.stop();
            tts.shutdown();
        }
        super.onDestroy();
    }

    @Override
    public void handleNewLocation(Location location) {
        if(location != null) {
            Log.i(TAG, "Received new location: " + location);
            StatusPublisher statusPublisher = new StatusPublisher(pollServerAddress);
            Map<String, String> params = new HashMap<String, String>();
            params.put("action", "COORDINATE");
            params.put("lat", Double.toString(location.getLatitude()));
            params.put("long", Double.toString(location.getLongitude()));
            statusPublisher.execute(params);
        }
    }

    @Override
    public void handleNewOrientation(double orientation) {
        Log.i(TAG, "Received new orientation: " + orientation);
        StatusPublisher statusPublisher = new StatusPublisher(pollServerAddress);
        Map<String, String> params = new HashMap<String, String>();
        params.put("action", "ORIENTATION");
        params.put("orientation", Double.toString(orientation));
        statusPublisher.execute(params);
    }

    public void mapCallback(boolean mapEnabled) {
        if(normalModeCallback != null) {
            normalModeCallback.getMap(mapEnabled);
        }
    }

    public void alertUtteranceFinished() {
        StatusPublisher statusPublisher = new StatusPublisher(pollServerAddress);
        Map<String, String> params = new HashMap<String, String>();
        params.put("action", "SPEECH_FINISHED");
        statusPublisher.execute(params);
    }

    public void enableMap(Location location) {
        normalModeCallback.getMap(true);
    }

    public void disableMap() {
        normalModeCallback.getMap(false);
    }

    public abstract interface NormalModeCallback {
        public void getMap(boolean mapEnabled);

        public void setCurrentLocation(Location currentLocation);
    }
}
