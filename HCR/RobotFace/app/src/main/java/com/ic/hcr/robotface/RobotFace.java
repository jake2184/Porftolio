package com.ic.hcr.robotface;

import android.content.Intent;
import android.location.Location;
import android.speech.tts.TextToSpeech;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.Window;
import android.widget.Button;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.RadioGroup;

import java.util.Locale;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

/**
 * An example full-screen activity that shows and hides the system UI (i.e.
 * status bar and navigation/system bar) with user interaction.
 */
public class RobotFace extends AppCompatActivity {
    private static final String TAG = "RobotFace";

    public static final String POLL_SERVER_URL_MESSAGE = "pollServerUrl";
    public static final String POLL_SERVER_PORT_MESSAGE = "pollServerPort";
    public static final String SEQUENCE_ID_MESSAGE = "sequenceId";

    private static final String DEFAULT_POLL_SERVER_URL = "129.31.208.229";
    private static final int DEFAULT_POLL_SERVER_PORT = 8000;
    private static final int DEFAULT_SEQUENCE_ID  = 0;

    private Button startButton;
    private RadioGroup radioGroup;

    private EditText serverUrlEntry;
    private EditText serverPortEntry;
    private EditText sequenceIdEntry;

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_robot_face);

        startButton = (Button) findViewById(R.id.startButton);
        radioGroup = (RadioGroup) findViewById(R.id.radioMode);
        startButton.setOnClickListener(new OnStartClickListener());
        serverUrlEntry = (EditText) findViewById(R.id.pollServerUrlEntry);
        serverUrlEntry.setText(DEFAULT_POLL_SERVER_URL);
        serverPortEntry = (EditText) findViewById(R.id.pollServerPortEntry);
        serverPortEntry.setText(""+DEFAULT_POLL_SERVER_PORT);
        sequenceIdEntry = (EditText) findViewById(R.id.sequenceIdEntry);
        sequenceIdEntry.setText("" + DEFAULT_SEQUENCE_ID);
    }

    private class OnStartClickListener implements View.OnClickListener {
        public void onClick(View v) {
            String pollServerUrl = serverUrlEntry.getText().toString();
            int pollServerPort = Integer.valueOf(serverPortEntry.getText().toString());
            int sequenceId = Integer.valueOf(sequenceIdEntry.getText().toString());
            if(radioGroup.getCheckedRadioButtonId() == R.id.normalMode) {
                startMode(NormalMode.class, pollServerUrl, pollServerPort, sequenceId);
            } else {
                startMode(ContextualMode.class, pollServerUrl, pollServerPort, sequenceId);
            }
        }
    }

    private void startMode(Class<?> modeClass, String serverUrl, int serverPort, int sequenceId) {
        Intent intent = new Intent(this, modeClass);
        intent.putExtra(POLL_SERVER_URL_MESSAGE, serverUrl);
        intent.putExtra(POLL_SERVER_PORT_MESSAGE, serverPort);
        intent.putExtra(SEQUENCE_ID_MESSAGE, sequenceId);
        startActivity(intent);
    }
}
