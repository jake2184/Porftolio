package com.ic.hcr.robotface;

import android.util.Log;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;

/**
 * Created by Cian on 29/11/2015.
 */
public class PollForROSMessage implements Runnable {
    private static final String TAG = "PFROSMessageRunnable";

    private final FaceMode parentFaceMode;
    private final String url;

    public PollForROSMessage(FaceMode parentFaceMode, String url) {
        this.parentFaceMode = parentFaceMode;
        this.url = url;
    }

    @Override
    public void run() {
        try {
            ROSMessage newMessage = getROSMessage(url, parentFaceMode.getCurrentSequenceId());
            if(newMessage != null && newMessage.isValid()) { // Got a valid message!
                    RobotFaceAction action = RobotFaceAction.createNewAction(newMessage.getSequenceId(), newMessage);
                    if(action != null) {
                        Log.d(TAG, "Adding new " + action.getType() + " message.");
                        parentFaceMode.addAction(action);
                        parentFaceMode.setCurrentSequenceId(parentFaceMode.getCurrentSequenceId() + 1);
                    }
            } else {
                    Log.d(TAG, "Did not receive a new message.");
            }
        } catch(MalformedURLException mue) {
            Log.e(TAG, "Malformed URL: " + mue.getMessage());
        } catch(IOException ioe) {
            Log.e(TAG, "IOException: " + ioe.getMessage());
        }
    }

    private ROSMessage getROSMessage(String serverAddress, int currentSequenceId) throws IOException {
        URL serverURL = new URL(serverAddress + currentSequenceId + "&action=GET_UPDATE");
        HttpURLConnection conn = (HttpURLConnection) serverURL.openConnection();
        conn.setRequestMethod("GET");
        int responseCode = conn.getResponseCode();
        if(responseCode == 200) {
            Log.d(TAG, "Received a response");
            BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()));
            String inputLine;
            StringBuffer response = new StringBuffer();
            while((inputLine = in.readLine()) != null) {
                response.append(inputLine);
            }
            in.close();
            Log.d(TAG, "Sequence ID: " + currentSequenceId + ", message: " + response.toString());
            return new ROSMessage(currentSequenceId, response.toString());
        } else {
            Log.w(TAG, "Error response from server.");
        }
        return null;
    }
}
