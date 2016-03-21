package com.ic.hcr.robotface;

import android.os.AsyncTask;
import android.util.Log;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Map;
import java.util.Set;

/**
 * Created by Cian on 06/12/2015.
 */
public class StatusPublisher extends AsyncTask<Map<String, String>, Void, Void> {
    private static final String TAG = "StatusPublisher";

    private final String pollServerAddress;

    public StatusPublisher(String pollServerAddress) {
        this.pollServerAddress = pollServerAddress;
    }

    @Override
    protected Void doInBackground(Map<String, String>... params) {
        String parameters = "";
        for(Map.Entry<String, String> entry : params[0].entrySet()) {
            parameters += entry.getKey() + "=" + entry.getValue() + "&";
        }
        parameters = parameters.substring(0, parameters.length() - 1); // Remove last &
        Log.i(TAG, "Parameter list: " + parameters);
        try {
            publishInfo(parameters);
        } catch(IOException ioe) {
            Log.w(TAG, "IOException while publishing status: " + ioe);
        }
        return null;
    }

    private void publishInfo(String parameters) throws IOException {
        URL serverURL = new URL(pollServerAddress + "/publishInformation?" + parameters);
        HttpURLConnection conn = (HttpURLConnection) serverURL.openConnection();
        conn.setRequestMethod("GET");
        int responseCode = conn.getResponseCode();
    }
}
