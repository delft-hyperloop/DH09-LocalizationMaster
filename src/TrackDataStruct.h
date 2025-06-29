#ifndef TRACK_DATA_H
#define TRACK_DATA_H

// Define the TrackData structure
struct TrackData {
    int id;                // Unique identifier for the track
    float length;         // Length of the track in meters
    float actualStart;    // Actual start position of the track in meters
    float actualEnd;      // Actual end position of the track in meters
    float barcodeStart;   // Start position of the barcode in meters        
    float barcodeEnd;     // End position of the barcode in meters
};

#endif