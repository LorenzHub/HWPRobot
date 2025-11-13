#include "position.h"
#include "encoder.h"
#include <communication/communication.h>
#include <math.h>
#include <inttypes.h>

// Aktuelle erwartete Pose des Roboters
static Pose_t expectedPose = { 200.0f, 0.0f, M_PI_2 };

// Letzte Encoder-Werte für Delta-Berechnung
static int16_t lastEncoderR = 0;
static int16_t lastEncoderL = 0;
static uint8_t initialized = 0;

// Forward declaration
const RobotParameters_t* getRobotParams(void);

void position_init(const Pose_t* initialPose) {
    if (initialPose != NULL) {
        expectedPose = *initialPose;
    }
    
    // Initialisiere Encoder-Referenzwerte
    lastEncoderR = encoder_getCountR();
    lastEncoderL = encoder_getCountL();
    initialized = 1;
}

void position_updateExpectedPose(void) {
    if (!initialized) {
        // Erste Initialisierung
        lastEncoderR = encoder_getCountR();
        lastEncoderL = encoder_getCountL();
        initialized = 1;
        return;
    }
    
    // Lese aktuelle Encoder-Werte
    int16_t currentEncoderR = encoder_getCountR();
    int16_t currentEncoderL = encoder_getCountL();
    
    // Berechne Deltas seit letztem Update
    int16_t deltaR = currentEncoderR - lastEncoderR;
    int16_t deltaL = currentEncoderL - lastEncoderL;
    
    // Speichere aktuelle Werte für nächstes Update
    lastEncoderR = currentEncoderR;
    lastEncoderL = currentEncoderL;
    
    // Hole Robot-Parameter
    const RobotParameters_t* params = getRobotParams();
    
    // Berechne Differenz zwischen den Rädern
    int16_t diffLR = deltaR - deltaL;
    int16_t absDiffLR = (diffLR > 0) ? diffLR : -diffLR;
    
    // Prüfe ob Geradeausfahrt (nur bei sehr kleinen Differenzen)
    // Threshold: 2-3 Ticks für numerische Stabilität
    const int16_t threshold = 3;
    
    if (absDiffLR < threshold) {
        // Fall 1: Geradeausfahrt
        // Verwende Durchschnitt beider Räder für genauere Berechnung
        float avgDelta = ((float)(deltaR + deltaL) / 2.0f);
        float d = avgDelta * params->distPerTick;
        expectedPose.x += d * cosf(expectedPose.theta);
        expectedPose.y += d * sinf(expectedPose.theta);
        // Theta bleibt unverändert bei Geradeausfahrt
    } else {
        // Fall 2: Kurvenfahrt
        // Berechne Winkeländerung
        float dTheta = (float)diffLR * params->distPerTick / params->axleWidth;
        
        // Berechne Kurvenradius
        float R = ((float)(deltaR + deltaL) / (float)diffLR) * (params->axleWidth / 2.0f);
        
        // Berechne Positionsänderung
        expectedPose.x += R * (sinf(expectedPose.theta + dTheta) - sinf(expectedPose.theta));
        expectedPose.y += R * (cosf(expectedPose.theta) - cosf(expectedPose.theta + dTheta));
        
        // Aktualisiere Theta
        expectedPose.theta += dTheta;
    }
    
    // Normalisiere Theta auf [-π, +π]
    while (expectedPose.theta > M_PI) {
        expectedPose.theta -= 2.0f * M_PI;
    }
    while (expectedPose.theta < -M_PI) {
        expectedPose.theta += 2.0f * M_PI;
    }
}

const Pose_t* position_getCurrentPose(void) {
    return &expectedPose;
}

void position_setPose(const Pose_t* newPose) {
    if (newPose != NULL) {
        expectedPose = *newPose;
        
        // Normalisiere Theta auf [-π, +π]
        while (expectedPose.theta > M_PI) {
            expectedPose.theta -= 2.0f * M_PI;
        }
        while (expectedPose.theta < -M_PI) {
            expectedPose.theta += 2.0f * M_PI;
        }
    }
}

