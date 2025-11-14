#include "position.h"
#include "encoder.h"
#include <communication/communication.h>
#include <tools/timeTask/timeTask.h>
#include <math.h>
#include <inttypes.h>
#include <stddef.h>

// Konstanten für Kamera-Pose und Korrektur
#define CAMERA_POSE_MAX_AGE_MS 10000         // 10 Sekunden
#define CORRECTION_POSITION_THRESHOLD_MM 50.0f  // 50mm
#define CORRECTION_ANGLE_THRESHOLD_RAD 0.2f     // ≈11°
#define ERROR_POSITION_THRESHOLD_MM 100.0f       // 100mm
#define ERROR_ANGLE_THRESHOLD_RAD 0.3f           // ≈17°

// Aktuelle erwartete Pose des Roboters (Odometrie)
static Pose_t expectedPose = { 200.0f, 0.0f, M_PI_2 };

// Kamera-Pose (AprilTag)
static Pose_t truePose = { 200.0f, 0.0f, M_PI_2 };
static timeTask_time_t lastCameraUpdate = { 0, 0 };
static uint8_t cameraPoseValid = 0;  // Flag ob Kamera-Pose jemals empfangen wurde

// Pose-Differenz zwischen Odometrie und Kamera
static Pose_t poseDifference = { 0.0f, 0.0f, 0.0f };

// Letzte Encoder-Werte für Delta-Berechnung
static int16_t lastEncoderR = 0;
static int16_t lastEncoderL = 0;
static uint8_t initialized = 0;

// Forward declaration
const RobotParameters_t* getRobotParams(void);

void position_init(const Pose_t* initialPose) {
    if (initialPose != NULL) {
        expectedPose = *initialPose;
        // Synchronisiere auch Kamera-Pose auf Start-Pose
        truePose = *initialPose;
    }
    
    // Initialisiere Encoder-Referenzwerte
    lastEncoderR = encoder_getCountR();
    lastEncoderL = encoder_getCountL();
    initialized = 1;
    
    // Kamera-Pose ist noch nicht empfangen, aber auf Start-Pose gesetzt
    cameraPoseValid = 0;  // Wird auf 1 gesetzt, wenn erste echte Kamera-Pose kommt
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

// Schritt 1: Kamera-Pose-Speicherung

void position_setAprilTagPose(const Pose_t* cameraPose) {
    if (cameraPose != NULL) {
        // Wenn dies die erste Kamera-Pose ist, synchronisiere Odometrie darauf
        // (nur wenn noch keine große Bewegung stattgefunden hat)
        if (!cameraPoseValid) {
            // Erste Kamera-Pose: Synchronisiere Odometrie
            expectedPose = *cameraPose;
            
            // Normalisiere Theta auf [-π, +π]
            while (expectedPose.theta > M_PI) {
                expectedPose.theta -= 2.0f * M_PI;
            }
            while (expectedPose.theta < -M_PI) {
                expectedPose.theta += 2.0f * M_PI;
            }
        }
        
        truePose = *cameraPose;
        
        // Normalisiere Theta auf [-π, +π]
        while (truePose.theta > M_PI) {
            truePose.theta -= 2.0f * M_PI;
        }
        while (truePose.theta < -M_PI) {
            truePose.theta += 2.0f * M_PI;
        }
        
        // Aktualisiere Zeitstempel
        timeTask_getTimestamp(&lastCameraUpdate);
        cameraPoseValid = 1;
    }
}

const Pose_t* position_getAprilTagPose(void) {
    return &truePose;
}

uint32_t position_getLastCameraUpdateTime(void) {
    if (!cameraPoseValid) {
        return UINT32_MAX;  // Noch nie empfangen
    }
    
    timeTask_time_t now;
    timeTask_getTimestamp(&now);
    return timeTask_getDuration(&lastCameraUpdate, &now);
}

// Schritt 2: Pose-Differenz-Berechnung

uint8_t position_hasValidCameraPose(void) {
    if (!cameraPoseValid) {
        return 0;  // Noch nie empfangen
    }
    
    // Prüfe ob Kamera-Pose nicht zu alt ist
    uint32_t age_us = position_getLastCameraUpdateTime();
    uint32_t maxAge_us = CAMERA_POSE_MAX_AGE_MS * 1000UL;
    
    return (age_us < maxAge_us) ? 1 : 0;
}

void position_calculatePoseDifference(void) {
    if (!position_hasValidCameraPose()) {
        // Keine gültige Kamera-Pose - Differenz auf 0 setzen
        poseDifference.x = 0.0f;
        poseDifference.y = 0.0f;
        poseDifference.theta = 0.0f;
        return;
    }
    
    // Berechne Differenz: truePose - expectedPose
    poseDifference.x = truePose.x - expectedPose.x;
    poseDifference.y = truePose.y - expectedPose.y;
    
    // Theta-Differenz normalisieren (kürzester Weg)
    poseDifference.theta = truePose.theta - expectedPose.theta;
    
    // Normalisiere auf [-π, +π]
    while (poseDifference.theta > M_PI) {
        poseDifference.theta -= 2.0f * M_PI;
    }
    while (poseDifference.theta < -M_PI) {
        poseDifference.theta += 2.0f * M_PI;
    }
}

const Pose_t* position_getPoseDifference(void) {
    return &poseDifference;
}

uint8_t position_wasCameraPoseReceived(void) {
    return cameraPoseValid;
}

uint8_t position_syncPoses(void) {
    // Debug-Informationen
    if (!cameraPoseValid) {
        return 0;  // Kamera-Pose wurde noch nie empfangen
    }
    
    // Prüfe ob Kamera-Pose zu alt ist
    uint32_t age_us = position_getLastCameraUpdateTime();
    uint32_t maxAge_us = CAMERA_POSE_MAX_AGE_MS * 1000UL;
    
    if (age_us >= maxAge_us) {
        // Kamera-Pose ist zu alt
        return 0;
    }
    
    // Synchronisiere Odometrie auf Kamera-Pose
    expectedPose = truePose;
    
    // Normalisiere Theta auf [-π, +π]
    while (expectedPose.theta > M_PI) {
        expectedPose.theta -= 2.0f * M_PI;
    }
    while (expectedPose.theta < -M_PI) {
        expectedPose.theta += 2.0f * M_PI;
    }
    
    return 1;  // Erfolgreich synchronisiert
}

void position_applyCorrection(void) {
    // Prüfe ob gültige Kamera-Pose vorhanden
    if (!position_hasValidCameraPose()) {
        return;  // Keine Korrektur möglich
    }
    
    // Hole aktuelle Differenz (muss bereits berechnet sein)
    const Pose_t* diff = position_getPoseDifference();
    
    // Prüfe ob Differenz über Threshold liegt
    float absPosDiff = sqrtf(diff->x * diff->x + diff->y * diff->y);
    float absAngleDiff = (diff->theta > 0) ? diff->theta : -diff->theta;
    
    // Prüfe ob Korrektur nötig ist
    if (absPosDiff < CORRECTION_POSITION_THRESHOLD_MM && 
        absAngleDiff < CORRECTION_ANGLE_THRESHOLD_RAD) {
        return;  // Differenz zu klein, keine Korrektur nötig
    }
    
    // Bestimme ob Roboter geradeaus fährt oder dreht
    // Prüfe ob große Winkeländerung in letzter Zeit (heuristisch: große Theta-Differenz deutet auf Drehen hin)
    uint8_t isTurning = (absAngleDiff > CORRECTION_ANGLE_THRESHOLD_RAD) ? 1 : 0;
    
    // Gewichtete Fusion
    float alphaPos = 0.7f;  // 70% Odometrie, 30% Kamera für Position
    float alphaTheta;
    
    if (isTurning) {
        // Beim Drehen: Mehr Gewicht auf Kamera (Theta-Korrektur wichtiger)
        alphaTheta = 0.5f;  // 50% Odometrie, 50% Kamera
    } else {
        // Bei Geradeausfahrt: Mehr Gewicht auf Odometrie (Position bereits gut)
        alphaTheta = 0.9f;  // 90% Odometrie, 10% Kamera
    }
    
    // Korrigiere Position
    expectedPose.x = alphaPos * expectedPose.x + (1.0f - alphaPos) * truePose.x;
    expectedPose.y = alphaPos * expectedPose.y + (1.0f - alphaPos) * truePose.y;
    
    // Korrigiere Theta (kürzester Weg)
    // Berechne gewichtete Theta-Differenz
    float thetaDiff = truePose.theta - expectedPose.theta;
    
    // Normalisiere Theta-Differenz auf [-π, +π]
    while (thetaDiff > M_PI) {
        thetaDiff -= 2.0f * M_PI;
    }
    while (thetaDiff < -M_PI) {
        thetaDiff += 2.0f * M_PI;
    }
    
    // Wende gewichtete Korrektur an
    expectedPose.theta += (1.0f - alphaTheta) * thetaDiff;
    
    // Normalisiere Theta auf [-π, +π]
    while (expectedPose.theta > M_PI) {
        expectedPose.theta -= 2.0f * M_PI;
    }
    while (expectedPose.theta < -M_PI) {
        expectedPose.theta += 2.0f * M_PI;
    }
    
    // Logge nur bei größeren Abweichungen (vermeidet Log-Spam)
    static uint16_t correctionCount = 0;
    if (++correctionCount >= 50) {  // Alle ~1 Sekunde (50 * 20ms)
        int16_t posDiff_mm = (int16_t)absPosDiff;
        int16_t angleDiff_mrad = (int16_t)(absAngleDiff * 1000.0f);
        communication_log(LEVEL_FINE, "Odometrie korrigiert: Pos=%d mm, Theta=%d mrad", 
                         posDiff_mm, angleDiff_mrad);
        correctionCount = 0;
    }
}

