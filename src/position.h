#ifndef POSITION_H
#define POSITION_H

#include <communication/packetTypes.h>
#include <stdint.h>

/**
 * @brief Initialisiert das Position-Modul mit einer Start-Pose
 * 
 * @param initialPose Start-Pose des Roboters (x, y, theta)
 */
void position_init(const Pose_t* initialPose);

/**
 * @brief Aktualisiert die erwartete Pose basierend auf Encoder-Werten
 * 
 * Berechnet die neue Position des Roboters mit Differential-Drive-Kinematik.
 * Verwendet Delta-Berechnung (speichert letzte Encoder-Werte intern).
 * Wird regelmäßig aufgerufen (z.B. alle 10ms).
 */
void position_updateExpectedPose(void);

/**
 * @brief Gibt die aktuelle Pose des Roboters zurück
 * 
 * @return Zeiger auf die aktuelle Pose (nicht NULL)
 */
const Pose_t* position_getCurrentPose(void);

/**
 * @brief Setzt die Pose manuell (für Kalibrierung oder Reset)
 * 
 * @param newPose Neue Pose des Roboters
 */
void position_setPose(const Pose_t* newPose);

#endif /* POSITION_H */

