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
 * Wird regelmäßig aufgerufen (z.B. alle 20ms).
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

/**
 * @brief Setzt die Kamera-Pose (AprilTag) vom HWPCS
 * 
 * @param cameraPose Kamera-Pose vom AprilTag-Tracking
 */
void position_setAprilTagPose(const Pose_t* cameraPose);

/**
 * @brief Gibt die Kamera-Pose zurück
 * 
 * @return Zeiger auf die Kamera-Pose (nicht NULL)
 */
const Pose_t* position_getAprilTagPose(void);

/**
 * @brief Gibt die Zeit seit dem letzten Kamera-Update zurück
 * 
 * @return Zeit in Mikrosekunden seit letztem Update, oder UINT32_MAX wenn noch nie empfangen
 */
uint32_t position_getLastCameraUpdateTime(void);

/**
 * @brief Prüft ob eine gültige Kamera-Pose vorhanden ist
 * 
 * Eine Kamera-Pose ist gültig, wenn sie empfangen wurde und nicht zu alt ist (< 2 Sekunden).
 * 
 * @return 1 wenn gültig, 0 sonst
 */
uint8_t position_hasValidCameraPose(void);

/**
 * @brief Berechnet die Differenz zwischen Odometrie und Kamera-Pose
 * 
 * Berechnet poseDifference = truePose - expectedPose
 * Muss aufgerufen werden, bevor position_getPoseDifference() verwendet wird.
 */
void position_calculatePoseDifference(void);

/**
 * @brief Gibt die berechnete Pose-Differenz zurück
 * 
 * @return Zeiger auf die Pose-Differenz (nicht NULL)
 */
const Pose_t* position_getPoseDifference(void);

/**
 * @brief Synchronisiert Odometrie-Pose mit Kamera-Pose
 * 
 * Setzt expectedPose = truePose, wenn eine gültige Kamera-Pose vorhanden ist.
 * Kann manuell aufgerufen werden, um beide Posen wieder zusammenzusetzen.
 * 
 * @return 1 wenn synchronisiert wurde, 0 wenn keine gültige Kamera-Pose vorhanden
 */
uint8_t position_syncPoses(void);

/**
 * @brief Gibt zurück, ob eine Kamera-Pose jemals empfangen wurde
 * 
 * @return 1 wenn jemals empfangen, 0 sonst
 */
uint8_t position_wasCameraPoseReceived(void);

/**
 * @brief Wendet automatische Korrektur auf Odometrie basierend auf Kamera-Pose an
 * 
 * Verwendet gewichtete Sensorfusion, um Odometrie mit Kamera-Pose zu kombinieren.
 * Korrigiert besonders Theta beim Drehen, um das "zu wenig drehen" Problem zu lösen.
 * Wird nach position_updateExpectedPose() und position_calculatePoseDifference() aufgerufen.
 */
void position_applyCorrection(void);

#endif /* POSITION_H */

