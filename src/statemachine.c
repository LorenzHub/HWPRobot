#include "statemachine.h"
#include "calibration.h"
#include "encoder.h"
#include <motor/motor.h>
#include <tools/timeTask/timeTask.h>
#include <communication/communication.h>
#include <inttypes.h>
#include "calcPathCommand.h"
#include "labyrinth.h"
#include "position.h"
#include <stdio.h>
#include <stdlib.h>

/* Define the current state here (single-definition) */
state currentState = IDLE;

static timeTask_time_t start;

// Statische Variable für Ziel-Distanz in mm
static uint16_t targetDistance_mm = 100;  // Default: 100mm

// Statische Variable für Ziel-Ticks
static int16_t targetTicks = 1000;  // Default: 1000 Ticks

// Statische Variable für PWM-Wert des rechten Motors
static int16_t targetPWM = 3000;  // Default: 3000 PWM

// Statische Variable für Ziel-Winkel in Grad
static int16_t targetAngle_degrees = 90;  // Default: 90°

void stateMachine() {
    switch(currentState){
        case IDLE:
            Motor_stopAll();
            break;
        case Turn_On_Spot:
            /* rotate in-place */
            Motor_setPWM(3000, -3000);
            break;
        case Drive_Forward: //Motor A drives left wheel, Motor B drives right wheel
            break;
        case Drive_Forward_5sec:
            drive_Forward_5sec();
            break;
        case Calibrate_Distance:
            calibration_run();
            break;
        case Drive_Forward_Distance:
            drive_Forward_distance_mm(targetDistance_mm, targetPWM);
            break;
        case Drive_Forward_Ticks:
            drive_Forward_ticks(targetTicks, targetPWM);
            break;
        case Turn_On_Spot_Degrees:
            turn_On_Spot_degrees(targetAngle_degrees, targetPWM);
            break;
        case ExploreMaze:
            mazeExplore();
            break;
        case drive_Forward_distance_then_explore:
            drive_Forward_distance_mm_then_explore(targetDistance_mm, targetPWM);
            break;
        case turn_On_Spot_degrees_then_drive:
            turn_degrees_then_drive(targetAngle_degrees, targetPWM);
            break;    
        case FollowThePath:
            break;
    }
}

void mazeExplore(void) {
    setLabyrinthPose(*position_getCurrentPose());
    exploreMaze();
}

void drive_Forward_distance_mm_then_explore(uint16_t distance_mm, int16_t pwmRight){
    drive_Forward_distance_mm(distance_mm, pwmRight);
    if(currentState == IDLE) setState(ExploreMaze);
}

void turn_degrees_then_drive(int16_t angle_degrees, int16_t pwm){
    turn_On_Spot_degrees(angle_degrees, pwm);
    if(currentState == IDLE) setState(drive_Forward_distance_then_explore);
}

void drive_Forward_1000ticks() {
    static uint8_t initialized = 0;
    static uint8_t phase = 0;  // 0 = Soft-Start, 1 = Fahren
    static int16_t startEncoderR = 0;
    static int16_t startEncoderL = 0;
    static int16_t targetPWMLeft = 0;
    static int16_t targetPWMRight = 0;
    static int16_t currentSoftStartPWM = 0;
    static timeTask_time_t softStartTime;
    // 1 Encoder-Tick = 0.0688 mm (2048 Ticks = 1 Rad-Umdrehung)
    // Für 10 cm (100 mm): 100 mm / 0.0688 mm = 1453 Ticks
    const int16_t targetTicks = 1000;  // 10 cm = 100 mm
    const int16_t softStartMinPWM = 1000;  // Start-PWM für Soft-Start
    const int16_t softStartStep = 200;     // PWM-Schrittweite für Soft-Start
    const uint32_t softStartInterval = 50000UL; // 50ms zwischen Soft-Start-Schritten
    
    if (!initialized) {
        startEncoderR = encoder_getCountR();
        startEncoderL = encoder_getCountL();
        // Hole kalibrierte PWM-Werte für 3000 PWM (Standard)
        targetPWMLeft = calibration_getPWMLeft(3000);
        targetPWMRight = calibration_getPWMRight(3000);
        currentSoftStartPWM = softStartMinPWM;
        Motor_setPWM(currentSoftStartPWM, currentSoftStartPWM);
        timeTask_getTimestamp(&softStartTime);
        phase = 0;  // Starte mit Soft-Start
        initialized = 1;
        communication_log(LEVEL_INFO, "Fahre 10 cm (100 mm) mit Soft-Start, Ziel-PWM L=%" PRId16 " R=%" PRId16 "...", 
                         targetPWMLeft, targetPWMRight);
    }
    
    if (phase == 0) {
        // Phase 0: Soft-Start - PWM schrittweise hochfahren
        timeTask_time_t now;
        timeTask_getTimestamp(&now);
        
        if (timeTask_getDuration(&softStartTime, &now) >= softStartInterval) {
            currentSoftStartPWM += softStartStep;
            
            // Prüfe ob beide Ziel-PWM-Werte erreicht sind
            int16_t minTargetPWM = (targetPWMLeft < targetPWMRight) ? targetPWMLeft : targetPWMRight;
            if (currentSoftStartPWM >= minTargetPWM) {
                // Ziel-PWM erreicht - wechsle zu Phase 1 (Fahren)
                Motor_setPWM(targetPWMLeft, targetPWMRight);
                // Setze Start-Werte neu nach Soft-Start für genauere Messung
                startEncoderR = encoder_getCountR();
                startEncoderL = encoder_getCountL();
                phase = 1;  // Wechsle zu Fahren
                communication_log(LEVEL_INFO, "Soft-Start abgeschlossen, fahre jetzt...");
            } else {
                // Erhöhe PWM weiter
                Motor_setPWM(currentSoftStartPWM, currentSoftStartPWM);
                timeTask_getTimestamp(&softStartTime);
            }
        }
    } else if (phase == 1) {
        // Phase 1: Fahren bis Ziel erreicht
        // Lese aktuelle Encoder-Werte
        int16_t currentEncoderR = encoder_getCountR();
        int16_t currentEncoderL = encoder_getCountL();
        int16_t deltaR = currentEncoderR - startEncoderR;
        int16_t deltaL = currentEncoderL - startEncoderL;
        
        // Verwende absolute Werte für den Fall, dass Encoder negativ sind
        int16_t absDeltaR = (deltaR > 0) ? deltaR : -deltaR;
        int16_t absDeltaL = (deltaL > 0) ? deltaL : -deltaL;
        int16_t maxDelta = (absDeltaR > absDeltaL) ? absDeltaR : absDeltaL;
        
        // Berechne Differenz zwischen den Encodern (wichtig für gerade Fahrt)
        int16_t encoderDiff = deltaR - deltaL;
        int16_t absEncoderDiff = (encoderDiff > 0) ? encoderDiff : -encoderDiff;
        
        // Prüfe ob Ziel erreicht
        if (maxDelta >= targetTicks) {
            Motor_stopAll();
            
            // Berechne gefahrene Distanz in mm
            // Durchschnitt beider Encoder für genauere Distanz
            int16_t avgDelta = (deltaL + deltaR) / 2;
            uint16_t distance_mm = (uint16_t)((int32_t)avgDelta * 688 / 10000);  // avgDelta * 0.0688 mm (2048 Ticks = 1 Rad-Umdrehung)
            
            // Berechne prozentuale Abweichung der Differenz
            int16_t percentDiff = 0;
            if (avgDelta != 0) {
                percentDiff = ((int32_t)encoderDiff * 100) / avgDelta;
            }
            
            // Ausgabe: Klare Anzeige der Encoder-Differenz
            communication_log(LEVEL_INFO, "=== Fahrt abgeschlossen ===");
            communication_log(LEVEL_INFO, "Ziel: 1000 Ticks (~86 mm)");
            communication_log(LEVEL_INFO, "Encoder Links:  %" PRId16 " (Delta: %" PRId16 " Ticks)", currentEncoderL, deltaL);
            communication_log(LEVEL_INFO, "Encoder Rechts: %" PRId16 " (Delta: %" PRId16 " Ticks)", currentEncoderR, deltaR);
            communication_log(LEVEL_INFO, "Encoder-Differenz: %" PRId16 " Ticks (Absolut: %" PRId16 ")", encoderDiff, absEncoderDiff);
            if (avgDelta != 0) {
                communication_log(LEVEL_INFO, "Prozentuale Abweichung: %" PRId16 "%%", percentDiff);
            }
            communication_log(LEVEL_INFO, "Gefahrene Distanz: ~%u mm (Durchschnitt)", distance_mm);
            
            // Bewertung der Kalibrierung
            if (absEncoderDiff <= 2) {
                communication_log(LEVEL_INFO, "Kalibrierung: EXZELLENT (Differenz <= 2 Ticks)");
            } else if (absEncoderDiff <= 5) {
                communication_log(LEVEL_INFO, "Kalibrierung: GUT (Differenz <= 5 Ticks)");
            } else if (absEncoderDiff <= 10) {
                communication_log(LEVEL_INFO, "Kalibrierung: AKZEPTABEL (Differenz <= 10 Ticks)");
            } else {
                communication_log(LEVEL_WARNING, "Kalibrierung: SCHLECHT (Differenz > 10 Ticks) - Neue Kalibrierung empfohlen!");
            }
            communication_log(LEVEL_INFO, "===========================");
            
            setState(IDLE);
            initialized = 0;
        }
    }
}

// Fahre eine bestimmte Anzahl von Encoder-Ticks vorwärts mit kontinuierlicher Korrektur
void drive_Forward_ticks(int16_t targetTicksValue, int16_t pwmRight) {
    /* logs for this function suppressed */
    #define communication_log(level, ...) ((void)0)
    static uint8_t initialized = 0;
    static int16_t startEncoderR = 0;
    static int16_t startEncoderL = 0;
    static int16_t targetPWMLeft = 0;
    static int16_t targetPWMRight = 0;
    
    static uint8_t phase = 0;  // 0 = Soft-Start, 1 = Fahren
    static int16_t currentSoftStartPWM = 0;
    static timeTask_time_t startOfDrive;
    static timeTask_time_t endOfDrive;
    static timeTask_time_t softStartTime;
    static timeTask_time_t waitStart;  // Wartezeit für Betriebsdrehzahl
    static timeTask_time_t speedMeasureStart;  // Startzeit für Geschwindigkeitsmessung
    static int16_t speedMeasureEncoderR = 0;  // Encoder-Werte bei Start der Geschwindigkeitsmessung
    static int16_t speedMeasureEncoderL = 0;
    static uint8_t speedMeasureInitialized = 0;  // Flag ob Geschwindigkeitsmessung initialisiert wurde
    static uint8_t correctionActive = 0;  // Flag ob Korrektur aktiv ist (Hysterese)
    static timeTask_time_t lastLogTime;  // Zeitpunkt der letzten Log-Nachricht
    
    // Konstanten
    const int16_t softStartMinPWM = 1000;  // Start-PWM für Soft-Start
    const int16_t softStartStep = 200;     // PWM-Schrittweite für Soft-Start
    const uint32_t softStartInterval = 50000UL; // 50ms zwischen Soft-Start-Schritten
    const uint32_t baseSpeedWaitTime = 200000UL; // 200ms Basis-Wartezeit nach Soft-Start
    const uint32_t timePerPWM = 50UL; // 0.05ms pro PWM-Einheit (50µs)
    const int16_t minPWMForCalc = 3000; // Referenz-PWM für Berechnung
    const uint32_t speedMeasureInterval = 200000UL; // 200ms für Geschwindigkeitsmessung
    const int16_t minSpeedTicks = 10;  // Mindest-Ticks für gültige Geschwindigkeitsmessung
    
    if (!initialized) {
        timeTask_getTimestamp(&startOfDrive);
        // Setze Start-Encoder-Werte BEVOR Motoren starten
        startEncoderR = encoder_getCountR();
        startEncoderL = encoder_getCountL();
        communication_log(LEVEL_INFO, "Start-Encoder: R=%" PRId16 " L=%" PRId16, startEncoderR, startEncoderL);
        
        // Rechter Motor: Erhält PWM-Wert vom Parameter
        targetPWMRight = pwmRight;
        
        // Linker Motor: Startet mit kalibriertem Wert für den gewünschten PWM-Wert
        targetPWMLeft = calibration_getPWMLeft(pwmRight);
        
        // Sicherheitsprüfung: Falls PWM-Werte 0 oder negativ sind, verwende Standard-Werte
        if (targetPWMLeft == 0 || targetPWMLeft < 0) {
            communication_log(LEVEL_WARNING, "PWM Left ist 0 oder negativ (%" PRId16 "), verwende %" PRId16, targetPWMLeft, pwmRight);
            targetPWMLeft = pwmRight;
        }
        if (targetPWMRight == 0 || targetPWMRight < 0) {
            communication_log(LEVEL_WARNING, "PWM Right ist 0 oder negativ (%" PRId16 "), verwende 3000", targetPWMRight);
            targetPWMRight = 3000;
        }
        
        // Starte mit Soft-Start
        currentSoftStartPWM = softStartMinPWM;
        Motor_setPWM(currentSoftStartPWM, currentSoftStartPWM);
        timeTask_getTimestamp(&softStartTime);
        timeTask_getTimestamp(&waitStart);
        phase = 0;  // Starte mit Soft-Start
        speedMeasureInitialized = 0;
        correctionActive = 0;  // Reset Hysterese
        lastLogTime.time_ms = 0;  // Reset Log-Timer
        lastLogTime.time_us = 0;
        initialized = 1;
        communication_log(LEVEL_INFO, "Fahre %" PRId16 " Ticks mit Soft-Start, Start-PWM L=%" PRId16 " R=%" PRId16 "...", 
                         targetTicksValue, targetPWMLeft, targetPWMRight);
    }
    
    if (phase == 0) {
        // Phase 0: Soft-Start - PWM schrittweise hochfahren
        timeTask_time_t now;
        timeTask_getTimestamp(&now);
        
        // Prüfe ob Ziel bereits erreicht (auch während Soft-Start)
        int16_t currentEncoderR = encoder_getCountR();
        int16_t currentEncoderL = encoder_getCountL();
        int16_t deltaR = currentEncoderR - startEncoderR;
        int16_t deltaL = currentEncoderL - startEncoderL;
        int16_t absDeltaR = (deltaR > 0) ? deltaR : -deltaR;
        int16_t absDeltaL = (deltaL > 0) ? deltaL : -deltaL;
        int16_t maxDelta = (absDeltaR > absDeltaL) ? absDeltaR : absDeltaL;
        
        if (maxDelta >= targetTicksValue) {
            // Ziel bereits erreicht - stoppe sofort
            Motor_stopAll();
            timeTask_getTimestamp(&endOfDrive);
            
            // Berechne gefahrene Distanz in mm
            int16_t avgDelta = (deltaL + deltaR) / 2;
            uint16_t actualDistance_mm = (uint16_t)((int32_t)avgDelta * 688 / 10000);  // avgDelta * 0.0688 mm (2048 Ticks = 1 Rad-Umdrehung)
            
            // Berechne Differenz zwischen den Encodern
            int16_t encoderDiff = deltaR - deltaL;
            int16_t absEncoderDiff = (encoderDiff > 0) ? encoderDiff : -encoderDiff;
            
            // Berechne prozentuale Abweichung der Differenz
            int16_t percentDiff = 0;
            if (avgDelta != 0) {
                percentDiff = ((int32_t)encoderDiff * 100) / avgDelta;
            }
            
            // Ausgabe: Klare Anzeige der Encoder-Differenz
            communication_log(LEVEL_INFO, "=== Fahrt abgeschlossen (während Soft-Start) ===");
            communication_log(LEVEL_INFO, "Ziel: %" PRId16 " Ticks", targetTicksValue);
            communication_log(LEVEL_INFO, "Encoder Links:  %" PRId16 " (Delta: %" PRId16 " Ticks)", currentEncoderL, deltaL);
            communication_log(LEVEL_INFO, "Encoder Rechts: %" PRId16 " (Delta: %" PRId16 " Ticks)", currentEncoderR, deltaR);
            communication_log(LEVEL_INFO, "Encoder-Differenz: %" PRId16 " Ticks (Absolut: %" PRId16 ")", encoderDiff, absEncoderDiff);
            if (avgDelta != 0) {
                communication_log(LEVEL_INFO, "Prozentuale Abweichung: %" PRId16 "%%", percentDiff);
            }
            communication_log(LEVEL_INFO, "Gefahrene Distanz: ~%u mm (Durchschnitt)", actualDistance_mm);
            communication_log(LEVEL_INFO, "Finale PWM-Werte: L=%" PRId16 " R=%" PRId16, targetPWMLeft, targetPWMRight);
            communication_log(LEVEL_INFO, "Gebrauchte Zeit: %" PRId16, timeTask_getDuration(&startOfDrive,&endOfDrive)/1000000);
            
            // Bewertung der Kalibrierung
            if (absEncoderDiff <= 2) {
                communication_log(LEVEL_INFO, "Kalibrierung: EXZELLENT (Differenz <= 2 Ticks)");
            } else if (absEncoderDiff <= 5) {
                communication_log(LEVEL_INFO, "Kalibrierung: GUT (Differenz <= 5 Ticks)");
            } else if (absEncoderDiff <= 10) {
                communication_log(LEVEL_INFO, "Kalibrierung: AKZEPTABEL (Differenz <= 10 Ticks)");
            } else {
                communication_log(LEVEL_WARNING, "Kalibrierung: SCHLECHT (Differenz > 10 Ticks) - Neue Kalibrierung empfohlen!");
            }
            communication_log(LEVEL_INFO, "===========================");
            
            setState(IDLE);
            initialized = 0;
            return;
        }
        
        if (timeTask_getDuration(&softStartTime, &now) >= softStartInterval) {
            currentSoftStartPWM += softStartStep;
            
            // Prüfe ob beide Ziel-PWM-Werte erreicht sind
            int16_t minTargetPWM = (targetPWMLeft < targetPWMRight) ? targetPWMLeft : targetPWMRight;
            if (currentSoftStartPWM >= minTargetPWM) {
                // Ziel-PWM erreicht - wechsle zu Phase 1 (Fahren)
                Motor_setPWM(targetPWMLeft, targetPWMRight);
                // Start-Werte bleiben unverändert - alle Ticks (auch während Soft-Start) werden mitgezählt
                timeTask_getTimestamp(&waitStart);
                phase = 1;  // Wechsle zu Fahren
                speedMeasureInitialized = 0;  // Reset für Geschwindigkeitsmessung
                communication_log(LEVEL_INFO, "Soft-Start abgeschlossen, fahre jetzt mit PWM L=%" PRId16 " R=%" PRId16 "...", 
                                 targetPWMLeft, targetPWMRight);
            } else {
                // Erhöhe PWM weiter
                Motor_setPWM(currentSoftStartPWM, currentSoftStartPWM);
                timeTask_getTimestamp(&softStartTime);
            }
        }
    } else if (phase == 1) {
        // Phase 1: Fahren mit kontinuierlicher Korrektur
        timeTask_time_t now;
        timeTask_getTimestamp(&now);
        
        // Warte erst auf Betriebsdrehzahl (wenn gerade aus Soft-Start gekommen)
        int16_t pwmDiff = (targetPWMRight > minPWMForCalc) ? (targetPWMRight - minPWMForCalc) : 0;
        uint32_t requiredWaitTime = baseSpeedWaitTime + (pwmDiff * timePerPWM);
        
        if (timeTask_getDuration(&waitStart, &now) >= requiredWaitTime) {
            // Betriebsdrehzahl erreicht - setze Encoder-Startwerte für erste Messung
            if (!speedMeasureInitialized) {
                speedMeasureEncoderR = encoder_getCountR();
                speedMeasureEncoderL = encoder_getCountL();
                timeTask_getTimestamp(&speedMeasureStart);
                speedMeasureInitialized = 1;
            }
            
            // Starte Geschwindigkeitsmessung
            if (timeTask_getDuration(&speedMeasureStart, &now) >= speedMeasureInterval) {
                // Messperiode abgelaufen - messe Geschwindigkeit
                int16_t currentEncoderR = encoder_getCountR();
                int16_t currentEncoderL = encoder_getCountL();
                int16_t speedR = currentEncoderR - speedMeasureEncoderR;
                int16_t speedL = currentEncoderL - speedMeasureEncoderL;
                
                // Berechne kumulative Position-Differenz (aufgelaufene Ticks)
                int16_t deltaR = currentEncoderR - startEncoderR;
                int16_t deltaL = currentEncoderL - startEncoderL;
                int16_t positionDiff = deltaR - deltaL;  // Positive = R ist weiter, Negative = L ist weiter
                
                // Prüfe ob genug Bewegung für gültige Messung
                int16_t absSpeedR = (speedR > 0) ? speedR : -speedR;
                int16_t absSpeedL = (speedL > 0) ? speedL : -speedL;
                
                if (absSpeedR >= minSpeedTicks && absSpeedL >= minSpeedTicks) {
                    // Gültige Messung - berechne Differenz
                    int16_t speedDiff = speedR - speedL;
                    int16_t absSpeedDiff = (speedDiff > 0) ? speedDiff : -speedDiff;
                    
                    // Passe linken Motor an (kontinuierliche Korrektur - aggressiv wie beim Kalibrieren)
                    // speedDiff = speedR - speedL
                    // Wenn speedDiff > 0: R ist schneller -> erhöhe pwmLeft
                    // Wenn speedDiff < 0: L ist schneller -> reduziere pwmLeft
                    
                    // Adaptive Anpassung basierend auf Geschwindigkeitsdifferenz UND kumulativer Position-Differenz
                    // Ziel: Differenz zwischen 0-3 Ticks halten, aber Oszillationen vermeiden
                    // Zusätzlich: Kumulative Position-Differenz ausgleichen (übersteuern)
                    int16_t adjustment = 0;
                    int16_t positionAdjustment = 0;
                    
                    // 1. Geschwindigkeitsdifferenz-basierte Korrektur (wie bisher)
                    // Hysterese-Logik: Verhindert schnelles Ein-/Ausschalten der Korrektur
                    if (absSpeedDiff > 6) {
                        // Differenz zu groß - aktiviere Korrektur
                        correctionActive = 1;
                    } else if (absSpeedDiff <= 3) {
                        // Differenz gut - deaktiviere Korrektur (Dead Zone)
                        correctionActive = 0;
                    }
                    // Bei 3 < Diff <= 6: Behalte aktuellen Zustand (correctionActive bleibt unverändert)
                    
                    // Berechne Geschwindigkeits-Anpassung nur wenn Korrektur aktiv ist (konservativer)
                    // WICHTIG: adjustment wird mit Vorzeichen berechnet (positiv/negativ)
                    if (correctionActive) {
                        int16_t adjustmentMagnitude = 0;
                        if (absSpeedDiff > 30) {
                            adjustmentMagnitude = absSpeedDiff / 3; // Sehr große Differenz - konservativer
                            if (adjustmentMagnitude > 15) adjustmentMagnitude = 15; // Max. 15 PWM
                        } else if (absSpeedDiff > 15) {
                            adjustmentMagnitude = absSpeedDiff / 4; // Große Differenz - konservativer
                            if (adjustmentMagnitude > 10) adjustmentMagnitude = 10; // Max. 10 PWM
                        } else if (absSpeedDiff > 6) {
                            adjustmentMagnitude = absSpeedDiff / 3; // Mittlere Differenz - konservativer
                            if (adjustmentMagnitude > 5) adjustmentMagnitude = 5; // Max. 5 PWM
                        } else {
                            // Diff zwischen 3-6: Sehr kleine, gedämpfte Korrektur
                            adjustmentMagnitude = 1; // Nur 1 PWM pro Schritt
                        }
                        
                        // Richtung: speedDiff = speedR - speedL
                        // Wenn speedDiff > 0: R ist schneller -> erhöhe pwmLeft (L schneller machen)
                        // Wenn speedDiff < 0: L ist schneller -> reduziere pwmLeft (L langsamer machen)
                        if (speedDiff > 0) {
                            adjustment = adjustmentMagnitude; // R schneller -> erhöhe pwmLeft
                        } else {
                            adjustment = -adjustmentMagnitude; // L schneller -> reduziere pwmLeft
                        }
                    }
                    
                    // 2. Kumulative Position-Differenz-basierte Korrektur (übersteuern)
                    // Wenn eine Position-Differenz aufgelaufen ist, korrigiere diese zusätzlich
                    // ABER: Sehr konservativ, um Überkorrektur zu vermeiden
                    int16_t absPositionDiff = (positionDiff > 0) ? positionDiff : -positionDiff;
                    if (absPositionDiff > 20) {  // Erhöht von 5 auf 20 - nur bei größeren Differenzen
                        // Signifikante Position-Differenz - korrigiere diese sehr konservativ
                        // Position-Diff wird in Geschwindigkeits-Anpassung umgerechnet
                        // Je größer die Position-Diff, desto stärker die Korrektur (aber gedämpft)
                        if (absPositionDiff > 100) {
                            positionAdjustment = absPositionDiff / 20; // Sehr große Position-Diff - sehr konservativ
                            if (positionAdjustment > 5) positionAdjustment = 5; // Max. 5 PWM
                        } else if (absPositionDiff > 50) {
                            positionAdjustment = absPositionDiff / 15; // Große Position-Diff - konservativ
                            if (positionAdjustment > 4) positionAdjustment = 4; // Max. 4 PWM
                        } else {
                            positionAdjustment = absPositionDiff / 12; // Mittlere Position-Diff - konservativ
                            if (positionAdjustment > 3) positionAdjustment = 3; // Max. 3 PWM
                        }
                        
                        // Richtung: positionDiff = deltaR - deltaL
                        // Wenn positionDiff > 0: R ist weiter -> erhöhe pwmLeft (L schneller machen)
                        // Wenn positionDiff < 0: L ist weiter -> reduziere pwmLeft (L langsamer machen)
                        if (positionDiff < 0) {
                            // L ist weiter (negativ) -> reduziere pwmLeft (langsamer machen)
                            positionAdjustment = -positionAdjustment;
                        }
                        // Wenn positionDiff > 0: R ist weiter -> positionAdjustment bleibt positiv (pwmLeft erhöhen)
                    }
                    
                    // Kombiniere beide Anpassungen (sehr konservativ)
                    int16_t totalAdjustment = adjustment;
                    if (positionAdjustment != 0) {
                        // Position-Korrektur wird nur als kleine Zusatzkorrektur verwendet
                        // Geschwindigkeits-Korrektur hat Vorrang
                        if (absPositionDiff > 50 && absPositionDiff > absSpeedDiff * 2) {
                            // Nur wenn Position-Diff sehr groß ist UND deutlich größer als Speed-Diff
                            // Dann verwende hauptsächlich Position-Korrektur (aber gedämpft)
                            totalAdjustment = positionAdjustment;
                            // Füge kleine Geschwindigkeits-Korrektur hinzu (wenn aktiv)
                            if (adjustment != 0) {
                                totalAdjustment += adjustment / 3; // Sehr gedämpft
                            }
                        } else {
                            // Normalerweise: Geschwindigkeits-Korrektur hat Vorrang
                            totalAdjustment = adjustment;
                            // Füge kleine Position-Korrektur hinzu (sehr gedämpft)
                            totalAdjustment += positionAdjustment / 3; // Sehr gedämpft (1/3 statt 1/2)
                        }
                    }
                    
                    // Maximum-Anpassung begrenzen (nicht zu große Sprünge)
                    int16_t absTotalAdjustment = (totalAdjustment > 0) ? totalAdjustment : -totalAdjustment;
                    if (absTotalAdjustment > 15) {
                        totalAdjustment = (totalAdjustment > 0) ? 15 : -15;
                    }
                    
                    adjustment = totalAdjustment;
                    
                    // Flüssige Anpassung nur des linken Motors
                    if (adjustment != 0) {
                        // adjustment kann jetzt auch negativ sein (durch Position-Korrektur)
                        targetPWMLeft += adjustment;
                    }
                    
                    // Begrenze linken PWM-Wert (nicht zu weit vom Basis-Wert entfernen)
                    int16_t maxDeviation = targetPWMRight / 2; // Max. 50% Abweichung
                    if (targetPWMLeft < targetPWMRight - maxDeviation) targetPWMLeft = targetPWMRight - maxDeviation;
                    if (targetPWMLeft > targetPWMRight + maxDeviation) targetPWMLeft = targetPWMRight + maxDeviation;
                    
                    // Absolute Grenzen: PWM muss zwischen 1000 und 8191 sein
                    if (targetPWMLeft < 1000) targetPWMLeft = 1000;
                    if (targetPWMLeft > 8191) targetPWMLeft = 8191;
                    
                    // Setze neue PWM-Werte direkt (Motoren laufen weiter)
                    Motor_setPWM(targetPWMLeft, targetPWMRight);
                    
                    // Logge alle Messungen regelmäßig (alle 200ms)
                    timeTask_time_t now;
                    timeTask_getTimestamp(&now);
                    uint32_t timeSinceLastLog = 0;
                    if (lastLogTime.time_ms != 0 || lastLogTime.time_us != 0) {
                        timeSinceLastLog = timeTask_getDuration(&lastLogTime, &now);
                    }
                    
                    // Logge immer: Bei jeder Messung (alle 200ms) ODER wenn Korrektur stattfindet
                    if (adjustment != 0 || timeSinceLastLog >= 200000UL || (lastLogTime.time_ms == 0 && lastLogTime.time_us == 0)) {
                        communication_log(LEVEL_INFO, "Korrektur: Speed L=%" PRId16 " R=%" PRId16 " SpeedDiff=%" PRId16 " PosDiff=%" PRId16 " -> PWM L=%" PRId16 " R=%" PRId16 " (Adj=%" PRId16 ")", 
                                         speedL, speedR, speedDiff, positionDiff, targetPWMLeft, targetPWMRight, adjustment);
                        timeTask_getTimestamp(&lastLogTime);
                    }
                    
                    // Starte neue Geschwindigkeitsmessung
                    speedMeasureEncoderR = encoder_getCountR();
                    speedMeasureEncoderL = encoder_getCountL();
                    timeTask_getTimestamp(&speedMeasureStart);
                } else {
                    // Noch nicht genug Bewegung - warte weiter
                    speedMeasureEncoderR = encoder_getCountR();
                    speedMeasureEncoderL = encoder_getCountL();
                    timeTask_getTimestamp(&speedMeasureStart);
                }
            }
        }
        
        // Prüfe ob Ziel erreicht (unabhängig von Korrektur)
        int16_t currentEncoderR = encoder_getCountR();
        int16_t currentEncoderL = encoder_getCountL();
        int16_t deltaR = currentEncoderR - startEncoderR;
        int16_t deltaL = currentEncoderL - startEncoderL;
        
        // Debug: Logge Encoder-Werte regelmäßig (alle 1 Sekunde)
        static timeTask_time_t lastEncoderLogTime;
        static uint8_t encoderLogInitialized = 0;
        if (!encoderLogInitialized) {
            timeTask_getTimestamp(&lastEncoderLogTime);
            encoderLogInitialized = 1;
        }
        if (timeTask_getDuration(&lastEncoderLogTime, &now) >= 1000000UL) { // Alle 1 Sekunde
            communication_log(LEVEL_INFO, "Encoder Status: R=%" PRId16 " (Delta: %" PRId16 ") L=%" PRId16 " (Delta: %" PRId16 ") Start: R=%" PRId16 " L=%" PRId16, 
                             currentEncoderR, deltaR, currentEncoderL, deltaL, startEncoderR, startEncoderL);
            timeTask_getTimestamp(&lastEncoderLogTime);
        }
        
        // Verwende absolute Werte für den Fall, dass Encoder negativ sind
        int16_t absDeltaR = (deltaR > 0) ? deltaR : -deltaR;
        int16_t absDeltaL = (deltaL > 0) ? deltaL : -deltaL;
        int16_t maxDelta = (absDeltaR > absDeltaL) ? absDeltaR : absDeltaL;
        
        // Prüfe ob Ziel erreicht (mindestens einer der Encoder hat targetTicksValue erreicht)
        if (maxDelta >= targetTicksValue) {
            Motor_stopAll();
            timeTask_getTimestamp(&endOfDrive);
            
            // Berechne gefahrene Distanz in mm
            int16_t avgDelta = (deltaL + deltaR) / 2;
            uint16_t actualDistance_mm = (uint16_t)((int32_t)avgDelta * 688 / 10000);  // avgDelta * 0.0688 mm (2048 Ticks = 1 Rad-Umdrehung)
            
            // Berechne Differenz zwischen den Encodern
            int16_t encoderDiff = deltaR - deltaL;
            int16_t absEncoderDiff = (encoderDiff > 0) ? encoderDiff : -encoderDiff;
            
            // Berechne prozentuale Abweichung der Differenz
            int16_t percentDiff = 0;
            if (avgDelta != 0) {
                percentDiff = ((int32_t)encoderDiff * 100) / avgDelta;
            }
            
            // Ausgabe: Klare Anzeige der Encoder-Differenz
            communication_log(LEVEL_INFO, "=== Fahrt abgeschlossen ===");
            communication_log(LEVEL_INFO, "Ziel: %" PRId16 " Ticks", targetTicksValue);
            communication_log(LEVEL_INFO, "Encoder Links:  %" PRId16 " (Delta: %" PRId16 " Ticks)", currentEncoderL, deltaL);
            communication_log(LEVEL_INFO, "Encoder Rechts: %" PRId16 " (Delta: %" PRId16 " Ticks)", currentEncoderR, deltaR);
            communication_log(LEVEL_INFO, "Encoder-Differenz: %" PRId16 " Ticks (Absolut: %" PRId16 ")", encoderDiff, absEncoderDiff);
            if (avgDelta != 0) {
                communication_log(LEVEL_INFO, "Prozentuale Abweichung: %" PRId16 "%%", percentDiff);
            }
            communication_log(LEVEL_INFO, "Gefahrene Distanz: ~%u mm (Durchschnitt)", actualDistance_mm);
            communication_log(LEVEL_INFO, "Finale PWM-Werte: L=%" PRId16 " R=%" PRId16, targetPWMLeft, targetPWMRight);
            communication_log(LEVEL_INFO, "Gebrauchte Zeit: %" PRId16, timeTask_getDuration(&startOfDrive,&endOfDrive)/1000000);
            
            // Bewertung der Kalibrierung
            if (absEncoderDiff <= 2) {
                communication_log(LEVEL_INFO, "Kalibrierung: EXZELLENT (Differenz <= 2 Ticks)");
            } else if (absEncoderDiff <= 5) {
                communication_log(LEVEL_INFO, "Kalibrierung: GUT (Differenz <= 5 Ticks)");
            } else if (absEncoderDiff <= 10) {
                communication_log(LEVEL_INFO, "Kalibrierung: AKZEPTABEL (Differenz <= 10 Ticks)");
            } else {
                communication_log(LEVEL_WARNING, "Kalibrierung: SCHLECHT (Differenz > 10 Ticks) - Neue Kalibrierung empfohlen!");
            }
            communication_log(LEVEL_INFO, "===========================");
            
            setState(IDLE);
            initialized = 0;
        }
    }
#undef communication_log
}

void drive_Forward_5sec() {
    static uint8_t initialized = 0;
    timeTask_time_t now;
    
    if (!initialized) {
        timeTask_getTimestamp(&start);
        Motor_setPWM(3000, 3000);
        initialized = 1;
    }
    
    timeTask_getTimestamp(&now);
    if (timeTask_getDuration(&start, &now) > 5000000UL) {  // 5 seconds in microseconds
        setState(IDLE);
        initialized = 0;
    }
}

void setState(state newState) {
    currentState = newState;
}

// Setze Ziel-Distanz für Drive_Forward_Distance
void statemachine_setTargetDistance(uint16_t distance_mm) {
    targetDistance_mm = distance_mm;
}

// Setze Ziel-Ticks für Drive_Forward_Ticks
void statemachine_setTargetTicks(int16_t ticks) {
    targetTicks = ticks;
}

// Setze PWM-Wert für rechten Motor
void statemachine_setTargetPWM(int16_t pwm) {
    targetPWM = pwm;
}

// Fahre eine bestimmte Distanz in mm vorwärts (Wrapper-Funktion)
// Rechnet mm in Ticks um und ruft drive_Forward_ticks auf
void drive_Forward_distance_mm(uint16_t distance_mm, int16_t pwmRight) {
    // 1 Encoder-Tick = 0.0688 mm (2048 Ticks = 1 Rad-Umdrehung)
    // Umrechnung: distance_mm / 0.0688 mm = targetTicks
    // 1/0.0688 ≈ 14.5349, verwende 14535/1000 für bessere Genauigkeit
    int16_t targetTicks = (int16_t)((int32_t)distance_mm * 14535 / 1000);  // distance_mm / 0.0688 mm ≈ targetTicks
    
    // Rufe die tick-basierte Funktion mit dem übergebenen PWM-Wert für rechten Motor auf
    drive_Forward_ticks(targetTicks, pwmRight);
}

// Setze Ziel-Winkel für Drehung auf der Stelle
void statemachine_setTargetAngle(int16_t angle_degrees) {
    targetAngle_degrees = angle_degrees;
}

/*Drehe den Roboter um einen bestimmten Winkel auf der Stelle
  angle_degrees: Winkel in Grad (positiv = rechts drehen, negativ = links drehen)
  pwm: PWM-Wert für beide Motoren (absolut)*/
void turn_On_Spot_degrees(int16_t angle_degrees, int16_t pwm) {
    /* logs for this function suppressed */
    #define communication_log(level, ...) ((void)0)
    static uint8_t initialized = 0;
    static int16_t startEncoderR = 0;
    static int16_t startEncoderL = 0;
    static int16_t targetTicksValue = 0;
    static int16_t pwmLeft = 0;
    static int16_t pwmRight = 0;
    
    static uint8_t phase = 0;  // 0 = Soft-Start, 1 = Drehen
    static int16_t currentSoftStartPWM = 0;
    static timeTask_time_t softStartTime;
    static timeTask_time_t waitStart;
    
    // Konstanten
    const int16_t softStartMinPWM = 1000;  // Start-PWM für Soft-Start
    const int16_t softStartStep = 200;     // PWM-Schrittweite für Soft-Start
    const uint32_t softStartInterval = 50000UL; // 50ms zwischen Soft-Start-Schritten
    const uint32_t baseSpeedWaitTime = 200000UL; // 200ms Basis-Wartezeit nach Soft-Start
    const uint32_t timePerPWM = 50UL; // 0.05ms pro PWM-Einheit (50µs)
    const int16_t minPWMForCalc = 3000; // Referenz-PWM für Berechnung
    
    // Physikalische Konstanten
    const float wheelbase_mm = 166.2f;  // Kalibrierter Radabstand (90deg Code -> 90deg real)
    const float mmPerTick = 0.0688f;    // mm pro Tick
    const float pi = 3.14159265359f;
    
    if (!initialized) {
        // Setze Start-Encoder-Werte BEVOR Motoren starten
        startEncoderR = encoder_getCountR();
        startEncoderL = encoder_getCountL();
        
        // Berechne benötigte Ticks für den Winkel
        // Umfang der Drehbewegung = π × Radabstand
        // Distanz pro Rad = (π × Radabstand × Winkel) / 360°
int16_t absAngle= (angle_degrees > 0) ? angle_degrees : -angle_degrees;
        float distancePerWheel_mm = (pi * wheelbase_mm * (float)absAngle ) / 360.0f;
        targetTicksValue = (int16_t)(distancePerWheel_mm / mmPerTick);
        
        // Bestimme Drehrichtung: positiv = links drehen (L vorwärts, R rückwärts)
        int16_t absPWM = (pwm > 0) ? pwm : -pwm;
        if (angle_degrees > 0) {
            // Links drehen: L vorwärts (+), R rückwärts (-)
            pwmLeft = absPWM;
            pwmRight = -absPWM;
        } else {
            // Rechts drehen: L rückwärts (-), R vorwärts (+)
            pwmLeft = -absPWM;
            pwmRight = absPWM;
        }
        
        // Sicherheitsprüfung
        if (absPWM <= 0) {
            communication_log(LEVEL_WARNING, "PWM ist 0 oder negativ (%" PRId16 "), verwende 3000", pwm);
            absPWM = 3000;
            if (angle_degrees > 0) {
                pwmLeft = 3000;
                pwmRight = -3000;
            } else {
                pwmLeft = -3000;
                pwmRight = 3000;
            }
        }
        
        // Starte mit Soft-Start
        currentSoftStartPWM = softStartMinPWM;
        Motor_setPWM(pwmLeft, pwmRight);
        timeTask_getTimestamp(&softStartTime);
        timeTask_getTimestamp(&waitStart);
        phase = 0;  // Starte mit Soft-Start
        initialized = 1;
        communication_log(LEVEL_INFO, "Drehe um %" PRId16 "° auf der Stelle, benötigt %" PRId16 " Ticks pro Rad, PWM=%" PRId16 "...", 
                         angle_degrees, targetTicksValue, absPWM);
    }
    
    if (phase == 0) {
        // Phase 0: Soft-Start - PWM schrittweise hochfahren
        timeTask_time_t now;
        timeTask_getTimestamp(&now);
        
        // Prüfe ob Ziel bereits erreicht (auch während Soft-Start)
        int16_t currentEncoderR = encoder_getCountR();
        int16_t currentEncoderL = encoder_getCountL();
        int16_t deltaR = currentEncoderR - startEncoderR;
        int16_t deltaL = currentEncoderL - startEncoderL;
        // Für Drehung: Beide Deltas sollten gleich groß sein (aber unterschiedliche Vorzeichen)
        int16_t absDeltaR = (deltaR > 0) ? deltaR : -deltaR;
        int16_t absDeltaL = (deltaL > 0) ? deltaL : -deltaL;
        int16_t maxDelta = (absDeltaR > absDeltaL) ? absDeltaR : absDeltaL;
        
        if (maxDelta >= targetTicksValue) {
            // Ziel bereits erreicht - stoppe sofort
            Motor_stopAll();
            
            communication_log(LEVEL_INFO, "=== Drehung abgeschlossen (während Soft-Start) ===");
            communication_log(LEVEL_INFO, "Ziel: %" PRId16 "° (%" PRId16 " Ticks pro Rad)", targetAngle_degrees, targetTicksValue);
            communication_log(LEVEL_INFO, "Encoder Links:  %" PRId16 " (Delta: %" PRId16 " Ticks)", currentEncoderL, deltaL);
            communication_log(LEVEL_INFO, "Encoder Rechts: %" PRId16 " (Delta: %" PRId16 " Ticks)", currentEncoderR, deltaR);
            communication_log(LEVEL_INFO, "===========================");
            
            setState(IDLE);
            initialized = 0;
            return;
        }
        
        if (timeTask_getDuration(&softStartTime, &now) >= softStartInterval) {
            currentSoftStartPWM += softStartStep;
            
            int16_t absPWM = (pwmLeft > 0) ? pwmLeft : -pwmLeft;
            if (currentSoftStartPWM >= absPWM) {
                // Ziel-PWM erreicht - wechsle zu Phase 1 (Drehen)
                Motor_setPWM(pwmLeft, pwmRight);
                timeTask_getTimestamp(&waitStart);
                phase = 1;  // Wechsle zu Drehen
                communication_log(LEVEL_INFO, "Soft-Start abgeschlossen, drehe jetzt mit PWM L=%" PRId16 " R=%" PRId16 "...", 
                                 pwmLeft, pwmRight);
            } else {
                // Erhöhe PWM weiter (beide Motoren in entgegengesetzte Richtung)
                if (pwmLeft > 0) {
                    Motor_setPWM(currentSoftStartPWM, -currentSoftStartPWM);
                } else {
                    Motor_setPWM(-currentSoftStartPWM, currentSoftStartPWM);
                }
                timeTask_getTimestamp(&softStartTime);
            }
        }
    } else if (phase == 1) {
        // Phase 1: Drehen bis Ziel erreicht
        timeTask_time_t now;
        timeTask_getTimestamp(&now);
        
        // Warte auf Betriebsdrehzahl
        int16_t absPWM = (pwmLeft > 0) ? pwmLeft : -pwmLeft;
        int16_t pwmDiff = (absPWM > minPWMForCalc) ? (absPWM - minPWMForCalc) : 0;
        uint32_t requiredWaitTime = baseSpeedWaitTime + (pwmDiff * timePerPWM);
        
        if (timeTask_getDuration(&waitStart, &now) >= requiredWaitTime) {
            // Betriebsdrehzahl erreicht - prüfe ob Ziel erreicht
            int16_t currentEncoderR = encoder_getCountR();
            int16_t currentEncoderL = encoder_getCountL();
            int16_t deltaR = currentEncoderR - startEncoderR;
            int16_t deltaL = currentEncoderL - startEncoderL;
            int16_t absDeltaR = (deltaR > 0) ? deltaR : -deltaR;
            int16_t absDeltaL = (deltaL > 0) ? deltaL : -deltaL;
            int16_t maxDelta = (absDeltaR > absDeltaL) ? absDeltaR : absDeltaL;
            
            // Prüfe ob Ziel erreicht
            if (maxDelta >= targetTicksValue) {
                Motor_stopAll();
                
                communication_log(LEVEL_INFO, "=== Drehung abgeschlossen ===");
                communication_log(LEVEL_INFO, "Ziel: %" PRId16 "° (%" PRId16 " Ticks pro Rad)", targetAngle_degrees, targetTicksValue);
                communication_log(LEVEL_INFO, "Encoder Links:  %" PRId16 " (Delta: %" PRId16 " Ticks)", currentEncoderL, deltaL);
                communication_log(LEVEL_INFO, "Encoder Rechts: %" PRId16 " (Delta: %" PRId16 " Ticks)", currentEncoderR, deltaR);
                communication_log(LEVEL_INFO, "Finale PWM-Werte: L=%" PRId16 " R=%" PRId16, pwmLeft, pwmRight);
                communication_log(LEVEL_INFO, "===========================");
                
                setState(IDLE);
                initialized = 0;
            }
        }
    }
#undef communication_log
}
