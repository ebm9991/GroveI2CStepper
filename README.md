# GroveI2CStepper

Bibliotheque Arduino non bloquante pour commander un moteur pas a pas bipolaire 4 fils avec le **Grove - Mini I2C Motor Driver v1.1 (Seeed 105020010)**, qui contient **2 x DRV8830**.

Elle est destinee notamment a l'**Arduino UNO R4 Minima** et n'utilise que l'API Arduino standard `Wire`.

## Materiel vise

- Arduino UNO R4 Minima (compatible aussi avec d'autres cartes Arduino)
- Grove - Mini I2C Motor Driver v1.1
- Moteur pas a pas bipolaire 4 fils, par exemple NEMA17
- CH1 du Grove -> une bobine
- CH2 du Grove -> l'autre bobine

Adresses par defaut en notation Arduino/Wire 7 bits :

- CH1 : `0x65`
- CH2 : `0x60`

Ces adresses ont ete confirmees par scan I2C sur le Grove Mini I2C Motor Driver v1.1.

Pour une ancienne carte v1.0 utilisant `0x62` et `0x60`, les adresses peuvent toujours etre passees explicitement au constructeur :

```cpp
GroveI2CStepper motor(200, 0x62, 0x60);
```

## Installation

Copier le dossier `GroveI2CStepper` dans le dossier `libraries` d'Arduino, ou installer le fichier ZIP depuis **Croquis > Inclure une bibliotheque > Ajouter la bibliotheque .ZIP**.

## Exemple minimal

```cpp
#include <GroveI2CStepper.h>

GroveI2CStepper motor(200);

void setup() {
  motor.begin();
  motor.setDriveLevel(30);
  motor.setMaxSpeed(100);
  motor.setSpeed(30);
}

void loop() {
  motor.runSpeed();
}
```

## API principale

```cpp
motor.begin();
motor.setDriveLevel(30);  // VSET DRV8830: 0..63
motor.setMaxSpeed(100);   // pas/s
motor.setSpeed(+50);      // pas/s, sens positif
motor.setSpeed(-50);      // pas/s, sens inverse
motor.runSpeed();         // non bloquant, a appeler continuellement
motor.stop();             // arret, maintien de la phase actuelle
motor.release();          // bobines desactivees
motor.hold();             // maintien de la position actuelle
```

Pour un NEMA17 1,8 deg/pas : `200` pas/tour.

## Utilisation avec un PID

La sortie du PID peut directement etre interpretee comme une vitesse signee :

```cpp
float erreur = consigne - angleMesure;
float commande = calculPID(erreur);

motor.setSpeed(commande);
motor.runSpeed();
```

La bibliotheque masque la sequence des deux bobines et les deux DRV8830. Le programme principal reste donc centre sur la mesure et le PID.

## Remarques importantes

- Le Grove Mini I2C Motor Driver v1.1 limite le courant a environ 200 mA par canal dans sa configuration d'origine.
- Pour un NEMA17 nominalement prevu pour 1 A/phase, le couple disponible sera donc nettement reduit.
- Commencer a faible vitesse (par exemple 10 a 30 pas/s) et avec une charge mecanique legere.
- `runSpeed()` est non bloquant, mais doit etre appele aussi souvent que possible dans `loop()`.
- Cette version utilise le pas entier deux phases alimentees (4 etats).

## Version 1.0.2

Fixes speed-change scheduling for closed-loop/PID use. When the commanded speed increases after a very low non-zero speed, the next step deadline is now rescheduled so an old long interval cannot delay the motor response. Direction reversals are also rescheduled immediately.
