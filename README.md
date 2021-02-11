# balise_dgac_web_Mavlink
Drone identification beacon according to french law modified to work via Mavlink

Libraries files included.

"
Mon travail est lui-même basé et largement inspiré sur une autre version de cette même balise qui comporte un gps BN220 et une carte D1 mini.
Page d’explication ici : https://www.tranquille-informatique.fr/modelisme/divers/balise-dgac-signalement-electronique-a-distance-drone-aeromodelisme.html
Pour les bidouilleurs le fil complet ici (je vous préviens c’est long): https://discuss.ardupilot.org/t/open-source-french-drone-identification/56904/

La différence avec mon système? Je vole en pixhawk et ce gps vient en redondance avec celui branché sur le pixhawk. J’ai donc entrepris de récupérer toutes les informations via mavlink pour me passé de ce BN220 ce qui au passage économise 10€ et quelques grammes dont je souhaitais me passer et en faire bénéficier autrui.
"

Pour installer les cartes dans Arduino IDE, collez ceci dans fichiers/préférences champs gestionnaire de cartes:
http://arduino.esp8266.com/stable/package_esp8266com_index.json,https://dl.espressif.com/dl/package_esp32_index.json

[11/02/2021] Version "CAM" ajoutée, permet le pilotage d'une caméra xiaomi YI (première version mais fonctionne probablement avec les suivantes), canal RC n°9 PWM<1300 rien ou arrêt d'enregistrement 1300<PWM<1700 = photo (si pwm continue photo tout le 4s environ), si 1700<PWM<2200 enregistrement vidéo.
Pour un usage en mission avec déclenchement automatique, passer servo7 à la valeur 10 (camera trigger) dans mission planner, réglez dans la partie gimbal la PWM de base à 1100, trigger à 1350 et la durée à 2 (200ms).
