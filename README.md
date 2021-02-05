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

