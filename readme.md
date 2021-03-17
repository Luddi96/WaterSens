Universaler Zisternen-Füllstandsmesser

Einleitung:

Jeder Besitzer einer Regenzisterne kennt diese Frage: Wie viel Wasser verbleibt mir noch und wie viel hat der letzte Regen wieder aufgefüllt? Um Abhilfe zu schaffen gibt es eine Reihe an Messgeräten und Sensoren: Ganz simpel mit mehren Elektroden auf verschiedenen Höhen, kapazitiv unter Zuhilfenahme der Dielektrizitätskonstante des Wassers selbst, per Ultraschall oder per Druckmessung. Einige dieser Messungen haben Vorteile gegenüber anderen. In diesem Beitrag möchte ich Ihnen eine Methode zur Messung vorstellen, welche weder durch Temperatur, Luftdruck oder Leitfähigkeit des Wassers verfälscht wird und außerdem selbstreinigend (wartungsfrei) agiert. Die Messgenauigkeit liegt bei umgerechnet 20 Liter in einer 7500 Liter Zisterne, also etwa 0,3%!
Das Projekt involviert die Programmierung eines ATMega Mikrocontrollers, das Löten von SMD Bauteilen (es könnte aber auch als THT aufgebaut werden) sowie grundlegende mechanische Fähigkeiten.
Das Funktionsprinzip:
Wie funktioniert der Sensor? Im Prinzip ganz einfach: Man misst den Druck, den das Wasser an der tiefsten Stelle der Zisterne erzeugt. Dieser ist proportional zur Höhe des Wassers und somit (bei einer gleichförmigen Zisterne) ebenfalls proportional zum Füllstand. Würde man aber einfach einen Sensor auf den Grund der Zisterne herablassen, so stößt man auf 3 Probleme: 1. Solche Sensoren sind recht teuer (müssen ja auch wasserdicht sein) 2. Es gibt keine Möglichkeit, den Sensor periodisch zu nullen 3. Der Luftdruck beeinflusst das Ergebnis: 10hPa Druckunterschied bedeuten bereits 100mm Wassersäule oder einen Messfehler von etwa 750 Liter in bzw. 10% in einer 7500 Liter Zisterne.
Mit einem Trick können aber alle der eben genannten Einflüsse kompensiert werden: In die Zisterne wird lediglich ein dünner Schlauch mit einem Gewicht am offenen Ende eingeführt und bis zum Boden herabgelassen. In das andere Ende wird mit einer kleinen Luftpumpe so lange Luft gepumpt, bis dieser komplett gefüllt ist und erste Luftblasen austreten. Der Druck im Schlauch entspricht dann genau dem Druck am Boden der Zisterne. Mit einem empfindlichen Drucksensor kann dieser dann ausgelesen werden. Wenn der Schlauch vor und nach der Messung mittels einem Magnetventil entleert wird, können Umgebungsdruck und Langzeit-Drift des Drucksensor eliminiert werden. Ebenso ist das System durch das "Ausspülen" des Schlauches bei jeder Messung selbstreinigend.

Komponenten:

1x Miniatur-Luftpumpe JQB031 oder ähnlich https://bit.ly/2ENsoDM
2x 3-Wege Magnetventil DQF2-6A-4 von dyxminipump https://bit.ly/2G6OzFo
1x Schlauch-Verbinder-Sortiment https://bit.ly/31LYU29
1x 50cm Silikonschlauch (2mm ID) https://bit.ly/2QGRM0I
1x 5m Silikonschlauch (4mm ID) https://bit.ly/34PzMJN
1x MPRLS Sensor-Modul https://www.adafruit.com/product/3965
1x Funkmodul, z.B. NRF24L01
1x Hauptleiterplatte, BOM als ZIP im Anhang
1x LiPo Einzelzelle 1000mAh https://bit.ly/2EHX8WZ

Aufbau:

Die Ansteuerung der Pumpe und Magnetventile sowie das Auslesen und Auswerten des Drucksensor und das anschließende Versenden der Daten per Funk übernimmt die Hauptleiterplatte. Auf dieser befindet sich ein ATMega328P-AU, geflasht mit dem Arudino Nano Bootloader. Zur Ansteuerung der Aktoren genügt ein IRF7103. Da der Sensor mehrere Jahre ohne Batteriewechsel auskommen soll, wird zur Spannungsversorgung ein TPS7A05 LDO mit einer Ausgangsspannung von 3,3V und einem Eigenstromverbrauch von nur 1uA verbaut. Auch der Spannungsteiler zur Messung der Batteriespannung wird über einen Optokoppler geschalten. So kommt das Gesamtsystem auf 6uA Sleep-Strom und bis zu 2 Jahre Lebensdauer bei 2 Messungen am Tag.
Die Pumpe sowie Magnetventile werden wie folgt verbunden:

Mit dem ersten Magnetventil kann der Druck aus dem System abgelassen werden, das andere Magnetventil schneidet "Messschlauch" und Drucksensor vom Rest ab, um einen Druckabfall während der Messung zu verhindern.
Das ganze wird auf einem 3D-Druck-Träger (.stl in der zip-Datei enthalten) in ein Modulgehäuse wasserdicht eingebaut. Der Druckausgleich wird mit einem Druckausgleichselement erreicht.
In der Software sieht der Ablauf wie folgt aus:
Initialisierung: Ports, Variablen, etc. initialisieren, Sensorwert auf 0 setzen
Start einer Messung:
-Druck im offenen System 3x messen, Wert als "before" speichern
-Magnetventil 1 schließen, Pumpe aktivieren
-Nach bestimmter Zeit Magnetventil 2 schließen, Pumpe abschalten und Magnetventil 1 wieder öffnen (der Schlauch steht jetzt unter Druck)
-Druck 3x messen, Wert als "high" speichern
-Magnetventil 2 öffnen und Druck entweichen lassen
-Druck 3x messen und als "after" speichern
-Aus before, after und high die Druckdifferenz berechnen, das ist das Ergebnis.
Eine solche Messung dauert etwa 8 Sekunden.
Fazit und Ergebnisse:
Ich habe das System nun schon seit 2 Monaten in Betrieb. Um die Haltbarkeit zu testen habe ich den Messintervall auf 60 Sekunden verringert und insgesamt hat das System bereits über 25.000 Messungen durchgeführt - ohne Probleme. Die Standardabweichung der Messung liegt bei etwa 2,5mm Wassersäule oder 20 Liter bei einer 7500 Liter Zisterne. Mir gefällt das System sehr gut. Ich habe bereits ein weiteres System gebaut, welches bei einem Bekannten eingebaut wird. Dort ersetzen wir das Funkmodul durch ein LAN-Modul, das System wird zudem dauerhaft mit Strom versorgt.
