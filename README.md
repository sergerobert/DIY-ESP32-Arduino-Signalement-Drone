# DIY-ESP32---Dispositif-de-signalement-Drone-
Do it yourself : 

Ce dépôt décrit comment réaliser soi-même un dispositif de signalement électronique conforme à la réglementation (voir les textes ci-après).
Décret n° 2019-1114 du 30 octobre 2019 pris pour l'application de l'article L. 34-9-2 du code des postes et des communications électroniques - Légifrance (legifrance.gouv.fr)
Légifrance - Publications officielles - Journal officiel - JORF n° 0302 du 29/12/2019 (legifrance.gouv.fr)

https://www.legifrance.gouv.fr/codes/article_lc/LEGIARTI000039307454/2020-05-01

Il s’agit d’évoquer ici des concepts et des éléments (codes, bibliothèques,…) qui ont été publiés précédemment (notamment ceux des revues Helico Micro, Modèle Magazine) avec comme particularité, de donner la possibilité de fabriquer son propre module de signalement électronique pour sa propre flotte de drones.
La DSAC a attribué un trigramme « 000 » pour ceux qui fabriquent eux-mêmes leurs dispositifs (Le trigramme « 000 » ne peux pas être utilisé si vous souhaitez commercialiser le dispositif).

[Extrait des informations de la DSAC Réf. : 21-001 DSAC/NO/NAV]
Utilisation du code « 000 » comme trigramme constructeur
L’utilisateur qui utilise le code « 000 » comme trigramme constructeur doit compléter l’identifiant par :
-	Un code sur 3 caractères, censé représenter le modèle de la balise ;
-	Un code sur 24 caractères, censé représenter le numéro de série de la balise.
Recommandations :
- Pour le code « modèle » : utilisation d’un trigramme de 3 lettres majuscules, au choix de l’utilisateur 
- Pour le code « numéro de série » : numéro d’ordre* croissant1, 2, 3.... (Complété par des 0 à gauche de sorte que le nombre comporte 24 caractères au total)
* dans le cas où l’utilisateur possède plusieurs balises pour son usage personnel
Exemples d’identifiants électroniques valides (les espaces ne sont représentés qu’à des fins de lisibilité et ne doivent pas être saisis dans AlphaTango) :
-	000 NMR 000 000 000 000 000 000 000 001
-	000 BPN 000 000 000 000 000 000 000 017

Description du dispositif :
Le dispositif est composé de deux parties, une carte ESP32 (Lolin32 par exemple) avec un micrologiciel pour être programmé dans un environnement Arduino (Arduino - Home). 
Les différentes bibliothèques utilisées devront être installées dans l’environnement (1.8.13) IDE Arduino.

Pour l'alimentation sur le Drone deux possibilités, utiliser un connecteur micro USB pour utiliser le régulateur embarqué ou utiliser une batterie Lipo qui permettra un fonctionement autonome (surtout si vous souhaitez faire des test).

Masse : 20 grammes pour le carte ESP32 et le recepteur GPS cablés + 20 grammes pour une batterie Lipo 3,7v. 

Carte ESP32 utilisée (coût environ 11 Euros)
ESP32 Carte de développement Module double WiFi Bluetooth ESP WROOM Lolin32 V1.0.0 WIFI pour Arduino: Amazon.fr

Carte récepteur GPS utilisée (environ 6 euros)
Carte de développement de Module de positionnement par Satellite GPS Neo 6m NEO 7M 7M NEO 8M pour microcontrôleur Arduino STM32 C51 51 MCU | AliExpress

Pour tester le dispostif, il est possible d'utiliser le programme (écrit en Python) qui est proposé par la Gendarmerie Nationale.
https://github.com/GendarmerieNationale/ReceptionInfoDrone

La Bibliothéque C++ "droneID_FR.h" de référence est déposée sur ce lien https://github.com/khancyr/droneID_FR


