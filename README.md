# Auxiliaire
Indicateur de télescope parqué

Indique au dome que le télescope est bien positionné.

Fonctionnement:
La broche "Park" passe à l'état haut et la LED s'allume quand le télescope est en position parquée.
Ainsi, l'abri roulant peut se fermer.

Calibrage:
1. Positionner le télescope dans sa position de park
2. Maintenir appuyé le bouton. La LED s'éclaire en blanc, puis en orange.
3. Relacher le bouton. Le clibrage est effectué.

Relacher le bouton avant que la LED passe en orange annule le calibrage. Maintenir le bouton appuyé quand la LED est orange annule aussi le calibrage.

Réglage des offsets 
1. Ouvrir le boitier
2. Placer le télescope en position horizontale (à l'aide d'un niveau)
3. appuyer sur le bouton intérieur.

Le calibrage des offsets permet d'améliorer la précision de la position "home"

Il faut refaire un calibrage de la position de park après avoir calibré les offsets.

Entrées/Sorties:
- DIN 5 broches:
	1. GND
	2. 5V
	3. Park
	4. (Limit) Pas encore implémenté
- Bouton poussoir:
	Calibrage de la position de park
- Bouton poussoir interne:
	Calibrage des offsets X, Y
- LED RGB:
	* Vert: 	axe X parqué (précision 0,5°)
	* Rouge: 	axe Y parqué (précision 0,5°)
	* Orange: axes XY parqués (précision 0,5°)
	* Bleu:	télescope parqué (précision 5°)

	* Rouge: Position home (vise le pole céleste) (précision 2°)

	* Blanc: Après appui long sur le bouton
	* Jaune: Position de park en cours de validation
	* Rouge clignotant: Position de park validée


## Broches sur le Wemos:
* D0:	Télescope parqué (sortie vers l'abri roulant)
* D1/D2: 	I2c (magnétomètre)
* D3:	Bouton de calibrage du park.
* D4:	Bouton de calibrage des offsets
* D5: 	LED rouge
* D6:	LED verte
* D7:	LED bleue
* D8:	Limites (pas encore implémenté)
