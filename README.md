# Auxiliaire
Indicateur de télescope parqué

Indique au dome que le télescope est bien positionné.

Fonctionnement:
La broche "Park" passe à l'état haut et la LED s'allume quand le télescope est en position parquée.
Ainsi, l'abri roulant peut se fermer.

Calibrage:
1. Positionner le télescope dans sa position de park
2. Maintenir appuyé le bouton. La LED s'éclaire en bleu, puis en orange.
3. Relacher le bouton. Le calibrage est effectué. La LED clignote en rouge.

Relacher le bouton avant que la LED passe en orange annule le calibrage. Maintenir le bouton appuyé jusqu'à ce que la LED repasse en bleu annule aussi le calibrage.

Réglage des offsets 
1. Placer le télescope en position horizontale (à l'aide d'un niveau)
2. appuyer sur le bouton jusqu'à ce que la LED passe en rouge. La LED clignote en rouge.

Le calibrage des offsets permet d'améliorer la précision de la position "home"

Il faut refaire un calibrage de la position de park après avoir calibré les offsets.

Entrées/Sorties:
- DIN 5 broches:
	1. GND
	2. 5V
	3. Park
	4. Limites
- Bouton poussoir:
	Calibrage de la position de park et des offsets
- LED RGB (APA106):
	* Vert: 	axe X parqué (précision 0,5°)
	* Rouge: 	axe Y parqué (précision 0,5°)
	* Orange: axes XY parqués (précision 0,5°)
	* Bleu:	télescope parqué (précision 3°)

	* Orange: Position home (vise le pole céleste) (précision 2°)

	* Bleu: Après appui long sur le bouton
	* Jaune: Position de park en cours de validation
	* Rouge clignotant: Position de park validée


## Broches sur le Wemos:
* D3:	Télescope parqué (sortie vers l'abri roulant)
* D1/D2: 	I2c (magnétomètre)
* D6:	Bouton de calibrage du park.
* D7:	LED (APA106)
* D4:	Limites (Sortie vers OnStep)

## Matériel:
* Wemos mini D1 pro
* MPU9250 (accéléromètre, gyroscope, magnétomètre)
* 1 bouton pousoir
* 1 LED RVB APA106
* 1 boitier
