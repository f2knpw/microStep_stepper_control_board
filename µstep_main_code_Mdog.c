#include "pic16f5x.h"
#include <pic.h>
__CONFIG(HS & WDTDIS & UNPROTECT);

#define STEP RA1	//Entrée Step
#define DIR RA0		//Entrée Dir
#define J1 RA3		//Entrée Jumper 1
#define J2 RA2		//Entrée Jumper 2
#define DAC1 PORTB	//Port de sortie du premier DAC
#define DAC2 PORTC	//Port de sortie du second DAC
#define RED 4000	//Durée avant réduction d'intensité en x10ms. Ex: 4000 pour 40s

void apply(int i);	//Fonction de modification des sorties associées aux DAC
char add;			//Variable contenant le pas
char step=1;		//Variable contenant la position courante dans la table des DAC
int time;			//Variable de gestion du temps pour la réduction d'intensitée

void main (void){
	TRISA = 0b1111 ;		//Port des entrées
	TRISB = 0b00000000 ;		//Sortie DAC 1
	TRISC = 0b00000000 ;		//Sortie DAC 2
	OPTION = 0b11000111;		//Configuration du TMR0, periode 51,2µs

	// Au début du programme, on teste l'état des entrées Config
	// pour connaître la position des jumpers et définir la
	// résolution en conséquence. La résolution est définie par
	// un pas. Si le pas est de 1, la résolution est de 1/16e,
	// si le pas est de 2, la résolution est de 1/8e etc...
	if(J1==0 && J2==0)		add = 1;
	else if(J1==1 && J2==0)	add = 2;
	else if(J1==0 && J2==1)	add = 4;
	else	 				add = 8;

	//La suite du programme se répète en boucle
	apply(1);			//On démarre les moteurs, pas n°1.
	for(;;){
		time=0;			//Variable de comptage du temps pour réduction d'intensité
		TMR0=0;
		// On attend un front montant sur STEP, pendant ce temps, on comptabilise le temps
		// qui passe sans qu'il n'y ait d'impulsion
		// Si le temps de réduction d'intensité est atteint, on sort de la boucle

		//On attend que STEP repasse à 0 si ce n'est pas déjà fait
		while(STEP==1);
		//On attend une impulsion sur STEP, ou que le temps de réduction soit atteint
		while(STEP==0 && time<RED){
			if(TMR0>195){			//Le TMR0 atteint 196 toutes les 10ms
				time++;				//time est donc incrémenté toutes les 10ms.
				TMR0=0;
			}
		}

		// Si on est sortis de la boucle parceque le temps de réduction était atteint
		// On rejoint le point de réduction d'intensité le plus proche dans la table
		if(time>=RED){
			if(step<=5 || step>=62) 		apply(65);
			else if(step>=6 && step<=13)	apply(66);
			else if(step>=14 && step<=21)	apply(67);
			else if(step>=22 && step<=29)	apply(68);
			else if(step>=30 && step<=37)	apply(69);
			else if(step>=38 && step<=45)	apply(70);
			else if(step>=46 && step<=53)	apply(71);
			else if(step>=54 && step<=61)	apply(72);

		// Si on est sortis de la boucle en raison d'une impulsion STEP
		}else{
			// On calcule la nouvelle position dans la table
			// en fonction du sens DIR
			if(DIR){
				if(step+add>64) step=1;
				else step += add;
			}else{
				if(step-add<1) step=64;
				else step -= add;
			}
			// On met à jour les sorties
			apply(step);
		}
	}
}

//Cette fonction contient l'ensemble des valeures des DAC pour
//un pas donné (max 1/16e de pas, soit 16*4 = 64 valeures)
//Les valeures sont envoyées aux DAC par appel de cette fonction
//avec en parametre le nouvelle position à adopter.
//Les valeures négatives sur les DAC sont obtenues en passant
//le 5e bit du port associé à 1. Ainsi l'info de signe est envoyée 
//aux LMD18245 sur leur entrée DIR. Notezrque M4 est connecté au bit 0, 
//M3 au bit 1 etc. (les valeures envoyées sont donc symétriques à la 
//valeure reçue par le DAC). Par exemple pour envoyer 0011 au DAC, 
//il faut écrire 1100 sur les 4 bit de poid faible du port du PIC.

void apply(int i){
	switch (i){
	case 1:
	        // DAC1=0        DAC2=-15
	        DAC1=0b00000000;        DAC2=0b00011111;        break;
	case 2:
	        // DAC1=1        DAC2=-14
	        DAC1=0b00001000;        DAC2=0b00010111;        break;
	case 3:
	        // DAC1=2        DAC2=-14
	        DAC1=0b00000100;        DAC2=0b00010111;        break;
	case 4:
	        // DAC1=4        DAC2=-14
	        DAC1=0b00000010;        DAC2=0b00010111;        break;
	case 5:
	        // DAC1=5        DAC2=-13
	        DAC1=0b00001010;        DAC2=0b00011011;        break;
	case 6:
	        // DAC1=7        DAC2=-13
	        DAC1=0b00001110;        DAC2=0b00011011;        break;
	case 7:
	        // DAC1=8        DAC2=-12
	        DAC1=0b00000001;        DAC2=0b00010011;        break;
	case 8:
	        // DAC1=9        DAC2=-11
	        DAC1=0b00001001;        DAC2=0b00011101;        break;
	case 9:
	        // DAC1=10       DAC2=-10
	        DAC1=0b00000101;        DAC2=0b00010101;        break;
	case 10:
	        // DAC1=11       DAC2=-9
	        DAC1=0b00001101;        DAC2=0b00011001;        break;
	case 11:
	        // DAC1=12       DAC2=-8
	        DAC1=0b00000011;        DAC2=0b00010001;        break;
	case 12:
	        // DAC1=13       DAC2=-7
	        DAC1=0b00001011;        DAC2=0b00011110;        break;
	case 13:
	        // DAC1=13       DAC2=-5
	        DAC1=0b00001011;        DAC2=0b00011010;        break;
	case 14:
	        // DAC1=14       DAC2=-4
	        DAC1=0b00000111;        DAC2=0b00010010;        break;
	case 15:
	        // DAC1=14       DAC2=-2
	        DAC1=0b00000111;        DAC2=0b00010100;        break;
	case 16:
	        // DAC1=14       DAC2=-1
	        DAC1=0b00000111;        DAC2=0b00011000;        break;
	case 17:
	        // DAC1=15       DAC2=0
	        DAC1=0b00001111;        DAC2=0b00000000;        break;
	case 18:
	        // DAC1=14       DAC2=1
	        DAC1=0b00000111;        DAC2=0b00001000;        break;
	case 19:
	        // DAC1=14       DAC2=2
	        DAC1=0b00000111;        DAC2=0b00000100;        break;
	case 20:
	        // DAC1=14       DAC2=4
	        DAC1=0b00000111;        DAC2=0b00000010;        break;
	case 21:
	        // DAC1=13       DAC2=5
	        DAC1=0b00001011;        DAC2=0b00001010;        break;
	case 22:
	        // DAC1=13       DAC2=7
	        DAC1=0b00001011;        DAC2=0b00001110;        break;
	case 23:
	        // DAC1=12       DAC2=8
	        DAC1=0b00000011;        DAC2=0b00000001;        break;
	case 24:
	        // DAC1=11       DAC2=9
	        DAC1=0b00001101;        DAC2=0b00001001;        break;
	case 25:
	        // DAC1=10       DAC2=10
	        DAC1=0b00000101;        DAC2=0b00000101;        break;
	case 26:
	        // DAC1=9        DAC2=11
	        DAC1=0b00001001;        DAC2=0b00001101;        break;
	case 27:
	        // DAC1=8        DAC2=12
	        DAC1=0b00000001;        DAC2=0b00000011;        break;
	case 28:
	        // DAC1=7        DAC2=13
	        DAC1=0b00001110;        DAC2=0b00001011;        break;
	case 29:
	        // DAC1=5        DAC2=13
	        DAC1=0b00001010;        DAC2=0b00001011;        break;
	case 30:
	        // DAC1=4        DAC2=14
	        DAC1=0b00000010;        DAC2=0b00000111;        break;
	case 31:
	        // DAC1=2        DAC2=14
	        DAC1=0b00000100;        DAC2=0b00000111;        break;
	case 32:
	        // DAC1=1        DAC2=14
	        DAC1=0b00001000;        DAC2=0b00000111;        break;
	case 33:
	        // DAC1=0        DAC2=15
	        DAC1=0b00000000;        DAC2=0b00001111;        break;
	case 34:
	        // DAC1=-1       DAC2=14
	        DAC1=0b00011000;        DAC2=0b00000111;        break;
	case 35:
	        // DAC1=-2       DAC2=14
	        DAC1=0b00010100;        DAC2=0b00000111;        break;
	case 36:
	        // DAC1=-4       DAC2=14
	        DAC1=0b00010010;        DAC2=0b00000111;        break;
	case 37:
	        // DAC1=-5       DAC2=13
	        DAC1=0b00011010;        DAC2=0b00001011;        break;
	case 38:
	        // DAC1=-7       DAC2=13
	        DAC1=0b00011110;        DAC2=0b00001011;        break;
	case 39:
	        // DAC1=-8       DAC2=12
	        DAC1=0b00010001;        DAC2=0b00000011;        break;
	case 40:
	        // DAC1=-9       DAC2=11
	        DAC1=0b00011001;        DAC2=0b00001101;        break;
	case 41:
	        // DAC1=-10      DAC2=10
	        DAC1=0b00010101;        DAC2=0b00000101;        break;
	case 42:
	        // DAC1=-11      DAC2=9
	        DAC1=0b00011101;        DAC2=0b00001001;        break;
	case 43:
	        // DAC1=-12      DAC2=8
	        DAC1=0b00010011;        DAC2=0b00000001;        break;
	case 44:
	        // DAC1=-13      DAC2=7
	        DAC1=0b00011011;        DAC2=0b00001110;        break;
	case 45:
	        // DAC1=-13      DAC2=5
	        DAC1=0b00011011;        DAC2=0b00001010;        break;
	case 46:
	        // DAC1=-14      DAC2=4
	        DAC1=0b00010111;        DAC2=0b00000010;        break;
	case 47:
	        // DAC1=-14      DAC2=2
	        DAC1=0b00010111;        DAC2=0b00000100;        break;
	case 48:
	        // DAC1=-14      DAC2=1
	        DAC1=0b00010111;        DAC2=0b00001000;        break;
	case 49:
	        // DAC1=-15      DAC2=0
	        DAC1=0b00011111;        DAC2=0b00000000;        break;
	case 50:
	        // DAC1=-14      DAC2=-1
	        DAC1=0b00010111;        DAC2=0b00011000;        break;
	case 51:
	        // DAC1=-14      DAC2=-2
	        DAC1=0b00010111;        DAC2=0b00010100;        break;
	case 52:
	        // DAC1=-14      DAC2=-4
	        DAC1=0b00010111;        DAC2=0b00010010;        break;
	case 53:
	        // DAC1=-13      DAC2=-5
	        DAC1=0b00011011;        DAC2=0b00011010;        break;
	case 54:
	        // DAC1=-13      DAC2=-7
	        DAC1=0b00011011;        DAC2=0b00011110;        break;
	case 55:
	        // DAC1=-12      DAC2=-8
	        DAC1=0b00010011;        DAC2=0b00010001;        break;
	case 56:
	        // DAC1=-11      DAC2=-9
	        DAC1=0b00011101;        DAC2=0b00011001;        break;
	case 57:
	        // DAC1=-10      DAC2=-10
	        DAC1=0b00010101;        DAC2=0b00010101;        break;
	case 58:
	        // DAC1=-9       DAC2=-11
	        DAC1=0b00011001;        DAC2=0b00011101;        break;
	case 59:
	        // DAC1=-8       DAC2=-12
	        DAC1=0b00010001;        DAC2=0b00010011;        break;
	case 60:
	        // DAC1=-7       DAC2=-13
	        DAC1=0b00011110;        DAC2=0b00011011;        break;
	case 61:
	        // DAC1=-5       DAC2=-13
	        DAC1=0b00011010;        DAC2=0b00011011;        break;
	case 62:
	        // DAC1=-4       DAC2=-14
	        DAC1=0b00010010;        DAC2=0b00010111;        break;
	case 63:
	        // DAC1=-2       DAC2=-14
	        DAC1=0b00010100;        DAC2=0b00010111;        break;
	case 64:
	        // DAC1=-1       DAC2=-14
	        DAC1=0b00011000;        DAC2=0b00010111;        break;
			
	//Ces cas sont utilisés pour les réductions d'intensité	
	case 65:
	        // DAC1=0        DAC2=-6
	        DAC1=0b00000000;        DAC2=0b00010110;        break;
	case 66:
	        // DAC1=4        DAC2=-4
	        DAC1=0b00000010;        DAC2=0b00010010;        break;
	case 67:
	        // DAC1=6        DAC2=0
	        DAC1=0b00000110;        DAC2=0b00000000;        break;
	case 68:
	        // DAC1=5        DAC2=4
	        DAC1=0b00001010;        DAC2=0b00000010;        break;
	case 69:
	        // DAC1=0        DAC2=6
	        DAC1=0b00000000;        DAC2=0b00000110;        break;
	case 70:
	        // DAC1=-4       DAC2=4
	        DAC1=0b00010010;        DAC2=0b00000010;        break;
	case 71:
	        // DAC1=-6       DAC2=0
	        DAC1=0b00010110;        DAC2=0b00000000;        break;
	case 72:
	        // DAC1=-4       DAC2=-4
	        DAC1=0b00010010;        DAC2=0b00010010;        break;
	default :
			DAC1=0 ; 	DAC2=0 ;	break;
	}

}
