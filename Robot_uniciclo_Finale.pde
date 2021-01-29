/*partecipanti al progetto: Simone Ricciotti (0253093), Emanuele Arilli (0266387), Luca Sugamosto (0252792).

Questo sketch effettua un controllo sulla base della stima della posa dell'uniciclo
ottenuta mediante un filtro di Kalman esteso che fonde l'odometria (movimento ruote)
con le misure di distanza da un certo numero di landmark.

Quindi, si disegna un robot mobile di tipo 'uniciclo' e lo si controlla in velocità
(senza limiti di saturazione) verso il punto cliccato col mouse mediante la legge di
controllo proporzionale, tuttavia il calcolo del controllo è fatto sulle variabili
stimate anzichè su quelle vere.
Sulla schermata viene anche disegnato (in giallo) il robot nella posizione stimata
(quello vero è disegnato in rosso).
In più, dei triangoli in bianco col bordo rosso rappresentano la posizione dei
landmark presenti nell'ambiente. All'interno di ciascun triangolo viene anche
riportato un identificatore del tipo 'Li' per il landmark i-esimo.

I landmark sono visibili solo quando cadono dentro un settore circolare di raggio rMax e angolo betaMax,
centrato nell'uniciclo e simmetrico rispetto alla direzione di avanzamento dell'uniciclo stesso.
Quando un landmark diventa visibile (cioè cade nella regione di visibilità), si può utilizzare la misura di distanza da tale landmark.
Tale misura è caratterizzata da un errore gaussiano a media nulla e deviazione standard σL che va presa pari a 15 pixel.
I landmark che cadono nella regione di visibilità sono evidenziati poichè viene modificato il colore.

Il robot possiede anche una bussola che gli fornisce una misura del suo orientamento.
Cioè, nel vettore z, oltre alle misure di distanza dai landmark visibili, c'è una componente zb = θ + nb,
che fornisce una misura dell'orientamento θ dell'uniciclo ed è caratterizzata da un errore nb da prendere gaussiano a media nulla con deviazione standard σb = 10 gradi.
Quando nessun landmark è visibile, l'unica misura disponibile è quella della bussola.

Premendo i tasti 'r' e 'R' è possibile diminuire e, rispettivamente, aumentare il range massimo rMax del settore circolare.
Analogamente, con i tasti 'b' e 'B', è possibile variare l'angolo di bearing massimo betaMax che definisce l'apertura del settore circolare
(nel senso che l'angolo di apertura di tale settore è 2*betaMax).
Il valore minimo positivo per rangeMax è 50 pixel e i valori dell'angolo betaMax sono compresi tra :  10 < betaMax < 140 gradi. 

Con le frecce è possibile modificare la frequenza delle misure di distanza dai 
landmark visibili o, più precisamente, il tempo 'tStep' che intercorre tra una misura e la 
successiva (comunque non inferiore al ciclo di Processing, per default dt = 1/60 s):
con le frecce SU e GIU' si incrementa e decrementa rispettivamente di 1 tale tempo 
mentre con le frecce DESTRA e SINISTRA lo si moltiplica e divide rispettivamente per 2
(per modificarlo più velocemente).
Quando tale tempo è molto grande la ricostruzione diviene puramente odometrica.

Sulla schermata vengono riportati: le coordinate (x, y, theta) dell'uniciclo,
le coordinate (xDes, Ydes) del punto da raggiungere e il tempo impiegato per farlo.
Inoltre si riportano le velocità angolari (omegaR, OmegaL) delle due ruote,
la velocità longitudinale v1 e quella angolare v2.
Infine si indicano anche le grandezze stimate (xHat, yHat, thetaHat) con la loro 
matrice di covarianza 'P' e il tempo 'tStep' (modificabile da tastiera) che 
intercorre tra una lettura e la successiva. */

//Dimensione della finestra
int sizeX = 1250;
int sizeY = 750;

//Coordinate attuali dell'unicliclo
float x = 0;
float y = 0;
float theta = 0;

//Coordinate desiderate dell'uniciclo
float xDes = 0;
float yDes = 0;
float thetaDes;                                                                             //orientamento desiderato (per raggiungere il target)

//Caratteristiche fisiche dell'uniciclo
float r = 8;                                                                                //raggio ruote [pixel]
float d = 25;                                                                               //distanza tra le ruote [pixel]
float w = 5;                                                                                //spessore della ruota [pixel]
float R = 1.2 * sqrt(pow(r,2) + pow(d/2 + w/2,2));    //raggio del robot
float A;                                                                                    //area del settore circolare

float dt = (float) 1/60;                                                                    //tempo di campionamento

float e_p = 0;                                                                              //errore posizionamento
int nGiri = 0;                                                                              //n° di giri su se stesso che ha fatto il robot
long nStime = 0;                                                                            //n° progressivo totale delle stime fatte
long nStimePRE = 0;                                                                         //stime del ciclo precedente

float v1 = 0;                                                                               //velocità longitudinale del robot
float v2 = 0;                                                                               //velocità rotazionale del robot
float kv1 = 1;                                                                              //costante legge proporzionale controllo v1
float kv2 = 2;                                                                              //costante legge proporzionale controllo v2

float t0, tempo;                                                                            //tempo di inizio e attuale missione
float tempoUltimaMisura;                                                                    //tempo in cui si è effettuata l'ultima misura

float omegaR = 0;                                                                           //velocità angolare ruota destra
float omegaL = 0;                                                                           //velocità angolare ruota sinistra
float uR, uL, uRe, uLe;                                                                     //spostamenti (veri e presunti) ruota destra e sinistra

//Variabili relative alla stima e al filtro di Kalman esteso
float sigmaX0 = 10;
float sigmaY0 = 10;
float sigmaTheta0 = 10*PI/180;

float xHat = x + Gaussian(0, sigmaX0);
float yHat = y + Gaussian(0, sigmaY0);
float thetaHat = theta + Gaussian(0, sigmaTheta0);

float xHatMeno, yHatMeno, thetaHatMeno;                                                      //stime a priori di x, y, theta
float KR = 0.01;
float KL = 0.01;
float stDevR, stDevL;

//creazione e inizializzazione dei landmark e dei vettori ad essi associati
float [][] Landmark = {{0,(sizeY/2)-25}, {(-sizeX/2)+25,(sizeY/2)-350}, {0, (sizeY/2)-350}, {(sizeX/2)-25,(sizeY/2)-350},
                       {(-sizeX/2)+25,(sizeY/2)-700}, {0,(sizeY/2)-700}, {(sizeX/2)-25,(sizeY/2)-700}};    
                                                                                             //coordinate landmark (visibili e non)
int nL = Landmark.length;                                                                    //n° totale di landmark (visibili e non)

int nV;                                                                                      //n° totale di landmark visibili
float [][] LandmarkV = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};             //coordinate landmark visibili (inizialmente tutte nulle)

//Seguono le matrici utilizzate dal filtro di kalman esteso (EKF)
float [][] F = {{1, 0, 0},{0, 1, 0},{0, 0, 1}};                                              //matrice jacobiana F = df/dx
float [][] P = {{pow(sigmaX0,2), 0, 0},{0, pow(sigmaY0,2), 0},{0, 0, pow(sigmaTheta0,2)}};   //matrice di covarianza P
float [][] Pmeno = new float[3][3];                                                          //matrice di covarianza a priori P-
float [][] W = {{1, 0},{0,1},{1/d,-1/d}};                                                    //matrice jacobiana W = df/dw
float [][] Q = {{1, 0},{0,1}};                                                               //matrice di covarianza del rumore di misura                       

float DeltaX, DeltaY, DeltaXY;                                                               //variabili di supporto
float sigmaLandmark = 15;                                                                    //deviazione standard errore di misura di distanza dai landmark visibili [pixel]

float [][] correzione = new float[3][1];                                                     //termine correttivo stima

float tStep = 0;                                                                             //tempo [ms] tra una misura e la successiva (variabile da tastiera)

//Parametri dell'arco (settore circolare) intorno al robot
float rMax = 100;                                                                            //distanza massima iniziale del settore circolare
float betaMax = PI/6;                                                                        //raggio massimo iniziale del settore circolare

//interi per la funzione "get"
int colorX;
int colorY;

//colore arco

color verde = color(0,255,0);

void setup() {
  size(1250, 750);
  tempoUltimaMisura = 0;                                                                     //inizializzo a zero il tempo in cui è stata effettuata l'ultima misura
}

void draw() {
  background(0);
  nV = 0;    //inizializzo a zero il numero di landmark visibili in ogni ciclo
  //Reimposto la matrice dei LandmarkV
  int indice;
  for (indice=0; indice<nL;indice++){
       LandmarkV[indice][0] = 0; 
       LandmarkV[indice][1] = 0;
  }
  
  pushMatrix();
  translate(sizeX/2, sizeY/2);
  
  if (keyPressed) {
    if (keyCode == UP) {
      tStep += 1;                                                                            //aumento di 1 il tempo tra una misura e la successiva
    }
    if (keyCode == DOWN) {
      tStep = max(0, tStep - 1);                                                             //decremento di 1 il tempo tra una misura e la successiva
    }
    if (keyCode == RIGHT) {
      tStep = (tStep * 2);                                                                   //moltiplico per due il tempo tra una lettura e la successiva
    }
    if (keyCode == LEFT) {
      tStep = (tStep / 2);                                                                   //divido per due il tempo tra una lettura e la successiva
    }
    if (key == 'r') {
      if (rMax > 50) {
        rMax -= 10;                                                                          //decremento raggio d'azione del settore circolare (fino ad un minimo di 50 pixel)
      }
    }
    if (key == 'R') {
      rMax += 10;                                                                            //incremento raggio d'azione del settore circolare (fino all'infinito)
    }
    if (key == 'b') {
      if (betaMax > 20*PI/180) {
        betaMax -= .1;                                                                       //decremento angolo d'azione del settore circolare (fino ad un minimo di 10 gradi)
      }
    }
    if (key == 'B') {
      if (betaMax < 140*PI/180) {
        betaMax += .1;                                                                       //incremento angolo d'azione del settore circolare (fino ad un massimo di 140 gradi)
      }
    }
  }
  
  if (mousePressed) {
    xDes = mouseX - (sizeX / 2);
    yDes = (sizeY / 2) - mouseY;
    t0 = millis();
  }
  
//Calcolo errore e controllo basandomi sul valore stimato (xHat,yHat,thetaHat) delle variabili dell'uniciclo
  e_p = sqrt(pow(xDes-xHat, 2) + pow(yDes-yHat, 2));
  
  if (e_p > 1) {                                                                             //mi muovo solo se l'errore è maggiore di una certa quantità
    tempo = (millis()-t0) / 1000;                                                            //tempo missione in secondi

    //assegno velocità secondo legge proporzionale (in termini di quantità stimate)
    v1 = -kv1 * ((xHat-xDes)*cos(thetaHat) + (yHat-yDes)*sin(thetaHat));

    //calcolo l'angolo verso il target: scelgo il multiplo di 2PI più vicino all'orientamento corrente (stimato) del robot
    thetaDes = atan2(yDes-yHat, xDes-xHat) + (nGiri*2*PI);
    
    if (abs(thetaDes + (2*PI) - thetaHat) < abs(thetaDes - thetaHat)) {
      thetaDes = (thetaDes+2*PI);
      nGiri += 1;
    }   
    else {      
      if (abs(thetaDes - (2*PI) - thetaHat) < abs(thetaDes - thetaHat)) {
        thetaDes = (thetaDes-2*PI);
        nGiri += -1;
      }
    }   
    //assegno velocità angolare secondo legge proporzionale sempre in funzione delle quantità stimate   
    v2 = kv2 * (thetaDes - thetaHat);
  }
  else {                                                                                     //se penso di essere vicino al target mi fermo
    v1 = 0;
    v2 = 0;
  }
  
//Calcolo i movimenti da impartire alle ruote in base alle v1 e v2 trovate
  omegaR = (v1 + v2*d/2) / r;
  omegaL = (v1 - v2*d/2) / r;
  uRe = (r*omegaR*dt);                                                                       //spostamento comandato alla ruota destra
  uLe = (r*omegaL*dt);                                                                       //spostamento comandato alla ruota sinistra
  
//Perturbo i due movimenti: gli spostamenti reali delle ruote non saranno mai esattamente uguali a quelli comandati 
  stDevR = sqrt(KR * abs(uRe));
  stDevL = sqrt(KL * abs(uLe));
  uR = uRe + Gaussian(0,stDevR);                                                             //spostamento vero ruota destra
  uL = uLe + Gaussian(0,stDevL);                                                             //spostamento vero ruota sinistra
  
//Dinamica effettiva dell'uniciclo
  x = x + ((uR+uL) / 2) * cos(theta);
  y = y + ((uR+uL) / 2) * sin(theta);
  theta = theta + (uR-uL)/d;
  
//Disegno l'arco verde utilizzato per distinguere i landmark che l'arco tocca (landmark visibili) e quelli che non tocca (landmark non visibili)
  arco(x, y, theta, rMax, betaMax); 
  
//Controllo e identificazione dei landmark visibili tra i landmark totali, vedendo quali sono di colore verde = Toccati dall'arco
  int i = 0;
  for (int index = 0; index < nL; index++) {                                                 //ciclo for che scorre tutti i landmark e trova quelli visibili
      colorX = (int) Landmark[index][0];
      colorY = (int) Landmark[index][1];
      if (get(colorX+625,-colorY+375) == verde) { //Se la coordinata del landmark è dentro l'arco, allora è visibile
        LandmarkV[i][0] = Landmark[index][0];                                                //inserimento del landmark visibile all'interno dell'array dei landmark visibili
        LandmarkV[i][1] = Landmark[index][1];
        nV++;                                                                                //incremento del numero di landmark visibili
        i++;
    }
  }
  
/*Definizione delle matrici e dei vettori utilizzati per il calcolo della posizione/orientamento dell'uniciclo,
  le quali tengono conto sia delle misure dei landmark visibili sia della bussola*/
  float[] misureLandmark = new float [nV+1];                                                 //vettore con le misure vere di distanza dagli nV landmark visibili
  float[] misureAtteseLandmark = new float [nV+1];                                           //vettore con le misure attese di distanza dagli nV landmark visibili
  
  float[][] H = new float[nV+1][3];
  float[][] K = new float[3][nV+1];
  
  float[][] Rs = idMat((nV+1), pow(sigmaLandmark,2));                                        // Per semplicità inserisco l'ultimo elemento errato
  
  Rs[nV][nV] = pow(sigmaTheta0,2);                                                           // Cambio l'ultimo elemento della matrice Rs con la sigmaTheta della bussola
  
  float[][] innovazione = new float[nV+1][1];
  
//STIMA FILTRO KALMAN ESTESO: PASSO di PREDIZIONE
  xHatMeno = xHat + ((uRe+uLe) / 2) * cos(thetaHat);
  yHatMeno = yHat + ((uRe+uLe) / 2) * sin(thetaHat);
  thetaHatMeno = thetaHat + (uRe-uLe)/d;

//Aggiorno la jacobiana F (solo gli elementi variabili)
  F[0][2] = -(uRe+uLe) * sin(thetaHat)/2;
  F[1][2] = (uRe+uLe) * cos(thetaHat)/2;
  
//Aggiorno W (solo gli elementi variabili)
  W[0][0] = .5 * cos(thetaHat);
  W[0][1] = .5 * cos(thetaHat);
  W[1][0] = .5 * sin(thetaHat);
  W[1][1] = .5 * sin(thetaHat);

  //Aggiorno Q (solo gli elementi variabili)
  Q[0][0] = KR * abs(uRe);
  Q[1][1] = KL * abs(uLe);
  
//Calcolo matrice covarianza a priori
  Pmeno = mSum(mProd(mProd(F,P), trasposta(F)), mProd(mProd(W,Q), trasposta(W)));            //Pmeno = F*P*F' + W*Q*W'
  
//STIMA FILTRO DI KALMAN ESTESO: PASSO di CORREZIONE
  if (millis()-tempoUltimaMisura >= tStep) {                                                 //attuo la correzione solo se ho le misure (che arrivano ogni tStep ms)
    tempoUltimaMisura = millis();                                                            //memorizzo il tempo in cui ho fatto l'ultima misura
    nStime++;                                                                                //incremento il contatore delle stime fatte
    
    for (int indLandmark = 0; indLandmark < nV; indLandmark++) {
      misureLandmark[indLandmark] = sqrt(pow(x-LandmarkV[indLandmark][0],2) + pow(y-LandmarkV[indLandmark][1],2)) + Gaussian(0,sigmaLandmark);
      misureAtteseLandmark[indLandmark] = sqrt(pow(xHatMeno-LandmarkV[indLandmark][0],2) + pow(yHatMeno-LandmarkV[indLandmark][1],2));
      DeltaX = xHatMeno - LandmarkV[indLandmark][0];
      DeltaY = yHatMeno - LandmarkV[indLandmark][1];
      DeltaXY = sqrt(pow(DeltaX,2) + pow(DeltaY,2));
      H[indLandmark][0] = DeltaX / DeltaXY;
      H[indLandmark][1] = DeltaY / DeltaXY;  
      H[indLandmark][2] = 0;
      innovazione[indLandmark][0] = misureLandmark[indLandmark] - misureAtteseLandmark[indLandmark];
    }
    
  //Calcolo orientamento dell'uniciclo ricavato dalla bussola
    misureLandmark[nV] = theta + Gaussian(0,sigmaTheta0);
    misureAtteseLandmark[nV] = thetaHatMeno;
    H[nV][0] = 0;
    H[nV][1] = 0;
    H[nV][2] = 1; 
    innovazione[nV][0] = misureLandmark[nV] - misureAtteseLandmark[nV];
    
  //Calcolo guadagno Kalman e aggiorno covarianza
    K = mProd(mProd(Pmeno,trasposta(H)), invMat(mSum(mProd(mProd(H,Pmeno), trasposta(H)),Rs)));
    P = mProd(mSum(idMat(3,1), mProd(idMat(3,-1), mProd(K,H))), Pmeno);

  //Correggo la stima    
    correzione = mProd(K,innovazione);
    xHat = xHatMeno + correzione[0][0];
    yHat = yHatMeno + correzione[1][0];
    thetaHat = thetaHatMeno + correzione[2][0];    
  }
  else {                                                                                      //se non ho misure non correggo nulla
    xHat = xHatMeno;
    yHat = yHatMeno;
    thetaHat = thetaHatMeno;
    P = Pmeno;
  }
// FINE EKF

//Disegno il robot vero, il robot stimato e il settore circolare (arco)
  robot(x, y, theta, 1);                                                                      //l'argomento 1 fa un robot rosso (robot reale)                                             
  robot(xHat, yHat, thetaHat, 0);                                                             //l'argomento 0 un robot giallo (robot nella posa stimata)

//Disegno i landmark con dei triangoli bianchi col contorno rosso e l'identificativo del landmark all'interno
  stroke(120,0,0);
  strokeWeight(1);
  
  if ((nV == 0) || (nStime == nStimePRE) ) {                                                  //caso in cui nessun landmark è vibile e/o non viene aggiornata la stima
    for (int indLandmark = 0; indLandmark < nL; indLandmark++) {
      fill(90,90,90);
      triangle(Landmark[indLandmark][0]-15,-Landmark[indLandmark][1]+15,Landmark[indLandmark][0]+15,-Landmark[indLandmark][1]+15,Landmark[indLandmark][0],-Landmark[indLandmark][1]-15);
      textSize(10);
      fill(0,0,0);
      text("L",Landmark[indLandmark][0]-5,-Landmark[indLandmark][1]+8);
      text(indLandmark+1,Landmark[indLandmark][0]+1,-Landmark[indLandmark][1]+8);
    }
  }
  else {                                                                                      //caso in cui almeno un landmark è visibile e/o viene aggiornata la stima
    int ii = 0;                                                                               //colorazione dei landmark visibili
    for (int indLandmark = 0; indLandmark < nL; indLandmark++) {
      if ((Landmark[indLandmark][0] == LandmarkV[ii][0]) && (Landmark[indLandmark][1] == LandmarkV[ii][1])) {
        stroke(255,0,0);
        strokeWeight(2);
        fill(255,255,255);
        triangle(Landmark[indLandmark][0]-15,-Landmark[indLandmark][1]+15,Landmark[indLandmark][0]+15,-Landmark[indLandmark][1]+15,Landmark[indLandmark][0],-Landmark[indLandmark][1]-15);
        textSize(10);
        fill(0,0,0);
        text("L",Landmark[indLandmark][0]-5,-Landmark[indLandmark][1]+8);
        text(indLandmark+1,Landmark[indLandmark][0]+1,-Landmark[indLandmark][1]+8);
        ii++;
      }
      else {                                                                                  //colorazione dei restanti landmark non visibili
        stroke(120,0,0);
        strokeWeight(2);
        fill(90,90,90);
        triangle(Landmark[indLandmark][0]-15,-Landmark[indLandmark][1]+15,Landmark[indLandmark][0]+15,-Landmark[indLandmark][1]+15,Landmark[indLandmark][0],-Landmark[indLandmark][1]-15);
        textSize(10);
        fill(0,0,0);
        text("L",Landmark[indLandmark][0]-5,-Landmark[indLandmark][1]+8);
        text(indLandmark+1,Landmark[indLandmark][0]+1,-Landmark[indLandmark][1]+8);
      }
    }
  }
  
  nStimePRE = nStime;                                                                         //riporto il valore delle stime del ciclo attuale per controllare se nel ciclo successivo
                                                                                              //questa grandezza varia o rimane come il precedente
  stroke(0);
  strokeWeight(1);
  
  popMatrix();

  textSize(20);
  fill(0,0,255);
  text("v1 (pixel/s) = ",10,20); 
  text(v1,200,20);
  text("v2 (gradi/s) = ",10,50); 
  text(v2*180/PI,200,50);
  
  fill(255,0,0);  
  text("x = ",10,160); 
  text(x,80,160);
  text("y = ",10,190); 
  text(y,80,190);
  text("theta = ",10,220); 
  text(theta*180/PI,100,220);  

  fill(255,255,255);
  text("tempo = ",10,110); 
  text(tempo,120,110);  

  fill(0,0,255);
  text("omegaR (gradi/s) = ",650,20); 
  text(omegaL*180/PI,900,20);
  text("omegaL (gradi/s) = ",650,50); 
  text(omegaL*180/PI,900,50);
  
  fill(255,0,0);
  text("xDes = ",700,100); 
  text(xDes,800,100);
  text("yDes = ",700,130); 
  text(yDes,800,130);

  fill(255,255,0);  
  text("xHat = ",10,280); 
  text(xHat,120,280);
  text("yHat = ",10,310); 
  text(yHat,120,310);
  text("thetaHat = ",10,340); 
  text(thetaHat*180/PI,160,340);  

  fill(255,255,255);
  text("nStime = ",10,390); 
  text(nStime,120,390);  

  fill(255,255,255);
  text("tStep (ms) = ",10,420); 
  text(tStep,150,420);  

  fill(255,255,0);  
  text("P = ",10,460); 
  text(P[0][0],10,490); text(P[0][1],100,490); text(P[0][2],190,490);
  text(P[1][0],10,520); text(P[1][1],100,520); text(P[1][2],190,520); 
  text(P[2][0],10,550); text(P[2][1],100,550); text(P[2][2],190,550);
}

void robot(float x, float y, float theta, int colore) {
// funzione che disegna uniciclo in (x,y) con orientamento theta      
  pushMatrix();
  translate(x,-y);
  rotate(-theta);
  stroke(0,0,0);
  
  if (colore == 1) {
    fill(255,0,0);
  }
  else {
    fill(255,255,0);
  }
  
  ellipse(0,0,2*R,2*R);
  fill(0,0,255);  
  rect(-r,-d/2-w/2,2*r,w);
  rect(-r,d/2-w/2,2*r,w);
  fill(255);
  ellipse(.8*R,0,.2*R,.2*R);
  ellipse(-.8*R,0,.2*R,.2*R);  
  fill(0,255,0);
  triangle(-.1*R,.3*R,-.1*R,-.3*R,.5*R,0);
  popMatrix();
}

void arco(float x,float y,float theta,float rMax, float betaMax) {                                        //funzione che realizza l'arco intorno al robot rosso (robot reale)
  fill(0,255,0);
  pushMatrix();
  translate(x,-y);
  rotate(-theta);
  arc(0, 0, 2*rMax, 2*rMax, -betaMax/2, betaMax/2);
  popMatrix();
}

/******************************************************
/******************************************************
  DA QUI IN POI CI SONO FUNZIONI DI SERVIZIO: 
  1) CALCOLO ERRORE GAUSSIANO
  2) ALGEBRA MATRICIALE
/******************************************************
/******************************************************/

float Gaussian(float media, float stDev) // Restituisce variabile N(media,stDev^2) approssimata sfruttando il teorema del limite centrale
{
  float somma = 0;
  for (int k=0; k<27; k+=1) // 27 in modo che sqrt(3/27)=1/3
  {
    somma = somma + random(-stDev/3,stDev/3);
  }
  return media + somma;
}

float[][] mProd(float[][] A,float[][] B) // Calcola prodotto di due matrici A e B (si assume, senza controllarlo, che numero colonne A = numero righe B!)
{
  int nA = A.length;
  int nAB = A[0].length;
  int nB = B[0].length;
  
  float[][] C = new float[nA][nB]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nB; j++) 
    {  
      for (int k=0; k < nAB; k++) 
      {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return C;
}

float[][] mSum(float[][] A,float[][] B) // Calcola la somma di due matrici A e B (si assume, senza controllarlo, che A e B abbiano stesse dimensioni!)
{
  int nA = A.length;
  int nB = A[0].length;
  
  float[][] C = new float[nA][nB]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nB; j++) 
    {  
      C[i][j] = A[i][j] + B[i][j];
    }
  }
  return C;
}

float[][] trasposta(float[][] A) // Calcola la trasposta di una matrice A
{
  int nR = A.length;
  int nC = A[0].length; 
  
  float[][] C = new float[nC][nR]; 

  for (int i=0; i < nC; i++) 
  {
    for (int j=0; j < nR; j++) 
    {  
      C[i][j] = A[j][i];
    }
  }
  return C;
}


float[][] minore(float[][] A, int i, int j) // Determina il minore (i,j) di una matrice A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float[][] C = new float[nA-1][nA-1];
  
  for (int iM = 0; iM < i; iM++)
  {
    for (int jM = 0; jM < j; jM++)
    {
      C[iM][jM] = A[iM][jM];
    } 
    for (int jM = j; jM < nA-1; jM++)
    {
      C[iM][jM] = A[iM][jM+1];
    } 
  }
  for (int iM = i; iM < nA-1; iM++)
  {
    for (int jM = 0; jM < j; jM++)
    {
      C[iM][jM] = A[iM+1][jM];
    } 
    for (int jM = j; jM < nA-1; jM++)
    {
      C[iM][jM] = A[iM+1][jM+1];
    } 
  }
  return C;
}


float det(float[][] A) // Calcola il determinante di A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float determinante = 0;
  
  if (nA == 1)
  {
    determinante = A[0][0];
  }
  else
  {
    for (int j=0; j < nA; j++) 
    {
      determinante = determinante + A[0][j]*pow(-1,j)*det(minore(A,0,j));
    }
  }
  return determinante;
}


float[][] invMat(float[][] A) // Calcola l'inversa di una matrice A (si assume, senza controllarlo, che A sia quadrata!)
{
  int nA = A.length;
  float[][] C = new float[nA][nA];
  float detA = det(A);

/*
  if (abs(detA)<0.001) // Per evitare casi singolari con determinanti vicini a 0
  {
    if (detA>0)
    {
      detA = 0.001;
    }
    else
    {
      detA = -0.001;
    }
  }
*/  
  
  if (nA == 1)
  {
    C[0][0] = 1/detA;
  }
  else
  {
    for (int i=0; i < nA; i++) 
    {
      for (int j=0; j < nA; j++) 
      {
        C[j][i] = pow(-1,i+j)*det(minore(A,i,j))/detA;
      }
    }
  }
  return C;
}

float[][] idMat(int nA, float sigma) // Assegna una matrice identità di ordine nA moltiplicata per una costante sigma
{
  float[][] I = new float[nA][nA]; 

  for (int i=0; i < nA; i++) 
  {
    for (int j=0; j < nA; j++) 
    {  
      I[i][j] = 0;
    }
    I[i][i] = sigma;
  }
  return I;
}
