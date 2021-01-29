//partecipanti al progetto: Simone Ricciotti (0253093), Emanuele Arilli (0266387), Luca Sugamosto (0252792).

/*Istruzioni per l'uso del programma sviluppato:
-A partire dal tasto 1 fino a tasto 5 è possibile selezionare i rispettivi giunti, cioè con il tasto 1 si seleziona il giunto 1 e così via...;

-Premendo il tasto 6 è possibile aprire o chiudere la pinza utilizzando le frecce direzionali sinistra e destra;

-Una volta selezionato il giunto che si vuole muovere, quest'ultima operazione è possibile, utilizzando le frecce direzionali DESTRA e SINISTRA;

-I giunti 2 e 4 (d2 e theta4) sono vincolati e cioè si muovono entro un certo intervallo impostato;

-A schermo sono riportati i valori in gradi dei rispettivi giunti, inoltre sono riportati anche l'orientamento della pinza (cioè (x5,y5,z5) rispetto alla terna di base (x0,y0,z0))
e la sua posizione (sempre rispetto alla terna di base);

-Premendo il tasto del mouse è possibile spostare la base del robot in qualsiasi punto si voglia;

-Premendo le frecce direzionali SU e GIU' è possible variare la vista verticale del robot (tale valore è espresso anche sullo schermo);

-Il valore dei vari angoli di giunto sono riportati a schermo in gradi con un valore compreso tra -180 e 180.*/

  int giunto = 0;
  
  int segno = 1;
  
  float eyeY = 0;
  
  float posPinza = 0;
  
  float Px, Py, Pz;
  
  float d1, d2, l3, d5;
  
  //coordinate centro link 0
  float xbase;
  float ybase;
  
  //dimensioni link 0 (cilindro)
  float bottom = 67;
  float top = 67;
  float h = 40;
  int sides = 32;

  //dimensioni link 1
  float d1x = 130;
  float d1y = 40;
  float d1z = 50;

  //dimensioni link 2
  float d2x = 120;
  float d2y = 30;
  float d2z = 30;

  //dimensioni link 3
  float d3x = 30;
  float d3y = 110;
  float d3z = 30;

  //dimensioni giunto tra link 3 e link 4 (cilindro)
  float top2 = 19;
  float bottom2 = 19;
  float h2 = 40;
  float sides2 = 32;
  
  //dimensioni link 4
  float d4x = 30;
  float d4y = 30;
  float d4z = 110;

  //dimensioni link 5
  float d5x = 50;
  float d5y = 30;
  float d5z = 10;
 
  //dimensioni elementi pinza 
  float d6x = 8;
  float d6y = 30;
  float d6z = 20;
  
  //variabili di giunto (theta1, d2, theta3, theta4, theta5, d6)
  float[] theta = {0, d2x/2, -PI/2, 0, 0, 0};
  
void setup() {
  size(1000, 750, P3D);
  stroke(204, 102, 0);
  strokeWeight(0.8);
  
  xbase = width/2;
  ybase = height/2;
}

void draw() {
  background(120);
  lights();
  camera((width/2.0), height/2 - eyeY, (height/2.0) / tan(PI*60.0 / 360.0), width/2.0, height/2.0, 0, 0, 1, 0);
  
  robot();
  
  if(mousePressed) {
    xbase = mouseX;
    ybase = mouseY;
  }
  
  if (keyPressed) {
    
    //movimento camera
    if (keyCode == DOWN) {
      eyeY -= 5;
    }
    if (keyCode == UP) {
      eyeY += 5;
    }
    
    //selezione del giunto
    if (key == '1') {
      giunto = 0;
    }
    if (key == '2') {
      giunto = 1;
    }
    if (key == '3') {
      giunto = 2;
    }
    if (key == '4') {
      giunto = 3;
    }
    if (key == '5') {
      giunto = 4;
    }
    if (key == '6') {
      giunto = 5;
    }
    
    //movimento del giunto selezionato
    if (keyCode == LEFT) {
      segno = -1;
      move();
    }
    if (keyCode == RIGHT) {
      segno = 1;
      move();      
    }
  }
}

void robot() {
  pushMatrix();
  fill(204, 102, 0);
  
  //link 0 del robot
  translate(xbase, ybase);
  drawCylinder(top, bottom, h, sides);
  
  //link 1(si muove con theta1 = theta[0])
  rotateY(theta[0]);
  translate(0, -(d1y)/2, 0);
  box(d1x, d1y, d1z);
  
  //link 2(si muove con d2 = theta[1])
  translate(theta[1], 0, 0);
  box(d2x, d2y, d2z);
  
  //link 3(si muove con theta3 = theta[2])
  translate((d2x+d3x)/2, 0, 0);
  rotateX(-PI/2);
  rotateX(-theta[2]);
  translate(0, -(d3y-d2y)/2, 0); //mi posiziono per la rotazione nella parte bassa del link
  box(d3x, d3y, d3z);
  
  //giunto tra link 3 e link 4
  translate(0, -(d3y+(top2/2))/2, 0);
  drawCylinder2(top2, bottom2, h2, sides2);
  
  //link 4 (si muove con theta4 = [theta[3])
  rotateX(-PI/2);
  rotateX(-theta[3]+PI/2);
  translate(0, 0, (d3z+d4z)/2);
  box(d4x, d4y, d4z);
  
  //link 5 (pinza che si muove con theta5 = theta[4])
  fill(255, 0, 0);
  rotateZ(-theta[4]);
  translate(0, 0, (d5z+d4z)/2);
  box(d5x, d5y, d5z);
  
  //pinza
  stroke(0);
  translate(-theta[5]-d6x/2-posPinza, 0, (d5z+d6z)/2);
  box(d6x, d6y, d6z);
  translate(2*theta[5]+d6x+posPinza, 0, 0);
  box(d6x, d6y, d6z);
  
  //sistema di riferimento associato alla pinza
  translate(+theta[5],0,0);
  translate(-2*theta[5],0,0);
  drawLine5();
  
  popMatrix();
  
  textSize(20);
  fill(255,0,0);  
  text("giunto = ", 10, 20); 
  text(giunto+1, 110, 20);
  
  text("theta1 = ", 10, 70); 
  text(atan2(sin(theta[0]), cos(theta[0]))*180/PI, 110, 70);
  
  text("d2*     = ", 10, 120); 
  text(theta[1], 110, 120);
  
  text("theta3 = ", 10, 170); 
  text(atan2(sin(theta[2]), cos(theta[2]))*180/PI, 110, 170);
  
  text("theta4 = ", 10, 220);
  text(atan2(sin(theta[3]), cos(theta[3]))*180/PI, 110, 220);
  
  text("theta5 = ", 10, 270); 
  text(atan2(sin(theta[4]), cos(theta[4]))*180/PI, 110, 270);
  
  fill(0,255,0);  
  text("coordinata y vista = ", 500, 30); 
  text(eyeY, 700, 30);
  
  //sistema di riferimento associato al link 0
  drawLine0();
  
  //valori delle coordinate e dell'orientamento della pinza rispetto alla terna di base
  d1 = h;
  d2 = d1x/2 + theta[1];
  l3 = d3y;
  d5 = d4z + d5z + d6z;
  
  Px = l3*cos(theta[0])*cos(theta[2]) - d2*sin(theta[0]) - d5*(cos(theta[0])*cos(theta[2])*sin(theta[3]) + cos(theta[0])*cos(theta[3])*sin(theta[2]));
  Py = d2*cos(theta[0]) - d5*(cos(theta[2])*sin(theta[0])*sin(theta[3]) + cos(theta[3])*sin(theta[0])*sin(theta[2])) + l3*cos(theta[2])*sin(theta[0]);
  Pz = d1 - d5*(cos(theta[2])*cos(theta[3]) - sin(theta[2])*sin(theta[3])) - l3*sin(theta[2]);
  
  float[] Rx = {sin(theta[0])*sin(theta[4]) - cos(theta[4])*(cos(theta[0])*sin(theta[2])*sin(theta[3]) - cos(theta[0])*cos(theta[2])*cos(theta[3])),
                -cos(theta[0])*sin(theta[4]) - cos(theta[4])*(sin(theta[0])*sin(theta[2])*sin(theta[3]) - cos(theta[2])*cos(theta[3])*sin(theta[0])),
                -cos(theta[4])*(cos(theta[2])*sin(theta[3]) + cos(theta[3])*sin(theta[2]))};
  
  float[] Ry = {sin(theta[0])*cos(theta[4]) + sin(theta[4])*(cos(theta[0])*sin(theta[2])*sin(theta[3]) - cos(theta[0])*cos(theta[2])*cos(theta[3])),
                -cos(theta[0])*cos(theta[4]) + sin(theta[4])*(sin(theta[0])*sin(theta[2])*sin(theta[3]) - cos(theta[2])*cos(theta[3])*sin(theta[0])),
                sin(theta[4])*(cos(theta[2])*sin(theta[3]) + cos(theta[3])*sin(theta[2]))};
  
  float[] Rz = {-cos(theta[0])*cos(theta[2])*sin(theta[3]) - cos(theta[0])*cos(theta[3])*sin(theta[2]),
                -cos(theta[2])*sin(theta[0])*sin(theta[3]) - cos(theta[3])*sin(theta[0])*sin(theta[2]),
                sin(theta[2])*sin(theta[3]) - cos(theta[2])*cos(theta[3])};
  
  textSize(20);
  fill(255,0,0);
  text("Px      = ", 10, 320);
  text(Px, 110, 320);
  
  text("Py      = ", 10, 370);
  text(Py, 110, 370);
  
  text("Pz      = ", 10, 420);
  text(Pz, 110, 420);
  
  text("Rx      = ", 10, 470);
  text(Rx[0], 110, 470);
  text(Rx[1], 110, 490);
  text(Rx[2], 110, 510);
  
  text("Ry      = ", 10, 540);
  text(Ry[0], 110, 540);
  text(Ry[1], 110, 560);
  text(Ry[2], 110, 580);
  
  text("Rz      = ", 10, 610);
  text(Rz[0], 110, 610);
  text(Rz[1], 110, 630);
  text(Rz[2], 110, 650);
}

void drawCylinder(float top, float bottom, float h, int sides) {
  noStroke();
  float angle = 0;
  float angleIncrement = TWO_PI / sides;
    
  beginShape(QUAD_STRIP);
  for (int i = 0; i < sides + 1; ++i) {
    vertex(top*cos(angle), 0, top*sin(angle));
    vertex(bottom*cos(angle), h, bottom*sin(angle));
    angle += angleIncrement;
  }
  endShape();
  
  //disegno base superiore circolare
  if (top != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);
    
    vertex(0, 0, 0);
    for (int i = 0; i < sides + 1; i++) {
      vertex(top*cos(angle), 0, top*sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }

  //disegno base inferiore circolare
  if (bottom != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);

    // Center point
    vertex(0, h, 0);
    for (int i = 0; i < sides + 1; i++) {
      vertex(bottom*cos(angle), h, bottom*sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }
}

void drawCylinder2(float top2,float bottom2,float h2,float sides2) {
  noStroke();
  float angle = 0;
  float angleIncrement = TWO_PI / sides2;
    
  beginShape(QUAD_STRIP);
  for (int i = 0; i < sides2 + 1; ++i) {
    vertex(-h2, top2*cos(angle), top2*sin(angle));
    vertex(h2, bottom2*cos(angle), bottom2*sin(angle));
    angle += angleIncrement;
  }
  endShape();
  
  //disegno base superiore circolare
  if (top2 != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);
    
    vertex(-h2, 0, 0);
    for (int i = 0; i < sides2 + 1; i++) {
      vertex(-h2, top2*cos(angle), top2*sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }

  //disegno base inferiore circolare
  if (bottom2 != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);

    // Center point
    vertex(h2, 0, 0);
    for (int i = 0; i < sides2 + 1; i++) {
      vertex(h2, bottom2*cos(angle), bottom2*sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }
}

void drawLine0() {
  textSize(12);
  //asse z0
  line(xbase, (ybase+h), 0, xbase, (ybase+h-150), 0);
  line(xbase, (ybase+h-150), 0, (xbase-5), (ybase+h-145), 0);
  line(xbase, (ybase+h-150), 0, (xbase+5), (ybase+h-145), 0);
  fill(0);
  text("z0", (xbase+10), (ybase+h-150), 0);
  
  //asse x0
  line(xbase, (ybase+h), 0, xbase, (ybase+h), 150);
  line(xbase, (ybase+h), 150, (xbase+5), (ybase+h), 145);
  line(xbase, (ybase+h), 150, (xbase-5), (ybase+h), 145);
  fill(0);
  text("x0", (xbase+10), (ybase+h), 150);
  
  //asse y0
  line(xbase, (ybase+h), 0, (xbase+150), (ybase+h), 0);
  line((xbase+150), (ybase+h), 0, (xbase+145), (ybase+h-5), 0);
  line((xbase+150), (ybase+h), 0, (xbase+145), (ybase+h+5), 0);
  fill(0);
  text("y0", (xbase+150), (ybase+h-10), 0);
}

void drawLine5() {
  textSize(12);
  //asse x5
  line((-d6x/2-posPinza), 0, (d5z+d6z-d6x)/2, (-d6x/2-posPinza), -60, (d5z+d6z-d6x)/2);
  line((-d6x/2-posPinza), -60, (d5z+d6z-d6x)/2, (-d6x/2-posPinza)-5, -55, (d5z+d6z-d6x)/2);
  line((-d6x/2-posPinza), -60, (d5z+d6z-d6x)/2, (-d6x/2-posPinza)+5, -55, (d5z+d6z-d6x)/2);
  fill(0);
  text("x5", ((-d6x/2-posPinza)+5), -60, (d5z+d6z-d6x)/2);
  
  //asse z5
  line((-d6x/2-posPinza), 0, (d5z+d6z-d6x)/2, (-d6x/2-posPinza), 0, (d5z+d6z-d6x)/2+60);
  line((-d6x/2-posPinza), 0, (d5z+d6z-d6x)/2+60, (-d6x/2-posPinza-5), 0, (d5z+d6z-d6x)/2+55);
  line((-d6x/2-posPinza), 0, (d5z+d6z-d6x)/2+60, (-d6x/2-posPinza+5), 0, (d5z+d6z-d6x)/2+55);
  fill(0);
  text("z5", (-d6x/2-posPinza)+5, 0, (d5z+d6z-d6x)/2+60);
  
  //asse y5
  line((-d6x/2-posPinza), 0, (d5z+d6z-d6x)/2, (-d6x/2-posPinza)-60, 0, (d5z+d6z-d6x)/2);
  line((-d6x/2-posPinza)-60, 0, (d5z+d6z-d6x)/2, (-d6x/2-posPinza)-55, -5, (d5z+d6z-d6x)/2);
  line((-d6x/2-posPinza)-60, 0, (d5z+d6z-d6x)/2, (-d6x/2-posPinza)-55, 5, (d5z+d6z-d6x)/2);
  fill(0);
  text("y5", ((-d6x/2-posPinza)-60), -5, (d5z+d6z-d6x)/2);  
}

void move(){
 if (giunto == 0) {
    theta[giunto] += segno*.02;
  }
  
  if (giunto == 1) {
    if (theta[giunto] >= ((d1x-d2x)/2) && segno*theta[giunto]-d2x < 0 ) { 
      if (theta[giunto] == ((d1x-d2x)/2) && segno < 0) {
          theta[giunto] = ((d1x-d2x)/2);
      } else theta[giunto] += segno*1;
    }
  }
  
  if (giunto == 2) {
    theta[giunto] += segno*.02;
  }
  
   if (giunto == 3) {
    if (theta[giunto] - (60*PI/180) < 0 || segno < 0) {//devo rientrare nel finecorsa
       if(theta[giunto] + (240*PI/180) >0 || segno >0){
      theta[giunto] += segno*.02;
    } 
    }
  }
  
  if (giunto == 4) {
    theta[giunto] += segno*.02;
  }
  if (giunto == 5){
    if(segno == -1 && theta[giunto]==0){
     theta[giunto] = 0; 
    }
    else if (segno*theta[giunto]-d6y/2 < 0){
    theta[giunto] += segno*1 ;
    }
  }
}
