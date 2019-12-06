//Copyright 2019 Michael White
//License Notice at Notice.md

import controlP5.*;
import processing.serial.*;
import peasy.*;

class myPacket {
  public int pid;
  public int px;
  public int py;
  public int pz;
  public int ptemp;
  public int po2;
  public int pbpm;
  
  public myPacket(int pid, int px, int py, int pz, int ptemp, int po2, int pbpm) {
    this.pid = pid;
    this.px = px;
    this.py = py;
    this.pz = pz;
    this.ptemp = ptemp;
    this.po2 = po2;
    this.pbpm = pbpm;
  }
}
  
  

ControlP5 cp5;
Serial port;
Map map;
PeasyCam cam;

myPacket packet;
PFont font;
Button a;
Button b;
Button c;
Button d;
Button e;
Button f;
Button g;
int col1 = color(0,255,0);
int col2 = color(0,255,0);
int col3 = color(0,255,0);
int col4 = color(255,0,0);
int count = 0;
int[] id = {0, 1};
int[][] x = new int[2][100];
int[][] y = new int[2][100];
int[][] z = new int[2][100];
int[] temp = {23, 95};
int[] o2 = {97, 100};
int[] bpm = {163, 83};
char[] smoke = {'N', 'Y'};
char[] safe = {'Y', 'Y'};
//float angle = 0.0;
int gridSize = 40;
float xm, ym, zm;
boolean isUp = true;
int plot_count = 0;
boolean isPlotFull = false;
int idTrack = 0;
int[] input = new int[7];

void settings() {
  fullScreen();
}

void setup(){
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 100; j++) {
      x[i][j] = 0;
      y[i][j] = 0;
      z[i][j] = 0;
    }
  }
  printArray(Serial.list());
  port = new Serial(this, "COM3", 115200);
  cp5 = new ControlP5(this);
  map = new Map();
  font = createFont("Arial", 24, true);
  textFont(font, 40);
  a = cp5.addButton("Distress1")
    .setPosition(1400,215)
    .setSize(125,50)
    .setFont(font)
    .setLabel("Send")
    .setColorBackground(col1)
    ;
  b = cp5.addButton("Distress2")
    .setPosition(1400,290)
    .setSize(125,50)
    .setFont(font)
    .setLabel("Send")
    .setColorBackground(col2)
    ;
  c = cp5.addButton("Reset")
    .setPosition(1400,380)
    .setSize(125,50)
    .setFont(font)
    ;
  d = cp5.addButton("Map1")
    .setPosition(1590,215)
    .setSize(125,50)
    .setFont(font)
    .setLabel("Map")
    .setColorBackground(col3)
    ;
  e = cp5.addButton("Map2")
    .setPosition(1590,290)
    .setSize(125,50)
    .setFont(font)
    .setLabel("Map")
    .setColorBackground(col3)
    ;
  f = cp5.addButton("Generate")
    .setPosition(1300,70)
    .setSize(135,50)
    .setFont(font)
    ;
    
  g = cp5.addButton("Extreme")
    .setPosition(1450,70)
    .setSize(125,50)
    .setFont(font)
    ;
}

void draw(){
  a.setColorBackground(col1);
  b.setColorBackground(col2);
  d.setColorBackground(col3);
  e.setColorBackground(col4);
  background(0,0,0);
  for (int i = 0; i < 2; i++) {
    if (temp[i] > 100) {
      safe[i] = 'N';
    }
    else {
      safe[i] = 'Y';
    }
  }

  //if (count % 10 == 0) {
  //  x[0] = int(random(999));
 //   x[1] = int(random(999));
 // }
  fill(255);
  text("ID    X        Y       Z      Temp     O2     BPM     Safe     Distress      Map", 450, 180);
  if (safe[0] == 'N') {
    fill(255,0,0);
  }
  text(" "+(id[0] + 1)+"   "+nf(x[0][plot_count], 3)+"    "+nf(y[0][plot_count], 3)+"     "+nf(z[0][plot_count], 2)+"      "+nf(temp[0], 3)+"      "+nf(o2[0], 3)+"    "+nf(bpm[0], 3)+"         "+safe[0], 450, 255);
  if (safe[1] == 'N') {
    fill(255,0,0);
  }
  else {
    fill(255);
  }
  text(" "+(id[1] + 1)+"   "+nf(x[1][plot_count], 3)+"    "+nf(y[1][plot_count], 3)+"     "+nf(z[1][plot_count], 2)+"      "+nf(temp[1], 3)+"      "+nf(o2[1], 3)+"    "+nf(bpm[1], 3)+"         "+safe[1], 450, 330);
  
  rectMode(CORNERS);
  noFill();
  stroke(255);
  rect(425,130,1750,355);
  line(425,205,1750,205);
  line(500,130,500,355);
  line(610,130,610,355);
  line(720,130,720,355);
  line(820,130,820,355);
  line(975,130,975,355);
  line(1085,130,1085,355);
  line(1210,130,1210,355);
  line(1375,130,1375,355);
  line(1550,130,1550,355);
  line(425,275,1750,275);
 // count++;
}



void Distress1() {
 col1 = color(255,0,0);
 port.write('D');
}

void Distress2() {
  col2 = color(255,0,0);
  port.write('O');
}

void Reset() {
  col1 = color(0,255,0);
  col2 = color(0,255,0);
  Map1();
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 100; j++) {
      x[i][j] = 0;
      y[i][j] = 0;
      z[i][j] = 0;
    }
    temp[i] = 0;
    o2[i] = 0;
    bpm[i] = 0;
  }
  count = 0;
}

void Map1() {
  if (idTrack != 0) {
    col3 = color(0,255,0);
    col4 = color(255,0,0);
    idTrack = 0;
  }
}

void Map2() {
  if (idTrack != 1) {
    col3 = color(255,0,0);
    col4 = color(0,255,0);
    idTrack = 1;
  }
}

void Generate() {
  port.write('R');
  String myString = port.readStringUntil('\n');
  
  if (myString != null) {
    plot_count++;
    if (plot_count == 100) {
      plot_count = 1;
      isPlotFull = true;
    }
    input = int(split(myString, '\t'));
    id[0] = input[0];
    x[id[0]][plot_count] = input[1];
    y[id[0]][plot_count] = input[2];
    z[id[0]][plot_count] = input[3];
    temp[id[0]] = input[4];
    o2[id[0]] = input[5];
    bpm[id[0]] = input[6];
    count++;
    x[1][plot_count] = int(random(500));
    y[1][plot_count] = int(random(500));
    z[1][plot_count] = int(random(10));
  }
}

void Extreme() {
  temp[0] = 200;
}

class Map extends PApplet {
  
  public Map() {
    super();
    PApplet.runSketch(new String[]{this.getClass().getName()}, this);
  }
  
  public void settings() {
    size(700,650,P3D);
    xm = width/2;
    ym = height/2;
    zm = 0;
  }
  
  public void setup() {
    surface.setTitle("Map");
    surface.setAlwaysOnTop(true);
    background(0);
    cam = new PeasyCam(this, 700);
    cam.setMinimumDistance(500);
    cam.setMaximumDistance(1250);
  }
  
  public void draw() {
    //if (plot_count < 100) {
      //plot_count++;
    //if (plot_count == 100) {
    //  plot_count = 1;
    //  isPlotFull = true;
   // }
      /*if (is_up == 1) {
        x[0][plot_count] = x[0][plot_count-1] + 10;
        if (x[0][plot_count] > 500) {
          is_up = 0;
        }
      }
      else {
        x[0][plot_count] = x[0][plot_count-1] - 10;
        if (x[0][plot_count] < 10) {
          is_up = 1;
        }
      }*/
      /*x[0][plot_count] = int(random(500));
      y[0][plot_count] = int(random(500));
      z[0][plot_count] = int(random(10));
      x[1][plot_count] = int(random(500));
      y[1][plot_count] = int(random(500));
      z[1][plot_count] = int(random(10));*/
    //}
 
    rotateX(-.001);
    rotateY(-.001);
    background(0);
    translate(0,0,0);
 
 
    pushMatrix();
 

    stroke(0,100,0); 
    line(0, 0, 0, 150, 0, 0);
    fill(0,100,0);
    text("X Axis",140,-5,0);
   
    stroke(200);

 
 
    stroke(255,0,0);
    line(0, 0, 0, 0, 150, 0);
   
    pushMatrix();
    rotate(-HALF_PI);
    fill(255,0,0);
    text("Y Axis",-160,-5,0);
    popMatrix();
   
  stroke(0,0,255);
  line(0, 0, 0, 0, 0, 150);
  pushMatrix();
    rotateY(-HALF_PI);
    fill(0,0,255);
    text("Z Axis",140,-5,0);
    popMatrix();
 
 
 
  stroke(200);
  for (int i = 0; i <= 500; i += 20) {
    line(i, 0, 0, i, 0, 500);
    line(0, i, 0, 0, i, 500);
    line(0, 0, i, 0, 500, i);
    line(0, 0, i, 500, 0, i);
    line(i, 0, 0, i, 500, 0);
    line(0, i, 0, 500, i, 0);
  }

 
 
  if (!isPlotFull) {
    for (int j = 0; j < plot_count; j++) {
      plot(x[idTrack][j],y[idTrack][j],z[idTrack][j], color(255,0,0));
    }
    plot(x[idTrack][plot_count],y[idTrack][plot_count],z[idTrack][plot_count], color(0,255,0));
  }
  else {
    for (int j = 0; j < 100; j++) {
      if (j != plot_count) {
        plot(x[idTrack][j],y[idTrack][j],z[idTrack][j], color(255,0,0));
      }
    }
      plot(x[idTrack][plot_count],y[idTrack][plot_count],z[idTrack][plot_count], color(0,255,0));
  }
      
  popMatrix();
 
  printCamera();
  
  text("Count = "+count,20,530,0);
  textSize(32);
  text("X = "+x[idTrack][plot_count]+"  Y = "+y[idTrack][plot_count]+"  Z = "+z[idTrack][plot_count],20,560,0);

  }
  
  public void plot (int xplot, int yplot, int zplot, int colour) {
    translate(xplot, yplot, zplot * 50);
    noStroke();
    lights();
    fill(color(colour));
    sphere(5);
    translate(-xplot, -yplot, -(zplot * 50));
  }
}
