import controlP5.*;
import processing.serial.*;
import peasy.*;

ControlP5 cp5;
Serial port;
Map map;
PeasyCam cam;;

PFont font;
Button a;
Button b;
Button c;
int col1 = color(0,255,0);
int col2 = color(0,255,0);
int count = 0;
int[] id = {1, 2};
int[] x = {0, 0};
int[] y = {5, 507};
int[] z = {90, 45};
int[] temp = {108, 95};
int[] o2 = {97, 100};
int[] bpm = {163, 83};
char[] smoke = {'N', 'Y'};
//float angle = 0.0;
int gridSize = 40;
float xm, ym, zm;

void settings() {
  fullScreen();
}

void setup(){
  printArray(Serial.list());
  port = new Serial(this, "COM3", 9600);
  cp5 = new ControlP5(this);
  map = new Map();
  font = createFont("Arial", 24, true);
  textFont(font, 40);
  a = cp5.addButton("Distress1")
    .setPosition(1400,335)
    .setSize(125,50)
    .setFont(font)
    .setLabel("Send")
    .setColorBackground(col1);
    ;
  b = cp5.addButton("Distress2")
    .setPosition(1400,410)
    .setSize(125,50)
    .setFont(font)
    .setLabel("Send")
    .setColorBackground(col2);
    ;
  c = cp5.addButton("Reset")
    .setPosition(1400,500)
    .setSize(125,50)
    .setFont(font)
    ;
}

void draw(){
  a.setColorBackground(col1);
  b.setColorBackground(col2);
  background(0,0,0);
  if (count % 10 == 0) {
    x[0] = int(random(999));
    x[1] = int(random(999));
  }
  text("ID    X        Y       Z      Temp     O2    BPM    Smoke    Distress      Map", 450, 300);
  text(" "+id[0]+"   "+nf(x[0], 3)+"    "+nf(y[0], 3)+"    "+nf(z[0], 3)+"     "+nf(temp[0], 3)+"      "+nf(o2[0], 3)+"    "+nf(bpm[0], 3)+"         "+smoke[0], 450, 375);
  text(" "+id[1]+"   "+nf(x[1], 3)+"    "+nf(y[1], 3)+"    "+nf(z[1], 3)+"     "+nf(temp[1], 3)+"      "+nf(o2[1], 3)+"    "+nf(bpm[1], 3)+"         "+smoke[1], 450, 450);
  
  rectMode(CORNERS);
  noFill();
  stroke(255);
  rect(425,250,1750,475);
  line(425,325,1750,325);
  line(500,250,500,475);
  line(610,250,610,475);
  line(720,250,720,475);
  line(820,250,820,475);
  line(975,250,975,475);
  line(1085,250,1085,475);
  line(1210,250,1210,475);
  line(1375,250,1375,475);
  line(1550,250,1550,475);
  count++;
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
}

class Map extends PApplet {
  
  public Map() {
    super();
    PApplet.runSketch(new String[]{this.getClass().getName()}, this);
  }
  
  public void settings() {
    size(700,700,P3D);
    xm = width/2;
    ym = height/2;
    zm = 0;
  }
  
  public void setup() {
    surface.setTitle("Map");
    background(0);
    cam = new PeasyCam(this, 500);
    cam.setMinimumDistance(100);
    cam.setMaximumDistance(1000);
  }
  
  public void draw() {
    color blue = color (0,0,255);
    color yellow = color (255,255,0);
 
    rotateX(-.001);
    rotateY(-.001);
    background(255);
    translate(0,0,0);
 
 
    pushMatrix();
      fill(200);
    rect(0,0,125,125);
 
    // asse x
    stroke(0,100,0); 
    line(0, 0, 0, 150, 0, 0);
    fill(0,100,0);
    text("X Axis",140,-5,0);
   
    stroke(200);
    line(0, 0, 10, 100, 0, 10);
    line(0, 0, 20, 100, 0, 20);
    line(0, 0, 30, 100, 0, 30);
    line(0, 0, 40, 100, 0, 40);
    line(0, 0, 50, 100, 0, 50);
 
 
    stroke(255,0,0);
    line(0, 0, 0, 0, 150, 0);
   
    pushMatrix();
    rotate(-HALF_PI);
    fill(255,0,0);
    text("Y Axis",-160,-5,0);
    popMatrix();
   
   
   
    stroke(200);
    line(0, 0, 10, 0, 100, 10);
    line(0, 0, 20, 0, 100, 20);
    line(0, 0, 30, 0, 100, 30);
    line(0, 0, 40, 0, 100, 40);
    line(0, 0, 50, 0, 100, 50);
   
   
 
 
 
  stroke(0,0,255);
  line(0, 0, 0, 0, 0, 150);
  pushMatrix();
    rotateY(-HALF_PI);
    fill(0,0,255);
    text("Z Axis",140,-5,0);
    popMatrix();
 
 
 
  stroke(200);
  line(10, 0, 0, 10, 0, 100);
  line(20, 0, 0, 20, 0, 100);
  line(30, 0, 0, 30, 0, 100);
  line(40, 0, 0, 40, 0, 100);
  line(50, 0, 0, 50, 0, 100);
  line(0, 10, 0, 0, 10, 100);
  line(0, 20, 0, 0, 20, 100);
  line(0, 30, 0, 0, 30, 100);
  line(0, 40, 0, 0, 40, 100);
  line(0, 50, 0, 0, 50, 100);
 
 
  translate(10, 10, 10);
  noStroke();
  lights();
  fill(0,255,0);
  sphere(5);
 
 
  translate(25, 10, 50);
  noStroke();
  lights();
  fill(blue);
  sphere(5);
 
  translate(25, 30, 10);
  noStroke();
  lights();
  fill(255,0,0);
  sphere(5);
 
  translate(75, 10, 50);
  noStroke();
  lights();
  fill(yellow);
  sphere(5);
 
  translate(0,0,50);
  popMatrix();
 
  printCamera();

  }
}
