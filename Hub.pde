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
int[][] x = new int[2][100];
int[][] y = new int[2][100];
int[][] z = new int[2][100];
int[] temp = {108, 95};
int[] o2 = {97, 100};
int[] bpm = {163, 83};
char[] smoke = {'N', 'Y'};
//float angle = 0.0;
int gridSize = 40;
float xm, ym, zm;
int is_up = 1;
int plot_count = 0;
int plot_full = 0;

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
  //if (count % 10 == 0) {
  //  x[0] = int(random(999));
 //   x[1] = int(random(999));
 // }
  text("ID    X        Y       Z      Temp     O2    BPM    Smoke    Distress      Map", 450, 300);
  text(" "+id[0]+"   "+nf(x[0][plot_count], 3)+"    "+nf(y[0][plot_count], 3)+"     "+nf(z[0][plot_count], 2)+"      "+nf(temp[0], 3)+"      "+nf(o2[0], 3)+"    "+nf(bpm[0], 3)+"         "+smoke[0], 450, 375);
  text(" "+id[1]+"   "+nf(x[1][plot_count], 3)+"    "+nf(y[1][plot_count], 3)+"     "+nf(z[1][plot_count], 2)+"      "+nf(temp[1], 3)+"      "+nf(o2[1], 3)+"    "+nf(bpm[1], 3)+"         "+smoke[1], 450, 450);
  
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
    cam = new PeasyCam(this, 700);
    cam.setMinimumDistance(500);
    cam.setMaximumDistance(1250);
  }
  
  public void draw() {
    if (count % 10 == 0 && plot_count < 100) {
      plot_count++;
      if (plot_count == 100) {
        plot_count = 1;
        plot_full = 1;
      }
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
      x[0][plot_count] = int(random(500));
      y[0][plot_count] = int(random(500));
      z[0][plot_count] = int(random(10));
    }
 
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

 
 
  if (plot_full == 0) {
    for (int j = 0; j < plot_count; j++) {
      plot(x[0][j],y[0][j],z[0][j], color(255,0,0));
    }
    plot(x[0][plot_count],y[0][plot_count],z[0][plot_count], color(0,255,0)); //<>//
  }
  else {
    for (int j = 0; j < 100; j++) {
      if (j != plot_count) {
        plot(x[0][j],y[0][j],z[0][j], color(255,0,0));
      }
    }
      plot(x[0][plot_count],y[0][plot_count],z[0][plot_count], color(0,255,0));
  }
      
  popMatrix();
 
  printCamera();
  
  text("Count = "+(count / 10),250,520,0);

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
