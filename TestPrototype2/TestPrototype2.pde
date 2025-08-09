 /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  


/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 


/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 3;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           rendering_force                     = false;
/* end device block definition *****************************************************************************************/

/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 60;

/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                      = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           pos_ee                              = new PVector(0, 0);
PVector           f_ee                                = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 16.25; 
float         worldPixelWidth                     = worldWidth*pixelsPerCentimeter;
float         worldPixelHeight                    = worldHeight*pixelsPerCentimeter;

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;


/* Initialization of virtual tool */
PImage sImg;
HVirtualCoupling  s;

/* Words bodies*/
FCircle c1;
FCircle c2;
FCircle c3;
FCircle c4;
FLine fLine;




//Khushi's variables
boolean startLine = false;
boolean startCircle = false;
boolean finishLine = false;
PVector last_pos;
PVector curr_pos = new PVector(0,0);
PVector force = new PVector(0,0);


/* General variables*/
PFont f;
int mode=1;
float x;
float y;

//interface variables
FBox lineMenu;
PImage lineMenuImg;
PShape square;
FBox circleMenu;
PImage circleMenuImg;
PShape square2;

//experiment sequence
boolean isCircle=false;
boolean isLine=false;
boolean isGuiding=false;
ArrayList<PVector> lineStartCoords;
ArrayList<PVector> lineEndCoords;
ArrayList<PVector> circleStartCoords;
ArrayList<PVector> circleEndCoords;
//indexes
int currentIndex = 0;
int currentCircleIndex = 0;

int interval = 100000; // 10 seconds in milliseconds
int lastUpdateTime = 0;

//pen tool
PImage pencilTool;
CopyOnWriteArrayList<PVector> points;
boolean drawing = false;


/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 650);
  
  lineMenuImg = loadImage("icons/line.png");
  circleMenuImg = loadImage("icons/circle.png");
  sImg = loadImage("icons/cursor.png");
  pencilTool = loadImage("icons/pencil.png");
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[2], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();

  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
  
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  f                   = createFont("Arial", 16, true);
  textFont(f);
  textAlign(LEFT, CENTER);
  points = new CopyOnWriteArrayList<PVector>();

  //menu options
  square = createShape(RECT, 0, 0, 80, 80);
  lineMenu = new FBox(2, 2);
  lineMenu.setPosition(worldWidth -3, (worldHeight / 2)-2);
  //lineMenu.setFill(0,50,0);
  lineMenu.setStatic(true);
  lineMenu.setStroke(255, 0, 0);
  world.add(lineMenu);
  lineMenuImg.resize((int)(hAPI_Fisica.worldToScreen(2)), (int)(hAPI_Fisica.worldToScreen(2)));
  lineMenu.attachImage(lineMenuImg); 
  
  
  //text("practice lines", (worldWidth-3),(worldHeight/2));
  //fill(0,0,100);

  square2 = createShape(RECT, 0, 0, 80, 80);
  circleMenu = new FBox(2, 2);
  circleMenu.setPosition(worldWidth -3, (worldHeight / 2)+2);
  //lineMenu.setFill(0,50,0);
  circleMenu.setStatic(true);
  circleMenu.setStroke(255, 0, 0);
  world.add(circleMenu);
  circleMenuImg.resize((int)(hAPI_Fisica.worldToScreen(2)), (int)(hAPI_Fisica.worldToScreen(2)));
  circleMenu.attachImage(circleMenuImg); 

  // Initialize and populate the coordinates ArrayList
  lineStartCoords = new ArrayList<PVector>();
  lineStartCoords.add(new PVector(3, 4));
  lineStartCoords.add(new PVector(4, 4));
  lineStartCoords.add(new PVector(6, 3));
  lineStartCoords.add(new PVector(9, 5));
  lineStartCoords.add(new PVector(11, 4));
  lineStartCoords.add(new PVector(3, 6));
  //lineStartCoords.add(new PVector(400, 250));
  //lineStartCoords.add(new PVector(500, 300));

  lineEndCoords = new ArrayList<PVector>();
  lineEndCoords.add(new PVector(12, 4));
  lineEndCoords.add(new PVector(6, 11));
  lineEndCoords.add(new PVector(6, 12));
  lineEndCoords.add(new PVector(7, 13));
  lineEndCoords.add(new PVector(11, 14));
  lineEndCoords.add(new PVector(14, 6));
  //lineEndCoords.add(new PVector(400, 250));
  //lineEndCoords.add(new PVector(500, 300));

  circleStartCoords = new ArrayList<PVector>();
  circleStartCoords.add(new PVector(7, 7));
  circleStartCoords.add(new PVector(9, 9));
  circleStartCoords.add(new PVector(9, 10));
  circleStartCoords.add(new PVector(8, 7));
  circleStartCoords.add(new PVector(11, 9));
  circleStartCoords.add(new PVector(10, 8));
  //circleStartCoords.add(new PVector(400, 250));
  //circleStartCoords.add(new PVector(500, 300));

  circleEndCoords = new ArrayList<PVector>();
  circleEndCoords.add(new PVector(9, 7));
  circleEndCoords.add(new PVector(14, 9));
  circleEndCoords.add(new PVector(15, 10));
  circleEndCoords.add(new PVector(11, 7));
  circleEndCoords.add(new PVector(15, 9));
  circleEndCoords.add(new PVector(13, 8));
  //circleEndCoords.add(new PVector(400, 250));
  //circleEndCoords.add(new PVector(500, 300));
  
  c1 = new FCircle(0.15);
  c1.setPosition(lineStartCoords.get(0).x,lineStartCoords.get(0).y);
  //c1.setFill(200,0,0);
  c1.setNoStroke();
  c1.setSensor(true);
  c1.setStatic(true);
  world.add(c1);
  
  c2 = new FCircle(0.15);
  c2.setPosition(lineEndCoords.get(0).x,lineEndCoords.get(0).y);
  //c2.setFill(100,200,0);
  c2.setNoStroke();
  c2.setSensor(true);
  c2.setStatic(true);
  world.add(c2);
  
  c3 = new FCircle(0.18);
  c3.setPosition(circleStartCoords.get(0).x,circleStartCoords.get(0).y);//7,9
  c3.setFill(150,100,0);
  c3.setSensor(true);
  c3.setStatic(true);
  //world.add(c3);
  
  c4 = new FCircle(0.18);
  c4.setPosition(circleEndCoords.get(0).x,circleEndCoords.get(0).y);//10,9
  c4.setFill(100,200,0);
  c4.setSensor(true);
  c4.setStatic(true);
  //world.add(c4);

  fLine = new FLine(3,4,10,4);
  
  //world.add(fLine);

  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.5)); 
  s.h_avatar.setDensity(2); 
  s.h_avatar.setFill(0,60,0); 
  s.h_avatar.setNoStroke();
  sImg.resize((int)(hAPI_Fisica.worldToScreen(0.5)), (int)(hAPI_Fisica.worldToScreen(0.5)));
  pencilTool.resize((int)(hAPI_Fisica.worldToScreen(0.5)), (int)(hAPI_Fisica.worldToScreen(0.5)));
  s.h_avatar.attachImage(sImg); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), (0.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/





/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  background(255);
  textFont(f, 20);
  fill(0, 0, 100);
  textAlign(CENTER);
  shape(square, 840, 205);
  shape(square2, 840, 365);
  text("practice lines", (worldWidth-3)*pixelsPerCentimeter,((worldHeight/2)-0.5)*pixelsPerCentimeter);
  text("practice circles", (worldWidth-3)*pixelsPerCentimeter,((worldHeight/2)+3.5)*pixelsPerCentimeter);
  if(isLine && currentIndex>0){
    text("s", lineStartCoords.get(currentIndex-1).x*pixelsPerCentimeter, lineStartCoords.get(currentIndex-1).y*pixelsPerCentimeter+1.5);
    text("e", lineEndCoords.get(currentIndex-1).x*pixelsPerCentimeter, lineEndCoords.get(currentIndex-1).y*pixelsPerCentimeter+1.5);
  }
  if(isCircle && currentCircleIndex>0){
    text("<-r->", (circleStartCoords.get(currentCircleIndex-1).x+0.9)*pixelsPerCentimeter, circleStartCoords.get(currentCircleIndex-1).y*pixelsPerCentimeter+1.5);
    text("s", circleEndCoords.get(currentCircleIndex-1).x*pixelsPerCentimeter, circleEndCoords.get(currentCircleIndex-1).y*pixelsPerCentimeter+1.5);
  }
  //text("Mode: "+Integer.toString(mode), width/2, height/2);
  
  stroke(0);
  strokeWeight(2);
  noFill();
  beginShape();
  //println("p len: "+points.size());
  for(PVector point : points){
    vertex(point.x*pixelsPerCentimeter, point.y*pixelsPerCentimeter);
  }
  endShape();

  world.draw(); 

  
}
/* end draw section ****************************************************************************************************/

void keyPressed(){
  if(key == 'n'){
    if(isCircle){
      updateCircleCoordinatesPositions();
    }
    if(isLine){
      updateLineCoordinatesPositions();
    }
    
  }
}


/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  private int lastSwitchTime = 0;
  private boolean switchFlag = true;
  private PVector tempForce = new PVector(0,0);
  private float randomNumber =0;

  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    rendering_force = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      pos_ee.set(widgetOne.get_device_position(angles.array()));
      pos_ee.set(pos_ee.copy().mult(200));
      //println("pos x: "+pos_ee.x + " tool: "+(edgeTopLeftX+worldWidth/2-(pos_ee).x+2));
      //println("pos y: "+pos_ee.y+ " tool: "+(edgeTopLeftY+(pos_ee).y-3.46));
      x = edgeTopLeftX+worldWidth/2-(pos_ee).x+2;
      y= edgeTopLeftY+(pos_ee).y-3.46;
      s.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+2, edgeTopLeftY+(pos_ee).y-3.46); 
      s.updateCouplingForce();

      //if(!startLine){
        //println("fee x: "+f_ee.x);
        //println("fee y: "+f_ee.y);
        f_ee.set(-s.getVCforceX(), s.getVCforceY());
        //println("vc x: "+ -s.getVCforceX());
        //println("vc y: "+ s.getVCforceY());
        f_ee.div(20000); //
      //}
    }
    /*if (millis() - lastSwitchTime > 500) {
      switchFlag = !switchFlag;
      lastSwitchTime = millis();
    }*/
    
    if(startLine || startCircle){
      s.h_avatar.attachImage(pencilTool); 
    }
    else{
      s.h_avatar.attachImage(sImg);
    }

    if(startLine){
      points.add(new PVector(x,y));
      PVector projectedPoint = getClosestPointOnLine(lineStartCoords.get(currentIndex-1), lineEndCoords.get(currentIndex-1), new PVector(x,y));//3,4 10,4
      //PVector projectedPoint = getClosestPointOnLine(new PVector(3,4), new PVector(10,4), new PVector(x,y));
      if(isGuiding){
        f_ee.set(calculateForceTowardPath(projectedPoint, x, y));
      }
      else{
        tempForce.set(calculateDisturbanceTowardPath(projectedPoint, x, y));
        if(tempForce.mag()<=0.4){
          f_ee.set(tempForce);
        }
      }
      
      //println("proj pos x: "+projectedPoint.x + " xe:"+x);
      //println("proj pos y: "+projectedPoint.y + " ye:"+y);  
      f_ee.set(graphics_to_device(f_ee));

    }

    if(startCircle){
      points.add(new PVector(x,y));
      PVector projectedPoint = getClosestPointOnCircle(circleStartCoords.get(currentCircleIndex-1), (float)(circleEndCoords.get(currentCircleIndex-1).x-circleStartCoords.get(currentCircleIndex-1).x), new PVector(x,y));//3,4 10,4
      //PVector projectedPoint = getClosestPointOnCircle(new PVector(7,7),3,new PVector(x,y));
      if(isGuiding){
        f_ee.set(calculateForceTowardPath(projectedPoint, x, y));
      }
      else{
        tempForce.set(calculateForceTowardPath(projectedPoint, x, y));
        if(tempForce.mag()<=0.3){
          f_ee.set(tempForce);
        }
      }
      
      f_ee.set(graphics_to_device(f_ee));
    }
    
    torques.set(widgetOne.set_device_torques(f_ee.array()));
    widgetOne.device_write_torques();

    
    
    //select shape
    if(s.h_avatar.isTouchingBody(circleMenu)){
      //println("touching circle");
      isCircle = true;
      isLine = false;
      c1.setFill(250,0,0);
      c2.setFill(250,0,0);
      currentCircleIndex=0;
      updateCircleCoordinatesPositions();
      //c1.setPosition(circleStartCoords.get(0).x,circleStartCoords.get(0).y);//7,9
      //c2.setPosition(circleEndCoords.get(0).x,circleEndCoords.get(0).y);//10,9     
    }
    if(s.h_avatar.isTouchingBody(lineMenu)){
      //println("touching line");
      isCircle = false;
      isLine = true;
      c1.setFill(0,250,0);
      c2.setFill(0,250,0);
      currentIndex=0;
      updateLineCoordinatesPositions();
      //c1.setPosition(lineStartCoords.get(0).x,lineStartCoords.get(0).y);
      //c2.setPosition(lineEndCoords.get(0).x,lineEndCoords.get(0).y);     
    }

    if(s.h_avatar.isTouchingBody(c1)){
      randomNumber = random(1);
      if (randomNumber < 0.5) {
        isGuiding=true;
      } else {
        isGuiding=false;
      }
      if(isLine){
        println("is line "+ isGuiding);
          //lastUpdateTime = millis();
        startLine = true;
        //finishLine = false;
        drawing = true;
        points.clear();
        points.add(new PVector(x,y));
      }
    }
    if(s.h_avatar.isTouchingBody(c2)){
      if(isLine){
        points.add(new PVector(x,y));
        startLine = false;
      }
      if(isCircle){
        randomNumber = random(1);
        if (randomNumber < 0.5) {
          isGuiding=true;
        } else {
          isGuiding=false;
        }
        startCircle = !startCircle;
        points.clear();
        points.add(new PVector(x,y));
      }
        
        //lastUpdateTime = millis();
        
        //finishLine = true;
        //println("on circle "+ s.getToolPositionX() + " y: "+s.getToolPositionY());
        //println("on circle "+ s.getAvatarPositionX() + " y: "+s.getAvatarPositionY());
    }
    if(s.h_avatar.isTouchingBody(c4)){
        startCircle = !startCircle;
        points.clear();
        points.add(new PVector(x,y));
    }
    
    
    world.step(1.0f/1000.0f);
  
    rendering_force = false;
  }
}
/* end simulation section **********************************************************************************************/


/* helper functions section, place helper functions here ***************************************************************/
void updateLineCoordinatesPositions() {
  
  if (currentIndex >= lineStartCoords.size()) {
    currentIndex = 0; // Reset to loop through coordinates
  }
  
  // Update the positions of the circles
  PVector startPos = lineStartCoords.get(currentIndex);
  PVector endPos = lineEndCoords.get(currentIndex);
  
  c1.setPosition(startPos.x, startPos.y);
  c2.setPosition(endPos.x, endPos.y);
  currentIndex++;
}

void updateCircleCoordinatesPositions() {
  
  if (currentCircleIndex >= circleStartCoords.size()) {
    currentCircleIndex = 0; // Reset to loop through coordinates
  }
  
  // Update the positions of the circles
  PVector startPos = circleStartCoords.get(currentCircleIndex);
  PVector endPos = circleEndCoords.get(currentCircleIndex);
  
  c1.setPosition(startPos.x, startPos.y);
  c2.setPosition(endPos.x, endPos.y);
  currentCircleIndex++;
}

PVector calculateForceTowardPath(PVector travelledPt, float xe, float ye) {
  PVector forceDirection = PVector.sub(travelledPt, new PVector(xe, ye));
  float distance = forceDirection.mag();
  //println("distance: " + distance);
  PVector forceFeedback = new PVector(0, 0);

  // no force feedback if it's too close to the path, partly to reduce oscillation
  if(distance>0.025){ //0.025 is perf
    forceDirection.normalize();
    forceFeedback=forceDirection.mult(3.5*distance);//4.75//1.9 is perfect // 0.6 for disturbance
    //println("fx bef: "+ forceFeedback.x + " fy bef: "+forceFeedback.y);
    //forceFeedback.mult(-1);
    
    //println("fx aft: "+ forceFeedback.x + " fy aft: "+forceFeedback.y);
  }
  return forceFeedback;
}

PVector calculateDisturbanceTowardPath(PVector travelledPt, float xe, float ye) {
  PVector forceDirection = PVector.sub(travelledPt, new PVector(xe, ye));
  float distance = forceDirection.mag();
  //println("distance: " + distance);
  PVector forceFeedback = new PVector(0, 0);

  // no force feedback if it's too close to the path, partly to reduce oscillation
  if(distance>0.025){ //0.025 is perf
    forceDirection.normalize();
    forceFeedback=forceDirection.mult(3.5*distance);//4.75//1.9 is perfect // 0.6 for disturbance
    //println("fx bef: "+ forceFeedback.x + " fy bef: "+forceFeedback.y);
    forceFeedback.mult(-1);
    
    //println("fx aft: "+ forceFeedback.x + " fy aft: "+forceFeedback.y);
  }
  return forceFeedback;
}

PVector getClosestPointOnCircle(PVector circleCenter, float radius, PVector currentPos) {
    // Step 1: Calculate the vector from the circle's center to the current position
    PVector direction = PVector.sub(currentPos, circleCenter);
    
    // Step 2: Normalize this vector
    direction.normalize();
    
    // Step 3: Scale the normalized vector by the radius
    direction.mult(radius);
    
    // Step 4: Add this scaled vector to the circle's center to get the closest point
    PVector closestPoint = PVector.add(circleCenter, direction);
    
    return closestPoint;
}

PVector getClosestPointOnLine(PVector lineStart, PVector lineEnd, PVector currentLocation) {  
  PVector lineVector = PVector.sub(lineEnd, lineStart);
  PVector pointVector = PVector.sub(currentLocation, lineStart);
  
  // Calculate the projection of the point onto the line
  float t = PVector.dot(pointVector, lineVector) / PVector.dot(lineVector, lineVector);
  
  // Clamp t to the range [0, 1] to handle the segment case
  t = constrain(t, 0, 1);
  
  // Calculate the closest point
  PVector closestPoint = PVector.add(lineStart, PVector.mult(lineVector, t));
  //float projectionLength = pointVector.dot(lineVector) / lineVector.magSq();
  //projectionLength = constrain(projectionLength, 0, 1);
  //return PVector.add(start, PVector.mult(lineVector, projectionLength));
  
  return closestPoint;
}

PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}

PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}
/* end helper functions section ****************************************************************************************/
