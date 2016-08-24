// or is it a flock?


/**************************************

sound herd
an experiment with sound by Christine Sunu
August 2016

kiss_fft adapted from the Audio program for L3D cube by Owen Trueblood
October 2014

buzzer code adapted from the Internet Button library for Particle by jenesaisdiq

color sensor code based off of examples from the Adafruit_TCS34725 library

 *************************************/


#include "application.h"
#include <stdarg.h>
#include <math.h>

#include "kiss_fftr.h"
#include "Adafruit_TCS34725.h"
#include "neopixel.h"

#define microphonePin A1
#define touchPin D6
#define speakerPin D2
#define ledPin D3
// wire color pins to D0 (SDA) and D1 (SCL)

#define pop 4                           // population: should be 2, 3, or 4. (it should be 4, can we do 4?)
#define myNote 1                        // the note you play, defined as the coordinate in noteArray
#define calibrateButton D7

#define FFT_SIZE 256
kiss_fftr_cfg fft_cfg;
kiss_fft_scalar *fft_in;
kiss_fft_cpx *fft_out;

uint8_t height[pop][pop];

#define PIXEL_COUNT 1
#define PIXEL_TYPE WS2812B

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, ledPin, PIXEL_TYPE);

String noteArray[8] = {"C5","E5","G5","C6","D5","F5","A5","B5"};
int noteValsBuzz[8] = {107,105,125,119,120,111,100,112};      // to be processed with +/- 1 per indicated value to tune it
//in order: {107,120,105,111,125,100,112,119};
int noteValsVoice[8] = {12,15,18,24,14,17,20,23};    // requires more precision; ambiguity limited to one of these 2 numbers
// in order: {12,14,15,17,18,20,23,24}

// color detection
boolean commonAnode = false;
char szInfo[128];
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
String rgbVal;
String rgbAvg;
int rVal=100;
int gVal=100;
int bVal=100;
int rScale=0;
int gScale=0;
int bScale=0;
int rAvg;
int gAvg;
int bAvg;
int fadeIn=0;
int colorDetect=1;

// touch detection
int squeezeRise = 100;
int lastSqueeze = 0;  // ms of last squeeze
int touchLast = 0; // last touch reading
int touchNow=0;
int peak = 0; // peak reading
int squeezeWaitThreshold = 2500;  // the length of time that you must wait after deciding that the little guy has been squeezed

int count[pop][pop];                    // who each of the buddies can perceive as being nearby
                                        // [my note][my indication of a friend in the room (0 or 1)]
                                        // if count[0]={1,0,0,1,1,0,1,1} then the buddy playing note 0 did not hear notes 1,2, and 5
                                        // we check with the other indicated friends to see if they counted them
                                        // count[3] might be {1,0,1,1,1,0,1,1}
                                        // sum all the counts
                                        // you will get something like {5,4,4,4,2,6,1,4}
                                        // depending on the consensus threshold (defined below) we can say with some confidence that 4 and 6 are not there

int consensus = 1;                      // at least this many in the herd must hear the buddy for there to be consensus that it is there

int allcount[pop];
int sum=0;

int empty[pop][pop];

int timeNow;                            // the current time
int cycleLength=12000;

// time syncing variables
int period=12;
int offset;

// cycle variables
int currentNote;    // the current note you are listening for during the roll call

int cycleType=0;
// cycleType 0 = roll call
// cycleType 1 = data sharing
// cycleType 2 = calling for missing friends
int nextCycle=0;

// state variables
int t;              // the time that has passed in each cycle
int state=0;        // your position in the current cycle (plugs into timing array)
int timing[12];  // the array that determines when you switch between states
int newState=1;     // a flag for when you have just entered a new state
int calibrateLastRead=1;
int calibrateStartTime=0;

String missing[pop];
// stores any missing values
int missingNum; // number of relevant values in missing array

int startTime;  // the time at which we started singing to each other, based on 12 hr time
int startTimeWait=cycleLength;
int onTime;

int timeToSleep=360000;  // go to sleep if no interaction for this amount of ms

float fftVals[128];

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(speakerPin, OUTPUT);
  pinMode(touchPin,INPUT_PULLUP);
  pinMode(calibrateButton, INPUT_PULLUP);

  Particle.function("getColor", getColor);
  Particle.function("setColor", setColor);
  Particle.function("sleep", birdSleep);
  Particle.function("reset", resetStartTime);

  Serial.begin(9600);

  if (tcs.begin()) {
      Serial.println("Found sensor");
  } else {
      Serial.println("No TCS34725 found ... check your connections");
      /*while (1) {Particle.process();}; // halt!*/
  }

  fft_cfg = kiss_fftr_alloc(FFT_SIZE, FALSE, NULL, NULL);
  fft_in = (kiss_fft_scalar*)malloc(FFT_SIZE * sizeof(kiss_fft_scalar));
  fft_out = (kiss_fft_cpx*)malloc(FFT_SIZE / 2 * sizeof(kiss_fft_cpx) + 1);

  clearCount();

  Serial.println("Generating timing...");
  generateTiming();

  Particle.subscribe("herd_roll_data", subscribeRollData, MY_DEVICES);
  Particle.subscribe("herd_color_data", subscribeColorData, MY_DEVICES);
  Particle.subscribe("herd_start_time", subscribeStartTime, MY_DEVICES);

  Serial.println("Syncing time...");

  Particle.syncTime();

  // wait for the turnover of one second
  int currentSec=Time.second();
  while (Time.second()==currentSec) {Particle.process();}

  // get the time at which we switched on
  onTime=1000*(Time.hour()*60*60+Time.minute()*60+Time.second())-millis();

  calibrateTime();

}

void calibrateTime() {
  int timeAtCalibrate=millis();

  if (startTime==0) {
    // find out startTime and then publish it out
    startTime=onTime;
    Serial.println("Publishing StartTime as "+String(startTime));
    Particle.publish("herd_start_time",String(startTime),period,PRIVATE);
  }

  offset=(onTime-startTime)%cycleLength;

  if ((millis()+offset)%cycleLength>=4000) {
    nextCycle=1;
    cyclePrep();
  }
  else {nextCycle=0; cyclePrep();}

}

void loop() {
  if (digitalRead(calibrateButton)==0) {
    if (calibrateLastRead==1) {
      calibrateStartTime=millis();
    }
    if (millis()-calibrateStartTime>3000) {
      birdSleep("");
    }
    else {
      calibrateTime();
    }
    calibrateLastRead=0;
  }
  else {
    calibrateLastRead=1;
  }

  if (digitalRead(touchPin)==0) {
    if (millis()>(lastSqueeze+squeezeWaitThreshold)) {
      lastSqueeze = millis();
      if (colorDetect==1) {
        Serial.println("Getting color...");
        getColor("rgb");
      }
    }
    if (millis()-lastSqueeze>squeezeWaitThreshold+3000) {birdSleep("");}
  }

  if (millis()-lastSqueeze>timeToSleep) {
    // go to sleep!
    birdSleep("");
  }

  else {
    // light if necessary
    if (fadeIn=1) {
      colorFadeIn();
    }
  }

  // check cycle
  t = (millis()+onTime-startTime)%cycleLength;

  // check for sounds
  updateFFT();

  for (int r=0; r<128; r++) {
    screenFFT(r);
    if (cycleType==0) {rollCallListen(r);}
  }

  // remember:   timing={0,1000,2000,3000,4000,4500,7000,8000,9000,10000,11000,12000};

  // roll call cycle
  if (state>=0 && state<4) {
    currentNote=state;
    if (currentNote==myNote) {
      // your turn
      if (newState==1) {
        Serial.println("My turn to sing! I am "+noteArray[myNote]+"!");
        //sing your note
        strip.clear();
        playSong(noteArray[myNote]+",1");
        count[myNote][myNote]=1;
        newState=0;
      }
    }
    else {Particle.process();}
  }

  else if (state==4) {
    if (newState==1) {
      String pubData=String(myNote);
      for (int q=0; q<pop; q++) {
        pubData=pubData+String(count[myNote][q]);
      }
      Serial.println("Now publishing: "+pubData);
      // push data
      Particle.publish("herd_roll_data",pubData,period,PRIVATE);
      newState=0;
    }
  }

  else if (state==5) {
    if (newState==1) {
      processRollData();
      newState=0;
    }
  }

  if (state>=6 && state<10) {
  // if friends missing...
    if (nextCycle==2) {
        if (newState==1) {
          strip.clear();
          String missingFriend=missing[random(0,missingNum)];
          if (random(0,2)) { playSong(missingFriend+",2,"+missingFriend+",2"); }
          else { playSong(missingFriend+",4,"+missingFriend+",2,"+missingFriend+",4"); }
          newState=0;
        }
      }
    //else...
    else if (nextCycle==1) {
      if (state==6) {
        if (newState==1) {
          triumphSong();
          newState=0;
        }
      }
    }
  }
  else if (state==10) {
    if (newState==1) { nextCycle=0; cyclePrep(); newState=0; }
  }

  incrementState();
  touchLast=touchNow;
}

void clearCount() {
  for (int y=0; y<pop; y++) {
    for (int x=0; x<pop; x++) {
      count[y][x]=0;
    }
    allcount[y]=0;
  }
}

void generateTiming() {
  /*Serial.println("Generated timing: ");
  for (int x=0; x<(pop+1); x++) {
    timing[x]=1000*x;
    // this gives 0,1000,2000,3000,4000, etc
    Serial.println(timing[x]);
  }
  timing[pop+1]=timing[pop]+500;        // if you have 8, this is 8500 (end of waiting for data)
  timing[pop+2]=timing[pop]+1500;       // if you have 8, this is 9500 (end of data analysis and note feedback)
  timing[pop+3]=timing[pop]+2000;       // if you have 8, this is 10000 (end of next-cycle-preparations)
  // for 8 notes, gives you:
  // 0,1000,2000,3000,4000,5000,6000,7000,8000,8500,9500,10000

  Serial.println(timing[pop+1]);
  Serial.println(timing[pop+2]);
  Serial.println(timing[pop+3]);
  Serial.println();*/

  // we are going to have the timing be the same every time, for a population of 4...
  // should be:
  // roll call: 0,1000,2000,3000,4000
  // process for 2 seconds: 6000
  // do feedback, 500 ms: 6500
  // prep for sing or friend search, 500 ms: 7000
  // sing or search for friends: 8000,9000,10000,11000

  timing[0]=0;
  timing[1]=1000;
  timing[2]=2000;
  timing[3]=3000;
  timing[4]=4000;
  timing[5]=4500;
  timing[6]=7000;
  timing[7]=8000;
  timing[8]=9000;
  timing[9]=10000;
  timing[10]=11000;
  timing[11]=12000;

  cycleLength=12000;

}

void updateFFT() {
    kiss_fft_scalar pt;

    for(int i=0; i < FFT_SIZE; i++) {
        fft_in[i] = ((float)analogRead(microphonePin))/4096.f;
    }

    kiss_fftr(fft_cfg, fft_in, fft_out);
}

void screenFFT(int r) { // must do this for each value
    float t;
    if (fft_out[r].i == NULL) { fftVals[r]=0; }
    else {
      t = fft_out[r].i;
      if ((r<=10 && r<=30) || (r>=100 && r<130)) { fftVals[r]=t; }
      else {fftVals[r]=0;}
    }
}

void incrementState() {
  if (state!=10) {
    if (t>timing[state+1]) {
      state++;
      newState=1;
      Serial.print("State is now "); Serial.print(state); Serial.print(" at "); Serial.print(t); Serial.print(" until "); Serial.println(timing[state+1]);
    }
  }
  else {
    if (t<timing[1]) {
      state=0;
      newState=1;
      Serial.print("State is now "); Serial.print(state); Serial.print(" at "); Serial.print(t); Serial.print(" until "); Serial.println(timing[state+1]);
    }
  }
}

void cyclePrep() {

  if (nextCycle==0) {
    // clear everything
    // reset the count indicators
    sum=0;
    clearCount();
    colorDetect=1;
    Serial.println("Roll Call cycle...");
  }

  else if (nextCycle==1) {
    // start data share by getting the randomization of notes you will play in what segments
    colorDetect=1;
    Serial.println("Playing indicator notes...");
  }

  else if (nextCycle==2)
  {
    // find out the randomization of missing notes to play
    // first populate the missing array
    int incr=0;
    for (int x=0; x<pop; x++) {
      if (count[myNote][x]==0) {
        missing[incr]=noteArray[x];
        incr++;
      }
    }
    missingNum=incr;
    // then play randomized missing notes in the next cycle
    colorDetect=1;
    Serial.println("Friend call cycle...");
  }

  cycleType=nextCycle;

}


void rollCallListen(int r) {
  if (r==noteValsBuzz[currentNote]) {
    if (fftVals[r]>0.50) {
        // if this is true, then you heard it
        Serial.print(noteArray[currentNote]);
        Serial.print(" (buzzer): ");
        Serial.println(fftVals[r]);
        count[myNote][currentNote]=1;   // add info to 'count' (roll call)
    }
  }
  // comment this in for listening for human voice as well:
  if (r==noteValsVoice[currentNote]) {
    if (fftVals[r]>0.50) {
        Serial.print(noteArray[currentNote]);
        Serial.print(" (voice): ");
        Serial.println(fftVals[r]);
        count[myNote][currentNote]=1;   // add info to 'count' (roll call)
    }
  }
}

void publishRollData() {
  String pubData=String(myNote);
  for (int q=0; q<pop; q++) {
    pubData=pubData+String(count[myNote][q]);
  }
  Serial.println("Now publishing: "+pubData);
  // push data
  Particle.publish("herd_roll_data",pubData,period,PRIVATE);
}

int resetStartTime(String command) {
  char inputStr[64];
  command.toCharArray(inputStr,64);
  startTime=atoi(inputStr);
  // now actually do the reset based on that
  calibrateTime();
}

// do the next part to handle publishes
void subscribeStartTime(const char *event, const char *data) {
  // can get or set
  if (data) {
    // check to see if this time is more recent than the time on record
    // if yes, then send out your time
    // if no or if your time is 0, then adopt the other time as startTime.
    char input[64];
    strcpy(input,data);
    int publishedTime = atoi(input);
    if (startTime>publishedTime) {
      startTime=publishedTime;
      resetStartTime(String(startTime));
      Serial.println("Resetting startTime as "+String(startTime));
    }
    else if (startTime<publishedTime) {
      Particle.publish("herd_start_time",String(startTime),period,PRIVATE);
      Serial.println("Replying with more accurate startTime as "+String(startTime));
    }
    else {
      Serial.println("Our startTime is already "+String(startTime));
    }
  }
}

void subscribeRollData(const char *event, const char *data) {
  if (data) {
    int y = data[0] - '0';
    for (int x=1; x<(pop+1); x++) {
      count[y][x-1]=data[x] - '0';
    }
    if (cycleType==1) {
      Serial.println("Got data from "+String(y)+"!");
      Serial.println(data);
      for (int i=0; i<y+1; i++) { playSong(noteArray[myNote]+",8"); }
    }
  }
}

void subscribeColorData(const char *event, const char *data) {
  if (data) {
    char input[64];
    strcpy(input,data);
    char *p;
    p = strtok(input,",");
    int note=atoi(p);
    p = strtok(NULL,",");
    rAvg = (rAvg+atoi(p))/2;
    p = strtok(NULL,",");
    gAvg = (gAvg+atoi(p))/2;
    p = strtok(NULL,",");
    bAvg = (bAvg+atoi(p))/2;
    p = strtok(NULL,",");
    rgbAvg=String(rAvg)+","+String(gAvg)+","+String(bAvg);
    rVal=rAvg; bVal=bAvg; gVal=gAvg;
    rScale=0; gScale=0; bScale=0;
    fadeIn=1;
    strip.clear();
  }
}

void processRollData() {
  // now we evaluate data
  if (newState==1) {
    Serial.println("Processing data from my roll call...");

    // evaluate the data we have
    Serial.println("Counting sums:");
    for (int x=0; x<pop; x++) {
      for (int y=0; y<pop; y++) {
        allcount[x]=allcount[x]+count[y][x];
        Serial.print(allcount[x]); Serial.print(" ");
      }
      Serial.println();
    }
    Serial.println();

    for (int i=0; i<pop; i++) {
      if (allcount[i]>=consensus) {
        Serial.println("We heard "+String(i)+"!"); sum++;
      }
    }
    if (sum==pop) {
      Serial.println("All present and accounted for.");
      nextCycle=1;
      cyclePrep();
    }

    else {
      // act super distressed by playing the notes for all the ones you think are missing
      Serial.println("Missing " + String(pop-sum) + " of us on the roll call...");
      nextCycle=2;
      cyclePrep();
    }
    newState=0;
  }
}

void triumphSong() {
  // plays fast roll call
  for (int i=0; i<pop; i++) {
    playSong(noteArray[i]+",8");
  }
  // share the data
  if (rgbVal!=NULL) {Particle.publish("herd_color_data",rgbVal,period,PRIVATE);}

}


// from the Particle Internet Button library by jenesaisdiq

void playSong(String song){
    char inputStr[200];
    song.toCharArray(inputStr,200);

    Serial.println(inputStr);

    char *note = strtok(inputStr,",");
    char *duration = strtok(NULL,",");
    playNote(note,atoi(duration));

    while(duration != NULL){
        note = strtok(NULL,",");
        /*Serial.println(note);*/
        duration = strtok(NULL,", \n");
        /*Serial.println(duration);*/
        //if(atoi(duration) <= 0){
        //    break;
        //}
        playNote(note,atoi(duration));
    }
}

void playNote(String note, int duration){
    int noteNum = 0;
    int octave = 5;
    int freq = 256;

     //if(9 - int(command.charAt(1)) != null){
    char octavo[5];
    String tempString = note.substring(1,2);
    tempString.toCharArray(octavo,5);
    octave = atoi(octavo);
    //}

    if(duration != 0){
        duration = 900/duration;
    }

    switch(note.charAt(0)){
        case 'C':
            noteNum = 0;
            break;
        case 'D':
            noteNum = 2;
            break;
        case 'E':
            noteNum = 4;
            break;
        case 'F':
            noteNum = 5;
            break;
        case 'G':
            noteNum = 7;
            break;
        case 'A':
            noteNum = 9;
            break;
        case 'B':
            noteNum = 11;
            break;
        case 'R':          // Rest note
            octave = -1;
            break;
        default:
            break;
            //return -1;
    }

    // based on equation at http://www.phy.mtu.edu/~suits/NoteFreqCalcs.html and the Verdi tuning
    // fn = f0*(2^1/12)^n where n = number of half-steps from the reference frequency f0
    freq = float(256*pow(1.05946,(     12.0*(octave-4)        +noteNum)));
    //          C4^  (2^1/12)^    12 half-steps in an octave      ^how many extra half-steps within that octave, 0 for a C

    tone(speakerPin,int(freq),duration);
    delay(duration);
    noTone(speakerPin);
    //return freq;
}


// color code
int getColor(String command) {
  uint16_t clear, red, green, blue;
  tcs.setInterrupt(false);      // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);  // turn off LED

  uint32_t sum = clear;
  float rRaw, gRaw, bRaw;

  /*Serial.print("clear: ");
  Serial.println((int)clear);

  Serial.print("red: ");
  Serial.println((int)red);
  Serial.print("blue: ");
  Serial.println((int)blue);
  Serial.print("green: ");
  Serial.println((int)green);*/


  rRaw = red; rRaw /= sum;
  gRaw = green; gRaw /= sum;
  bRaw = blue; bRaw /= sum;
  rRaw *= 256; gRaw *= 256; bRaw *= 256;

  sprintf(szInfo, "%d,%d,%d", (int)rRaw, (int)gRaw, (int)bRaw);

  Serial.println(szInfo);

  rgbVal = String((int)rRaw)+","+String((int)gRaw)+","+String((int)bRaw);

  if (command=="r") {
    return rRaw;
  }
  else if (command=="g") {
    return gRaw;
  }
  else if (command=="b") {
    return bRaw;
  }
  else if (command=="rgb") {
    rVal=(int)rRaw;
    gVal=(int)gRaw;
    bVal=(int)bRaw;
    rScale=0;
    gScale=0;
    bScale=0;
    fadeIn=1;
    return 0;
  }
  else {
    return -1;
  }
}

void colorFadeIn() {
  if (rScale>=100 && gScale>=100 && bScale>=100) {
    fadeIn=0;
    setColor(String(rVal)+","+String(gVal)+","+String(bVal));
  }
  else {
    if (rScale<100) {
      if (rScale<20) {rScale++;}
      else {
        rScale=rScale+rScale/5;
      }
    }
    if (gScale<gVal) {
      if (gScale<20) {gScale++;}
      else {
        gScale=gScale+gScale/5;
      }
    }
    if (bScale<bVal) {
      if (bScale<20) {bScale++;}
      else {
        bScale=bScale+bScale/5;
      }
    }

    /*Serial.println("relaying RGB value: "+String(rScale)+","+String(gScale)+","+String(bScale));
    Serial.println("out of RGB value: "+String(rVal)+","+String(gVal)+","+String(bVal));*/
    setColor(String(rScale*rVal/100)+","+String(gScale*gVal/100)+","+String(bScale*bVal/100));
  }
}

int setColor(String command) {
  char inputStr[64];
  command.toCharArray(inputStr,64);
  char *p = strtok(inputStr,",");
  int rd = atoi(p);
  p = strtok(NULL,",");
  int gn = atoi(p);
  p = strtok(NULL,",");
  int bl = atoi(p);
  p = strtok(NULL,",");
  colorAll(strip.Color(rd, gn, bl), 10);
}

int birdSleep(String command) {
  System.sleep(D6,FALLING);
}

// Set all pixels in the strip to a solid color, then wait (ms)
void colorAll(uint32_t c, uint8_t wait) {
  uint16_t i;

  strip.clear();

  for(i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
  delay(wait);
}

int maxOf(int x, int y, int z) {
  if ((x==max(x,y) && x==max(x,z)) || (x==y && x==max(x,z)) || (x==z && x==max(x,y))) {
    return x;
  }
  else if ((y==max(x,y) && y==max(y,z)) || (y==x && y==max(y,z)) || (y==z && y==max(x,y))) {
    return y;
  }
  else if ((z==max(x,z) && y==max(y,z)) || (z==x && z==max(x,z)) || (z==y && z==max(y,z))) {
    return z;
  }
  else {
    return 0;
  }
}
