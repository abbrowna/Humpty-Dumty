/*
Name:		Linefollower.ino
Created:	3/8/2017 5:54:13 PM
Author:	Anthony_Brown &Ricky Nyairo
*/

/**/
//#include <MD_TCS230.h> // This is library is used to control the RGB sensor mounted at the very top of the robot arm.
#include <Servo.h> // The robot arm has 4 Servo motors which are contrled by this arduio servo library. 
/*NOTE: The the Servo motors and the RGB sensors to be used in the same code a changes MUST be made to the servo library
due to conflicting timers. in the Servotimers.h file, coment out '_usetimer5' and also from the line that follows.*/

//SERVOS	declare sevo objects.
Servo panServo;
Servo tiltServo;
Servo clawServo;
Servo clawTilt;

//SERVO VARIABLES
//these variables contain the default  specific positions, in terms of degrees, for each of the servos that make up the arm to bring a certain arm config. 
//Each servo can rotate its shaft 180 degrees with a precition of 1 degree
//int pos;
int panDef = 80;
int panDrop = 160;
int clawDef = 50;
int clawClamp = 60;
int clawOpen = 5;
int clawRelease = 35;
int tiltDef = 70;
int tiltForward = 90;
int tiltBack = 100;
int tiltPick = 145;
int bin1Pos = 110;
int bin2Pos = 45;
int ballot3 = 0;
int clawTiltBin = 13;
int clawTiltDef = 18;
int clawTiltDrop = 120;
int clawTiltPick = 130;
//the following variables hold the pin numbers for the arduino pins connected to the L293D terminals.
//the L293D is an IC that controls the 4 motors bidirectionally in the $ wheel drive system.
//LEFT MOTOR
int enablepinA = 7;
int logic1A = 28;
int logic2A = 40;
//RIGHT MOTOR
int enablepinB = 8;
int logic3A = 24;
int logic4A = 22;
//IR sensors are used to detect the line. Two in-front and on either side of the line.
//Another two at between the two wheel axles agin on either side.
//INFRA RED
int LI = 21;
int LO = 18;
int RO = 20;
int RI = 19;

//RGB	The RGB LED used to show what the robot is thinking and doing by different colors.
//these variables declare the pins to which the rgb is connected to the Mega
int Rled = 10;
int Gled = 12;
int Bled = 13;
volatile bool Rstate = 0;
volatile bool Gstate = 0;
volatile bool ledstate = 0;

//VELOCITY ENCODERS		they keep track of the wheel movements
const int lsig = 2;		//mega pins
const int rsig = 3;
volatile unsigned lvcount = 0;	//varables to store distance in terms of VE ticks
volatile unsigned rvcount = 0;
unsigned storedlvcount;
unsigned storedrvcount;
//these hold values used by the velocity oncoders to perform a specific maneuver.
int turnticksleft = 10;
int turnticksright = 11;
int fullturnright = 21;
int fullturnleft = 20;
int setspeed = 255; //this variable stores the overall speed of the bot. aif changed, it should also be changed after it is reset at the top of ramp
//NOTE: To high of a setpeed may cause line sensors to not detect aline in time and either overcorrect or not correct at all.
int rotspeed = 255;
int reversepeed = 200;
int correction = 4;
int bigcorrection = 6;
int backtrack = 2;
int startoff = 6;
int frocorrect = 4;
int paperdistance = 1;
int redbox = 7;
int yellowbox = 7;
int bluebox = 7;
bool started = 0;

//OBJECTIVES	this structure holds the information associated with each desired objective.
//an objective is where you want the bot to go nex. it entails X & Y coordinates and the direction to face.
struct objective {
	int x;
	int y;
	int direction;
};

//Required objects for the Specified gamefield. These can be edited to suit other gamefields or other obstacles.
objective ver_in = { 6,4,90 };
objective ver_out = { 11,4,90 };
objective ver = { 9,4,90 };
objective redpapproach = { 18,1,180 };
objective redpaper = { 18,0,180 };
objective yellowpapproach = { 17,1,90 };
objective yellowpaper = { 17,0,180 };
objective bluepapproach = { 16,1,90 };
objective bluepaper = { 16,0,180 };
objective booth_in = { 17,7,270 };
objective booth = { 15,7,270 };
//objective booth_out = { 13,7,270 };
objective rampbase = { 10,8,270 };
//objective ramptop = { 2,8,270 };
objective ramprest = { 1,7,270 };
objective boxready = { 1,8,270, };
objective pitstop = { 7,0,270 };
objective start = { 0,0,0, };

//LOCATORS	holds  the location and direction the bot is facing at any one time
struct pos {
	int x = 0;
	int y = 0;
	int direction = 90;
};
pos location;

//FUNCTION DEFINITIONS
//these define varias robot motion. Refer to datasheet of L293D.
//Optional parameter of the distance to cover as determined by the velocity encoders.

void curve_right(int left)
{
	analogWrite(enablepinA, left);
	analogWrite(enablepinB, setspeed);
	digitalWrite(logic1A, HIGH);
	digitalWrite(logic3A, HIGH);
	digitalWrite(logic2A, LOW);
	digitalWrite(logic4A, LOW);
}
void curve_left(int right)
{
	analogWrite(enablepinA, setspeed);
	analogWrite(enablepinB, right);
	digitalWrite(logic1A, HIGH);
	digitalWrite(logic3A, HIGH);
	digitalWrite(logic2A, LOW);
	digitalWrite(logic4A, LOW);
}
void moveforward(int ticks = -1, bool stop = 1) //aditional optional parameter gives more control as to wether it should stop or not
{
	if (ticks == -1)
	{
		analogWrite(enablepinA, setspeed);
		analogWrite(enablepinB, setspeed);
		digitalWrite(logic1A, HIGH);
		digitalWrite(logic3A, HIGH);
		digitalWrite(logic2A, LOW);
		digitalWrite(logic4A, LOW);
		return;
	}
	else
	{
		lvcount = 0; //resets te elocity encoder values to start counting afresh.
		rvcount = 0;
		analogWrite(enablepinA, setspeed);
		analogWrite(enablepinB, setspeed);
		digitalWrite(logic1A, HIGH);
		digitalWrite(logic3A, HIGH);
		digitalWrite(logic2A, LOW);
		digitalWrite(logic4A, LOW);
		while (lvcount < ticks || rvcount < ticks) {}
		if (stop == 1)
			stationary();
	}
	//enable the motor driver
}

void reverse(int ticks = -1)
{
	if (ticks == -1)
	{
		analogWrite(enablepinA, reversepeed);
		analogWrite(enablepinB, reversepeed);
		digitalWrite(logic1A, LOW);
		digitalWrite(logic3A, LOW);
		digitalWrite(logic2A, HIGH);
		digitalWrite(logic4A, HIGH);
		return;
	}
	else
	{
		lvcount = 0;
		rvcount = 0;
		analogWrite(enablepinA, reversepeed);
		analogWrite(enablepinB, reversepeed);
		digitalWrite(logic1A, LOW);
		digitalWrite(logic3A, LOW);
		digitalWrite(logic2A, HIGH);
		digitalWrite(logic4A, HIGH);
		while (lvcount <
			ticks || rvcount < ticks) {
		}
		revstationary();
	}
}
void freeroll_stop() //roll to a complete stop
{
	digitalWrite(enablepinA, LOW);
	digitalWrite(enablepinB, LOW);
}
void stationary() { //use power to stop instanteneously
	reverse();
	delay(100);
	digitalWrite(enablepinA, LOW);
	digitalWrite(enablepinB, LOW);
}
void revstationary() { //use power to stop instantenouesly after reersing.
	moveforward();
	delay(100);
	digitalWrite(enablepinA, LOW);
	digitalWrite(enablepinB, LOW);
}

void rotateright_specific()
{
	while (digitalRead(RI))
	{
		analogWrite(enablepinA, rotspeed);
		analogWrite(enablepinB, rotspeed);
		digitalWrite(logic1A, HIGH);
		digitalWrite(logic3A, LOW);
		digitalWrite(logic2A, LOW);
		digitalWrite(logic4A, HIGH);
	}
	stationary();
}
void rotateleft_specific()
{
	while (digitalRead(LI))
	{
		analogWrite(enablepinA, rotspeed);
		analogWrite(enablepinB, rotspeed);
		digitalWrite(logic1A, LOW);
		digitalWrite(logic3A, HIGH);
		digitalWrite(logic2A, HIGH);
		digitalWrite(logic4A, LOW);
	}
	stationary();
}
void rotateleft_axial(int ticks = -1) //rotate the robot by turning right set forward and left set backwards.
{
	if (ticks == -1)
	{
		analogWrite(enablepinA, rotspeed);
		analogWrite(enablepinB, rotspeed);
		digitalWrite(logic1A, LOW);
		digitalWrite(logic3A, HIGH);
		digitalWrite(logic2A, HIGH);
		digitalWrite(logic4A, LOW);
		return;
	}
	else
	{
		lvcount = 0;
		rvcount = 0;
		analogWrite(enablepinA, rotspeed);
		analogWrite(enablepinB, rotspeed);
		digitalWrite(logic1A, LOW);
		digitalWrite(logic3A, HIGH);
		digitalWrite(logic2A, HIGH);
		digitalWrite(logic4A, LOW);
		while (rvcount < ticks || rvcount < ticks) {}
		freeroll_stop();
	}
}
void rotateright_axial(int ticks = -1)
{
	if (ticks == -1)
	{
		analogWrite(enablepinA, rotspeed);
		analogWrite(enablepinB, rotspeed);
		digitalWrite(logic1A, HIGH);
		digitalWrite(logic3A, LOW);
		digitalWrite(logic2A, LOW);
		digitalWrite(logic4A, HIGH);
		return;
	}
	else
	{
		lvcount = 0;
		rvcount = 0;
		analogWrite(enablepinA, rotspeed);
		analogWrite(enablepinB, rotspeed);
		digitalWrite(logic1A, HIGH);
		digitalWrite(logic3A, LOW);
		digitalWrite(logic2A, LOW);
		digitalWrite(logic4A, HIGH);
		while (rvcount < ticks || lvcount < ticks) {}
		freeroll_stop();
	}
}
void rotateleft_radial(int ticks = -1) //keeps one set of wheels stationary while the other is moving. gived softer turn.
{
	if (ticks == -1)
	{
		digitalWrite(enablepinA, LOW);
		analogWrite(enablepinB, rotspeed);
		digitalWrite(logic1A, LOW);
		digitalWrite(logic3A, HIGH);
		digitalWrite(logic2A, HIGH);
		digitalWrite(logic4A, LOW);
		return;
	}
	else
	{
		lvcount = 0;
		rvcount = 0;
		digitalWrite(enablepinA, LOW);
		analogWrite(enablepinB, rotspeed);
		digitalWrite(logic1A, LOW);
		digitalWrite(logic3A, HIGH);
		digitalWrite(logic2A, HIGH);
		digitalWrite(logic4A, LOW);
		while (rvcount <= ticks) {}
		freeroll_stop();
	}
}
void rotateright_radial(int ticks = -1)
{
	if (ticks == -1)
	{
		analogWrite(enablepinA, rotspeed);
		digitalWrite(enablepinB, LOW);
		digitalWrite(logic1A, HIGH);
		digitalWrite(logic3A, LOW);
		digitalWrite(logic2A, LOW);
		digitalWrite(logic4A, HIGH);
		return;
	}
	else
	{
		lvcount = 0;
		rvcount = 0;
		analogWrite(enablepinA, rotspeed);
		digitalWrite(enablepinB, LOW);
		digitalWrite(logic1A, HIGH);
		digitalWrite(logic3A, LOW);
		digitalWrite(logic2A, LOW);
		digitalWrite(logic4A, HIGH);
		while (lvcount <= ticks) {}
		freeroll_stop();
	}
}

//These two are interrupt functions called by the velocity encoders every time they trigger.
//being an interupt, they will pause whatever else is going on so they can excecute first.
void lvisr()
{
	lvcount++;
}
void rvisr()
{
	rvcount++;
}
//as the nae suggests, it makes the bot move to the passed objective.
void goto_objective(struct objective obj)
{
	/*location holds the bots current location at any one time and is used to decide the required motion
	in order to reach the objective according to the following logic*/
	int intersectsx = obj.x - location.x;
	int intersectsy = obj.y - location.y;
	if (intersectsx < 0)			// turn to either 90 or 270 degrees to do the X axis first
	{
		if (location.direction == 0) {
			//reverse(backtrack);
			rotateleft_axial(turnticksleft);
		}
		else if (location.direction == 90) {
			rotateright_axial(fullturnright);
		}
		else if (location.direction == 180) {
			reverse(backtrack);
			rotateright_axial(turnticksright);
		}
		location.direction = 270;
	}
	else if (intersectsx>0)
	{
		if (location.direction == 0) {
			reverse(backtrack);
			rotateright_axial(turnticksright);
		}
		else if (location.direction == 180) {
			reverse(backtrack);
			rotateleft_axial(turnticksleft);
		}
		else if (location.direction == 270) {
			rotateleft_axial(fullturnleft);
		}
		location.direction = 90;
	}
	location.x = obj.x;
	followline(abs(intersectsx)); //Execute x  part of objective first.

	if (intersectsy < 0) {			//tuen to either 0 or 180 to degreees for the y part of the objective
		if (location.direction == 90) {
			reverse(backtrack);
			rotateright_axial(turnticksright);
		}
		else if (location.direction == 270) {
			reverse(backtrack);
			rotateleft_axial(turnticksleft);
			reverse(backtrack);
		}
		else if (location.direction == 0)
		{
			rotateright_axial(fullturnright);
			reverse(backtrack);
		}
		location.direction = 180;
	}
	else if (intersectsy > 0) {
		if (location.direction == 90) {
			reverse(backtrack);
			rotateleft_axial(turnticksleft);
			reverse(backtrack);
		}
		else if (location.direction == 270) {
			reverse(backtrack);
			rotateright_axial(turnticksright);
			reverse(backtrack);
		}
		else if (location.direction == 180)
		{
			rotateleft_axial(fullturnleft);
			reverse(backtrack);
		}
		location.direction = 0;
	}
	location.y = obj.y;
	stationary();
	delay(500);
	followline(abs(intersectsy)); //Execute Y part of objectve next.

	//Turn to the direction specified by the objective.
	if (abs(location.direction - obj.direction) == 180) {
		rotateright_axial(fullturnright);
	}
	else if ((location.direction - obj.direction) == -90) {
		reverse(backtrack); //reversing alitle first makes sure if falls perfectly in possition when it turns;
		rotateright_axial(turnticksright);
		//rotateright_specific();
	}
	else if ((location.direction - obj.direction) == 90) {
		reverse(backtrack); //reversing may also remove any miscounting errors
		rotateleft_axial(turnticksleft);
		//rotateleft_specific();
	}
	else if ((location.direction - obj.direction) == -270) {
		reverse(backtrack);
		rotateleft_axial(turnticksleft);
		//rotateleft_specific();
	}
	else if ((location.direction - obj.direction) == 270) {
		reverse(backtrack);
		rotateright_axial(turnticksright);
		//rotateright_specific();
	}
	location.direction = obj.direction;
}

void followline(int sects) //parameters are the number of intersections where an intersection is a cross formed by two white lines
{
	if (sects == 0)
		return;
	int intersects = 0;
	while (intersects <= sects)//less than or equal to2 because it likely started off on an intersect. 
	{
		//stay in this loop untill we have crossed the required number of intersections.
		bool lo = !digitalRead(LO) && !digitalRead(LO);
		bool li = !digitalRead(LI) && !digitalRead(LI); //exclamations flip logic so that a white line gives a one. to the line sensors. 
		bool ri = !digitalRead(RI) && !digitalRead(RI);
		bool ro = !digitalRead(RO) && !digitalRead(RO);

		if (lo && ro) //if both outer sensors trigger, intersection reached, increment appropriately
		{
			digitalWrite(Rled, 0);
			digitalWrite(Gled, 0);
			digitalWrite(Bled, 1);
			//reverse(backtrack);
			stationary();
			intersects++;
			if (intersects != sects + 1)
			{
				if (started)
				{
					followlinedistance(startoff);
				}
				else
					moveforward(startoff, 0);
			}

		}
		else if (!lo && !ro) //if both outer sensor are triggerd...
		{
			if (!li && !ri) //if both inner ones are untriggered... moveforward
			{
				moveforward();
				digitalWrite(Rled, 0);
				digitalWrite(Gled, 0);
				digitalWrite(Bled, 0);
			}
			else if (li) //if only one is triggers, we are getting off the line and we ahould correct
			{
				rotateleft_radial();
				digitalWrite(Rled, 1);
				digitalWrite(Gled, 0);
				digitalWrite(Bled, 0);
			}
			else
			{
				rotateright_radial();
				digitalWrite(Rled, 0);
				digitalWrite(Gled, 1);
				digitalWrite(Bled, 0);
			}
		}
		else if (!lo || !ro) // if only one of the outer ones are triggered, then we are still of the line and we need to correct.
		{
			if (!li && !ri)
			{
				moveforward();
				digitalWrite(Rled, 0);
				digitalWrite(Gled, 0);
				digitalWrite(Bled, 0);
			}
			else if (li)
			{
				rotateleft_radial();
				digitalWrite(Rled, 1);
				digitalWrite(Gled, 0);
				digitalWrite(Bled, 0);
			}
			else
			{
				rotateright_radial();
				digitalWrite(Rled, 0);
				digitalWrite(Gled, 1);
				digitalWrite(Bled, 0);
			}
		}
	}
}
//follows a line for a certain distance rather than for a number of intersections
//NOTE: this function does not update the bot position so a reverse of the same distance is necessary to keep location accuracy.
void followlinedistance(int dist)
{
	lvcount = 0;
	rvcount = 0;
	if (dist == 0)
		return;
	while (lvcount <= dist || rvcount <= dist)//less than or equal 2 because it likely started off on an intersect
	{
		bool lo = !digitalRead(LO) && !digitalRead(LO);
		bool li = !digitalRead(LI) && !digitalRead(LI);
		bool ri = !digitalRead(RI) && !digitalRead(RI);
		bool ro = !digitalRead(RO) && !digitalRead(RO);

		if (!li && !ri)
		{
			moveforward();
			digitalWrite(Rled, 0);
			digitalWrite(Gled, 0);
			digitalWrite(Bled, 0);
		}
		else if (li)
		{
			rotateleft_radial();
			digitalWrite(Rled, 1);
			digitalWrite(Gled, 0);
			digitalWrite(Bled, 0);
		}
		else
		{
			rotateright_radial();
			digitalWrite(Rled, 0);
			digitalWrite(Gled, 1);
			digitalWrite(Bled, 0);
		}
	}
	stationary();
}


//SERVO FUNCTIONS
void okota(int binPos) {
	if (binPos > 0) {
		//moveServo(panServo, panDef, panDef);//pan to pick ballot
		//moveServo(tiltServo, tiltServo.read(), tiltForward);//tilt down
		moveServo(clawServo, clawServo.read(), clawOpen);
		moveServo(clawTilt, clawTilt.read(), clawTiltPick); //tilt the clamp
		delay(500);
		moveServo(clawServo, clawServo.read(), clawClamp);//grab object
		//begin placing ballot
		moveServo(clawTilt, clawTilt.read(), clawTiltDef); //tilt clamp back
		moveServo(panServo, panServo.read(), binPos); //pan to bin 1
		moveServo(tiltServo, tiltServo.read(), tiltBack); //tilt arm back towards bin
		moveServo(clawServo, clawServo.read(), clawRelease); //release object to bin
		moveServo(clawTilt, clawTilt.read(), clawTiltDef); //tilt back clamp
		moveServo(tiltServo, tiltServo.read(), tiltDef); //tilt back arm
		moveServo(panServo, panServo.read(), panDef);
	}
	else {
		//moveServo(tiltServo, tiltServo.read(), tiltForward);//tilt down
		moveServo(clawServo, clawServo.read(), clawOpen);
		moveServo(clawTilt, clawTilt.read(), clawTiltPick); //tilt the clamp
		moveServo(clawServo, clawServo.read(), clawClamp);//grab object
		moveServo(clawTilt, clawTilt.read(), clawTiltDef); //tilt clamp back
		moveServo(tiltServo, tiltServo.read(), tiltDef); //tilt arm back to default
	}
}
void dropOff(int binPos) {
	if (binPos > 0) {
		if (binPos == bin2Pos)
		{
			moveServo(clawServo, clawServo.read(), clawRelease); //open claw
			moveServo(panServo, panServo.read(), binPos);//pan to pick ballot
			//moveServo(clawTilt, clawTilt.read(), 130);//tilt the clamp lower for picking
			moveServo(tiltServo, tiltServo.read(), tiltBack);//tilt down  
			delay(1000);
			moveServo(clawTilt, clawTilt.read(), clawTiltBin);
			moveServo(clawServo, clawServo.read(), clawClamp);//grab object
			//moveServo(clawTilt, clawTilt.read(), clawTiltDef); //tilt clamp back
			moveServo(tiltServo, tiltServo.read(), tiltDef); //tilt arm back to default
			moveServo(panServo, panServo.read(), 45); //pan to the side
			//moveServo(tiltServo, tiltServo.read(), 160); //tilt towards the voting bucket
			moveServo(clawTilt, clawTilt.read(), clawTiltDrop); //tilt clamp for drop off 
			moveServo(clawServo, clawServo.read(), clawRelease); //open clamp
			armDefault();
		}
		else if (binPos == bin1Pos)
		{
			moveServo(clawServo, clawServo.read(), clawRelease); //open claw
			moveServo(panServo, panServo.read(), binPos);//pan to pick ballot
			//moveServo(clawTilt, clawTilt.read(), 130);//tilt the clamp lower for picking
			moveServo(tiltServo, tiltServo.read(), tiltBack);//tilt down  
			delay(1000);
			moveServo(clawTilt, clawTilt.read(), clawTiltBin);
			moveServo(clawServo, clawServo.read(), clawClamp);//grab object
			//moveServo(clawTilt, clawTilt.read(), clawTiltDef); //tilt clamp back
			moveServo(tiltServo, tiltServo.read(), tiltDef); //tilt arm back to default
			moveServo(panServo, panServo.read(), panDef); //pan to the side
			//moveServo(tiltServo, tiltServo.read(), 160); //tilt towards the voting bucket
			moveServo(clawTilt, clawTilt.read(), clawTiltDrop); //tilt clamp for drop off 
			moveServo(clawServo, clawServo.read(), clawRelease); //open clamp
			armDefault();
		}
	}
	else {
		moveServo(tiltServo, tiltServo.read(), tiltDef); //tilt to default
		moveServo(panServo, panServo.read(), panDrop); //pan to to the side
													   //moveServo(tiltServo, tiltServo.read(), tiltBack); //tilt closer to voting bucket
		moveServo(clawTilt, clawTilt.read(), clawTiltDrop); //tilt claw to drop off into bin
		moveServo(clawServo, clawServo.read(), clawRelease);
		armDefault();
	}
}
void armDefault() {
	moveServo(clawServo, clawServo.read(), clawDef);
	moveServo(clawTilt, clawTilt.read(), clawTiltDef); //tilt claw to drop off to bin
	moveServo(panServo, panServo.read(), panDef); //pan to default
	moveServo(tiltServo, tiltServo.read(), tiltDef); //tolt to default
}
void moveServo(Servo myServo, int startPos, int stopPos) {
	if (startPos < stopPos) {
		for (int i = startPos; i <= stopPos; i++) {
			myServo.write(i);
			delay(10);
		}
	}
	else {
		for (int i = startPos; i >= stopPos; i--) {
			myServo.write(i);
			delay(10);
		}
	}
}
void setup() {
	Serial.begin(115200); //start the serial monitor if required fro debugging purposes
						  //SERVO SETUP
	clawServo.attach(4); // connect the decalere servos to the mega pins to which they are attached.
	clawTilt.attach(5);
	tiltServo.attach(6);
	panServo.attach(9);
	clawServo.write(clawDef);
	clawTilt.write(clawTiltDef);
	tiltServo.write(tiltDef);
	panServo.write(panDef);
	delay(100);

	//MOTORS
	pinMode(enablepinA, OUTPUT);
	pinMode(enablepinB, OUTPUT);
	pinMode(logic1A, OUTPUT);
	pinMode(logic3A, OUTPUT);
	pinMode(logic2A, OUTPUT);
	pinMode(logic4A, OUTPUT);

	//VELOCITY ENCODER
	pinMode(lsig, INPUT);
	pinMode(rsig, INPUT);
	attachInterrupt(digitalPinToInterrupt(lsig), lvisr, RISING);
	attachInterrupt(digitalPinToInterrupt(rsig), rvisr, RISING);

	//IR TRANSMITTER/RECEIVER
	pinMode(LO, INPUT);
	pinMode(LI, INPUT);
	pinMode(RI, INPUT);
	pinMode(RO, INPUT);

	pinMode(Rled, OUTPUT);
	pinMode(Gled, OUTPUT);
	pinMode(Bled, OUTPUT);
	delay(3000);
}

//void loop mainly entails a sequence of objectives to go to.
//tasks too complicated to be an objective are also done sequentialy here such as arm movement.
void loop()
{
	/*un-omment the following and comment out the rest of void loop to see what IR sensor values are being retrieved.
	It is usefull for debugging and recalibration of the voltage comparetors' potentiometer, used with the IR sensors.
	white should retrieve 1 and black 0*/

	//bool lo = !digitalRead(LO) && !digitalRead(LO);
	//bool li = !digitalRead(LI) && !digitalRead(LI);
	//bool ri = !digitalRead(RI) && !digitalRead(RI);
	//bool ro = !digitalRead(RO) && !digitalRead(RO);

	//Serial.print(lo);
	//Serial.print(" // ");
	//Serial.print(li);
	//Serial.print(" // ");
	//Serial.print(ri);
	//Serial.print(" // ");
	//Serial.println(ro);
	//delay(500);

	//okota(bin2Pos);

	followline(2);
	location.x = 1;
	started = 1;

	goto_objective(ver_in);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(500);

	goto_objective(ver);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(500);

	goto_objective(bluepaper);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(500);
	okota(bin1Pos);	//put the blue ballot paper into slot two
	turnticksleft = 11; //change turnticks manually

	goto_objective(bluepapproach);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(500);

	goto_objective(yellowpapproach);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(500);

	goto_objective(yellowpaper);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(500);
	//followlinedistance(paperdistance);
	okota(bin2Pos);//put the yellow paper in bin 2
	turnticksleft = 10; //resett turnticks

	//goto_objective(redpaper);
	//digitalWrite(Rled, 0);
	//digitalWrite(Gled, 1);
	//digitalWrite(Bled, 1);
	//delay(1000);
	////moveforward(paperdistance);
	//followlinedistance(paperdistance);
	//okota(ballot3); //red ballot
	////reverse(paperdistance + 3);

	goto_objective(booth_in);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(500);

	goto_objective(booth);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(500);

	//goto_objective(booth_out);
	//digitalWrite(Rled, 0);
	//digitalWrite(Gled, 1);
	//digitalWrite(Bled, 1);
	//delay(1000);

	//goto_objective(rampbase);
	//digitalWrite(Rled, 0);
	//digitalWrite(Gled, 1);
	//digitalWrite(Bled, 1);
	//delay(1000);

	//setspeed = 255; // increase in power to climb ramp with ease

	goto_objective(ramprest);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(2000); //give a chance to recover from climbing the ramp

	goto_objective(boxready);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(500);

	//setspeed = 200; //reset power at end of ramp to a more maneagable value. should be change if setepeed is changed in decleration.
	followlinedistance(4);

	//goto_objective(boxready);
	//digitalWrite(Rled, 0);
	//digitalWrite(Gled, 1);
	//digitalWrite(Bled, 1);
	//delay(1000);

	//followlinedistance(redbox);
	//dropOff(0);

	//followlinedistance(yellowbox);
	dropOff(bin2Pos);

	//followlinedistance(bluebox);
	dropOff(bin1Pos);

	reverse(2);

	goto_objective(pitstop);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(500);

	goto_objective(start);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	for (;;);

}




