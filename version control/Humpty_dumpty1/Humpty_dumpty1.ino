/*
Name:		Linefollower.ino
Created:	3/8/2017 5:54:13 PM
Author:	Anthony_Brown
*/
#include <MD_TCS230.h>
#include <Servo.h>
//SERVOS
Servo panServo;
Servo tiltServo;
Servo clawServo;
Servo clawTilt;

//SERVO VARIABLES
int clawTiltDef = 140;
int panDef = 80;
int clawDef = 50;
int tiltDef = 130;

int clawClamp = 80, clawOpen = 30, clawRelease = 40;

int clawTiltBack = 160; //tilt claw to bin position

int panBackCentre, panBackLeft, panBackRight;
int clawHold;
int tiltForward = 110, tiltBack = 145, tiltPick = 165;

int bin1Pos = 100, bin2Pos = 60, binTilt = 155, binTiltLow = 160, clawTiltPick = 30, ballot3 = 0;

//LEFT MOTOR
int enablepinA = 7;
int logic1A = 28;
int logic2A = 40;
//RIGHT MOTOR
int enablepinB = 8;
int logic3A = 24;
int logic4A = 22;
//INFRA RED
int LI = 21; //green
int LO = 18; //purple
int RO = 20; //blue
int RI = 19; //white

			 //RGB
int Rled = 10;
int Gled = 12;
int Bled = 13;
volatile bool Rstate = 0;
volatile bool Gstate = 0;
volatile bool ledstate = 0;

//VELOCITY ENCODERS
const int lsig = 2;
const int rsig = 3;
volatile unsigned lvcount = 0;
volatile unsigned rvcount = 0;
unsigned storedlvcount;
unsigned storedrvcount;
int turnticksleft = 9;
int turnticksright = 12;
int fullturnright = 24;
int fullturnleft = 21;

//OBJECTIVES
struct objective {
	int priority;
	int x;
	int y;
	int direction;
	int distleft;
	int distright;
};

objective ver_in = { 1,6,4,90,0,0 };
objective ver_out = { 2,10,4,90,0,0 };
objective redpapproach = { 3,18,2,180,0,0 };
objective redpaper = { 3,18,0,180,0,0 };
objective yellowpapproach = { 4,17,2,180,0,0 };
objective yellowpaper = { 4,17,0,180,0,0 };
objective bluepapproach = { 4,16,2,180,0,0 };
objective bluepaper = { 4,16,0,180,0,0 };
objective booth_in = { 5,17,7,270,0,0 };
objective booth_out = { 6,13,7,270,0,0 };
objective ramp = { 5,7,8,270,0,0, };
objective test = { 0,1,2,0,0,0 };
objective start = { 0,0,0,0,0,0 };

//LOCATORS
struct pos {
	int x = 0;
	int y = 0;
	int direction = 90;
};
pos location;

//SPEEDOMETER VARIABLES
int setspeed = 100;
int rotspeed = 200;
int correction = 4;
int bigcorrection = 6;
int backtrack = 2;
int startoff = 5;
int frocorrect = 4;
int paperdistance = 10;

//FUNCTION DEFINITIONS

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
void moveforward(int ticks = -1)
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
		lvcount = 0;
		rvcount = 0;
		analogWrite(enablepinA, setspeed);
		analogWrite(enablepinB, setspeed);
		digitalWrite(logic1A, HIGH);
		digitalWrite(logic3A, HIGH);
		digitalWrite(logic2A, LOW);
		digitalWrite(logic4A, LOW);
		while (lvcount < ticks || rvcount < ticks) {}
		stationary();
	}
	//enable the motor driver

}

void reverse(int ticks = -1)
{
	if (ticks == -1)
	{
		analogWrite(enablepinA, setspeed);
		analogWrite(enablepinB, setspeed);
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
		analogWrite(enablepinA, setspeed);
		analogWrite(enablepinB, setspeed);
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
void freeroll_stop()
{
	digitalWrite(enablepinA, LOW);
	digitalWrite(enablepinB, LOW);
}
void stationary() {
	reverse();
	delay(100);
	digitalWrite(enablepinA, LOW);
	digitalWrite(enablepinB, LOW);
}
void revstationary() {
	moveforward();
	delay(100);
	digitalWrite(enablepinA, LOW);
	digitalWrite(enablepinB, LOW);
}

void rotateleft_axial(int ticks = -1)
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
void rotateleft_radial(int ticks = -1)
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

void lvisr()
{
	lvcount++;
}
void rvisr()
{
	rvcount++;
}

void goto_objective(struct objective obj)
{
	int intersectsx = obj.x - location.x;
	int intersectsy = obj.y - location.y;
	if (intersectsx < 0)
	{
		if (location.direction == 0) {
			reverse(backtrack);
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
	followline(abs(intersectsx)); //Execute x maneuver first.
	if (intersectsy < 0) {
		if (location.direction == 90) {
			reverse(backtrack);
			rotateright_axial(turnticksright);
		}
		else if(location.direction == 270){
			reverse(backtrack);
			rotateleft_axial(turnticksleft);
		}
		else if (location.direction == 0)
		{
			rotateright_axial(fullturnright);
		}
		location.direction = 180;
	}
	else if (intersectsy > 0) {
		if (location.direction == 90) {
			reverse(backtrack);
			rotateleft_axial(turnticksleft);
		}
		else if(location.direction == 270) {
			reverse(backtrack);
			rotateright_axial(turnticksright);
		}
		else if (location.direction == 180)
		{
			rotateleft_axial(fullturnleft);
		}
		location.direction = 0;
	}
	location.y = obj.y;
	followline(abs(intersectsy)); //Execute Y maneuver

	//Turn to the direction specified by the objective.
	if (abs(location.direction - obj.direction) == 180) {
		rotateright_axial(fullturnright);
	}
	else if ((location.direction - obj.direction) == -90) {
		reverse(backtrack);
		rotateright_axial(turnticksright);
	}
	else if ((location.direction - obj.direction) == 90) {
		reverse(backtrack);
		rotateleft_axial(turnticksleft);
	}
	else if ((location.direction - obj.direction) == -270) {
		reverse(backtrack);
		rotateleft_axial(turnticksleft);
	}
	else if ((location.direction - obj.direction) == 270) {
		reverse(backtrack);
		rotateright_axial(turnticksright);
	}
	location.direction = obj.direction;
}

void followline(int sects) //parameters are the number of intersects both on x and y axes
{
	if (sects == 0)
		return;
	int intersects = 0;
	while (intersects <= sects)//less than or equal 2 because it likely started off on an intersect
	{
		bool lo = !digitalRead(LO) && !digitalRead(LO);
		bool li = !digitalRead(LI) && !digitalRead(LI);
		bool ri = !digitalRead(RI) && !digitalRead(RI);
		bool ro = !digitalRead(RO) && !digitalRead(RO);

		//Serial.print(lo);
		//Serial.print(" // ");
		//Serial.print(li);
		//Serial.print(" // ");
		//Serial.print(ri);
		//Serial.print(" // ");
		//Serial.println(ro);
		//delay(500);

		if (lo && ro) //intersection reached, increment appropriately
		{
			digitalWrite(Rled, 0);
			digitalWrite(Gled, 0);
			digitalWrite(Bled, 1);
			//reverse(backtrack);
			stationary();
			intersects++;
			if (intersects != sects + 1)
				moveforward(startoff);
		}
		else if (!lo && !ro)
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
		else if (lo && !ro)
		{
			rotateleft_radial();
		}
		else if (ro && !lo)
		{
			rotateright_radial();
		}
		//else if (!lo || !ro)
		//{
		//	if (!li && !ri)
		//	{
		//		moveforward();
		//		digitalWrite(Rled, 0);
		//		digitalWrite(Gled, 0);
		//		digitalWrite(Bled, 0);
		//	}
		//	else if (li)
		//	{
		//		rotateleft_radial();
		//		digitalWrite(Rled, 1);
		//		digitalWrite(Gled, 0);
		//		digitalWrite(Bled, 0);
		//	}
		//	else
		//	{
		//		rotateright_radial();
		//		digitalWrite(Rled, 0);
		//		digitalWrite(Gled, 1);
		//		digitalWrite(Bled, 0);
		//	}
		//}
	}
}

//SERVO FUNCTIONS
void okota(int binPos) {
	if (binPos > 0) {
		//moveServo(panServo, panDef, panDef);//pan to pick ballot
		moveServo(tiltServo, tiltServo.read(), tiltForward);//tilt down
		moveServo(clawTilt, clawTiltDef, clawTiltPick); //tilt the clamp
		moveServo(clawServo, clawDef, clawClamp);//grab object
		moveServo(clawTilt, clawTilt.read(), clawTiltDef); //tilt clamp back
		moveServo(tiltServo, tiltServo.read(), tiltDef); //tilt arm back to default
		moveServo(panServo, panServo.read(), binPos); //pan to bin 1
		moveServo(tiltServo, tiltServo.read(), tiltBack); //tilt arm lower for drop off
		moveServo(clawTilt, clawTilt.read(), binTilt); //tilt clamp for drop off
		moveServo(clawServo, clawServo.read(), clawRelease); //release object to bin
		moveServo(clawServo, clawServo.read(), clawOpen);
		moveServo(clawTilt, clawTilt.read(), clawTiltDef); //tilt back clamp
		moveServo(tiltServo, tiltServo.read(), tiltDef); //tilt back arm
		moveServo(panServo, panServo.read(), panDef);
	}
	else {
		moveServo(tiltServo, tiltServo.read(), tiltForward);//tilt down
		moveServo(clawTilt, clawTiltDef, clawTiltPick); //tilt the clamp
		moveServo(clawServo, clawDef, clawClamp);//grab object
		moveServo(clawTilt, clawTilt.read(), clawTiltDef); //tilt clamp back
		moveServo(tiltServo, tiltServo.read(), tiltDef); //tilt arm back to default
	}
}
void dropOff(int binPos) {
	if (binPos > 0) {
		moveServo(panServo, panDef, binPos);//pan to pick ballot
		moveServo(clawTilt, clawTiltDef, 130);//tilt the clamp lower for picking
		moveTiltServo(tiltServo, tiltServo.read(), 160);//tilt down     
		moveServo(clawServo, clawDef, clawClamp);//grab object
		moveServo(clawTilt, clawTilt.read(), clawTiltDef); //tilt clamp back
		moveTiltServo(tiltServo, tiltServo.read(), tiltDef); //tilt arm back to default
		moveServo(panServo, panServo.read(), panDef);
		moveTiltServo(tiltServo, tiltServo.read(), tiltForward); //tilt arm lower for drop off
		moveServo(clawTilt, clawTilt.read(), clawTiltPick); //tilt clamp for drop off
		moveServo(clawServo, clawServo.read(), clawRelease); //open clamp
		armDefault();
	}
	else {
		moveTiltServo(tiltServo, tiltServo.read(), tiltDef); //tolt to default
		moveServo(panServo, panServo.read(), panDef); //pan to default
		moveServo(clawTilt, clawTilt.read(), clawTiltPick); //tilt claw to drop off to bin
		moveServo(clawServo, clawServo.read(), clawRelease);
		armDefault();

	}
}
void armDefault() {
	moveTiltServo(tiltServo, tiltServo.read(), tiltDef); //tolt to default
	moveServo(panServo, panServo.read(), panDef); //pan to default
	moveServo(clawTilt, clawTilt.read(), clawTiltDef); //tilt claw to drop off to bin
}
void moveServo(Servo myServo, int startPos, int stopPos) {
	if (startPos < stopPos) {
		for (int i = startPos; i <= stopPos; i++) {
			myServo.write(i);
			delay(20);
		}
	}
	else {
		for (int i = startPos; i >= stopPos; i--) {
			myServo.write(i);
			delay(20);
		}
	}
}
void moveTiltServo(Servo myServo, int startPos, int stopPos) {
	if (startPos < stopPos) {
		for (int i = startPos; i <= stopPos; i++) {
			myServo.write(i);
			delay(65);
		}
	}
	else {
		for (int i = startPos; i >= stopPos; i--) {
			myServo.write(i);
			delay(65);
		}
	}
}

void setup() {
	Serial.begin(115200);
	//SERVO SETUP
	clawServo.attach(4);
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

void loop()
{
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

	followline(2);
	location.x = 1;

	goto_objective(ver_in);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(1000);
	
	goto_objective(ver_out);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(1000);

	goto_objective(redpapproach);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(1000);
	
	goto_objective(redpaper);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(1000);
	//moveforward(paperdistance);
	okota(bin1Pos); //put the red ballot paper into slot one
	//(paperdistance);

	goto_objective(bluepapproach);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(1000);
		
	goto_objective(bluepaper);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(1000);
	//moveforward(paperdistance);
	okota(bin2Pos);	//put the blue ballot paper into slot two
	//reverse(paperdistance);

	goto_objective(yellowpapproach);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(1000);

	goto_objective(yellowpaper);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(1000);
	//moveforward(paperdistance);
	okota(ballot3);//Pick and carry the yellow ballot paper
	//reverse(paperdistance);

	goto_objective(booth_in);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(1000);
	
	goto_objective(booth_out);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(1000);
	
	goto_objective(ramp);
	digitalWrite(Rled, 0);
	digitalWrite(Gled, 1);
	digitalWrite(Bled, 1);
	delay(1000);
}





