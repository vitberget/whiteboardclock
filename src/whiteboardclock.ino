#include <cmath>
#define PI 3.14159265

Servo sLeft;
Servo sRight;
Servo upDown;

const double lServoX = -2.5;
const double rServoX = 2.5;

const double arm1 = 12;
const double arm2 = 7;
const double botharms = arm1+arm2;
const double botharms_sq = botharms * botharms;

void debug(String message) {
    Spark.publish("DEBUG", message);
}

void setup() {
    sLeft.attach(D0);
    sRight.attach(D1);
    upDown.attach(D2);

    penUp();
    delay(25);
    sLeft.write(90);
    sRight.write(90);
    Particle.function("xy",gotoStrXY);
    Particle.function("pu",penUpStr); 
    Particle.function("pd",penDownStr); 
    Particle.function("text",drawText);
}

int gotoStrXY(String command) {
    int index = command.indexOf(":");
    if(index<0) return -200;
    
    float x = atof(command.substring(0,index));
    float y = atof(command.substring(index+1));
    
    return gotoXY(x,y);
}

int penUpStr(String ignore) {
    penUp();
    return 0;
}


int penDownStr(String ignore) {
    penDown();
    return 0;
}

void penUp() {
    // upDown.write(0);
}


void penDown() {
    // upDown.write(0);
}

double r2d(double r) {
	return r * 180 / PI;
}

// x is zero in the middle of the servos, positive to the right
// y is zero at the line of two servos, positive below
// Desired point to move the pen to referred to 'the point'
int gotoXY(const double x, const double y) {
    
    if(y<5) {
		debug("Above y threashold, risc of flipping the wrong way");
		return -1;
	}
    
    // Calculate the point relative to each servo
    const double d_lx = x - lServoX;
    const double d_lx_sq = d_lx * d_lx;
    const double d_rx = x - rServoX;
    const double d_rx_sq = d_rx * d_rx;
    const double y_sq = y * y;
    
    
    // See if the point is impossible far away (longer than both arms)
    if( d_lx_sq + y_sq >= botharms_sq) {
        debug("Out of reach, left arm");
        return -10;
    }
    if( d_rx_sq + y_sq >= botharms_sq) {
        debug("Out of reach, right arm");
        return -11;
    }

    // Angles from each servo to the point
    const double v1_l = atan2(d_lx, y);
    const double v1_r = atan2(d_rx, y);

    // Calculate the imaginary point in the "middle" of the vector from each servo to the point
    const double mp_lx = (d_lx * arm1) /  botharms;
    const double mp_rx = (d_rx * arm1) /  botharms;
    const double mp_y = (y * arm1) / botharms;
	const double mp_y_sq = mp_y * mp_y;
    
    // Lenght to the imaginary "middle" point
    const double mp_len_l = sqrt(mp_lx*mp_lx + mp_y_sq);
    const double mp_len_r = sqrt(mp_rx*mp_rx + mp_y_sq);
    
    // We know the length of the arm, distance to the middle point and that one angle is 90 degrees, mathy
    const double v2_l = acos(mp_len_l/arm1);
    const double v2_r = acos(mp_len_r/arm1);

    // Sum the two angles together
    const double v_lr = v2_l - v1_l;
	const double v_rr = -v2_r - v1_r;
	
	// Convert radian to degrees
	const double v_l_1 = r2d(v_lr);
	const double v_r_1 = r2d(v_rr);
	
	// Offset 90 degrees
	const double v_l = 90 + v_l_1;
	const double v_r = 90 + v_r_1;
    
    // Sanity check of angles
    if(v_l<0) return -21; // Left angle to low
    if(v_r<0) return -22; // Right angle to low
    if(v_l>180) return -23; // Left angle to high
    if(v_r>180) return -24; // Right angle to high
    
    // Finally - move the servos
    sLeft.write(v_l);
    sRight.write(v_r);
    
    return 0;
}

void loop(){}

const String font[] = {
    "CHAR 1 1",
    "PU",
    "XY 0:0.5",
    "PD",
    "XY 0.5:0",
    "XY 0.5:0.5",
    "XY 0.5:1",
    "XY 0.5:1.5",
    "XY 0.5:2",
    "PU",
    "XY 0:2",
    "PD",
    "XY 0.5:2",
    "XY 1:2",
    "PU",
    
    
};


int drawText(String text) {
    double dx = -8;
    for(int i=0; i<text.length(); i++) {
        dx += drawCharacter(text.charAt(i), dx, 7);   
    }
    return 0;
}

double drawCharacter(const char c, const double dx, const double dy) {
    bool drawing = false;
    double width = 0;
    
    for(int i=0;;i++) {
        String cmdLine = font[i];
        if(cmdLine.startsWith("CHAR ")) {
            if(drawing) 
                return width;
            if(cmdLine.charAt(5)==c) {
                drawing = true;
                width = atof(cmdLine.substring(7));
            }
        }
        else if(cmdLine.startsWith("PU")) {
            if(drawing) penUp();
        }
        else if(cmdLine.startsWith("PD")) {
            if(drawing) penDown();
        }
        else if(cmdLine.startsWith("XY ")) {
            if(drawing) gotoStrXY(cmdLine.substring(3));
        }
        else {
            return width; 
        }
    }
	return width;
}