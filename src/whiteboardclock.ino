#include <cmath>
#define PI 3.14159265

Servo servo_left;
Servo servo_right;
Servo servo_lift;

const float left_servo_x            = -4;
const float right_servo_x           = 4;
const float inner_arm               = 13;
const float outer_arm               = 22;
const float inner_arm_sq            = inner_arm * inner_arm;
const float outer_arm_sq            = outer_arm * outer_arm;
const float cosine_rule_helper_sq   = inner_arm_sq - outer_arm_sq;
const float botharms_sq             = (inner_arm + outer_arm) * (inner_arm + outer_arm);

void setup() {
    servo_left.attach(D0);
    servo_right.attach(D1);
    servo_lift.attach(D2);

    penUp();
    delay(25);
    servo_left.write(90);
    servo_right.write(90);
    
    Particle.function("xy",gotoStrXY);
    Particle.function("pu",penUpStr); 
    Particle.function("pd",penDownStr); 
    Particle.function("text",drawText);
}

void loop(){}

int gotoStrXY(String command) {
    int index = command.indexOf(":");
    if(index<0) return -200;
    
    float pen_x = atof(command.substring(0,index));
    float pen_y = atof(command.substring(index+1));
    
    return gotoXY(pen_x,pen_y);
}

int penUpStr(String ignore)   { penUp();   return 0; }
int penDownStr(String ignore) { penDown(); return 0; }
void penUp()   { servo_lift.write(12); }
void penDown() { servo_lift.write(120);}
float radian2degrees(float r) {	return r * 180.0f / PI; }

// origin x is zero in the middle of the servos, positive to the right
// origin y is zero at the line of two servos, positive below
// Desired point to move the pen to referred to 'the point'
int gotoXY(const float pen_x, const float pen_y) {
    if(pen_y<5) return -1;
    
    // Calculate the point relative to each servo
    const float delta_left_x  = pen_x - left_servo_x;
	const float delta_right_x = pen_x - right_servo_x;
    const float pen_y_sq = pen_y * pen_y;
    
    // Distances servos to pen, squared
	const float pen_distance_left_sq  = delta_left_x  * delta_left_x  + pen_y_sq;
	const float pen_distance_right_sq = delta_right_x * delta_right_x + pen_y_sq;
    
    // See if the point is impossible far away (longer than both arms)
    if( pen_distance_left_sq  >= botharms_sq) return -10;
    if( pen_distance_right_sq >= botharms_sq) return -11;

    // Angles from each servo to the point
    const float pen_angle_left  = atan2(delta_left_x,  pen_y);
    const float pen_angle_right = atan2(delta_right_x, pen_y);

	// Cosine rule for the win (we know all three lenght of the "triangle")
	const float bend_angle_left  = acos( (cosine_rule_helper_sq + pen_distance_left_sq )  / (2*inner_arm*sqrt(pen_distance_left_sq)));
	const float bend_angle_right = acos( (cosine_rule_helper_sq + pen_distance_right_sq ) / (2*inner_arm*sqrt(pen_distance_right_sq)));

	// Sum the two angles together (but they are "opposite") and convert radian to degrees, with 90 degrees offset
	const float servo_angle_left  = 90.0f + radian2degrees( bend_angle_left  - pen_angle_left);
	const float servo_angle_right = 90.0f + radian2degrees(-bend_angle_right - pen_angle_right);
    
    // Sanity check of angles
    if(servo_angle_left <0) return -21; // Left angle to low
    if(servo_angle_right<0) return -22; // Right angle to low
    if(servo_angle_left >180) return -23; // Left angle to high
    if(servo_angle_right>180) return -24; // Right angle to high
    
    // Finally - move the servos
    servo_left.write(servo_angle_left);
    servo_right.write(servo_angle_right);
    return 0;
}

const String font[] = {
// 1    
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
    float dx = -8.0f;
    for(int i=0; i<text.length(); i++) {
        dx += drawCharacter(text.charAt(i), dx, 7.0f);   
    }
    return 0;
}

float drawCharacter(const char c, const float dx, const float dy) {
    bool drawing = false;
    float width = 0.0f;
    
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