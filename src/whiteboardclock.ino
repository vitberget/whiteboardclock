#include <cmath>
#include <string.h>
#include <stdlib.h>
const float PI = 3.14159265f;

const char* font[] = {
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
    
    
    
    "END",
    NULL
};

Servo servo_left;
Servo servo_right;
Servo servo_lift;

const float left_servo_x            = -4.0f;
const float right_servo_x           = 4.0f;
const float left_angle_offset       = 0.0f;
const float right_angle_offset      = 0.0f;
const float inner_arm               = 13.0f;
const float outer_arm               = 22.0f;
const float inner_arm_sq            = inner_arm * inner_arm;
const float outer_arm_sq            = outer_arm * outer_arm;
const float cosine_rule_helper_sq   = inner_arm_sq - outer_arm_sq;
const float botharms_sq             = (inner_arm + outer_arm) * (inner_arm + outer_arm);

void setup() {
    servo_left.attach(  D0 );
    servo_right.attach( D1 );
    servo_lift.attach(  D2 );

    penUp(NULL);
    delay(5);
    setAngles(90,90);

    Particle.function("xy", gotoStrXY);
    Particle.function("pu", penUp); 
    Particle.function("pd", penDown); 
    Particle.function("text", drawText);
    Particle.function("angles", setAnglesStr);
}

void loop()                     {}
int penUp(String ignore)        { servo_lift.write(12);   return 0; }
int penDown(String ignore)      { servo_lift.write(120);  return 0; }
int gotoStrXY(String command)   { return gotoStrXY_with_delta(command, 0.0f, 0.0f); }

int gotoStrXY_with_delta(const String command, const float dx, const float dy) {
    int index = command.indexOf(":");
    if(index<0) return -200;
    
    float pen_x = dx + atof(command.substring(0,index));
    float pen_y = dy + atof(command.substring(index+1));
    
    return gotoXY(pen_x, pen_y);
}

int setAnglesStr(String command) {
    int index = command.indexOf(":");
    if(index<0) return -200;
    
    float left  = atof(command.substring(0,index));
    float right = atof(command.substring(index+1));
    
    return setAngles(left, right);
}

// origin x is zero in the middle of the servos, positive to the right
// origin y is zero at the line of two servos, positive below
// Desired point to move the pen to referred to 'the point'
int gotoXY(const float pen_x, const float pen_y) {
    if(pen_y<5) return -1;
    
    // Calculate the point relative to each servo
    const float delta_left_x  = pen_x - left_servo_x;
	const float delta_right_x = pen_x - right_servo_x;
    
    // Distances servos to pen, squared
    const float pen_y_sq = pen_y * pen_y;
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
	const float servo_angle_left  = 90.0f + ( bend_angle_left  - pen_angle_left)  * 180.0f / PI;
	const float servo_angle_right = 90.0f + (-bend_angle_right - pen_angle_right) * 180.0f / PI;
    
    // Finally - move the servos
    return setAngles(servo_angle_left, servo_angle_right); 
}

int setAngles(const float left_angle_in, const float right_angle_in) {
    const float left_angle  = left_angle_in  + left_angle_offset;
    const float right_angle = right_angle_in + right_angle_offset;
    
    // Sanity check of angles
    if(left_angle <0) return -21; 
    if(right_angle<0) return -22; 
    if(left_angle >180) return -23; 
    if(right_angle>180) return -24; 
    
    servo_left.write(left_angle);
    servo_right.write(right_angle);
    return 0;   
}

int drawText(String text) {
    float dx = -8.0f;
    for(int i=0; i<text.length(); i++) {
        dx += drawCharacter(text.charAt(i), dx, 7.0f);   
    }
    return 0;
}

float drawCharacter(const char character, const float dx, const float dy) {
    bool drawing = false;
    float width  = 0.0f;
    
    for(int i=0;i<30;i++) {
        const char* cmdLine = font[i];
        if(strncmp("CHAR ",cmdLine,5) == 0) {
            if(drawing) break;
            if(cmdLine[5]==character) {
                drawing = true;
                width = atof(&cmdLine[7]);
            }
        }
        else if(strncmp("XY ",cmdLine,3) == 0) { if(drawing) gotoStrXY_with_delta(String(&cmdLine[3]), dx, dy); }
        else if(strncmp("PU",cmdLine,2) == 0)  { if(drawing) penUp(NULL); }
        else if(strncmp("PD",cmdLine,2) == 0)  { if(drawing) penDown(NULL); }
        else if(cmdLine==NULL || strncmp("END",cmdLine,3) == 0) break;
        else break; // Unknown command
        }
    }
	return width;
}