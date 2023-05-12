#include <stdio.h>
#include <stdlib.h>
#include <math.h>

extern "C" {
	#include "node.h"
	#include "graph.h"
	#include "logic.h"
	#include "extApi.h"
}

#define CLAMP(num) (num<0.0?0.0:(num>1.0?1.0:num))
#define FCLAMP(num) (num<FIX_ZERO?FIX_ZERO:(num>FIX_ONE?FIX_ONE:num))

#define LSIZE 2
#define BSIZE 5

#include <sys/time.h>
struct timeval timeStart, timeEnd, timeTotal;
void timerStart(void)
{
	gettimeofday(&timeStart, NULL);
}
void timerAdd(void)
{
	gettimeofday(&timeEnd, NULL);
	timeTotal.tv_sec += timeEnd.tv_sec - timeStart.tv_sec;
	timeTotal.tv_usec += timeEnd.tv_usec - timeStart.tv_usec;
}
void timerEnd(const char *tag)
{
	printf("TIMER: %s took %fus\n", tag, ((double)(timeTotal.tv_sec) * 1000000.0 + (double)(timeTotal.tv_usec)));
	timeTotal.tv_sec = 0;
	timeTotal.tv_usec = 0;
}

bool boolean = false;
double BOOLEAN_TARGET_THRESHOLD = 0.3;
double BOOLEAN_OBSTACLE_THRESHOLD = 0.1;

enum {L = 0, FL = 1, F = 2, FR = 3, R = 4};

bool collision = false;
fix obstacle[BSIZE] = {FIX_ZERO};
fix target[BSIZE] = {FIX_ZERO};

bool lspeed = false;
fix bspeed = FIX_ZERO;
fix direction[BSIZE] = {FIX_ZERO};
fix motordir[BSIZE] = {FIX_ZERO};
fix motorf[BSIZE] = {FIX_ZERO};
fix motorb[BSIZE] = {FIX_ZERO};

#define MAX_SPEED 0.2
float motorSpeeds[2];

//fix os1 = FIX_ZERO, os2 = FIX_ZERO, os3 = FIX_ZERO, os4 = FIX_ZERO, os5 = FIX_ZERO, t2 = FIX_ZERO, t3 = FIX_ZERO, t4 = FIX_ZERO;

FILE *out = NULL;
unsigned int nn[MAX_NODES] = {0};

void initialize(unsigned int nn[])
{
	fix lnull[LSIZE]  = {FIX_ZERO, FIX_ZERO};
	fix ltrue[LSIZE]  = {FIX_ZERO, FIX_ONE};
	fix lfalse[LSIZE] = {FIX_ONE, FIX_ZERO};
	fix leven[LSIZE] = {DOUBLE_TO_FIX(0.5), DOUBLE_TO_FIX(0.5)};
	//fix bzero[BSIZE]  = {0};

	// Speed (interpreted as a likelihood of no-collision given forward direction)
	nn[0] = addBNode("speed", lnull, LSIZE);
	nn[2] = addBNode("forward", lnull, LSIZE);

	// Direction (interpreted as a likelihood of no-collision given the direction indicated
	//nn[1] = addBNode("direction", bzero, BSIZE);

	// Inertial sensing
	//nn[2] = addBNode("target", bzero, BSIZE);

	// Inertial sensing
	//nn[3] = addBNode("position", bzero, BSIZE);

	// Vision sensing output
	//nn[4] = addBNode("obstacle", bzero, BSIZE);

	// Rules for logic output
	//nn[5] = addBNode("miao", bzero, BSIZE);

	// Collision sensor
	nn[10] = addBNode("collision", lfalse, LSIZE);
	nn[5] = addBNode("not-collision", lfalse, LSIZE);
	addBParent(nn[5], nn[10]);
	makeLogic(NOT, nn[5]);

	// Navigation Target
	nn[11] = addBNode("target_L", lfalse, LSIZE);
	nn[12] = addBNode("target_FL", lfalse, LSIZE);
	nn[13] = addBNode("target_F", lfalse, LSIZE);
	nn[14] = addBNode("target_FR", lfalse, LSIZE);
	nn[15] = addBNode("target_R", lfalse, LSIZE);
//	addBParent(nn[11], nn[2]);
//	addBParent(nn[12], nn[2]);
//	addBParent(nn[13], nn[2]);
//	addBParent(nn[14], nn[2]);
//	addBParent(nn[15], nn[2]);

	// Obstacle presence
	nn[21] = addBNode("not-obstacle_L", ltrue, LSIZE);
	nn[22] = addBNode("not-obstacle_FL", ltrue, LSIZE);
	nn[23] = addBNode("not-obstacle_F", ltrue, LSIZE);
	nn[24] = addBNode("not-obstacle_FR", ltrue, LSIZE);
	nn[25] = addBNode("not-obstacle_R", ltrue, LSIZE);
//	addBParent(nn[21], nn[4]);
//	addBParent(nn[22], nn[4]);
//	addBParent(nn[23], nn[4]);
//	addBParent(nn[24], nn[4]);
//	addBParent(nn[25], nn[4]);

	// Direction Control
	nn[31] = addBNode("direction_L", leven, LSIZE);
	nn[32] = addBNode("direction_FL", leven, LSIZE);
	nn[33] = addBNode("direction_F", leven, LSIZE);
	nn[34] = addBNode("direction_FR", leven, LSIZE);
	nn[35] = addBNode("direction_R", leven, LSIZE);
	addBParent(nn[31], nn[11]);
	addBParent(nn[31], nn[21]);
	addBParent(nn[32], nn[12]);
	addBParent(nn[32], nn[22]);
	addBParent(nn[33], nn[13]);
	addBParent(nn[33], nn[23]);
	addBParent(nn[34], nn[14]);
	addBParent(nn[34], nn[24]);
	addBParent(nn[35], nn[15]);
	addBParent(nn[35], nn[25]);
	makeLogic(AND, nn[31]);
	makeLogic(AND, nn[32]);
	makeLogic(AND, nn[33]);
	makeLogic(AND, nn[34]);
	makeLogic(AND, nn[35]);

	// Speed control
	addBParent(nn[2], nn[12]);
	addBParent(nn[2], nn[13]);
	addBParent(nn[2], nn[14]);
	makeLogic(OR, nn[2]);
	addBParent(nn[0], nn[2]);
	addBParent(nn[0], nn[5]);
	makeLogic(AND, nn[0]);

	// Direction control
	//addBParent(nn[1], nn[31]);
	//addBParent(nn[1], nn[32]);
	//addBParent(nn[1], nn[33]);
	//addBParent(nn[1], nn[34]);
	//addBParent(nn[1], nn[35]);

	// Left motor fan-in
	nn[41] = addBNode("turn_L", leven, LSIZE);
	addBParent(nn[41], nn[35]);
	addBParent(nn[41], nn[34]);
	addBParent(nn[41], nn[33]);
	makeLogic(OR, nn[41]);

	// Right motor fan-in
	nn[42] = addBNode("turn_R", leven, LSIZE);
	addBParent(nn[42], nn[31]);
	addBParent(nn[42], nn[32]);
	addBParent(nn[42], nn[33]);
	makeLogic(OR, nn[42]);

	// Left motor forward speed
	nn[6] = addBNode("motor_forward_L", lfalse, LSIZE);
	addBParent(nn[6], nn[0]);
	addBParent(nn[6], nn[41]);
	makeLogic(AND, nn[6]);

	// Right motor forward speed
	nn[7] = addBNode("motor_forward_R", lfalse, LSIZE);
	addBParent(nn[7], nn[0]);
	addBParent(nn[7], nn[42]);
	makeLogic(AND, nn[7]);

	// Left motor backward speed
	nn[8] = addBNode("motor_reverse_L", lfalse, LSIZE);
	addBParent(nn[8], nn[10]);
	addBParent(nn[8], nn[31]);
	makeLogic(OR, nn[8]);

	// Right motor backward speed
	nn[9] = addBNode("motor_reverse_R", lfalse, LSIZE);
	//addBParent(nn[9], nn[10]);
	addBParent(nn[9], nn[35]);
	makeLogic(OR, nn[9]);

	timerAdd();
	timerEnd("initialization");
}

void update_boolean(unsigned int iteration)
{
	bool collision_F = collision;

	bool target_L = (target[L] > DOUBLE_TO_FIX(BOOLEAN_TARGET_THRESHOLD));
	bool target_FL = (target[FL] > DOUBLE_TO_FIX(BOOLEAN_TARGET_THRESHOLD));
	bool target_F = (target[F] > DOUBLE_TO_FIX(BOOLEAN_TARGET_THRESHOLD));
	bool target_FR = (target[FR] > DOUBLE_TO_FIX(BOOLEAN_TARGET_THRESHOLD));
	bool target_R = (target[R] > DOUBLE_TO_FIX(BOOLEAN_TARGET_THRESHOLD));

	bool notobstacle_L = !(obstacle[L] > DOUBLE_TO_FIX(BOOLEAN_OBSTACLE_THRESHOLD));
	bool notobstacle_FL = !(obstacle[FL] > DOUBLE_TO_FIX(BOOLEAN_OBSTACLE_THRESHOLD));
	bool notobstacle_F = !(obstacle[F] > DOUBLE_TO_FIX(BOOLEAN_OBSTACLE_THRESHOLD));
	bool notobstacle_FR = !(obstacle[FR] > DOUBLE_TO_FIX(BOOLEAN_OBSTACLE_THRESHOLD));
	bool notobstacle_R = !(obstacle[R] > DOUBLE_TO_FIX(BOOLEAN_OBSTACLE_THRESHOLD));

	bool direction_L = target_L && notobstacle_L;
	bool direction_FL = target_FL && notobstacle_FL;
	bool direction_F = target_F && notobstacle_F;
	bool direction_FR = target_FR && notobstacle_FR;
	bool direction_R = target_R && notobstacle_R;

	if(direction_L) direction[L] = DOUBLE_TO_FIX(1.0); else direction[L] = DOUBLE_TO_FIX(0.0);
	if(direction_FL) direction[FL] = DOUBLE_TO_FIX(1.0); else direction[FL] = DOUBLE_TO_FIX(0.0);
	if(direction_F) direction[F] = DOUBLE_TO_FIX(1.0); else direction[F] = DOUBLE_TO_FIX(0.0);
	if(direction_FR) direction[FR] = DOUBLE_TO_FIX(1.0); else direction[FR] = DOUBLE_TO_FIX(0.0);
	if(direction_R) direction[R] = DOUBLE_TO_FIX(1.0); else direction[R] = DOUBLE_TO_FIX(0.0);

	bool speed = (target_FL || target_F || target_FR) && !collision_F;

	bool motordir_L = direction_R || direction_FR || direction_F;
	bool motordir_R = direction_L || direction_FL || direction_F;

	bool motor_FW_L = motordir_L && speed;
	bool motor_FW_R = motordir_R && speed;

	bool motor_BW_L = collision || direction_L;
	bool motor_BW_R = direction_R;

	motorSpeeds[0] = (motor_FW_L?MAX_SPEED:0.0) - (motor_BW_L?MAX_SPEED:0.0);
	motorSpeeds[1] = (motor_FW_R?MAX_SPEED:0.0) - (motor_BW_R?MAX_SPEED:0.0);

	timerAdd();

	fprintf(out, "lt(%d)=%.3f; lC(%d)=%d; lT(%d,1:5)=[%d,%d,%d,%d,%d]; lnO(%d,1:5)=[%d,%d,%d,%d,%d]; lD(%d,1:5)=[%d,%d,%d,%d,%d]; lSp(%d)=%d; lMD(%d,1:2)=[%d,%d]; lMF(%d,1:2)=[%d,%d]; lMB(%d,1:2)=[%d,%d]; lS(%d,1:2)=[%.1f,%.1f];\n",
			iteration+1, ((double)iteration),
			iteration+1, collision_F==true,
			iteration+1, target_L==true,target_FL==true,target_F==true,target_FR==true,target_R==true,
			iteration+1, notobstacle_L==true,notobstacle_FL==true,notobstacle_F==true,notobstacle_FR==true,notobstacle_R==true,
			iteration+1, direction_L==true,direction_FL==true,direction_F==true,direction_FR==true,direction_R==true,
			iteration+1, speed==true,
			iteration+1, motordir_L==true, motordir_R==true,
			iteration+1, motor_FW_L==true, motor_FW_R==true,
			iteration+1, motor_BW_L==true, motor_BW_R==true,
			iteration+1, motorSpeeds[0], motorSpeeds[1]
			);

	printf("lt(%d)=%.3f; lC(%d)=%d; lT(%d,1:5)=[%d,%d,%d,%d,%d]; lnO(%d,1:5)=[%d,%d,%d,%d,%d]; lD(%d,1:5)=[%d,%d,%d,%d,%d]; lSp(%d)=%d; lMD(%d,1:2)=[%d,%d]; lMF(%d,1:2)=[%d,%d]; lMB(%d,1:2)=[%d,%d]; lS(%d,1:2)=[%.1f,%.1f];\n",
			iteration+1, ((double)iteration),
			iteration+1, collision_F==true,
			iteration+1, target_L==true,target_FL==true,target_F==true,target_FR==true,target_R==true,
			iteration+1, notobstacle_L==true,notobstacle_FL==true,notobstacle_F==true,notobstacle_FR==true,notobstacle_R==true,
			iteration+1, direction_L==true,direction_FL==true,direction_F==true,direction_FR==true,direction_R==true,
			iteration+1, speed==true,
			iteration+1, motordir_L==true, motordir_R==true,
			iteration+1, motor_FW_L==true, motor_FW_R==true,
			iteration+1, motor_BW_L==true, motor_BW_R==true,
			iteration+1, motorSpeeds[0], motorSpeeds[1]
			);
}

void update(unsigned int iteration)
{
	/*
	if(iteration == 0)
	{ os1 = DOUBLE_TO_FIX(0.5); os2 = FIX_ZERO; os3 = FIX_ZERO; os4 = FIX_ZERO; os5 = FIX_ZERO; t2 = DOUBLE_TO_FIX(0.4); t3 = FIX_ZERO; t4 = FIX_ZERO;}
	*/
	/*
	double ci = ceil((double)MAX_ITERATIONS * 0.5 / 5.0); //collision_iteration
	double ci1 = ceil((double)MAX_ITERATIONS * 0.7  / 5.0); //collision_iteration
	double ci2 = ceil((double)MAX_ITERATIONS * 1.0 / 5.0); //collision_iteration
	double ci3 = ceil((double)MAX_ITERATIONS * 1.1 / 5.0); //collision_iteration
	double pd = CLAMP(((double)iteration)/((double)MAX_ITERATIONS*1.5)); //%done
	double cd = CLAMP(((double)iteration)/((double)MAX_ITERATIONS*1.5-ci)); //%to_collision
	double cd1 = CLAMP(((double)iteration)/((double)MAX_ITERATIONS*1.5-ci1));
	double cd11 = CLAMP(((double)iteration)/((double)MAX_ITERATIONS*1.5-ci2));
	double cd2 = CLAMP(((double)iteration)/((double)MAX_ITERATIONS*1.5-ci3));
	obstacle[L] = DOUBLE_TO_FIX(pd);
	obstacle[FL] = DOUBLE_TO_FIX(cd2);
	obstacle[F] = DOUBLE_TO_FIX(cd1);
	obstacle[FR] = DOUBLE_TO_FIX(cd);
	obstacle[R] = DOUBLE_TO_FIX(cd11);

	double d1 = 1.0 - CLAMP(fixabs((double)iteration - (double)MAX_ITERATIONS * 2.0 / 5.0)/(0.1*MAX_ITERATIONS)); //direction
	double d2 = 1.0 - CLAMP(fixabs((double)iteration - (double)MAX_ITERATIONS * 2.5 / 5.0)/(0.1*MAX_ITERATIONS)); //direction
	double d3 = 1.0 - CLAMP(fixabs((double)iteration - (double)MAX_ITERATIONS * 3.0 / 5.0)/(0.1*MAX_ITERATIONS)); //direction
	double d4 = 1.0 - CLAMP(fixabs((double)iteration - (double)MAX_ITERATIONS * 3.8 / 5.0)/(0.1*MAX_ITERATIONS)); //direction
	double d5 = 1.0 - CLAMP(fixabs((double)iteration - (double)MAX_ITERATIONS * 4.3 / 5.0)/(0.1*MAX_ITERATIONS)); //direction
	if((double) iteration < (double)MAX_ITERATIONS * 1.5 / 5.0) d1 += 0.6;
	target[L] = DOUBLE_TO_FIX(d1);
	target[FL] = DOUBLE_TO_FIX(d2);
	target[F] = DOUBLE_TO_FIX(d3);
	target[FR] = DOUBLE_TO_FIX(d4);
	target[R] = DOUBLE_TO_FIX(d5);
	if(iteration > MAX_ITERATIONS - ci2) collision = true; else collision = false;
	*/

	//fix dir1[MAX_ITERATIONS];
	//makeGaussian(dir1, MAX_ITERATIONS/10, MAX_ITERATIONS/2, MAX_ITERATIONS);
	//fix sum = 0; printf("["); for(int i = 0; i < MAX_ITERATIONS; i++) { printf("%.2f ", FIX_TO_DOUBLE(dir1[i])); sum += dir1[i]; } printf("] sum=%.3f\n", FIX_TO_DOUBLE(sum)); getchar();
	//fix sum2 = 0; printf("["); for(int i = 0; i < MAX_ITERATIONS; i++) { printf("%.2f ", FIX_TO_DOUBLE(normal(i, MAX_ITERATIONS/10, MAX_ITERATIONS/2))); sum2 += dir1[i]; } printf("] sum2=%.3f\n", FIX_TO_DOUBLE(sum2)); getchar();

	/*
	double cstart = MAX_ITERATIONS*0.55;
	double cend = cstart+MAX_ITERATIONS*0.1;

	if(iteration >  cstart && iteration < cend ) collision = true; else collision = false;

	os1 += - fixmult(gaussian(iteration, MAX_ITERATIONS/16, MAX_ITERATIONS*0.1),DOUBLE_TO_FIX(0.5));
	os2 += gaussian(iteration, MAX_ITERATIONS/16, MAX_ITERATIONS*0.1) - gaussian(iteration, MAX_ITERATIONS/12, MAX_ITERATIONS*0.2);
	os3 += gaussian(iteration, MAX_ITERATIONS/12, MAX_ITERATIONS*0.35) - gaussian(iteration, MAX_ITERATIONS/6, MAX_ITERATIONS*0.5)
		- fixmult(gaussian(iteration, MAX_ITERATIONS/20, MAX_ITERATIONS*0.6),DOUBLE_TO_FIX(0.3)) + fixmult(gaussian(iteration, MAX_ITERATIONS/20, MAX_ITERATIONS*0.65),DOUBLE_TO_FIX(0.3));
	os4 += gaussian(iteration, MAX_ITERATIONS/16, MAX_ITERATIONS*0.55) - gaussian(iteration, MAX_ITERATIONS/16, MAX_ITERATIONS*1.0)
		- fixmult(gaussian(iteration, MAX_ITERATIONS/20, MAX_ITERATIONS*0.6),DOUBLE_TO_FIX(0.1)) + fixmult(gaussian(iteration, MAX_ITERATIONS/20, MAX_ITERATIONS*0.65),DOUBLE_TO_FIX(0.1));
	os5 += gaussian(iteration, MAX_ITERATIONS/16, MAX_ITERATIONS*0.8);
	obstacle[L] = FCLAMP(os1);
	obstacle[FL] = FCLAMP(os2);
	obstacle[F] = FCLAMP(os3);
	obstacle[FR] = FCLAMP(os4);
	obstacle[R] = FCLAMP(os5);
	//normalizeArray(obstacle, BSIZE);

	target[L] = normal(iteration, MAX_ITERATIONS/6, MAX_ITERATIONS*0.0);
	t2 += fixmult(gaussian(iteration, MAX_ITERATIONS/10, MAX_ITERATIONS*0.15),DOUBLE_TO_FIX(1.25)) - fixmult(gaussian(iteration, MAX_ITERATIONS/5, MAX_ITERATIONS*0.4),DOUBLE_TO_FIX(1.75))
		+ fixmult(gaussian(iteration, MAX_ITERATIONS/20, MAX_ITERATIONS*0.6),DOUBLE_TO_FIX(0.3)) - fixmult(gaussian(iteration, MAX_ITERATIONS/20, MAX_ITERATIONS*0.7),DOUBLE_TO_FIX(0.15));
	target[FL] = t2;//normal(iteration, MAX_ITERATIONS/6, MAX_ITERATIONS*0.2);
	t3 += fixmult(gaussian(iteration, MAX_ITERATIONS/8, MAX_ITERATIONS*0.2),DOUBLE_TO_FIX(1.18)) - fixmult(gaussian(iteration, MAX_ITERATIONS/4, MAX_ITERATIONS*0.8),DOUBLE_TO_FIX(1.18))
		+ fixmult(gaussian(iteration, MAX_ITERATIONS/20, MAX_ITERATIONS*0.6),DOUBLE_TO_FIX(0.09)) - fixmult(gaussian(iteration, MAX_ITERATIONS/20, MAX_ITERATIONS*0.68),DOUBLE_TO_FIX(0.09));
	target[F] = t3; //normal(iteration, MAX_ITERATIONS/3, MAX_ITERATIONS*0.5);
	t4 += fixmult(gaussian(iteration, MAX_ITERATIONS/5, MAX_ITERATIONS*0.6),DOUBLE_TO_FIX(1.2)) - fixmult(gaussian(iteration, MAX_ITERATIONS/4, MAX_ITERATIONS*1.2),DOUBLE_TO_FIX(1.2))
		+ fixmult(gaussian(iteration, MAX_ITERATIONS/20, MAX_ITERATIONS*0.6),DOUBLE_TO_FIX(0.05)) - fixmult(gaussian(iteration, MAX_ITERATIONS/20, MAX_ITERATIONS*0.68),DOUBLE_TO_FIX(0.05));
	target[FR] = t4; //normal(iteration, MAX_ITERATIONS/5, MAX_ITERATIONS*0.9);
	target[R] = normal(iteration, MAX_ITERATIONS/4, MAX_ITERATIONS*1.7);
	//normalizeArray(target, BSIZE);
	*/

	// Set (not) collision
	if(collision)
		setLogic(nn[10], FIX_ONE);
	else setLogic(nn[10], FIX_ZERO);

	// Set target
	setLogic(nn[11], target[L]);
	setLogic(nn[12], target[FL]);
	setLogic(nn[13], target[F]);
	setLogic(nn[14], target[FR]);
	setLogic(nn[15], target[R]);

	// Set not_obstacle
	setLogic(nn[21], FIX_ONE-obstacle[L]);
	setLogic(nn[22], FIX_ONE-obstacle[FL]);
	setLogic(nn[23], FIX_ONE-obstacle[F]);
	setLogic(nn[24], FIX_ONE-obstacle[FR]);
	setLogic(nn[25], FIX_ONE-obstacle[R]);

	inferBNetwork(nn[6]);
	inferBNetwork(nn[7]);
	inferBNetwork(nn[8]);
	inferBNetwork(nn[9]);

	bspeed = getBNodeInfElement(nn[0], 1);

	direction[L] = getBNodeInfElement(nn[31], 1);
	direction[FL] = getBNodeInfElement(nn[32], 1);
	direction[F] = getBNodeInfElement(nn[33], 1);
	direction[FR] = getBNodeInfElement(nn[34], 1);
	direction[R] = getBNodeInfElement(nn[35], 1);

	motordir[0] = getBNodeInfElement(nn[41], 1);
	motordir[1] = getBNodeInfElement(nn[42], 1);

	motorf[0] = getBNodeInfElement(nn[6], 1);
	motorf[1] = getBNodeInfElement(nn[7], 1);
	motorb[0] = getBNodeInfElement(nn[8], 1);
	motorb[1] = getBNodeInfElement(nn[9], 1);

	motorSpeeds[0] = FIX_TO_DOUBLE(motorf[0]) - FIX_TO_DOUBLE(motorb[0]);
	motorSpeeds[1] = FIX_TO_DOUBLE(motorf[1]) - FIX_TO_DOUBLE(motorb[1]);

	timerAdd();

	fprintf(out, "bt(%d)=%.3f; bC(%d)=%d; bT(%d,1:5)=[%.2f,%.2f,%.2f,%.2f,%.2f]; bnO(%d,1:5)=[%.2f,%.2f,%.2f,%.2f,%.2f]; bD(%d,1:5)=[%.2f,%.2f,%.2f,%.2f,%.2f]; bSp(%d)=%.2f; bMD(%d,1:2)=[%.2f,%.2f]; bMF(%d,1:2)=[%.2f,%.2f]; bMB(%d,1:2)=[%.2f,%.2f]; bS(%d,1:2)=[%.2f,%.2f];\n",
			iteration+1, ((double)iteration),
			iteration+1, collision==true,
			iteration+1, FIX_TO_DOUBLE(target[L]), FIX_TO_DOUBLE(target[FL]), FIX_TO_DOUBLE(target[F]), FIX_TO_DOUBLE(target[FR]), FIX_TO_DOUBLE(target[R]),
			iteration+1, FIX_TO_DOUBLE(FIX_ONE-obstacle[L]), FIX_TO_DOUBLE(FIX_ONE-obstacle[FL]), FIX_TO_DOUBLE(FIX_ONE-obstacle[F]), FIX_TO_DOUBLE(FIX_ONE-obstacle[FR]), FIX_TO_DOUBLE(FIX_ONE-obstacle[R]),
			iteration+1, FIX_TO_DOUBLE(direction[L]), FIX_TO_DOUBLE(direction[FL]), FIX_TO_DOUBLE(direction[F]), FIX_TO_DOUBLE(direction[FR]), FIX_TO_DOUBLE(direction[R]),
			iteration+1, FIX_TO_DOUBLE(bspeed),
			iteration+1, FIX_TO_DOUBLE(motordir[0]), FIX_TO_DOUBLE(motordir[1]),
			iteration+1, FIX_TO_DOUBLE(motorf[0]), FIX_TO_DOUBLE(motorf[1]),
			iteration+1, FIX_TO_DOUBLE(motorb[0]), FIX_TO_DOUBLE(motorb[1]),
			iteration+1, motorSpeeds[0], motorSpeeds[1]
			);

	printf("bt(%d)=%.3f; bC(%d)=%d; bT(%d,1:5)=[%.2f,%.2f,%.2f,%.2f,%.2f]; bnO(%d,1:5)=[%.2f,%.2f,%.2f,%.2f,%.2f]; bD(%d,1:5)=[%.2f,%.2f,%.2f,%.2f,%.2f]; bSp(%d)=%.2f; bMD(%d,1:2)=[%.2f,%.2f]; bMF(%d,1:2)=[%.2f,%.2f]; bMB(%d,1:2)=[%.2f,%.2f]; bS(%d,1:2)=[%.2f,%.2f];\n",
			iteration+1, ((double)iteration),
			iteration+1, collision==true,
			iteration+1, FIX_TO_DOUBLE(target[L]), FIX_TO_DOUBLE(target[FL]), FIX_TO_DOUBLE(target[F]), FIX_TO_DOUBLE(target[FR]), FIX_TO_DOUBLE(target[R]),
			iteration+1, FIX_TO_DOUBLE(FIX_ONE-obstacle[L]), FIX_TO_DOUBLE(FIX_ONE-obstacle[FL]), FIX_TO_DOUBLE(FIX_ONE-obstacle[F]), FIX_TO_DOUBLE(FIX_ONE-obstacle[FR]), FIX_TO_DOUBLE(FIX_ONE-obstacle[R]),
			iteration+1, FIX_TO_DOUBLE(direction[L]), FIX_TO_DOUBLE(direction[FL]), FIX_TO_DOUBLE(direction[F]), FIX_TO_DOUBLE(direction[FR]), FIX_TO_DOUBLE(direction[R]),
			iteration+1, FIX_TO_DOUBLE(bspeed),
			iteration+1, FIX_TO_DOUBLE(motordir[0]), FIX_TO_DOUBLE(motordir[1]),
			iteration+1, FIX_TO_DOUBLE(motorf[0]), FIX_TO_DOUBLE(motorf[1]),
			iteration+1, FIX_TO_DOUBLE(motorb[0]), FIX_TO_DOUBLE(motorb[1]),
			iteration+1, motorSpeeds[0], motorSpeeds[1]
			);
}

fix sensorModel(simxFloat robotPos[3], simxFloat targetPos[3])
{
	double targetVector[3];
	targetVector[0] = (double)targetPos[0] - (double)robotPos[0];
	targetVector[1] = (double)targetPos[1] - (double)robotPos[1];
	targetVector[2] = (double)targetPos[2] - (double)robotPos[2];
	double targetRange = sqrt(targetVector[0]*targetVector[0]+targetVector[1]*targetVector[1]);
	if(targetRange == 0)
		return 0;
	else
	{
		double targetProbability = 0.2 / targetRange;
		if(targetProbability > 1.0)
			targetProbability = 1.0;
		return DOUBLE_TO_FIX(targetProbability);
	}
}

int main(int argc, char **argv)
{
	char bdot[32768];
	unsigned int iteration = 0;

	timerStart();

	initialize(nn);

	int portNb=20000;
	int hFloor, hBody, hTarget, hMotorL, hMotorR;
	int hSensorC, hSensorF, hSensorFL, hSensorFR, hSensorL, hSensorR;

	printf("Connecting to simulation\n");
	int simulator_clientID = simxStart((simxChar*)"127.0.0.1",19997,true,true,5000,5);
	simxStartSimulation(simulator_clientID, simx_opmode_blocking);
	extApi_sleepMs(1000);
	int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);
	if (clientID!=-1)
	{
		printf("Connected to V-REP\n");

		if(boolean)
			out = fopen("/home/mark/york/publications/IEEE_Access_2018_Inference/data/hybridlogicl.m", "w");
		else
			out = fopen("/home/mark/york/publications/IEEE_Access_2018_Inference/data/hybridlogicb.m", "w");
		if(out == NULL) { perror("Could not open data output file"); return(-1); }
		//fprintf(out, "clear;\n");

		printf("Byte Sizes: short=%ld int=%ld long=%ld fix=%ld fixrad=%ld\n",
			sizeof(short), sizeof(int), sizeof(long), sizeof(fix), sizeof(fixrad));

		if(simxGetObjectHandle(clientID, "ResizableFloor_5_25", &hFloor, simx_opmode_blocking) != simx_return_ok)
			{ fprintf(stderr, "Could not get handle to Floor object\n"); return(-1); }
		if(simxGetObjectHandle(clientID, "NestBeacon", &hTarget, simx_opmode_blocking) != simx_return_ok)
			{ fprintf(stderr, "Could not get handle to Target object\n"); return(-1); }
		if(simxGetObjectHandle(clientID, "MaoBot", &hBody, simx_opmode_blocking) != simx_return_ok)
			{ fprintf(stderr, "Could not get handle to MaoBot object\n"); return(-1); }
		if(simxGetObjectHandle(clientID, "MotorLeft", &hMotorL, simx_opmode_blocking) != simx_return_ok)
			{ fprintf(stderr, "Could not get handle to MotorLeft object\n"); return(-1); }
		if(simxGetObjectHandle(clientID, "MotorRight", &hMotorR, simx_opmode_blocking) != simx_return_ok)
			{ fprintf(stderr, "Could not get handle to MotorRight object\n"); return(-1); }
		if(simxGetObjectHandle(clientID, "SensorCollision", &hSensorC, simx_opmode_blocking) != simx_return_ok)
			{ fprintf(stderr, "Could not get handle to SensorCollision object\n"); return(-1); }
		if(simxGetObjectHandle(clientID, "SensorFront", &hSensorF, simx_opmode_blocking) != simx_return_ok)
			{ fprintf(stderr, "Could not get handle to SensorFront object\n"); return(-1); }
		if(simxGetObjectHandle(clientID, "SensorFrontLeft", &hSensorFL, simx_opmode_blocking) != simx_return_ok)
			{ fprintf(stderr, "Could not get handle to SensorFrontLeft object\n"); return(-1); }
		if(simxGetObjectHandle(clientID, "SensorLeft", &hSensorL, simx_opmode_blocking) != simx_return_ok)
			{ fprintf(stderr, "Could not get handle to SensorLeft object\n"); return(-1); }
		if(simxGetObjectHandle(clientID, "SensorFrontRight", &hSensorFR, simx_opmode_blocking) != simx_return_ok)
			{ fprintf(stderr, "Could not get handle to SensorFrontRight object\n"); return(-1); }
		if(simxGetObjectHandle(clientID, "SensorRight", &hSensorR, simx_opmode_blocking) != simx_return_ok)
			{ fprintf(stderr, "Could not get handle to SensorRight object\n"); return(-1); }

		timerStart();

		while(simxGetConnectionId(clientID) != -1)
		{
			simxUChar sensorValue = 0;
			simxFloat sensorPoint[3], sensorSurfaceNormalVector[3];
			simxInt sensorObjectHandle;
			simxFloat robotPos[3], robotOrientation[3], targetPos[3], targetVector[3];
			float targetRange, targetProbability, targetAngle;
			int targetMean = floor(BSIZE/2);

			if(simxGetObjectPosition(clientID,hBody,hFloor,robotPos,simx_opmode_streaming) == simx_return_ok
			&& simxGetObjectOrientation(clientID,hBody,hFloor,robotOrientation,simx_opmode_streaming) == simx_return_ok
			&& simxGetObjectPosition(clientID,hTarget,hFloor,targetPos,simx_opmode_streaming) == simx_return_ok)
			{
				if(simxReadProximitySensor(clientID,hSensorC,&sensorValue,sensorPoint,&sensorObjectHandle,sensorSurfaceNormalVector,simx_opmode_streaming) == simx_return_ok)
					if(sensorValue) collision = true; else collision = false;
				if(simxReadProximitySensor(clientID,hSensorL,&sensorValue,sensorPoint,&sensorObjectHandle,sensorSurfaceNormalVector,simx_opmode_streaming) == simx_return_ok)
					if(sensorValue) obstacle[L] = sensorModel(robotPos,sensorPoint); else obstacle[L] = FIX_ZERO;
				if(simxReadProximitySensor(clientID,hSensorFL,&sensorValue,sensorPoint,&sensorObjectHandle,sensorSurfaceNormalVector,simx_opmode_streaming) == simx_return_ok)
					if(sensorValue) obstacle[FL] = sensorModel(robotPos,sensorPoint); else obstacle[FL] = FIX_ZERO;
				if(simxReadProximitySensor(clientID,hSensorF,&sensorValue,sensorPoint,&sensorObjectHandle,sensorSurfaceNormalVector,simx_opmode_streaming) == simx_return_ok)
					if(sensorValue) obstacle[F] = sensorModel(robotPos,sensorPoint); else obstacle[F] = FIX_ZERO;
				if(simxReadProximitySensor(clientID,hSensorFR,&sensorValue,sensorPoint,&sensorObjectHandle,sensorSurfaceNormalVector,simx_opmode_streaming) == simx_return_ok)
					if(sensorValue) obstacle[FR] = sensorModel(robotPos,sensorPoint); else obstacle[FR] = FIX_ZERO;
				if(simxReadProximitySensor(clientID,hSensorR,&sensorValue,sensorPoint,&sensorObjectHandle,sensorSurfaceNormalVector,simx_opmode_streaming) == simx_return_ok)
					if(sensorValue) obstacle[R] = sensorModel(robotPos,sensorPoint); else obstacle[R] = FIX_ZERO;

				targetVector[0] = ((targetPos[0]-robotPos[0])*cos(robotOrientation[2])-(targetPos[1]-robotPos[1])*sin(robotOrientation[2]));
				targetVector[1] = ((targetPos[0]-robotPos[0])*sin(robotOrientation[2])-(targetPos[1]-robotPos[1])*cos(robotOrientation[2]));
				targetVector[2] = targetPos[2] - robotPos[2];
				targetRange = sqrt(targetVector[0]*targetVector[0]+targetVector[1]*targetVector[1]);
				targetProbability = (1.0 < 1/targetRange?1.0:1/targetRange);
				targetAngle = atan2((float)targetVector[1], (float)targetVector[0]);
				target[L] = gaussian(-M_PI/3, targetRange/2, targetAngle);
				target[FL] = gaussian(-M_PI/6, targetRange/2, targetAngle);
				target[F] = gaussian(0, targetRange/2, targetAngle);
				target[FR] = gaussian(M_PI/6, targetRange/2, targetAngle);
				target[R] = gaussian(M_PI/3, targetRange/2, targetAngle);
				if(target[L] > FIX_ONE) target[L] = FIX_ONE;
				if(target[FL] > FIX_ONE) target[FL] = FIX_ONE;
				if(target[F] > FIX_ONE) target[F] = FIX_ONE;
				if(target[FR] > FIX_ONE) target[FR] = FIX_ONE;
				if(target[R] > FIX_ONE) target[R] = FIX_ONE;
			}

			printf("Sensors: [%.02f %.02f %.02f %.02f %.02f] Target: [%.02f %.02f %.02f] %.02f %.02f [%.02f %.02f %.02f %.02f %.02f] @%.02f #%d\t",
					FIX_TO_FLOAT(obstacle[L]), FIX_TO_FLOAT(obstacle[FL]), FIX_TO_FLOAT(obstacle[F]), FIX_TO_FLOAT(obstacle[FR]), FIX_TO_FLOAT(obstacle[R]),
					targetVector[0], targetVector[1], targetVector[2],
					targetRange, targetProbability,
					FIX_TO_FLOAT(target[L]), FIX_TO_FLOAT(target[FL]), FIX_TO_FLOAT(target[F]), FIX_TO_FLOAT(target[FR]), FIX_TO_FLOAT(target[R]),
					targetAngle, targetMean);

			int simulationTime=simxGetLastCmdTime(clientID);

			if(boolean)
				update_boolean(++iteration);
			else
				update(++iteration);

			if(motorSpeeds[0] > MAX_SPEED) motorSpeeds[0] = MAX_SPEED;
			if(motorSpeeds[0] < -MAX_SPEED) motorSpeeds[0] = -MAX_SPEED;
			if(motorSpeeds[1] > MAX_SPEED) motorSpeeds[1] = MAX_SPEED;
			if(motorSpeeds[1] < -MAX_SPEED) motorSpeeds[1] = -MAX_SPEED;
			simxSetJointTargetVelocity(clientID,hMotorL,motorSpeeds[0],simx_opmode_oneshot);
			simxSetJointTargetVelocity(clientID,hMotorR,motorSpeeds[1],simx_opmode_oneshot);

			extApi_sleepMs(100);
		}
		printf("Simulation Disconnected\n");

		if(boolean)
		{
			timerEnd("Boolean Navigation");
		} else {
			timerEnd("Bayesian Navigation");
			printBNetwork();
			generateDotGraph(bdot, LSIZE);
			DisplayDotGraph(bdot);
			char filename[] = "/home/mark/york/publications/IEEE_Access_2018_Inference/data/graph.dot";
			ExportDotGraph(bdot, filename);
		}
		simxFinish(clientID);
		freeBNetwork();
		fclose(out);
		printf("Simulation Ended\n");
	}
	else printf("Could not connect to simulation\n");

	return 0;
}
