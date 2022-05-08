//
//
//
// Player 1 - Robot A - Attack
//
//
//
#include <cstdio>
#include <iostream>
#include <fstream>
#include <Windows.h>


using namespace std;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )
#define PI 3.14159

#include "image_transfer.h"

// Include this header file for computer vision functions
#include "vision.h"
#include "robot.h"
#include "vision_simulation.h"
#include "timer.h"

extern robot_system S1;

// Identifying the front and rear of Robot A Identifier_Robot_A_Front
void Identifier_Robot_A_Front(image& rgb, image& aaf, image& baf, image& labelaf, image& rgb1f, double& acRAfi, double& acRAfj);
void Identifier_Robot_A_Rear(image& rgb, image& aar, image& bar, image& labelar, image& rgb1r, double& acRAri, double& acRArj);
// Identifying the front and rear of Robot B
void Identifier_Robot_B_Front(image& rgb, image& abf, image& bbf, image& labelbf, image& rgb2f, double& acRBfi, double& acRBfj);
void Identifier_Robot_B_Rear(image& rgb, image& abr, image& bbr, image& labelbr, image& rgb2r, double& acRBri, double& acRBrj);
//Identifying the obstacles by segregating from the robot
void Identifier_Obstacles(image& rgb, image& a, image& b, image& label, image& rgb0, double* icc, double* jcc, int& no_of_obs, double *acOBi, double* acOBj, double& acRAfi, double& acRAfj, double& acRAri, double& acRArj, double& acRBfi, double& acRBfj, double& acRBri, double& acRBrj);

void Robot_Orientation(double& acRAfi, double& acRAfj, double& acRAri, double& acRArj, double& thetaA);
void Path_Planner(image& rgb, double& acRAfi, double& acRAfj, double& acRAri, double& acRArj, double& acRBfi, double& acRBfj, double& acRBri, double& acRBrj, int& no_of_obs, double* acOBi, double* acOBj, double* aicc, double* ajcc, int& localtarget, double* ccnode, double* faicc, double* fajcc, double& distcostRA, double& distcostRAr, double& distcostob1, double& distcostob2, double& distcostRB, double& mincost, double& distcomp, double* localcost, double* lastvisit, double* opennode, double* lastvisited, double& startpoint, double& minstart, double& mingoal, double& goalpoint, int* obsnodes, int& job, int& ico, int& jco, double& aimi, double& aimj);
void control_robot(image& rgb, int& counter, int& pwr, int& pwl, double& acRAfi, double& acRAfj, double& acRAri, double& acRArj, double& acRBfi, double& acRBfj, double& acRBri, double& acRBrj, int& no_of_obs, double* acOBi, double* acOBj, double& thetaA, double tc, double tc0, double& errorprev, double* aicc, double* ajcc, int& localtarget, double& timerc, double* ccnode, double* faicc, double* fajcc, double& aimi, double& aimj);


int main()
{
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0, tp; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	double* icc, * jcc;
	double iccc[7] = {}, jccc[7] = {};
	icc = iccc;
	jcc = jccc;
	double acRAfi, acRAfj, acRAri, acRArj;
	double acRBfi, acRBfj, acRBri, acRBrj;
	double acOBi[10] = {},acOBj[10] = {};
	int counter = 10;
	double thetaA;
	double errorprev = 1.e-10;
	double aicc[200] = {}, ajcc[200] = {};
	double faicc[200] = {}, fajcc[200] = {};
	int dpw = 500;
	int ico = 0, jco = 0;

	double distcostRA, distcostRAr, distcostob1, distcostob2, distcostRB;
	double mincost, distcomp = 700;
	double localcost[4] = {};
	double lastvisit[4] = {};
	int localtarget = 0;
	double timerc = 0;
	double opennode[200] = {};
	double ccnode[200] = {};
	double lastvisited[200] = {};
	double startpoint = 0, minstart = 0, mingoal = 0;
	double goalpoint = 0, aimi = 0, aimj = 0;
	int obsnodes[200] = {};
	int job = 0;
	int no_of_obs = 0;



	// TODO: it might be better to put this model initialization
	// section in a separate function

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1 = 640;
	height1 = 480;

	// number of obstacles
	N_obs = 2;

	x_obs[1] = 270; // pixels
	y_obs[1] = 270; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 135; // pixels
	y_obs[2] = 135; // pixels
	size_obs[2] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	// set robot model parameters ////////

	D = 121.0; // distance between front wheels (pixels)

	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;

	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;

	alpha_max = 3.14159 / 2; // max range of laser / gripper (rad)

	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;

	cout << "\npress space key to begin program.";
	pause();

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	// note it's assumed that the robot points upware in its bmp file

	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	// note that the robot opponent is not currently implemented in 
	// the library, but it will be implemented soon.

	activate_simulation(width1, height1, x_obs, y_obs, size_obs, N_obs,
		"robot_A.bmp", "robot_B.bmp", "background.bmp", "obstacle.bmp", D, Lx, Ly,
		Ax, Ay, alpha_max, n_robot);

	// open an output file if needed for testing or plotting
//	ofstream fout("sim1.txt");
//	fout << scientific;

	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 1;
	level = 1;
	set_simulation_mode(mode, level);

	// set robot initial position (pixels) and angle (rad)
	x0 = 500;
	y0 = 130;
	theta0 = 3.14159 / 4;
	set_robot_position(x0, y0, theta0);

	// set opponent initial position (pixels) and angle (rad)
//	x0 = 200;
//	y0 = 405;
//	theta0 = 3.14159 / 4;
//	set_opponent_position(x0, y0, theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)

	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)

	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;

	// set initial inputs
	set_inputs(pw_l, pw_r, pw_laser, laser,
		light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);

	// opponent inputs
	//pw_l_o = 1500; // pulse width for left wheel servo (us)
	//pw_r_o = 1500; // pulse width for right wheel servo (us)
	//pw_laser_o = 1500; // pulse width for laser servo (us)
	//laser_o = 0; // laser input (0 - off, 1 - fire)

	// manually set opponent inputs for the simulation
	// -- good for testing your program
	//set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
//		opponent_max_speed);

	// regular vision program ////////////////////////////////

	// note that at this point you can write your vision program
	// exactly as before.

	// in addition, you can set the robot inputs to move it around
	// the image and fire the laser.

	image rgb;
	image rgb1f, aaf, baf;
	image labelaf;
	image rgb1r, aar, bar;
	image labelar;
	image rgb2r, abr, bbr;
	image labelbr;
	image rgb2f, abf, bbf;
	image labelbf;
	image rgb0, a, b;
	image label;

	int height, width;
	width = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;

	aaf.type = GREY_IMAGE;
	aaf.width = width;
	aaf.height = height;

	baf.type = GREY_IMAGE;
	baf.width = width;
	baf.height = height;

	labelaf.type = LABEL_IMAGE;
	labelaf.width = width;
	labelaf.height = height;

	rgb1f.type = RGB_IMAGE;
	rgb1f.width = width;
	rgb1f.height = height;

	aar.type = GREY_IMAGE;
	aar.width = width;
	aar.height = height;

	bar.type = GREY_IMAGE;
	bar.width = width;
	bar.height = height;

	labelar.type = LABEL_IMAGE;
	labelar.width = width;
	labelar.height = height;

	rgb1r.type = RGB_IMAGE;
	rgb1r.width = width;
	rgb1r.height = height;

	abf.type = GREY_IMAGE;
	abf.width = width;
	abf.height = height;

	bbf.type = GREY_IMAGE;
	bbf.width = width;
	bbf.height = height;

	labelbf.type = LABEL_IMAGE;
	labelbf.width = width;
	labelbf.height = height;

	rgb2f.type = RGB_IMAGE;
	rgb2f.width = width;
	rgb2f.height = height;

	abr.type = GREY_IMAGE;
	abr.width = width;
	abr.height = height;

	bbr.type = GREY_IMAGE;
	bbr.width = width;
	bbr.height = height;

	labelbr.type = LABEL_IMAGE;
	labelbr.width = width;
	labelbr.height = height;

	rgb2r.type = RGB_IMAGE;
	rgb2r.width = width;
	rgb2r.height = height;

	a.type = GREY_IMAGE;
	a.width = width;
	a.height = height;

	b.type = GREY_IMAGE;
	b.width = width;
	b.height = height;

	label.type = LABEL_IMAGE;
	label.width = width;
	label.height = height;

	rgb0.type = RGB_IMAGE;
	rgb0.width = width;
	rgb0.height = height;

	// note that the vision simulation library currently
	// assumes an image size of 640x480

	// allocate memory for the images
	allocate_image(rgb);

	allocate_image(aaf);
	allocate_image(baf);
	allocate_image(labelaf);
	allocate_image(rgb1f);

	allocate_image(aar);
	allocate_image(bar);
	allocate_image(labelar);
	allocate_image(rgb1r);

	allocate_image(abf);
	allocate_image(bbf);
	allocate_image(labelbf);
	allocate_image(rgb2f);

	allocate_image(abr);
	allocate_image(bbr);
	allocate_image(labelbr);
	allocate_image(rgb2r);

	allocate_image(a);
	allocate_image(b);
	allocate_image(label);
	allocate_image(rgb0);

	wait_for_player();

	// measure initial clock time
	tc0 = high_resolution_time();

	while (1) {

		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);

		tc = high_resolution_time() - tc0;


		set_inputs(pw_l, pw_r, pw_laser, laser, light, light_gradient, light_dir, image_noise, max_speed, opponent_max_speed);
		
		// stop for default case
//		pw_l_o = 1500;
//		pw_r_o = 1500;

/*		// read the keyboard and set the opponent inputs
		if (KEY('I')) {
			pw_l_o = 1500 - dpw;
			pw_r_o = 1500 + dpw;
		}

		if (KEY('K')) {
			pw_l_o = 1500 + dpw;
			pw_r_o = 1500 - dpw;
		}

		if (KEY('J')) {
			pw_l_o = 1500 + dpw;
			pw_r_o = 1500 + dpw;
		}

		if (KEY('L')) {
			pw_l_o = 1500 - dpw;
			pw_r_o = 1500 - dpw;
		}

		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o,
			opponent_max_speed);
*/
		Identifier_Robot_A_Front(rgb, aaf, baf, labelaf, rgb1f, acRAfi, acRAfj); // Identifies robot A front, using green color centroid while erasing the obstacles if it has the same color
		Identifier_Robot_A_Rear(rgb, aar, bar, labelar, rgb1r, acRAri, acRArj); // Identifies robot A rear, using red color centroid while erasing the obstacles if it has the same color
		Identifier_Robot_B_Front(rgb, abf, bbf, labelbf, rgb2f, acRBfi, acRBfj); // Identifies robot B front, using orange color centroid while erasing the obstacles if it has the same color
		Identifier_Robot_B_Rear(rgb, abr, bbr, labelbr, rgb2r, acRBri, acRBrj); // Identifies robot B rear, using cyan (Blue) color centroid while erasing the obstacles if it has the same color

		// Identifies the obstacles regardless of their size, shape and colour. By segregating it from the two robots
		Identifier_Obstacles(rgb, a, b, label, rgb0, icc, jcc,no_of_obs,acOBi,acOBj, acRAfi, acRAfj, acRAri, acRArj, acRBfi, acRBfj, acRBri, acRBrj);

		//Genreates the optimal path to the target location
		Path_Planner(rgb, acRAfi, acRAfj, acRAri, acRArj, acRBfi, acRBfj, acRBri, acRBrj,no_of_obs,acOBi,acOBj,aicc, ajcc, localtarget, ccnode, faicc, fajcc, distcostRA, distcostRAr, distcostob1, distcostob2, distcostRB, mincost, distcomp, localcost, lastvisit, opennode, lastvisited, startpoint, minstart, mingoal, goalpoint, obsnodes, job, ico, jco,aimi,aimj);

		control_robot(rgb, counter, pw_r, pw_l, acRAfi, acRAfj, acRAri, acRArj, acRBfi, acRBfj, acRBri, acRBrj,no_of_obs,acOBi,acOBj, thetaA, tc, tc0, errorprev, aicc, ajcc, localtarget, timerc, ccnode, faicc, fajcc,aimi,aimj);



		double distABf = (sqrt((pow(((double)acRBfi - acRAfi), 2.0)) + (pow(((double)acRBfj - acRAfj), 2.0))));
		double distABr = (sqrt((pow(((double)acRBri - acRAfi), 2.0)) + (pow(((double)acRBrj - acRAfj), 2.0))));
		

		if ((distABf < 96) || (distABr < 89))
			{
				//Fire Laser
			laser = 1;
			}
			// turn off the lasers so we can fire it again later
			else laser = 0;


		view_rgb_image(rgb);

		// don't need to simulate too fast
		Sleep(10); // 100 fps max
		tc0 = tc;
	}
	// free the image memory before the program completes
	free_image(rgb);

	free_image(aaf);
	free_image(baf);
	free_image(labelaf);
	free_image(rgb1f);

	free_image(aar);
	free_image(bar);
	free_image(labelar);
	free_image(rgb1r);

	free_image(abf);
	free_image(bbf);
	free_image(labelbf);
	free_image(rgb2f);

	free_image(abr);
	free_image(bbr);
	free_image(labelbr);
	free_image(rgb2r);

	free_image(a);
	free_image(b);
	free_image(label);
	free_image(rgb0);

	deactivate_vision();
	deactivate_simulation();

	cout << "\ndone.\n";

	return 0;
}


//=========================================
//Identifier Robot A Front identifies the Robot A using the green color as a unique identifier.
//If the objects is of the same color the functions segragtes it by selecting the object with the least amount of mass. 
void Identifier_Robot_A_Front(image& rgb, image& aaf, image& baf, image& labelaf, image& rgb1f, double& acRAfi, double& acRAfj)
{
	int i, j, k;
	ibyte R, G, B;
	int height, width;
	int nlabels;
	ibyte* paf, * pcaf;
	i2byte* plaf;

	width = rgb.width;
	height = rgb.height;
	int size = height * width;

	copy(rgb, rgb1f);
	paf = rgb1f.pdata;

	// green higlight
	for (j = 0; j < height; j++)
	{
		for (i = 0; i < width; i++)
		{
			k = i + width * j;
			pcaf = paf + 3 * k; // pointer to the kth pixel (3 bytes/pixel)
			B = *pcaf;
			G = *(pcaf + 1);
			R = *(pcaf + 2);
			// find green pixels 
			if ((B < 150) && (R < 90) && (G > 160))
			{
				*pcaf = 0;
				*(pcaf + 1) = 255;
				*(pcaf + 2) = 0;
			}
		}
	}
	// invert 
	for (i = 0; i < size; i++)
	{
		B = *paf;
		G = *(paf + 1);
		R = *(paf + 2);

		if ((B == 0) && (R == 0) && (G == 255))
		{
			*paf = 255;
			*(paf + 1) = 255;
			*(paf + 2) = 255;
		}
		else
		{
			*paf = 0;
			*(paf + 1) = 0;
			*(paf + 2) = 0;
		}
		paf += 3;
	}
	copy(rgb1f, aaf);
	erode(aaf, baf);
	copy(baf, aaf);
	copy(aaf, rgb1f);

	// label the objects in a binary image
	// labels go from 1 to nlabels
	label_image(aaf, labelaf, nlabels);
	const int nl = (int)nlabels;
	plaf = (i2byte*)labelaf.pdata;
	int n, m[10] = {};
	if (nlabels > 1)
	{
		// caluclate mass of all labelled objects and store in mass array to finally compare the least mass value as robot
		for (n = 1; n <= nl; n++)
		{
			for (j = 0; j < height; j++) 
			{
				for (i = 0; i < width; i++) 
				{
					if (plaf[j * width + i] == n) 
					{
						m[n]++;
					}
				}
			}
		}
		int min_mass_label = 1, min_mass = m[1]; // minimum mass lab
		for (n = 1; n <= nl; n++)
		{
			if (m[n] < min_mass)
			{
				min_mass_label = n;
				min_mass = m[n];
			}
		}
		centroid(aaf, labelaf, min_mass_label, acRAfi, acRAfj);
	}
	else { // if only 1 label oject consider it as the robot
		centroid(aaf, labelaf, 1, acRAfi, acRAfj);
	}
}


//=========================================
//Identifier Robot A Rear identifies the Robot A using the red color as a unique identifier.
//If the objects is of the same color the functions segragtes it by selecting the object with the least amount of mass.
void Identifier_Robot_A_Rear(image& rgb, image& aar, image& bar, image& labelar, image& rgb1r, double& acRAri, double& acRArj)
{
	int i, j, k;
	ibyte R, G, B;
	int height, width;
	int nlabels;
	ibyte* par, * pcar;
	i2byte* plar;
	width = rgb.width;
	height = rgb.height;

	int size = height * width;

	copy(rgb, rgb1r);
	par = rgb1r.pdata;

	// Red higlight
	for (j = 0; j < height; j++)
	{
		for (i = 0; i < width; i++)
		{
			k = i + width * j;
			pcar = par + 3 * k; // pointer to the kth pixel (3 bytes/pixel)
			B = *pcar;
			G = *(pcar + 1);
			R = *(pcar + 2);
			// find Red pixels 
			if ((B < 100) && (R > 200) && (G < 100))
			{
				*pcar = 0;
				*(pcar + 1) = 255;
				*(pcar + 2) = 0;
			}
		}
	}
	// invert 
	for (i = 0; i < size; i++)
	{
		B = *par;
		G = *(par + 1);
		R = *(par + 2);
		if ((B == 0) && (R == 0) && (G == 255))
		{
			*par = 255;
			*(par + 1) = 255;
			*(par + 2) = 255;
		}
		else
		{
			*par = 0;
			*(par + 1) = 0;
			*(par + 2) = 0;

		}
		par += 3;
	}
	copy(rgb1r, aar);
	erode(aar, bar);
	copy(bar, aar);
	copy(aar, rgb1r);
	// label the objects in a binary image
	// labels go from 1 to nlabels
	label_image(aar, labelar, nlabels);
	const int nl = (int)nlabels;
	plar = (i2byte*)labelar.pdata;
	int n, m[10] = {};
	if (nlabels > 1)
	{
		// caluclate mass of all labelled objects and store in mass array to finally compare the least mass value as robot
		for (n = 1; n <= nl; n++)
		{
			for (j = 0; j < height; j++)
			{
				for (i = 0; i < width; i++)
				{
					if (plar[j * width + i] == n)
					{
						m[n]++;
					}
				}
			}
		}
		int min_mass_label = 1, min_mass = m[1]; // minimum mass lab
		for (n = 1; n <= nl; n++)
		{
			if (m[n] < min_mass)
			{
				min_mass_label = n;
				min_mass = m[n];
			}
		}
		centroid(aar, labelar, min_mass_label, acRAri, acRArj);
	}
	else { // if only 1 label oject consider it as the robot

		centroid(aar, labelar, 1, acRAri, acRArj);
	}

}


//=========================================
//Identifier Robot B Front identifies the Robot A using the orange color as a unique identifier.
//If the objects is of the same color the functions segragtes it by selecting the object with the least amount of mass.
void Identifier_Robot_B_Front(image& rgb, image& abf, image& bbf, image& labelbf, image& rgb2f, double& acRBfi, double& acRBfj)
{
	int i, j, k;
	ibyte R, G, B;
	int height, width;
	int nlabels;
	ibyte* pbf, * pcbf;
	i2byte* plbf;
	width = rgb.width;
	height = rgb.height;
	int size = height * width;

	copy(rgb, rgb2f);
	pbf = rgb2f.pdata;

	// orange scale 
	for (j = 0; j < height; j++)
	{
		for (i = 0; i < width; i++)
		{
			k = i + width * j;
			pcbf = pbf + 3 * k; // pointer to the kth pixel (3 bytes/pixel)
			B = *pcbf;
			G = *(pcbf + 1);
			R = *(pcbf + 2);
			// find orange pixels and calculate their centroid stats
			if ((115 < B < 133 && 180 < G < 198 && R > 245))
			{
				R = 0;
				G = 0;
				B = 255;
				*pcbf = 255;
				*(pcbf + 1) = 0;
				*(pcbf + 2) = 0;
			}
		}
	}
	// invert 
	for (i = 0; i < size; i++)
	{
		B = *pbf;
		G = *(pbf + 1);
		R = *(pbf + 2);

		if ((B == 255) && (R == 0) && (G == 0))
		{
			// useful visual feedback
			*pbf = 255;
			*(pbf + 1) = 255;
			*(pbf + 2) = 255;
		}
		else
		{
			*pbf = 0;
			*(pbf + 1) = 0;
			*(pbf + 2) = 0;
		}
		pbf += 3;
	}
	copy(rgb2f, abf);
	erode(abf, bbf);
	erode(abf, bbf);
	erode(abf, bbf);
	copy(bbf, abf);
	copy(abf, rgb2f);

	// label the objects in a binary image
	// labels go from 1 to nlabels
	label_image(abf, labelbf, nlabels);
	const int nl = (int)nlabels;
	plbf = (i2byte*)labelbf.pdata;
	int n, m[10] = {};
	if (nlabels > 1)
	{// caluclate mass of all labelled objects and store in mass array to finally compare the least mass value as robot
		for (n = 1; n <= nl; n++)
		{
			for (j = 0; j < height; j++)
			{
				for (i = 0; i < width; i++)
				{
					if (plbf[j * width + i] == n)
					{
						m[n]++;
					}
				}
			}
		}
		int min_mass_label = 1, min_mass = m[1]; // minimum mass lab
		for (n = 1; n <= nl; n++)
		{
			if (m[n] < min_mass)
			{
				min_mass_label = n;
				min_mass = m[n];
			}
		}
		centroid(abf, labelbf, min_mass_label, acRBfi, acRBfj);
	}
	else { // if only 1 label oject consider it as the robot
		centroid(abf, labelbf, 1, acRBfi, acRBfj);
	}
}


//=========================================
//Identifier Robot B Rear identifies the Robot A using the cyan(blue) color as a unique identifier.
//If the objects is of the same color the functions segragtes it by selecting the object with the least amount of mass.
void Identifier_Robot_B_Rear(image& rgb, image& abr, image& bbr, image& labelbr, image& rgb2r, double& acRBri, double& acRBrj)
{
	int i, j, k;
	ibyte R, G, B;
	int height, width;
	int nlabels;
	ibyte* pbr, * pcbr;
	i2byte* plbr;
	width = rgb.width;
	height = rgb.height;
	int size = height * width;

	copy(rgb, rgb2r);
	pbr = rgb2r.pdata;

	// cyan scale 
	for (j = 0; j < height; j++)
	{
		for (i = 0; i < width; i++)
		{
			k = i + width * j;
			pcbr = pbr + 3 * k; // pointer to the kth pixel (3 bytes/pixel)
			B = *pcbr;
			G = *(pcbr + 1);
			R = *(pcbr + 2);
			// find cyan pixels and calculate their centroid stats
			if (R < 80 && B > 190 && G < 180)
			{
				R = 0;
				G = 0;
				B = 255;
				*pcbr = 255;
				*(pcbr + 1) = 0;
				*(pcbr + 2) = 0;
			}
		}
	}
	// invert 
	for (i = 0; i < size; i++) {
		B = *pbr;
		G = *(pbr + 1);
		R = *(pbr + 2);
		if ((B == 255) && (R == 0) && (G == 0))
		{
			*pbr = 255;
			*(pbr + 1) = 255;
			*(pbr + 2) = 255;
		}
		else
		{
			*pbr = 0;
			*(pbr + 1) = 0;
			*(pbr + 2) = 0;
		}
		pbr += 3;
	}
	copy(rgb2r, abr);
	erode(abr, bbr);
	erode(abr, bbr);
	erode(abr, bbr);
	copy(bbr, abr);
	copy(abr, rgb2r);
	// label the objects in a binary image
	// labels go from 1 to nlabels
	label_image(abr, labelbr, nlabels);
	const int nl = (int)nlabels;
	plbr = (i2byte*)labelbr.pdata;
	int n, m[10] = {};
	if (nlabels > 1)
	{// caluclate mass of all labelled objects and store in mass array to finally compare the least mass value as robot
		for (n = 1; n <= nl; n++)
		{
			for (j = 0; j < height; j++)
			{
				for (i = 0; i < width; i++)
				{
					if (plbr[j * width + i] == n)
					{
						m[n]++;
					}
				}
			}
		}
		int min_mass_label = 1, min_mass = m[1]; // minimum mass lab
		for (n = 1; n <= nl; n++)
		{
			if (m[n] < min_mass)
			{
				min_mass_label = n;
				min_mass = m[n];
			}
		}
		centroid(abr, labelbr, min_mass_label, acRBri, acRBrj);
	}
	else { // if only 1 label oject consider it as the robot
		centroid(abr, labelbr, 1, acRBri, acRBrj);

	}

}

//=============================================================
//Identifier Obstacle, finds the centroid of the all the objects in the image , by comparing the euclidean distances 
// with the above obtained centroids of the robots and segregating the obstacles from it.
void Identifier_Obstacles(image& rgb, image& a, image& b, image& label, image& rgb0, double* icc, double* jcc, int & no_of_obs, double* acOBi, double* acOBj, double& acRAfi, double& acRAfj, double& acRAri, double& acRArj, double& acRBfi, double& acRBfj, double& acRBri, double& acRBrj)
{
	int k;
	double ic, jc, icd[10] ={}, jcd[10] ={};
	ibyte R, G, B;
	int height, width;
	int nlabels;
	ibyte* pb, * pcb;
	i2byte* pl;
	width = rgb.width;
	height = rgb.height;
	int size = height * width;

	copy(rgb, rgb0);

	pb = rgb0.pdata;

	// to enhance all the colours of the robot circle centroids and the obstacles.
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
		{
			k = i + width * j;
			pcb = pb + 3 * k; // pointer to the kth pixel (3 bytes/pixel)
			B = *pcb;
			G = *(pcb + 1);
			R = *(pcb + 2);
			// find orange pixels
			if ((115 < B < 129 && 180 < G < 191 && R > 245))
			{
				R = 0;
				G = 0;
				B = 0;
				*pcb = B;
				*(pcb + 1) = G;
				*(pcb + 2) = R;
				// Red scale 
				if (R > 200 && B < 100 && G < 100)
				{
					B = 0;
					G = 0;
					R = 0;
					*pcb = B;
					*(pcb + 1) = G;
					*(pcb + 2) = R;
				}

				// Green scale 
				if (B < 140 && R < 140 && G > 160)
				{
					B = 0;
					G = 0;
					R = 0;
					*pcb = B;
					*(pcb + 1) = G;
					*(pcb + 2) = R;
				}
				// cyan scale 
				if (R < 60 && B > 200 && G < 165)
				{
					B = 0;
					G = 0;
					R = 0;
					*pcb = B;
					*(pcb + 1) = G;
					*(pcb + 2) = R;
				}
			}
		}
	}
	// invert the image
	copy(rgb0, a);
	scale(a, b);
	copy(b, rgb0);
	lowpass_filter(b, a);
	copy(a, b);
	copy(a, rgb0);
	threshold(a, b, 170);
	copy(b, a);
	copy(a, rgb0); // convert to RGB image format
	// invert the image
	invert(a, b);
	copy(b, a);
	copy(a, rgb0);    // convert to RGB image format
	// perform an erosion function to remove noise (small objects)
	erode(a, b);
	erode(b, a);
	erode(a, b);
	erode(b, a);
	erode(a, b);
	erode(b, a);
	erode(a, b);
	erode(b, a);
	erode(a, b);
	erode(b, a);
	erode(a, b);
	dialate(b, a);
	copy(a, b);
	copy(a, rgb0);    // convert to RGB image format
	// label the objects in a binary image; hence using image a
	// labels go from 1 to nlabels
	label_image(a, label, nlabels);
	// compute the centroid of the last object
	for (int i = 1; i <= nlabels; i++)
	{
		centroid(a, label, i, ic, jc);
		icc[i] = (int)ic;
		jcc[i] = (int)jc;
		//	cout << " \n icc = " << icc[i];
		//	cout << " \n jcc = " << jcc[i] << "\n";

	}
	no_of_obs = nlabels - 4;
	// convert to RGB image format
	copy(a, rgb0);
	double comp_robot_centroid[10] = {};
	int flag = 1,chk =1;
	for (int i = 1; i <= nlabels; i++)
	{
		comp_robot_centroid[1] = sqrt((pow(((double)acRAfi - icc[i]), 2.0)) + (pow(((double)acRAfj - jcc[i]), 2.0)));
		comp_robot_centroid[2] = sqrt((pow(((double)acRAri - icc[i]), 2.0)) + (pow(((double)acRArj - jcc[i]), 2.0)));
		comp_robot_centroid[3] = sqrt((pow(((double)acRBfi - icc[i]), 2.0)) + (pow(((double)acRBfj - jcc[i]), 2.0)));
		comp_robot_centroid[4] = sqrt((pow(((double)acRBri - icc[i]), 2.0)) + (pow(((double)acRBrj - jcc[i]), 2.0)));

		if ((comp_robot_centroid[1] < 5) || (comp_robot_centroid[2] < 5) || (comp_robot_centroid[3] < 5) || (comp_robot_centroid[4] < 5))
		{
		}
		else
		{
			flag = chk;
			acOBi[flag] = (int)icc[i];
			acOBj[flag] = (int)jcc[i];	
			chk++;
		}
	}


}



//Using the A* path planning method the following fuction is able to generate the most optimal trajectory between the desired start and goal points.
void Path_Planner(image& rgb, double& acRAfi, double& acRAfj, double& acRAri, double& acRArj, double& acRBfi, double& acRBfj, double& acRBri, double& acRBrj, int& no_of_obs, double* acOBi, double* acOBj, double* aicc, double* ajcc, int& localtarget, double* ccnode, double* faicc, double* fajcc, double& distcostRA, double& distcostRAr, double& distcostob, double& distcostob2, double& distcostRB, double& mincost, double& distcomp, double* localcost, double* lastvisit, double* opennode, double* lastvisited, double& startpoint, double& minstart, double& mingoal, double& goalpoint, int* obsnodes, int& job, int& ico, int& jco, double& aimi, double& aimj)
{

	ico = 0;
	for (int i = 90; i <= 580; i += 40)
	{

		for (int j = 90; j <= 420; j += 50)
		{
			aicc[ico] = i;
			ajcc[ico] = j;
			ico++;
		}
	}

	// cost fnx
	// distance b/w robot A & nearest way point co ordinates - dist b/w selected way point & obstacles  & dist b/w way point & goal
	// use local & global waypoints for conituous path planning
	// local cost calculate and convert to global 
	// save all nodes inside open node

	// set obstacle nodes
	job = 0;
	for (int i = 0; i < ico; i++)
	{
		for (int  j = 1; j <= no_of_obs; j++)
		{
			distcostob = sqrt((pow(((double)acOBi[j] - aicc[i]), 2.0)) + (pow(((double)acOBj[j] - ajcc[i]), 2.0)));
			if (distcostob < 80 )
			{
				obsnodes[job] = i;
				job++;
			}
		}
	}


	
	// open node store the blue pixels ( all pixels - red pixels i value)
	int jj = 0;
	for (int i = 0; i < ico; i++)
	{
		int checker = 0;
		for (int j = 0; j < job; j++)
		{
			if (i == obsnodes[j])
			{
				checker = 1;
				break;
			}
		}
		if (checker == 0)
		{
			opennode[jj] = i;
			jj++;
		}
	}

	for (int i = 0; i < jj; i++)
	{
		const int val = opennode[i];
		faicc[i] = aicc[val];
		fajcc[i] = ajcc[val];

		//draw_point_rgb(rgb, faicc[i], fajcc[i], 0, 0, 255);
	}

	// set start node
	distcomp = 700;
	for (int i = 0; i < jj; i++)
	{
		distcostRA = sqrt((pow(((double)acRAfi - faicc[i]), 2.0)) + (pow(((double)acRAfj - fajcc[i]), 2.0)));
		if (distcomp > distcostRA)
		{
			minstart = i;
			distcomp = distcostRA;
		}
	}
	startpoint = minstart;

	// set goal node
	distcomp = 700;
	for (int i = 0; i < jj; i++)
	{
		distcostRB = sqrt((pow(((double)aimi - faicc[i]), 2.0)) + (pow(((double)aimj - fajcc[i]), 2.0)));
		if (distcomp > distcostRB)
		{
			mingoal = i;
			distcomp = distcostRB;
		}
	}
	goalpoint = mingoal;
	ccnode[0] = startpoint;
	lastvisited[0] = startpoint;

	for (int i = 0; i < jj; i++)
	{
		int upd = 0;
		// reset previous local last visits
		const int lastvisitnode = lastvisited[i];
		int chkk = lastvisitnode;
		int chkk1 = lastvisitnode;
		int chkk2 = lastvisitnode;
		int mino = lastvisitnode;
		for (int j = 0; j < 4; j++)
		{
			distcomp = 800;
			for (int io = 0; io < jj; io++)
			{
				// to find the nearest node to the last visited node
				if ((io == lastvisitnode) || (io == chkk) || (io == chkk1) || (io == chkk2))
				{
				}
				else
				{
					distcostRA = sqrt((pow(((double)faicc[lastvisitnode] - faicc[io]), 2.0)) + (pow(((double)fajcc[lastvisitnode] - fajcc[io]), 2.0)));
					if (distcomp > distcostRA)
					{
						mino = io;
						distcomp = distcostRA;
					}
				}
			}

			const int min = mino;
			const int goal = goalpoint;
			distcostRB = sqrt((pow(((double)faicc[goal] - faicc[min]), 2.0)) + (pow(((double)fajcc[goal] - fajcc[min]), 2.0)));
			localcost[upd] = (distcostRB);
			lastvisit[upd] = min;// visited  donot choose it 
			if (j == 0)
			{
				chkk = lastvisit[upd];
				upd++;
			}
			else if (j == 1)
			{
				chkk1 = lastvisit[upd];
				upd++;
			}
			else if (j == 2)
			{
				chkk2 = lastvisit[upd];
				upd++;
			}
			else
			{
				upd++;
			}

			// finally we obtain the local cost values of the nearest three nodes and below we compare the cost and chose our next visit node
		}
		// local 3 node cost comparision and selecting the minimum as the next visit point & same time store in closednode
		mincost = localcost[0];
		localtarget = lastvisit[0];
		for (int i = 0; i < 4; i++)
		{
			if (mincost > localcost[i])
			{
				mincost = localcost[i];
				const int minu = mincost;
				localtarget = lastvisit[i];
			}
		}
		lastvisited[i + 1] = localtarget;
		// ccnode will store the previously visited location and determine the next forward trajectory from the said location. 
		ccnode[i + 1] = localtarget;
		localtarget = 0;
		localcost[3] = {};
		lastvisit[3] = {};
	}

	for (int i = 0; i < jj; i++)
	{
		const int closedpath = ccnode[i];
		//draw_point_rgb(rgb, faicc[closedpath], fajcc[closedpath], 255, 0, 0);

	}

}




// we want to design a system where we have desired xdes and y des values to go, we compare it from the sim step x & y values
// compute error & design a pid to set input to system with input as velocity and moves robot to x & y co ordinates
// design a pid controller to move to desired xdes & ydes values

void control_robot(image& rgb, int& counter, int& pwr, int& pwl, double& acRAfi, double& acRAfj, double& acRAri, double& acRArj, double& acRBfi, double& acRBfj, double& acRBri, double& acRBrj, int& no_of_obs, double* acOBi, double* acOBj, double& thetaA, double tc, double tc0, double& errorprev, double* aicc, double* ajcc, int& localtarget, double& timerc, double* ccnode, double* faicc, double* fajcc,double &aimi,double &aimj)
{
	double dpw = 500;
	int height, width;
	int dtt = 1;
	int dt = 1.e-10, kp = 60, ki = 0.2, kd = 0.6;
	double error = 0, errord, errorp = 0, errori = 0, pidout;
	width = rgb.width;
	height = rgb.height;
	double x, xdes, y, ydes, theta;
	const int pw_Mean = 1500;
	const int pw_Max = 500;
	const int pw_Min = -500;
	Robot_Orientation(acRAfi, acRAfj, acRAri, acRArj, thetaA);
	x = acRAri;
	y = acRArj;
	theta = thetaA;

	int prev = ccnode[100];
	int ii = 0;
	counter = 10;
	dt = tc - tc0;

	const int target = ccnode[1];

	xdes = faicc[target];
	ydes = fajcc[target];

	prev = ccnode[1];
	double dist = (sqrt((pow(((double)faicc[target] - acRAfi), 2.0)) + (pow(((double)fajcc[target] - acRAfj), 2.0))));
	double distABf = (sqrt((pow(((double)acRBfi - acRAfi), 2.0)) + (pow(((double)acRBfj - acRAfj), 2.0))));
	double distABr = (sqrt((pow(((double)acRBri - acRAfi), 2.0)) + (pow(((double)acRBrj - acRAfj), 2.0))));

	aimi = acRBri; aimj = acRBrj;

	//if (counter == 0) {
	//	aimi = acRBri; aimj = acRBrj;
//	}
//	else {
		// 480 x 640 = 240 x 320
	//	if ((acRBfi < 320 || acRBfj < 240)) { aimi = faicc[70]; aimj = faicc[70]; }
	//	else { aimi = faicc[3]; aimj = faicc[3]; }
	//	cout << "\n i changed the aim beacuse counter is 1";
//	}
//	if ((distABr < distABf)) counter = 0;

	
	int dist_flag_r = 0, dist_flag_f = 0;
	double distOBr[10] = {}, distOBf[10] = {};

	// sonar to detect if any obs is less than some pixels distance.
	for (int i = 1; i <= no_of_obs; i++)
	{
		distOBf[i] = (sqrt((pow(((double)acOBi[i] - acRAfi), 2.0)) + (pow(((double)acOBj[i] - acRAfj), 2.0))));
		distOBr[i] = (sqrt((pow(((double)acOBi[i] - acRAri), 2.0)) + (pow(((double)acOBj[i] - acRArj), 2.0))));

		if (distOBf[i] < 130.0) dist_flag_f = 1;
		else dist_flag_f = 0;

		if (distOBr[i] < 90.0) dist_flag_r = 1;
		else dist_flag_r = 0;

	}

	// boundary & obstacle avoidance from rear of robot
	if (((acRArj > (height - 45.0)) || (acRAri > (width - 45.0)) || (acRAri < 45.0) || (acRArj < 45.0) || (dist_flag_r == 1)))
	{
		// go straight & escape collision
		//cout << "\n boundary & object collision is activated";
		pwl = 1000;
		pwr = 2000;
	}
	else
	{
		if (dist < 7)
		{
			// cout << "\n " << 0 << "st  local target reached!! = " << ccnode[1];
			ii++;
		}
		else
		{
			double radians, target_angle, g_target_angle, ang_diff, targetslope, robotslope;
			robotslope = (acRAfj - acRArj) / ((acRAfi - acRAri));
			targetslope = (ydes - acRArj) / ((xdes - acRAri));
			radians = atan2((robotslope - targetslope), (1 + (robotslope * targetslope)));
			
			target_angle = (int((radians * (180 / PI)) + 360) % 360); // Angle between the robots front and the next waypoint.
			int pos = (acRAfi - xdes) * (acRArj - ydes) - (acRAfj - ydes) * (acRAri - xdes);

			// checks if the next target is to the left ot to the right of the current robot position.
			if (pos < 0) {
				g_target_angle = theta + target_angle;
				ang_diff = (int)(g_target_angle - theta);
			}
			else
			{
				g_target_angle = theta - target_angle;
				ang_diff = (int)(g_target_angle - theta);
			}
		
			
			int chu =0;
			if ((dist_flag_r) || (dist_flag_f))
			{
				chu = 4;
			}
			else
			{
				chu = 18;	
			}

			if ((ang_diff > chu))
			{
				error = abs(ang_diff);
				errori += (error * dt);
				errord = (error - errorprev) / (dt + 1.e-10);
				pidout = (kp * error) + (ki * errori) + (kd * errord);
				if (pidout > pw_Max) pidout = pw_Max;	//bound check
				if (pidout < pw_Min) pidout = pw_Min;	//bound check

				pwl = (int)(pw_Mean + pidout);
				pwr = (int)(pw_Mean + pidout);
				// Turning Left

			}
			else if ((ang_diff < -chu))
			{
				error = abs(ang_diff);
				errori += (error * dt);
				errord = (error - errorprev) / (dt + 1.e-10);
				pidout = (kp * error) + (ki * errori) + (kd * errord);
				if (pidout > pw_Max) pidout = pw_Max;	//bound check
				if (pidout < pw_Min) pidout = pw_Min;	//bound check

				pwl = (int)(pw_Mean - pidout);
				pwr = (int)(pw_Mean - pidout);
				// Turning Right

			}
			else
			{
				if ((distABf < 100) || (distABr < 94))
				{
					if ((distABf < 90) || (distABr < 90))
					{
						//go backards by default
						pwl = (2000);
						pwr = (1000);
						// too close to opponent robot
					}

					else
					{
						//cout << "\n I am swirling attacking now";
						// swirl robot to target robot front 
						double radianss, target_angles, g_target_angles, ang_diffs, targetslopes, robotslopes;
						double xxdes = acRBfi; double yydes = acRBfj;
						xdes = xxdes; ydes = yydes;
						robotslopes = (acRAfj - acRArj) / ((acRAfi - acRAri));
						targetslopes = (yydes - acRArj) / ((xxdes - acRAri));
						radianss = atan2((robotslopes - targetslopes), (1 + (robotslopes * targetslopes)));
						target_angles = (int((radianss * (180 / PI)) + 360) % 360); // Angle between the robots front and the next waypoint.

						int pos = (acRAfi - xxdes) * (acRArj - yydes) - (acRAfj - yydes) * (acRAri - xxdes);
						if (pos < 0) {
							g_target_angles = theta + target_angles;
							ang_diffs = (int)(g_target_angles - theta);
						}
						else
						{
							g_target_angles = theta - target_angles;
							ang_diffs = (int)(g_target_angles - theta);
						}
						int chus = 1; // stationary tracking car

						if ((ang_diffs > chus))
						{
							error = abs(ang_diffs);
							errori += (error * dt);
							errord = (error - errorprev) / (dt + 1.e-10);
							pidout = (kp * error) + (ki * errori) + (kd * errord);
							if (pidout > pw_Max) pidout = pw_Max;	//bound check
							if (pidout < pw_Min) pidout = pw_Min;	//bound check

							pwl = (int)(pw_Mean + pidout);
							pwr = (int)(pw_Mean + pidout);
							//swirl Left
					

						}
						else if ((ang_diffs < -chus))
						{
							error = abs(ang_diffs);
							errori += (error * dt);
							errord = (error - errorprev) / (dt + 1.e-10);
							pidout = (kp * error) + (ki * errori) + (kd * errord);
							if (pidout > pw_Max) pidout = pw_Max;	//bound check
							if (pidout < pw_Min) pidout = pw_Min;	//bound check

							pwl = (int)(pw_Mean - pidout);
							pwr = (int)(pw_Mean - pidout);
							//swirl Right
						}
					}
				}
				else
				{
					//go straight by default
					pwl = (1000);
					pwr = (2000);
				}
			}
			errorprev = error;
		}
	}
	
}


//Robot Orientation determines the angle of the robot with respect to the global cordinates.
void Robot_Orientation(double& acRAfi, double& acRAfj, double& acRAri, double& acRArj, double& thetaA)
{
	//identify X & Y points
	// find the slope of the robot A using 2 front & Rear points
	// using slope we will calculate the angle using atan2
	int x = (int)acRAfi;
	int y = (int)acRAfj;
	double radians;
	radians = atan2((acRAfj - acRArj), (acRAfi - acRAri)); // radians
	thetaA = (int((radians * (180 / PI)) + 360) % 360);
	//cout << " \n Angle of robot A (degrees) = " << thetaA;
}