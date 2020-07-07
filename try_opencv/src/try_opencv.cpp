// PA_Gen2.cpp : Defines the entry point for the console application.
//

#include "stdio.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <math.h>

#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
//=== MAIN PROGRAM (NEW)===

//Update ==> Mulai seleksi bagian tiang gawang (mentah)

//=== Coba Pakai Vector ===
using namespace cv;
using namespace std;

//*
//=== NEW Data ===
//int GminH = 21, GminS = 131, GminV = 63; int GmaxH = 118, GmaxS = 239, GmaxV = 138;
//int WminH = 25, WminS = 0, WminV = 89; int WmaxH = 123, WmaxS = 136, WmaxV = 168;
//Point centerP = Point(665,320);
//=== OLD Data ===
int GminH = 65, GminS = 114, GminV = 93; int GmaxH = 92, GmaxS = 255, GmaxV = 235;
int WminH = 58, WminS = 4, WminV = 97; int WmaxH = 108, WmaxS = 163, WmaxV = 164;
Point centerP = Point(620,380);
//=== OLD Data (new-fail) ===
// int GminH = 70, GminS = 78, GminV = 73; int GmaxH = 86, GmaxS = 255, GmaxV = 207;
// int WminH = 45, WminS = 0, WminV = 129; int WmaxH = 125, WmaxS = 175, WmaxV = 177;
// Point centerP = Point(620,380);

int filecount = 0, datacount = 0; //increment this
int frameCount = 0;
stringstream filename;

FILE *dtCoor;

float rT1, rT2, Xpos, Ypos;
float rT1_copy, rT2_copy, Xpos_copy, Ypos_copy;

struct myMark  // informasi titik terduga bagian gawang [coor, deg, rad]
{
	Point coor;
	int deg;
	int radius;
	int high;
} tempM;

struct GArea  // informasi garis yang terduga gawang [coorA, coorB, dist, degA, degB]
{
	Point coorA;
	Point coorB;
	int dist;
	int degA;
	int degB;
	int highest;

	int idx0;
	int idx1;
} tempP;

struct P_Point // informasi titik sederhana [coor, deg]
{
	Point coor;
	int deg;
};

Mat frame, frame_HSV, frame_out, getFrame;
Mat lapangan;
Vec3b colorHSV[360][401];
//vector<Point> myGoal;
vector<myMark> allPoint;
//vector<GArea> pole;
const double PI = 3.14159265;

P_Point PoleA, PoleB;
P_Point TiangA, TiangB; 
float arrayT1[3]={NULL,NULL,NULL};
float arrayT2[3]={NULL,NULL,NULL};

int ti=0;
float hitungTiang(float Tiang[3], float nilai) // Mencoba menstabilkan posisi
{
    if(Tiang[2]==NULL){
        Tiang[ti]=nilai;
        ti++;
        return(nilai);
    }
    else
    {
        float sum = 0.0;
        for(ti=0; ti<2; ti++)
        {
            Tiang[ti] = Tiang[ti+1];
            sum += Tiang[ti];
        }
        sum += nilai;
        Tiang[2]=sum/3;

        return (sum/3);
    }
}

float jarak(Point tiang) // mengkonversikan deteksi dalam meter
{
	float a1 = 1.2170;
	float a2 = 1.6429;
	float a3 = 1.3874;
	float f = 616.15286;
	float bag1,bag2,bag3,bag4,result;

	int pix = sqrt(pow((tiang.x-centerP.x),2) + pow((tiang.y-centerP.y),2));

	bag1 = a1*a3*f*pix;
	bag2 = pow(pix, 2)*pow(a3, 2);
	bag3 = pow(f, 2) - (pow(pix, 2)*(pow(a2, 2) - pow(a1, 2)));
	bag4 = pow(f, 2) - (pow(pix, 2)*pow(a2, 2));

	result = (bag1 + sqrt(bag2*bag3)) / bag4;

	return (result);
}

void estimasi(float T1, float T2) // mengkonversikan untuk dapat kordinat
{

	float s = (2 + T1 + T2) / 2;
	float aCosA = acos((pow(T1,2) + 4 - pow(T2,2)) / (4*T1));
	Ypos = sin(aCosA) * T1;
	Xpos = (Ypos / tan(aCosA)) + 3;

	//printf("arcCos= %5f \n", aCosA);
	//printf("Posisi X = %4.2f   Y = %4.2f \n\n", Xpos, Ypos);
}

void estimasi_copy(float T1, float T2) // mengkonversikan untuk dapat kordinat
{

	float s = (2 + T1 + T2) / 2;
	float aCosA = acos((pow(T1,2) + 4 - pow(T2,2)) / (4*T1));
	Ypos_copy = sin(aCosA) * T1;
	Xpos_copy = (Ypos / tan(aCosA)) + 3;

	//printf("arcCos= %5f \n", aCosA);
	//printf("Posisi X = %4.2f   Y = %4.2f \n\n", Xpos, Ypos);
}

float orient; //sementara
int xDraw, yDraw;
void orientasi()
{
	float degShould = atan((3-Xpos) / Ypos)  * 57.2957795;
	orient = degShould - (TiangA.deg - 90);
	if (orient < -180) orient = 360 + orient;
	//printf("hitung: %5.2f\tradial: %3d\nselisih: %5.2f \n\n", degShould, (TiangA.deg - 90), orient);

	xDraw = sin(orient*PI/180)*12;
	yDraw = cos(orient*PI/180)*12;
}

bool zone = true;
float tha[2]={NULL,NULL};
void all_estimate(float theta)
{
	tha[1] = theta;
	if(tha[0]==NULL) tha[0] = theta;

	if (max_element(tha,tha+1) - min_element(tha,tha+1) > 90) zone = !zone;

	if(zone = false)
	{
		Xpos = 8 - Xpos;
		Ypos = 12 - Ypos;
		if (orient > 0) orient = orient - 180;
		else orient = orient + 180;
	}
	
	tha[0] = tha[1];
}

void marker(vector<myMark> allPoint) // mengelompokkan beberapa titik ke dalam beberapa garis untuk selanjutnya diidentifikasi garis mana yang merupakan gawang
{	//Mengelompokkan titik
	vector<GArea> pole;
	bool NewPole = true;
	int dBefore;
	int i = 0;
	for (int v = 0; v < allPoint.size(); v++)
	{
		if (NewPole == true)
		{
			tempP.coorA = allPoint[v].coor;
			tempP.coorB = allPoint[v].coor;
			tempP.dist = 0;
			tempP.degA = allPoint[v].deg;
			tempP.degB = allPoint[v].deg;
			tempP.highest = allPoint[v].high;
			tempP.idx0 = v;
			tempP.idx1 = v;

			pole.push_back(tempP);
			NewPole = false;
		}

		if ((allPoint[v].deg - pole[i].degB) < 8) //toleransi jarak antar titik
		{
			pole[i].dist++;
			pole[i].coorB = allPoint[v].coor;
			pole[i].degB = allPoint[v].deg;
			pole[i].idx1 = v;

			if (pole[i].highest < allPoint[v].high) pole[i].highest = allPoint[v].high;
		}
		else
		{
			//dBefore = allPoint[v].deg;
			v--;
			NewPole = true;
			i++;
		}
	}
	//printf("\n");
	// for(int i = 0; i < pole.size(); i++)
	// 	printf("LINE %2d : H= %3d    L=%3d\n", i, pole[i].highest, pole[i].dist);

	//Mencari garis terpanjang
	GArea PP; PP.dist = 0;
	for (int i = 0; i < pole.size(); i++)
	{
		if (pole[i].dist > PP.dist)
			PP = pole[i];
	}
	line(frame, PP.coorA, PP.coorB, Scalar(155, 0, 155), 2);

	int h = 0;
	for(int i = PP.idx0; i < (PP.dist/2.5 + PP.idx0); i++)
	{
		if(allPoint[i].high > h)
		{
			TiangA.coor = allPoint[i].coor;
			TiangA.deg = allPoint[i].deg;
			h = allPoint[i].high;
		}
	}
	circle(frame, TiangA.coor ,5,Scalar(255,0,255),3);
	line(frame, centerP, TiangA.coor, Scalar(255,255,0), 2);

	h = 0;
	for(int i = PP.idx1; i > (PP.idx1 - PP.dist/2.5); i--)
	{
		if(allPoint[i].high > h)
		{
			TiangB.coor = allPoint[i].coor;
			TiangB.deg = allPoint[i].deg;
			h = allPoint[i].high;
		}
	}
	circle(frame, TiangB.coor , 5, Scalar(255,0,255), 3);
	line(frame, centerP, TiangB.coor, Scalar(255,255,0), 2);

	line(frame, TiangA.coor, TiangB.coor, Scalar(255,255,0), 2);

	rT1 = hitungTiang(arrayT1,jarak(TiangA.coor));
	rT2 = hitungTiang(arrayT2,jarak(TiangB.coor));
	estimasi(rT1, rT2);

    //rT1_copy = jarak(TiangA.coor);
	//rT2_copy = jarak(TiangB.coor);
	//estimasi_copy(rT1_copy, rT2_copy);
	orientasi();

	all_estimate(orient);

	ROS_INFO("Posisi X = %4.2f   Y = %4.2f   Theta = %5.2f", Xpos, Ypos, orient);
	
	pole.clear();

	PoleA.coor = PP.coorA;
	PoleA.deg = PP.degA;
	PoleB.coor = PP.coorB;
	PoleB.deg = PP.degB;
}

void detDeg(int deg) // Pencarian titik-titik yang diindikasi bagian gawang melalui deteksi pertemuan warna [Hijau-Putih]
{
	int j = 0;
	while (j < 250)
	{
		int x = centerP.x - ((400-j) * cos(deg*PI / 180)); //if (x > 1280) x = 1280; else if (x < 0) x = 0;
		int y = centerP.y - ((400-j) * sin(deg*PI / 180)); if (y >= 720) y = 719; else if (y < 0) y = 0;
		//line(frame, Point(centerP.x, centerP.y), Point(x, y), Scalar(0, 125, 125));

		colorHSV[deg][j] = frame_HSV.at<Vec3b>(y, x);
		if (GminH < colorHSV[deg][j].val[0] && colorHSV[deg][j].val[0] < GmaxH && GminS < colorHSV[deg][j].val[1] && colorHSV[deg][j].val[1] < GmaxS && GminV < colorHSV[deg][j].val[2] && colorHSV[deg][j].val[2] < GmaxV) //Warna Hijau
			if (WminH < colorHSV[deg][j - 1].val[0] && colorHSV[deg][j - 1].val[0] < WmaxH && WminS < colorHSV[deg][j - 1].val[1] && colorHSV[deg][j - 1].val[1] < WmaxS && WminV < colorHSV[deg][j - 1].val[2] && colorHSV[deg][j - 1].val[2] < WmaxV) //Warna Putih
			{
				static bool hColor = true;
				int t = 1;
				while (1)
				{
					if (WminH < colorHSV[deg][j - t].val[0] && colorHSV[deg][j - t].val[0] < WmaxH && WminS < colorHSV[deg][j - t].val[1] && colorHSV[deg][j - t].val[1] < WmaxS && WminV < colorHSV[deg][j - t].val[2] && colorHSV[deg][j - t].val[2] < WmaxV) t++;
					else break;
				}
				int x0 = centerP.x - (((400 - j) + t) * cos(deg*PI / 180));
				int y0 = centerP.y - (((400 - j) + t) * sin(deg*PI / 180));
				//printf("\nTinggi = %d", t);
				line(frame, Point(x0, y0), Point(x, y), Scalar(255, 0, 0));
				circle(frame, Point(x, y), 3, Scalar(0, 0, 255), 1);
				
				//myallPoint.push_back(Point(x, y));

				tempM.coor = Point(x, y);
				tempM.deg = deg;
				tempM.radius = j;
				tempM.high = t;
				//tempM.height = t;

				allPoint.push_back(tempM);
				break;
			}
			else break;
		else j++;
	}
}

void takeSS()
{
	filename << "imgData/basic_" << std::setw(4) << std::setfill('0') << filecount << ".jpg";
	imwrite(filename.str().c_str(), getFrame);
	filename.str("");
	filecount++;
}

void cetakTxt()
{
	fprintf(dtCoor, "%5f\t%5f\t%5f\t%5f\t%5f\n", rT1, rT2, Xpos, Ypos, orient );
}

void takeData()
{
	filename << "../myTrySpace/data_" << std::setw(3) << std::setfill('0') << datacount << "A" << ".jpg";
	imwrite(filename.str().c_str(), frame);
	filename.str("");
	
	filename << "../myTrySpace/data_" << std::setw(3) << std::setfill('0') << datacount << "B" << ".jpg";
	imwrite(filename.str().c_str(), lapangan);
	filename.str("");

	datacount++;

	cetakTxt();
}


int main(int argc, char** argv) 
{
	if(argc != 3){
		cout << "  === FORMAT INPUT SALAH! === \n format => \"try_opencv [video file] [field image file]\"" << endl;
		cout << "   Example :\n      try_opencv ~/Documents/Data_PA/oldEdit.mp4 ~/Documents/Data_PA/desain_Lapangan_new.png" << endl;
		return -1;
	}
	const char* name = "Gambar Lapangan";
	//VideoCapture Vid("../../Data_PA/oldEdit.mp4");
	VideoCapture Vid(argv[1]);
	//dtCoor = fopen("../myTrySpace/coorLog_F.txt","w");
	//fprintf(dtCoor,"Jarak1\tJarak2\tPosisiX\tPosisiY\tOrientasi\n");

	bool play = true;
	if (!Vid.isOpened()) {
		cout << "Error opening video stream or file! -rakasiwi" << endl;
		return -1;
	}

	static bool initialized;
	while (1) {

		while (play == false)
		{
			char key = waitKey(5);
			if (key == 'p')
				play = !play;
		}
		// Capture frame-by-frame
		if (play == true)
		{
			Vid >> frame;
			Vid >> getFrame;
		}

		// If the frame is empty, break immediately
		if (frame.empty())
			break;
		lapangan = imread(argv[2]);

		cvtColor(frame, frame_HSV, COLOR_BGR2HSV);

		for (int i = 0; i < 360; i++)detDeg(i);
		marker(allPoint);

		allPoint.clear();
		// Display the resulting frame
		circle(frame, centerP, 3, Scalar(255, 155, 100), 2);
		circle(frame, centerP, 400, Scalar(10, 225, 105), 2);
		
		circle(lapangan, Point((Xpos*37.5+10),(Ypos*37.5+10)), 6, Scalar(0,0,255), 4);
		line(lapangan, Point((Xpos*37.5+10),(Ypos*37.5+10)), Point(Xpos*37.5+10+xDraw, Ypos*37.5+10-yDraw), Scalar(255,0,0), 2);
		//circle(lapangan, Point((Xpos_copy*37.5+10),(Ypos_copy*37.5+10)), 4, Scalar(255,0,0), 3);

		//video.write(frame);
		imshow("Frame", frame);
		cvNamedWindow(name);
		imshow(name, lapangan);

		//cetakTxt();

		// if((frameCount % 25) == 0)
		// {
		// 	takeSS();
		// 	takeData();
		// }
		// frameCount++;

		char key = waitKey(5);
		if (key == 'p')
			play = !play;
		else if (key == 'q')
			break;
		else if (key == 's')
		{
			takeSS();
		}
	}

	//fclose(dtCoor);
	// When everything done, release the video capture object
	Vid.release();
	// Closes all the frames
	destroyAllWindows();

	return 0;

}
//*/

