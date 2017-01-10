//#include "stdafx.h"
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include "opencv2/core/core.hpp"
//#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/legacy/legacy.hpp>
#include "ekf.hpp"
#include "_Matrix.h"

using namespace cv;
using namespace std;


FILE *fpTarget = NULL;
FILE *fpRecord = NULL;
FILE *fpEkfReord = NULL;
/*
double x0;
double y0;
double z0;
double t0;

double x1;
double y1;
*/
double ekf_init[2][4];
double t_prev;


/*       
_Matrix A(6,6);  
_Matrix R(4,4);  
_Matrix Q(6,6);
_Matrix P(6,6);  
_Matrix M(6,1);    
    
    //初始化内存  
A.init_matrix();  
R.init_matrix();  
Q.init_matrix();  
P.init_matrix();  
M.init_matrix(); 
ekf.delt_T = 0.1;
*/
long TotalLines = 0;
int dataNum = 0;

int WriteTest(char *filename)
{
	 ofstream  file;
	 file.open (filename); //
   
	for(int i=0;i<5;i++)
	{
    file << i<<"\n";
	}
    cout << "successfully write data to " <<filename<< endl;
    file.close ();
     
	return 0;
}

int ReadTest(void)
{   
	int tImage = 0;
	int ul,vl,ur,vr;

	if(fpTarget == NULL) return -1;
	if (feof(fpTarget)) 
	   fseek(fpTarget, 0L, SEEK_SET);
	int RDNUM = fscanf(fpTarget, "%d\t%d\t%d\t%d\t%d\n",&tImage,&ul,&vl,&ur,&vr);
	return RDNUM;
}

int DataPara::ReadDataPara(void)
{   
	char sysTime[]="12345678123456789 ";
	char timeStr[15];
	if(fpTarget == NULL) return -1;
	if (feof(fpTarget))   //see if the pointer is pointed at the end of the file
	   fseek(fpTarget, 0L, SEEK_SET);  //if the pointer is at the end of the file,set pointer to the start of the file
	int RDNUM = fscanf(fpTarget, "%s\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%d\t%d\t%d\t%d\n",&sysTime,&data1.xGPS,&data1.yGPS,&data1.zGPS,
		&data1.yawLeft,&data1.pitchLeft,&data1.yawRight,&data1.pitchRight,&data1.ul,&data1.vl,&data1.ur,&data1.vr);

    //transform time from char to int 
    sprintf(timeStr,"%c%c%c%c%c%c%c%c%c",sysTime[8],sysTime[9],sysTime[10],sysTime[11],sysTime[12],sysTime[13],sysTime[14],sysTime[15],sysTime[16]);
	data1.sysT = atoi(timeStr);//string to num
	//printf("%s\n",timeStr);
	//printf("time is %d\n",data1.sysT);

	return RDNUM;
}
int DataPara::WriteDataPara(void)
{
	

	return 0;
}

void TriAngle::LineSolve(int i)
{

	double a_l,b_l,c_l,a_r,b_r,c_r;
	double k1,k2,k3,k4,k5,k6;
	double X_l,Y_l,Z_l,X_r,Y_r,Z_r;//interscation points


    //calcute the angle between uav goal optical axis
    triangle1.Pan_2_l=atan((data1.ul - 320.0)*tan(0.5*FieldView_Pan_l)/320.0);
    triangle1.Title_2_l=atan((data1.vl - 240.0)*tan(0.5*FieldView_Tile_l)/240.0);
    triangle1.Pan_2_r=atan((data1.ur - 320.0)*tan(0.5*FieldView_Pan_r)/320.0);
    triangle1.Title_2_r=atan((data1.vr - 240.0)*tan(0.5*FieldView_Tile_r)/240.0);

    //yaw(with runway Y axis) and pitch(with horizontal) angle  at present
    yaw_l   = (data1.yawLeft + delt_Pan_l)*PI/180.0 + triangle1.Pan_2_l;
    pitch_l = (data1.pitchLeft + delt_Tile_l)*PI/180.0 + triangle1.Title_2_l;
    yaw_r   = (data1.yawRight + delt_Pan_r)*PI/180.0 + triangle1.Pan_2_r;
    pitch_r = (data1.pitchRight + delt_Tile_r)*PI/180.0 + triangle1.Title_2_r;
    
    //parameter of line to be solved
    a_l=cos(pitch_l)*sin(yaw_l);
    b_l=cos(pitch_l)*cos(yaw_l);
    c_l=sin(pitch_l);
         
    a_r=cos(pitch_r)*sin(yaw_r);
    b_r=cos(pitch_r)*cos(yaw_r);
    c_r=sin(pitch_r);


    k1=a_l*a_l + b_l*b_l + c_l*c_l;
    k2=a_r*a_r + b_r*b_r + c_r*c_r;
    k3=(a_l*b_r-b_l*a_r)*(a_l*b_r-b_l*a_r) + (b_l*c_r-c_l*b_r)*(b_l*c_r-c_l*b_r) + (a_l*c_r-c_l*a_r)*(a_l*c_r-c_l*a_r);
    k4=a_l*a_r + b_l*b_r + c_l*c_r;
    k5=(a_l*k2 - a_r*k4)/k3;
    k6=(a_l*k4 - a_r*k1)/k3;
            
    X_l=D*a_l*k5;
    Y_l=D*b_l*k5;
    Z_l=D*c_l*k5;
    X_r=D*(a_r*k6+1.0);
    Y_r=D*b_r*k6;
    Z_r=D*c_r*k6;
     
    triangle1.x = w*X_l+(1.0-w)*X_r;
    triangle1.y = w*Y_l+(1.0-w)*Y_r;
    triangle1.z=w*Z_l+(1.0-w)*Z_r + 33.2128595;  
    

    fprintf(fpRecord,"%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
    	data1.sysT,data1.xGPS,data1.yGPS,data1.zGPS,triangle1.x,triangle1.y,triangle1.z,triangle1.Pan_2_r,data1.yawRight ,yaw_r,delt_Pan_r);
    if(i<2)
    {
    	ekf_init[i][0] = triangle1.x;
    	ekf_init[i][1] = triangle1.y;
    	ekf_init[i][2] = triangle1.z;
    	ekf_init[i][3] = data1.sysT;
    }
 
}  



long countLines(void)
{
	if (fpTarget == NULL) return 0L;
	char buf[1024];
	long linenum = 0;
	while (fgets(buf, 1024, fpTarget) != NULL) linenum++;
	fseek(fpTarget,0L,SEEK_SET);//SEEK_SET 从距文件开头offset(0L) 位移量为新的读写位置
	return linenum;

}

/*
void EkfMatrixInit(void)
{
	//A.  
    //R.  
    //Q.  
    //P. 
    //M.
    for(int i=0;i<6;i++)
    {
    	if(i<3)
    	{
    		A.write(i,i+3,ekf.delt_T);
    		Q.write(i,i,1);
    	}
    	else
    	{
         Q.write(i,i,0.001);
    	}
    	A.write(i,i,1);
    	P.write(i,i,0.01);
    	if(i<4)
    	{
    		R.write(i,i,1);
    	}

    }
    

}

*/
int main()
{  
    double matlabdata=1.0;
    char  TriangleSolveData[] ="TriangleSolveData.txt";
	char  CompleteData[] ="newdata_1_position3.txt";  //GPS PTU  imagedata
	//char  ImagePosition[] = "newdata1.txt"; 
	char  EkfRecord[]= "EkfRecord.txt";     //
	
    class _Matrix_Calc m_c;  
/**************  AT ONE FUNCTION?*************************/
    _Matrix A(6,6);  //
    _Matrix R(4,4);  //4
    _Matrix Q(6,6);  //
    _Matrix P(6,6);  //
    _Matrix predict_P(6,6);   // 
    _Matrix M(6,1); ////
    _Matrix predict_M(6,1); ////
    _Matrix h_predict_M(6,1);   ////   
    _Matrix H(4,6);   /////
    _Matrix h_gt(4,1); 
    _Matrix V(4,4);   //4
    _Matrix W(6,6);  //
    _Matrix S(4,4);  //4
    _Matrix K(6,4);   ///

    _Matrix temp44_1(4,4);
    _Matrix temp44_2(4,4);
    _Matrix temp44_3(4,4);
    _Matrix temp44_4(4,4);
    _Matrix temp66_1(6,6);
    _Matrix temp66_2(6,6);
    _Matrix temp66_3(6,6);
    _Matrix temp66_4(6,6);
    _Matrix temp46_1(4,6);
    _Matrix temp64_1(6,4);
    _Matrix temp41_1(4,1);
    _Matrix temp61_1(6,1);
    
    //初始化内存  
    A.init_matrix();  
    R.init_matrix();  
    Q.init_matrix();  
    P.init_matrix();  
    predict_P.init_matrix(); 
    M.init_matrix(); 
    predict_M.init_matrix();
    h_predict_M.init_matrix();
    H.init_matrix(); 
    h_gt.init_matrix(); 
    V.init_matrix(); 
    W.init_matrix(); 
    S.init_matrix(); 
    K.init_matrix(); 
    temp44_1.init_matrix(); 
    temp44_2.init_matrix(); 
    temp44_3.init_matrix();
    temp44_4.init_matrix();
    temp66_1.init_matrix();
    temp66_2.init_matrix();
    temp66_3.init_matrix();
    temp66_4.init_matrix();
    temp46_1.init_matrix();
    temp41_1.init_matrix();
    temp61_1.init_matrix();
    temp64_1.init_matrix();

    ekf.delt_T = 100;

    for(int i=0;i<6;i++)  //6*6  initial 0
    {
    	for(int j=0;j<6;j++) //6*6
    	{
          A.write(i,j,0.0);
          Q.write(i,j,0.0);
          P.write(i,j,0.0);
          predict_P.write(i,j,0.0);
          W.write(i,j,0.0);


          if(j<4) //6*4
          {
          	K.write(i,j,0.0);
          }

          if(j<1) //6*1
          {
          	h_predict_M.write(i,j,0.0);
          	predict_M.write(i,j,0.0);
          	M.write(i,j,0.0);
          }

          if(i<4)  //4*6
          {
            H.write(i,j,0.0);
          if(j<4)
          {
            R.write(i,j,0.0);
            V.write(i,j,0.0);
            S.write(i,j,0.0);
          }   

          }

    	}

    }

//intial value
        for(int i=0;i<6;i++)
    {
    	if(i<3)
    	{
    		A.write(i,i+3,104);
    		Q.write(i,i,1);
    	}
    	else
    	{
         Q.write(i,i,0.001);
    	}
    	A.write(i,i,1);
    	P.write(i,i,0.01);
    	W.write(i,i,1);
    	if(i<4)
    	{
    		R.write(i,i,1);
    		V.write(i,i,1);
    	}

    }
    printf("A= \n");
    printff_matrix(&A);
    printf("R= \n");
    printff_matrix(&R);
    printf("Q= \n");
    printff_matrix(&Q);
    printf("V= \n");
    printff_matrix(&V);
    printf("P= \n");
    printff_matrix(&P);
    printf("W= \n");
    printff_matrix(&W);

    


 /**************  AT ONE FUNCTION?*************************/
	//WriteTest(WriteFlie);
	//////////////////////////////////////////////
	
	fpTarget = fopen(CompleteData,"r");
	if(fpTarget == NULL)
	{
		cout<<"failed to open file newdata1.txt"<<endl;
		waitKey(1000);
	}
	fpRecord = fopen(TriangleSolveData,"w");
	if(fpRecord == NULL)
	{
     cout<<"failed to create file TriangleSolveData.txt"<<endl;
	 waitKey(1000);
	}

    TotalLines = countLines();
    cout<<TotalLines<<"\n"<<endl;
    waitKey(1000);
	for(long i=0;i<TotalLines;i++)
	{
		
		dataNum = data1.ReadDataPara();  ////read data needed

		triangle1.LineSolve(i);
		//cout<<"there are "<<dataNum<<" data per line"<<endl;
		waitKey(10);//Psuppose that the time delay is 100ms

	}

   	fclose(fpTarget);
	fclose(fpRecord);
    fpTarget = NULL;
    fpEkfReord = NULL;
	//////////////////////////////////////////////

    fpTarget = fopen(CompleteData,"r");
	if(fpTarget == NULL)
	{
		cout<<"failed to open file newdata1.txt"<<endl;
		waitKey(1000);
	}


    fpEkfReord=fopen(EkfRecord,"w");
    if(fpEkfReord == NULL)
	{
     cout<<"failed to create file EkfRecord.txt"<<endl;
	 waitKey(1000);
	}
    
   
    ////inital state from secnd frame
    ekf.x=ekf_init[1][0];
    ekf.y=ekf_init[1][1];
    ekf.z=ekf_init[1][2];
    ekf.delt_T=ekf_init[1][3]-ekf_init[0][3];
    ekf.vx=(ekf_init[1][0]-ekf_init[0][0])/ekf.delt_T;
    ekf.vy=(ekf_init[1][1]-ekf_init[0][1])/ekf.delt_T;
    ekf.vz=(ekf_init[1][2]-ekf_init[0][2])/ekf.delt_T;


//////////// Set Data as in Matlab/////////////////////
    ekf.x=-10.0473;
    ekf.y=451.4212;
    ekf.z=49.8086;
    ekf.delt_T=ekf_init[1][3]-ekf_init[0][3];
    ekf.vx=0.00025718;
    ekf.vy=-0.0225;
    ekf.vz=-0.0011;
//////////// Set Data as in Matlab/////////////////////

    printf("ekf.x = %f\n",ekf.x);
    printf("ekf.y = %f\n",ekf.y);
    printf("ekf.z = %f\n",ekf.z);
    printf("ekf.vx = %f\n",ekf.vx);
    printf("ekf.vy = %f\n",ekf.vy);
    printf("ekf.vz = %f\n",ekf.vz);
    printf("ekf.delt_T = %f\n",ekf.delt_T);
    
   

     for (int i=0;i<2;i++)
    {
     dataNum = data1.ReadDataPara();  ////read data needed for ekf
    }
   
//&data1.yawLeft,&data1.pitchLeft,&data1.yawRight,&data1.pitchRight,&data1.ul,&data1.vl,&data1.ur,&data1.vr);

    ekf.ul = data1.ul;
    ekf.ur = data1.ur;
    ekf.vl = data1.vl;
    ekf.vr = data1.vr;

    M.write(0,0,ekf.x);
    M.write(1,0,ekf.y);
    M.write(2,0,ekf.z);
    M.write(3,0,ekf.vx);
    M.write(4,0,ekf.vy);
    M.write(5,0,ekf.vz);
    
    //predict_M = M
    for(int i=0;i<6;i++)  
    {
    	predict_M.write(i,0,M.read(i,0));
    }

    printf("predict_M= \n");
    printff_matrix(&predict_M);
     

    
    //predict_P = A*P*A' + W*Q*W'
    m_c.multiply(&A,&P,&temp66_1);  //A*P
    m_c.transpos(&A,&temp66_2); //A'
    m_c.multiply(&temp66_1,&temp66_2,&temp66_3); //A*P*A'  temp66_3
    m_c.multiply(&W,&Q,&temp66_1);  //W*Q temp66_1
    m_c.transpos(&W,&temp66_2); //W' temp66_2
    m_c.multiply(&temp66_1,&temp66_2,&temp66_4);//W*Q*W'  temp66_4
    m_c.add(&temp66_3,&temp66_4,&predict_P);
    
    printf("predict_P= \n");
    printff_matrix(&predict_P);
     
    //////////// Set Data as in Matlab/////////////////////   
    data1.yawLeft = -0.0022;
    data1.pitchLeft = 0.0561;
    data1.yawRight = -0.0671;
    data1.pitchRight = 0.0252;


    
    calculateHh(data1.yawLeft,data1.pitchLeft,data1.yawRight,data1.pitchRight,
    	       ekf.x,ekf.y,ekf.z,&ekf.ul,&ekf.vl,&ekf.ur,&ekf.vr,&H);
    printf("data1.yawLeft = %f\n",data1.yawLeft);
    printf("data1.pitchLeft = %f\n",data1.pitchLeft);
    printf("data1.yawRight = %f\n",data1.yawRight);
    printf("data1.pitchRight = %f\n",data1.pitchRight);

    h_predict_M.write(0,0,ekf.ul);
    h_predict_M.write(1,0,ekf.vl);
    h_predict_M.write(2,0,ekf.ur);
    h_predict_M.write(3,0,ekf.vr);
    
    printf("h_predict_M= \n");
    printff_matrix(&h_predict_M);
   
    printf("H= \n");
    printff_matrix(&H);
    matlabdata=2767291362382745/144115188075855872;
    printf("matlab data = %lf \n",&matlabdata);
    return 0;


   	for(long i=2;i<TotalLines;i++)
	{
		t_prev = data1.sysT;
		dataNum = data1.ReadDataPara();     ////read data needed
        ekf.delt_T = data1.sysT - t_prev;
        
        h_gt.write(0,0,data1.ul);
        h_gt.write(1,0,data1.vl);
        h_gt.write(2,0,data1.ur);
        h_gt.write(3,0,data1.vr);


        //S = (V*R*V' + H*predict_P*H')
        m_c.multiply(&V,&R,&temp44_1);  //V*R
        m_c.transpos(&V,&temp44_2); //V'
        m_c.multiply(&temp44_1,&temp44_2,&temp44_3); //V*R*V'  temp44_3
        m_c.multiply(&H,&predict_P,&temp44_1);  //H*predict_P  temp44_1
        m_c.transpos(&H,&temp44_2); //H' temp44_2
        m_c.multiply(&temp44_1,&temp44_2,&temp44_4);//H*predict_P*H'  temp44_4
        m_c.add(&temp44_3,&temp44_4,&S);
     

        //K= predict_P*H'/S
        m_c.inverse(&S,&temp44_3);  // 1/S temp44_3
        m_c.multiply(&predict_P,&temp44_2,&temp44_1);  //predict_P*H' temp44_1
        m_c.multiply(&temp44_1,&temp44_3,&K);  //predict_P*H'/S    K


        //M = predict_M + K*(h_gt - h_predict_M);
        m_c.subtract(&h_gt,&h_predict_M,&temp41_1);    //h_gt - h_predict_M    temp41_1
        m_c.multiply(&K,&temp41_1,&temp61_1);   //K*(h_gt - predict_M)     temp61_1
        m_c.add(&predict_M,&temp61_1,&M); //M 

        //P =  predict_P -K*S*K'
        m_c.multiply(&K,&S,&temp64_1);  //K*S
        m_c.transpos(&K,&temp46_1); //K'
        m_c.multiply(&temp64_1,&temp46_1,&temp66_1);  //K*S*K'
        m_c.subtract(&predict_P,&temp66_1,&P); 

        //predict_M = A*M
        m_c.multiply(&A,&M,&predict_M); 

        //predict_P = A*P*A' + W*Q*W'
        m_c.multiply(&A,&P,&temp66_1);  //A*P
        m_c.transpos(&A,&temp66_2); //A'
        m_c.multiply(&temp66_1,&temp66_2,&temp66_3); //A*P*A'  temp66_3
        m_c.multiply(&W,&Q,&temp66_1);  //W*Q temp66_1
        m_c.transpos(&W,&temp66_2); //W' temp66_2
        m_c.multiply(&temp66_1,&temp66_2,&temp66_4);//W*Q*W'  temp44_4
        m_c.add(&temp66_3,&temp66_4,&predict_P);

        //




        calculateHh(data1.yawLeft,data1.pitchLeft,data1.yawRight,data1.pitchRight,
    	       ekf.x,ekf.y,ekf.z,&ekf.ul,&ekf.vl,&ekf.ur,&ekf.vr,&H);

        h_predict_M.write(0,0,ekf.ul);
        h_predict_M.write(1,0,ekf.vl);
        h_predict_M.write(2,0,ekf.ur);
        h_predict_M.write(3,0,ekf.vr);

       fprintf(fpEkfReord,"%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
    	data1.sysT,data1.xGPS,data1.yGPS,data1.zGPS,ekf.x,ekf.y,ekf.z,ekf.ul,ekf.vl,ekf.ur,ekf.vr);
		waitKey(10); //Psuppose that the time delay is 100ms
        }
        

	



	fclose(fpTarget);
	//fclose(fpRecord);
    fclose(fpEkfReord);

	return 0;
}