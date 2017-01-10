#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include <iostream>
using namespace cv;
using namespace std;
//int i=0;

int main(  )
{
	



        Mat srcImage = imread("137.jpg");
	imshow("original image",srcImage);

        Mat grayImage;
	cvtColor(srcImage, grayImage, CV_BGR2GRAY);
        

//------------------detect SURF points and extract descripter ----------------------


	
	
        
        int minHessian = 240;
        SurfFeatureDetector detector( minHessian );
        vector<KeyPoint> keyPoint1, keyPoint2;
        detector.detect( grayImage, keyPoint1 );

        //
        SurfDescriptorExtractor extractor;
        Mat descriptors1, descriptors2;
        extractor.compute( grayImage, keyPoint1, descriptors1 );

        //
        BruteForceMatcher< L2<float> > matcher;
	vector< DMatch > matches;
        //
        Mat  captureImage, captureImage_gray;//
	while(1)
	{
	   double time0 = static_cast<double>(getTickCount( ));//
		
	    captureImage = imread("138.jpg");
	    imshow("image captured",captureImage);
	    cvtColor(captureImage, captureImage_gray, CV_BGR2GRAY);

            detector.detect( captureImage_gray, keyPoint2 );
            //
            extractor.compute( captureImage_gray, keyPoint2, descriptors2 );
            //

            matcher.match( descriptors1, descriptors2, matches );

            Mat imgMatches;
	    drawMatches( srcImage, keyPoint1, captureImage_gray, keyPoint2, matches, imgMatches );//
            imshow("matched window", imgMatches);

	    cout << ">帧率= " << getTickFrequency() / (getTickCount() - time0) << endl;
	    cout << "matches.size()=" << matches.size()<< endl;
        cout << "keyPoint2.size()=" << keyPoint2.size()<< endl;
       // cout << "matches.distance=" << matches[1].distance<< endl;
		//.queryIdx

            //vector<int>vecMyHouse;
            cv::Point testPoint;
            int size = keyPoint1.size();  //获得keyPoint1 已有数据个数
            for(int i=0;i<size;i++)
            {
             //vecMyHouse.push_back(i);
            	//
       cout<<"keyPoint1 第"<<i<<"个 vector数据 "<<keyPoint1[i].pt.y<<"\n"<<endl;
        cout << "matches"<<i<<"  "<<matches[i].queryIdx<<"  "<<matches[i].trainIdx<<"  "<<matches[i].distance<< endl;    
            } 
           // cout<<"第 %d个 vector数据 ,&i"<<keyPoint1[i].pt.y<<endl;
            //testPoint=keyPoint1[i].pt;
           // cout<<testPoint.x<<endl;
		while(1)
		{
		if(char(waitKey(1)) == 27) break;
		}
           
		
	}
            return 0;
}

/////////////////////////////////////////////////////////////////////////////////
/*
	Mat grayImage;
	cvtColor(srcImage, grayImage, CV_BGR2GRAY);
	double time0 = static_cast<double>(getTickCount( ));//
	Mat srcImage1 = imread("51.jpg",1);
	Mat srcImage2 = imread("52.jpg",1);
	if( !srcImage1.data || !srcImage2.data )
	{ printf("¶ÁÈ¡ÍŒÆ¬ŽíÎó£¬ÇëÈ·¶šÄ¿ÂŒÏÂÊÇ·ñÓÐimreadº¯ÊýÖž¶šµÄÍŒÆ¬ŽæÔÚ~£¡ \n"); return false; }  

	//¡Ÿ2¡¿Ê¹ÓÃSURFËã×ÓŒì²â¹ØŒüµã
	int minHessian = 400;//SURFËã·šÖÐµÄhessianãÐÖµ
	SurfFeatureDetector detector( minHessian );//¶šÒåÒ»žöSurfFeatureDetector£šSURF£© ÌØÕ÷Œì²âÀà¶ÔÏó  
	std::vector<KeyPoint> keyPoint1, keyPoints2;//vectorÄ£°åÀà£¬Žæ·ÅÈÎÒâÀàÐÍµÄ¶¯Ì¬Êý×é

	//¡Ÿ3¡¿µ÷ÓÃdetectº¯ÊýŒì²â³öSURFÌØÕ÷¹ØŒüµã£¬±£ŽæÔÚvectorÈÝÆ÷ÖÐ
	detector.detect( srcImage1, keyPoint1 );
	detector.detect( srcImage2, keyPoints2 );

	//¡Ÿ4¡¿ŒÆËãÃèÊö·û£šÌØÕ÷ÏòÁ¿£©
	SurfDescriptorExtractor extractor;
	Mat descriptors1, descriptors2;
	extractor.compute( srcImage1, keyPoint1, descriptors1 );
	extractor.compute( srcImage2, keyPoints2, descriptors2 );

	//¡Ÿ5¡¿Ê¹ÓÃBruteForceœøÐÐÆ¥Åä
	// ÊµÀý»¯Ò»žöÆ¥ÅäÆ÷
	BruteForceMatcher< L2<float> > matcher;
	std::vector< DMatch > matches;
	//Æ¥ÅäÁœ·ùÍŒÖÐµÄÃèÊö×Ó£šdescriptors£©


	matcher.match( descriptors1, descriptors2, matches );

	//¡Ÿ6¡¿»æÖÆŽÓÁœžöÍŒÏñÖÐÆ¥Åä³öµÄ¹ØŒüµã
	Mat imgMatches;
	drawMatches( srcImage1, keyPoint1, srcImage2, keyPoints2, matches, imgMatches );//œøÐÐ»æÖÆ

	//¡Ÿ7¡¿ÏÔÊŸÐ§¹ûÍŒ
	imshow("Æ¥ÅäÍŒ", imgMatches );
	cout << ">Ö¡ÂÊ= " << getTickFrequency() / (getTickCount() - time0) << endl;

	waitKey(0);
	return 0;
}

*/
