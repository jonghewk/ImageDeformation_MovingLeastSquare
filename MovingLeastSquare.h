/*  Based on the papaer
 *
 *	"Image Deformation Using Moving Least Square "
 *  by Scheafer, McPhail, Warren
 *
 *  Implementation refencing the work of Gabriele Lombardi wriiten in MATLAB 
 *  This header file includes functions to do affine deformation
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <opencv/highgui.h>
#include <opencv/cv.h>

using namespace cv;
using namespace std;

typedef struct _typeA
{
	Mat a;
	Mat b;
	Mat c;
	Mat d;

}typeA;

typedef struct _typeRigid
{
	vector <_typeA> A;
	Mat normof_v_Pstar;

}typeRigid;


/*Function to Compute the Weights*/
Mat precomputeWeights(Mat p , Mat v, double a)
{
	Mat w = Mat::zeros(p.cols, v.cols, CV_32F);
	Mat p_resize;
	Mat norms = Mat::zeros(2, v.cols, CV_32F);
	Mat norms_a;
	Mat p_v;

	//Iterate through the control points
	for(int i= 0 ; i<p.cols; i++)
	{
		//compute the norms
		p_resize = repeat(p.col(i),1, v.cols );
		p_v = p_resize -v;		
		pow(p_v,2,p_v);
		norms = p_v.row(0) + p_v.row(1);
		pow(norms,a,norms_a);
		//compute the weights
		w.row(i) = 1.0 / norms_a;
	}
	return w;
}

/*Function to Precompute Weighted Centroids*/
Mat precomputeWCentroids(Mat p, Mat w)
{
	Mat Pstar;
	Mat mult;
	Mat resize;
	Mat sum = Mat::zeros(1, w.cols, CV_32F);
	mult= p*w;

	for(int i = 0; i< w.rows; i++)
		sum += w.row(i);
	resize = repeat(sum, p.rows, 1);

	Pstar = mult / resize;
	return Pstar;
}
/*Function to Precompute Affine Deformation*/
Mat precomputeAffine(Mat p, Mat v, Mat w)
{
	//Precompute Weighted Centroids
	Mat Pstar = precomputeWCentroids(p,w);
	Mat M1;
	Mat a=Mat::zeros(1,Pstar.cols,CV_32F);
	Mat b=Mat::zeros(1,Pstar.cols,CV_32F); 
	Mat d=Mat::zeros(1,Pstar.cols,CV_32F);
	Mat row1, row2, power_b;
	Mat det;
	//Precompute the first matrix
	M1 = v- Pstar;
	vector <Mat> Phat;
	//Iterate through control points
	for(int i = 0; i< p.cols; i++)
	{
		Mat t = repeat(p.col(i) ,1, Pstar.cols ) - Pstar;

		pow(t.row(0), 2, row1);
		pow(t.row(1), 2, row2);
		//compute matrix elements
		a = a + (w.row(i)).mul(row1);
		b = b + ((w.row(i)).mul( t.row(0))).mul(t.row(1));
		d = d + row2.mul( w.row(i));
		//save the hat points
		Phat.push_back(t);
	}

	pow(b,2,power_b);
	//compute determinant
	det = a.mul(d) - power_b;
	//compute the inverse
	Mat Ia = d/det;
	Mat Ib = -1*b / det;
	Mat Id = a / det;

	Mat Iab, Ibd;
	vconcat(Ia,Ib, Iab);
	vconcat(Ib,Id, Ibd);

	Mat m  = M1.mul(Iab);
	Mat sum1 = Mat::zeros(1, m.cols, CV_32F);
	for(int i =0; i< m.rows ; i++)
		sum1+=m.row(i);

	Mat n  = M1.mul(Ibd);
	Mat sum2 = Mat::zeros(1, n.cols, CV_32F);
	for(int i= 0; i< n.rows; i++)
		sum2+=n.row(i);

	//compute the first product element
	Mat F1;
	vconcat(sum1, sum2,F1);

	Mat A = Mat::zeros(p.cols, Pstar.cols, CV_32F);
	Mat F1_mul;


	for(int j =0 ; j< p.cols; j++)
	{
		F1_mul = F1.mul(Phat.at(j));
		Mat temp = Mat::zeros(1, F1_mul.cols, CV_32F);
		for(int i =0; i< F1_mul.rows; i++)
			temp+= F1_mul.row(i);
		A.row(j) = temp.mul(w.row(j));
	}
	Mat data = A;


	return data;
}

//precompute Asimilar
vector <_typeA> precomputeA(Mat Pstar, vector <Mat> Phat, Mat v, Mat w)
{
	vector <_typeA> A;

	//fixed part
	Mat R1 = v - Pstar;
	Mat R2;
	vconcat(R1.row(1),-R1.row(0),R2);

	for(int i = 0 ; i< Phat.size() ; i++)
	{
		//precompute
		typeA temp;
		Mat L1 = Phat.at(i);
		Mat L2;
		vconcat(L1.row(1),(L1.row(0)).mul(-1),L2);
		
		Mat L1R1 = L1.mul(R1);
		Mat sumL1R1 = Mat::zeros(1,L1R1.cols, CV_32F);

		Mat L1R2 = L1.mul(R2);
		Mat sumL1R2 = Mat::zeros(1, L1R2.cols, CV_32F);

		Mat L2R1 = L2.mul(R1);
		Mat sumL2R1 = Mat::zeros(1, L2R1.cols,CV_32F);

		Mat L2R2 = L2.mul(R2);
		Mat sumL2R2 = Mat::zeros(1, L2R2.cols, CV_32F);

		for(int j = 0; j<L1R1.rows; j++)
			sumL1R1 += L1R1.row(j);

		for(int j = 0; j<L1R2.rows; j++)
			sumL1R2 += L1R2.row(j);

		for(int j = 0; j<L2R1.rows; j++)
			sumL2R1 += L2R1.row(j);

		for(int j = 0; j<L2R2.rows; j++)
			sumL2R2 += L2R2.row(j);

		temp.a = (w.row(i)).mul(sumL1R1);
		temp.b = (w.row(i)).mul(sumL1R2);
		temp.c = (w.row(i)).mul(sumL2R1);
		temp.d = (w.row(i)).mul(sumL2R2);

		A.push_back(temp);
	}

	return A;
}

vector <_typeA> precomputeSimilar(Mat p, Mat v, Mat w)
{
	Mat Pstar = precomputeWCentroids(p, w);
	vector <Mat> Phat;
	Mat mu = Mat::zeros(1, Pstar.cols, CV_32F);
	Mat t1; 
	Mat product;
	for(int i =0 ; i<p.cols; i++)
	{
		Mat t = repeat(p.col(i) ,1, Pstar.cols ) - Pstar;
		Mat sum = Mat::zeros(1, t.cols, CV_32F);
		pow(t, 2, t1);
		for(int j=0; j<t1.rows; j++)
			sum+=t1.row(j);

		mu = mu+ (w.row(i)).mul(sum);
		Phat.push_back(t);
	}

	vector <_typeA> A = precomputeA(Pstar, Phat, v, w);

	for(int i=0; i< A.size(); i++)
	{
		(A.at(i)).a =  ((A.at(i)).a).mul(1/mu); 
		(A.at(i)).b =  ((A.at(i)).b).mul(1/mu); 
		(A.at(i)).c =  ((A.at(i)).c).mul(1/mu); 
		(A.at(i)).d =  ((A.at(i)).d).mul(1/mu); 
	}
	return A;
}

_typeRigid precomputeRigid(Mat p, Mat v, Mat w)
{
	typeRigid data;
	Mat Pstar = precomputeWCentroids(p, w);
	vector <Mat> Phat;
	for(int i =0 ; i<p.cols; i++)
	{
		Mat t = repeat(p.col(i) ,1, Pstar.cols ) - Pstar;
		Phat.push_back(t);
	}

	vector <_typeA> A = precomputeA(Pstar, Phat, v, w);
	Mat v_Pstar = v-Pstar;
	Mat vpower;
	pow(v_Pstar, 2, vpower);
	Mat sum = Mat::zeros(1,vpower.cols,CV_32F);
	for(int i =0; i<vpower.rows; i++)
		sum += vpower.row(i);
	
	sqrt(sum, data.normof_v_Pstar);
	data.A = A;
	return data;
}

Mat PointsTransformRigid(Mat w, _typeRigid mlsd, Mat q)
{
	Mat Qstar = precomputeWCentroids(q, w);
	Mat Qhat;
	Mat fv2 = Mat::zeros(Qstar.rows, Qstar.cols, CV_32F);
	Mat prod1, prod2;
	Mat con1, con2;
	Mat update;
	Mat repmat;
	Mat npower;
	Mat normof_fv2;
	Mat fv = Mat::zeros(Qstar.rows, Qstar.cols, CV_32F);
	for( int i = 0 ; i< q.cols; i++)
	{
		Qhat = repeat(q.col(i) ,1, Qstar.cols ) - Qstar;
		
		vconcat((mlsd.A.at(i)).a, (mlsd.A.at(i)).c, con1);
		prod1 = Qhat.mul(con1);
		Mat sum1 = Mat::zeros(1, prod1.cols, CV_32F);
		for(int j = 0 ; j<prod1.rows; j++)
			sum1+=prod1.row(j);

		vconcat((mlsd.A.at(i)).b, (mlsd.A.at(i)).d, con2);
		prod2 = Qhat.mul(con2);
		Mat sum2 = Mat::zeros(1, prod2.cols, CV_32F);
		for(int j = 0 ; j<prod2.rows; j++)
			sum2+=prod2.row(j);
		
		vconcat(sum1, sum2, update);
		fv2 = fv2 + update;
	}
	npower = fv2.mul(fv2);

	Mat sumfv2 = Mat::zeros(1,npower.cols, CV_32F);
	for(int i =0; i<npower.rows; i++)
		sumfv2 += npower.row(i);
	
	
	sqrt(sumfv2,normof_fv2);

	Mat norm_fact = (mlsd.normof_v_Pstar).mul(1/normof_fv2);

	repmat = repeat(norm_fact, fv2.rows, 1);
	fv = fv2.mul(repmat) + Qstar;

	return fv;
}

Mat PointsTransformSimilar(Mat w, vector <_typeA> A, Mat q)
{
	Mat Qstar = precomputeWCentroids(q, w);

	Mat fv = Qstar.clone();
	Mat Qhat;
	Mat resize; 
	
	Mat prod1, prod2;
	Mat con1, con2;

	Mat update;

	for( int i = 0 ; i< q.cols; i++)
	{
		Qhat = repeat(q.col(i) ,1, Qstar.cols ) - Qstar;
		vconcat((A.at(i)).a, (A.at(i)).c, con1);
		prod1 = Qhat.mul(con1);
		Mat sum1 = Mat::zeros(1, prod1.cols, CV_32F);
		for(int j = 0 ; j<prod1.rows; j++)
			sum1+=prod1.row(j);
		
		vconcat((A.at(i)).b, (A.at(i)).d, con2);
		prod2 = Qhat.mul(con2);
		Mat sum2 = Mat::zeros(1, prod2.cols, CV_32F);
		for(int j = 0 ; j<prod2.rows; j++)
			sum2+=prod2.row(j);
		
		vconcat(sum1, sum2, update);
		fv = fv + update;
	}
	return fv;
}



Mat PointsTransformAffine(Mat w, Mat A, Mat q)
{
	//compute weighted centroids for q
	Mat Qstar = precomputeWCentroids(q, w);

	Mat fv = Qstar.clone();
	Mat Qhat;
	Mat resize;
	//add the affine parts
	for(int j = 0 ; j< q.cols ; j++)
	{
		//compute hat points
		Qhat = repeat(q.col(j) ,1, Qstar.cols ) - Qstar;
		resize =  repeat(A.row(j), Qhat.rows, 1);
		//update
		fv = fv + Qhat.mul(resize);
	}
	return fv;
}