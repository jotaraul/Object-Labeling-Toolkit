/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "CDifodo_multi.h"
#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/round.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace std;
using namespace Eigen;
using mrpt::utils::round;
using mrpt::utils::square;

CDifodo::CDifodo()
{
	rows = 60;
	cols = 80;
	fovh = M_PI*58.6/180.0;
	fovv = M_PI*45.6/180.0;
	cam_mode = 1;			// (1 - 640 x 480, 2 - 320 x 240, 4 - 160 x 120)
	downsample = 1;
	ctf_levels = 1;
	width = 640/(cam_mode*downsample);
	height = 480/(cam_mode*downsample);
	fast_pyramid = true;

	//Resize pyramid
    const unsigned int pyr_levels = round(log(float(width/cols))/log(2.f)) + ctf_levels;

	for (unsigned int c=0; c<NC; c++)
	{
		for (unsigned int i = 0; i<pyr_levels; i++)
		{
			unsigned int s = pow(2.f,int(i));
			cols_i = width/s; rows_i = height/s;
			depth[c][i].resize(rows_i, cols_i);
			depth_inter[c][i].resize(rows_i, cols_i);
			depth_old[c][i].resize(rows_i, cols_i);
			depth[c][i].assign(0.0f);
			depth_old[c][i].assign(0.0f);
			xx[c][i].resize(rows_i, cols_i);
			xx_inter[c][i].resize(rows_i, cols_i);
			xx_old[c][i].resize(rows_i, cols_i);
			xx[c][i].assign(0.0f);
			xx_old[c][i].assign(0.0f);
			yy[c][i].resize(rows_i, cols_i);
			yy_inter[c][i].resize(rows_i, cols_i);
			yy_old[c][i].resize(rows_i, cols_i);
			yy[c][i].assign(0.0f);
			yy_old[c][i].assign(0.0f);
			zz_global[c][i].resize(rows_i, cols_i);
			xx_global[c][i].resize(rows_i, cols_i);
			yy_global[c][i].resize(rows_i, cols_i);

			if (cols_i <= cols)
			{
				depth_warped[c][i].resize(rows_i,cols_i);
				xx_warped[c][i].resize(rows_i,cols_i);
				yy_warped[c][i].resize(rows_i,cols_i);
			}
		}

		depth_wf[c].setSize(height,width);
	}

	//Resize the transformation matrices
	for (unsigned int l = 0; l<pyr_levels; l++)
		global_trans[l].resize(4,4);

	for (unsigned int c=0; c<NC; c++)	
		for (unsigned int l = 0; l<pyr_levels; l++)
			transformations[c][l].resize(4,4);

	//Resize the calibration matrices
	for (unsigned int c=0; c<NC; c++)
		calib_mat[c].resize(4,4);

	//Initialize some variables
	previous_speed_const_weight = 0.05f;
	previous_speed_eig_weight = 0.5f;
	kai_loc_old.assign(0.f);
	num_valid_points = 0;

	//Compute gaussian mask
	VectorXf v_mask(4);
	v_mask(0) = 1.f; v_mask(1) = 2.f; v_mask(2) = 2.f; v_mask(3) = 1.f;
	for (unsigned int i = 0; i<4; i++)
		for (unsigned int j = 0; j<4; j++)
			f_mask(i,j) = v_mask(i)*v_mask(j)/36.f;

	//Compute gaussian mask
	float v_mask2[5] = {1,4,6,4,1};
	for (unsigned int i = 0; i<5; i++)
		for (unsigned int j = 0; j<5; j++)
			g_mask[i][j] = v_mask2[i]*v_mask2[j]/256.f;
}

void CDifodo::buildCoordinatesPyramid()
{
	const float max_depth_dif = 0.1f;

	//Push coordinates back
	for (unsigned int c=0; c<NC; c++)
		for (unsigned int i=0; i<NL; i++)
		{
			depth_old[c][i].swap(depth[c][i]);
			xx_old[c][i].swap(xx[c][i]);
			yy_old[c][i].swap(yy[c][i]);				
		}


	//The number of levels of the pyramid does not match the number of levels used
	//in the odometry computation (because we might want to finish with lower resolutions)

	unsigned int pyr_levels = round(log(float(width/cols))/log(2.f)) + ctf_levels;

	//Generate levels
	for (unsigned int c=0; c<NC; c++)
	{
		for (unsigned int i = 0; i<pyr_levels; i++)
		{
			unsigned int s = pow(2.f,int(i));
			cols_i = width/s;
			rows_i = height/s;
			const int rows_i2 = 2*rows_i;
			const int cols_i2 = 2*cols_i;
			const int i_1 = i-1;

			if (i == 0)
				depth[c][i].swap(depth_wf[c]);		


			//                              Downsampling
			//-----------------------------------------------------------------------------
			else
			{
				for (unsigned int u = 0; u < cols_i; u++)
				for (unsigned int v = 0; v < rows_i; v++)
				{
					const int u2 = 2*u;
					const int v2 = 2*v;
					const float dcenter = depth[c][i_1](v2,u2);

					//Inner pixels
					if ((v>0)&&(v<rows_i-1)&&(u>0)&&(u<cols_i-1))
					{
						if (dcenter > 0.f)
						{
							float sum = 0.f;
							float weight = 0.f;

							for (int l = -2; l<3; l++)
							for (int k = -2; k<3; k++)
							{
								const float abs_dif = abs(depth[c][i_1](v2+k,u2+l)-dcenter);
								if (abs_dif < max_depth_dif)
								{
									const float aux_w = g_mask[2+k][2+l]*(max_depth_dif - abs_dif);
									weight += aux_w;
									sum += aux_w*depth[c][i_1](v2+k,u2+l);
								}
							}
							depth[c][i](v,u) = sum/weight;
						}
						else
						{
							float min_depth = 10.f;
							for (int l = -2; l<3; l++)
							for (int k = -2; k<3; k++)
							{
								const float d = depth[c][i_1](v2+k,u2+l);
								if ((d > 0.f)&&(d < min_depth))
									min_depth = d;
							}

							if (min_depth < 10.f)
								depth[c][i](v,u) = min_depth;
							else
								depth[c][i](v,u) = 0.f;
						}
					}

					//Boundary
					else
					{
						if (dcenter > 0.f)
						{
							float sum = 0.f;
							float weight = 0.f;

							for (int l = -2; l<3; l++)
							for (int k = -2; k<3; k++)
							{
								const int indv = v2+k,indu = u2+l;
								if ((indv>=0)&&(indv<rows_i2)&&(indu>=0)&&(indu<cols_i2))
								{
									const float abs_dif = abs(depth[c][i_1](indv,indu)-dcenter);
									if (abs_dif < max_depth_dif)
									{
										const float aux_w = g_mask[2+k][2+l]*(max_depth_dif - abs_dif);
										weight += aux_w;
										sum += aux_w*depth[c][i_1](indv,indu);
									}
								}
							}
							depth[c][i](v,u) = sum/weight;
						}
						else
						{
							float min_depth = 10.f;
							for (int l = -2; l<3; l++)
							for (int k = -2; k<3; k++)
							{
								const int indv = v2+k,indu = u2+l;
								if ((indv>=0)&&(indv<rows_i2)&&(indu>=0)&&(indu<cols_i2))
								{
									const float d = depth[c][i_1](indv,indu);
									if ((d > 0.f)&&(d < min_depth))
										min_depth = d;
								}
							}

							if (min_depth < 10.f)
								depth[c][i](v,u) = min_depth;
							else
								depth[c][i](v,u) = 0.f;
						}
					}
				}
			}

			//Calculate coordinates "xy" of the points
			const float inv_f_i = 2.f*tan(0.5f*fovh)/float(cols_i);
			const float disp_u_i = 0.5f*(cols_i-1);
			const float disp_v_i = 0.5f*(rows_i-1);

			for (unsigned int u = 0; u < cols_i; u++)
			for (unsigned int v = 0; v < rows_i; v++)
			if (depth[c][i](v,u) > 0.f)
			{
				xx[c][i](v,u) = (u - disp_u_i)*depth[c][i](v,u)*inv_f_i;
				yy[c][i](v,u) = (v - disp_v_i)*depth[c][i](v,u)*inv_f_i;
			}
			else
			{
				xx[c][i](v,u) = 0.f;
				yy[c][i](v,u) = 0.f;
			}
		}
	}
}

void CDifodo::buildCoordinatesPyramidFast()
{
	const float max_depth_dif = 0.1f;
	
	//Push coordinates back
	for (unsigned int c=0; c<NC; c++)
		for (unsigned int i=0; i<NL; i++)
		{
			depth_old[c][i].swap(depth[c][i]);
			xx_old[c][i].swap(xx[c][i]);
			yy_old[c][i].swap(yy[c][i]);
		}

    //The number of levels of the pyramid does not match the number of levels used
    //in the odometry computation (because we might want to finish with lower resolutions)

    unsigned int pyr_levels = round(log(float(width/cols))/log(2.f)) + ctf_levels;

    //Generate levels
	for (unsigned int c=0; c<NC; c++)
	{
		for (unsigned int i = 0; i<pyr_levels; i++)
		{
			unsigned int s = pow(2.f,int(i));
			cols_i = width/s;
			rows_i = height/s;
			const int rows_i2 = 2*rows_i;
			const int cols_i2 = 2*cols_i;
			const int i_1 = i-1;

			if (i == 0)
				depth[c][i].swap(depth_wf[c]);

			//                              Downsampling
			//-----------------------------------------------------------------------------
			else
			{            
				for (unsigned int u = 0; u < cols_i; u++)
					for (unsigned int v = 0; v < rows_i; v++)
					{
						const int u2 = 2*u;
						const int v2 = 2*v;
					
						//Inner pixels
						if ((v>0)&&(v<rows_i-1)&&(u>0)&&(u<cols_i-1))
						{		
							const Matrix4f d_block = depth[c][i_1].block<4,4>(v2-1,u2-1);
							float depths[4] = {d_block(5),d_block(6),d_block(9),d_block(10)};
							float dcenter;

							//Sort the array (try to find a good/representative value)
							for (signed char k = 2; k>=0; k--)
							if (depths[k+1] < depths[k])
								std::swap(depths[k+1],depths[k]);
							for (unsigned char k = 1; k<3; k++)
							if (depths[k] > depths[k+1])
								std::swap(depths[k+1],depths[k]);
							if (depths[2] < depths[1])
								dcenter = depths[1];
							else
								dcenter = depths[2];
						
							if (dcenter > 0.f)
							{	
								float sum = 0.f;
								float weight = 0.f;

								for (unsigned char k = 0; k<16; k++)
									{
										const float abs_dif = abs(d_block(k) - dcenter);
										if (abs_dif < max_depth_dif)
										{
											const float aux_w = f_mask(k)*(max_depth_dif - abs_dif);
											weight += aux_w;
											sum += aux_w*d_block(k);
										}
									}
								depth[c][i](v,u) = sum/weight;
							}
							else
								depth[c][i](v,u) = 0.f;

						}

						//Boundary
						else
						{
							const Matrix2f d_block = depth[c][i_1].block<2,2>(v2,u2);
							const float new_d = 0.25f*d_block.sumAll();
							if (new_d < 0.4f)
								depth[c][i](v,u) = 0.f;
							else
								depth[c][i](v,u) = new_d;
						}
					}
			}

			//Calculate coordinates "xy" of the points
			const float inv_f_i = 2.f*tan(0.5f*fovh)/float(cols_i);
			const float disp_u_i = 0.5f*(cols_i-1);
			const float disp_v_i = 0.5f*(rows_i-1);

			for (unsigned int u = 0; u < cols_i; u++) 
				for (unsigned int v = 0; v < rows_i; v++)
					if (depth[c][i](v,u) > 0.f)
					{
						xx[c][i](v,u) = (u - disp_u_i)*depth[c][i](v,u)*inv_f_i;
						yy[c][i](v,u) = (v - disp_v_i)*depth[c][i](v,u)*inv_f_i;
					}
					else
					{
						xx[c][i](v,u) = 0.f;
						yy[c][i](v,u) = 0.f;
					}
		}
	}
}

void CDifodo::performWarping()
{
	//Camera parameters - For now we use the same default values for all the cameras
	const float f = float(cols_i)/(2.f*tan(0.5f*fovh));
	const float disp_u_i = 0.5f*float(cols_i-1);
    const float disp_v_i = 0.5f*float(rows_i-1);
	const float cols_lim = float(cols_i-1);
	const float rows_lim = float(rows_i-1);

	for (unsigned int c=0; c<NC; c++)
	{
		//Rigid transformation estimated up to the present level
		Matrix4f acu_trans; 
		acu_trans.setIdentity();
		for (unsigned int i=1; i<=level; i++)
			acu_trans = transformations[c][i-1]*acu_trans;

		MatrixXf wacu(rows_i,cols_i); wacu.assign(0.f);
		depth_warped[c][image_level].assign(0.f);

		//						Warping loop
		//---------------------------------------------------------
		for (unsigned int j = 0; j<cols_i; j++)
			for (unsigned int i = 0; i<rows_i; i++)
			{		
				const float z = depth[c][image_level](i,j);
			
				if (z > 0.f)
				{
					//Transform point to the warped reference frame
					const float depth_w = acu_trans(0,0)*z + acu_trans(0,1)*xx[c][image_level](i,j) + acu_trans(0,2)*yy[c][image_level](i,j) + acu_trans(0,3);
					const float x_w = acu_trans(1,0)*z + acu_trans(1,1)*xx[c][image_level](i,j) + acu_trans(1,2)*yy[c][image_level](i,j) + acu_trans(1,3);
					const float y_w = acu_trans(2,0)*z + acu_trans(2,1)*xx[c][image_level](i,j) + acu_trans(2,2)*yy[c][image_level](i,j) + acu_trans(2,3);

					//Calculate warping
					const float uwarp = f*x_w/depth_w + disp_u_i;
					const float vwarp = f*y_w/depth_w + disp_v_i;

					//The warped pixel (which is not integer in general) contributes to all the surrounding ones
					if (( uwarp >= 0.f)&&( uwarp < cols_lim)&&( vwarp >= 0.f)&&( vwarp < rows_lim))
					{
						const int uwarp_l = uwarp;
						const int uwarp_r = uwarp_l + 1;
						const int vwarp_d = vwarp;
						const int vwarp_u = vwarp_d + 1;
						const float delta_r = float(uwarp_r) - uwarp;
						const float delta_l = uwarp - float(uwarp_l);
						const float delta_u = float(vwarp_u) - vwarp;
						const float delta_d = vwarp - float(vwarp_d);

						//Warped pixel very close to an integer value
						if (abs(round(uwarp) - uwarp) + abs(round(vwarp) - vwarp) < 0.05f)
						{
							depth_warped[c][image_level](round(vwarp), round(uwarp)) += depth_w;
							wacu(round(vwarp), round(uwarp)) += 1.f;
						}
						else
						{
							const float w_ur = square(delta_l) + square(delta_d);
							depth_warped[c][image_level](vwarp_u,uwarp_r) += w_ur*depth_w;
							wacu(vwarp_u,uwarp_r) += w_ur;

							const float w_ul = square(delta_r) + square(delta_d);
							depth_warped[c][image_level](vwarp_u,uwarp_l) += w_ul*depth_w;
							wacu(vwarp_u,uwarp_l) += w_ul;

							const float w_dr = square(delta_l) + square(delta_u);
							depth_warped[c][image_level](vwarp_d,uwarp_r) += w_dr*depth_w;
							wacu(vwarp_d,uwarp_r) += w_dr;

							const float w_dl = square(delta_r) + square(delta_u);
							depth_warped[c][image_level](vwarp_d,uwarp_l) += w_dl*depth_w;
							wacu(vwarp_d,uwarp_l) += w_dl;
						}
					}
				}
			}

		//Scale the averaged depth and compute spatial coordinates
		const float inv_f_i = 1.f/f;
		for (unsigned int u = 0; u<cols_i; u++)
			for (unsigned int v = 0; v<rows_i; v++)
			{	
				if (wacu(v,u) > 0.f)
				{
					depth_warped[c][image_level](v,u) /= wacu(v,u);
					xx_warped[c][image_level](v,u) = (u - disp_u_i)*depth_warped[c][image_level](v,u)*inv_f_i;
					yy_warped[c][image_level](v,u) = (v - disp_v_i)*depth_warped[c][image_level](v,u)*inv_f_i;
				}
				else
				{
					depth_warped[c][image_level](v,u) = 0.f;
					xx_warped[c][image_level](v,u) = 0.f;
					yy_warped[c][image_level](v,u) = 0.f;
				}
			}
	}
}

void CDifodo::calculateCoord()
{	
	num_valid_points = 0;

	for (unsigned int c=0; c<NC; c++)
	{
		null[c].resize(rows_i, cols_i);
		null[c].assign(false);

		const Matrix4f inv_cmat = calib_mat[c].inverse();
	
		for (unsigned int u = 0; u < cols_i; u++)
			for (unsigned int v = 0; v < rows_i; v++)
			{
				if ((depth_old[c][image_level](v,u)) == 0.f || (depth_warped[c][image_level](v,u) == 0.f))
				{
					depth_inter[c][image_level](v,u) = 0.f;
					xx_inter[c][image_level](v,u) = 0.f;
					yy_inter[c][image_level](v,u) = 0.f;
					null[c](v, u) = true;
				}
				else
				{
					depth_inter[c][image_level](v,u) = 0.5f*(depth_old[c][image_level](v,u) + depth_warped[c][image_level](v,u));
					xx_inter[c][image_level](v,u) = 0.5f*(xx_old[c][image_level](v,u) + xx_warped[c][image_level](v,u));
					yy_inter[c][image_level](v,u) = 0.5f*(yy_old[c][image_level](v,u) + yy_warped[c][image_level](v,u));
					null[c](v, u) = false;
					if ((u>0)&&(v>0)&&(u<cols_i-1)&&(v<rows_i-1))
						num_valid_points++;

					//Obtain the global coordinates - Cuidado con cómo estén los ejes definidos para la matrix de calibración!!!!
					const float &zi = depth_inter[c][image_level](v,u);
					const float &xi = xx_inter[c][image_level](v,u);
					const float &yi = yy_inter[c][image_level](v,u);

					zz_global[c][image_level](v, u) = inv_cmat(0, 0)*zi + inv_cmat(0, 1)*xi + inv_cmat(0, 2)*yi + inv_cmat(0, 3);
					xx_global[c][image_level](v, u) = inv_cmat(1, 0)*zi + inv_cmat(1, 1)*xi + inv_cmat(1, 2)*yi + inv_cmat(1, 3);
					yy_global[c][image_level](v, u) = inv_cmat(2, 0)*zi + inv_cmat(2, 1)*xi + inv_cmat(2, 2)*yi + inv_cmat(2, 3);
				}
			}
	}
}

void CDifodo::calculateDepthDerivatives()
{
	for (unsigned int c=0; c<NC; c++)
	{
		dt[c].resize(rows_i,cols_i); dt[c].assign(0.f);
		du[c].resize(rows_i,cols_i); du[c].assign(0.f);
		dv[c].resize(rows_i,cols_i); dv[c].assign(0.f);

		//Compute connectivity
		MatrixXf rx_ninv(rows_i,cols_i);
		MatrixXf ry_ninv(rows_i,cols_i);
		rx_ninv.assign(1.f); ry_ninv.assign(1.f);

		for (unsigned int u = 0; u < cols_i-1; u++)
			for (unsigned int v = 0; v < rows_i; v++)
				if (null[c](v,u) == false)
				{
					rx_ninv(v,u) = sqrtf(square(xx_inter[c][image_level](v,u+1) - xx_inter[c][image_level](v,u))
										+ square(depth_inter[c][image_level](v,u+1) - depth_inter[c][image_level](v,u)));
				}

		for (unsigned int u = 0; u < cols_i; u++)
			for (unsigned int v = 0; v < rows_i-1; v++)
				if (null[c](v,u) == false)
				{
					ry_ninv(v,u) = sqrtf(square(yy_inter[c][image_level](v+1,u) - yy_inter[c][image_level](v,u))
										+ square(depth_inter[c][image_level](v+1,u) - depth_inter[c][image_level](v,u)));
				}


		//Spatial derivatives
		for (unsigned int v = 0; v < rows_i; v++)
		{
			for (unsigned int u = 1; u < cols_i-1; u++)
				if (null[c](v,u) == false)
					du[c](v,u) = (rx_ninv(v,u-1)*(depth_inter[c][image_level](v,u+1)-depth_inter[c][image_level](v,u)) + rx_ninv(v,u)*(depth_inter[c][image_level](v,u) - depth_inter[c][image_level](v,u-1)))/(rx_ninv(v,u)+rx_ninv(v,u-1));

			du[c](v,0) = du[c](v,1);
			du[c](v,cols_i-1) = du[c](v,cols_i-2);
		}

		for (unsigned int u = 0; u < cols_i; u++)
		{
			for (unsigned int v = 1; v < rows_i-1; v++)
				if (null[c](v,u) == false)
					dv[c](v,u) = (ry_ninv(v-1,u)*(depth_inter[c][image_level](v+1,u)-depth_inter[c][image_level](v,u)) + ry_ninv(v,u)*(depth_inter[c][image_level](v,u) - depth_inter[c][image_level](v-1,u)))/(ry_ninv(v,u)+ry_ninv(v-1,u));

			dv[c](0,u) = dv[c](1,u);
			dv[c](rows_i-1,u) = dv[c](rows_i-2,u);
		}

		//Temporal derivative
		for (unsigned int u = 0; u < cols_i; u++)
			for (unsigned int v = 0; v < rows_i; v++)
				if (null[c](v,u) == false)
					dt[c](v,u) = depth_warped[c][image_level](v,u) - depth_old[c][image_level](v,u);
	}
}

void CDifodo::computeWeights()
{
	for (unsigned int c=0; c<NC; c++)
	{
		weights[c].resize(rows_i, cols_i);
		weights[c].assign(0.f);

		//Parameters for the measurmente error
		const float kz2 = 0.01f;  //square(1.425e-5) / 25
	
		//Parameters for linearization error
		const float kduvt = 200.f;
		const float k2dt = 5.f;
		const float k2duv = 5.f;
	
		for (unsigned int u = 1; u < cols_i-1; u++)
			for (unsigned int v = 1; v < rows_i-1; v++)
				if (null[c](v,u) == false)
				{
					//					Compute measurment error (simplified)
					//-----------------------------------------------------------------------
					const float error_m = kz2*square(square(depth_inter[c][image_level](v,u)));

				
					//					Compute linearization error
					//-----------------------------------------------------------------------
					const float ini_du = depth_old[c][image_level](v,u+1) - depth_old[c][image_level](v,u-1);
					const float ini_dv = depth_old[c][image_level](v+1,u) - depth_old[c][image_level](v-1,u);
					const float final_du = depth_warped[c][image_level](v,u+1) - depth_warped[c][image_level](v,u-1);
					const float final_dv = depth_warped[c][image_level](v+1,u) - depth_warped[c][image_level](v-1,u);

					const float dut = ini_du - final_du;
					const float dvt = ini_dv - final_dv;
					const float duu = du[c](v,u+1) - du[c](v,u-1);
					const float dvv = dv[c](v+1,u) - dv[c](v-1,u);
					const float dvu = dv[c](v,u+1) - dv[c](v,u-1); //Completely equivalent to compute duv

					const float error_l = kduvt*square(dt[c](v,u) + square(du[c](v,u)) + square(dv[c](v,u))) + k2dt*(square(dut) + square(dvt))
												+ k2duv*(square(duu) + square(dvv) + square(dvu));

					//Weight
					weights[c](v,u) = sqrt(1.f/(error_m + error_l));
				
				}
	}
	
	//Normalize weights in the range [0,1]
	float max_weight = 0.f;

	for (unsigned int c=0; c<NC; c++)
	{
		if (weights[c].maximum() > max_weight)
			max_weight = weights[c].maximum();
	}

	const float inv_max = 1.f/max_weight;
	for (unsigned int c=0; c<NC; c++)
		weights[c] *= inv_max;
}

void CDifodo::solveOneLevel()
{
	//Fill the matrix A and the vector B
	//The order of the unknowns is (vz, vx, vy, wz, wx, wy)
	//The points order will be (1,1), (1,2)...(1,cols-1), (2,1), (2,2)...(row-1,cols-1).

	MatrixXf A(num_valid_points,6);
	MatrixXf B(num_valid_points,1);
	unsigned int cont = 0;
	const float f_inv = float(cols_i)/(2.f*tan(0.5f*fovh));

	for (unsigned int c=0; c<NC; c++)
	{
		Eigen::Matrix3f T_rot = calib_mat[c].block<3,3>(0,0);

		for (unsigned int u = 1; u < cols_i-1; u++)
			for (unsigned int v = 1; v < rows_i-1; v++)
				if (null[c](v,u) == false)
				{
					// Precomputed expressions
					const float d = depth_inter[c][image_level](v,u);
					const float inv_d = 1.f/d;
					const float x = xx_inter[c][image_level](v,u);
					const float y = yy_inter[c][image_level](v,u);
					const float dycomp = du[c](v,u)*f_inv*inv_d;
					const float dzcomp = dv[c](v,u)*f_inv*inv_d;
					const float tw = weights[c](v,u);
					const float z_global = zz_global[c][image_level](v,u);
					const float x_global = xx_global[c][image_level](v,u);
					const float y_global = yy_global[c][image_level](v,u);

					//Fill A and b
					Eigen::Matrix<float, 1, 3> J_cam; J_cam << -1.f - dycomp*x*inv_d - dzcomp*y*inv_d, dycomp, dzcomp;
					Eigen::Matrix<float, 3, 6> J_rig; J_rig.assign(0.f); J_rig(0,0) = -1.f; J_rig(1,1) = -1.f; J_rig(2,2) = -1.f;
					J_rig(0,4) = -y_global; J_rig(0,5) = x_global; J_rig(1,3) = y_global; J_rig(1,5) = -z_global; J_rig(2,3) = -x_global; J_rig(2,4) = z_global;
					A.row(cont) = tw*J_cam*T_rot*J_rig;

					B(cont,0) = tw*(-dt[c](v,u));
					cont++;
				}
	}
	
	//Solve the linear system of equations using weighted least squares
	MatrixXf AtA, AtB;
	AtA.multiply_AtA(A);
	AtB.multiply_AtB(A,B);
	MatrixXf Var = AtA.ldlt().solve(AtB);

	//Covariance matrix calculation 
	MatrixXf res = -B;
	for (unsigned int k = 0; k<6; k++)
		res += Var(k)*A.col(k);

	est_cov = (1.f/float(num_valid_points-6))*AtA.inverse()*res.squaredNorm();

	//Update last velocity in local coordinates
	kai_loc_level = Var;
}

void CDifodo::odometryCalculation()
{
	//Clock to measure the runtime
	utils::CTicTac clock;
	clock.Tic();

	//Build the gaussian pyramid
	if (fast_pyramid)	buildCoordinatesPyramidFast();
	else				buildCoordinatesPyramid();

    //Coarse-to-fines scheme
    for (unsigned int i=0; i<ctf_levels; i++)
    {
        //Previous computations
        global_trans[i].setIdentity();
        for (unsigned int c=0; c<NC; c++)
            transformations[c][i].setIdentity();

        level = i;
        unsigned int s = pow(2.f,int(ctf_levels-(i+1)));
        cols_i = cols/s; rows_i = rows/s;
        image_level = ctf_levels - i + round(log(float(width/cols))/log(2.f)) - 1;

        //1. Perform warping
        if (i == 0)
            for (unsigned int c=0; c<NC; c++)
            {
                depth_warped[c][image_level] = depth[c][image_level];
                xx_warped[c][image_level] = xx[c][image_level];
                yy_warped[c][image_level] = yy[c][image_level];
            }
        else
            performWarping();

        //2. Calculate inter coords and find null measurements
        calculateCoord();

        //3. Compute derivatives
        calculateDepthDerivatives();

        //4. Compute weights
        computeWeights();

        //5. Solve odometry
        if (num_valid_points > 6)
            solveOneLevel();

        //6. Filter solution
        filterLevelSolution();
    }

    //Update poses
    poseUpdate();

    //Save runtime
    execution_time = 1000.f*clock.Tac();
}

void CDifodo::filterLevelSolution()
{  
	//		Calculate Eigenvalues and Eigenvectors
	//----------------------------------------------------------
	SelfAdjointEigenSolver<MatrixXf> eigensolver(est_cov);
	if (eigensolver.info() != Success) 
	{ 
		printf("\n Eigensolver couldn't find a solution. Pose is not updated");
		return;
	}
	
	//First, we have to describe both the new linear and angular velocities in the "eigenvector" basis
	//-------------------------------------------------------------------------------------------------
	Matrix<float,6,6> Bii;
	Matrix<float,6,1> kai_b;
	Bii = eigensolver.eigenvectors();
	kai_b = Bii.colPivHouseholderQr().solve(kai_loc_level);

	//Second, we have to describe both the old linear and angular velocities in the "eigenvector" basis
	//-------------------------------------------------------------------------------------------------
	Matrix<float,6,1> kai_loc_sub = kai_loc_old;

	//Important: we have to substract the previous levels' solutions from the old velocity.
	Matrix4f acu_trans;
	acu_trans.setIdentity();
	for (unsigned int i=0; i<level; i++)
		acu_trans = global_trans[i]*acu_trans;

    Matrix<float, 4, 4> log_trans;
#if EIGEN_VERSION_AT_LEAST(3,1,0)  // Eigen 3.1.0 needed for Matrix::log()
    log_trans = acu_trans.log();
#else
    //Alternative to compute the logarithm of the matrix
    Matrix<float,4,4> aux;
    aux = acu_trans - MatrixXf::Identity(4,4);
    log_trans = aux - 0.5f*aux*aux + 0.33333f*aux*aux*aux - 0.25f*aux*aux*aux*aux + 0.2f*aux*aux*aux*aux*aux;
#endif
    kai_loc_sub(0) -= log_trans(0,3); kai_loc_sub(1) -= log_trans(1,3); kai_loc_sub(2) -= log_trans(2,3);
    kai_loc_sub(3) += log_trans(1,2); kai_loc_sub(4) -= log_trans(0,2); kai_loc_sub(5) += log_trans(0,1);


	//Transform that local representation to the "eigenvector" basis
	Matrix<float,6,1> kai_b_old;
	kai_b_old = Bii.colPivHouseholderQr().solve(kai_loc_sub);

	//									Filter velocity
	//--------------------------------------------------------------------------------
	const float cf = previous_speed_eig_weight*expf(-int(level)), df = previous_speed_const_weight*expf(-int(level));
	Matrix<float,6,1> kai_b_fil;
	for (unsigned int i=0; i<6; i++)
		kai_b_fil(i) = (kai_b(i) + (cf*eigensolver.eigenvalues()(i,0) + df)*kai_b_old(i))/(1.f + cf*eigensolver.eigenvalues()(i) + df);

	//Transform filtered velocity to the local reference frame 
	Matrix<float, 6, 1> kai_loc_fil = Bii.inverse().colPivHouseholderQr().solve(kai_b_fil);

	//Compute the rigid transformation with respect to the robot coordinate system
	Matrix<float,4,4> local_mat; local_mat.assign(0.f); 
	local_mat(0,1) = -kai_loc_fil(5); local_mat(1,0) = kai_loc_fil(5);
	local_mat(0,2) = kai_loc_fil(4); local_mat(2,0) = -kai_loc_fil(4);
	local_mat(1,2) = -kai_loc_fil(3); local_mat(2,1) = kai_loc_fil(3);
	local_mat(0,3) = kai_loc_fil(0);
	local_mat(1,3) = kai_loc_fil(1);
	local_mat(2,3) = kai_loc_fil(2);
	global_trans[level] = local_mat.exp();

	//Compute the rigid transformations associated to the local coordinate systems of each camera
	//-------------------------------------------------------------------------------------------
	for (unsigned int c=0; c<NC; c++)
	{
		//cout << endl << "Calib matrix: " << endl << calib_mat[c];
		//cout << endl << "Calib matrix inv: " << endl << calib_mat[c].inverse();
		//cout << endl << "CM*CM_inv: " << endl << calib_mat[c]*calib_mat[c].inverse();

		//This should be the right one!
		transformations[c][level] = calib_mat[c]*global_trans[level]*calib_mat[c].inverse();
		//cout << endl << "Tc: " << endl << transformations[c][level];
	}
}

void CDifodo::poseUpdate()
{
	//First, compute the overall transformation
	//---------------------------------------------------
	Matrix4f acu_trans;
	acu_trans.setIdentity();
	for (unsigned int i=1; i<=ctf_levels; i++)
		acu_trans = global_trans[i-1]*acu_trans;


	//Compute the new estimates in the local and absolutes reference frames
	//---------------------------------------------------------------------
    Matrix<float, 4, 4> log_trans;
#if EIGEN_VERSION_AT_LEAST(3,1,0)  // Eigen 3.1.0 needed for Matrix::log()
    log_trans = acu_trans.log();
#else
    //Alternative to compute the logarithm of the matrix
    Matrix<float,4,4> aux;
    aux = acu_trans - MatrixXf::Identity(4,4);
    log_trans = aux - 0.5f*aux*aux + 0.33333f*aux*aux*aux - 0.25f*aux*aux*aux*aux + 0.2f*aux*aux*aux*aux*aux;
#endif
	kai_loc(0) = log_trans(0,3); kai_loc(1) = log_trans(1,3); kai_loc(2) = log_trans(2,3);
	kai_loc(3) = -log_trans(1,2); kai_loc(4) = log_trans(0,2); kai_loc(5) = -log_trans(0,1);

	CMatrixDouble33 inv_trans;
	CMatrixFloat31 v_abs, w_abs;

    global_pose.getRotationMatrix(inv_trans);
	v_abs = inv_trans.cast<float>()*kai_loc.topRows(3);
	w_abs = inv_trans.cast<float>()*kai_loc.bottomRows(3);
	kai_abs.topRows<3>() = v_abs;
	kai_abs.bottomRows<3>() = w_abs;	


	//						Update poses
	//-------------------------------------------------------	
    global_oldpose = global_pose;
	CMatrixDouble44 aux_acu = acu_trans;
	poses::CPose3D pose_aux(aux_acu);
    global_pose = global_pose + pose_aux;


	//Compute the velocity estimate in the new ref frame (to be used by the filter in the next iteration)
	//---------------------------------------------------------------------------------------------------
    global_pose.getRotationMatrix(inv_trans);
	kai_loc_old.topRows<3>() = inv_trans.inverse().cast<float>()*kai_abs.topRows(3);
	kai_loc_old.bottomRows<3>() = inv_trans.inverse().cast<float>()*kai_abs.bottomRows(3);
}





