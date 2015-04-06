/*
Copyright (C) 2014 Steven Hickson

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA

*/
// TestVideoSegmentation.cpp : Defines the entry point for the console application.
//

#include "poisson_reconstruction.h"

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
	printHelp (int, char **argv)
{
	print_error ("Syntax is: %s input.pcd output.vtk <options>\n", argv[0]);
	print_info ("  where options are:\n");
	print_info ("                     -depth X          = set the maximum depth of the tree that will be used for surface reconstruction (default: ");
	print_value ("%d", default_depth); print_info (")\n");
	print_info ("                     -solver_divide X  = set the the depth at which a block Gauss-Seidel solver is used to solve the Laplacian equation (default: ");
	print_value ("%d", default_solver_divide); print_info (")\n");
	print_info ("                     -iso_divide X     = Set the depth at which a block iso-surface extractor should be used to extract the iso-surface (default: ");
	print_value ("%d", default_iso_divide); print_info (")\n");
	print_info ("                     -point_weight X   = Specifies the importance that interpolation of the point samples is given in the formulation of the screened Poisson equation. The results of the original (unscreened) Poisson Reconstruction can be obtained by setting this value to 0. (default: ");
	print_value ("%f", default_point_weight); print_info (")\n");
}

void prepareCloud (pcl::PCLPointCloud2 &cloud)
{
	PointCloud<PointXYZRGBA>::Ptr output (new PointCloud<pcl::PointXYZRGBA> ());
	fromPCLPointCloud2(cloud,*output);
	cloud.width = 640;
	cloud.height = 480;
	cloud.is_dense = true;
	output->width = 640;
	output->height = 480;
	output->is_dense = true;

	//Need to set 0s to nan
	PointCloud<PointXYZRGBA>::iterator pCloud = output->begin();
	while(pCloud != output->end())
	{
		if(pCloud->z == 0)
			pCloud->z = FP_NAN;
		++pCloud;
	}

	// estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(output);
	ne.compute(*normals);
	PCLPointCloud2 out,out2;
	toPCLPointCloud2(*normals, out);
	toPCLPointCloud2(*output, out2);
	concatenateFields(out,out2,cloud);

	/*MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
	mls.setInputCloud (output);
	mls.setSearchRadius (0.01);
	mls.setPolynomialFit (true);
	mls.setPolynomialOrder (2);
	mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
	mls.setUpsamplingRadius (0.005);
	mls.setUpsamplingStepSize (0.003);
	PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ> ());
	mls.process (*cloud_smoothed);

	NormalEstimationOMP<PointXYZ, Normal> ne;
	ne.setNumberOfThreads (8);
	ne.setInputCloud (cloud_smoothed);
	ne.setRadiusSearch (0.01);
	Eigen::Vector4f centroid;
	compute3DCentroid (*cloud_smoothed, centroid);
	ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
	PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());
	ne.compute (*cloud_normals);
	for (size_t i = 0; i < cloud_normals->size (); ++i)
	{
	cloud_normals->points[i].normal_x *= -1;
	cloud_normals->points[i].normal_y *= -1;
	cloud_normals->points[i].normal_z *= -1;
	}
	PointCloud<PointNormal>::Ptr cloud_smoothed_normals (new PointCloud<PointNormal> ());
	concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);
	PCLPointCloud2 out;
	toPCLPointCloud2(*cloud_smoothed_normals, out);
	concatenateFields(cloud,out,cloud);*/
}

bool
	loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
	TicToc tt;
	print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

	tt.tic ();
	if (loadPCDFile (filename, cloud) < 0)
		return (false);
	prepareCloud(cloud);

	print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
	print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

	return (true);
}

void
	compute (const pcl::PCLPointCloud2::ConstPtr &input, PolygonMesh &output,
	int depth, int solver_divide, int iso_divide, float point_weight)
{
	PointCloud<PointNormal>::Ptr xyz_cloud (new pcl::PointCloud<PointNormal> ());
	fromPCLPointCloud2 (*input, *xyz_cloud);

	print_info ("Using parameters: depth %d, solverDivide %d, isoDivide %d\n", depth, solver_divide, iso_divide);

	Poisson<PointNormal> poisson;
	poisson.setDepth (depth);
	poisson.setSolverDivide (solver_divide);
	poisson.setIsoDivide (iso_divide);
	poisson.setPointWeight (point_weight);
	poisson.setInputCloud (xyz_cloud);

	TicToc tt;
	tt.tic ();
	print_highlight ("Computing ...");
	poisson.reconstruct (output);

	print_info ("[Done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}

void
	saveCloud (const std::string &filename, const PolygonMesh &output)
{
	TicToc tt;
	tt.tic ();

	print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
	saveVTKFile (filename, output);

	print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}

/* ---[ */
int
	main (int argc, char** argv)
{
	print_info ("Compute the surface reconstruction of a point cloud using the Poisson surface reconstruction (pcl::surface::Poisson). For more information, use: %s -h\n", argv[0]);

	if (argc < 3)
	{
		printHelp (argc, argv);
		return (-1);
	}

	// Parse the command line arguments for .pcd files
	std::vector<int> pcd_file_indices;
	pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
	if (pcd_file_indices.size () != 1)
	{
		print_error ("Need one input PCD file and one output VTK file to continue.\n");
		return (-1);
	}

	std::vector<int> vtk_file_indices = parse_file_extension_argument (argc, argv, ".vtk");
	if (vtk_file_indices.size () != 1)
	{
		print_error ("Need one output VTK file to continue.\n");
		return (-1);
	}

	// Command line parsing
	int depth = default_depth;
	parse_argument (argc, argv, "-depth", depth);
	print_info ("Using a depth of: "); print_value ("%d\n", depth);

	int solver_divide = default_solver_divide;
	parse_argument (argc, argv, "-solver_divide", solver_divide);
	print_info ("Setting solver_divide to: "); print_value ("%d\n", solver_divide);

	int iso_divide = default_iso_divide;
	parse_argument (argc, argv, "-iso_divide", iso_divide);
	print_info ("Setting iso_divide to: "); print_value ("%d\n", iso_divide);

	float point_weight = default_point_weight;
	parse_argument (argc, argv, "-point_weight", point_weight);
	print_info ("Setting point_weight to: "); print_value ("%f\n", point_weight);

	// Load the first file
	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
	if (!loadCloud (argv[pcd_file_indices[0]], *cloud))
		return (-1);

	// Apply the Poisson surface reconstruction algorithm
	PolygonMesh output;
	compute (cloud, output, depth, solver_divide, iso_divide, point_weight);

	// Save into the second file
	saveCloud (argv[vtk_file_indices[0]], output);

	printf("Done!\n");
	cin.get();
}