#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

typedef pcl::PointXYZ Point;

void
PointCloud2Vector3d (pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);

void
visualizeCurve (ON_NurbsCurve &curve,
                ON_NurbsSurface &surface,
                pcl::visualization::PCLVisualizer &viewer);