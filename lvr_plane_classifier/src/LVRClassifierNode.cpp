#include <lvr/io/ModelFactory.hpp>
#include <lvr/geometry/HalfEdgeMesh.hpp>
#include <lvr/classification/FurnitureFeatureClassifier.hpp>
#include <semantic_object_maps_msgs/PlanarPatch.h>
#include <semantic_object_maps_msgs/PlanarPatchArray.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace lvr;

const int 	PLANE_ITERATIONS 		= 3;
const float NORMAL_THRESHOLD 		= 0.95;
const int 	SMALL_REGION_THRESHOLD 	= 0;
const int 	MIN_PLANE_SIZE 			= 7;
const int	FILL_HOLES 				= 10;
const float LINE_FUSION_THRESHOLD 	= 0.00999999978;
const int	CONTOUR_ITERATIONS 		= 0;


typedef ColorVertex<float, unsigned char>  cVertex;
typedef Normal<float> cNormal;
typedef Region<cVertex, cNormal> cRegion;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "lvr_classifier_node");
	ros::NodeHandle nh;

	ros::Publisher plane_publisher = nh.advertise<semantic_object_maps_msgs::PlanarPatchArray>("lvr_classified_planes", 1000);
	ros::Publisher contour_publisher = nh.advertise<visualization_msgs::MarkerArray>("lvr_plane_contours", 1000);

	// Load data from given file
	ModelPtr model = ModelFactory::readModel( std::string(argv[1]));

	// Generate half edge mesh representation
	HalfEdgeMesh<cVertex, cNormal > mesh( model->m_mesh );

	vector<cRegion* >* regionPtr = mesh.getRegionsPtr();

	FurnitureFeatureClassifier<cVertex , cNormal > classifier(regionPtr);
	mesh.setClassifier(&classifier);

	mesh.cleanContours(CONTOUR_ITERATIONS);
	mesh.setDepth(100);

	// Apply mesh optimization filters
	mesh.optimizePlanes(
			PLANE_ITERATIONS,
			NORMAL_THRESHOLD,
			MIN_PLANE_SIZE,
			SMALL_REGION_THRESHOLD,
			true);


	mesh.fillHoles(FILL_HOLES);
	mesh.optimizePlaneIntersections();
	mesh.restorePlanes(MIN_PLANE_SIZE);


	// Create a marker for each contour
	mesh.resetUsedFlags();
	vector<visualization_msgs::Marker> contour_markers;
	for(size_t i = 0; i < regionPtr->size(); i++)
	{
		cRegion* r = regionPtr->at(i);
		vector<vector<cVertex> > contours = r->getContours(LINE_FUSION_THRESHOLD);
		if(contours.size() > 0)
		{
			visualization_msgs::Marker contour_marker;
			contour_marker.header.frame_id = "map";
			contour_marker.ns = "lvr_contours";
			contour_marker.id = i;
			contour_marker.type = 4;
			contour_marker.action = 0;
			contour_marker.color.a = 1.0;
			contour_marker.color.r = 0.0;
			contour_marker.color.g = 0.0;
			contour_marker.color.b = 0.0;
			contour_marker.scale.x = 0.01;
			contour_marker.pose.orientation.x = 0;
			contour_marker.pose.orientation.y = 0;
			contour_marker.pose.orientation.z = 0;
			contour_marker.pose.orientation.w = 1;

			contour_marker.lifetime = ros::Duration(0);

			vector<cVertex> c = contours[0];


			for(size_t j = 0; j < c.size(); j++)
			{
				cVertex vertex = c[j];
				geometry_msgs::Point p;
				p.x = vertex.x;
				p.y = vertex.y;
				p.z = vertex.z;
				contour_marker.points.push_back(p);
			}
			contour_markers.push_back(contour_marker);
		}
		else
		{
			// Should not happen
			cout << "Region " << i << " has no contour!" << endl;
		}
	}


	// Finalize and reduce mesh, includes classification
	mesh.finalizeAndRetesselate(
			false, // Textures not yet supported in this tool
			LINE_FUSION_THRESHOLD);

	// Create output model and save to file
	ModelPtr out_model( new Model( mesh.meshBuffer() ) );
	ModelFactory::saveModel( out_model, "optimized_mesh.ply");

	// Test classifier
	semantic_object_maps_msgs::PlanarPatchArray patch_array;
	visualization_msgs::MarkerArray markers;
	for(int i = 0; i < classifier.numFeatures(); i++)
	{
		PlanarClusterFeature pf = classifier.getFeature(i);
		semantic_object_maps_msgs::PlanarPatch patch;

		patch.area = 		pf.area;
		patch.bbox.x = 		pf.w;
		patch.bbox.y = 		pf.h;
		patch.bbox.z = 		pf.d;
		patch.normal.x = 	pf.nx;
		patch.normal.y = 	pf.ny;
		patch.normal.z = 	pf.nz;
		patch.centroid.x = 	pf.cx;
		patch.centroid.y = 	pf.cy;
		patch.centroid.z = 	pf.cz;

		std::stringstream ss;
		ss << pf.index;
		patch.id = ss.str();

		visualization_msgs::Marker m = contour_markers[pf.index];
		switch(pf.orientation)
		{
		case HORIZONTAL:
			patch.orientation = 1;
			m.color.r = 1.0;
			break;
		case VERTICAL:
			m.color.b = 1.0;
			patch.orientation = 2;
			break;
		default:
			m.color.r = 0.3;
			m.color.b = 0.3;
			m.color.g = 0.3;
			patch.orientation = 0;
		}

		patch_array.patches.push_back(patch);
		markers.markers.push_back(m);

	}

	plane_publisher.publish(patch_array);
	contour_publisher.publish(markers);
	ros::spin();

	return 0;
}
