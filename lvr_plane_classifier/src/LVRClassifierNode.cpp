#include <lvr/io/ModelFactory.hpp>
#include <lvr/geometry/HalfEdgeMesh.hpp>
#include <lvr/classification/FurnitureFeatureClassifier.hpp>
#include <semantic_object_maps_msgs/PlanarPatch.h>
#include <semantic_object_maps_msgs/PlanarPatchArray.h>

#include <ros/ros.h>

using namespace lvr;

const int 	PLANE_ITERATIONS 		= 3;
const float NORMAL_THRESHOLD 		= 0.95;
const int 	SMALL_REGION_THRESHOLD 	= 0;
const int 	MIN_PLANE_SIZE 			= 7;
const int	FILL_HOLES 				= 10;
const float LINE_FUSION_THRESHOLD 	= 0.00999999978;
const int	CONTOUR_ITERATIONS 		= 0;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "lvr_classifier_node");
	ros::NodeHandle nh;

	ros::Publisher plane_publisher = nh.advertise<semantic_object_maps_msgs::PlanarPatchArray>("lvr_classified_planes", 1000);


	// Load data from given file
	ModelPtr model = ModelFactory::readModel( std::string(argv[1]));

	// Generate half edge mesh representation
	HalfEdgeMesh<ColorVertex<float, unsigned char> , Normal<float> > mesh( model->m_mesh );

	FurnitureFeatureClassifier<ColorVertex<float, unsigned char> , Normal<float> > classifier(mesh.getRegionsPtr());
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

	// Finalize and reduce mesh, includes classification
	mesh.finalizeAndRetesselate(
			false, // Textures not yet supported in this tool
			LINE_FUSION_THRESHOLD);

	// Create output model and save to file
	ModelPtr out_model( new Model( mesh.meshBuffer() ) );
	ModelFactory::saveModel( out_model, "optimized_mesh.ply");

	// Test classifier
	semantic_object_maps_msgs::PlanarPatchArray patch_array;
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

		switch(pf.orientation)
		{
		case HORIZONTAL:
			patch.orientation = 1;
			break;
		case VERTICAL:
			patch.orientation = 2;
			break;
		default:
			patch.orientation = 0;
		}

		patch_array.patches.push_back(patch);

//		std::cout << "Index: " << pf.index << std::endl;
//		std::cout << "Centroid: " << pf.cx << " " << pf.cy << " " << pf.cz << std::endl;
//		std::cout << "W/H/D: " << pf.w << " " << pf.h << " " << pf.d << std::endl;
//		std::cout << "Normal: " << pf.nx << " " << pf.ny << " " << pf.nz << std::endl;
//		std::cout << "Area: " << pf.area << std::endl;
//		switch(pf.orientation)
//		{
//		case HORIZONTAL:
//			std::cout << "Orientation: Horizontal" << std::endl;
//			break;
//		case VERTICAL:
//			std::cout << "Orientation: Vertical" << std::endl;
//			break;
//		default:
//			std::cout << "Orientation: Undefined" << std::endl;
//		}
//		std::cout << std::endl;
	}

	plane_publisher.publish(patch_array);
	ros::spin();

	return 0;
}
