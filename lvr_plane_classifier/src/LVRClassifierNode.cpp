#include <lvr/io/ModelFactory.hpp>
#include <lvr/geometry/HalfEdgeMesh.hpp>
#include <lvr/classification/FurnitureFeatureClassifier.hpp>
#include <semantic_object_maps_msgs/PlanarPatch.h>
#include <semantic_object_maps_msgs/PlanarPatchArray.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/LinearMath/Transform.h>

#include <Eigen/Geometry>

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

/******************************************************************************
 * @brief	Transforms the mesh model according to the given transformation
 *
 * @param	model			A LVR model instance
 * @param 	transfrom		A transformation
 *
 ******************************************************************************/
void transformModel(ModelPtr& model, const tf::Transform& transform)
{
	MeshBufferPtr mesh = model->m_mesh;
	size_t n;
	floatArr vertices = mesh->getVertexArray(n);
	for(int i = 0; i < n; i++)
	{
		tf::Vector3 v(vertices[3 * i], vertices[3 * i + 1], vertices[3 * i + 2]);
		tf::Vector3 t = transform * v;

		vertices[i * 3    ] = t.x();
		vertices[i * 3 + 1] = t.y();
		vertices[i * 3 + 2] = t.z();
	}
}

/******************************************************************************
 * @brief	Constructs a ROS transformation from a calibration file. The
 *			calibration for the model should be stored in a file that has the
 *			same filename as the model but the suffix .calibration.
 *
 * @param	filename		The name of the loaded model
 ******************************************************************************/
tf::Transform getTransfromFromCalibration(std::string filename)
{
	tf::Transform t;

	// Construct name for calibration file
	std::string calibrationFileName = "";
	size_t dot = filename.find_last_of(".");
	if (dot != std::string::npos)
	{
		calibrationFileName = filename.substr(0, dot) + ".calibration";
	}

	// Try to load calibration
	ifstream in(calibrationFileName.c_str());
	if(!in.good())
	{
		ROS_WARN ("Could not open calibration file %s", calibrationFileName.c_str());
		return t;
	}
	float qx, qy, qz, qw, x, y, z;
	in >> qx >> qy >> qz >> qw >> x >> y >> z;

	ROS_INFO("Found calibration %s: %f %f %f %f %f %f %f", calibrationFileName.c_str(), qx, qy, qz, qw, x, y, z);

	tf::Vector3 origin(x, y, z);
	tf::Quaternion quat(qx, qy, qz, qw);
	t = tf::Transform(quat, origin);
	return t;
}

/*******************************************************************************
 * @brief 	Computes a marker for each region
 *
 * @param	regionPtr		A pointer to a vector of regions
 * @param	contour_markers	A vector to store the created markers
 ******************************************************************************/
void generateMarkerArray(vector<visualization_msgs::Marker>& contour_markers, vector<cRegion* >* regionPtr)
{
	for(size_t i = 0; i < regionPtr->size(); i++)
	{
		cRegion* r = regionPtr->at(i);
		vector<vector<cVertex> > contours = r->getContours(LINE_FUSION_THRESHOLD);
		if(contours.size() > 0)
		{
			// Setup new marker in map frame and lvr_contours namespace
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

			// Only store the shape of the outer contour that is always
			// stored in the first element of the contour vector
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
			ROS_WARN("Region %lu has no contour", i);
		}
	}
}

/*******************************************************************************
 * @brief 	Generates the sent ROS messages from the information stored in the
 * 			classifier and colors the markers for the extracted contours
 * 			according to their orientation class (horizontal = blue,
 * 			vertical = red and unknown = grey).
 *
 * @param	classifier		The used furniture classifier
 * @param	contour_markers	The pre-computed markers for all regions
 * @param	marker_array	The final marker array to be published
 * @param	patch_array		The message containing the classification info
 ******************************************************************************/
void generateMessages(
		FurnitureFeatureClassifier<cVertex , cNormal >& classifier,
		vector<visualization_msgs::Marker>& contour_markers,
		visualization_msgs::MarkerArray& marker_array,
		semantic_object_maps_msgs::PlanarPatchArray& patch_array)
{
	for(int i = 0; i < classifier.numFeatures(); i++)
		{
			// Setup a planar feature element with information from
			// the classifier
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

			// Get the marker for this region from the marker array and
			// set the color according to the orientation class
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
			marker_array.markers.push_back(m);
		}
}

int main(int argc, char** argv)
{
	// Init ros and create two publishers: One for the classification message
	// and one for the visualization markers
	ros::init(argc, argv, "lvr_classifier_node");
	ros::NodeHandle nh;

	ros::Publisher plane_publisher = nh.advertise<semantic_object_maps_msgs::PlanarPatchArray>("lvr_classified_planes", 1000);
	ros::Publisher contour_publisher = nh.advertise<visualization_msgs::MarkerArray>("lvr_plane_contours", 1000);

	// Load data from given file
	string modelFileName(argv[1]);
	ModelPtr model = ModelFactory::readModel(modelFileName);

	// Search for a calibration for the given model
	tf::Transform t = getTransfromFromCalibration(modelFileName);

	// Transform the model according to the loaded transformation
	transformModel(model, t.inverse());

	// Generate half edge mesh representation of the loaded mesh
	HalfEdgeMesh<cVertex, cNormal > mesh( model->m_mesh );

	// Get a pointer to he planar regions and create a furniture classifier
	vector<cRegion* >* regionPtr = mesh.getRegionsPtr();
	FurnitureFeatureClassifier<cVertex , cNormal > classifier(regionPtr);
	mesh.setClassifier(&classifier);

	// Apply mesh optimization filters
	mesh.cleanContours(CONTOUR_ITERATIONS);
	mesh.setDepth(100);

	mesh.optimizePlanes(
			PLANE_ITERATIONS,
			NORMAL_THRESHOLD,
			MIN_PLANE_SIZE,
			SMALL_REGION_THRESHOLD,
			true);

	mesh.fillHoles(FILL_HOLES);
	mesh.optimizePlaneIntersections();
	mesh.restorePlanes(MIN_PLANE_SIZE);

	// Create a marker for each contour (we to generate them here since mesh.finalize
	// will destroy the topology of the initial mesh when re-triangulating the
	// contours
	mesh.resetUsedFlags();
	vector<visualization_msgs::Marker> contourMarkers;
	generateMarkerArray(contourMarkers, regionPtr);

	// Finalize and reduce mesh, includes classification
	mesh.finalizeAndRetesselate(
			false, // Textures not yet supported in this tool
			LINE_FUSION_THRESHOLD);

	// Create output model and save to file
	ModelPtr out_model( new Model( mesh.meshBuffer() ) );
	ModelFactory::saveModel( out_model, "optimized_mesh.ply");

	// Generate final messages from classifier and pre-computed markers
	semantic_object_maps_msgs::PlanarPatchArray patch_array;
	visualization_msgs::MarkerArray markerArray;
	generateMessages(classifier, contourMarkers, markerArray, patch_array);

	// Publish the stuff
	plane_publisher.publish(patch_array);
	contour_publisher.publish(markerArray);
	ros::spin();

	return 0;
}
