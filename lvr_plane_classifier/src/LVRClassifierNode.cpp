#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

#include <lvr/io/ModelFactory.hpp>
#include <lvr/geometry/HalfEdgeMesh.hpp>
#include <lvr/classification/FurnitureFeatureClassifier.hpp>
#include <semantic_object_maps_msgs/PlanarPatch.h>
#include <semantic_object_maps_msgs/PlanarPatchArray.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/LinearMath/Transform.h>
<<<<<<< HEAD
=======
#include <mesh_msgs/TriangleMesh.h>
#include <mesh_msgs/TriggerMesh.h>
#include <std_msgs/String.h>
#include <lvr/io/DataStruct.hpp>
#include <lvr/io/MeshBuffer.hpp>
#include <lvr/io/Model.hpp>
>>>>>>> master

using namespace lvr;

int 	PLANE_ITERATIONS 		= 3;
float 	NORMAL_THRESHOLD 		= 0.95;
int 	SMALL_REGION_THRESHOLD 	= 0;
int 	MIN_PLANE_SIZE 			= 7;
int		FILL_HOLES 				= 10;
float 	LINE_FUSION_THRESHOLD 	= 0.00999999978;
int		CONTOUR_ITERATIONS 		= 0;

typedef ColorVertex<float, unsigned char>  cVertex;
typedef Normal<float> cNormal;
typedef Region<cVertex, cNormal> cRegion;

void getParams(string filename)
{
	using boost::property_tree::ptree;
	ptree pt;
	read_xml(filename, pt);

	ptree sub = pt.get_child("config");
	NORMAL_THRESHOLD 		= sub.get<float>("normal_threshold", 0.95);
	PLANE_ITERATIONS 		= sub.get<int>("plane_iterations", 3);
	SMALL_REGION_THRESHOLD	= sub.get<int>("small_region_limit", 0);
	MIN_PLANE_SIZE 			= sub.get<int>("min_plane_size", 7);
	FILL_HOLES 				= sub.get<int>("fill_holes", 10);
	LINE_FUSION_THRESHOLD 	= sub.get<float>("line_fusion", 0.009999);
	CONTOUR_ITERATIONS 		= sub.get<int>("contour_iterations", 0);

	ROS_INFO("Found parameter file %s: "
			"\n Normal Threshold: %f"
			"\n Plane Iterations: %d"
			"\n Small Regions: %d"
			"\n Min Planes: %d"
			"\n Fill Holes: %d"
			"\n Line Fusion %f"
			"\n Contour Iterations: %d", filename.c_str(),
			NORMAL_THRESHOLD,
			PLANE_ITERATIONS,
			SMALL_REGION_THRESHOLD,
			MIN_PLANE_SIZE, FILL_HOLES,
			LINE_FUSION_THRESHOLD,
			CONTOUR_ITERATIONS);
}


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
<<<<<<< HEAD
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

=======
tf::Transform getTransfromFromCalibration()
{
	tf::Transform t;

	float qx, qy, qz, qw, x, y, z;
	x = 0.186;
	y = 0.8;
	z = -0.64;
	qx = 1.25;
	qy = -0.4;
	qz = 0.6;
	qw = 1;
>>>>>>> master
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
			contour_marker.scale.y = 0.01;
			contour_marker.scale.z = 0.01;
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
 * @brief 	Save the model in STL-format to /tmp and stores this as ressource
 * 			in a mesh marker.
 *
 * @param	filename		The filename of the model
 * @param	contour_markers	A pointer to model data
 * @param	marker			A mesh marker
 ******************************************************************************/
<<<<<<< HEAD
void createMeshMarker(string filename, ModelPtr model, visualization_msgs::Marker& mesh_marker)
{
	// Construct name for calibration file
	std::string stlFileName = "";
	size_t dot = filename.find_last_of(".");
	if (dot != std::string::npos)
	{
		stlFileName = "/tmp/" + filename.substr(0, dot) + ".stl";
	}
=======
void createMeshMarker(ModelPtr model, visualization_msgs::Marker& mesh_marker)
{
	// Construct name for calibration file
	std::string stlFileName = "meshFile";
	stlFileName = "/tmp/meshFile1.stl";
>>>>>>> master

	string stlRessource = "file://" + stlFileName;

	ModelFactory::saveModel(model, stlFileName);
	mesh_marker.header.frame_id = "map";
	mesh_marker.ns = "lvr_mesh";
	mesh_marker.id = 1;
	mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	mesh_marker.action = 0;
	mesh_marker.color.a = 0.5;
	mesh_marker.color.r = 0.0;
	mesh_marker.color.g = 1.0;
	mesh_marker.color.b = 0.0;
	mesh_marker.scale.x = 1.0;
	mesh_marker.scale.y = 1.0;
	mesh_marker.scale.z = 1.0;
	mesh_marker.pose.orientation.x = 0;
	mesh_marker.pose.orientation.y = 0;
	mesh_marker.pose.orientation.z = 0;
	mesh_marker.pose.orientation.w = 1;
	mesh_marker.mesh_resource= stlRessource;
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

			patch.header.frame_id = "map";
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
	// Loads parameters fromconfig file
	getParams("config.xml");

	// Init ros and create two publishers: One for the classification message
	// and one for the visualization markers
	ros::init(argc, argv, "lvr_classifier_node");
	ros::NodeHandle nh;
<<<<<<< HEAD

	ros::Publisher plane_publisher = nh.advertise<semantic_object_maps_msgs::PlanarPatchArray>("lvr_classified_planes", 1000);
	ros::Publisher contour_publisher = nh.advertise<visualization_msgs::MarkerArray>("lvr_plane_contours", 1000);
	ros::Publisher mesh_publisher = nh.advertise<visualization_msgs::Marker>("lvr_mesh", 1000);

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

	// Create a mesh marker
	visualization_msgs::Marker mesh_marker;
	createMeshMarker(modelFileName, model, mesh_marker);

	// Publish stuff
	plane_publisher.publish(patch_array);
	contour_publisher.publish(markerArray);
	mesh_publisher.publish(mesh_marker);
	ros::spin();

	return 0;
}
=======
	ros::Rate loop_rate(0.2);
	ros::Publisher plane_publisher = nh.advertise<semantic_object_maps_msgs::PlanarPatchArray>("lvr_classified_planes", 1000);
	ros::Publisher contour_publisher = nh.advertise<visualization_msgs::MarkerArray>("lvr_plane_contours", 1000);
	ros::Publisher mesh_publisher = nh.advertise<visualization_msgs::Marker>("lvr_mesh", 1000);
	ros::ServiceClient service = nh.serviceClient<mesh_msgs::TriggerMesh>("/kinfu/mesh_srv");
	while(ros::ok())	
	{
		int vertexSize = 0;	
		mesh_msgs::TriggerMesh srv;
		srv.request.request = mesh_msgs::TriggerMesh::Request::GET;
		MeshBufferPtr mBuffer(new MeshBuffer);
		bool selvic = service.exists();
		ROS_INFO("Checking if service exists... : %d",selvic);
		if(service.call(srv))
		{
			uintArr faces(new uint[srv.response.mesh.mesh.triangles.size() * 3]);
			floatArr vertices(new float[srv.response.mesh.mesh.vertices.size() * 3]);
			int facePosition = 0;
			int vertexPosition = 0;
			int faceSize = srv.response.mesh.mesh.triangles.size();
			vertexSize = srv.response.mesh.mesh.vertices.size();
			for(int i = 0; i < faceSize; i++ )
			{
				faces[facePosition++] = (uint) srv.response.mesh.mesh.triangles[i].vertex_indices[0];
				faces[facePosition++] = srv.response.mesh.mesh.triangles[i].vertex_indices[1];
				faces[facePosition++] = srv.response.mesh.mesh.triangles[i].vertex_indices[2];
			}
			for(int i = 0; i < vertexSize; i++ )
			{
				vertices[vertexPosition++] = srv.response.mesh.mesh.vertices[i].x;
				vertices[vertexPosition++] = srv.response.mesh.mesh.vertices[i].y;
				vertices[vertexPosition++] = srv.response.mesh.mesh.vertices[i].z;
			}
			
			mBuffer->setFaceArray(faces,(size_t) faceSize);
			mBuffer->setVertexArray(vertices, (size_t) vertexSize);
		}else
		{
			ROS_ERROR("NO MESH RECEIVED!");
			return 1;
		}

		ModelPtr model(new Model(mBuffer));
		// Search for a calibration for the given model
		tf::Transform t = getTransfromFromCalibration();

		// Transform the model according to the loaded transformation
		transformModel(model, t.inverse());
		// Generate half edge mesh representation of the loaded mesh
		HalfEdgeMesh<cVertex, cNormal> mesh( model->m_mesh );
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

		// Create a mesh marker
		visualization_msgs::Marker mesh_marker;
		createMeshMarker(model, mesh_marker);
	
		// Publish stuff
		plane_publisher.publish(patch_array);
		contour_publisher.publish(markerArray);
		mesh_publisher.publish(mesh_marker);
		ros::spin();
		//loop_rate.sleep();
		//Maybe-TODO: Broadcast idle
	}
	return 0;
}

>>>>>>> master
