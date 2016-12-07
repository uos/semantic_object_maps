#include <lvr/io/ModelFactory.hpp>
#include <lvr/geometry/HalfEdgeMesh.hpp>
#include <lvr/classification/FurnitureFeatureClassifier.hpp>

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
	for(int i = 0; i < classifier.numFeatures(); i++)
	{
		PlanarClusterFeature pf = classifier.getFeature(i);
		std::cout << "Index: " << pf.index << std::endl;
		std::cout << "Centroid: " << pf.cx << " " << pf.cy << " " << pf.cz << std::endl;
		std::cout << "W/H/D: " << pf.w << " " << pf.h << " " << pf.d << std::endl;
		std::cout << "Normal: " << pf.nx << " " << pf.ny << " " << pf.nz << std::endl;
		std::cout << "Area: " << pf.area << std::endl;
		switch(pf.orientation)
		{
		case HORIZONTAL:
			std::cout << "Orientation: Horizontal" << std::endl;
			break;
		case VERTICAL:
			std::cout << "Orientation: Vertical" << std::endl;
			break;
		default:
			std::cout << "Orientation: Undefined" << std::endl;
		}


		std::cout << std::endl;
	}

	return 0;
}
