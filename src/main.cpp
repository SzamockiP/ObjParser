#include <print>
#include <string>
#include <filesystem>
#include <vector>
#include <unordered_map>

#include <CLI11.hpp>

#define TINYOBJLOADER_DISABLE_FAST_FLOAT
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

int main(int argc, char** argv)
{
	CLI::App app{ "ObjParser - .obj to morton code ordered voxel storage file" };

	std::string input_obj_path;
	std::string output_bin_path;

	std::uint32_t resolution = 1024;

	app.add_option("-i,--input", input_obj_path, ".obj input filepath")
		->required()
		->check(CLI::ExistingFile);

	app.add_option("-o,--output", output_bin_path, ".bin output filepath")
		->required();	

	app.add_option("-r,--resolution", resolution, "Voxel grid resolution (default: 1024)");

	CLI11_PARSE(app, argc, argv);


	std::filesystem::path out_path(output_bin_path);
	if (out_path.has_parent_path() && !std::filesystem::exists(out_path.parent_path()))
	{
		std::println(stderr, "Output directory doesn't exist.");
		return 1;
	}

	tinyobj::ObjReaderConfig obj_reader_config;
	obj_reader_config.mtl_search_path = std::filesystem::path(input_obj_path).parent_path().string();

	std::println("Loading model: {}", input_obj_path);
	tinyobj::ObjReader obj_reader;
	if (!obj_reader.ParseFromFile(input_obj_path, obj_reader_config))
	{
		if (!obj_reader.Error().empty())
		{
			std::println(stderr, "TinyObjReader error: {}.", obj_reader.Error());
			return 1;
		}
	}

	if (!obj_reader.Warning().empty())
	{
		std::println("TinyObjReader warning: {}", obj_reader.Warning());
	}

	auto& attrib = obj_reader.GetAttrib();
	auto& shapes = obj_reader.GetShapes();
	auto& materials = obj_reader.GetMaterials();

	std::println("Model loaded");
	std::println(" - Vertices: {}", attrib.vertices.size() / 3);
	std::println(" - Shapes: {}", shapes.size());
	std::println(" - Materials: {}", materials.size());



	

	return 0;
}
