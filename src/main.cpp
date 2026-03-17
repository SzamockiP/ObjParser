#include <print>
#include <string>
#include <filesystem>
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
		std::println("Output directory doesn't exist. Exiting...");
		return 0;
	}

	return 0;
}
