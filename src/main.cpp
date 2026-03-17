#include <print>
#include <string>
#include <filesystem>
#include <vector>
#include <unordered_map>
#include <iostream>

#include <CLI11.hpp>

#define TINYOBJLOADER_DISABLE_FAST_FLOAT
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

struct Texture
{
	int width = 0;
	int height = 0;
	int channels = 0;
	unsigned char* data = nullptr;

	~Texture()
	{
		if (data != nullptr)
		{
			stbi_image_free(data);
		}
	}

	Texture() = default;
	Texture(const Texture&) = delete;
	Texture& operator=(const Texture&) = delete;

	Texture(Texture&& other) noexcept : width(other.width), height(other.height), channels(other.channels), data(other.data)
	{
		other.data = nullptr;
	}
};
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


	std::println("Loading textures");
	std::unordered_map<std::string, Texture> texture_cache;

	std::filesystem::path base_dir = std::filesystem::path(input_obj_path).parent_path();
	int not_loaded = 0;
	int texture_count = 0;
	for (const auto& material : materials)
	{
		if (material.diffuse_texname.empty() || texture_cache.contains(material.diffuse_texname))
		{
			continue;
		}
		++texture_count;

		std::filesystem::path texture_path = base_dir / material.diffuse_texname;

		Texture texture;
		texture.data = stbi_load(texture_path.string().c_str(), &texture.width, &texture.height, &texture.channels, 4);
		texture.channels = 4;

		if (texture.data)
		{
			std::println("Loaded texture: {}", material.diffuse_texname);
			texture_cache.emplace(material.diffuse_texname, std::move(texture));
		}
		else
		{
			std::println(stderr, "Couldn't load texture: {}", texture_path.string());
			++not_loaded;
		}
	}

	if (not_loaded > 0)
	{
		std::println(stderr, "Couldn't load {}/{} textures.", not_loaded, texture_count);
		std::print("Do you wish to proceed? [y/N]: ");

		std::string answer;
		while (std::cin >> answer)
		{
			if (answer == "y" || answer == "Y")
			{
				std::println("Proceeding...");
				break;
			}
			else if (answer == "n" || answer == "N")
			{
				std::println("Exiting...");
				return 1;
			}
			else
			{
				std::print("Invalid input. Do you wish to proceed? [y/N]: ");
			}
		}
	}

	std::println("Loaded {} textures.", texture_cache.size());

	return 0;
}
