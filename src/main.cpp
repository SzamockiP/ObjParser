#include <print>
#include <string>
#include <filesystem>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <limits>
#include <algorithm>

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

struct Vec3
{
	float x, y, z;

	Vec3 operator-(const Vec3& other) const
	{
		return { x - other.x, y - other.y, z - other.z };
	}

	Vec3 cross(const Vec3& other) const
	{
		return {
			y * other.z - z * other.y,
			z * other.x - x * other.z,
			x * other.y - y * other.x
		};
	}

	float length() const
	{
		return std::sqrt(x * x + y * y + z * z);
	}
};

struct AABB
{
	Vec3 min, max;

	AABB()
	{
		min = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
		max = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };
	}

	void expand(const Vec3& point)
	{
		min.x = std::min(min.x, point.x);
		min.y = std::min(min.y, point.y);
		min.z = std::min(min.z, point.z);

		max.x = std::max(max.x, point.x);
		max.y = std::max(max.y, point.y);
		max.z = std::max(max.z, point.z);
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

	std::println("Model loaded.");
	std::println(" - Vertices: {}", attrib.vertices.size() / 3);
	std::println(" - Shapes: {}", shapes.size());
	std::println(" - Materials: {}", materials.size());


	std::println("Loading textures...");
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
	
	std::println("Calculating model bounding box.");
	AABB model_aabb;

	for (std::size_t i = 0; i < attrib.vertices.size(); i += 3)
	{
		Vec3 v = {
			attrib.vertices[i],
			attrib.vertices[i + 1],
			attrib.vertices[i + 2],
		};

		model_aabb.expand(v);
	}
	std::println("Model aabb min: ({:.3f}, {:.3f}, {:.3f})", model_aabb.min.x, model_aabb.min.y, model_aabb.min.z);
	std::println("Model aabb max: ({:.3f}, {:.3f}, {:.3f})", model_aabb.max.x, model_aabb.max.y, model_aabb.max.z);

	Vec3 size = {
		model_aabb.max.x - model_aabb.min.x,
		model_aabb.max.y - model_aabb.min.y,
		model_aabb.max.z - model_aabb.min.z
	};

	float max_dimension = std::max({ size.x, size.y, size.z });
	float voxel_size = max_dimension / static_cast<float>(resolution);

	std::println("Max model size: {:.3f}", max_dimension);
	std::println("Voxel size: {:.3f}", voxel_size);

	std::println("\nEstimating size");
	double total_area = 0.0;
	std::uint64_t estimated_voxels = 0;
	const float voxel_face_area = voxel_size * voxel_size;

	for (const auto& shape : shapes)
	{
		std::size_t index_offset = 0;
		for(const auto face_vert_num : shape.mesh.num_face_vertices)
		{
			if (face_vert_num != 3)
			{
				index_offset += face_vert_num;
				continue;
			}

			auto idx0 = shape.mesh.indices[index_offset];
			auto idx1 = shape.mesh.indices[index_offset + 1];
			auto idx2 = shape.mesh.indices[index_offset + 2];

			Vec3 v0 = { attrib.vertices[3 * idx0.vertex_index], attrib.vertices[3 * idx0.vertex_index + 1], attrib.vertices[3 * idx0.vertex_index + 2] };
			Vec3 v1 = { attrib.vertices[3 * idx1.vertex_index], attrib.vertices[3 * idx1.vertex_index + 1], attrib.vertices[3 * idx1.vertex_index + 2] };
			Vec3 v2 = { attrib.vertices[3 * idx2.vertex_index], attrib.vertices[3 * idx2.vertex_index + 1], attrib.vertices[3 * idx2.vertex_index + 2] };

			Vec3 e1 = v1 - v0;
			Vec3 e2 = v2 - v0;

			float area = 0.5f * e1.cross(e2).length();
			total_area += area;

			// 1.5f is heuristic
			float voxels_for_tri = (area / voxel_face_area) * 1.5f;
			estimated_voxels += std::max(1ull, static_cast<std::uint64_t>(std::ceil(voxels_for_tri)));

			index_offset += 3;
		}
	}

	double temp_disk_gb = (estimated_voxels * 16) / (1024.0 * 1024.0 * 1024.0);
	double final_disk_gb = (estimated_voxels * 12) / (1024.0 * 1024.0 * 1024.0);

	std::println(" - Voxels: {}", estimated_voxels);
	std::println(" - Temp files: {:.3f} GB", temp_disk_gb);
	std::println(" - Final file: {:.3f} GB", final_disk_gb);

	if (not_loaded > 0)
	{
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

	return 0;
}
