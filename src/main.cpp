#include <print>
#include <string>
#include <filesystem>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <limits>
#include <algorithm>
#include <queue>

#include <CLI11.hpp>

#define TINYOBJLOADER_DISABLE_FAST_FLOAT
#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

constexpr float SAT_EPSILON = 1e-5f;

struct Vec2
{
	float x, y;
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

	float dot(const Vec3& other) const { return x * other.x + y * other.y + z * other.z; }
};

inline bool axis_test(float p0, float p1, float p2, float box_radius)
{
	float proj_min = std::min({ p0, p1, p2 });
	float proj_max = std::max({ p0, p1, p2 });

	return !((proj_max < -box_radius - SAT_EPSILON) || (proj_min > box_radius + SAT_EPSILON));
}

bool check_voxel_triangle_intersect(const Vec3& box_center, const Vec3& box_half_size, const Vec3& tri_v0, const Vec3& tri_v1, const Vec3& tri_v2)
{
	Vec3 v0 = tri_v0 - box_center;
	Vec3 v1 = tri_v1 - box_center;
	Vec3 v2 = tri_v2 - box_center;

	Vec3 e0 = v1 - v0;
	Vec3 e1 = v2 - v1;
	Vec3 e2 = v0 - v2;

	if (std::min({ v0.x, v1.x, v2.x }) > box_half_size.x + SAT_EPSILON || std::max({ v0.x, v1.x, v2.x }) < -box_half_size.x - SAT_EPSILON) return false;
	if (std::min({ v0.y, v1.y, v2.y }) > box_half_size.y + SAT_EPSILON || std::max({ v0.y, v1.y, v2.y }) < -box_half_size.y - SAT_EPSILON) return false;
	if (std::min({ v0.z, v1.z, v2.z }) > box_half_size.z + SAT_EPSILON || std::max({ v0.z, v1.z, v2.z }) < -box_half_size.z - SAT_EPSILON) return false;

	float rad, p0, p1, p2;

	p0 = v0.z * e0.y - v0.y * e0.z;
	p2 = v2.z * e0.y - v2.y * e0.z;
	rad = box_half_size.y * std::abs(e0.z) + box_half_size.z * std::abs(e0.y);
	if (!axis_test(p0, p0, p2, rad)) return false;

	p0 = v0.z * e1.y - v0.y * e1.z;
	p1 = v1.z * e1.y - v1.y * e1.z;
	rad = box_half_size.y * std::abs(e1.z) + box_half_size.z * std::abs(e1.y);
	if (!axis_test(p0, p1, p1, rad)) return false;

	p0 = v0.z * e2.y - v0.y * e2.z;
	p1 = v1.z * e2.y - v1.y * e2.z;
	rad = box_half_size.y * std::abs(e2.z) + box_half_size.z * std::abs(e2.y);
	if (!axis_test(p0, p1, p0, rad)) return false;

	p0 = v0.x * e0.z - v0.z * e0.x;
	p2 = v2.x * e0.z - v2.z * e0.x;
	rad = box_half_size.x * std::abs(e0.z) + box_half_size.z * std::abs(e0.x);
	if (!axis_test(p0, p0, p2, rad)) return false;

	p0 = v0.x * e1.z - v0.z * e1.x;
	p1 = v1.x * e1.z - v1.z * e1.x;
	rad = box_half_size.x * std::abs(e1.z) + box_half_size.z * std::abs(e1.x);
	if (!axis_test(p0, p1, p1, rad)) return false;

	p0 = v0.x * e2.z - v0.z * e2.x;
	p1 = v1.x * e2.z - v1.z * e2.x;
	rad = box_half_size.x * std::abs(e2.z) + box_half_size.z * std::abs(e2.x);
	if (!axis_test(p0, p1, p0, rad)) return false;

	p0 = v0.y * e0.x - v0.x * e0.y;
	p2 = v2.y * e0.x - v2.x * e0.y;
	rad = box_half_size.x * std::abs(e0.y) + box_half_size.y * std::abs(e0.x);
	if (!axis_test(p0, p0, p2, rad)) return false;

	p0 = v0.y * e1.x - v0.x * e1.y;
	p1 = v1.y * e1.x - v1.x * e1.y;
	rad = box_half_size.x * std::abs(e1.y) + box_half_size.y * std::abs(e1.x);
	if (!axis_test(p0, p1, p1, rad)) return false;

	p0 = v0.y * e2.x - v0.x * e2.y;
	p1 = v1.y * e2.x - v1.x * e2.y;
	rad = box_half_size.x * std::abs(e2.y) + box_half_size.y * std::abs(e2.x);
	if (!axis_test(p0, p1, p0, rad)) return false;

	Vec3 normal = e0.cross(e1);
	float plane_dist = normal.x * v0.x + normal.y * v0.y + normal.z * v0.z;

	rad = box_half_size.x * std::abs(normal.x) + box_half_size.y * std::abs(normal.y) + box_half_size.z * std::abs(normal.z);

	if (std::abs(plane_dist) > rad + SAT_EPSILON) return false;

	return true;
}

inline void get_barycentric(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& p, float& u, float& v, float& w)
{
	Vec3 v0 = b - a, v1 = c - a, v2 = p - a;

	float d00 = v0.dot(v0);
	float d01 = v0.dot(v1);
	float d11 = v1.dot(v1);
	float d20 = v2.dot(v0);
	float d21 = v2.dot(v1);

	float denom = d00 * d11 - d01 * d01;

	if (std::abs(denom) < 1e-8f)
	{
		u = 1.0f; v = 0.0f; w = 0.0f;
		return;
	}

	v = (d11 * d20 - d01 * d21) / denom;
	w = (d00 * d21 - d01 * d20) / denom;
	u = 1.0f - v - w;
}

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
#pragma pack(push, 1)
struct VoxelData
{
	std::uint64_t morton;
	std::uint8_t r, g, b, a;

	bool operator<(const VoxelData& other) const
	{
		return morton < other.morton;
	}
};
#pragma pack(pop)

struct MergeItem
{
	VoxelData voxel;
	int stream_index;

	bool operator>(const MergeItem& other) const
	{
		return voxel.morton > other.voxel.morton;
	}
};

inline std::uint64_t split_by_3(std::uint64_t a)
{
	a &= 0x1ffffff;
	a = (a | a << 32) & 0x1f00000000ffff;
	a = (a | a << 16) & 0x1f0000ff0000ff;
	a = (a | a << 8) & 0x100f00f00f00f00f;
	a = (a | a << 4) & 0x10c30c30c30c30c3;
	a = (a | a << 2) & 0x1249249249249249;
	return a;
}

inline std::uint64_t encode_morton(std::uint32_t x, std::uint32_t y, std::uint32_t z)
{
	return split_by_3(x) | (split_by_3(y) << 1) | (split_by_3(z) << 2);
}

void flush_voxel_buffer(std::vector<VoxelData>& buffer, int& chunk_id, const std::filesystem::path& temp_dir)
{
	if (buffer.empty()) return;

	std::sort(buffer.begin(), buffer.end());

	std::string filename = std::format("temp_chunk_{}.bin", chunk_id++);
	std::filesystem::path full_chunk_path = temp_dir / filename;

	std::ofstream out_file(full_chunk_path, std::ios::binary);

	if (!out_file.is_open())
	{
		throw std::runtime_error(std::format("Failed to open temp file for writing: {}", full_chunk_path.string()));
	}

	out_file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size() * sizeof(VoxelData));
	out_file.close();
	buffer.clear();
}

void merge_and_deduplicate_chunks(int chunk_count, const std::filesystem::path& temp_dir, const std::filesystem::path& out_path)
{
	std::println("\nMerging {} temp files", chunk_count);

	std::vector<std::ifstream> streams;
	std::priority_queue<MergeItem, std::vector<MergeItem>, std::greater<MergeItem>> pq;

	for (int i = 0; i < chunk_count; ++i)
	{
		std::filesystem::path chunk_path = temp_dir / std::format("temp_chunk_{}.bin", i);
		streams.emplace_back(chunk_path, std::ios::binary);

		if (!streams.back().is_open())
		{
			throw std::runtime_error(std::format("Couldn't open file to merge: {}", chunk_path.string()));
		}

		VoxelData first_voxel;
		if (streams.back().read(reinterpret_cast<char*>(&first_voxel), sizeof(VoxelData)))
		{
			pq.push({ first_voxel, i });
		}
	}

	std::ofstream out_file(out_path, std::ios::binary);
	if (!out_file.is_open())
	{
		throw std::runtime_error(std::format("Couldn't make output file: {}", out_path.string()));
	}

	const size_t MERGE_BUFFER_SIZE = 10'000'000;
	std::vector<VoxelData> out_buffer;
	out_buffer.reserve(MERGE_BUFFER_SIZE);

	uint64_t last_morton = ~0ULL;
	size_t final_voxels = 0;

	while (!pq.empty())
	{
		MergeItem current = pq.top();
		pq.pop();

		if (current.voxel.morton != last_morton)
		{
			out_buffer.push_back(current.voxel);
			last_morton = current.voxel.morton;
			final_voxels++;

			if (out_buffer.size() >= MERGE_BUFFER_SIZE)
			{
				out_file.write(reinterpret_cast<const char*>(out_buffer.data()), out_buffer.size() * sizeof(VoxelData));
				out_buffer.clear();
			}
		}

		VoxelData next_voxel;
		if (streams[current.stream_index].read(reinterpret_cast<char*>(&next_voxel), sizeof(VoxelData)))
		{
			pq.push({ next_voxel, current.stream_index });
		}
	}

	if (!out_buffer.empty())
	{
		out_file.write(reinterpret_cast<const char*>(out_buffer.data()), out_buffer.size() * sizeof(VoxelData));
	}

	for (auto& s : streams)
	{
		s.close();
	}

	for (int i = 0; i < chunk_count; ++i)
	{
		std::filesystem::remove(temp_dir / std::format("temp_chunk_{}.bin", i));
	}

	if (std::filesystem::is_empty(temp_dir))
	{
		std::filesystem::remove(temp_dir);
	}

	std::println("Files merged\n - Size: {:.2f} MB\n - Saved: {} voxels",
		static_cast<double>(final_voxels * sizeof(VoxelData)) / (1024.0 * 1024.0), final_voxels);
}

int main(int argc, char** argv)
{
	CLI::App app{ "ObjParser - .obj to morton code ordered voxel storage file" };

	std::string input_obj_path;
	std::string output_bin_path = "output.bin";
	std::string custom_temp_dir = "";

	std::uint32_t resolution = 1024;

	app.add_option("-i,--input", input_obj_path, ".obj input filepath")
		->required()
		->check(CLI::ExistingFile);

	app.add_option("-o,--output", output_bin_path, ".bin output filepath")
		->required();

	app.add_option("-t,--temp", custom_temp_dir, "/temp folder directory path");

	app.add_option("-r,--resolution", resolution, "Voxel grid resolution (default: 1024)");

	CLI11_PARSE(app, argc, argv);

	if (!output_bin_path.ends_with(".bin"))
	{
		output_bin_path += ".bin";
	}

	std::filesystem::path out_path(output_bin_path);
	if (out_path.has_parent_path() && !std::filesystem::exists(out_path.parent_path()))
	{
		std::println(stderr, "Output directory doesn't exist.");
		return 1;
	}

	std::filesystem::path temp_dir;

	if (!custom_temp_dir.empty())
	{
		temp_dir = custom_temp_dir;

		if (!std::filesystem::is_directory(temp_dir))
		{
			std::println(stderr, "Error: Provided temp path does not exist or is not a directory.");
			return 1;
		}
	}
	else
	{
		temp_dir = out_path.has_parent_path() ? out_path.parent_path() / "temp" : std::filesystem::path("temp");

		if (!std::filesystem::exists(temp_dir))
		{
			std::filesystem::create_directories(temp_dir);
		}
	}

	std::println("Temp directory set to: {}", temp_dir.string());

	tinyobj::ObjReaderConfig obj_reader_config;
	obj_reader_config.mtl_search_path = std::filesystem::path(input_obj_path).parent_path().string();

	std::println("\nLoading model: {}", input_obj_path);
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


	std::println("\nLoading textures...");
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
		std::println(stderr, "Couldn't load {} / {} textures.", not_loaded, texture_count);
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

	std::println("\nCalculating model bounding box.");
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
	std::println(" - aabb min: ({:.3f}, {:.3f}, {:.3f})", model_aabb.min.x, model_aabb.min.y, model_aabb.min.z);
	std::println(" - aabb max: ({:.3f}, {:.3f}, {:.3f})", model_aabb.max.x, model_aabb.max.y, model_aabb.max.z);

	Vec3 size = {
		model_aabb.max.x - model_aabb.min.x,
		model_aabb.max.y - model_aabb.min.y,
		model_aabb.max.z - model_aabb.min.z
	};

	float max_dimension = std::max({ size.x, size.y, size.z });
	float voxel_size = max_dimension / static_cast<float>(resolution);

	std::println(" - Max model size: {:.3f}", max_dimension);
	std::println(" - Voxel size: {:.3f}", voxel_size);

	std::println("\nEstimating size");
	double total_area = 0.0;
	std::uint64_t estimated_voxels = 0;
	const float voxel_face_area = voxel_size * voxel_size;

	for (const auto& shape : shapes)
	{
		std::size_t index_offset = 0;
		for (const auto face_vert_num : shape.mesh.num_face_vertices)
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

	double temp_disk_gb = (estimated_voxels * 16) / (1024.0 * 1024.0);

	std::println(" - Voxels: {}", estimated_voxels);
	std::println(" - Temp files: {:.3f} MB", temp_disk_gb);


	std::println("\nDo you wish to proceed? [y/N]: ");

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


	std::println("Starting voxelization.");

	float inv_voxel_size = 1.0f / voxel_size;
	Vec3 box_half_size = { voxel_size * 0.5f, voxel_size * 0.5f, voxel_size * 0.5f };

	auto world_to_voxel = [&](float pos, float global_min) -> int
		{
			int idx = static_cast<int>(std::floor((pos - global_min) * inv_voxel_size));
			return std::clamp(idx, 0, static_cast<int>(resolution) - 1);
		};

	std::size_t processed_triangles = 0;
	std::size_t generated_voxels = 0;
	std::size_t total_shapes = shapes.size();

	const size_t MAX_BUFFER_SIZE = 10'000'000;
	std::vector<VoxelData> voxel_buffer;
	voxel_buffer.reserve(MAX_BUFFER_SIZE);
	int chunk_counter = 0;
	try
	{
		for (size_t s = 0; s < total_shapes; s++)
		{
			const auto& shape = shapes[s];

			std::print("\rProcessing shape: {} / {} ({:.2f}%)...",
				s + 1,
				total_shapes,
				(static_cast<double>(s) * 100) / total_shapes);

			std::size_t index_offset = 0;
			for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++)
			{
				const auto face_vert_num = shape.mesh.num_face_vertices[f];

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

				Vec2 uv0 = { 0.0f, 0.0f }, uv1 = { 0.0f, 0.0f }, uv2 = { 0.0f, 0.0f };
				if (!attrib.texcoords.empty() &&
					idx0.texcoord_index >= 0 &&
					idx1.texcoord_index >= 0 &&
					idx2.texcoord_index >= 0)
				{
					uv0 = { attrib.texcoords[2 * idx0.texcoord_index], attrib.texcoords[2 * idx0.texcoord_index + 1] };
					uv1 = { attrib.texcoords[2 * idx1.texcoord_index], attrib.texcoords[2 * idx1.texcoord_index + 1] };
					uv2 = { attrib.texcoords[2 * idx2.texcoord_index], attrib.texcoords[2 * idx2.texcoord_index + 1] };
				}

				int material_id = -1;
				if (f < shape.mesh.material_ids.size())
				{
					material_id = shape.mesh.material_ids[f];
				}

				AABB tri_aabb;
				tri_aabb.expand(v0);
				tri_aabb.expand(v1);
				tri_aabb.expand(v2);

				int min_voxel_x = world_to_voxel(tri_aabb.min.x, model_aabb.min.x);
				int max_voxel_x = world_to_voxel(tri_aabb.max.x, model_aabb.min.x);
				int min_voxel_y = world_to_voxel(tri_aabb.min.y, model_aabb.min.y);
				int max_voxel_y = world_to_voxel(tri_aabb.max.y, model_aabb.min.y);
				int min_voxel_z = world_to_voxel(tri_aabb.min.z, model_aabb.min.z);
				int max_voxel_z = world_to_voxel(tri_aabb.max.z, model_aabb.min.z);


				// for every voxel in aabb of triangle
				for (int z = min_voxel_z; z <= max_voxel_z; ++z)
				{
					for (int y = min_voxel_y; y <= max_voxel_y; ++y)
					{
						for (int x = min_voxel_x; x <= max_voxel_x; ++x)
						{
							Vec3 box_center = {
								model_aabb.min.x + (x + 0.5f) * voxel_size,
								model_aabb.min.y + (y + 0.5f) * voxel_size,
								model_aabb.min.z + (z + 0.5f) * voxel_size
							};

							if (check_voxel_triangle_intersect(box_center, box_half_size, v0, v1, v2))
							{
								generated_voxels++;

								float u, v, w;
								get_barycentric(v0, v1, v2, box_center, u, v, w);

								u = std::clamp(u, 0.0f, 1.0f);
								v = std::clamp(v, 0.0f, 1.0f);
								w = std::clamp(w, 0.0f, 1.0f);
								float sum = u + v + w;
								u /= sum; v /= sum; w /= sum;

								float final_u = u * uv0.x + v * uv1.x + w * uv2.x;
								float final_v = u * uv0.y + v * uv1.y + w * uv2.y;

								// default color white
								std::uint8_t color_r = 255, color_g = 255, color_b = 255, color_a = 255;

								if (material_id >= 0 && material_id < materials.size())
								{
									std::string tex_name = materials[material_id].diffuse_texname;

									if (!tex_name.empty() && texture_cache.contains(tex_name))
									{
										auto& tex = texture_cache[tex_name];

										int tex_x = std::clamp(static_cast<int>(final_u * tex.width), 0, tex.width - 1);
										int tex_y = std::clamp(static_cast<int>((1.0f - final_v) * tex.height), 0, tex.height - 1);

										int pixel_index = (tex_y * tex.width + tex_x) * 4;
										color_r = tex.data[pixel_index + 0];
										color_g = tex.data[pixel_index + 1];
										color_b = tex.data[pixel_index + 2];
										color_a = tex.data[pixel_index + 3];
									}
								}

								uint64_t morton_code = encode_morton(x, y, z);

								voxel_buffer.push_back({
									morton_code,
									color_r, color_g, color_b, color_a
									});

								if (voxel_buffer.size() >= MAX_BUFFER_SIZE)
								{
									std::print("\rSaving temp file: {} ...                              ", chunk_counter);
									flush_voxel_buffer(voxel_buffer, chunk_counter, temp_dir);
									std::print("\rSaved temp file: {}                                   ", chunk_counter-1);
									std::print("\nProcessing shape: {} / {} ({:.2f}%)...", s + 1, total_shapes, (static_cast<double>(s) * 100.0) / total_shapes);
								}
							}
						}
					}
				}
				index_offset += 3;
				processed_triangles++;
			}
		}
		if (!voxel_buffer.empty())
		{
			std::print("\rSaving temp file: {} ...                              ", chunk_counter);
			flush_voxel_buffer(voxel_buffer, chunk_counter, temp_dir);
			std::println("\rSaved temp file: {}                                   ", chunk_counter - 1);
		}

		if (chunk_counter > 0)
		{

			std::println("\nFinishing");
			std::println(" - Processed: {} triangles", processed_triangles);
			std::println(" - Generated: {} voxels", generated_voxels);
			merge_and_deduplicate_chunks(chunk_counter, temp_dir, out_path);
		}
		else
		{
			std::println("No voxels generated. Output file hasn't been created.");
		}
	}
	catch (const std::exception& e)
	{
		std::println(stderr, "\nFATAL ERROR: {}", e.what());
		return 1;
	}
	
	return 0;
}
