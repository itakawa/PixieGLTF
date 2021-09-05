//-----------------------------------------------
//
//	This file is part of the Siv3D Engine.
//
//	Copyright (c) 2008-2019 Ryo Suzuki
//	Copyright (c) 2016-2019 OpenSiv3D Project
//
//	Licensed under the MIT License.
//
//-----------------------------------------------

# pragma once
# include <Siv3D.hpp>
//# include "../Siv3D/include/Siv3D/Fwd.hpp"
//# include "../Siv3D/include/Siv3D/PointVector.hpp"
//# include "../Siv3D/include/Siv3D/Quaternion.hpp"
# include "AABB.hpp"
//# include "../Siv3D/src/Siv3D/Common/Siv3DEngine.hpp"


namespace s3d
{
	struct OBB
	{
		Vec3 center;

		Vec3 size;

		Mat4x4 matrix;//追加

		Quaternion orientation;

		OBB() = default;

		OBB(const OBB&) = default;

		OBB& operator=(const OBB&) = default;

		OBB(OBB&&) = default;

		OBB& operator=(OBB&&) = default;

		const OBB(double _size) noexcept
			: center(0, 0, 0)
			, size(_size, _size, _size) {}

		const OBB(double sx, double sy, double sz) noexcept
			: center(0, 0, 0)
			, size(sx, sy, sz) {}

		const OBB(const Vec3& _size) noexcept
			: center(0, 0, 0)
			, size(_size) {}

		constexpr OBB(double _size, const Quaternion& _orientation) noexcept
			: center(0, 0, 0)
			, size(_size, _size, _size)
			, orientation(_orientation) {}

		constexpr OBB(double sx, double sy, double sz, const Quaternion& _orientation) noexcept
			: center(0, 0, 0)
			, size(sx, sy, sz)
			, orientation(_orientation) {}

		constexpr OBB(const Vec3& _size, const Quaternion& _orientation) noexcept
			: center(0, 0, 0)
			, size(_size)
			, orientation(_orientation) {}

		const OBB(double x, double y, double z, double _size) noexcept
			: center(x, y, z)
			, size(_size, _size, _size) {}

		const OBB(double x, double y, double z, double sx, double sy, double sz) noexcept
			: center(x, y, z)
			, size(sx, sy, sz) {}

		constexpr OBB(double x, double y, double z, double _size, const Quaternion& _orientation) noexcept
			: center(x, y, z)
			, size(_size, _size, _size)
			, orientation(_orientation) {}

		constexpr OBB(double x, double y, double z, double sx, double sy, double sz, const Quaternion& _orientation) noexcept
			: center(x, y, z)
			, size(sx, sy, sz)
			, orientation(_orientation) {}

		const OBB(const Vec3& _center, double _size) noexcept
			: center(_center)
			, size(_size, _size, _size) {}

		const OBB(const Vec3& _center, double sx, double sy, double sz) noexcept
			: center(_center)
			, size(sx, sy, sz) {}

		const OBB(const Vec3& _center, const Vec3& _size) noexcept
			: center(_center)
			, size(_size) {}

		constexpr OBB(const Vec3& _center, double _size, const Quaternion& _orientation) noexcept
			: center(_center)
			, size(_size, _size, _size)
			, orientation(_orientation) {}

		constexpr OBB(const Vec3& _center, double sx, double sy, double sz, const Quaternion& _orientation) noexcept
			: center(_center)
			, size(sx, sy, sz)
			, orientation(_orientation) {}

		constexpr OBB(const Vec3& _center, const Vec3& _size, const Quaternion& _orientation) noexcept
			: center(_center)
			, size(_size)
			, orientation(_orientation) {}

		const OBB(const AABB& aabb) noexcept
			: center(aabb.center)
			, size(aabb.size) {}

		constexpr OBB(const AABB& aabb, const Quaternion& _orientation) noexcept
			: center(aabb.center)
			, size(aabb.size)
			, orientation(_orientation) {}

		constexpr OBB& setPos(double x, double y, double z) noexcept
		{
			center.set(x, y, z);
			return *this;
		}

		constexpr OBB& setPos(const Vec3& pos) noexcept
		{
			center.set(pos);
			return *this;
		}

		constexpr OBB& setSize(double sx, double sy, double sz) noexcept
		{
			size.set(sx, sy, sz);
			return *this;
		}

		constexpr OBB& setSize(const Vec3& _size) noexcept
		{
			size.set(_size);
			return *this;
		}

		constexpr OBB& setOrientation(const Quaternion& _orientation) noexcept
		{
			orientation = _orientation;
			return *this;
		}

		constexpr OBB& setMatrix(const Mat4x4& _mat) noexcept
		{
			matrix = _mat;
			return *this;
		}

		constexpr OBB& set(double _size, const Quaternion& _orientation) noexcept
		{
			center.set(0, 0, 0);
			size.set(_size, _size, _size);
			orientation = Quaternion(_orientation);
			return *this;
		}

		constexpr OBB& set(double sx, double sy, double sz, const Quaternion& _orientation) noexcept
		{
			center.set(0, 0, 0);
			size.set(sx, sy, sz);
			orientation = Quaternion(_orientation);
			return *this;
		}

		constexpr OBB& set(const Vec3& _size, const Quaternion& _orientation) noexcept
		{
			center.set(0, 0, 0);
			size.set(_size);
			orientation = Quaternion(_orientation);
			return *this;
		}

		constexpr OBB& set(double x, double y, double z, double _size, const Quaternion& _orientation) noexcept
		{
			center.set(x, y, z);
			size.set(_size, _size, _size);
			orientation = Quaternion(_orientation);
			return *this;
		}

		constexpr OBB& set(double x, double y, double z, double sx, double sy, double sz, const Quaternion& _orientation) noexcept
		{
			center.set(x, y, z);
			size.set(sx, sy, sz);
			orientation = Quaternion(_orientation);
			return *this;
		}

		constexpr OBB& set(const Vec3& _center, double _size, const Quaternion& _orientation) noexcept
		{
			center.set(_center);
			size.set(_size, _size, _size);
			orientation = Quaternion(_orientation);
			return *this;
		}

		constexpr OBB& set(const Vec3& _center, double sx, double sy, double sz, const Quaternion& _orientation) noexcept
		{
			center.set(_center);
			size.set(sx, sy, sz);
			orientation = Quaternion(_orientation);
			return *this;
		}

		constexpr OBB& set(const Vec3& _center, const Vec3& _size, const Quaternion& _orientation) noexcept
		{
			center.set(_center);
			size.set(_size);
			orientation = Quaternion(_orientation);
			return *this;
		}

		constexpr OBB& set(const AABB& aabb, const Quaternion& _orientation) noexcept
		{
			center.set(aabb.center);
			size.set(aabb.size);
			orientation = Quaternion(_orientation);
			return *this;
		}

		[[nodiscard]] const OBB movedBy(double x, double y, double z) const noexcept
		{
			return{ center.movedBy(x, y, z), size };
		}

		[[nodiscard]] const OBB movedBy(const Vec3& v) const noexcept
		{
			return{ center.movedBy(v), size };
		}

		constexpr OBB& moveBy(double x, double y, double z) noexcept
		{
			center.moveBy(x, y, z);
			return *this;
		}

		constexpr OBB& moveBy(const Vec3& v) noexcept
		{
			center.moveBy(v);
			return *this;
		}

		[[nodiscard]] const OBB stretched(double xyz) const noexcept
		{
			return stretched({ xyz, xyz, xyz });
		}

		[[nodiscard]] const OBB stretched(double _x, double _y, double _z) const noexcept
		{
			return stretched({ _x, _y, _z });
		}

		[[nodiscard]] const OBB stretched(const Vec3& xyz) const noexcept
		{
			return{ center, size + xyz * 2 };
		}

		[[nodiscard]] const OBB scaled(double s) const noexcept
		{
			return scaled({ s, s, s });
		}

		[[nodiscard]] const OBB scaled(double sx, double sy, double sz) const noexcept
		{
			return scaled({ sx, sy, sz });
		}

		[[nodiscard]] const OBB scaled(const Vec3& s) const noexcept
		{
			return{ center, size * s };
		}

		void getCorners(DirectX::XMFLOAT3 corners[]) const
		{
			constexpr __m128 BoxOffsetHalf[8] =
			{
				{ -0.5f,  0.5f, -0.5f, 0.0f },
				{  0.5f,  0.5f, -0.5f, 0.0f },
				{ -0.5f, -0.5f, -0.5f, 0.0f },
				{  0.5f, -0.5f, -0.5f, 0.0f },
				{  0.5f,  0.5f,  0.5f, 0.0f },
				{ -0.5f,  0.5f,  0.5f, 0.0f },
				{  0.5f, -0.5f,  0.5f, 0.0f },
				{ -0.5f, -0.5f,  0.5f, 0.0f },
			};
			// Load the box
			__m128 vCenter = SIMD_Float4(center, 0.0f);
			__m128 vExtents = SIMD_Float4(size, 0.0f);
			__m128 vOrientation = orientation;

			//			assert(detail::QuaternionIsUnit(vOrientation));

			for (size_t i = 0; i < 8; ++i)
			{
				__m128 C = DirectX::XMVector4Transform(
							DirectX::XMVectorAdd(
								DirectX::XMVector3Rotate(
									DirectX::XMVectorMultiply(vExtents, BoxOffsetHalf[i]),
									vOrientation),
								vCenter),
						   matrix);

				DirectX::XMStoreFloat3(&corners[i], C);
			}
		}

		void draw(const Mat4x4& matVP, const ColorF& color = Palette::White) const
		{
			constexpr size_t vertexCount = 8;
			const Float2 resolution = Graphics2D::GetRenderTargetSize();
			const Float4 colorF = color.toFloat4();

			DirectX::XMFLOAT3 corners[vertexCount];
			getCorners(corners);

			Float3 out[vertexCount];
			//			SIMD::Vector3TransformCoordStream(out.data(), corners.data(), vertexCount, vp);
			DirectX::XMVector3TransformCoordStream(reinterpret_cast<DirectX::XMFLOAT3*>(out), sizeof(Float3),
													reinterpret_cast<DirectX::XMFLOAT3*>(corners), sizeof(Float3),
													vertexCount, matVP);
			for (auto& v : out)
			{
				v.x += 1.0f;
				v.y += 1.0f;
				v.x *= 0.5f * (float)resolution.x;
				v.y *= 0.5f;
				v.y = 1.0f - v.y;
				v.y *= (float)resolution.y;
			}

			std::array<Vertex2D, vertexCount> vertices;
			for (size_t i = 0; i < vertexCount; ++i)
			{
				auto& v = vertices[i];
				v.pos = out[i].xy();
				v.color = colorF;

			}
			constexpr std::array<uint16, 36> Indices =
			{
				0, 1, 2, 2, 1, 3,
				5, 4, 0, 0, 4, 1,
				1, 4, 3, 3, 4, 6,
				5, 0, 7, 7, 0, 2,
				4, 5, 6, 6, 5, 7,
				2, 3, 7, 7, 3, 6,
			};

			//			Siv3DEngine::Get<ISiv3DRenderer2D>()->addSprite(vertices.data(), vertices.size(), detail::Indices.data(), detail::Indices.size());

			for (size_t i = 0; i < Indices.size(); i += 3)
				Triangle(out[Indices[i + 0]].xy(), out[Indices[i + 1]].xy(), out[Indices[i + 2]].xy()).draw(color);
		}


		[[nodiscard]] const OrientedBox getBox() const noexcept
		{

			return OrientedBox{ center, size / 2 };
		}
	};
}

//////////////////////////////////////////////////
//
//	Format
//
//////////////////////////////////////////////////
/*
namespace s3d
{
	void Formatter(FormatData& formatData, const OBB& value);

	template <class CharType>
	inline std::basic_ostream<CharType>& operator <<(std::basic_ostream<CharType>& output, const OBB& value)
	{
		return output << CharType('(')
			<< value.center << CharType(',') << CharType(' ')
			<< value.size << CharType(',') << CharType(' ')
			<< value.orientation << CharType(')');
	}

	template <class CharType>
	inline std::basic_istream<CharType>& operator >>(std::basic_istream<CharType>& input, OBB& value)
	{
		CharType unused;
		return input >> unused
			>> value.center >> unused
			>> value.size >> unused
			>> value.orientation >> unused;
	}
}
*/
//////////////////////////////////////////////////
//
//	Hash
//
//////////////////////////////////////////////////

namespace std
{
	template <>
	struct hash<s3d::OBB>
	{
		[[nodiscard]] size_t operator ()(const s3d::OBB& value) const noexcept
		{
			return s3d::Hash::FNV1a(value);
		}
	};
}

//////////////////////////////////////////////////
//
//	fmt
//
//////////////////////////////////////////////////
/*
namespace fmt_s3d
{
	template <>
	struct formatter<s3d::OBB, s3d::char32>
	{
		s3d::String tag;

		template <class ParseContext>
		auto parse(ParseContext& ctx)
		{
			return s3d::detail::GetFmtTag(tag, ctx);
		}

		template <class Context>
		auto format(const s3d::OBB& value, Context& ctx)
		{
			const s3d::String fmt = s3d::detail::MakeFmtArg(
				U"({:", tag, U"}, {:", tag, U"}, {:", tag, U"})"
			);

			return format_to(ctx.begin(), wstring_view(fmt.data(), fmt.size()), value.center, value.size, value.orientation);
		}
	};
}
*/
