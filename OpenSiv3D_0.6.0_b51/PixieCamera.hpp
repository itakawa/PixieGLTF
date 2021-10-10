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
//# include <DirectXMath.h>

namespace s3d
{
	class alignas(16) PixieCamera
	{
	public:
		static constexpr double DefaultVerticalFOV = 30_deg;

		static constexpr double DefaultNearClip = 0.1;

		PixieCamera() = default;

		PixieCamera(const PixieCamera&) = default;

		explicit PixieCamera(const Rect& sceneRect,
							 const double verticalFOV = DefaultVerticalFOV,
							 const Vec3& eyePosition = Vec3{ 0, 4, -4 },
							 const Vec3& focusPosition = Vec3{ 0, 0, 0 },
							 const Vec3& upDirection = Vec3{ 0, 1, 0 },
							 const double nearClip = DefaultNearClip) noexcept
			: m_sceneRect(sceneRect)
			, m_verticalFOV(verticalFOV)
			, m_nearClip{ nearClip }
			, m_eyePosition(eyePosition)
			, m_focusPosition(focusPosition)
			, m_upDirection(upDirection)
		{
			updateProj();
			updateView();
			updateViewProj();
		}

		virtual ~PixieCamera() = default;

//		[[nodiscard]]
//		virtual Mat4x4 getMat4x4() const;

		void setSceneSize(const Size& sceneSize) noexcept;

		void setProjection(	const Rect& sceneRect, double verticalFOV,
							double nearClip = DefaultNearClip) noexcept
		{
			m_sceneRect	= sceneRect;
			m_verticalFOV = verticalFOV;
			m_nearClip	= nearClip;

			updateProj();
			updateViewProj();
		}

		void setView(const Vec3& eyePosition, const Vec3& focusPosition,
			         const Vec3& upDirection = Vec3{ 0, 1, 0 }) noexcept
		{
			m_eyePosition	= eyePosition;
			m_focusPosition	= focusPosition;
			m_upDirection	= upDirection;

			updateView();
			updateViewProj();
		}

		[[nodiscard]]
		const Size& getSceneSize() const noexcept
		{
			return m_sceneRect.size ;
		}

		[[nodiscard]]
		const RectF getSceneRectF() const noexcept
		{
			return RectF{ m_sceneRect.x,m_sceneRect.y,m_sceneRect.w,m_sceneRect.h };
		}

		[[nodiscard]]
		const Rect& getSceneRect() const noexcept
		{
			return m_sceneRect;
		}

		[[nodiscard]] void setSceneRect( Rect &screenrect ) noexcept
		{
			m_sceneRect = screenrect ;
		}

		[[nodiscard]] void setSceneSize( Size &screensize ) noexcept
		{
			m_sceneRect.size = screensize ;
		}

		[[nodiscard]]
		double getVerticlaFOV() const noexcept
		{
			return m_verticalFOV;
		}

		[[nodiscard]]
		double getNearClip() const noexcept
		{
			return m_nearClip;
		}

		[[nodiscard]]
		const Vec3& getEyePosition() const noexcept
		{
			return m_eyePosition;
		}

		[[nodiscard]]
		const Vec3& getFocusPosition() const noexcept
		{
			return m_focusPosition;
		}

		[[nodiscard]]
		const Vec3& getUpDirection() const noexcept
		{
			return m_upDirection;
		}

		[[nodiscard]]
		const Mat4x4& SIV3D_VECTOR_CALL getProj() const noexcept
		{
			return m_proj;
		}

		[[nodiscard]]
		const Mat4x4& SIV3D_VECTOR_CALL getView() const noexcept
		{
			return m_view;
		}


		[[nodiscard]]
		const Mat4x4& SIV3D_VECTOR_CALL getViewProj() const noexcept
		{
			return m_viewProj;
		}

		[[nodiscard]]
		const Mat4x4& SIV3D_VECTOR_CALL getInvViewProj() const noexcept
		{
			return m_invViewProj;
		}

		[[nodiscard]]
		Float2 worldToScreenPoint(const Float3& pos) const noexcept
		{
			Float3 v = SIMD_Float4{ DirectX::XMVector3TransformCoord(SIMD_Float4{ pos, 0.0f }, m_viewProj) }.xyz();
			v.x += 1.0f;
			v.y += 1.0f;
			v.x *= 0.5f * (float)m_sceneRect.w;
			v.y *= 0.5f;
			v.y = 1.0f - v.y;
			v.y *= (float)m_sceneRect.h;

			v.x += (float)m_sceneRect.x;
			v.y += (float)m_sceneRect.y;
			return v.xy();
		}

		[[nodiscard]]
		Float3 screenToWorldPoint(const Float2& pos, float depth) const noexcept
		{
			Vec3 v(pos, depth);
			v.x -= m_sceneRect.x;
			v.y -= m_sceneRect.y;
			v.x /= (m_sceneRect.w * 0.5f);
			v.y /= (m_sceneRect.h * 0.5f);
			v.x -= 1.0f;
			v.y -= 1.0f;
			v.y *= -1.0f;
			const SIMD_Float4 worldPos = DirectX::XMVector3TransformCoord(SIMD_Float4{ v, 0.0f }, m_invViewProj);
			return worldPos.xyz();
		}

		[[nodiscard]]
		Ray screenToRay(const Vec2& pos) const noexcept
		{
			const Vec3 rayEnd = screenToWorldPoint(pos, 0.000005f);
			return Ray(m_eyePosition, (rayEnd - m_eyePosition).normalized());
		}

	protected:

		Mat4x4 m_proj = Mat4x4::Identity();

		Mat4x4 m_view = Mat4x4::Identity();

		Mat4x4 m_viewProj = Mat4x4::Identity();

		Mat4x4 m_invViewProj = Mat4x4::Identity();

		//
		// Proj
		//
		Rect m_sceneRect = Rect{0,0,1920,1080};

		double m_verticalFOV = DefaultVerticalFOV;

		double m_nearClip = DefaultNearClip;

		//
		// View
		//
		Vec3 m_eyePosition = Vec3(0, 4, -4);

		Vec3 m_focusPosition = Vec3(0, 0, 0);

		Vec3 m_upDirection = Vec3(0, 1, 0);

		void updateProj() noexcept
		{

			const double fov			= (1.0 / std::tan(m_verticalFOV * 0.5));
			const double aspectRatio	= static_cast<double>(m_sceneRect.w) / m_sceneRect.h;
			constexpr float e = 0.000001f;

			m_proj = Mat4x4{
				static_cast<float>(fov / aspectRatio),      0.0f, 0.0f,                    0.0f,
				                   0.0f, static_cast<float>(fov), 0.0f,                    0.0f,
				                   0.0f, 0.0f,                       e,                    1.0f,
				                   0.0f, 0.0f, static_cast<float>(m_nearClip * (1.0 - e)), 0.0f
			};
		}
			

		void updateView() noexcept
		{
			const SIMD_Float4 eyePosition(m_eyePosition, 0.0f);
			const SIMD_Float4 focusPosition(m_focusPosition, 0.0f);
			const SIMD_Float4 upDirection(m_upDirection, 0.0f);
			m_view = DirectX::XMMatrixLookAtLH(eyePosition, focusPosition, upDirection);

		}


		void updateViewProj() noexcept
		{
			m_viewProj = m_view * m_proj;
			m_invViewProj = m_viewProj.inverse();
		}


	public:



		PixieCamera& operator=(const PixieCamera&) = default;

		PixieCamera(PixieCamera&&) = default;

		PixieCamera& operator=(PixieCamera&&) = default;


// from SivBasicCamera3D


		PixieCamera& setEyePosition( const Vec3& eyePosition ) noexcept
		{
			m_eyePosition = eyePosition;
			return *this;
		}

		PixieCamera& setFocusPosition( const Vec3& focusPosition ) noexcept
		{
			m_focusPosition = focusPosition;
			return *this;
		}

		PixieCamera& setUpDirection( const Vec3& upDirection ) noexcept
		{
			m_upDirection = upDirection;
			return *this;
		}

		void setView() noexcept
		{
			updateView();
			updateViewProj();
		}

		[[nodiscard]]
		const Vec3& getFwdDirection() const noexcept
		{
			return m_focusPosition - m_eyePosition;
		}
/*
		[[nodiscard]]
		Mat4x4 getMat4x4() const
		{
			return m_viewProj;
		}
*/


		void drawLine( Float3 begin, Float3 end,const double thickness, const ColorF& color) const
		{
			constexpr size_t vertexCount = 2;
			const Float3 vec[vertexCount] = { begin, end };
			Float3 out[vertexCount];

//			SIMD::Vector3TransformCoordStream(out, vec, vertexCount, m_viewProj);//4.3
			DirectX::XMVector3TransformCoordStream(
				reinterpret_cast<DirectX::XMFLOAT3*>(out),	     sizeof(Float3),
				reinterpret_cast<const DirectX::XMFLOAT3*>(vec), sizeof(Float3),
				vertexCount, m_viewProj);


			for (auto& v : out)
			{
				v.x += 1.0f;
				v.y += 1.0f;
				v.x *= 0.5f * (float)m_sceneRect.w;
				v.y *= 0.5f;
				v.y = 1.0f - v.y;
				v.y *= (float)m_sceneRect.h;
			}
//			LOG_INFO(U"pos:{}"_fmt(out[0]));
			Line(out[0].xy(), out[1].xy()).draw(thickness, color);
		}

		float getBasisSpeed()
		{
			Float3 distance = m_focusPosition - m_eyePosition;
			Float3 identity = distance.normalized();
			return distance.length()/identity.length() /1000;                    
		}

		bool dolly( float forwardback, bool movefocus = false )
		{
			Float3 dirB = (m_focusPosition - m_eyePosition).normalized();
			m_eyePosition += forwardback * dirB;
			if( movefocus ) m_focusPosition += forwardback * dirB;

			Float3 dirA = (m_focusPosition - m_eyePosition);
			return ( Math::Sign(dirA.x) != Math::Sign(dirB.x) &&
					 Math::Sign(dirA.y) != Math::Sign(dirB.y) &&
					 Math::Sign(dirA.z) != Math::Sign(dirB.z) );
		}

		void panX( float leftright )
		{
			Float3 vec = m_focusPosition - m_eyePosition;
			Float3 dir = vec.normalized() ;
		    Float3 right = dir.cross( m_upDirection ).normalized() ;
		    m_upDirection = dir.cross( right ).normalized() ;

			SIMD_Float4 qrot = DirectX::XMQuaternionRotationAxis( SIMD_Float4{ m_upDirection, 0 }, leftright ) ;
			SIMD_Float4 focus = DirectX::XMVector3Rotate( SIMD_Float4{ vec, 0 }, qrot) ;
			m_focusPosition = Float3{ focus.getX(), focus.getY(), focus.getZ() } + m_eyePosition.xyz();
		}

		void panY( float updown )
		{
			Float3 vec = m_focusPosition - m_eyePosition;
			Float3 dir = vec.normalized() ;
		    Float3 right = dir.cross( m_upDirection ).normalized() ;
		    m_upDirection = dir.cross( right ).normalized() ;

			SIMD_Float4 qrot = DirectX::XMQuaternionRotationAxis( SIMD_Float4{ right, 0 }, updown );
			SIMD_Float4 focus = DirectX::XMVector3Rotate( SIMD_Float4{ vec, 0 }, qrot) ;
			m_focusPosition = Float3{ focus.getX(), focus.getY(), focus.getZ() } + m_eyePosition.xyz();
		}

		void trackX( float leftright )
		{
			Float3 vec = m_focusPosition - m_eyePosition;
			Float3 dir = vec.normalized() ;
		    Float3 right = dir.cross( m_upDirection ).normalized() ;

			dir = leftright * right;
			m_eyePosition += dir;
			m_focusPosition += dir;
		}

		void craneY( float updown )
		{
			Float3 vec = m_focusPosition - m_eyePosition;
			Float3 dir = vec.normalized() ;
		    Float3 right = dir.cross( m_upDirection ) ;
		    m_upDirection = right.cross( dir ).normalized() ;

			dir = updown * m_upDirection;
			m_eyePosition += dir;
			m_focusPosition += dir;
		}

		void tilt( float updown )
		{
			Float3 vec = m_focusPosition - m_eyePosition;
			Float3 dir = vec.normalized() ;
		    Float3 right = dir.cross( m_upDirection ) ;
		    m_upDirection = right.cross( m_upDirection ).normalized() ;
			
			SIMD_Float4 qrot = DirectX::XMQuaternionRotationAxis( SIMD_Float4{ right, 0 }, updown );
			SIMD_Float4 focus = DirectX::XMVector3Rotate( SIMD_Float4{ vec, 0 }, qrot) ;
			m_focusPosition = Float3{ focus.getX(), focus.getY(), focus.getZ() } + m_eyePosition.xyz();
		}

		void arcballX( float leftright )
		{
			Float3 vec = m_eyePosition - m_focusPosition ;
			Float3 dir = (-vec).normalized() ;
			Float3 right = m_upDirection.cross( dir ) ;
			m_upDirection = dir.cross( right ).normalized() ;

			SIMD_Float4 qrot = DirectX::XMQuaternionRotationAxis( SIMD_Float4{ m_upDirection, 0 }, leftright );
			SIMD_Float4 eye = DirectX::XMVector3Rotate( SIMD_Float4{ vec, 0 }, qrot) ;
			m_eyePosition = Float3{ eye.getX(), eye.getY(), eye.getZ() } + m_focusPosition;
		}

		void arcballY( float updown )
		{
			Float3 vec = m_eyePosition - m_focusPosition ;
			Float3 dir = (-vec).normalized() ;
			Float3 right = m_upDirection.cross( dir ) ;
			m_upDirection = dir.cross( right ).normalized() ;
			
			SIMD_Float4 qrot = DirectX::XMQuaternionRotationAxis( SIMD_Float4{ right, 0 }, updown );
			SIMD_Float4 eye = DirectX::XMVector3Rotate( SIMD_Float4{ vec, 0 }, qrot) ;
			m_eyePosition = Float3{ eye.getX(), eye.getY(), eye.getZ() } + m_focusPosition;

		}

		Quaternion lookAt(const Vec3& src, const Vec3& dst, Vec3 up = { 0,Math::Inf,0 })
		{
			if (up.y == Math::Inf)					//上ベクトル省略時
			{
				Vec3 dir = (dst - src).normalized();	//曲げたい方向ベクトル
				Float3 right = dir.cross( Vec3 {0,1,0} ).normalized() ;//右ベクトルを計算
				up = dir.cross( right ).normalized() ;	//右ベクトルと方向ベクトルから上ベクトル(外積)
			}
			Mat4x4 mrot = DirectX::XMMatrixLookAtLH(SIMD_Float4{ src,0 },
													SIMD_Float4{ dst,0 },
													SIMD_Float4{ up,0 } );
			Float3 s, t;
			Quaternion r;
			mrot.decompose(s, r, t);
			return r;
		}
	};
}
