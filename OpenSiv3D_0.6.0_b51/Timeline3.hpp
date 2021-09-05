#pragma once
# include <Siv3D.hpp>
# include <Siv3d/Easing.hpp>
# include <ThirdParty/Catch2/catch.hpp>
# include <typeinfo>

template <typename V> String GetTypeName( V var ) { return Unicode::WidenAscii(typeid(var).name()); }

//Array<uint32>を強引に指定型にキャスト
template <typename V> void *toVoid( V value ) { return  (void *)&value; }
template <typename V> uint32 toU32( V value ) { return  *(uint32 *)&value[0]; }
template <typename V>  int32 toI32( V value ) { return  *(int32 *)&value[0]; }
template <typename V> uint16 toU16( V value ) { return  *(uint16 *)&value[0]; }
template <typename V>  int16 toI16( V value ) { return  *(int16 *)&value[0]; }
template <typename V> uint8 toU8( V value ) { return  *(uint8 *)&value[0]; }
template <typename V>  int8 toI8( V value ) { return  *(int8 *)&value[0]; }
template <typename V>  bool toBool( V value ) { return  *(bool *)&value[0]; }
template <typename V> float toFloat( V value ) { return  *(float *)&value[0]; }
template <typename V> double toDouble( V value ) { return  (double)*(float *)&value[0]; }
template <typename V> Float2 toFloat2( V value ) { return  std::move( Float2{*(float *)&value[0],*(float *)&value[1]} ); }
template <typename V> Float3 toFloat3( V value ) { return  std::move( Float3{*(float *)&value[0],*(float *)&value[1],*(float *)&value[2]}); }
template <typename V> Float4 toFloat4( V value ) { return  std::move( Float4{*(float *)&value[0],*(float *)&value[1],*(float *)&value[2],*(float *)&value[3]}); }
template <typename V> Vec2 toVec2( V value ) { return  std::move(Vec2{*(float *)&value[0],*(float *)&value[1]}); }
template <typename V> Vec3 toVec3( V value ) { return  std::move(Vec3{*(float *)&value[0],*(float *)&value[1],*(float *)&value[2]}); }
template <typename V> Vec4 toVec4( V value ) { return  std::move(Vec4{*(float *)&value[0],*(float *)&value[1],*(float *)&value[2],*(float *)&value[3]}); }



Shape2D SpeechBaloon( const Float2& center,						// 中心座標
						const uint16& ngon = 6,                 // 基準図形の種類
						const  int32& angle = 0,                // 基準図形の角度
						const uint32& size = 150,               // 基準図形の大きさ
						const  int32& rotate = 0,               // 雲の回転(アフィン変換)
						const uint32& thickness = 10,           // 吹き出しの太さ 
						const Float2& scale = Float2{ 1,1 },    // 雲の拡縮(アフィン変換)
						const Float2& stretch = Float2{ 1,1 },  // 引伸ばし倍率
						const Float2& divide = Float2{ 0,0 },   // 分割点座標(中心座標からの相対位置)
						const Float2& target = Float2{ 0,0 })   // 吹出し目標点の座標(中心座標からの相対位置)
{
	Float2 div = center + divide;   // 原点から相対指定
	Float2 trg = center + target;
	Float2 str = stretch;
//	int32 ang = angle+45;

	if (str.x < 1) str.x = 1;       // 引伸しは縮小に対応しない
	if (str.y < 1) str.y = 1;

	//基準多角形
	Shape2D basis = Shape2D::Ngon(ngon,size,center,ToRadians(angle+45));
	Array<Float2> basis_v = basis.vertices();
	Array<s3d::TriangleIndex> basis_i = basis.indices();

	//基準引伸し
	uint32 isStretch = (str == Float2(1, 1)) ? 0 : 1;
	if (isStretch)
	{
		Float2 stretched = str / Float2(1, 1);
		Float2 resized = size * stretched - Float2(size, size);  //引き伸ばし後サイズの差分
		for (auto& vt : basis_v)
		{
			if (vt.x < div.x && vt.y < div.y) { vt.x = vt.x - resized.x;vt.y = vt.y - resized.y; }
			else if (vt.x >= div.x && vt.y < div.y) { vt.x = vt.x + resized.x;vt.y = vt.y - resized.y; }
			else if (vt.x < div.x && vt.y >= div.y) { vt.x = vt.x - resized.x;vt.y = vt.y + resized.y; }
			else if (vt.x >= div.x && vt.y >= div.y) { vt.x = vt.x + resized.x;vt.y = vt.y + resized.y; }
		}
	}

	//アフィン変換
	Mat3x2 mat32 = Mat3x2::Identity().scaled(scale, center).rotated(ToRadians(rotate), center);
	for (auto& vt : basis_v) vt = mat32.transformPoint(vt);

	//吹出し有無(原点＝吹出の場合キャンセル)
	if (!(center == trg))
	{
		double r = thickness;
		double ot90 = Math::Atan2(center.y - trg.y, center.x - trg.x) + ToRadians(90);
		Vec2 ot = Vec2(Math::Cos(ot90), Math::Sin(ot90)) *r;
		Line bottomline = Line(center + ot, center - ot).draw(Palette::Black);

		Line targetline1 = Line(center + ot, trg);
		Line edgeline1 = {};
		Optional<Line::position_type> xpoints1 = {};
		uint32 xiter1=1;
		for( ; xiter1<=basis_v.size(); xiter1++ )
		{
			const Line edgeline( basis_v[xiter1-1], (xiter1==basis_v.size()) ? basis_v[0]:basis_v[xiter1] );
			if( xpoints1 = edgeline.intersectsAt(targetline1) )
			{
				edgeline1 = edgeline;
				break ;
			}
		}

		Line targetline2 = Line(center - ot, trg);
		Line edgeline2 = {};
		Optional<Line::position_type> xpoints2 = {};
		uint32 xiter2=1;
		for( ; xiter2<=basis_v.size(); xiter2++ )
		{
			const Line edgeline( basis_v[xiter2-1], (xiter2==basis_v.size()) ? basis_v[0]:basis_v[xiter2] );
			if( xpoints2 = edgeline.intersectsAt(targetline2) )
			{
				edgeline2 = edgeline;
				break ;
			}
		}
        
		if( xpoints1 && xpoints2 )
		{
			uint16 last = (uint16)(basis_v.size())-1;

			basis_v << xpoints1.value() ; 
			basis_v << trg ; 
			basis_v << xpoints2.value() ;

			basis_i.resize( basis_i.size()+9 ) ;
			for(uint16 i=0;i<last+2; i++)
			{
				basis_i[i].i0 = last+3;
				basis_i[i].i1 = last+3-i-1;
				basis_i[i].i2 = last+3-i-2;
			}
		}
	}
	return { std::move(basis_v), std::move(basis_i) };
}

class Popup
{
private:

public:
	~Popup() = default;
	Popup() {}

	Popup( const Array<String> &menus, Array<uint32 *> datas,

			const Float2& center = Float2{0,0},		// 中心座標
			const uint16& ngon = 6,                 // 基準図形の種類
			const  int32& angle = 0,                // 基準図形の角度
			const uint32& size = 150,               // 基準図形の大きさ
			const  int32& rotate = 0,               // 雲の回転(アフィン変換)
			const uint32& thickness = 10,           // 吹き出しの太さ 
			const Float2& scale = Float2{ 1,1 },    // 雲の拡縮(アフィン変換)
			const Float2& stretch = Float2{ 1,1 },  // 引伸ばし倍率
			const Float2& divide = Float2{ 0,0 },   // 分割点座標(中心座標からの相対位置)
			const Float2& target = Float2{ 0,0 },   // 吹出し目標点の座標(中心座標からの相対位置)
			const ColorF& colfg = Palette::Dimgray,
			const ColorF& colbg = Palette::Bisque,
			
			const uint32& linktarget = 0)
	{
		m_center = center;
		m_texts = menus ;
		m_datas = datas;
		m_colorFG = colfg;
		m_colorBG = colbg;
		m_targetid = linktarget;

		m_ngon = ngon;              // 基準図形の種類
		m_angle = angle;            // 基準図形の角度
		m_size = size;              // 基準図形の大きさ
		m_rotate = rotate;          // 雲の回転(アフィン変換)
		m_thickness = thickness;    // 吹き出しの太さ 
		m_scale = scale;			// 雲の拡縮(アフィン変換)
		m_stretch = stretch;		// 引伸ばし倍率
		m_divide = divide;			// 分割点座標(中心座標からの相対位置)
		m_target = target;			// 吹出し目標点の座標(中心座標からの相対位置)
		m_shape = SpeechBaloon( m_center, m_ngon, m_angle, m_size, m_rotate,	//図形登録 
			            		m_thickness, m_scale, m_stretch, m_divide, m_target );
		m_rectUI = RectF { center.x,center.y,center.x,center.y };	//図形からマウスオーバー領域設定

		for( const auto & v:m_shape.vertices() )
		{
			if( m_rectUI.x > v.x ) m_rectUI.x = v.x;
			if( m_rectUI.y > v.y ) m_rectUI.y = v.y;
			if( m_rectUI.w < v.x ) m_rectUI.w = v.x;
			if( m_rectUI.h < v.y ) m_rectUI.h = v.y;
		}

		m_rectUI.w = m_rectUI.w - m_rectUI.x;
		m_rectUI.h = m_rectUI.h - m_rectUI.y + 20;	//ボタンの下のほうで押されると範囲外のためマージン確保
	}

	Float2 m_center = Float2{0,0};	   // 中心座標
	uint16 m_ngon = 6;                 // 基準図形の種類
	 int32 m_angle = 0;                // 基準図形の角度
	uint32 m_size = 150;               // 基準図形の大きさ
	 int32 m_rotate = 0;               // 雲の回転(アフィン変換)
	uint32 m_thickness = 10;           // 吹き出しの太さ 
	Float2 m_scale = Float2{ 1,1 };    // 雲の拡縮(アフィン変換)
	Float2 m_stretch = Float2{ 1,1 };  // 引伸ばし倍率
	Float2 m_divide = Float2{ 0,0 };   // 分割点座標(中心座標からの相対位置)
	Float2 m_target = Float2{ 0,0 };   // 吹出し目標点の座標(中心座標からの相対位置)

	Array<String> m_texts = {};
	Array<uint32 *> m_datas = {};

	uint32 m_targetid = 0;	//具体的なポップアップ対象指定が必要な場合※ライン番号識別など

	Shape2D m_shape;
	ColorF m_colorBG = Palette::Bisque;
	ColorF m_colorFG = Palette::Dimgray;
	RectF m_rectUI = {};						// マウスオーバー判定領域

	Shape2D &draw()
	{
		const Font &fontM = FontAsset(U"fontM");

		m_shape = SpeechBaloon( m_center, m_ngon, m_angle, m_size, m_rotate, 
			            		m_thickness, m_scale, m_stretch, m_divide, m_target );
		m_shape.draw( m_colorBG );
		
		int32 fs = fontM.fontSize()+2;
		RectF rect( m_rectUI.x+10, m_rectUI.y+2,m_rectUI.w, fs);
		for( uint32 i=0; i<m_texts.size(); i++ )
		{
			String text = m_texts[i];
			if( m_texts[i].indexOf(U"{}") ) text = m_texts[i].replace(U"{}",U""); 
			else
			{
				if( m_texts[i].indexOf(U"{unsigned int}") )        text = m_texts[i].replace(U"{unsigned int}",U"") + U":{}"_fmt( *(uint32 *)&m_datas[i]); 
				else if( m_texts[i].indexOf(U"{int}") )            text = m_texts[i].replace(U"{int}",U"") + U":{}"_fmt( *(int32 *)&m_datas[i]); 
				else if( m_texts[i].indexOf(U"{unsigned short}") ) text = m_texts[i].replace(U"{unsigned short}",U"") + U":{}"_fmt( *(uint16 *)&m_datas[i]);
				else if( m_texts[i].indexOf(U"{short}") )          text = m_texts[i].replace(U"{short}",U"") + U":{}"_fmt( *(int16 *)&m_datas[i]); 
				else if( m_texts[i].indexOf(U"{unsigned char}") )  text = m_texts[i].replace(U"{unsigned char}",U"") + U":{}"_fmt( *(uint8 *)&m_datas[i]); 
				else if( m_texts[i].indexOf(U"{signed char}") )    text = m_texts[i].replace(U"{signed char}",U"") + U":{}"_fmt( *(int8 *)&m_datas[i]);  
				else if( m_texts[i].indexOf(U"{bool}") )           text = m_texts[i].replace(U"{bool}",U"") + U":{}"_fmt( *(bool *)&m_datas[i]);  
				else if( m_texts[i].indexOf(U"{float}") )          text = m_texts[i].replace(U"{float}",U"") + U":{}"_fmt( *(float *)&m_datas[i]); 
				else if( m_texts[i].indexOf(U"{double}") )         text = m_texts[i].replace(U"{double}",U"") + U":{}"_fmt( (double)*(float *)&m_datas[i]);
				else if( m_texts[i].indexOf(U"{struct s3d::Vector2D<float>}" ))	text = m_texts[i].replace(U"{struct s3d::Vector2D<float>}",U"") + U":{}"_fmt( toFloat2(m_datas[i]));
				else if( m_texts[i].indexOf(U"{struct s3d::Vector3D<float>}" ))	text = m_texts[i].replace(U"{struct s3d::Vector3D<float>}",U"") + U":{}"_fmt( toFloat3(m_datas[i]));
				else if( m_texts[i].indexOf(U"{struct s3d::Vector4D<float>}" ))	text = m_texts[i].replace(U"{struct s3d::Vector4D<float>}",U"") + U":{}"_fmt( toFloat4(m_datas[i]));
				else if( m_texts[i].indexOf(U"{struct s3d::Vector2D<double>}" ))text = m_texts[i].replace(U"{struct s3d::Vector2D<double>}",U"") + U":{}"_fmt( toVec2(m_datas[i]));
				else if( m_texts[i].indexOf(U"{struct s3d::Vector3D<double>}" ))text = m_texts[i].replace(U"{struct s3d::Vector3D<double>}",U"") + U":{}"_fmt( toVec3(m_datas[i]));
				else if( m_texts[i].indexOf(U"{struct s3d::Vector4D<double>}" ))text = m_texts[i].replace(U"{struct s3d::Vector4D<double>}",U"") + U":{}"_fmt( toVec4(m_datas[i]));
				else											   text = m_texts[i].replace(U"{unsigned int}",U"") + U":{}"_fmt( *(uint32 *)&m_datas[i]); 
			}
			fontM( text ).draw( Float2{rect.x, rect.y}, rect.mouseOver() ? m_colorFG -ColorF(0.3) : m_colorFG );
			rect.y += fs ;
		}
		return m_shape;
	}

	bool isActive() { return ( m_texts.size() != 0 );}
};

Array<String> BezierEnable = { U"なし",U"あり"};

Array<String> EaseName = {  
	U"INLINEAR",   U"INSINE",    U"INQUAD",    U"INCUBIC",   U"INQUART",
	U"INQUINT",    U"INEXPO",    U"INCIRC",    U"INBACK",    U"INELASTIC",   U"INBOUNCE",  
	U"OUTLINEAR",  U"OUTSINE",   U"OUTQUAD",   U"OUTCUBIC",  U"OUTQUART",
	U"OUTQUINT",   U"OUTEXPO",   U"OUTCIRC",   U"OUTBACK",   U"OUTELASTIC",  U"OUTBOUNCE", 
	U"INOUTLINEAR",U"INOUTSINE", U"INOUTQUAD", U"INOUTCUBIC",U"INOUTQUART", 
	U"INOUTQUINT", U"INOUTEXPO", U"INOUTCIRC", U"INOUTBACK", U"INOUTELASTIC",U"INOUTBOUNCE"
};

class Timeline
{
private:
	MSRenderTexture m_RenderTexture;	// RTを生成

	enum IDAREA { 
		G_RELEASE, 
		TITLE, DRAGBAR, TOOLBAR, CURSOR, TRACKLIST, CONTENT, FRAME, LMARK, SELECTION, RMARK,
		TRACKL,TRACKR,TRACKHEAD,POSLINE,LINEMARKER,
		NUMAREA
	};

	enum IDGUI {
        rcRELEASE,
		rcTITLE, rcDRAGBAR, rcTOOLBAR,rcCURSOR, rcTRACKLIST, rcCONTENT, rcFRAME, rcLMARK, rcSELECTION, rcRMARK,
        rcTRACKL,rcTRACKR,rcTRACKHEAD,rcPOSLINE,rcLINEMARK,
		NUMGUI
    };
    RectF Gui[NUMGUI] = {};

	struct GRABSTATE
	{
		IDAREA area;
		uint32 trackNo;		//例えばトラック番号の識別など用
		uint32 lineNo;		//例えばライン番号の識別など用
		uint32 markerNo;	//例えばマーカー番号の識別など用
	};
	GRABSTATE m_grabState{ G_RELEASE, 0 };
	GRABSTATE m_onClickState{ G_RELEASE, 0 };//クリック時のトラック、ライン、マーカーを控えて混在除け

	template <typename V,typename T> 
	V guiValue(const Font &font,const String &text,Rect &guirect,double wheel, V initvalue, V value,T min,T max)
	{
		if ( guirect.mouseOver() && wheel != 0) value = Clamp(value+wheel,min,max); 
		if ( guirect.leftClicked()) value = initvalue;
		font(text).draw(guirect, guirect.mouseOver() ? ColorF(1) : ColorF(0)); 

		return value;
	}

	template <typename V> 
	V guiDrag(const Font &font,const String &text, Rect &guirect,Vec2 delta,GRABSTATE &grabstate, IDAREA garea, 
			  ColorF activecolor,ColorF inactivecolor, V value )
	{

		font(text).draw(Arg::center(value),guirect.mouseOver() ? activecolor : inactivecolor );
		if (grabstate.area == G_RELEASE && guirect.leftClicked()) grabstate.area = garea;
		if (grabstate.area == garea) { value += Vec2(Cursor::Delta()); guirect.x = value.x; guirect.y = value.y; }

		return value;
	}


	template <typename V> 
	double GetEasing( const uint32 &et, V numer, V denom ) 
	{ 
		double e = 0;
		double nd = (double)numer / (double)denom;
		switch (et)
		{
			case EASEINLINEAR:e = EaseInLinear(nd); break;
			case EASEINSINE:e = EaseInSine(nd); break;
			case EASEINQUAD:e = EaseInQuad(nd); break;
			case EASEINCUBIC:e = EaseInCubic(nd); break;
			case EASEINQUART:e = EaseInQuart(nd); break;
			case EASEINQUINT:e = EaseInQuint(nd); break;
			case EASEINEXPO:e = EaseInExpo(nd); break;
			case EASEINCIRC:e = EaseInCirc(nd); break;
			case EASEINBACK:e = EaseInBack(nd); break;
			case EASEINELASTIC:e = EaseInElastic(nd); break;
			case EASEINBOUNCE:e = EaseInBounce(nd); break;

			case EASEOUTLINEAR:e = EaseOutLinear(nd); break;
			case EASEOUTSINE:e = EaseOutSine(nd); break;
			case EASEOUTQUAD:e = EaseOutQuad(nd); break;
			case EASEOUTCUBIC:e = EaseOutCubic(nd); break;
			case EASEOUTQUART:e = EaseOutQuart(nd); break;
			case EASEOUTQUINT:e = EaseOutQuint(nd); break;
			case EASEOUTEXPO:e = EaseOutExpo(nd); break;
			case EASEOUTCIRC:e = EaseOutCirc(nd); break;
			case EASEOUTBACK:e = EaseOutBack(nd); break;
			case EASEOUTELASTIC:e = EaseOutElastic(nd); break;
			case EASEOUTBOUNCE:e = EaseOutBounce(nd); break;

			case EASEINOUTLINEAR:e = EaseInOutLinear(nd); break;
			case EASEINOUTSINE:e = EaseInOutSine(nd); break;
			case EASEINOUTQUAD:e = EaseInOutQuad(nd); break;
			case EASEINOUTCUBIC:e = EaseInOutCubic(nd); break;
			case EASEINOUTQUART:e = EaseInOutQuart(nd); break;
			case EASEINOUTQUINT:e = EaseInOutQuint(nd); break;
			case EASEINOUTEXPO:e = EaseInOutExpo(nd); break;
			case EASEINOUTCIRC:e = EaseInOutCirc(nd); break;
			case EASEINOUTBACK:e = EaseInOutBack(nd); break;
			case EASEINOUTELASTIC:e = EaseInOutElastic(nd); break;
			case EASEINOUTBOUNCE:e = EaseInOutBounce(nd); break;
		}
		return e;
	}

	ColorF m_colorFrame = ColorF(0.9);
	ColorF m_colorTitle = ColorF(0.8);
	ColorF m_colorToolbar = ColorF(0.1);
	ColorF m_colorDragbar = ColorF(0.2);
	ColorF m_colorContent = ColorF(0.3);
	ColorF m_colorTracklist = ColorF(0.4);

	ColorF m_colorScaleL = ColorF(0.6);
	ColorF m_colorScaleM = ColorF(0.7);
	ColorF m_colorScaleS = ColorF(0.8);

	String m_Caption = U" In-App Logic analyzer";
	bool m_Hidden;
	Rect m_Render;

	int32 m_frameThickness = 1;
	double m_paddingL = 8;
	double m_paddingR = 8;
	int32 m_fontHeight = 33;
	int32 m_trackWidth = 400;

	int32 m_intGrid = 16;

	Popup m_Popup;	//ポップアップ

	RectF  m_viewRange = RectF{ 0, -1, 900, 2 };	//横は0msから900ms 縦は-1から+1の範囲を表示
	double m_ratioH = 0;	//水平表示比
	double m_rangeVR = 0;	//水平表示幅
	double m_minVR = 0;		//水平表示最小値
	double m_maxVR = 0;		//水平表示最大値

	double m_timeFrom = 10;
	double m_timeTo = 100;
	double m_timeMin = 0;
	double m_timeMax = 3600*1000;
	double m_unitTime = 10;

public:
	enum CurveType { LINE, CURVE };
	enum EaseType {
		EASEINLINEAR, EASEINSINE, EASEINQUAD, EASEINCUBIC, EASEINQUART,
		EASEINQUINT, EASEINEXPO, EASEINCIRC, EASEINBACK, EASEINELASTIC,
		EASEINBOUNCE, EASEOUTLINEAR, EASEOUTSINE, EASEOUTQUAD, EASEOUTCUBIC,
		EASEOUTQUART, EASEOUTQUINT, EASEOUTEXPO, EASEOUTCIRC, EASEOUTBACK,
		EASEOUTELASTIC, EASEOUTBOUNCE, EASEINOUTLINEAR, EASEINOUTSINE,
		EASEINOUTQUAD, EASEINOUTCUBIC, EASEINOUTQUART, EASEINOUTQUINT,
		EASEINOUTEXPO, EASEINOUTCIRC, EASEINOUTBACK, EASEINOUTELASTIC,
		EASEINOUTBOUNCE
	};

	using Dword4 = Vector4D<uint32>;

	struct LinePoint
	{
		uint16 easeType;
		uint16 curveType;
		Array<int32> value;
		double duration;
	};

	struct Probe
	{
		void* value = nullptr;
		String type ;
		String name ;
		Color  color ;
	};
	Array<Probe> m_Probes;

	struct Track
	{
		String textCaption = U" In-App Logic analyzer";
		ColorF colorBG = ColorF{};
		double startTime = 0;
		double lengthTime = 0;		//※Δを取りたい場合があるのでsigned		
		 int32 intHeight = 0;
		uint32 intXDiv = 100;			//横軸の単位幅

		 int32 intTrackNo = 0;
		 int32 intScrollYpos = 0;		//トラック内スクロール高さ基準位置
		RectF rectCaption = RectF{};

		bool merged = false;
		int32 intTrackYpos = 0;

		Array<String> lineType;				//変数型名(RTTI.name())
		Array<String> lineNames;			//もうタイムラインないの線は水平のみ
		Array<Color> lineColors;
		Array<Array<LinePoint>> linePoints;	//ラインポイント情報[ライン番号][時系列]
		Array<Dword4> valuePrev ;			//valueの変化量(前回値)保持用[ライン番号]
		Array<int32> lineLinks ;			//-1:リンクなし
		Array<Array<int32>> lineLinkProbes;

		double dblLo = -1.0;
		double dblHi = 1.0;
		double dblYDiv = 1.0;

		~Track() = default;

		Track() {}

		Track(String textCaption, String colorBG, int32 startTime,
			int32 lengthTime, int32 intHeight, int32 intTrackNo,
			String lineR = U"", String lineG = U"", String lineB = U"",
			String lineC = U"", String lineY = U"", String lineM = U"", String lineK = U"")
			:textCaption(textCaption),
			colorBG(Color(colorBG)),
			startTime(startTime),
			lengthTime(lengthTime),
			intHeight(intHeight),
			intTrackNo(intTrackNo)
		{
			auto& ln = lineNames;
			auto& cl = lineColors;
			if (lineR != U"") { ln.emplace_back(lineR); cl.emplace_back(Color(U"#FF0000")); }
			if (lineG != U"") { ln.emplace_back(lineG); cl.emplace_back(Color(U"#00FF00")); }
			if (lineB != U"") { ln.emplace_back(lineB); cl.emplace_back(Color(U"#0000FF")); }
			if (lineC != U"") { ln.emplace_back(lineC); cl.emplace_back(Color(U"#FFFF00")); }
			if (lineY != U"") { ln.emplace_back(lineY); cl.emplace_back(Color(U"#FF00FF")); }
			if (lineM != U"") { ln.emplace_back(lineM); cl.emplace_back(Color(U"#00FFFF")); }
			if (lineK != U"") { ln.emplace_back(lineK); cl.emplace_back(Color(U"#000000")); }

		}
	};
	Array<Track> m_Tracks;


	double m_timeCursor = 0;
	double m_timePrevCtrlPoint = 0.0;
	double m_playSpeed = 100;	//PLAYボタンで再生するカーソルの移動速度マイナスで逆再生
	double m_timeForward = 100;	//F3ショートカットで登録時に自動でカーソルを進める量ms

	~Timeline() = default;
	Timeline() = default;

	void setHidden(bool hidden)	{ m_Hidden = hidden; }
	void setFontHeight(int32 height) { m_fontHeight = height; }

	void setup( const Rect& rect,
				int32 trackWidth = 400,
				int32 toolbarHeight = 60,
				RectF viewRange = RectF{ 0, -1, 900, 2 })
	{
		bool result;
		result = FontAsset::Register(U"fontS",10, Typeface::Medium );
		result = FontAsset::Register(U"fontM",15, Typeface::Medium );
		result = FontAsset::Register(U"iconS",20, Typeface::Icon_MaterialDesign);
		result = FontAsset::Register(U"iconM",25, Typeface::Icon_MaterialDesign);

		const Font &fontM = FontAsset(U"fontM");

		m_paddingL = m_paddingR = 8;
		m_Render = Rect(rect.pos, Max(rect.w, 40), Max(rect.h, m_fontHeight + (m_frameThickness * 2)));
		m_RenderTexture = MSRenderTexture(m_Render.w, m_Render.h);

		m_trackWidth = trackWidth;
		m_viewRange = viewRange;

		int32 &ft = m_frameThickness;
		int32 fh = fontM.fontSize() + 15;
		RectF& title = Gui[rcTITLE];
		RectF& frame = Gui[rcFRAME];
		RectF& toolbar = Gui[rcTOOLBAR];
		RectF& dragbar = Gui[rcDRAGBAR];
//		RectF& tracklist = Gui[rcTRACKLIST];

		title = RectF{ 0, 0, m_Render.w, fh };
		frame = RectF{ title.x, title.y + fh, title.w, m_Render.h - fh };
		toolbar = RectF{ frame.x + ft, frame.y + ft, trackWidth, toolbarHeight };
		dragbar = RectF{ toolbar.x + trackWidth, toolbar.y, frame.w - trackWidth, toolbar.h };
	}

	void addTrack(const Track& trackX) { m_Tracks << trackX; }
	
	template <typename V> 
	void PutValue(const Probe &probe, V value)	//Timelineの内部U32で処理だが符号考えたらI32の方か良いかも
	{											//probe.valueの先は任意変数なのでRTTI参照、キャストして格納
		if     ( probe.type == U"struct s3d::Vector2D<float>")	 *(Float2 *)probe.value = Float2{ ((float *)&value)[0], ((float *)&value)[1] };
		else if( probe.type == U"struct s3d::Vector3D<float>" )	 *(Float3 *)probe.value = Float3{ ((float *)&value)[0],((float *)&value)[1],((float *)&value)[2]};
		else if( probe.type == U"struct s3d::Vector4D<float>" )	 *(Float4 *)probe.value = Float4{ ((float *)&value)[0],((float *)&value)[1],((float *)&value)[2],((float *)&value)[3]};
		else if( probe.type == U"struct s3d::Vector2D<double>" ) *(Vec2 *)probe.value = Vec2{ ((double *)&value)[0], ((double *)&value)[1]};
		else if( probe.type == U"struct s3d::Vector3D<double>" ) *(Vec3 *)probe.value = Vec3{ ((double *)&value)[0], ((double *)&value)[1], ((double *)&value)[2]};
		else if( probe.type == U"struct s3d::Vector4D<double>" ) *(Vec4 *)probe.value = Vec4{ ((double *)&value)[0], ((double *)&value)[1], ((double *)&value)[2], ((double *)&value)[3]};
		else if( probe.type == U"unsigned int" )   *(uint32 *)probe.value = *(uint32*)&value; 
		else if( probe.type == U"int" )            *(int32 *)probe.value = *(int32*)&value; 
		else if (probe.type == U"unsigned short") *(uint16*)probe.value = (uint16)*(uint32*)&value ;
		else if (probe.type == U"short")          *(int16*)probe.value = (int16)*(int32*)&value ;
		else if (probe.type == U"unsigned char")  *(uint8*)probe.value = (uint8)*(uint32*)&value ;
		else if (probe.type == U"signed char")    *(int8*)probe.value = (int8)*(int32*)&value ;
		else if (probe.type == U"bool")           *(bool*)probe.value = (0 != *(uint32*)&value );
		else if( probe.type == U"float" )         *(float *)probe.value = *(float*)&value;
		else if (probe.type == U"double")         *(double*)probe.value = double{ *(float*)&value };
	}

	//任意の型をU32型配列で取得
	Array<int32> GetValueI32( const Probe &probe )
	{
		static Array<int32> val ;

		if( probe.type == U"float" )
		{
			val = Array<int32>{*(int32 *)probe.value}; 
//			LOG_INFO( U"FLT:{}"_fmt( *(float *)&val[0]) );
		}
		else if (probe.type == U"struct s3d::Vector2D<float>")
		{
			Float2 f2 = *( Float2 *)probe.value;
			val = Array<int32>{*(int32 *)&f2.x,*(int32 *)&f2.y}; 
//			LOG_INFO( U"FLT2:{},{}"_fmt( *(float *)&val[0],*(float *)&val[1]) ) ;
		}
		
		else if (probe.type == U"struct s3d::Vector3D<float>")
		{
			Float3 f3 = *( Float3 *)probe.value;
			val = Array<int32>{*(int32*)&f3.x,*(int32*)&f3.y,*(int32*)&f3.z };
		}

		else if (probe.type == U"struct s3d::Vector4D<float>")
		{
			Float4 f4 = *( Float4 *)probe.value;
			val = Array<int32>{*(int32*)&f4.x,*(int32*)&f4.y,*(int32*)&f4.z,*(int32*)&f4.w };
		}

		else if( probe.type == U"unsigned int" )
		{
			val = Array<int32>{*(int32 *)probe.value}; 
//			LOG_INFO(U"U32:{}"_fmt(val[0]));
		}
		else if( probe.type == U"int" )
		{
			val = Array<int32>{*(int32 *)probe.value}; 
//			LOG_INFO(U"I32:{}"_fmt(val[0]));
		}
		else if( probe.type == U"unsigned short" ) 
			val = Array<int32>{(int32)*(uint16 *)probe.value}; 

		else if( probe.type == U"short" ) 
			val = Array<int32>{(int32)*(int16 *)probe.value}; 
		
		else if( probe.type == U"unsigned char" ) 
			val = Array<int32>{(int32)*(uint8 *)probe.value}; 
		
		else if( probe.type == U"signed char" ) 
			val = Array<int32>{(int32)*( int8 *)probe.value}; 
		
		else if( probe.type == U"bool" ) 
			val = Array<int32>{*( bool *)probe.value ? 1:0}; 
		
		else if (probe.type == U"double")	//doubleは非対応
		{
			float dbl = (float)*( double *)probe.value;
			val = Array<int32>{ *(int32*)&dbl };
		}


		else if (probe.type == U"struct s3d::Vector2D<double>")	//doubleは非対応
		{
			Float2 v2 = *( Vec2 *)probe.value;
			val = Array<int32>{ *(int32*)&v2.x,*(int32*)&v2.y };
		}

		else if (probe.type == U"struct s3d::Vector3D<double>")	//doubleは非対応
		{
			Float3 v3 = *( Vec3 *)probe.value;
			val = Array<int32>{ *(int32*)&v3.x,*(int32*)&v3.y,*(int32*)&v3.z };
		}

		else if (probe.type == U"struct s3d::Vector4D<double>")	//doubleは非対応
		{
			Float4 v4 = *( Vec4 *)probe.value;
			val = Array<int32>{ *(int32*)&v4.x,*(int32*)&v4.y,*(int32*)&v4.z,*(int32*)&v4.w };
		}

		return val;
	}

	void setProbe(String caption,uint32 trackno,uint32 trackheight, Color colortrack, 
				const Array<Probe> &probes, double starttime = 0, uint32 lengthtime = 1000 )
	{
		Track tr ;
		m_Probes = probes ;
		tr.linePoints.resize(probes.size());
		for(uint32 i=0;i<probes.size(); i++)
		{
			tr.textCaption = caption;
			tr.colorBG = colortrack;
			tr.startTime = starttime;
			tr.lengthTime = lengthtime;	//とりあえず最初は1秒
			tr.intTrackNo = trackno;		//トラック番号
			tr.intHeight = trackheight;		//トラック表示高

			tr.lineNames << probes[i].name;
			tr.lineColors << Color(probes[i].color);
			tr.lineLinks << -1;	//負はリンクなし
			tr.lineLinkProbes << Array<int32>{};

			tr.linePoints[i] << LinePoint {EASEINLINEAR, LINE, GetValueI32( m_Probes[i] ), (double)lengthtime};
			tr.linePoints[i] << LinePoint {EASEINLINEAR, LINE, GetValueI32( m_Probes[i] ), 0};
		}

		m_Tracks << tr ;
	}

	bool update()
	{
		bool has_update = false;

		updateTitleAndFrame();
		updateDrag();
		updateCursor();
		updateSelection();
		updateShortcut();
		updateToolbar();
		updateTrackToolbar();
		updateLineMarker();
		updatePopup();

		if (MouseL.up()) m_grabState.area = G_RELEASE;
		return has_update;
	}

	void drawTrack()
	{
		const RectF& r1 = Gui[rcTRACKLIST];
		const RectF& r2 = Gui[rcFRAME];
		RectF { r1.x,r1.y, r2.w, m_frameThickness*3 }.draw(ColorF(0.5));

		//トラック背景グリッド描画
		for (int32 i = 0; i < m_Tracks.size(); ++i)
		{
			Track& tr = m_Tracks[i];

			double unit;
			if (m_rangeVR < 1000)        unit = m_intGrid;
			else if (m_rangeVR < 10000)  unit = m_intGrid * 10;
			else if (m_rangeVR < 100000) unit = m_intGrid * 100;
			else                         unit = m_intGrid * 1000;

			int32 cnt =  (int32)m_minVR / (int32)unit;
			double xxx = (int32)m_minVR % (int32)unit;

			for (int32 x = int32(m_minVR - xxx); x < int32(m_maxVR); x += int32(unit) )
			{
				RectF rectT(x, Gui[rcTRACKLIST].y + m_frameThickness + tr.intTrackYpos, unit, m_Tracks[tr.intTrackNo].intHeight);
				auto lb = (rectT.x < m_minVR) ? m_minVR - rectT.x : 0;
				auto rb = (rectT.x + rectT.w > m_maxVR) ? (rectT.x + rectT.w) - m_maxVR : 0;
				rectT.w = (rectT.w - lb - rb) * m_ratioH;
				rectT.x = (int32)Gui[rcDRAGBAR].x + (Clamp(rectT.x, m_minVR, m_maxVR) - m_minVR) * m_ratioH;

				rectT.draw((++cnt & 1) ? ColorF(0.30) : ColorF(0.31));
			}
		}

		drawTrackList();	//トラック一覧描画
	}

	//トラック一覧描画
	void drawTrackList()
	{
		const Font &fontM = FontAsset(U"fontM");
		const Font &iconM = FontAsset(U"iconM");
        const Transformer2D transform(Mat3x2::Identity(), Mat3x2::Translate(m_Render.pos));

		const int32 trackSpacing = 2;
		int32 ypos = 2;
//		int32 tx = Gui[rcTRACKLIST].x + m_frameThickness;

		int32 fs = fontM.fontSize();
		int32 is = iconM.fontSize() + 1;

		for (int32 i = 0; i < m_Tracks.size(); ++i)
		{
			Track& tr = m_Tracks[i];
			double tx = Gui[rcTRACKLIST].x;
			double ty = Gui[rcTRACKLIST].y + m_frameThickness + tr.intTrackYpos ;
			RectF rectT(tr.startTime, ty, tr.lengthTime, m_Tracks[tr.intTrackNo].intHeight);	//トラック領域(仮想)

			tr.intTrackYpos = ypos;

			//トラック背景描画
			if ( (rectT.x + rectT.w) < m_minVR || m_maxVR < rectT.x ) {} //範囲外
			else
			{
				RectF rectD = rectT;
				double lb = (rectT.x < m_minVR) ? m_minVR - rectT.x : 0;
				double rb = (rectT.x + rectT.w > m_maxVR) ? (rectT.x + rectT.w) - m_maxVR : 0;
				rectD.w = (rectT.w - lb - rb) * m_ratioH;
				rectD.x = (int32)Gui[rcDRAGBAR].x + (Clamp(rectT.x, m_minVR, m_maxVR) - m_minVR) * m_ratioH;
				RectF rectH = rectD;
				rectD.h -= is;rectD.y += is;
				rectD.draw(tr.colorBG);
				rectH.h = is;
				rectH.draw(tr.colorBG+ColorF(0.2));

				ColorF colOFF = ColorF(0.3);
				ColorF colON = ColorF(0.9);
				RectF rectL = rectH;
				rectL.w = rectL.h = is; 
				RectF rectR = rectL;
				rectR.x += rectH.w - is;
				iconM(U"\xF0DBA").draw(rectL, !m_Popup.isActive() && rectL.mouseOver() ? colON : colOFF);
				iconM(U"\xF0DBB").draw(rectR, !m_Popup.isActive() && rectR.mouseOver() ? colON : colOFF);

				if( rectH.mouseOver() )	//トラックヘッダにマウスあればGUI対象に追加
				{
					m_grabState.trackNo = i;
					Gui[rcTRACKL] = rectL;
					Gui[rcTRACKR] = rectR;
					rectH.x += is; rectH.w -= is*2;
					Gui[rcTRACKHEAD] = rectH;
				}
			}


			if (tr.intTrackNo != -1 && i != tr.intTrackNo)
			{
				tr.merged = true;
				tr.intTrackYpos = m_Tracks[tr.intTrackNo].intTrackYpos;
			}
			else ypos += tr.intHeight + trackSpacing;

			//トラックリスト描画
			if (tr.merged == false)
			{
				tr.rectCaption = RectF{ tx, ty, m_trackWidth, rectT.h };
				tr.rectCaption.draw( ColorF(0.2) );

				if( tr.rectCaption.mouseOver() )	//トラックリストにマウスあればGUI対象に追加
				{
					m_grabState.trackNo = i;
				}

				fontM(tr.textCaption).draw(tr.rectCaption.movedBy(m_paddingL, 0), ColorF(0.7));

				RectF btn{ tx + fs * 10, ty, is, is };
				for (int32 ii=0; ii<m_TrackToolbar.size(); ii++)
				{
					auto& tbi = m_TrackToolbar[ii];
					iconM(tbi.code).draw(btn, !m_Popup.isActive() && btn.mouseOver() ? ColorF(0.9) : ColorF(0.5));
					btn.x += btn.w;
					if (btn.x > tx + m_trackWidth) break;
				}

				for (int32 ii = 0;ii < tr.lineNames.size();ii++)
				{
					const int32 ttbh = fs*2;					//トラックツールバー高
					double yy = tr.intScrollYpos + ii * fs * 3 + ttbh;
					if (yy < ttbh || yy > rectT.h) continue;	//トラックラインの表示上限

					double bx = tr.rectCaption.x + m_paddingL;
					double by = tr.rectCaption.y + yy;
					fontM(tr.lineNames[ii]).draw(bx, by, tr.lineColors[ii]);
					
					//ここでカーソル位置の表示しようと思ったがたいへん
					//値は、イージング、エルミート補間だし、RTTIバリアント。
					//現在の構造は、再生時にやっとFloat3だけの対応。
					//そもそも3次元エルミートも正しくない。値をどこかにバッファできてれば表示するだけだったが。
					//m_fontS(U"{}"_fmt()).draw(bx, by+fs, tr.lineColors[ii]);

					RectF btn2 { bx + fs * 6, by, double(is), double(is) };
					for (int32 iii=0; iii<m_LineToolbar.size(); iii++)
					{
						auto& tbi = m_LineToolbar[iii];
						ColorF offcolor = ColorF(0.5);
						if( tbi.code == LTB_LINK && tr.lineLinks[ii] != -1 ) offcolor = ColorF(0,1,0);
						iconM(tbi.code).draw(btn2, !m_Popup.isActive() && btn2.mouseOver() ? ColorF(0.9) : offcolor);

						btn2.x += btn2.w;
						if (btn2.x > tx + m_trackWidth) break;
					}

					//トラックライン描画
					drawTrackLine( rectT, tr, ii, double(is), by );
				}
			}


		}
	}

	//トラックライン描画
	void drawTrackLine( const RectF &rectT, const Track &tr, const uint32& lineno, const double& iconsize, const double& by )
	{
	    const Font &fontS = FontAsset(U"fontS");
	    const Font &iconS = FontAsset(U"iconS");
        const Transformer2D transform(Mat3x2::Identity(), Mat3x2::Translate(m_Render.pos));

		RectF rectDraw = { (double)Gui[rcDRAGBAR].x, (double)by, (double)Gui[rcDRAGBAR].w, (double)tr.intHeight } ;//描画領域

		auto &lp = tr.linePoints[lineno];

		//ライン描画
		double yy = rectDraw.y + iconsize/2;
		ColorF lc = tr.lineColors[lineno];

		double lx = rectT.x;								//最初の時間座標
		LineString ls;
		for( int32 i=0; i<lp.size(); i++ )					//各ライン制御点を検索
		{
			if( lp[i].duration == 0 ) break ;				//duration=0は終端

			double lxd = lx + lp[i].duration;				//線分+長さ→終点

			if ( lxd < m_minVR ){}							//範囲外
			else
			{
				double clx = Clamp(lx, m_minVR, m_maxVR);		//始点範囲内クランプ
				double clxd = Clamp(Clamp(lxd,(double)tr.startTime,(double)tr.startTime+tr.lengthTime), m_minVR, m_maxVR);		//終点範囲内クランプ
				double bx = rectDraw.x + (clx-m_minVR) * m_ratioH;  //始点描画座標
				double ex = rectDraw.x + (clxd-m_minVR) * m_ratioH;//終点描画座標
			
				Line(bx, yy, ex, yy).draw(LineStyle::RoundCap, iconsize - 15, lc - ColorF(0.2));
			}
			lx = lxd;											//次の線分
			if( lx > rectT.x+rectT.w || lx > m_maxVR ) break ;	//トラック超過したら終了
		}

		//ラインマーカー描画
		lx = rectT.x;										//最初の時間座標
		int32 i=0;									
		RectF rectM;
		for( ; i<lp.size(); i++ )							//各ライン制御点を検索
		{
			if( lp[i].duration == 0 ) break ;				//duration=0は終端

			double lxd = lx + lp[i].duration;				//線分+長さ→終点

			if ( lx < m_minVR ) {}							//範囲外
			else
			{
				double clx = Clamp(lx, m_minVR, m_maxVR);			//始点範囲内クランプ
				double bx = rectDraw.x + (clx-m_minVR) * m_ratioH;	//始点描画座標
		
				rectM = RectF{ bx - iconsize, yy - iconsize,iconsize * 2,iconsize * 2 } ;
				iconS( U"\xF0E7E" ).draw( Arg::center( bx, yy), !m_Popup.isActive() && rectM.mouseOver()? lc+ColorF(0.5):lc );//ラインマーカー描画

				String text = U"";
				const uint32 &ii=lineno;
				if( m_Probes[ii].type == U"int" )                             text = U"{}"_fmt( *(int32 *)&lp[i].value[0]) ;
				else if( m_Probes[ii].type == U"unsigned short" )             text = U"{}"_fmt( *(uint16 *)&lp[i].value[0]) ;
				else if( m_Probes[ii].type == U"short" )                      text = U"{}"_fmt( *(int16 *)&lp[i].value[0]) ;
				else if( m_Probes[ii].type == U"unsigned char" )              text = U"{}"_fmt( *(uint8 *)&lp[i].value[0]) ;
				else if( m_Probes[ii].type == U"signed char" )                text = U"{}"_fmt( *(int8 *)&lp[i].value[0]) ;
				else if( m_Probes[ii].type == U"bool" )                       text = U"{}"_fmt( *(bool *)&lp[i].value[0] ?U"TRUE":U"FALSE" ) ;
				else if( m_Probes[ii].type == U"float" )                      text = U"{:.1f}"_fmt( *(float *)&lp[i].value[0]) ;
				else if( m_Probes[ii].type == U"double" )                     text = U"{:.2f}"_fmt(  (double)*(float *)&lp[i].value[0]) ;
				else if( m_Probes[ii].type == U"struct s3d::Vector2D<float>") text = U"{:.1f}"_fmt(   Float2{*(float *)&lp[i].value[0], *(float *)&lp[i].value[1]}) ;
				else if( m_Probes[ii].type == U"struct s3d::Vector3D<float>") text = U"{:.1f}"_fmt(   Float3{*(float *)&lp[i].value[0], *(float *)&lp[i].value[1],*(float *)&lp[i].value[2]});                    
				else if( m_Probes[ii].type == U"struct s3d::Vector4D<float>") text = U"{:.1f}"_fmt(   Float4{*(float *)&lp[i].value[0], *(float *)&lp[i].value[1],*(float *)&lp[i].value[2],*(float *)&lp[i].value[3]});
				else if( m_Probes[ii].type == U"struct s3d::Vector2D<double>") text = U"{:.2f}"_fmt(  Vec2{*(float *)&lp[i].value[0],*(float *)&lp[i].value[1]});                                           
				else if( m_Probes[ii].type == U"struct s3d::Vector3D<double>") text = U"{:.2f}"_fmt(  Vec3{*(float *)&lp[i].value[0],*(float *)&lp[i].value[1],*(float *)&lp[i].value[2]});                       
				else if( m_Probes[ii].type == U"struct s3d::Vector4D<double>") text = U"{:.2f}"_fmt(  Vec4{*(float *)&lp[i].value[0],*(float *)&lp[i].value[1],*(float *)&lp[i].value[2],*(float *)&lp[i].value[3]});   
				else                                                           text = U"{}"_fmt( *(uint32 *)&lp[i].value[0]) ;
				fontS( U"{}"_fmt( text ) ).draw( Arg::center( bx, yy+15), lc );

				fontS( U"{}"_fmt( (uint32)lx ) ).draw( Arg::center( bx, rectT.y+20), ColorF(0.8));
			}

			lx = lxd;											//次の線分
			if( lx > rectT.x+rectT.w || lx > m_maxVR ) break;	//トラック超過したら終了
		}

		double clx = Clamp(lx, m_minVR, m_maxVR);				//終点範囲内クランプ
		if( clx <= tr.startTime+tr.lengthTime )
		{
			double ex = rectDraw.x + (clx-m_minVR) * m_ratioH;		//始点描画座標
			rectM = RectF{ ex - iconsize, yy - iconsize,iconsize * 2,iconsize * 2 };
			iconS( U"\xF0E7E" ).draw( Arg::center( ex, yy), !m_Popup.isActive() && rectM.mouseOver()? lc+ColorF(0.5):lc );//ラインマーカー描画
		}
	}


	//タイムラインUIのドラッグ操作
	void updateDrag()
	{
	    const Font &fontM = FontAsset(U"fontM");
        const Transformer2D transform(Mat3x2::Identity(), Mat3x2::Translate(m_Render.pos));

//		RectF& title = Gui[rcTITLE];
//		RectF& frame = Gui[rcFRAME];
//		RectF& toolbar = Gui[rcTOOLBAR];
		RectF& dragbar = Gui[rcDRAGBAR];
		RectF& tracklist = Gui[rcTRACKLIST];
		RectF& track_l = Gui[rcTRACKL];
		RectF& track_r = Gui[rcTRACKR];
		RectF& track_head = Gui[rcTRACKHEAD];

		RectF& select = Gui[rcSELECTION];
		RectF& lmark = Gui[rcLMARK];
		RectF& rmark = Gui[rcRMARK];
		RectF& cursor = Gui[rcCURSOR];
//		RectF& content = Gui[rcCONTENT];

//		RectF& posline  = Gui[rcPOSLINE];
//		RectF& linemark = Gui[rcLINEMARK];

		//左ドラッグ開始
		if      (select.leftClicked())		m_grabState.area = SELECTION;
		else if (lmark.leftClicked())		m_grabState.area = LMARK;
		else if (rmark.leftClicked())		m_grabState.area = RMARK;
		else if (cursor.leftClicked())		m_grabState.area = CURSOR;
		else if (dragbar.leftClicked())		m_grabState.area = DRAGBAR;
		else if (tracklist.leftClicked())	m_grabState.area = TRACKLIST;
		else if (track_r.leftClicked())		m_grabState.area = TRACKR;
		else if (track_l.leftClicked())		m_grabState.area = TRACKL;
		else if (track_head.leftClicked())	m_grabState.area = TRACKHEAD;

		// 左ドラッグ（ドラッグバー用）
		double vx = (Cursor::Delta().x * m_viewRange.w / dragbar.w);
		double min = m_viewRange.x;
		double max = m_viewRange.x + m_viewRange.w;

		if (m_grabState.area == SELECTION)
		{
			m_timeFrom = Clamp(m_timeFrom + vx, min, max);
			m_timeTo = Clamp(m_timeTo + vx, min, max);
		}
		else if (m_grabState.area == LMARK)   m_timeFrom = Clamp(m_timeFrom + vx, min, m_timeTo);
		else if (m_grabState.area == RMARK)   m_timeTo = Clamp(m_timeTo + vx, m_timeFrom, max);
		else if (m_grabState.area == CURSOR )
		{
			if( isRecTime == INACTIVE ) m_timeCursor = Clamp(m_timeCursor + vx, min, max);
		}
		else if (m_grabState.area == DRAGBAR) m_viewRange.x = Clamp(m_viewRange.x - vx, m_timeMin, m_timeMax);
		else if (m_grabState.area == TRACKLIST)
		{
			const int32 fs = fontM.fontSize();
			const int32 ttbh = fs*2;					//トラックツールバー高
			Track& tr = m_Tracks[ m_grabState.trackNo ];

			int32 rangeY = int32(tr.lineNames.size() * fs * 3 + ttbh) ;
			tr.intScrollYpos += Cursor::Delta().y;
			if ( tr.intScrollYpos > 0) tr.intScrollYpos = 0;
			if ( tr.intScrollYpos < tr.intHeight-rangeY) tr.intScrollYpos = tr.intHeight-rangeY ;
		}

		else if (m_grabState.area == TRACKR)			//トラック終端変更
		{
			Track& tr = m_Tracks[ m_grabState.trackNo ];
			double delta = tr.startTime + tr.lengthTime; 
			double deltalength = tr.lengthTime;
			tr.lengthTime = Clamp(delta + vx, (double)tr.startTime+10, m_viewRange.x+m_viewRange.w)- tr.startTime;

			deltalength = tr.lengthTime - deltalength;
			for( auto &lp: tr.linePoints ) lp[lp.size()-2].duration += deltalength;//最後の手前のDurationに差分を適用
		}

		else if (m_grabState.area == TRACKL)			//トラック始端変更
		{
			Track& tr = m_Tracks[ m_grabState.trackNo ];
			double delta = tr.startTime; 
			tr.startTime = Clamp( (double)tr.startTime + vx, m_viewRange.x, 
			                         (double)tr.startTime+tr.lengthTime-10);
			delta = tr.startTime - delta;
			tr.lengthTime -= delta;
		}

		else if (m_grabState.area == TRACKHEAD)			//トラック移動
		{
			Track& tr = m_Tracks[ m_grabState.trackNo ];
			tr.startTime = Clamp(tr.startTime + vx, m_viewRange.x, m_viewRange.x+m_viewRange.w);
		}

		else if ( m_grabState.area == POSLINE &&					//位置マーカー移動
			      m_grabState.lineNo == m_onClickState.lineNo && 
			      m_grabState.markerNo == m_onClickState.markerNo )	//位置マーカー詳細一致
		{
			Track& tr = m_Tracks[ m_grabState.trackNo ];
			Array<LinePoint> &lp = tr.linePoints[m_grabState.lineNo];
			uint32 &i = m_grabState.markerNo;
			Float2 pos = Float2{ *(float *)&lp[i].value[0], *(float *)&lp[i].value[1] } ;
			pos += Float2( Cursor::Delta() );
			lp[i].value[0] = *(uint32 *)&pos.x;
			lp[i].value[1] = *(uint32 *)&pos.y;
		}

		else if (m_grabState.area == LINEMARKER &&					//ラインマーカー移動
			m_grabState.lineNo == m_onClickState.lineNo &&
			m_grabState.markerNo == m_onClickState.markerNo)	//ラインマーカー詳細一致
		{
			Track& tr = m_Tracks[m_grabState.trackNo];
			Array<LinePoint>& lp = tr.linePoints[m_grabState.lineNo];
			uint32& i = m_grabState.markerNo;
			if (i == 0 || lp[i].duration == 0) {}
			else
			{
				double dmax = lp[i - 1].duration + lp[i].duration ;
				lp[i-1].duration = Clamp( lp[i-1].duration + vx, 1.0, dmax-1 );
				lp[i].duration = dmax - lp[i-1].duration ;
			}
		}

		//ドラッグバー上Wheel操作
		if( !m_Popup.isActive() )			//ポップアップ中はキャンセル
		{
			double wheel = Mouse::Wheel();
			if (dragbar.mouseOver() && wheel)
			{
				if (KeyControl.pressed())	//スクロール(Wheel+Ctrl)
				{
					double vel = wheel * m_viewRange.w;
					m_viewRange.x = Clamp(m_viewRange.x + vel, m_timeMin, m_timeMax);//最大1時間
				}
				else						//拡縮(Wheel)
				{
					double vel = wheel * m_viewRange.w / 10.0;
					m_viewRange.w = Clamp(m_viewRange.w + vel, 1.0, 100000.0);
				}
			}
		}

	}

	void updateTitleAndFrame()
	{
	    const Font &fontM = FontAsset(U"fontM");
        const Transformer2D transform(Mat3x2::Identity(), Mat3x2::Translate(m_Render.pos));

		RectF& title = Gui[rcTITLE];
		RectF& frame = Gui[rcFRAME];
		RectF& toolbar = Gui[rcTOOLBAR];
		RectF& dragbar = Gui[rcDRAGBAR];
		RectF& tracklist = Gui[rcTRACKLIST];

		int32& ft = m_frameThickness;
		int32 fh = fontM.fontSize() + 15;

		title = RectF{ 0, 0, m_Render.w, fh };
		frame = RectF{ title.x, title.y + fh, title.w, m_Render.h - fh };
		toolbar = RectF{ frame.x + ft, frame.y + ft, toolbar.w, toolbar.h };
		dragbar = RectF{ toolbar.x + toolbar.w, toolbar.y, frame.w - toolbar.w, toolbar.h };
		tracklist = RectF{ toolbar.x, toolbar.y+ toolbar.h, toolbar.w, m_Render.h - toolbar.h - title.h};

		if (m_grabState.area == G_RELEASE && title.leftClicked()) m_grabState.area = TITLE;
		if (m_grabState.area == TITLE)
		{
			Point po = Cursor::Delta();
			m_Render.x += po.x;
			m_Render.y += po.y;
		}
	}

	void draw()
	{
		const Font &fontM = FontAsset(U"fontM");
		const Font &iconM = FontAsset(U"iconM");

		RectF& title = Gui[rcTITLE];
		RectF& frame = Gui[rcFRAME];
		RectF& toolbar = Gui[rcTOOLBAR];
		RectF& dragbar = Gui[rcDRAGBAR];
//		RectF& tracklist = Gui[rcTRACKLIST];

//		RectF& select = Gui[rcSELECTION];
//		RectF& lmark = Gui[rcLMARK];
//		RectF& rmark = Gui[rcRMARK];
//		RectF& cursor = Gui[rcCURSOR];
//		RectF& content = Gui[rcCONTENT];

		//影はm_RenderTextureでなく直接描画
		RectF{ m_Render.pos, title.w, title.h }.drawShadow(Vec2(6, 6), 24, 3).draw();
		RectF{ m_Render.x, m_Render.y+title.h, frame.w, frame.h }.drawShadow(Vec2(6, 6), 24, 3).draw();

		{
			ScopedRenderTarget2D target(m_RenderTexture);		// 描画対象をRTに指定
			title.draw(m_colorTitle);
			frame.draw(m_colorFrame);

			iconM(U"\xF0697").draw(title, Palette::Black);
			fontM(m_Caption).draw(title.moveBy(iconM.fontSize(), 0), Palette::Black);

			toolbar.draw(m_colorToolbar);
			dragbar.draw(m_colorDragbar);

			drawScale(Point(int32(dragbar.x), int32(toolbar.y) + 40));
			drawTrack();
			drawToolbar();
			drawCursor();
			drawSelection();
			drawPopup();
		}

	}

	Float2 getPosLS( const LineString &ls, double distance )
	{
		if (ls.size() == 0) return {};
		double length = 0.0;
		uint32 i = 0;
		for (; i < ls.size(); i++)
		{
			if (length + ls[i].length() > distance) break;
			length += ls[i].length();
		}
		if( i == ls.size() ) return ls[i-1];
		return ls[i].lerp(ls[i + 1], (distance - length) / ls[i].length());
	}
/*
	//カメラ位置ライン描画
	void drawPosline( uint32 trackno, uint32 lineno )
	{
		static double move = 0.0;
		move += Scene::DeltaTime() * 10;

		Track& tr = m_Tracks[ 0 ];
		Array<LinePoint> &lp = tr.linePoints[lineno];

		ColorF lc = tr.lineColors[lineno];
		int32 is = (*m_iconM).fontSize()+1;
		Rect mark;

		LineString ls;	//lsで描画はよいのだが、3D→2D変換してから①登録時に３D登録する。②表示時にスクリーン座標変換する

		ls << toFloat2(lp[0].value) ;
		uint16 type = lp[0].bezierType;
		for( int32 i=1; i<lp.size(); i++ )					//各ライン制御点を検索
		{
			if( lp[i].bezierType == type )
			{
				ls << toFloat2(lp[i].value);
			}
			else	//切り替わり						
			{
				if (type == LINE)	//Line→Curve※登録せず線引きして１こ前からカーブに変更
				{
					ls.draw( LineStyle::SquareDot(move), 2 );
					ls.clear();
					ls << toFloat2(lp[i-1].value);
					ls << toFloat2(lp[i].value);
					type = lp[i].bezierType;
				}
				else				//Curve→Line
				{
					ls << toFloat2(lp[i].value);
					ls.catmullRom(10).draw(LineStyle::SquareDot(move), 2);//曲線の中間点は分割+線形補間？
					ls.clear();
					ls << toFloat2(lp[i].value);
					type = lp[i].bezierType;
				}
			}
		}

		if ( ls.size() )
		{
			if (type == CURVE) ls.drawCatmullRom(LineStyle::SquareDot(move), 2);
			else               ls.draw(LineStyle::SquareDot(move), 2);
		}
	
		for (int32 i = 0; i < lp.size(); i++)					//各ライン制御点を検索
		{
			Float2 markpos = toFloat2(lp[i].value) ;
			mark = { (int32)markpos.x,(int32)markpos.y,is,is };
			(*m_iconM)(U"\xF0A10").draw(Arg::center(mark.x, mark.y), mark.mouseOver() ? ColorF(1) : lc);
		}
	} 
*/



	//目盛描画
	void drawScale( Point base )
	{
		double unitS = m_unitTime;

		RectF& dragbar = Gui[rcDRAGBAR];

		m_ratioH = dragbar.w / m_viewRange.w;	//水平表示比
		m_rangeVR = m_viewRange.w;				//水平表示幅
		m_minVR = m_viewRange.x;				//水平表示最小値
		m_maxVR = m_viewRange.x + m_rangeVR;	//水平表示最大値

		if      (m_rangeVR < 100)   drawScaleLine( base,     1, 100 );
		else if (m_rangeVR < 1000)  drawScaleLine( base, int32(unitS), 1000 );
		else if (m_rangeVR < 10000) drawScaleLine( base, int32(unitS), 10000 );
		else                        drawScaleLine( base, int32(unitS), 100000 );

	}

	//目盛線描画
	void drawScaleLine( Point base, int32 units, int32 digit )
	{
	    const Font &fontM = FontAsset(U"fontM");

		double xxx = (int32)m_minVR % 10;
		for (int32 x = m_minVR - xxx; x < m_maxVR; x += units)
		{
			int32 xx = base.x + (x - m_minVR) * m_ratioH;
			if (xx < base.x) continue;

			if (x % (digit/10) == 0)
			{
				String ms = U"{}"_fmt(x);
				fontM(ms).draw(Arg::center(xx, base.y - 2 - fontM.fontSize())), m_colorScaleM;
				Line(xx, base.y - 8, xx, base.y).draw(1, m_colorScaleM);
			}

			else
			{
				if (digit == 100) Line(xx, base.y - 6, xx, base.y).draw(1, m_colorScaleL);

				else if (digit == 1000)
				{
					if (x % 10 == 0) Line(xx, base.y - 8, xx, base.y).draw(1, m_colorScaleL);
					else Line(xx, base.y - 6, xx, base.y).draw(1, m_colorScaleL);
				}

				else if (digit == 10000)
				{
					if      (x % 100 == 0) Line(xx, base.y - 7, xx, base.y).draw(1, m_colorScaleM);
					else if (x % 10 == 0) Line(xx, base.y - 5, xx, base.y).draw(1, m_colorScaleL);
				}

				else
				{
					if      (x % 1000 == 0) Line(xx, base.y - 8, xx, base.y).draw(1, m_colorScaleS);
					else if (x % 100 == 0) Line(xx, base.y - 6, xx, base.y).draw(1, m_colorScaleS);
				}
			}
		}
	}

	//ツールバーボタン定義
	enum ID_BTN {ID_PLAY,ID_STEP,ID_STOP,ID_REC,ID_EDIT,ID_CURSOR,ID_FROM,ID_TO,ID_ANCHOR,ID_CONFIG,ID_LINK };
	String BTN_PLAY   = U"\xF040A",   BTN_STEP   = U"\xF040E",	BTN_STOP   = U"\xF04DB",
	       BTN_REC    = U"\xF044B",   BTN_EDIT   = U"\xF12C6",	BTN_CURSOR = U"\xF0523",
	       BTN_FROM   = U"\xF0367",   BTN_TO     = U"\xF0361",	BTN_ANCHOR = U"\xF0031",
	       BTN_CONFIG = U"\xF06F1",   BTN_LINK   = U"\xF0339";
	struct Toolbar
	{
		String code;
		double sec;
//		String popuptext;
//		ColorF colorON;
//		ColorF colorOFF;
	};
	const double INACTIVE = 0.0;
	Array<Toolbar> m_Toolbar = { { BTN_PLAY,  0.0},
								 { BTN_STEP,  0.0},
								 { BTN_STOP,  0.0},
								 { BTN_REC,   0.0},
								 { BTN_EDIT,  0.0},
								 { BTN_CURSOR,0.0},
								 { BTN_FROM,  0.0},
								 { BTN_TO,    0.0},
								 { BTN_ANCHOR,0.0},
								 { BTN_CONFIG,0.0},
								 { BTN_LINK,  0.0} };

	//トラックツールバーボタン定義
	enum ID_TTB {TT_FUNC1,TT_FUNC2,TT_FUNC3,TT_FUNC4,TT_FUNC5,TT_FUNC6,TT_FUNC7,TT_FUNC8 };

	String TTB_SET    = U"\xF06FF",   TTB_DEL     = U"\xF05E8",	TTB_FUNC3 = U"\xF06F4",
	       TTB_FUNC4  = U"\xF0B45",   TTB_FUNC5   = U"\xF02C3",	TTB_FUNC6 = U"\xF0031",
	       TTB_FUNC7  = U"\xF06F1",   TTB_FUNC8   = U"\xF0339";

	Array<Toolbar> m_TrackToolbar = { { TTB_SET,    0.0 },
								   	  { TTB_DEL,    0.0 },
									  { TTB_FUNC3,  0.0 },
									  { TTB_FUNC4,  0.0 },
									  { TTB_FUNC5,  0.0 },
									  { TTB_FUNC6,  0.0 },
									  { TTB_FUNC7,  0.0 },
									  { TTB_FUNC8,  0.0 }};

	//ラインツールバーボタン定義
	enum ID_LTB {LT_BACK,LT_FORWARD,LT_FUNC3,LT_FUNC4,LT_FUNC5,LT_FUNC6,LT_FUNC7,LT_FUNC8 };

	String  LTB_BACK  = U"\xF0051",    LTB_FORWARD  = U"\xF0058",
			LTB_SET    = U"\xF06FF",   LTB_LINK     = U"\xF0339",	LTB_FUNC5 = U"\xF06F4",
	        LTB_FUNC6  = U"\xF0B45",   LTB_FUNC7    = U"\xF02C3",	LTB_FUNC8 = U"\xF0031",
	        LTB_FUNC9  = U"\xF06F1",   LTB_FUNC10   = U"\xF0FA7";

	Array<Toolbar> m_LineToolbar = {{ LTB_BACK,   0.0 },
								   	{ LTB_FORWARD,0.0 },
									{ LTB_SET,    0.0 },
									{ LTB_LINK,   0.0 },
									{ LTB_FUNC5,  0.0 },
									{ LTB_FUNC6,  0.0 },
									{ LTB_FUNC7,  0.0 },
									{ LTB_FUNC8,  0.0 },
									{ LTB_FUNC9,  0.0 },
									{ LTB_FUNC10, 0.0 }};


	double& isPlayTime = m_Toolbar[ID_PLAY].sec;
	double& isRecTime = m_Toolbar[ID_REC].sec;
	double& isStepTime = m_Toolbar[ID_STEP].sec;


	//ツールバー描画
	void drawToolbar()
	{
	    const Font &iconM = FontAsset(U"iconM");
        const Transformer2D transform(Mat3x2::Identity(), Mat3x2::Translate(m_Render.pos));

		double is = iconM.fontSize() + 1;

		//アイコンマウスオーバー
		double tx = Gui[rcTOOLBAR].x + m_paddingL;
		double ty = Gui[rcTOOLBAR].y;
		RectF btn{ tx, ty, is, is };
		for (int32 i=0; i<m_Toolbar.size(); i++)
		{
			auto& tbi = m_Toolbar[i];
			if     ( tbi.code == BTN_PLAY && tbi.sec != INACTIVE )
			{
				iconM(tbi.code).draw(btn, ColorF(0,0.9,0) );
			}
			else if( tbi.code == BTN_REC && tbi.sec != INACTIVE ) iconM(tbi.code).draw(btn, ColorF(0.9,0,0) );
			else if( tbi.code == BTN_STEP && tbi.sec != INACTIVE ) iconM(tbi.code).draw(btn, ColorF(0,0,0.9) );
			else iconM(tbi.code).draw(btn, btn.mouseOver() ? ColorF(0.9) : ColorF(0.3));
			btn.x += btn.w;
			if (btn.x > tx + m_trackWidth) break;

		}
	}
	
	//レンダーテクスチャを使うって事は、起点座標がいつも０になるって事で、つまりその、、マウスオーバーが全滅するので、GUIは逆にたいへん

	//ポップアップ描画
	void drawPopup()
	{
		//再生ボタンのポップアップWheel操作
		if( !m_Popup.m_texts.size() ) return ;

		String &menu = m_Popup.m_texts[0];
		if( -1 != menu.indexOf(U"再生速度:"))				m_Popup.draw();		
		else if( -1 != menu.indexOf(U"イージング種別:"))	m_Popup.draw();		
		else if( -1 != menu.indexOf(U"カーソル送り時間:"))	m_Popup.draw();		
		else if( -1 != menu.indexOf(U"リンク対象:"))		m_Popup.draw();		
	}


	//制御点削除
	void deleteCtrlPoint( uint32 trackno )
	{
		Track& tr = m_Tracks[ trackno ];
		double timeC = m_timeCursor;
		if( timeC <= m_Tracks[ trackno ].startTime ) return ;//トラック範囲外無効	
		if( timeC > m_Tracks[ trackno ].startTime+tr.lengthTime ) return ;

		Array<Array<LinePoint>> &tlp = tr.linePoints;

		for( uint32 i=0; i<tlp.size(); i++ )					//各ライン挿入点を検索
		{
			Array<LinePoint> &lp = tlp[i];
			double linePeriod = 0;

			for(uint32 ii=0; ii<lp.size(); ii++ )				//各ライン挿入点を検索
			{
				if( lp[ii].duration == 0 ) break ;				//duration=0は終端
				if( linePeriod == timeC-tr.startTime )		//マーカー時間一致
				{
					lp[ii-1].duration += lp[ii].duration;	
					lp.erase( lp.begin()+ii );
				}
				linePeriod += lp[ii].duration;					//経過時間加算
			}
		}
	}

	//制御点登録
	void registerCtrlPoint( uint32 trackno )
	{
		Track& tr = m_Tracks[trackno];
		double timeC = m_timeCursor;
		if( timeC < tr.startTime ) return ;					//トラック範囲外無効	
		if( timeC > tr.startTime+tr.lengthTime ) return ;

		Array<Array<LinePoint>> &tlp = tr.linePoints;

		if( timeC == tr.startTime )							//始点に対する更新
		{
			for( uint32 i=0; i<tlp.size(); i++ )					//各ライン
			{
				Array<LinePoint> &lp = tlp[i];
				lp[ 0 ].value = GetValueI32( m_Probes[i] );			

			}
			return ;
		}

		Array<double> linePeriod(tlp.size(),int32(tr.startTime) );	//トラック開始時刻を開始点に指定 
		Array<uint32> lineIter(tlp.size(),0);					//要素番号を初期化
		Array<bool> found(tlp.size(),false);					//処理済みフラグを初期化

		for( uint32 i=0; i<tlp.size(); i++ )					//各ライン
		{
			Array<LinePoint> &lp = tlp[i];
			uint32 ii=0;
			for( ; ii<lp.size(); ii++ )							//各ライン挿入点を検索
			{
				if( found[i] || lp.size() <= ii ) continue ;
				if( lp[ii].duration == 0 ) break ;				//duration=0は終端
				linePeriod[i] += lp[ii].duration;				//経過時間加算
				if( linePeriod[i] >= timeC ) break ;			//超過したら終了
			}
			found[i] = true;
			lineIter[i] = ii;
		}

		for( uint32 i=0; i<tlp.size(); i++ )					//ライン登録
		{
			Array<LinePoint> &lp = tlp[i];
			double newb = 0;
			if( linePeriod[i] == timeC )						//すでにある場合
			{
				lp[ lineIter[i]+1 ].value = GetValueI32( m_Probes[i] );			
				continue;
			}
			else if( linePeriod[i] > timeC )					//挿入
			{
				newb = linePeriod[i] - timeC;
			}
			else												//追加
			{
				newb = timeC - linePeriod[i];
				if( lp[ lineIter[i] ].duration > 0)				//0でない場合は差分
					newb = lp[ lineIter[i] ].duration - newb;
			}
			
			LinePoint lpo = {EASEINLINEAR,LINE, {}, 0};
			lpo.value = GetValueI32( m_Probes[i] );				//ｍ＿Probesは現在マーカーのあるスクリーン座標
																//登録時は３D座標にする必要ある。というかもともと座標自体を
																//３D座標で持っておき、表示時にスクリーン座標だと思う。
//if(m_Probes[i].type == U"struct s3d::Vector3D<float>") LOG_INFO(U"[{}]{}"_fmt(i,toFloat3(lpo.value)));

			if( lp.size() == lineIter[i]+1 )					//追加（現在はトラック終端=ライン終端なので追加はない）
			{
				lp << lpo;
				lp[ lineIter[i] ].duration = newb;
			}
			else												//挿入
			{
				lp.insert( lp.begin()+lineIter[i]+1, lpo);
				lp[ lineIter[i] ].duration -= newb;
				lp[ lineIter[i]+1 ].duration = newb;
			}
		}					
	}

	//制御点再生
	//この関数は計算結果を返してどこかのメンバに現在の値を残すようにして表示できるように
	void playCtrlPoint( uint32 trackno, uint32 lineno )
	{
		Track& tr = m_Tracks[trackno];
		Array<LinePoint> &lp = tr.linePoints[lineno];

//		String &type = m_Probes[lineno].type;
		void *value = m_Probes[lineno].value;

		if( m_timeCursor <= tr.startTime ) return ;				//トラック範囲外無効	
		if( m_timeCursor > tr.startTime+tr.lengthTime ) return ;

		double timeT = m_timeCursor - tr.startTime;				//カーソル位置(相対)	
		double timeA = 0;	//処理対象時間
		double timeC = 0;	//曲線時間
		uint32 beginC = 0;	//曲線開始制御点

		Array<Float3> ls;
		ls << toFloat3( lp[0].value ) ;
		uint16 type = lp[0].curveType;
		for (uint32 i = 1; i < lp.size(); i++)					//各ライン制御点を検索
		{
			timeA += lp[i-1].duration;							//経過時間を加算

			if( lp[i].curveType == type )
			{
				ls << toFloat3( lp[i].value );
				if (timeA > timeT)
				{
					if (type == LINE)
					{
						Float3 p0 = ls[ ls.size()-2 ];
						Float3 p1 = ls[ ls.size()-1 ];
						double numer = timeT - (timeA - lp[i-1].duration) ;
						double e = GetEasing(lp[i-1].easeType, numer, lp[i-1].duration);
						*(Float3*)value = Float3{ p0.lerp(p1, float(e) ) };

						break;
					}
					else							//曲線区間は確定時に処理
					{
					}
				}
			}

			else	//切り替わり発生		
			{
				if (type == LINE)	//Line→Curve
				{
					ls.clear();						//直線のLSを破棄して曲線のLSに変更
					ls << toFloat3( lp[i-1].value);	//曲線は曲線区間の終了で位置決め
					ls << toFloat3( lp[i].value);
					type = lp[i].curveType;

					timeC = timeA-lp[i-1].duration;	//曲線開始時間
					beginC = i-1;
				}

				else				//Curve→Line(曲線確定)
				{
					if (timeA > timeT)				//曲線区間でヒット
					{
						ls << toFloat3( lp[i].value );

						LineString ls2d;
						for (Float3 f3 : ls) ls2d << f3.xz();
						LineString curve = ls2d.catmullRom(5);	//区間10分割曲線補間
						uint32 div = 5;	//曲線分割数

						uint16 et = lp[ beginC ].easeType;		//イージング種別
						double dur = lp[ beginC ].duration;		//とその区間長
						double numer = timeT - timeC;						//timeTの値を
						double remapE = dur * GetEasing(et, numer, dur);	//イージング補正して
						double timeE = timeC + remapE;						//イージング済目標時間とす

						for (uint32 ii=1; ii<curve.size() ;ii++)
						{
							if ((ii-1) % div == 0)			//制御点のタイミングでイージング情報更新
							{
								et = lp[ beginC + (ii-1)/div ].easeType;	//イージング種別
								dur = lp[ beginC + (ii-1)/div ].duration;	//とその区間長
								numer = timeT - timeC;						//timeTの値を
								remapE = dur * GetEasing(et, numer, dur);	//イージング補正して
								timeE = timeC + remapE;						//イージング済目標時間とす
							}
							double len = dur / div;

							Float2 p0 = curve[ii - 1];
							Float2 p1 = curve[ii];

							if (timeC+len > timeE)					//曲線区間でヒットする制御点を検索
							{
								Float2 f2 = p0.lerp( p1, float(timeE-(timeC) / len) );
								*(Float3*)value = Float3{ f2.x, 0, f2.y };
								return ;
							}
//							Line(p0, p1).drawArrow(2,Vec2(10,10),Palette::Blue);
							timeC += len;
						}
					}

					type = lp[i].curveType;
					ls.clear();
					ls << toFloat3( lp[i].value );
				}
			}
		}
	}


	//制御点記録
	void recordCtrlPoint()
	{
		Track& tr = m_Tracks[0];
		double time = (Scene::Time()-isRecTime) *1000;
		double period = m_timeTo-m_timeFrom;
		m_timeCursor = (period < time) ? m_timeFrom : time;  
		if(period < time)	// 満了
		{
			m_timeCursor = m_timeFrom;
			isPlayTime = INACTIVE;
		}
		else                // 記録中
		{
			m_timeCursor = time;

			if( m_timePrevCtrlPoint+16 < m_timeCursor )
			{
				for( int32 i=0; i<tr.linePoints.size(); i++ )    //ライン分の情報収集
				{
					LinePoint lpo = {EASEINLINEAR, LINE ,{}, 0};
					lpo.value = GetValueI32( m_Probes[i] );			
					tr.linePoints[i].back().duration = m_timeCursor - m_timeFrom; 
					tr.linePoints[i] << lpo;
				}					
				m_timePrevCtrlPoint = m_timeCursor;
			}

			else if( m_timePrevCtrlPoint > m_timeCursor )	//カーソル位置が最初に戻り
			{
				for( int32 i=0; i<tr.linePoints.size(); i++ )
				{
					tr.linePoints[i].clear();
					LinePoint lpo = {EASEINLINEAR,LINE, {}, 0};
					lpo.value = GetValueI32( m_Probes[i] );			
					tr.linePoints[i] << lpo;
				}
				m_timePrevCtrlPoint = m_timeCursor;
			}
		}
	}

	//ラインマーカー制御
	void updateLineMarker()
	{
	    const Font &fontM = FontAsset(U"fontM");
	    const Font &iconM = FontAsset(U"iconM");
        const Transformer2D transform(Mat3x2::Identity(), Mat3x2::Translate(m_Render.pos));

		const double is = iconM.fontSize() + 1;
		const int32 fs = fontM.fontSize();
		for (int32 t = 0; t < m_Tracks.size(); t++)
		{
			Track& tr = m_Tracks[t];
			double ty = Gui[rcTRACKLIST].y + m_frameThickness + tr.intTrackYpos ;
			RectF rectT(tr.startTime, ty, tr.lengthTime, m_Tracks[tr.intTrackNo].intHeight);	//トラック領域(仮想)

			const int32 ttbh = fs*2;								//トラックツールバー高

			for (int32 i = 0;i < tr.linePoints.size();i++)
			{
				auto &lp = tr.linePoints[i];

				double by = (ty + tr.intScrollYpos + ttbh) + (i * fs * 3);
				if( by < Gui[rcTRACKLIST].y ) continue ; 
				if( by > Gui[rcTRACKLIST].y+Gui[rcTRACKLIST].h ) break ;

				double lx = rectT.x;								//最初の時間座標
				int32 ii=0;
				for( ; ii<lp.size(); ii++ )							//各ライン制御点を検索
				{
					if( lp[ii].duration == 0 ) break ;				//duration=0は終端

					double lxd = lx + lp[ii].duration;				//線分+長さ→終点

					if ( lx < m_minVR ) {}							//範囲外
					else
					{
						double clx = Clamp(lx, m_minVR, m_maxVR);	//始点範囲内クランプ
						double bx = Gui[rcDRAGBAR].x + (clx - m_minVR) * m_ratioH;	//始点描画座標

						RectF rectM = RectF{ bx - is / 2, by, is, is };

						if (rectM.mouseOver())
						{
							if (rectM.rightClicked())
							{
								Array<String> menus = { U"イージング種別:",U"カーブ指定:" };
								Array<uint32*> datas = { (uint32*)(uint16*)&lp[ii].easeType };
								m_Popup = Popup( menus, datas,
								Float2(rectM.x + 50,rectM.y - 40),12,160,16,0,20,
								Float2(1,1),Float2(8,1.5),Float2(0,0),Float2(-50,40) );
							}
							if( rectM.leftClicked() )		//マウス乗ってる位置マーカーを選択してGUI制御対象に登録
							{
								Gui[rcLINEMARK] = rectM;	//TODO:位置POSもだけど、同じ処理はクローン対策
								m_grabState.trackNo = 0;
								m_grabState.lineNo = i;
								m_grabState.markerNo = ii;
								m_grabState.area = LINEMARKER;
								m_onClickState = m_grabState;
								break ;
							}

						}
					}
					lx = lxd;											//次の線分
					if( lx > rectT.x+rectT.w || lx > m_maxVR ) break;	//トラック超過したら終了
				}
			}
		}
	}

	//ポップアップ制御
	void updatePopup()
	{
		const Font &fontM = FontAsset(U"fontM");
        const Transformer2D transform(Mat3x2::Identity(), Mat3x2::Translate(m_Render.pos));

		if( !m_Popup.m_rectUI.mouseOver() ) 
			m_Popup.m_texts.clear() ;	//はずれたらポップアップ解除

		if( m_Popup.m_texts.size() == 0 ) return ;

		//ポップアップ操作
		else
		{
			RectF row = m_Popup.m_rectUI;
			row.y += 2 ;
			uint32 fs = fontM.fontSize();
			row.h = fs ;

			//再生ボタンのポップアップWheel操作
			Array<String> &menu = m_Popup.m_texts;
			if( -1 != menu[0].indexOf(U"再生速度:") )
			{
				m_playSpeed = Clamp(m_playSpeed + Mouse::Wheel(), -100.0, 100.0);
				menu[0] = U"再生速度:{}%"_fmt(m_playSpeed);
			}

			else if( -1 != menu[0].indexOf(U"イージング種別:") )
			{
				uint16 &easetype = ((uint16 *)m_Popup.m_datas[0])[0];
				uint16 &isbezier = ((uint16 *)m_Popup.m_datas[0])[1];
				if( row.mouseOver() )
				{
					double value = (double)easetype + Mouse::Wheel();
					easetype = (uint16)Clamp( value, (double)0, (double)EaseName.size()-1);
				}
				row.y += fs;
				if( row.mouseOver() )
				{
					double value = (double)isbezier + Mouse::Wheel();
					isbezier = (uint16)Clamp( value, (double)0, (double)1);
				}
				menu[0] = U"イージング種別:{}"_fmt( EaseName[ easetype ] );
				menu[1] = U"カーブ指定:{}"_fmt( BezierEnable[ isbezier ] );
			}

			else if( -1 != menu[0].indexOf(U"カーソル送り時間:") )
			{
				m_timeForward = Clamp(m_timeForward + Mouse::Wheel(), -100.0, 100.0);
				menu[0] = U"カーソル送り時間:{}ms"_fmt(m_timeForward);
			}

			else if( -1 != menu[0].indexOf(U"リンク対象:") )
			{
				Track &tr = m_Tracks[0];
				double vel = Mouse::Wheel();
				double now = (int32)*m_Popup.m_datas[0];
				
				//あーわからん。Popupは特定の情報が何かなんてしらない。でもPopupによっては選択不可の判断が必要
				int32 value = int32(now + vel);
				int32 targetid = Clamp( value, -1, (int32)tr.lineNames.size()-1);
				if( targetid < 0 ) menu[0] = U"リンク対象:無し"; 
				else
				{
					uint32 &lineno = m_Popup.m_targetid;
					menu[0] = U"リンク対象:{}"_fmt( tr.lineNames[ targetid ] );
					Array<int32> linktarget = GetValueI32( m_Probes[ targetid ] );  //リンク対象
					tr.lineLinkProbes[lineno] = linktarget;
				}
				*m_Popup.m_datas[0] = (uint32)targetid;	//コンテキストメニュー0の参照値(linelinks)に設定
			}
		}
	}

	//ショートカット
	void updateShortcut()
	{
		if( KeyF2.up() )
		{
			isPlayTime = INACTIVE ;
			if( isRecTime == INACTIVE ) isRecTime = Scene::Time();
		}

		if( KeyF3.up() )	//トラック0に現在の情報を登録
		{
			Track &tr = m_Tracks[0];										//トラック長を修正
			if( m_timeCursor <= tr.startTime ) return ;					//トラック範囲外無効	
			if( m_timeCursor > tr.startTime+tr.lengthTime ) return ;

			registerCtrlPoint(0);			//現在の位置を登録
			m_timeCursor += m_timeForward ;

		}
	}

	//ツールバー制御
	void updateToolbar()
	{
//	    const Font &fontM = FontAsset(U"fontM");
	    const Font &iconM = FontAsset(U"iconM");
        const Transformer2D transform(Mat3x2::Identity(), Mat3x2::Translate(m_Render.pos));

		int32 is = iconM.fontSize() + 1;
		RectF btn{ Gui[rcTOOLBAR].x + m_paddingL, Gui[rcTOOLBAR].y, is, is };

		for (int32 i=0; i<m_Toolbar.size(); i++)
		{
			auto& tbi = m_Toolbar[i];

			if( btn.mouseOver() && btn.rightClicked() )
			{
				if( tbi.code == BTN_PLAY )
				{
					//m_Popupに登録するとupdatePopupでパラメータ制御できる
					Array<String> menus = {U"再生速度:{}%"_fmt(m_playSpeed) };
					Array<uint32 *> datas = {(uint32 *)&m_playSpeed };
					m_Popup = Popup ( menus, datas,
					Float2(btn.x + 50,btn.y - 10),12,160,16,0,10,
					Float2(1,1),Float2(4,1),Float2(0,0),Float2(-40,25) );
				}
			}

			if( btn.mouseOver() && btn.leftClicked() )
			{
				if( tbi.code == BTN_PLAY )
				{
					isRecTime = isStepTime = INACTIVE ;
					if( isPlayTime == INACTIVE ) isPlayTime = Scene::Time();
				}
				else if( tbi.code == BTN_STEP )
				{
					isRecTime = isPlayTime = INACTIVE ;
					if( isStepTime == INACTIVE ) isStepTime = Scene::Time();

				}
				else if( tbi.code == BTN_STOP )
				{
					isPlayTime = isRecTime = isStepTime = INACTIVE ;
				}
				else if( tbi.code == BTN_REC )
				{
					isPlayTime = isStepTime = INACTIVE ;
					if( isRecTime == INACTIVE ) isRecTime = Scene::Time();
				}
				else if( tbi.code == BTN_EDIT )
				{
				}
				else if( tbi.code == BTN_CURSOR )
				{
				}
				else if( tbi.code == BTN_FROM )
				{
				}
				else if( tbi.code == BTN_TO )
				{
				}
				else if( tbi.code == BTN_ANCHOR )
				{
				}
				else if( tbi.code == BTN_CONFIG )
				{
				}
				else if( tbi.code == BTN_LINK )
				{
				}
			}
			btn.x += btn.w;

			if(isRecTime != INACTIVE)	//録画実行中
			{
				recordCtrlPoint();
			}

			if(isPlayTime != INACTIVE || isStepTime != INACTIVE)	//再生中
			{
				if(isPlayTime != INACTIVE)	//再生中のみ自動更新 
				{
					double time = fmod((Scene::Time()-isPlayTime)*m_playSpeed*10, (m_timeTo-m_timeFrom));
					//LOG_INFO(U"timepos:{}"_fmt(time));
					if( time >= 0) m_timeCursor =  m_timeFrom + time ; 
					else  		   m_timeCursor =  m_timeTo + time ; 
				}

				playCtrlPoint(0,0);	//Posision
				playCtrlPoint(0,1);	//Camera-R
				playCtrlPoint(0,2);	//Focus-R
			}

		}
	}
	
	//トラックツールバー制御
	void updateTrackToolbar()
	{
		const Font &fontM = FontAsset(U"fontM");
		const Font &iconM = FontAsset(U"iconM");
        const Transformer2D transform(Mat3x2::Identity(), Mat3x2::Translate(m_Render.pos));
// --- ここから ---
		const int32 trackSpacing = 2;
		int32 ypos = 2;

		int32 fs = fontM.fontSize();
		double is = iconM.fontSize() + 1;

		for (int32 i = 0; i < m_Tracks.size(); ++i)
		{
			Track& tr = m_Tracks[i];
			tr.intTrackYpos = ypos;

			if (tr.intTrackNo != -1 && i != tr.intTrackNo)
			{
				tr.merged = true;
				tr.intTrackYpos = m_Tracks[tr.intTrackNo].intTrackYpos;
			}
			else ypos += tr.intHeight + trackSpacing;

			double tx = Gui[rcTRACKLIST].x + m_frameThickness;
			double ty = Gui[rcTRACKLIST].y + m_frameThickness + tr.intTrackYpos;

			//トラックリスト描画
			if (tr.merged == false)
			{
				int32 &trackheight = m_Tracks[tr.intTrackNo].intHeight;
				tr.rectCaption = RectF{ tx, ty, m_trackWidth, trackheight };

				RectF btn{ tx + fs * 10, ty, is, is };

				
	// --- ここまで、描画と同じ処理 こういうのどう記述すりゃいいんだろ

				for (int32 ii=0; ii<m_TrackToolbar.size(); ii++)
				{
					auto& tbi = m_TrackToolbar[ii];

					if( btn.mouseOver() && btn.rightClicked() )
					{
						if( tbi.code == TTB_SET )
						{
							//m_Popupに登録するとupdatePopupでパラメータ制御できる
							Array<String> menus = {U"カーソル送り時間:{}ms"_fmt(m_timeForward) };
							Array<uint32 *> datas = {(uint32 *)&m_timeForward };
							m_Popup = Popup ( menus, datas,
							Float2(btn.x+50,btn.y-10),12,160,16,0,10,
							Float2(1,1),Float2(6,1),Float2(0,0),Float2(-40,25) );
						}
					}

					if( btn.mouseOver() && btn.leftClicked() )
					{
						if( tbi.code == TTB_SET )
						{
							registerCtrlPoint(i);			//現在の位置を登録
						}
						else if( tbi.code == TTB_DEL )
						{
							deleteCtrlPoint(i);				//現在の位置を削除
						}
						else if( tbi.code == TTB_FUNC3 )
						{
						}
						else if( tbi.code == TTB_FUNC4 )
						{
						}
						else if( tbi.code == TTB_FUNC5 )
						{
						}
						else if( tbi.code == TTB_FUNC6 )
						{
						}
						else if( tbi.code == TTB_FUNC7 )
						{
						}
						else if( tbi.code == TTB_FUNC8 )
						{
						}
					}

					btn.x += btn.w;
				}

				//ラインツールバー処理
				for (uint32 ii = 0;ii < tr.lineNames.size();ii++)
				{
					const int32 ttbh = fs*2;					//トラックツールバー高
					int32 yy = tr.intScrollYpos + ii * fs * 3 + ttbh ;
					if (yy < fs*2 || yy > trackheight) continue;	//トラックラインの表示上限

					double bx = tr.rectCaption.x + m_paddingL;
					double by = tr.rectCaption.y + yy;

					RectF btn2 { bx + fs * 6, by, is, is };

					for (uint32 iii=0; iii<m_LineToolbar.size(); iii++)
					{
						auto& tbi = m_LineToolbar[iii];
						if( btn2.mouseOver() && btn2.rightClicked() )
						{
							if( tbi.code == LTB_LINK )
							{
								//m_Popupに登録するとupdatePopupでパラメータ制御できる
								Array<String> menus = {U"リンク対象:"};
								Array<uint32 *> datas = { (uint32 *)&tr.lineLinks[ii] };
								m_Popup = Popup ( menus, datas,
								Float2(btn.x+50,btn.y-10),12,160,16,0,10,
								Float2(1,1),Float2(6,1),Float2(0,0),Float2(-40,25),
								Palette::Dimgray, Palette::Bisque, ii );
							}
						}

						if( btn.mouseOver() && btn.leftClicked() )
						{
							Array<LinePoint> &lp = tr.linePoints[ii];
							if( tbi.code == LTB_BACK )
							{
								double linePeriod = tr.startTime;	//トラック開始時刻を開始点に指定 
								for( uint32 iv=0; iv<lp.size()-2; iv++ )	//各ライン挿入点を検索
								{
									if( linePeriod+lp[iv].duration >= m_timeCursor ) break ;//超過したら終了
									linePeriod += lp[iv].duration;		//経過時間加算
								}
								m_timeCursor = linePeriod;				//カーソル時間を更新
							}
							else if( tbi.code == LTB_FORWARD )
							{
								double linePeriod = tr.startTime;	//トラック開始時刻を開始点に指定 
								int32 iv=0;
								for( ; iv<lp.size()-2; iv++ )	//各ライン挿入点を検索
								{
									if( linePeriod >= m_timeCursor ) break ;//超過したら終了
									linePeriod += lp[iv].duration;			//経過時間加算
								}
								if( m_timeCursor < tr.startTime) m_timeCursor = linePeriod;
								else m_timeCursor = linePeriod + lp[iv].duration;//カーソル時間を更新
							}
							else if( tbi.code == LTB_SET )
							{
							}
							else if( tbi.code == LTB_LINK )
							{
							}
							else if( tbi.code == LTB_FUNC5 )
							{
							}
							else if( tbi.code == LTB_FUNC6 )
							{
							}
							else if( tbi.code == LTB_FUNC7 )
							{
							}
							else if( tbi.code == LTB_FUNC8 )
							{
							}
						}
						btn.x += btn.w;
					}
				}

			}
		}
	}

	//カーソル更新
	void updateCursor()
	{
//		const Font &fontS = FontAsset(U"fontS");
		const Font &iconM = FontAsset(U"iconM");

//		int32 fs = fontS.fontSize();
		double is = iconM.fontSize() + 1;
		double ty = Gui[rcTRACKLIST].y + m_frameThickness;

		RectF& rectC = Gui[rcCURSOR];
		rectC = RectF { m_timeCursor, ty, is, Gui[rcTOOLBAR].h };

		if (m_maxVR < m_timeCursor)      rectC.x = m_maxVR;
		else if (m_timeCursor < m_minVR) rectC.x = m_minVR;

		rectC.x = Gui[rcDRAGBAR].x + 
					(Clamp((double)rectC.x, m_minVR, m_maxVR) - m_minVR) * m_ratioH;
		rectC = RectF{ rectC.x - is / 2, rectC.y - Gui[rcTOOLBAR].h, is, is };
	}

	//カーソル描画
	void drawCursor()
	{
		const Font &fontM = FontAsset(U"fontM");
		const Font &iconM = FontAsset(U"iconM");
        const Transformer2D transform(Mat3x2::Identity(), Mat3x2::Translate(m_Render.pos));

		int32 fs = fontM.fontSize();
		double is = iconM.fontSize() + 1;
//		double ty = Gui[rcTRACKLIST].y + m_frameThickness;

		RectF& rectC = Gui[rcCURSOR];

		Vec2 cc = Vec2(rectC.x + is / 2, Gui[rcDRAGBAR].y + is);

		ColorF colOFF,colON;
		if(isRecTime != INACTIVE)
		{
			colOFF = ColorF(0.9, 0, 0, 0.3);
			colON = ColorF(0.3, 0, 0, 0.3);
		}
		else if(isPlayTime != INACTIVE)
		{
			colOFF = ColorF(0, 0.9, 0, 0.3);
			colON = ColorF(0, 0.3, 0, 0.3);
		}
		else if(isStepTime != INACTIVE)
		{
			colOFF = ColorF(0, 0, 0.9, 0.3);
			colON = ColorF(0, 0, 0.3, 0.3);
		}
		else
		{
			colOFF = ColorF(0.9, 0.9, 0.9, 0.3);
			colON = ColorF(0.3, 0.3, 0.3, 0.3);
		}

		iconM(U"\xF0523").draw(rectC, !m_Popup.isActive() && rectC.mouseOver() ? colON : colOFF);

		static double move = 0.0;
		move += Scene::DeltaTime() * 10;
		Line(cc, cc + Vec2(0, Gui[rcFRAME].h-is)).draw(LineStyle::SquareDot(move), 2, colOFF);
		String duration = U"{}"_fmt((uint32)m_timeCursor);
		fontM(duration).draw(Arg::center(cc.x, cc.y - fs));
	}

	//領域選択更新
	void updateSelection()
	{
//		const Font &fontS = FontAsset(U"fontS");
		const Font &iconM = FontAsset(U"iconM");

//		int32 fs = fontS.fontSize();
		double is = iconM.fontSize() + 1;
		double ty = Gui[rcTRACKLIST].y + m_frameThickness;

		RectF rectS = RectF{ m_timeFrom, ty, m_timeTo - m_timeFrom, ty  };
		if (m_maxVR < m_timeFrom) rectS.x = m_maxVR; //範囲外
		if (m_timeTo < m_minVR) rectS.x = m_minVR;
		double lb = (rectS.x < m_minVR) ? m_minVR - rectS.x : 0;
		double rb = (rectS.x + rectS.w > m_maxVR) ? (rectS.x + rectS.w) - m_maxVR : 0;
		rectS.w = (rectS.w - lb - rb) * m_ratioH;
		rectS.x = Gui[rcDRAGBAR].x + (Clamp((double)rectS.x, m_minVR, m_maxVR) - m_minVR) * m_ratioH;

		Gui[rcLMARK] = RectF{ rectS.x-is ,ty-is+5,is, is };
		Gui[rcRMARK] = RectF{ rectS.x+ rectS.w ,ty-is+5,is, is };
	}

	//領域選択描画
	void drawSelection()
	{
	    const Font &fontM = FontAsset(U"fontM");
	    const Font &fontS = FontAsset(U"fontS");
	    const Font &iconM = FontAsset(U"iconM");
        const Transformer2D transform(Mat3x2::Identity(), Mat3x2::Translate(m_Render.pos));

		int32 fs = fontS.fontSize();
		double is = iconM.fontSize() + 1;
//		double ty = Gui[rcTRACKLIST].y + m_frameThickness;

		RectF& rectL = Gui[rcLMARK];
		RectF& rectR = Gui[rcRMARK];

		ColorF colOFF = ColorF(0.9, 0.9, 0.9, 0.3);
		ColorF colON = ColorF(0, 0, 0, 0.3);

		static double move = 0.0;
		move += Scene::DeltaTime() *-10;

		Vec2 ll = Vec2(rectL.x + is, Gui[rcDRAGBAR].y + is);
		iconM(U"\xF0367").draw(rectL, !m_Popup.isActive() && rectL.mouseOver() ? colON : colOFF);
		Line(ll, ll + Vec2(0, Gui[rcFRAME].h-is)).draw(LineStyle::SquareDot(move), 2, colON);
		String timeL = U"{}"_fmt((uint32)m_timeFrom);
		fontM(timeL).draw(Arg::center(ll.x, ll.y+fs+14));

		Vec2 rr = Vec2(rectR.x , Gui[rcDRAGBAR].y + is);
		iconM(U"\xF0361").draw(rectR, !m_Popup.isActive() && rectR.mouseOver() ? colON : colOFF);
		Line(rr, rr + Vec2(0, Gui[rcFRAME].h-is)).draw(LineStyle::SquareDot(move), 2, colON);
		String timeR = U"{}"_fmt((uint32)m_timeTo);
		fontM(timeR).draw(Arg::center(rr.x, rr.y + fs+14));
	}

	//タイムライン処理
	void main()
	{
		if (KeyPause.up()) m_Hidden = !m_Hidden;
		if (m_Hidden ) return;

		m_RenderTexture.clear(ColorF{0.8, 0.9,1.0,1.0});	// RTをクリア

		update();
		draw();

		Graphics2D::Flush();								// RTに描画
		m_RenderTexture.resolve();							// スムージング
		m_RenderTexture.draw(m_Render.pos);					// RTを描画
	}
};
