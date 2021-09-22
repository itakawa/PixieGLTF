# include <Siv3D.hpp> // OpenSiv3D v0.6
# include <Siv3D/EngineLog.hpp>


# include "Timeline3.hpp"
# include "PixieGLTF.hpp"
# include "PixieCamera.hpp"
# include "LineString3D.hpp"
 
# include <omp.h>
# include <algorithm>
# include <execution>
#include <thread>

enum IDAREA {  M_RELEASE, 
               M_ACT, M_EYE, M_FOCUS, 
               M_POSLINE,
               M_XY, M_XZ, M_YZ,M_ARROW,
               NUMGUI };
struct GRABSTATE
{
    IDAREA area ;
    int32 value ;
    uint32 trackNo;		//トラック番号の識別用
    uint32 lineNo;		//ライン番号の識別用
    uint32 markerNo;	//マーカー番号の識別用
    Float3 prevPos;     //マーカー3D座標
} ;

GRABSTATE grabState { M_RELEASE,0 };      //テスト(パラメータ)初期化
GRABSTATE onClickState { M_RELEASE, 0 };   //クリック時のトラック、ライン、マーカーを控えて混在除け
RectF Gui[NUMGUI] ;

//2Dマーカーの初期位置
void InitGUI()
{
    Gui[M_ACT] = Rect{ 0, 0, 30, 30 };
    Gui[M_EYE] = Rect{ 0, 0, 30, 30 };
    Gui[M_FOCUS] = Rect{ 0, 0, 30, 30 };
}

//マーカー操作
template <typename V> 
V guiDrag(const PixieCamera &cam, const OrientedBox &ob, const String &code, RectF &rectgui,GRABSTATE &grabstate, IDAREA garea, 
          ColorF activecolor,ColorF inactivecolor, V pos3d )
{
    const Font &iconM = FontAsset(U"iconM");

    Float2 pos2d = rectgui.pos;
    if (grabstate.area == M_RELEASE && rectgui.leftClicked()) grabstate.area = garea;
    if (grabstate.area == garea)
    {
        pos2d += Cursor::DeltaF();
        rectgui.x = pos2d.x;
        rectgui.y = pos2d.y;
        const Ray mouseRay = cam.screenToRay(pos2d);
	    if (const auto depth = mouseRay.intersects(ob))//衝突あり
	    {
		    Cursor::RequestStyle(CursorStyle::Hand);
            Vec3 pos = mouseRay.point_at(*depth);
            pos3d.x = pos.x;
            pos3d.y = pos.y;
            pos3d.z = pos.z;
        }
    }

    iconM(code).draw(Arg::center( rectgui.x,rectgui.y ),rectgui.mouseOver() ? activecolor : inactivecolor );
    return pos3d;
}

//2Dスクロール
Vec3 guiTrack(const Font &font,const String &text, Rect &rectgui,GRABSTATE &grabstate, IDAREA garea, 
          ColorF activecolor,ColorF inactivecolor )
{
    Vec3 value = Vec3(0,0,0);
    font(text).draw(Arg::center(rectgui.x,rectgui.y),(grabstate.area == garea || rectgui.mouseOver() ) ? activecolor : inactivecolor );
    if (grabstate.area == M_RELEASE && rectgui.leftClicked()) grabstate.area = garea;
    if (grabstate.area == garea) { value = Vec3(Cursor::Delta() / 4, 0); }
    if (rectgui.mouseOver()) { value.z = Mouse::Wheel(); }
    return value;
}

//GLTFメッシュ初期化

PixieGLTF meshArrow, meshXZ, meshXY, meshYZ, meshCam, meshGnd, meshStd, meshMarkA, meshMarkS,
          meshMarkF, meshSP, meshVRM, meshGrid, meshCUI;

//可動ボーン
int32 boneHead_01 = -1 ;
int32 boneNeck_01 = -1 ;
int32 boneEye_L_01 = -1 ;
int32 boneEye_R_01 = -1 ;
int32 boneOrigin_00 = -1 ;
int32 boneHip_01 = -1 ;
int32 boneSpine_01 = -1 ;
int32 boneArm_L_01 = -1 ;
int32 boneArm_L_02 = -1 ;
int32 boneArm_L_03 = -1 ;
int32 boneArm_R_01 = -1 ;
int32 boneArm_R_02 = -1 ;
int32 boneArm_R_03 = -1 ;
int32 boneLeg_L_01 = -1 ;
int32 boneLeg_L_02 = -1 ;
int32 boneLeg_R_01 = -1 ;
int32 boneLeg_R_02 = -1 ;

void InitGltf()
{						//glTFファイル  座標               拡縮率(1=100%)     オイラー回転(RPY)  相対座標
//  gltfTokyo = PixieGLTF(U"533945361_tokyo.glb",Float3{ 0, 0, 0 }, Float3{ 10, 10, 10 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });
//	gltfT19   = PixieGLTF(U"tutorial_19.glb",Float3{ 5, 0, 5 }, Float3{ 10, 10, 10 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });
//	gltfAtami = PixieGLTF(U"atami.glb",Float3{ 5, 0, 5 }, Float3{ 10, 10, 10 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });
//	gltfVRM =   PixieGLTF(U"Sinobu.011b.vrm", Float3{ 0, 0, 0 }, Float3{ 1, 1, 1 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });

	meshGrid =  PixieGLTF(U"Grid.glb", Float3{ 0, 0, 0 }, Float3{ 1, 1, 1 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });
	meshSP =    PixieGLTF(U"MarkSP.glb", Float3{ 0, 0, 0 }, Float3{ 5, 5, 5 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });
	meshArrow = PixieGLTF(U"Arrow.glb", Float3{ 0, 0, 0 }, Float3{ 1, 1, 1 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });

	meshXZ =    PixieGLTF(U"XZ.glb", Float3{ 0, 0, 0 }, Float3{ 1, 1, 1 }, Float3{ 0,180, 0 }, Float3{ 0, 0, 0 });
    meshXY =    PixieGLTF(U"XY.glb", Float3{ 0, 0, 0 }, Float3{ 1, 1, 1 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });
    meshYZ =    PixieGLTF(U"YZ.glb", Float3{ 0, 0, 0 }, Float3{ 1, 1, 1 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });

	meshCam =   PixieGLTF(U"Camera.glb", Float3{ 0, 0, 0 }, Float3{ 1, 1, 1 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });
    meshGnd =   PixieGLTF(U"TF2.002.glb", Float3{ 40, 0, -50 }, Float3{ 10, 10, 10 }, Float3{ 0, 90, 0 }, Float3{ 0, 0, 0 });
    meshMarkA = PixieGLTF(U"Sinobu.016A860.glb",Float3{ 0, 0, 0 }, Float3{ 1, 1, 1 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });
    meshMarkS = PixieGLTF(U"Sinobu.012.glb",Float3{ 2, 0, 0 }, Float3{ 1, 1, 1 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });
    meshMarkF = PixieGLTF(U"Triangle.glb",Float3{ 0, 0, 0 }, Float3{ 1, 1, 1 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });

	//　オブジェクト生成  フォントファイル	座標lPos(0,0,0)　 拡縮率lSca(x2,x2,x2)   オイラーPYR角eRot	　相対座標rPos(0,0,0)              																	
    meshCUI  = PixieGLTF(U"ToD4Font.glb",Float3{ 0, 0, 0 }, Float3{ 2, 2, 2 }, Float3{ 180, 0, 0 }, Float3{ 0, 0, 0 });

	meshVRM =   PixieGLTF(U"Siv3DKun2.glb", Float3{ 0, 100, 0 }, Float3{ 20, 20, 20 }, Float3{ 0, 180, 0 }, Float3{ 0, 0, 0 });
//	meshVRM =   PixieGLTF(U"Sinobu.013.vrm", Float3{ 0, 90, -50 }, Float3{ 30, 30, 30 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });
//	meshVRM =   PixieGLTF(U"Sinobu.012.glb", Float3{ 0, 100, 0 }, Float3{ 20, 20, 20 }, Float3{ 0, 0, 0 }, Float3{ 0, 0, 0 });

	bool OBBDEBUG = true;
//        gltfTokyo.initModel(MODELTYPE::MODELNOA); 
//        gltfT19.initModel( MODELTYPE::MODELANI,200 );      //60fpsが速度100％
//        gltfAtami.initModel( MODELTYPE::MODELNOA ); 

MillisecClock ms0;
#if 1  //順次処理
		meshGrid.initModel(MODELTYPE::MODELNOA); 
		meshArrow.initModel(MODELTYPE::MODELNOA); 
		meshSP.initModel(MODELTYPE::MODELNOA); 
		meshXZ.initModel(MODELTYPE::MODELNOA); 
		meshXY.initModel(MODELTYPE::MODELNOA); 
		meshYZ.initModel(MODELTYPE::MODELNOA); 
		meshCam.initModel(MODELTYPE::MODELNOA);
		meshGnd.initModel(MODELTYPE::MODELNOA); 
		meshMarkS.initModel(MODELTYPE::MODELNOA); 
		meshMarkF.initModel(MODELTYPE::MODELNOA); 
		meshVRM.initModel(MODELTYPE::MODELVRM);
		meshCUI.initModel(MODELTYPE::MODELNOA);

#else	//スレッド並列(少し早い？)
{
	const int32 NTHREAD = 12;
	std::thread th[NTHREAD];
	th[0] = std::thread { [&] {meshArrow.initModel(MODELTYPE::MODELNOA); } } ;
	th[1] = std::thread { [&] {meshSP.initModel(MODELTYPE::MODELNOA); } } ;
	th[2] = std::thread { [&] {meshXZ.initModel(MODELTYPE::MODELNOA);; } };
	th[3] = std::thread { [&] {meshXY.initModel(MODELTYPE::MODELNOA); } } ;
	th[4] = std::thread { [&] {meshYZ.initModel(MODELTYPE::MODELNOA); } } ;
	th[5] = std::thread { [&] {meshCam.initModel(MODELTYPE::MODELNOA); } } ;
	th[6] = std::thread { [&] {meshGnd.initModel(MODELTYPE::MODELNOA); } } ;
	th[7] = std::thread { [&] {meshMarkS.initModel(MODELTYPE::MODELNOA); } } ;
	th[8] = std::thread { [&] {meshMarkF.initModel(MODELTYPE::MODELNOA); } } ;
	th[9] = std::thread { [&] {meshVRM.initModel(MODELTYPE::MODELVRM); } } ;
	th[10]= std::thread { [&] {meshGrid.initModel(MODELTYPE::MODELNOA); } } ;
	th[11]= std::thread { [&] {meshBB.initModel(MODELTYPE::MODELNOA); } } ;
	for (int32 i = 0; i < NTHREAD; i++) th[i].join();
}
#endif
LOG_INFO( U"ワーカースレッド処理時間＝{}ms"_fmt(ms0.ms()) ); // ログ出力


/*
	{
		//1000個のスタジアム(7万頂点)
		const auto NUM = 100;
		static PixieGLTF meshes1[NUM];
		static PixieGLTF meshes2[NUM];
		for (auto i = 0; i < NUM; i++) 
		{
	        std::string err, warn;
			bool result;
			tinygltf::TinyGLTF loader;
	        meshes1[i] = PixieGLTF(U"TF2.002.glb");
	        meshes2[i] = PixieGLTF(U"TF2.002.glb");
			result = loader.LoadBinaryFromFile(&meshes1[i].gltfModel, &err, &warn, "TF2.002.glb");
			if (result == false)
				LOG_INFO(U"LOADエラー");
			result = loader.LoadBinaryFromFile(&meshes2[i].gltfModel, &err, &warn, "TF2.002.glb");
			if (result == false)
				LOG_INFO(U"LOADエラー");
		}

		std::thread threads[NUM];
		MillisecClock ms0;
		for (auto i = 0; i < NUM; i++) threads[i] = std::thread{ [&] {meshes1[i].gltfSetupNOA(); } };
		for (auto i = 0; i < NUM; i++) threads[i].join();
		ms0.log(); // 経過時間ログ出力

		MillisecClock ms1;
		for (auto i = 0; i < NUM; i++) meshes2[i].gltfSetupNOA(); 
		ms1.log(); // 経過時間ログ出力
	}
*/


//アニメ生成は内部でOMP使う
//	std::thread th = std::thread{ [&] {meshMarkA.initModel(MODELTYPE::MODELANI, false, 860, 0); } };	th.join();

	boneHead_01 = meshVRM.gltfGetJoint( U"Head_01" );
	boneNeck_01 = meshVRM.gltfGetJoint( U"Neck_01" );
	boneEye_L_01 = meshVRM.gltfGetJoint( U"Eye_L_01" );
	boneEye_R_01 = meshVRM.gltfGetJoint( U"Eye_R_01" );
	boneOrigin_00 = meshVRM.gltfGetJoint( U"Origin_00" );
	boneHip_01 = meshVRM.gltfGetJoint( U"Hip_01" );
	boneSpine_01 = meshVRM.gltfGetJoint( U"Spine_01" );
	boneArm_L_01 = meshVRM.gltfGetJoint( U"Arm_L_01" );
	boneArm_L_02 = meshVRM.gltfGetJoint( U"Arm_L_02" );
	boneArm_L_03 = meshVRM.gltfGetJoint( U"Arm_L_03" );
	boneArm_R_01 = meshVRM.gltfGetJoint( U"Arm_R_01" );
	boneArm_R_02 = meshVRM.gltfGetJoint( U"Arm_R_02" );
	boneArm_R_03 = meshVRM.gltfGetJoint( U"Arm_R_03" );
	boneLeg_L_01 = meshVRM.gltfGetJoint( U"Leg_L_01" );
	boneLeg_L_02 = meshVRM.gltfGetJoint( U"Leg_L_02" );
	boneLeg_R_01 = meshVRM.gltfGetJoint( U"Leg_R_01" );
	boneLeg_R_02 = meshVRM.gltfGetJoint( U"Leg_R_02" );
}

// プローブ対象
Float3 actorPos = { -5,  0, -0.2 };  //人物_座標
Float3 actorRot = {  0,  0, 0 };  //人物_回転
Float3 actorSca = {  1,  1, 1 };  //人物_拡縮

Float4 eyePosR   = { -5, 0, -0.6, 0};
Float3 focusPosR = { -5, 0, -0.4 };

// ウィンドウサイズ
Rect rectWindow = { 0, 0, 1900, 1000 };
Float4 eyePosGUI   = { 0, +50, 0, 0}; //視点 XYZは座標、Wはロール(オイラー角)を格納
Float3 focusPosGUI = { 0,   0, 0.001 };//注視点 ※TODO：視点-注視点が軸平行でVP変換すると頂点座標が全部中心に収束してしまい表示されない

Rect rectCUI = { 0, 0, 1900, 1000 };
Float4 eyePosCUI   = { 77,272, 80.000, 0};
Float3 focusPosCUI = { 77,222, 80.001};

Rect rectVRM = { -750, 250, 1900, 1000 };
Float4 eyePosVRM   = { 0, +10, +20, 0}; 
Float3 focusPosVRM = { 0, +10, 0};

//カメラ
PixieCamera cameraR, cameraGUI, cameraVRM, cameraCUI; 

enum PROBE_ID { PRB_ACT, PRB_CAM, PRB_FOCUS, PRB_ACTR, PRB_ACTS } ;

void InitTimeline( Timeline &timeline, Rect rect = {0,0,400,300}, bool hidden=true )
{
    const Array<Timeline::Probe> params_rtti = {
        { (void*)&actorPos,  GetTypeName(actorPos), U"ActorPos",  Color(U"#AAAAAA") },
        { (void*)&eyePosR,   GetTypeName(eyePosR),  U"CameraPosR",Color(U"#FF8888") },
        { (void*)&focusPosR, GetTypeName(focusPosR),U"FocusPosR", Color(U"#CC8888") }, 
        { (void*)&actorRot,  GetTypeName(actorRot), U"ActorRot",  Color(U"#AAAAAA") },
        { (void*)&actorSca,  GetTypeName(actorSca), U"ActorSca",  Color(U"#AAAAAA") },
    };

    timeline.setup( rect ,201 );
    timeline.setProbe( U"Catmullrom-3D",0, 170, Color(U"#334433"), params_rtti, 50, 100 );
	timeline.setHidden( hidden );
}


// GRID描画
void drawGrid( const PixieCamera &cam )
{
    int32 len = 40;
    for (int32 i = -len; i < len; i += 1)
    {
        cam.drawLine( Float3{-len,0,   i}, Float3{+len,0,   i }, 1, (i == 0) ? ColorF(0) : ColorF(0.8));
        cam.drawLine( Float3{   i,0,-len}, Float3{   i,0,+len }, 1, (i == 0) ? ColorF(0) : ColorF(0.8));
    }
}

//カメラ位置ライン描画
void drawPosline(PixieCamera& cam, Timeline& timeline, uint32 trackno, uint32 lineno, String code, PixieGLTF* model = nullptr)
{
    const Font& iconS = FontAsset(U"iconS");
    const Rect rectdraw = cam.getSceneRect();

    static double move = 0.0;
    move += Scene::DeltaTime() * 10;

    RasterizerState rs = RasterizerState::Default2D;
    rs.scissorEnable = true;
    rs.cullMode = CullMode::Back;
    Graphics2D::SetScissorRect(rectdraw);
    ScopedRenderStates2D renderState(rs);

    Timeline::Track& tr = timeline.m_Tracks[0];
    Array<Timeline::LinePoint>& lp = tr.linePoints[lineno];

    ColorF lc = tr.lineColors[lineno];
    int32 is = iconS.fontSize() + 1;
    Rect mark;

    LineString ls;
    LineString3D ls3;

    ls << cam.worldToScreenPoint(toFloat3(lp[0].value));
    ls3 << toFloat3(lp[0].value);
    uint16 type = lp[0].curveType;
    for (int32 i = 1; i < lp.size(); i++)			//各ライン制御点を検索
    {
        if (lp[i].curveType == type)
        {
            ls << cam.worldToScreenPoint(toFloat3(lp[i].value));
            ls3 << toFloat3(lp[i].value);
        }

        else	//切り替わり						
        {
            if (type == Timeline::CurveType::LINE)	//Line→Curve※登録せず線引きして１こ前からカーブに変更
            {
                ls.draw(LineStyle::SquareDot(move), 2);
                ls.clear();
                ls3.clear();

                Float3 b = toFloat3(lp[i - 1].value);
                Float3 e = toFloat3(lp[i].value);
                ls << cam.worldToScreenPoint(b);
                ls << cam.worldToScreenPoint(e);
                ls3 << b;
                ls3 << e;

                type = lp[i].curveType;
            }

            else				//Curve→Line
            {
                Float3 e = toFloat3(lp[i].value);
                ls << cam.worldToScreenPoint(e);
                ls3 << e;
//                ls.catmullRom(10).draw(LineStyle::SquareDot(move), 1);
//                ls3.catmullRom(10).draw(cam.getViewProj(),LineStyle::SquareDot(move), 2);//曲線の中間点は分割+線形補間？
                LineString ls2;
                for(const Vec3& v3: ls3.catmullRom(10))
                    ls2 << cam.worldToScreenPoint(v3);
                ls2.draw(LineStyle::SquareDot(move), 5);
//                for(uint32 i = 1; i < ls2.size(); i++ )
//                    Line(ls2[i-1],ls2[i]).drawArrow(5);

                ls.clear();
                ls3.clear();

                Float3 b = toFloat3(lp[i].value);
                ls << cam.worldToScreenPoint(b);
                ls3 << b;
                type = lp[i].curveType;
            }
        }
    }

    if (ls.size())
    {
        if (type == Timeline::CurveType::CURVE)
        {
//            ls.drawCatmullRom(LineStyle::SquareDot(move), 1);
//            ls3.catmullRom(10).draw(cam.getViewProj(),LineStyle::SquareDot(move), 2);
            LineString ls2;
            for(const Vec3& v3: ls3.catmullRom(10))
                ls2 << cam.worldToScreenPoint(v3);
            ls2.draw(LineStyle::SquareDot(move), 2);
//            for(uint32 i = 1; i < ls2.size(); i++ )
//                Line(ls2[i-1],ls2[i]).drawArrow(5);
        }

        else
        {
            ls.draw(LineStyle::SquareDot(move), 2);
        }
    }

//    ls3.clear();

    //マーカー描画
//    LOG_INFO(U"----");
    for (int32 i = 0; i < lp.size(); i++)					//各ライン制御点を検索
    {
        Float2 markpos = cam.worldToScreenPoint(toFloat3(lp[i].value));
        mark = { (int32)markpos.x,(int32)markpos.y,is,is };
        iconS(code).draw(Arg::center(mark.x, mark.y), mark.mouseOver() ? ColorF(1) : lc);

//        ls3 << toFloat3(lp[i].value);
//        LOG_INFO(U"{}: {}"_fmt(markpos, toFloat3(lp[i].value)));
    }
/*
    LineString ls2;
    ls3 = ls3.catmullRom();
    for(const Vec3& v3: ls3)
        ls2 << cam.worldToScreenPoint(v3);
    ls2.draw(LineStyle::SquareDot(move), 5);
*/
    //モデル描画
    if (model != nullptr)
    {
		if (model->modelType == MODELANI)
		{
			static int32 cnt = 0;
			model->drawAnime();
//			if ((++cnt&15)==0)
				model->nextFrame(0);
		}
        else model->setCamera( cam ).drawMesh();
    }
}

// 描画
void Draw( const PixieCamera &cam, Timeline &timeline )
{
		const ScopedRenderStates3D sr3{ SamplerState::RepeatAniso, RasterizerState::SolidCullFront };

//        meshGnd.drawMesh();

//        drawGrid( cam );                    // GRIDを描画

        drawPosline( cameraGUI, timeline, 0, PRB_ACT,  U"\xF0A10", &meshMarkA );//Actor
        drawPosline( cameraGUI, timeline, 0, PRB_CAM,  U"\xF0104" );	        //Camera-R
        drawPosline( cameraGUI, timeline, 0, PRB_FOCUS,U"\xF0E96" );            //Focus-R

//        meshMarkS.drawMesh();
//        meshMarkF.drawMesh();
//        meshBB.drawMesh();

		//暫定でモデルのカメラにVPを登録→OBBボックス描画(2D)
		meshArrow.drawMesh();
        meshSP.drawMesh();

		//                     円形文字列                  半径 カーニング  色 開始文字 文字数
		meshCUI.drawString(U"円:ABCDEFGHIJKLMNOPQRSTUVWXYZ", 30, 10, ColorF{ 1 }, 0, 26);






























		//Colorの４番目の引数が透明度でないと、違和感すごい
		const ColorF WHITE(1,1,1,USRCOLOR), RED(1,0.5,0.5,USRCOLOR),
			                              GREEN(0.5,1,0.5,USRCOLOR),
			                               BLUE(0.5,0.5,1,USRCOLOR) ;
        const Ray mouseRay = cam.screenToRay(Cursor::PosF());

		meshXY.setCamera(cam);
		if (grabState.area == M_XY)
		{
			meshGrid.eRot = { 0,90,90 };
			meshGrid.drawMesh();
			meshXY.drawMesh(WHITE);
		}
		else if (mouseRay.intersects(meshXY.ob)) meshXY.drawMesh(GREEN);
		else									 meshXY.drawMesh();

		meshYZ.setCamera(cam);
		if (grabState.area == M_YZ)
		{
			meshGrid.eRot = { 90,90,0 };
			meshGrid.drawMesh();
			meshYZ.drawMesh(WHITE);
		}
		else if (mouseRay.intersects(meshYZ.ob)) meshYZ.drawMesh(BLUE);
		else									 meshYZ.drawMesh();

		meshXZ.setCamera(cam);
		if (grabState.area == M_XZ)
		{
			meshGrid.eRot = { 0,0,0 };
			meshGrid.drawMesh();
			meshXZ.drawMesh(WHITE);
		}
		else if( mouseRay.intersects(meshXZ.ob)) meshXZ.drawMesh( RED );
		else									 meshXZ.drawMesh();

		Graphics3D::Flush();		//3Dを明示的に描画すると3D→2Dの順に描画(OBBのBOXは2D描画)
}

// キーボードショートカット操作
void updateShortcut(PixieCamera& camera)
{
	float speed = 1.01f;
	if (KeyLShift.pressed()) speed *= 5;                    //Shiftで5倍速

	if (KeyW.pressed()) camera.dolly(+speed,true);
	if (KeyA.pressed()) camera.trackX(+speed);
	if (KeyS.pressed()) camera.dolly(-speed,true);
	if (KeyD.pressed()) camera.trackX(-speed);

	if (KeyE.pressed()) camera.craneY(+speed);
	if (KeyX.pressed()) camera.craneY(-speed);
	if (KeyQ.pressed()) camera.tilt(+speed/100);
	if (KeyZ.pressed()) camera.tilt(-speed/100);

	if (KeyLeft.pressed()) camera.arcballX(+speed/100);
	if (KeyRight.pressed()) camera.arcballX(-speed/100);
	if (KeyUp.pressed())  camera.arcballY(+speed/100);
	if (KeyDown.pressed()) camera.arcballY(-speed/100);

	camera.setView();
}


//マウス操作
void updateMouse( const PixieGLTF &model, Float3 &focuspos, Float4 &eyepos )
{
    static Float3 point3D;

    if (grabState.area != M_RELEASE) return;        //何か操作中の時は処理キャンセル

    bool update = false;
    Float2 delta = Cursor::DeltaF();
    Float3 vector = eyepos.xyz() - focuspos;
    Float2 point2D = Cursor::PosF();

    if ( KeyLControl.pressed() || MouseM.pressed() ) //中ボタンウィール：拡縮
    {
        cameraGUI.dolly( (float)Mouse::Wheel() );
    }

    if ( MouseR.pressed() )                         //右ボタンドラッグ：平行移動
    {
        cameraGUI.trackX(delta.x/10);
        cameraGUI.craneY(delta.y/10);
		LOG_INFO(U"CAM_POS:{}"_fmt(cameraGUI.getEyePosition()));
    }

	updateShortcut( cameraGUI );
	updateShortcut( cameraCUI );
	cameraGUI.setUpDirection(Float3{0,1,0}).setView();	//カメラをGraphic3Dのビューに反映

    if ( MouseM.down() )                            //中ボタンドラッグ：回転
    {
        const Ray mouseRay = cameraGUI.screenToRay(Cursor::PosF());
        if (const auto depth = mouseRay.intersects( model.ob ))
        {
		    point3D = mouseRay.point_at(*depth);    //ポイントした3D座標を基点
        }
    }

    Mat4x4 mrot=Mat4x4::Identity();
    if ( MouseM.pressed() )
    {
        float speed = 1;
        Float4 ve = {0,0,0,0};  // 視点移動量
        Float3 vf = {0,0,0};    // 注視点移動量
        if (KeyLShift.pressed()) { speed = 5; ve *= speed; vf *= speed; }           //Shiftで5倍速

        cameraGUI.arcballX(delta.x/100);
        cameraGUI.arcballY(delta.y/100);
        cameraGUI.setUpDirection(Float3{0,1,0}).setView();
    }

    if ( update )
    {
    }
}

//モーフィングタイミングテーブル
#define FRAME 16
#define RE(NN) ((float)NN/(float)FRAME)
Array<float> WEIGHTTRANSITION =
{ RE(0),RE(1),RE(2),RE(3) ,RE(4) ,RE(5), RE(6), RE(7), RE(8), RE(9), RE(10),RE(11),RE(12),RE(13),RE(14),RE(15),-1 };

Array<float> WEIGHTBLINK =
{ RE(0), RE(1), RE(2), RE(3) ,RE(4) ,RE(5), RE(6), RE(7), RE(8), RE(9), RE(10),RE(11),RE(12),RE(13),RE(14),RE(15),
  RE(14),RE(13),RE(12),RE(11),RE(10),RE(9), RE(8), RE(7), RE(6), RE(5), RE(4),RE(3),RE(2),RE(1),RE(0),-1 };

void setFacial(int32 morphch, PixieGLTF &pg, int32 mtstate, float speed, Array<float> weighttable)
{
    auto& spd = pg.morphTargetInfo[morphch].Speed;
    auto& cnt = pg.morphTargetInfo[morphch].CntSpeed;
    auto& now = pg.morphTargetInfo[morphch].NowTarget;
    auto& dst = pg.morphTargetInfo[morphch].DstTarget;
    auto& idx = pg.morphTargetInfo[morphch].IndexTrans;
    auto& wt  = pg.morphTargetInfo[morphch].WeightTrans;

    if (now != mtstate && idx == 0)
    {
        if (morphch) now = 0;	//NowTargetが-1の場合はモーフ無効
        spd = speed;
        cnt = 0;
        dst = mtstate;
        idx = 1;				//IndexTransが-1の場合はWeightTrans[0]でマニュアルウェイト
        wt = (weighttable[0] == -1) ? WEIGHTTRANSITION : weighttable;
        wt.insert(wt.begin(), 0);
    }
}

enum MORPHCH {FACIAL=0,MOUTH=1,EYE=2};

void updateFacial()
{
    float morphspd = 1.0;
	if (Key0.pressed()) setFacial(FACIAL, meshVRM, 0,  morphspd, WEIGHTTRANSITION);
	if (Key1.pressed()) setFacial(MOUTH,  meshVRM, 1,  morphspd, WEIGHTBLINK);
	if (Key2.pressed()) setFacial(MOUTH,  meshVRM, 2,  morphspd, WEIGHTBLINK);
	if (Key3.pressed()) setFacial(EYE,    meshVRM, 3,  morphspd, WEIGHTBLINK);
	if (Key4.pressed()) setFacial(EYE,    meshVRM, 14, morphspd, WEIGHTBLINK);
	if (Key5.pressed()) setFacial(FACIAL, meshVRM, 10, morphspd, WEIGHTTRANSITION);
	if (Key6.pressed()) setFacial(FACIAL, meshVRM, 11, morphspd, WEIGHTTRANSITION);
	if (Key7.pressed()) setFacial(FACIAL, meshVRM, 12, morphspd, WEIGHTTRANSITION);
	if (Key8.pressed()) setFacial(FACIAL, meshVRM, 8,  morphspd, WEIGHTTRANSITION);
	if (Key9.pressed()) setFacial(FACIAL, meshVRM, 14, morphspd, WEIGHTTRANSITION);

    //カメラは注視点と視点の組み合わせで座標だけで角度が決まるがロール表現ができない
    //ロール変化を動画でやると酔うのであまりやらないが、できないのは残念なので、使わないけど入れる必要ある
    //カメラは注視点と合わせてロール以外の角度を持つので視点の座標をFloat3→Float4として、ｗにロール角を保持
}

void updateMorph(PixieGLTF &pg)
{
	//モデルが持つすべてのモーフィング毎に制御
    for (uint32 ii = 0; ii < pg.morphTargetInfo.size(); ii++)
    {
        auto& spd = pg.morphTargetInfo[ii].Speed;
        auto& cnt = pg.morphTargetInfo[ii].CntSpeed;
        auto& now = pg.morphTargetInfo[ii].NowTarget;
        auto& dst = pg.morphTargetInfo[ii].DstTarget;
        auto& idx = pg.morphTargetInfo[ii].IndexTrans;
        auto& wt  = pg.morphTargetInfo[ii].WeightTrans;

        if (now == -1) continue;

        cnt += spd;                 //モーフ変化量を加算
        if (cnt >= 1.0) cnt -= 1.0;
        else continue;

        if (now > -1 && idx)
        {
            idx++;
            if (wt[idx] < 0)
			{
                idx = 0;
                if (ii == FACIAL) now = dst; //プライマリ(表情)の場合は遷移する
                else              now = -1;  //セカンダリ(瞬き/口パク)の場合は遷移しない
            }
        }
    }
}



void updatePosline( const PixieCamera &cam, const PixieGLTF& model, Timeline &timeline, uint32 trackno, uint32 lineno )
{
//	const Font &fontM = FontAsset(U"fontM");
    const Font &iconM = FontAsset(U"iconM");

    static PixieCamera basecam ;
	static Vec2 pointBegin;
	static Float3 pointRot;
	static Quaternion nowQ,oldQ;


	int32 is = iconM.fontSize()+1;
	Timeline::Track& tr = timeline.m_Tracks[ 0 ];

    if ( grabState.area == M_POSLINE )    //左ドラッグ処理
    {
    	auto &lp = tr.linePoints[ grabState.lineNo ];
		uint32 &i = grabState.markerNo;
        const Ray mouseRay = cam.screenToRay(Cursor::PosF());
	    if (const auto depth = mouseRay.intersects(model.ob))//衝突あり
	    {
		    Cursor::RequestStyle(CursorStyle::Hand);
		    Float3 pos3 = mouseRay.point_at(*depth);

			lp[i].value[0] = *(uint32 *)&pos3.x;
			lp[i].value[1] = *(uint32 *)&pos3.y;
			lp[i].value[2] = *(uint32 *)&pos3.z;
            LOG_INFO(U"{}"_fmt(model.ob.size));
            
            meshArrow.lPos =
			meshGrid.lPos =
			meshXY.lPos =
			meshXZ.lPos =
			meshYZ.lPos = pos3;

        }
    }

    else if ( grabState.area == M_ARROW )    //中ドラッグ処理:3Dカーソル回転
    {
		pointBegin = Cursor::PosF();
        const Ray mouseRay = cam.screenToRay(pointBegin);
        if (const auto depth = mouseRay.intersects(meshArrow.ob))
        {
			Vec3 ipos = mouseRay.point_at(*depth);
			meshArrow.camera.setView(meshArrow.lPos, ipos);

			nowQ = meshArrow.camera.lookAt( meshArrow.lPos, ipos ) ;

			meshArrow.qRot = meshArrow.qRot * (oldQ*nowQ.inverse());
			meshXY.qRot =
			meshYZ.qRot =
			meshXZ.qRot =
			meshGrid.qRot =
			meshArrow.qRot;

			oldQ = nowQ ;

			meshSP.lPos = ipos;
        }
    }

	//軸選択中
    else if (grabState.area == M_XY || grabState.area == M_XZ || grabState.area == M_YZ )
    {
        const Ray mouseRay = cam.screenToRay(Cursor::PosF());
        Optional<float> depth;
		Float3 pos3{0,0,0};

		if (grabState.area == M_XY)//赤
		{
			depth = mouseRay.intersects( meshXY.ob.scaled(10,10,1) );
            pos3 = mouseRay.point_at(*depth);
			LOG_INFO(U"<XY> OB=org:{} new:{}"_fmt(grabState.prevPos, pos3 ));
		}

		else if (grabState.area == M_XZ)//緑
		{
			depth = mouseRay.intersects( meshXZ.ob.scaled(10,1,10) );
            pos3 = mouseRay.point_at(*depth);
			LOG_INFO(U"[XZ] OB=org:{} size:{}"_fmt(grabState.prevPos, pos3));
		}
		else if (grabState.area == M_YZ)//青
		{
			depth = mouseRay.intersects( meshYZ.ob.scaled(1,10,10) );
            pos3 = mouseRay.point_at(*depth);
			LOG_INFO(U"[YZ] OB=org:{} size:{}"_fmt(grabState.prevPos, pos3));
		}

		if ( depth )
        {
			Float3 delta3 = pos3 - grabState.prevPos;
			grabState.prevPos = pos3;

			meshArrow.lPos =
			meshXY.lPos =
			meshXZ.lPos =
			meshYZ.lPos += delta3;

			meshArrow.ob.moveBy( delta3 );
			meshXY.ob.moveBy( delta3 );
			meshXZ.ob.moveBy( delta3 );
			meshYZ.ob.moveBy( delta3 );


///			LOG_INFO(U"meshXY.lPos:{}"_fmt(meshXY.lPos));
		}
    }

    else if (grabState.area == M_RELEASE )    //左ドラッグ開始処理（解放状態であること）
    {

        auto &lp = tr.linePoints[lineno];
        for (int32 i = 0; i < lp.size(); i++)	//各ライン制御点を検索
        {
            Float2 pos = cam.worldToScreenPoint(toFloat3(lp[i].value));
            Rect mark = { (int32)pos.x,(int32)pos.y,is,is };

            if (mark.leftClicked())		//マウス乗ってる位置マーカーを選択してGUI制御対象に登録
            {
                Gui[M_POSLINE] = mark;
                grabState.trackNo = 0;
                grabState.lineNo = lineno;
                grabState.markerNo = i;
                grabState.area = M_POSLINE;
                onClickState = grabState;
                break;
            }

            if (mark.mouseOver() && mark.rightClicked())//右コンテキストメニュー処理
            {
/*                Array<String> menus = { U"イージング種別:",U"カーブ指定:" };
                Array<uint32*> datas = { (uint32*)(uint16*)&lp[i].easeType };
                guiPopup = Popup{ menus, datas,
                Float2(mark.x + 50,mark.y - 40),12,160,16,0,20,
                Float2(1,1),Float2(8,1.5),Float2(0,0),Float2(-50,40) };
*/            }

            // 3Dカーソルで左クリック軸選択
            if (MouseL.down() && grabState.area == M_RELEASE )
            {
                const Ray mouseRay = cam.screenToRay(Cursor::PosF());

				if (const auto depth = mouseRay.intersects(meshXY.ob))
                {
                    grabState.prevPos = mouseRay.point_at(*depth);
                    grabState.area = M_XY;
                }
                else if (const auto depth = mouseRay.intersects(meshXZ.ob))
                {
                    grabState.prevPos = mouseRay.point_at(*depth);
                    grabState.area = M_XZ;
                }
                else if (const auto depth = mouseRay.intersects(meshYZ.ob))
                {
                    grabState.prevPos = mouseRay.point_at(*depth);
                    grabState.area = M_YZ;
                }
				meshGrid.lPos = grabState.prevPos;
            }

            // 3Dカーソルで中クリックPAN回転
            if ( MouseM.down() )
            {
				pointBegin = Cursor::PosF();
                const Ray mouseRay = cam.screenToRay(pointBegin);
                if (const auto depth = mouseRay.intersects(meshArrow.ob))
                {
					Vec3 ipos = mouseRay.point_at(*depth);

					oldQ = meshArrow.camera.lookAt( meshArrow.lPos, ipos ) ;	//Mボタン押下時のqRotを保存

					meshSP.lPos = ipos;

					grabState.area = M_ARROW;
                }
            }
        }

    }
}


void Update(Timeline& timeline)
{
    cameraR.setEyePosition(eyePosR.xyz()).setFocusPosition(focusPosR);
//    updateShortcut(cameraR);
    eyePosR = Float4 {cameraR.getEyePosition(), eyePosR.w };
    focusPosR = cameraR.getFocusPosition();


    meshGnd.ob.size.y = 0.001;//地面なので薄く
    updatePosline(cameraGUI, meshGnd, timeline, 0, PRB_ACT);
    updatePosline(cameraGUI, meshGnd, timeline, 0, PRB_CAM);
    updatePosline(cameraGUI, meshGnd, timeline, 0, PRB_FOCUS);
}

//タイムラインコントロールのラインリンク処理
void linkMarker( Timeline& timeline, uint32 trackno, uint32 lineno, RectF &rectgui )
{
    Timeline::Track& tr = timeline.m_Tracks[ trackno ];
//	const Array<Timeline::LinePoint> &lp = tr.linePoints[ lineno ];

    if (tr.lineLinks[lineno] != -1) //リンクあり
    {
        //プローブはタイムラインのメンバじゃなくトラックのメンバでは
        Array<int32>& prevtpos = tr.lineLinkProbes[lineno];
        Array<int32> tpos = timeline.GetValueI32(timeline.m_Probes[ tr.lineLinks[lineno] ]); //新値
        if (prevtpos != tpos )                 //以前の値と比較で差異
        {
            Float3 delta = toFloat3(prevtpos) - toFloat3(tpos);//移動量
            prevtpos = tpos;
            
            Float3 pos3 = toFloat3( timeline.GetValueI32( timeline.m_Probes[lineno] )) - delta;
            timeline.PutValue(timeline.m_Probes[lineno], pos3);

            Float2 pos2 = cameraGUI.worldToScreenPoint( pos3 );
            rectgui.x = pos2.x;
            rectgui.y = pos2.y;
        }
    }
}

void updatedrawMarker( Timeline& timeline )
{
    //マーカー座標再計算
    Float2 apos = cameraGUI.worldToScreenPoint( actorPos );
    Gui[M_ACT].x = apos.x ;
    Gui[M_ACT].y = apos.y;

    Float2 fpos = cameraGUI.worldToScreenPoint( focusPosR );
    Gui[M_FOCUS].x = fpos.x ;
    Gui[M_FOCUS].y = fpos.y;

    Float2 epos = cameraGUI.worldToScreenPoint( eyePosR.xyz() );
    Gui[M_EYE].x = epos.x;
    Gui[M_EYE].y = epos.y;
	OrientedBox ob;

    //移動するマーカーはＺ座標をOBB衝突判定する２DはGui[]、３Dはactor等のパラメータ変数
    actorPos  = guiDrag(cameraGUI,meshGnd.ob, U"\xF1382", Gui[M_ACT],  grabState, M_ACT,ColorF(0,0,1),ColorF(1),    actorPos );
    eyePosR   = guiDrag(cameraGUI,meshGnd.ob, U"\xF0100", Gui[M_EYE],  grabState, M_EYE, ColorF(1),   Palette::Red, eyePosR );
    focusPosR = guiDrag(cameraGUI,meshGnd.ob, U"\xF0B69", Gui[M_FOCUS],grabState, M_FOCUS, ColorF(1), Palette::Red, focusPosR );

    //マーカーリンク処理
    linkMarker( timeline, 0, 0, Gui[M_ACT]);
    linkMarker( timeline, 0, 1, Gui[M_EYE]);
    linkMarker( timeline, 0, 2, Gui[M_FOCUS]);

    if (MouseL.up() || MouseM.up()) grabState.area = M_RELEASE;
}

// モーフターゲット実装
// 0:基本       8:驚眉      16:閉歯
// 1:閉目(下)   9:頑眉      17:開歯
// 2:開口       10:驚口     18:あ口
// 3:怒目       11:悔口     19:い口
// 4:怒眉       12:閉歯     20:う口
// 5:閉口(右上) 13:閉目(中) 21:え口
// 6:困眉       14:悲眉     22:お口
// 7:点目       15:閉目(上) 23:

// 瞬きは、13:閉目(中)
// 喋りは、16～20
// 12:閉歯は他と開口と合成して使う
// 眉と目は左右独立が必要
// 髪の毛もモーフィングにしようか。。

void Main()
{

    // ウィンドウ初期化
    Window::Resize(rectWindow.w, rectWindow.h);
    Window::SetStyle(WindowStyle::Sizable);				// ウィンドウを手動リサイズ可能にする
    Scene::SetBackground(ColorF(0.8, 0.9, 1.0));		// ウィンドウサイズに合わせて拡縮

#ifdef USE_WEBCAMERA
	//WEBカメラ初期化
	Webcam webcam;
	Image image;
    DynamicTexture webcamTex;

	for (const auto& wbc : System::EnumerateWebcams())
		LOG_INFO(U"webCam:{} Name:{}"_fmt(wbc.cameraIndex, wbc.name) );

	if (!webcam.open(0)) return;
	webcam.start();
#endif

	cameraR = PixieCamera(rectWindow, 45_deg, eyePosR.xyz(), focusPosR);
	cameraGUI = PixieCamera(rectWindow, 45_deg, eyePosGUI.xyz(), focusPosGUI);

	cameraVRM = PixieCamera(rectVRM, 45_deg, eyePosVRM.xyz(), focusPosVRM);
    MSRenderTexture renderTextureVRM = { (unsigned)rectVRM.w, (unsigned)rectVRM.h,
		                                  TextureFormat::R8G8B8A8_Unorm_SRGB, HasDepth::Yes  }; // RT初期化

	cameraCUI = PixieCamera(rectCUI, 45_deg, eyePosCUI.xyz(), focusPosCUI);
    MSRenderTexture renderTextureCUI = { (unsigned)rectCUI.w, (unsigned)rectCUI.h,
		                                  TextureFormat::R8G8B8A8_Unorm_SRGB, HasDepth::Yes  }; // RT初期化

    InitGltf();                                                                             // glTF初期化
    InitGUI();

    Timeline guiTimeline ;
    InitTimeline( guiTimeline, Rect{10,700,1800,265} ); // タイムラインGUI初期化

    // アセット登録
    FontAsset::Register(U"fontT",30, U"TOD4UI-Regular.otf");
//    const Font &fontT = FontAsset(U"fontT");
//    const Font &iconS = FontAsset(U"iconS");
//    const Font &fontS = FontAsset(U"fontS");
//    const Font &fontM = FontAsset(U"fontM");
//    const Font &iconM = FontAsset(U"iconM");

	//透過ブレンド
	BlendState blendstate = BlendState::Default3D;
	blendstate.srcAlpha = Blend::SrcAlpha;
	blendstate.dstAlpha = Blend::DestAlpha;
	blendstate.opAlpha = BlendOp::Max;

	//メインループ
	while (System::Update())	
	{
		//通常描画にカメラ指定
		Graphics3D::SetCameraTransform(cameraGUI.getViewProj(), cameraGUI.getEyePosition());
		Graphics3D::SetGlobalAmbientColor(ColorF{0.9});
		Update(guiTimeline);
		updateMouse( meshGnd, focusPosGUI, eyePosGUI );

		Draw( cameraGUI, guiTimeline );							//通常の描画

//#define ENABLE_VTUBER
#ifdef ENABLE_VTUBER
		//VRM描画にカメラ指定
		Graphics3D::SetCameraTransform(cameraVRM.getViewProj(), cameraVRM.getEyePosition());
		Graphics3D::SetGlobalAmbientColor(ColorF{0.8});

		renderTextureVRM.clear( ColorF{ 1,1,1,0 });				// RTをクリア
		{
			const ScopedRenderStates3D sr{ SamplerState::RepeatAniso, RasterizerState::SolidCullFront };
			const ScopedRenderStates3D bs{ blendstate };
			const ScopedRenderTarget3D rt{ renderTextureVRM };  // 描画対象をRTに変更

#ifdef USE_WEBCAM
			//Webカメラ描画
			if (webcam.hasNewFrame())
			{
				webcam.getFrame(image);
				webcamTex.fill(image);
			}
			webcamTex.draw();
#endif
			updateMorph( meshVRM );

			meshVRM.lPos.y = -15;
			meshVRM.eRot.z = 0;
			meshVRM.drawVRM();				// VRM描画

			if ( rectVRM.mouseOver() )
			{
				if (MouseL.pressed()) //中ボタン
				{
					static Float3 rotN;
					static double posN,eyeRrot,eyeLrot,eyeRpos,eyeLpos;
					double xx = ToRadians(Cursor::DeltaF().x*10);
					double yy = ToRadians(Cursor::DeltaF().y*-10);

					double vy = -Cursor::DeltaF().y/1000 ;
					rotN += Float3{yy,xx,0};
					meshVRM.gltfSetJoint(boneNeck_01, { 0,0,0 }, rotN );

					eyeRrot = (eyeRrot-xx/5 < -12.74)? -12.74: eyeRrot-xx/5 > 4.18   ?  4.18 : eyeRrot-xx/5;
					eyeLrot = (eyeLrot-xx/5 < -2.62) ? -2.62 : eyeLrot-xx/5 > 11.87  ? 11.87 : eyeLrot-xx/5;
					eyeRpos = (eyeRpos-vy/20 > 0.011) ? 0.011 : eyeRpos-vy/20 < -0.006 ? -0.006: eyeRpos-vy/20;
					eyeLpos = (eyeLpos-vy/20 > 0.011) ? 0.011 : eyeLpos-vy/20 < -0.006 ? -0.006: eyeLpos-vy/20;
					meshVRM.gltfSetJoint(boneEye_L_01, {0,0,eyeLpos}, { 0,0,eyeLrot });
					meshVRM.gltfSetJoint(boneEye_R_01, {0,0,eyeRpos}, { 0,0,eyeRrot });

				}
			}

			Graphics3D::Flush();								// RTに描画
			//renderTextureVRM.resolve();
			renderTextureVRM.draw(rectVRM.pos);					// RTを描画
		}
#endif

#define ENABLE_BILLBOARD
#ifdef ENABLE_BILLBOARD
		{
			Graphics3D::SetCameraTransform(cameraCUI.getViewProj(), cameraCUI.getEyePosition());
			Graphics3D::SetGlobalAmbientColor(ColorF{ 2.5 });

			renderTextureCUI.clear(ColorF{ 1,1,1,0 });				// RTをクリア
			{
				const ScopedRenderStates3D sr{ SamplerState::RepeatAniso, RasterizerState::SolidCullFront };
				const ScopedRenderStates3D bs{ blendstate };
				const ScopedRenderTarget3D rt{ renderTextureCUI };	// 描画対象をRTに変更

				static float i = 0.0;
				meshCUI.drawString(U"円:ABCDE", 1000, 5, ColorF{ 1 }, 0, 5);									// ビルボードフレーム描画
//				i = i + 0.05;
//				if (i >= 26.0) i = 0.0;

				eyePosCUI = Float4{ cameraCUI.getEyePosition(), 0 };
				focusPosCUI = cameraCUI.getFocusPosition();

				Graphics3D::Flush();								// RTに描画
				renderTextureCUI.resolve();
				renderTextureCUI.draw(rectCUI.pos);					// RTを描画
			}
		}
#endif

		guiTimeline.main();										// タイムラインGUI処理
		updatedrawMarker( guiTimeline );
	}
}

