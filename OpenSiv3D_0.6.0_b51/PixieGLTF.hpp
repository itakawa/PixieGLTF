# pragma once

# include <omp.h>
# include <thread>

# include <Siv3D.hpp> // OpenSiv3D v0.6
# include "PixieCamera.hpp"
# include "OBB.hpp"

#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE_WRITE
#include "App/3rd/tiny_gltf.h"

//# include <DirectXMath.h>
using namespace DirectX;
#define CPUSKINNING
#define BASIS 0			//BASIS
      
using Word4 = Vector4D<uint16>;

//通常描画の色の指定はテクスチャが有るならテクスチャ色、無いならマテリアル色にしている
//ただアプリから、任意色指定もしたい場合もあり区別したい。
//引数省略でColorF usrColor=Color(-1)としておいて、通常描画、それ以外の指定でアプリ指定色描画
//ユーザー指定色はベタだと、プリミティブが全部同じ色になってしまうので、相対値にしよう
//テクスチャ色にオフセット掛けるのは、なにがしかユニフォーム変数が必要
//実際の意味はusrColor.a==-1が相対色、-2が絶対色などのモードが決められる
constexpr auto USRCOLOR_NOTUSE = 0;
constexpr auto USRCOLOR_OFFSET = -1;
constexpr auto USRCOLOR = -2;
constexpr auto NOTUSE = (-1);
 
struct Channel
{
    uint8 typeDelta=0;
	uint8 numMorph=0;
    int16 idxNode=0;
    int32 idxSampler=0;
    Array<std::pair<int32, Array<float> >> deltaKeyframes;
	Channel() { numMorph = 0; typeDelta = 0; idxSampler = -1; idxNode = -1; }
};

struct Sampler
{
    Array<std::pair<int32, float>> interpolateKeyframes;
    float  minTime=0.0;
    float  maxTime=0.0;
    Sampler() { minTime = 0.0; maxTime = 0.0; }
};

struct Frame
{
    Array<MeshData>  MeshDatas; // メッシュデータ(定義)
    Array<DynamicMesh>	Meshes;	// メッシュ(実行時)※Mesh(モーフ無)/DynamicMesh(モーフ有)の選択が必要
    Array<uint8>     useTex;    // テクスチャ/頂点色識別子
    Array<Mat4x4>    morphMatBuffers;// モーフターゲット対象の姿勢制御行列
    Float3           obbSize{1,1,1};		//フレームのサイズ：再生時にフレームのサイズをローカル変換してOBBに反映
    Float3           obbCenter{0,0,0};		//原点オフセット：原点へのオフセット量
};

struct NodeParam	//ノード制御パラメータリスト旧extensions
{
	Mat4x4 matLocal;		// ローカル座標変換行列。原点に対するノードの姿勢に関する
	Mat4x4 matWorld;		// ワールド座標変換行列。最終頂点に適用
	Float3 posePos{0,0,0};	// ローカル座標変換(移動量)この３つはglTFの基本姿勢から取得して
	Float4 poseRot{0,0,0,0};	// ローカル座標変換(回転量)アニメーションの相対変化量を適用して
	Float3 poseSca{1,1,1};	// ローカル座標変換(拡縮量)matLocalを算出
	Mat4x4 matModify;		// VRMモードのローカル座標変換行列。ローカル座標変換行列に適用する。
	Float3 modPos{0,0,0};	// VRMモードのローカル座標変換(移動量)この３つは外部から姿勢操作
	Float4 modRot{0,0,0,0};	// VRMモードのローカル座標変換(回転量)する。
	Float3 modSca{0,0,0};	// VRMモードのローカル座標変換(拡縮量)
	bool update=false;		// VRMモードの更新要求matModifyを更新する。
};

struct PrecAnime      //gltfmodel.animationsに対応
{
    Array<ColorF>         meshColors; // 頂点色データ
    Array<Texture>        meshTexs;   // テクスチャデータ
                                      // ※頂点色とテクスチャは全フレームで共通
    Array<Frame>          Frames;     // フレームデータ。[フレーム]

	Array<Array<Sampler>>   Samplers;  // 補間形式(step,linear,spline)、前後フレーム、データ形式
    Array<Array<Channel>>   Channels;  // デルタ(移動,回転,拡縮,ウェイト)
};

struct MorphMesh
{
    Array<int32>			Targets;       // モデルのmeshノード順のモーフィング数(モーフ無しは0)
    Array<Array<Vertex3D>>	ShapeBuffers;  // モーフ形状バッファ		[モーフ番号][メッシュの頂点番号]
    Array<Array<Vertex3D>>	BasisBuffers;  // モーフ基本形状のコピー	[プリミティブ番号][メッシュの頂点番号]
	//※モーフ可能なメッシュはノード1つ固定の制約(プリミティブは複数)

    uint32                   TexCoordCount=0;	// モーフ有メッシュのUV座標数(aniModel用)
    Array<Float2>            TexCoord;			// モーフ有メッシュのUV座標(aniModel用)
};

//モーフ情報
struct MorphTargetInfo
{
    float Speed=1.0 ;			// モーフ速度
    float CntSpeed=0.0 ;		// モーフ速度制御カウント(>1.0で更新)
    int32 NowTarget=0 ;			// モーフターゲット(現在)  無効-1、通常0～
    int32 DstTarget=0 ;			// モーフターゲット(遷移先)通常-1、設定あれば0～
    int32 IndexTrans=-1 ;		// 遷移カウント、通常-1、遷移中0～
	Array<float> WeightTrans;	// 遷移ウェイトテーブル、-1か100％で終端
};

struct AnimeModel
{
    Array<PrecAnime>        precAnimes;
    MorphMesh               morphMesh;
};

struct NoAModel
{
    Array<String>           meshName;   // 名称
    Array<ColorF>           meshColors; // 頂点色データ
    Array<Texture>          meshTexs;   // テクスチャデータ
    Array<MeshData>			MeshDatas;	// メッシュデータ(定義)
    Array<DynamicMesh>		Meshes;		// メッシュ(実行時)※Mesh(モーフ無)/DynamicMesh(モーフ有)の選択が必要
    Array<int32>            useTex;     // テクスチャ頂点色識別子

    Array<Mat4x4>           morphMatBuffers;// モーフ対象の姿勢制御行列
    MorphMesh               morphMesh;
};

struct VRMModel
{
    Array<String>           meshName;   // 名称
    Array<ColorF>           meshColors; // 頂点色データ
    Array<Texture>          meshTexs;   // テクスチャデータ
    Array<MeshData>			MeshDatas;	// メッシュデータ(定義)
    Array<DynamicMesh>		Meshes;		// メッシュ(実行時)※Mesh(モーフ無)/DynamicMesh(モーフ有)の選択が必要
    Array<int32>            useTex;     // テクスチャ頂点色識別子

    Array<Array<Mat4x4>>    Joints;
    Array<Mat4x4>           morphMatBuffers;// モーフ対象の姿勢制御行列
    MorphMesh               morphMesh;
};


enum MODELTYPE { MODELNOA, MODELANI, MODELVRM };
constexpr uint32 NUMMIX = 3;				// TODO 3個制限は不要→全部に

class PixieGLTF
{
private:
	NoAModel        noaModel;
	AnimeModel      aniModel;
	VRMModel        vrmModel;
	bool			register1st = false;

	Array<NodeParam>		nodeParams;//NOA/VRM用。ANIは自動変数（とするとアニメは外部ボーン操作ができない）
public:
    tinygltf::Model gltfModel;
	Array<MorphTargetInfo>	morphTargetInfo ;

	bool					obbDebug = false;
	Float3                  obbSize{1,1,1};  // モデルのサイズ：モデルのサイズをローカル変換してOBBに反映
    Float3                  obbCenter{0,0,0};// 原点へのオフセット

    MODELTYPE modelType;
    String textFile = U"";

    Float3 lPos{0,0,0};         //ローカル変換1
    Float3 lSca{1,1,1};

	Float3 eRot{0,0,0};			//オイラー回転※最終的な角度は、四元数(オイラー回転)ｘ四元数回転
	Quaternion qRot{0,0,0,1};	//四元数回転	※オイラー回転使いたく無い時はRPY=000、四元数回転使いたくないときはXYZW=0001

    Float3 rPos{0,0,0};         //ローカル変換2(相対)

    int32 currentFrame=0;
	OrientedBox ob;
	OBB obb;
    Mat4x4 matVP;

    PixieCamera camera;  //カメラ

    int32 animeID = 0;
    int32 morphID = 0;

    ~PixieGLTF() = default;
    PixieGLTF() = default;
    PixieGLTF( String filename,                //glTFファイル名 
               Float3 localpos = Float3{0,0,0},//座標
               Float3 scale  = Float3{1,1,1},  //拡縮率
               Float3 rotate = Float3{0,0,0},  //回転(オイラー角)
               Float3 relpos = Float3{0,0,0},  //座標からの相対座標
               int32 frame=0,                  //フレーム番号
               int32 anime=0,                  //アニメ番号
               int32 morph=0)                  //モーフ番号
    {
        textFile = filename;
        lPos = localpos;
        rPos = relpos;
        lSca = scale;
        eRot = rotate;
        currentFrame = frame;
        animeID = anime;
        morphID = morph;
    }

    void initModel( MODELTYPE modeltype,bool debug=false, uint32 cycleframe = 60, int32 animeid=0)
    {
        //指定されたglTFファイルを取得
        std::string err, warn;
		bool result;
		tinygltf::TinyGLTF loader;

		result = loader.LoadBinaryFromFile(&gltfModel, &err, &warn, textFile.narrow());
		if (!result) result = loader.LoadASCIIFromFile(&gltfModel, &err, &warn, textFile.narrow());

        //モデルタイプ選択によりglTF→メッシュ生成
		if (result && modeltype == MODELNOA)
		{
			gltfSetupNOA();
			obbDebug = debug;
		}

		else if (result && modeltype == MODELANI)
		{
			gltfSetupANI(MODELANI, aniModel, gltfModel, cycleframe, animeid);
			obbDebug = debug;
		}

		else if (result && modeltype == MODELVRM)
		{
			gltfSetupVRM();
			obbDebug = debug;
		}

		//初期状態のカメラ、視点、注視点はY軸方向に+10の位置
        camera.setView(lPos, lPos + Float3(+0, +10, +0));
    }


    void setStartFrame( uint32 anime_no, int32 offsetframe )
    {
        PrecAnime &pa = aniModel.precAnimes[anime_no>>1];
        currentFrame = offsetframe % pa.Frames.size() ;
    }

    tinygltf::Buffer* getBuffer(tinygltf::Model& gltfmodel, tinygltf::Primitive& pr, int* offset, int* stride, int* componenttype)
    {
        if (pr.indices == -1) return nullptr;
        tinygltf::Accessor& ai = gltfmodel.accessors[pr.indices];
        tinygltf::BufferView& bi = gltfmodel.bufferViews[ai.bufferView];
        tinygltf::Buffer& buf = gltfmodel.buffers[bi.buffer];
        *offset = bi.byteOffset + ai.byteOffset;
        *stride = ai.ByteStride(bi);
        *componenttype = ai.componentType;
        return &buf;
    }

    tinygltf::Buffer* getBuffer(tinygltf::Model& gltfmodel, tinygltf::Primitive& pr, const std::string attr, int* offset, int* stride, int* componenttype)
    {
        if (pr.attributes.size() == 0) return nullptr;
        tinygltf::Accessor& ap = gltfmodel.accessors[pr.attributes[attr]];
        tinygltf::BufferView& bp = gltfmodel.bufferViews[ap.bufferView];
        tinygltf::Buffer& buf = gltfmodel.buffers[bp.buffer];
        *offset = bp.byteOffset + ap.byteOffset;
        *stride = ap.ByteStride(bp);
        *componenttype = ap.componentType;
        return &buf;
    }

    static tinygltf::Buffer* getBuffer(tinygltf::Model& model, tinygltf::Primitive& pr, const int32 &morphtarget, const char* attr, int* offset, int* stride, int* componenttype)
    {
        if (pr.targets.size() == 0) return nullptr;
        tinygltf::Accessor& ap = model.accessors[pr.targets[morphtarget].at(attr)];
        tinygltf::BufferView& bp = model.bufferViews[ap.bufferView];
        tinygltf::Buffer& buf = model.buffers[bp.buffer];
        *offset = bp.byteOffset + ap.byteOffset;
        *stride = ap.ByteStride(bp);
        *componenttype = ap.componentType;
        return &buf;
    }

	void gltfSetupPosture(tinygltf::Model& gltfmodel, int32 nodeidx, Array<NodeParam>& nodeParams )
    {
		NodeParam &np = nodeParams[nodeidx];

		auto& node = gltfmodel.nodes[nodeidx];
		auto& r = node.rotation;
        auto& t = node.translation;
        auto& s = node.scale;

        Quaternion rr = (node.rotation.size()) ?    Quaternion(r[0], r[1], r[2], r[3]):Quaternion(0,0,0,1);
        Float3     tt = (node.translation.size()) ? Float3(t[0], t[1], t[2]):Float3(0,0,0);
        Float3     ss = (node.scale.size()) ?       Float3(s[0], s[1], s[2]):Float3(1,1,1);
		Mat4x4 matlocal = Mat4x4::Identity().Scale(ss) *
			              Mat4x4(Quaternion(rr)) *
			              Mat4x4::Identity().Translate(tt);
		np.matLocal = matlocal;				// ローカル座標変換行列。原点に対するノードの姿勢を初期設定

		np.matWorld = Mat4x4::Identity();	// ワールド座標変換行列。最終頂点に適用
		np.posePos = Float3{0,0,0};			// ローカル座標変換(移動量)この３つはglTFの基本姿勢から取得して
		np.poseRot = Float4{0,0,0,1};		// ローカル座標変換(回転量)アニメーションの相対変化量を適用して
		np.poseSca = Float3{1,1,1};			// ローカル座標変換(拡縮量)matLocalを算出

		np.update = false;					// 初期化完了

		//子ノードを全て基本姿勢登録
		for (uint32 cc = 0; cc < node.children.size(); cc++)
            gltfSetupPosture(gltfmodel, node.children[cc], nodeParams);
	}

    void gltfCalcSkeleton(tinygltf::Model& gltfmodel, const Mat4x4& matparent,
		                  int32 nodeidx,Array<NodeParam>& nodeParams )
    {
		NodeParam &np = nodeParams[nodeidx];

		auto& node = gltfmodel.nodes[nodeidx];
		Mat4x4 matlocal = np.matLocal ;		  //glTFの基本姿勢定義

		Float4 rr = np.poseRot;			      //glTFアニメの姿勢変位
		Float3 tt = np.posePos;
		Float3 ss = np.poseSca;
		Mat4x4 matpose = Mat4x4::Identity().Scale(ss)*
			             Mat4x4(Quaternion(rr))*
			             Mat4x4::Identity().Translate(tt);

		if ( np.update )
			matlocal = matlocal * np.matModify ;//外部の姿勢操作

		if( matpose.isIdentity() ) matpose = matlocal;

		Mat4x4 mat = matpose * matlocal.inverse();
        Mat4x4 matworld = mat * matlocal * matparent;
		np.matWorld = matworld;

		for (int32 cc = 0; cc < node.children.size(); cc++)
            gltfCalcSkeleton(gltfmodel, matworld, node.children[cc], nodeParams );
	}

    void gltfSetupNOA()
    {
        modelType = MODELNOA;
		nodeParams.resize( gltfModel.nodes.size() );

		const MorphTargetInfo mti{1.0, 0.0, 0,0,-1,{ 0,-1 } };
		morphTargetInfo.resize( NUMMIX, mti );

		for (int32 nn = 0; nn < gltfModel.nodes.size(); nn++)
			gltfSetupPosture( gltfModel, nn, nodeParams );

		for (int32 nn = 0; nn < gltfModel.nodes.size(); nn++)
		{
			auto& mn = gltfModel.nodes[nn];
			if (mn.mesh >= 0)
				gltfCalcSkeleton( gltfModel, Mat4x4::Identity(), nn, nodeParams );

			for (int32 cc = 0; cc < mn.children.size(); cc++)
				gltfCalcSkeleton( gltfModel, Mat4x4::Identity(), mn.children[cc], nodeParams  );
		}

        Array<Array<Mat4x4>> Joints;
		if (gltfModel.skins.size() > 0)
		{
			Joints.resize(gltfModel.skins.size());
			for (int32 nn = 0; nn < gltfModel.nodes.size(); nn++)
			{
				auto& mn = gltfModel.nodes[nn];
				if (mn.skin >= 0)         //LightとCameraをスキップ
				{
					Joints[ mn.skin ].resize( gltfModel.skins[mn.skin].joints.size() );

					auto& msns = gltfModel.skins[mn.skin];
					auto& ibma = gltfModel.accessors[msns.inverseBindMatrices];
					auto& ibmbv = gltfModel.bufferViews[ibma.bufferView];
					auto  ibmd = gltfModel.buffers[ibmbv.buffer].data.data() + ibma.byteOffset + ibmbv.byteOffset;

					for (int32 ii = 0; ii < msns.joints.size(); ii++)
					{
						Mat4x4 ibm = *(Mat4x4*)&ibmd[ii * sizeof(Mat4x4)];
						Mat4x4& matworld = nodeParams[msns.joints[ii]].matWorld;
						Joints[mn.skin][ii] = ibm * matworld;
					}
				}
			}
		}

		for (uint32 nn = 0; nn < gltfModel.nodes.size(); nn++)
		{
			auto& node = gltfModel.nodes[nn];
			gltfSetupMorph( node, noaModel.morphMesh);
			gltfSetupNOA( node, nn, Joints );
		}

        //頂点の最大最小からサイズを計算してOBBを設定
        Float3 vmin = { FLT_MAX,FLT_MAX,FLT_MAX };
        Float3 vmax = { FLT_MIN,FLT_MIN,FLT_MIN };

        for (uint32 i = 0; i < noaModel.MeshDatas.size(); i++)
        {
            for (uint32 ii = 0; ii < noaModel.MeshDatas[i].vertices.size(); ii++)
            {
                Vertex3D& mv = noaModel.MeshDatas[i].vertices[ii];
				
                if (vmin.x > mv.pos.x) vmin.x = mv.pos.x;
                if (vmin.y > mv.pos.y) vmin.y = mv.pos.y;
                if (vmin.z > mv.pos.z) vmin.z = mv.pos.z;
                if (vmax.x < mv.pos.x) vmax.x = mv.pos.x;
                if (vmax.y < mv.pos.y) vmax.y = mv.pos.y;
                if (vmax.z < mv.pos.z) vmax.z = mv.pos.z;
            }
        }

		nodeParams.clear();
//		noaModel.MeshDatas.clear();	//モーフで使う

        //静的モデルのサイズ、中心オフセットを保存
        __m128 rot = XMQuaternionRotationRollPitchYaw(ToRadians(eRot.x), ToRadians(eRot.y), ToRadians(eRot.z));
        Mat4x4 mrot =  Mat4x4(Quaternion(rot)*qRot);
		Float3 t = lPos + rPos;
		Mat4x4 mat = Mat4x4::Identity().Scale(Float3{ -lSca.x,lSca.y,lSca.z }) * mrot * Mat4x4::Identity().Translate(t);        
/*
		obb.setMatrix(mat);
		obb.setPos( obbCenter = vmin + (vmax - vmin)/2 );
		obb.setSize( obbSize = (vmax - vmin) );
*/
		ob.setOrientation(Quaternion(rot) * qRot);
		ob.setPos(obbCenter = vmin + (vmax - vmin) / 2 );
		ob.setSize(obbSize = (vmax - vmin));
    }

    void gltfSetupNOA( tinygltf::Node& node, uint32 nodeidx, Array<Array<Mat4x4>> &Joints )
    {
		static uint32 morphidx = 0;         //モーフありメッシュの個数(フラットプリミティブリスト)
		static uint32 meshidx = 0;			//gltfはmeshes[].primitives[]に対して、Siv3Dはmesh[]管理のための要素番号
		if (node.mesh < 0) return;
		else if (node.mesh == 0)
		{
			morphidx = 0;
			meshidx = 0;
		}

		Array<Vertex3D> morphmv;
		int32 prsize = gltfModel.meshes[node.mesh].primitives.size();
        for (int32 pp = 0; pp < prsize; pp++)
        {
            //準備
            auto& pr = gltfModel.meshes[node.mesh].primitives[pp];
            auto& map = gltfModel.accessors[pr.attributes["POSITION"]];

            int32 opos = 0, otex = 0, onor = 0, ojoints = 0, oweights = 0, oidx = 0, stride = 0, texstride = 0;
            int32 type_p = 0,type_t = 0,type_n = 0,type_j = 0,type_w = 0,type_i = 0;
			int32 stride_p = 0, stride_t = 0, stride_n = 0, stride_j = 0, stride_w = 0, stride_i = 0;

            //Meshes->Primitive->Accessor(p,i)->BufferView(p,i)->Buffer(p,i)
			auto& bpos = *getBuffer(gltfModel, pr, "POSITION", &opos, &stride_p, &type_p);        //type_p=5126(float)
			auto& btex = *getBuffer(gltfModel, pr, "TEXCOORD_0", &otex, &stride_t, &type_t);	  //type_t=5126(float)
			auto& bnormal = *getBuffer(gltfModel, pr, "NORMAL", &onor, &stride_n, &type_n);       //type_n=5126(float)
			auto& bjoint = *getBuffer(gltfModel, pr, "JOINTS_0", &ojoints, &stride_j, &type_j);   //type_j=5121(uint8)/5123(uint16)
			auto& bweight = *getBuffer(gltfModel, pr, "WEIGHTS_0", &oweights, &stride_w, &type_w);//type_n=5126(float)
			auto& bidx = *getBuffer(gltfModel, pr, &oidx, &stride_i, &type_i);                    //type_j=5123(uint16)

			//プリミティブがモーフ有なら先にでっちあげとく
			if ( pr.targets.size() > 0 && noaModel.morphMesh.BasisBuffers.size() )  //モーフあり
			{
				morphmv = noaModel.morphMesh.BasisBuffers[morphidx];
				Array<Array<Vertex3D>>& buf = noaModel.morphMesh.ShapeBuffers;
				for (int32 ii = 0; ii < morphmv.size(); ii++)	//顔の全ての頂点
				{
					//この方式は、かならず1/0になるから0.3みたいな微妙な表情ができないな。
					for (int32 iii = 0; iii < NUMMIX; iii++)	//4chモーフ合成(目、口、表情を非同期で動かす)
					{
						int32& now = morphTargetInfo[iii].NowTarget;
						int32& dst = morphTargetInfo[iii].DstTarget;
						int32& idx = morphTargetInfo[iii].IndexTrans;
						Array<float>& wt = morphTargetInfo[iii].WeightTrans;

						if (now == -1) continue;   //NowTargetが-1の場合はモーフ無効
						if (idx == -1)             //IndexTransが-1の場合はWeightTrans[0]でマニュアルウェイト
						{
							morphmv[ii].pos += buf[morphidx * NUMMIX + iii][ii].pos * (1 - wt[0]) + buf[morphidx * NUMMIX + iii][ii].pos * wt[0];
							morphmv[ii].normal += buf[morphidx * NUMMIX + iii][ii].normal * (1 - wt[0]) + buf[morphidx * NUMMIX + iii][ii].normal * wt[0];
						}
						else
						{
							float weight = (wt[idx] < 0) ? 0 : wt[idx];
							morphmv[ii].pos += buf[morphidx * NUMMIX + now][ii].pos * (1 - weight) + buf[morphidx * NUMMIX + dst][ii].pos * weight;
							morphmv[ii].normal += buf[morphidx * NUMMIX + now][ii].normal * (1 - weight) + buf[morphidx * NUMMIX + dst][ii].normal * weight;
						}
					}
				}
				morphidx++;
			}

            //頂点座標を生成
            Array<Vertex3D> vertices;
            for (int32 vv = 0; vv < map.count; vv++)
            {
				Vertex3D mv;
				float* basispos = (float*)&bpos.data.at(vv * stride_p + opos);
				float* basistex = (float*)&btex.data.at(vv * stride_t + otex);
				float* basisnor = (float*)&bnormal.data.at(vv * stride_n + onor);

				mv.pos = Float3{ basispos[0], basispos[1], basispos[2] }; //モーフなし glTFから座標とってくる
				if ( pr.targets.size() > 0 && morphmv ) mv = morphmv[vv]; //モーフあり モーフの座標とってくる

				mv.tex = Float2(basistex[0], basistex[1]);
				mv.normal = Float3(basisnor[0], basisnor[1], basisnor[2]);

				//基本姿勢適用
				Mat4x4 matlocal = nodeParams[nodeidx].matLocal;
                SIMD_Float4 vec4pos = DirectX::XMVector4Transform(SIMD_Float4(mv.pos, 1.0f), matlocal);
                Mat4x4 matnor = matlocal.inverse().transposed();
                mv.normal = SIMD_Float4{ DirectX::XMVector4Transform(SIMD_Float4(mv.normal, 1.0f), matlocal) }.xyz();

                //スキニングあれば適用
				Mat4x4 matskin = Mat4x4::Identity();
				if( node.skin >= 0 )
                {
					uint8* jb = (uint8*)&bjoint.data.at(vv * stride_j + ojoints); //1頂点あたり4JOINT
					uint16* jw = (uint16*)&bjoint.data.at(vv * stride_j + ojoints);
					float* wf = (float*)&bweight.data.at(vv * stride_w + oweights);
                    Word4 j4 = (type_j == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) ? Word4(jw[0],jw[1],jw[2],jw[3]) :
						                                                            Word4(jb[0],jb[1],jb[2],jb[3]) ;
                    Float4 w4 = Float4(wf[0], wf[1], wf[2], wf[3]);

					//4ジョイント合成
                    matskin = w4.x * Joints[node.skin][j4.x] +
						   	  w4.y * Joints[node.skin][j4.y] +
							  w4.z * Joints[node.skin][j4.z] +
							  w4.w * Joints[node.skin][j4.w];

//モーフありならスキニング行列を保存
					if (pr.targets.size() > 0)
						noaModel.morphMatBuffers.emplace_back(matskin);

					vec4pos = DirectX::XMVector4Transform(SIMD_Float4(mv.pos, 1.0f), matskin);
					if (!(std::abs(matskin.determinant()) < 10e-10))
						matnor = matskin.inverse().transposed();
                }

				mv.pos = vec4pos.xyz() /vec4pos.getW();
				mv.normal = SIMD_Float4{ DirectX::XMVector4Transform(SIMD_Float4(mv.normal, 1.0f), matskin) }.xyz();
                vertices.emplace_back(mv);
            }

            //頂点インデクスを生成してメッシュ生成
            MeshData md;
            if (pr.indices > 0)
            {
                auto& mapi = gltfModel.accessors[pr.indices];
                Array<s3d::TriangleIndex32> indices;
                for (int32 i = 0; i < mapi.count; i+=3)
                {
					s3d::TriangleIndex32 idx;
					if (mapi.componentType == 5123)
					{
						uint16* ibuf = (uint16 *)&bidx.data.at(i * 2 + oidx); //16bit
						idx.i0 = ibuf[0]; idx.i1 = ibuf[1]; idx.i2 = ibuf[2];
					}
					else if (mapi.componentType == 5125)
					{
						uint32* ibuf = (uint32 *)&bidx.data.at(i * 4 + oidx); //32bit
						idx.i0 = ibuf[0]; idx.i1 = ibuf[1]; idx.i2 = ibuf[2];
					}
                    indices.emplace_back(idx);
                }

                md = MeshData(vertices, indices);
                vertices.clear();
                indices.clear();
            }

            //頂点色とテクスチャを登録
            int32 usetex = 0;// テクスチャ頂点色識別子 b0=頂点色 b1=テクスチャ
			Texture tex ;
            ColorF col = ColorF(1);

            if (pr.material >= 0)
            {
                auto& nt = gltfModel.materials[pr.material].additionalValues["normalTexture"];  //法線マップ
                int32 idx = -1;
                auto& mmv = gltfModel.materials[pr.material].values;
                auto& bcf = mmv["baseColorFactor"];												//色

				if (mmv.count("baseColorTexture"))
                    idx = gltfModel.textures[(int32)mmv["baseColorTexture"].json_double_value["index"]].source;

                //頂点色を登録
                if (bcf.number_array.size()) col = ColorF(  bcf.number_array[0], 
                                                            bcf.number_array[1], 
                                                            bcf.number_array[2], 
                                                            bcf.number_array[3]);
                else                         col = ColorF(1);

                //テクスチャを登録
                if (idx >= 0 && gltfModel.images.size())
                {
                    tex = Texture();
                    if (gltfModel.images[idx].bufferView >= 0)	// .glbテクスチャ
                    {
                        auto& bgfx = gltfModel.bufferViews[gltfModel.images[idx].bufferView];
                        auto bimg = &gltfModel.buffers[bgfx.buffer].data.at(bgfx.byteOffset);

						tex = Texture( MemoryReader { bimg,bgfx.byteLength}, TextureDesc::MippedSRGB);
						usetex = 1;
					}

					else
                    {
/* //TODO こっちのTEXTUREマップ落ちる。。.gltfテクスチャの場合
						auto& mii = gltfmodel.images[idx].image;
//                        ByteArray teximg((void*)&mii, mii.size());	//4.3
//                        tex = Texture(std::move(teximg), TextureDesc::Mipped);	//4.3
						tex = Texture( MemoryReader { (const void*)&mii, mii.size()}, TextureDesc::MippedSRGB);
						usetex = 1;
*/
                    }

                }
            }

			noaModel.meshTexs.emplace_back(tex);								// テクスチャ追加
			noaModel.meshColors.emplace_back(col);                              // 頂点色追加
            noaModel.meshName.emplace_back(Unicode::FromUTF8(gltfModel.meshes[node.mesh].name));  // ノード名追加
            noaModel.MeshDatas.emplace_back(md);                                // メッシュ追加(後でOBB設定用/デバッグ用)
			noaModel.Meshes.emplace_back( DynamicMesh{ md } );                  // メッシュ追加(静的)※動的は頂点座標のみ別バッファ→DynamicMesh生成で対応
            noaModel.useTex.emplace_back(usetex);                               // テクスチャ頂点色識別子追加
        }
    }

    void gltfSetupMorph( tinygltf::Node& node, MorphMesh& morph )
    {
		if (node.mesh < 0) return;		//メッシュノードのみ処理

		for (int32 pp = 0; pp < gltfModel.meshes[node.mesh].primitives.size(); pp++)
        {
            auto& pr = gltfModel.meshes[node.mesh].primitives[pp];
            morph.Targets.emplace_back((int32)pr.targets.size());	//モーフの個数...。こんなのいる？

            if (pr.targets.size() > 0)	//モーフあり
            {
                //Meshes->Primitive->Accessor(p,i)->BufferView(p,i)->Buffer(p,i)
                int32 offsetpos = 0, offsetnor = 0, stride = 0;
                int32 type_p = 0,type_n = 0;

                auto basispos = getBuffer(gltfModel, pr, "POSITION", &offsetpos, &stride, &type_p);
                auto basisnor = getBuffer(gltfModel, pr, "NORMAL", &offsetnor, &stride, &type_n);

                //元メッシュ(basis)の頂点座標バッファを生成
                Array<Vertex3D> basisvertices;
                auto& numvertex = gltfModel.accessors[pr.attributes["POSITION"]].count; //元メッシュの頂点/法線数
                for (int32 vv = 0; vv < numvertex; vv++)
                {
                    Vertex3D mv;
                    auto pos = (float*)&basispos->data.at(vv * 12 + offsetpos);
                    auto nor = (float*)&basisnor->data.at(vv * 12 + offsetnor);
                    mv.pos = Float3(pos[0], pos[1], pos[2]);
                    mv.normal = Float3(nor[0], nor[1], nor[2]);
                    basisvertices.emplace_back(mv);
                }

                morph.BasisBuffers.emplace_back(basisvertices);		//ここでツリー構造→フラットなプリミティブリストにするのはバグの元だろうか
                basisvertices.clear();

                //シェイプキー(複数)の頂点座標バッファを生成
                for (int32 tt = 0; tt < pr.targets.size(); tt++)
                {
                    //Meshes->Primitive->Target->POSITION->Accessor(p,i)->BufferView(p,i)->Buffer(p,i)
                    int32 offsetpos = 0, offsetnor = 0, stride = 0;
                    int32 type_p = 0,type_n = 0;

                    auto mtpos = getBuffer(gltfModel, pr, tt, "POSITION", &offsetpos, &stride, &type_p);
                    auto mtnor = getBuffer(gltfModel, pr, tt, "NORMAL", &offsetnor, &stride, &type_n);
                    Array<Vertex3D> shapevertices;

					//モーフターゲットメッシュの頂点座標バッファを生成
                    for (int32 vv = 0; vv < numvertex; vv++)
                    {
						Vertex3D mv;
                        auto pos = (float*)&mtpos->data.at(vv * 12 + offsetpos);
                        auto nor = (float*)&mtnor->data.at(vv * 12 + offsetnor);
                        mv.pos = Float3(pos[0], pos[1], pos[2]);
                        mv.normal = Float3(nor[0], nor[1], nor[2]);
                        shapevertices.emplace_back(mv);
                    }
                    morph.ShapeBuffers.emplace_back(shapevertices);
                    shapevertices.clear();
                }
            }
        }
    }


	void gltfSetupVRM()
	{
		modelType = MODELVRM;
		nodeParams.resize( gltfModel.nodes.size() );

		const MorphTargetInfo mti{1.0, 0.0, 0,0,-1,{ 0,-1 } };
		morphTargetInfo.resize( NUMMIX, mti );

		//ノードの基本姿勢を登録
		for (int32 nn = 0; nn < gltfModel.nodes.size(); nn++)
			gltfSetupPosture( gltfModel, nn, nodeParams );

		//ここで一回目の描画直前まで実行
		for (int32 nn = 0; nn < gltfModel.nodes.size(); nn++)
		{
			auto& mn = gltfModel.nodes[nn];

			Mat4x4 imat = Mat4x4::Identity();
			if (mn.mesh >= 0)
				gltfCalcSkeleton(gltfModel, imat, nn, nodeParams );

			for (int32 cc = 0; cc < mn.children.size(); cc++)
				gltfCalcSkeleton(gltfModel, imat, mn.children[cc], nodeParams  );
		}

		//CPUスキニング
		if (gltfModel.skins.size() > 0)
		{
			vrmModel.Joints.resize(gltfModel.skins.size());

			for (int32 nn = 0; nn < gltfModel.nodes.size(); nn++)
			{
				auto& node = gltfModel.nodes[nn];

				if (node.skin < 0) continue;         //LightとCameraをスキップ

				const auto& msns = gltfModel.skins[node.skin];
				const auto& ibma = gltfModel.accessors[msns.inverseBindMatrices];
				const auto& ibmbv = gltfModel.bufferViews[ibma.bufferView];
				auto  ibmd = gltfModel.buffers[ibmbv.buffer].data.data() + ibma.byteOffset + ibmbv.byteOffset;

				for (int32 ii = 0; ii < msns.joints.size(); ii++)
				{
					Mat4x4 ibm = *(Mat4x4*)&ibmd[ii * sizeof(Mat4x4)];
					Mat4x4 &matworld = nodeParams[ msns.joints[ii] ].matWorld;
					Mat4x4 matjoint = ibm * matworld;

					if (vrmModel.Joints[node.skin].size() <= ii) vrmModel.Joints[node.skin].emplace_back(matjoint);
					else vrmModel.Joints[node.skin][ii] = matjoint;
				}
			}
		}

		//1次処理を進められるとこまで
		for (uint32 nn = 0; nn < gltfModel.nodes.size(); nn++)
		{
			auto& node = gltfModel.nodes[nn];
			gltfSetupMorph(node, vrmModel.morphMesh);
			gltfSetupVRM(node, nn);
		}

		register1st = true;	//初回登録済
	}

	// 前処理ここまで。前処理で生成されたmatlocalに対して姿勢補正することがVRM対応の要件。
	// 後処理は残りの演算を行い、描画する。
	// matmodify修正無い場合は、直でNoA描画を行い実行速度を改善する。

	void drawVRM(int32 istart = NOTUSE, int32 icount = NOTUSE)
	{
		for (int32 nn = 0; nn < gltfModel.nodes.size(); nn++)
        {
            auto& mn = gltfModel.nodes[ nn ];

			Mat4x4 imat = Mat4x4::Identity();
            if (mn.mesh >= 0)
                gltfCalcSkeleton(gltfModel, imat, nn, nodeParams );

            for (int32 cc = 0; cc < mn.children.size(); cc++)
                gltfCalcSkeleton(gltfModel, imat, mn.children[cc], nodeParams );
        }

        //CPUスキニング

//面倒な事せずに、普通にnode[]みていくだけでよいのかも
		for (int32 nn = 0; nn < gltfModel.nodes.size(); nn++)//97
        {
            auto& node = gltfModel.nodes[ nn ];

//univrmは、node[]children[]の参照ノードにmesh[]が含まれず、jointリスﾄがつくれない
//jointリスト作るときにchildren参照する必要ないのかも

            if (node.skin < 0) continue;         //LightとCameraをスキップ

            const auto& msns = gltfModel.skins[node.skin];
            const auto& ibma = gltfModel.accessors[msns.inverseBindMatrices];
            const auto& ibmbv = gltfModel.bufferViews[ibma.bufferView];
            auto  ibmd = gltfModel.buffers[ibmbv.buffer].data.data() + ibma.byteOffset + ibmbv.byteOffset;

			for (int32 ii = 0; ii < vrmModel.Joints[node.skin].size(); ii++)
            {
                Mat4x4 ibm = *(Mat4x4*)&ibmd[ii * sizeof(Mat4x4)];
				Mat4x4 &matworld = nodeParams[ msns.joints[ii] ].matWorld;
				Mat4x4 matjoint = ibm * matworld ;

//2度目以降はjointの追加はない
				vrmModel.Joints[node.skin][ii] = matjoint;
			}
        }

		for (uint32 nn = 0; nn < gltfModel.nodes.size(); nn++)
		{
            auto& mn = gltfModel.nodes[nn];
			if (mn.mesh >= 0)
            {
                gltfSetupVRM( mn, nn );
				gltfDrawVRM( istart, icount );
            }
        }
    }

	//前工程で計算されたワールド行列を頂点座標に適用して描画

	void gltfSetupVRM(tinygltf::Node& node, uint32 nodeidx )
	{
		static uint32 morphidx = 0;         //モーフありメッシュの個数(フラットプリミティブリスト)
		static uint32 meshidx = 0;			//gltfはmeshes[].primitives[]に対して、Siv3Dはmesh[]管理のための要素番号
		if (node.mesh < 0) return;
		else if (node.mesh == 0)
		{
			morphidx = 0;
			meshidx = 0;
		}

		Array<Vertex3D> morphmv;
		int32 prsize = gltfModel.meshes[node.mesh].primitives.size();
        for (int32 pp = 0; pp < prsize; pp++)
		{
			//準備
			auto& pr = gltfModel.meshes[node.mesh].primitives[pp];
			auto& map = gltfModel.accessors[pr.attributes["POSITION"]];

			int32 opos = 0, otex = 0, onormal = 0, ojoints = 0, oweights = 0, oidx = 0, stride = 0, texstride = 0;
			int32 type_p = 0, type_t = 0, type_n = 0, type_j = 0, type_w = 0, type_i = 0;

//TODO この辺毎度とってくるの無駄
			auto bpos = getBuffer(gltfModel, pr, "POSITION", &opos, &stride, &type_p);
			auto btex = getBuffer(gltfModel, pr, "TEXCOORD_0", &otex, &texstride, &type_t);
			auto bnormal = getBuffer(gltfModel, pr, "NORMAL", &onormal, &stride, &type_n);
			auto bjoint = getBuffer(gltfModel, pr, "JOINTS_0", &ojoints, &stride, &type_j);
			auto bweight = getBuffer(gltfModel, pr, "WEIGHTS_0", &oweights, &stride, &type_w);
			auto bidx = getBuffer(gltfModel, pr, &oidx, &stride, &type_i);

			//プリミティブがモーフ有なら先にでっちあげとく
			if ( pr.targets.size() > 0)  //モーフあり
			{
				morphmv = vrmModel.morphMesh.BasisBuffers[morphidx];
				Array<Array<Vertex3D>>& buf = vrmModel.morphMesh.ShapeBuffers;
				for (int32 ii = 0; ii < morphmv.size(); ii++)	//顔の全ての頂点
				{
					//TODO この方式は、かならず1/0になるから0.3みたいな微妙な表情ができない。
					for (int32 iii = 0; iii < NUMMIX; iii++)	//4chモーフ合成(目、口、表情を非同期で動かす)
					{
						int32& now = morphTargetInfo[iii].NowTarget;
						int32& dst = morphTargetInfo[iii].DstTarget;
						int32& idx = morphTargetInfo[iii].IndexTrans;
						Array<float>& wt = morphTargetInfo[iii].WeightTrans;

						if (now == -1) continue;   //NowTargetが-1の場合はモーフ無効
						if (idx == -1)             //IndexTransが-1の場合はWeightTrans[0]でマニュアルウェイト
						{
							morphmv[ii].pos += buf[morphidx * NUMMIX + iii][ii].pos * (1 - wt[0]) + buf[morphidx * NUMMIX + iii][ii].pos * wt[0];
							morphmv[ii].normal += buf[morphidx * NUMMIX + iii][ii].normal * (1 - wt[0]) + buf[morphidx * NUMMIX + iii][ii].normal * wt[0];
						}
						else
						{
							float weight = (wt[idx] < 0) ? 0 : wt[idx];
							morphmv[ii].pos += buf[morphidx * NUMMIX + now][ii].pos * (1 - weight) + buf[morphidx * NUMMIX + dst][ii].pos * weight;
							morphmv[ii].normal += buf[morphidx * NUMMIX + now][ii].normal * (1 - weight) + buf[morphidx * NUMMIX + dst][ii].normal * weight;
						}
					}
				}
                morphidx++;
			}

			//頂点座標を生成
			Array<Vertex3D> vertices;
			for (int32 vv = 0; vv < map.count; vv++)
			{
				float* vertex = nullptr, * texcoord = nullptr, * normal = nullptr;
				vertex = (float*)&bpos->data.at(vv * 12 + opos);
				texcoord = (float*)&btex->data.at(vv * 8 + otex);
				normal = (float*)&bnormal->data.at(vv * 12 + onormal);

				Vertex3D mv;
				if ( pr.targets.size() > 0) mv = morphmv[vv];									//モーフあり モーフの座標とってくる
				else						mv.pos = Float3{ vertex[0], vertex[1], vertex[2] }; //モーフなし glTFから座標とってくる

				mv.tex = Float2{ texcoord[0], texcoord[1] };			//毎度これいる？UV変わらんとおもう。
				mv.normal = Float3{ normal[0], normal[1], normal[2] };	

				//基本姿勢適用
				Mat4x4 matlocal = nodeParams[nodeidx].matLocal;
                SIMD_Float4 vec4pos = DirectX::XMVector4Transform(SIMD_Float4(mv.pos, 1.0f), matlocal);
                Mat4x4 matnor = matlocal.inverse().transposed();
                mv.normal = SIMD_Float4{ DirectX::XMVector4Transform(SIMD_Float4(mv.normal, 1.0f), matlocal) }.xyz();

//TODO この辺まで毎度同じため無駄

                //スキニングあれば適用
				Mat4x4 matskin = Mat4x4::Identity();
				if( node.skin >= 0 )
				{
					uint8* jb = (uint8*)&bjoint->data.at(vv * 4 + ojoints); //1頂点あたり4JOINT
					uint16* jw = (uint16*)&bjoint->data.at(vv * 8 + ojoints);
					Word4 j4 = (type_j == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) ? Word4(jw[0], jw[1], jw[2], jw[3]) :
						                                                            Word4(jb[0], jb[1], jb[2], jb[3]) ;
//WEIGHT取得
					float* wf = (float*)&bweight->data.at(vv * 16 + oweights);
					Float4 w4 = Float4(wf[0], wf[1], wf[2], wf[3]);

					// 4ジョイント合成
					matskin = w4.x * vrmModel.Joints[node.skin][j4.x] +
						      w4.y * vrmModel.Joints[node.skin][j4.y] +
						      w4.z * vrmModel.Joints[node.skin][j4.z] +
						      w4.w * vrmModel.Joints[node.skin][j4.w];

//モーフありならスキニング行列を保存
					if (pr.targets.size() > 0)
					{
						if (register1st == false) vrmModel.morphMatBuffers.emplace_back(matskin);
						else					  vrmModel.morphMatBuffers[pp] = matskin;
					}
//スキニング適用→mv
					vec4pos = DirectX::XMVector4Transform(SIMD_Float4(mv.pos, 1.0f), matskin);
					if (!(std::abs(matskin.determinant()) < 10e-10))
						matnor = matskin.inverse().transposed();
				}

				mv.pos = vec4pos.xyz() /vec4pos.getW();
				mv.normal = SIMD_Float4{ DirectX::XMVector4Transform(SIMD_Float4(mv.normal, 1.0f), matskin) }.xyz();
//頂点登録
				vertices.emplace_back(mv);
            }

			if (register1st == false)
			{
//頂点インデクスを生成してメッシュ生成(インデクスは変わらないので最初だけでいい)
				MeshData md;
//コンポーネントタイプに注意してglTFから頂点インデクス取得→indices
				auto& mapi = gltfModel.accessors[pr.indices];
				Array<s3d::TriangleIndex32> indices;
				for (int32 i = 0; i < mapi.count; i += 3)
				{
					s3d::TriangleIndex32 idx;
					if (mapi.componentType == 5123)
					{
						uint16* ibuf = (uint16*)&bidx->data.at(i * 2 + oidx); //16bit
						idx.i0 = ibuf[0]; idx.i1 = ibuf[1]; idx.i2 = ibuf[2];
					}
					else if (mapi.componentType == 5125)
					{
						uint32* ibuf = (uint32*)&bidx->data.at(i * 4 + oidx); //32bit
						idx.i0 = ibuf[0]; idx.i1 = ibuf[1]; idx.i2 = ibuf[2];
					}
					indices.emplace_back(idx);
				}
//頂点座標リストと頂点インデクスリストを合わせてメッシュ構築→md
				md = MeshData(vertices, indices);
				indices.clear();
				vertices.clear();
//メッシュ登録
				vrmModel.MeshDatas.emplace_back(md);				// メッシュ追加(後でOBB設定用/デバッグ用)※これ無駄。OBBデータこっち持ってくる
				vrmModel.Meshes.emplace_back( DynamicMesh{ md });	// メッシュ追加(静的)※動的は頂点座標のみ別バッファ→DynamicMesh生成で対応
			}

			//描画2回目以降。すげ替え必要あるのは動いた時だけ
			else
			{
				bool result = vrmModel.Meshes[meshidx++].fill(vertices);//頂点座標を再計算後にすげ替え(Array<Vertex3D>のみ更新)
				vertices.clear();
			}

			//頂点色とテクスチャを登録(テクスチャも変わらないので最初だけ)
			if ( register1st == false )
			{
				int32 usetex = 0;// テクスチャ頂点色識別子 b0=頂点色 b1=テクスチャ
				Texture tex;
				ColorF col = ColorF(1, 1, 1, 1);
				if (pr.material >= 0)
				{
					auto& nt = gltfModel.materials[pr.material].additionalValues["normalTexture"];  //法線マップ
					auto& mmv = gltfModel.materials[pr.material].values;
					auto& bcf = mmv["baseColorFactor"];                                         //色
					int32 idx = -1;
					if (mmv.count("baseColorTexture"))
						idx = gltfModel.textures[(int32)mmv["baseColorTexture"].json_double_value["index"]].source;

					//頂点色を登録(ベースカラー)
					if (bcf.number_array.size()) col = ColorF(bcf.number_array[0],
															  bcf.number_array[1],
															  bcf.number_array[2],
															  bcf.number_array[3]);

					//テクスチャを登録
					if (idx >= 0 && gltfModel.images.size())
					{
						tex = Texture();
						if (gltfModel.images[idx].bufferView >= 0)	// .glbテクスチャ
						{
							auto& bgfx = gltfModel.bufferViews[gltfModel.images[idx].bufferView];
							auto bimg = &gltfModel.buffers[bgfx.buffer].data.at(bgfx.byteOffset);
							tex = Texture(MemoryReader{ bimg,bgfx.byteLength }, TextureDesc::MippedSRGB);//リニアSRGB(ガンマは補正)
							usetex = 1;
						}

						else
						{
//TODO こっちのTEXTUREマップ落ちる。。.gltfテクスチャ
//						auto& mii = gltfmodel.images[idx].image;
//                        ByteArray teximg((void*)&mii, mii.size());	//4.3
//                        tex = Texture(std::move(teximg), TextureDesc::Mipped);	//4.3
//						tex = Texture( MemoryReader { (const void*)&mii, mii.size()}, TextureDesc::MippedSRGB);
//						usetex |= 1;
						}

					}
				}

				vrmModel.meshTexs.emplace_back(tex);	// テクスチャ追加
				vrmModel.meshColors.emplace_back(col);// 頂点色追加
				vrmModel.meshName.emplace_back(Unicode::FromUTF8(gltfModel.meshes[node.mesh].name));  // ノード名追加
				vrmModel.useTex.emplace_back(usetex);     // テクスチャ頂点色識別子追加
			}
		}
	}


	//ここからやっと描画


    void gltfDrawVRM( int32 istart = NOTUSE, int32 icount = NOTUSE )
	{
        uint32 tid = 0;
//ここでQuaternionにしたら意味なくない？
        __m128 rot = XMQuaternionRotationRollPitchYaw(ToRadians(eRot.x), ToRadians(eRot.y), ToRadians(eRot.z));
        Mat4x4 mrot =  Mat4x4(Quaternion(rot)*qRot);

        Float3 t = lPos + rPos;
//GL系メッシュをDX系で描画時はXミラーして逆カリング
//		Mat4x4 mat = Mat4x4::Identity().scaled(lSca) * mrot * Mat4x4::Identity().translated(t) ;
		Mat4x4 mat = Mat4x4::Identity().Scale(Float3{ -lSca.x,lSca.y,lSca.z }) * mrot * Mat4x4::Identity().Translate(t);

		//前計算後のメッシュにモデルのローカル変換行列適用して全部描画
		for (uint32 i = 0; i < vrmModel.Meshes.size(); i++)
        {
			if (istart == -1)						//部分描画なし
			{
				if (vrmModel.useTex[i])  vrmModel.Meshes[i].draw(mat, vrmModel.meshTexs[i], vrmModel.meshColors[i]);	//テクスチャ描画
				else					 vrmModel.Meshes[i].draw(mat, vrmModel.meshColors[i]);							//カラー描画
			}
			else									//部分描画あり
			{
				if (vrmModel.useTex[i])  vrmModel.Meshes[i].drawSubset(istart, icount, mat, vrmModel.meshTexs[tid++]);	//テクスチャ描画
				else					 vrmModel.Meshes[i].drawSubset(istart, icount, mat, vrmModel.meshColors[i]);	//カラー描画
			}
        }
    }


    PixieGLTF &setCamera(const PixieCamera& cam)
    {
        camera = cam;
        matVP = cam.getViewProj();
        return *this;
    }

	void drawMesh(ColorF usrColor=ColorF(NOTUSE), int32 istart = NOTUSE, int32 icount = NOTUSE)
    {
		Rect rectdraw = camera.getSceneRect();

        NoAModel &noa = noaModel;
        uint32 morphidx = 0;                     //モーフありメッシュの個数
        uint32 tid = 0;

        __m128 rot = XMQuaternionRotationRollPitchYaw(ToRadians(eRot.x), ToRadians(eRot.y), ToRadians(eRot.z));
        Mat4x4 mrot =  Mat4x4(Quaternion(rot)*qRot);

        Float3 t = lPos + rPos;

//GL系メッシュをDX系で描画時はXミラーして逆カリング
		//Mat4x4 mat = Mat4x4::Identity().scaled(lSca) * mrot * Mat4x4::Identity().translated(t) ;
		Mat4x4 mat = Mat4x4::Identity().Scale(Float3{ -lSca.x,lSca.y,lSca.z }) * mrot * Mat4x4::Identity().Translate(t);        

		for (uint32 i = 0; i < noa.Meshes.size(); i++)
        {

//glTFノードを精査していき、モーフターゲットがあるメッシュはモーフィング100％のVertexを別途保存

//モーフィングは4種
// 固定メッシュ(NOA)、アニメメッシュ(ANI)に対して、モーフィング対応メッシュ部分だけをすげ替えて描画する。
// モーフィングのパターンは瞬き(BLINKタイプ)と遷移(TRANSITIONタイプ)とがあり、状態遷移の管理を要する。
// 表情の場合最低限、4chの合成ができる必要ある。(EYE/MOUTH/TOOTH/FACIAL)※TOOTHとFACIALはTRANSITIONタイプ
			int32 morphs = 0;
			if ( noa.morphMesh.Targets.size() ) morphs = noa.morphMesh.Targets[i];
			if ( morphs > 0 )  //モーフあり
            {
                Array<Vertex3D> morphmv = noa.morphMesh.BasisBuffers[morphidx]; //元メッシュのコピーを作業用で確保
                Array<Array<Vertex3D>>& buf = noa.morphMesh.ShapeBuffers;

                for (int32 ii = 0; ii < morphmv.size(); ii++)	//頂点
                {
                    for (int32 iii = 0; iii <NUMMIX ; iii++)   //4chモーフ合成(目、口、歯、表情を非同期で動かす)
                    {
                        int32& now = morphTargetInfo[iii].NowTarget;
                        int32& dst = morphTargetInfo[iii].DstTarget;
                        int32& idx = morphTargetInfo[iii].IndexTrans;
                        Array<float>& wt = morphTargetInfo[iii].WeightTrans;

                        if (now == -1) continue;   //NowTargetが-1の場合はモーフ無効
                        if (idx == -1)             //IndexTransが-1の場合はWeightTrans[0]でマニュアルウェイト
                        {
                            Array<float> weight = morphTargetInfo[BASIS].WeightTrans;
                            morphmv[ii].pos += buf[morphidx * NUMMIX + iii][ii].pos * (1 - wt[0]) + buf[morphidx * NUMMIX + iii][ii].pos * wt[0];
                            morphmv[ii].normal += buf[morphidx * NUMMIX + iii][ii].normal * (1 - wt[0]) + buf[morphidx * NUMMIX + iii][ii].normal * wt[0];
                        }
                        else
                        {
                            float weight = (wt[idx] < 0) ? 0 : wt[idx];
                            morphmv[ii].pos += buf[morphidx * NUMMIX + now][ii].pos * (1 - weight) + buf[morphidx * NUMMIX + dst][ii].pos * weight;
                            morphmv[ii].normal += buf[morphidx * NUMMIX + now][ii].normal * (1 - weight) + buf[morphidx * NUMMIX + dst][ii].normal * weight;
                        }
                    }

					Mat4x4& matskin = noa.morphMatBuffers[i];
                    SIMD_Float4 vec4pos = DirectX::XMVector4Transform(SIMD_Float4(morphmv[ii].pos, 1.0f), matskin);
                    Mat4x4 matnor = matskin.inverse().transposed();

                    morphmv[ii].pos = vec4pos.xyz() /vec4pos.getW();
					morphmv[ii].normal = SIMD_Float4{ DirectX::XMVector4Transform(SIMD_Float4(morphmv[ii].normal, 1.0f), matnor) }.xyz();
                    morphmv[ii].tex = noa.MeshDatas[i].vertices[ii].tex;
                }

				noa.Meshes[i].fill(morphmv);				//頂点座標を再計算後にすげ替え
                morphidx++;
            }

			if (istart == NOTUSE)	//部分描画なし
			{
				if (noa.useTex[i])	//テクスチャ色
					noa.Meshes[i].draw(mat, noa.meshTexs[i], noa.meshColors[i]);

				else				//マテリアル色
				{
					if ( usrColor.a >= USRCOLOR_NOTUSE )		//ユーザー色指定なし
						noa.Meshes[i].draw(mat, noa.meshColors[i]);
					else if	( usrColor.a == USRCOLOR_OFFSET )	//ユーザー色相対指定
						noa.Meshes[i].draw(mat, noa.meshColors[i] + ColorF(usrColor.rgb(),1) );
					else if	( usrColor.a == USRCOLOR )			//ユーザー色絶対指定
						noa.Meshes[i].draw(mat, ColorF(usrColor.rgb(),1) );
				}
			}
			else				//部分描画あり
			{
				if (noa.useTex[i])
					noa.Meshes[i].drawSubset(istart, icount, mat, noa.meshTexs[tid++]);
				else				//マテリアル色
				{
					if ( usrColor.a >= USRCOLOR_NOTUSE )		//ユーザー色指定なし
						noa.Meshes[i].drawSubset(istart, icount, mat, noa.meshColors[i]);
					else if	( usrColor.a == USRCOLOR_OFFSET )	//ユーザー色相対指定
						noa.Meshes[i].drawSubset(istart, icount,mat, noa.meshColors[i] + ColorF(usrColor.rgb(),1) );
					else if	( usrColor.a == USRCOLOR )			//ユーザー色絶対指定
						noa.Meshes[i].drawSubset(istart, icount,mat, ColorF(usrColor.rgb(),1) );
				}
            }
		}

		if (obbDebug) ob.draw(ColorF(0,0,1, 0.1) );
    }

	void gltfSetupANI( const MODELTYPE& modeltype, AnimeModel& animodel,
		               tinygltf::Model& gltfmodel, uint32 cycleframe = 60, int32 animeid=0)
    {
        modelType = modeltype;

		animodel.precAnimes.resize(gltfmodel.animations.size());
        animodel.morphMesh.TexCoordCount = (unsigned)-1;

        if (animeid == -1)                      //全アニメデコード対象
        {
            for (int32 aid = 0; aid < gltfmodel.animations.size(); aid++)
				gltfOmpSetupANI( aid, cycleframe);
        }
        else
            gltfOmpSetupANI( animeid, cycleframe); //1アニメデコード対象
    }

	bool gltfOmpSetupANI(int32 animeid = 0, int32 cycleframe = 60)
	{
        if (gltfModel.animations.size() == 0) return false;

		auto& man = gltfModel.animations[ animeid ];
        auto& mas = man.samplers;
        auto& mac = man.channels;

        auto& macc = gltfModel.accessors;
        auto& mbv = gltfModel.bufferViews;
        auto& mb = gltfModel.buffers;

        auto begintime = macc[mas[0].input].minValues[0];               // 現在時刻
        auto endtime = macc[mas[0].input].maxValues[0];                 // ループアニメ周期（アニメーション時間の最大はSamplerInputの最終要素に記録される）
        auto frametime = (endtime - begintime) / cycleframe;            // 1フレーム時間(cycleframeは１つのアニメを何フレームで実施するかを指定)
        auto currenttime = begintime;

		//メモリ確保
		aniModel.precAnimes[ animeid ].Frames.resize(cycleframe);

		//各スレッドに割り当てるフレーム時刻リストを生成
		Array<double> frametimes;
		for (int32 cf = 0; cf < cycleframe; cf++)
			frametimes.emplace_back(currenttime += frametime);

		//最大スレッド数でメモリ確保
		omp_set_num_threads(32);
		int32 tmax = omp_get_max_threads();
		aniModel.precAnimes[ animeid ].Samplers.resize(tmax);
		aniModel.precAnimes[ animeid ].Channels.resize(tmax);

		size_t bufsize = 0;
		for (int32 th = 0; th < tmax; th++)
		{
			auto& as = aniModel.precAnimes[ animeid ].Samplers[th];
			auto& ac = aniModel.precAnimes[ animeid ].Channels[th];
			as.resize(mas.size()); bufsize += mas.size()*sizeof(as);
			ac.resize(mac.size()); bufsize += mac.size()*sizeof(ac);

			for (int32 ss = 0; ss < mas.size(); ss++)
			{
				auto& mbsi = gltfModel.accessors[mas[ss].input];  // 前フレーム情報
				as[ss].interpolateKeyframes.resize(mbsi.count); bufsize += mbsi.count*sizeof(as[ss].interpolateKeyframes);
			}

			for (int32 cc = 0; cc < ac.size(); cc++)
			{
				auto& maso = gltfModel.accessors[mas[cc].output];
				ac[cc].deltaKeyframes.resize(maso.count);bufsize += maso.count *sizeof(ac[cc].deltaKeyframes);

				//メッシュの場合は、モーフ数を記録
				auto& mid = gltfModel.nodes[mac[cc].target_node].mesh;
				ac[cc].numMorph = (mid != -1) ? gltfModel.meshes[ mid ].weights.size() : 0;

				for (int32 ff = 0; ff < maso.count; ff++)
				{
					if (ac[cc].numMorph)
					{
						ac[cc].deltaKeyframes[ff].second.resize(ac[cc].numMorph); bufsize += ac[cc].numMorph * sizeof(ac[cc].deltaKeyframes[ff].second);
					}
					else
					{
						ac[cc].deltaKeyframes[ff].second.resize(4);                bufsize += 4;
					}
				}
			}
			for (int32 ss = 0; ss < mas.size(); ss++)
			{
				auto& mssi = gltfModel.accessors[mas[ss].input];  // 前フレーム情報
				as[ss].interpolateKeyframes.resize(mssi.count);		bufsize += mssi.count * sizeof(as[ss].interpolateKeyframes);
			}

			//モーフメッシュのTexCoordを初回アニメの初回フレームのみで収集するので制限数を設定
			if (th == 1 && aniModel.morphMesh.TexCoordCount == (unsigned)-1)
				aniModel.morphMesh.TexCoordCount = aniModel.morphMesh.TexCoord.size();
		}
		LOG_INFO(U"TOTAL BUFFER SIZE:{} Bytes"_fmt(bufsize)); //900,384,000 Bytes


#pragma omp parallel for
		for ( int32 cf = 0; cf < cycleframe; cf++)
		{
			int32 th = omp_get_thread_num();
			double bwt[10];
//			LOG_INFO(U"START:Thread[{}] Frame[{}]"_fmt(th,cf));
bwt[0] = omp_get_wtime();	//START

			auto& gm = gltfModel;
			auto& frametime = frametimes[cf];

			auto& man = gm.animations[ animeid ];
			auto& mas = man.samplers;
			auto& mac = man.channels;

			auto& macc = gm.accessors;
			auto& mbv = gm.bufferViews;
			auto& mb = gm.buffers;

			auto& as = aniModel.precAnimes[ animeid ].Samplers[th];
			auto& ac = aniModel.precAnimes[ animeid ].Channels[th];

			Array<NodeParam> nodeAniParams( gm.nodes.size() );

			//全ノードの基本姿勢を取得
			for (int32 nn = 0; nn < gm.nodes.size(); nn++)
				gltfSetupPosture(gm, nn, nodeAniParams );

bwt[1] = omp_get_wtime();	//GETPOSE

			// GLTFサンプラを取得
			for (int32 ss = 0; ss < mas.size(); ss++)
			{
				auto& msi = gm.accessors[mas[ss].input];  // 前フレーム情報

				as[ss].minTime = 0;
				as[ss].maxTime = 1;
				if (msi.minValues.size() > 0 && msi.maxValues.size() > 0)
				{
					as[ss].minTime = float(msi.minValues[0]);
					as[ss].maxTime = float(msi.maxValues[0]);
				}

				for (int32 kk = 0; kk < msi.count; kk++)
				{
					auto& bsi = mas[ss].input;
					const auto& offset = mbv[macc[bsi].bufferView].byteOffset + macc[bsi].byteOffset;
					const auto& stride = msi.ByteStride(mbv[msi.bufferView]);
					void* adr = &mb[mbv[macc[bsi].bufferView].buffer].data.at(offset + kk * stride);

					auto& ctype = macc[bsi].componentType;
					float value =   (ctype == 5126) ? *(float*)adr :
									(ctype == 5123) ? *(uint16*)adr :
									(ctype == 5121) ? *(uint8_t*)adr :
									(ctype == 5122) ? *(int16*)adr :
									(ctype == 5120) ? *(int8_t*)adr : 0.0;

					as[ss].interpolateKeyframes[kk] = std::make_pair(kk, value);
				}
			}
bwt[2] = omp_get_wtime();	//GETSAMPLER

			// GLTFチャネルを取得
			for (int32 cc = 0; cc < ac.size(); cc++)
			{
				auto& maso = gm.accessors[mas[cc].output];
				auto& macso = macc[mas[mac[cc].sampler].output];
				const auto& stride = maso.ByteStride(mbv[maso.bufferView]);
				const auto& offset = mbv[macso.bufferView].byteOffset + macso.byteOffset;

				ac[cc].idxNode = mac[cc].target_node;
				ac[cc].idxSampler = mac[cc].sampler;

				//メッシュの場合は、モーフ数を記録
				if ( gm.nodes[mac[cc].target_node].mesh != -1)
					ac[cc].numMorph = gm.meshes[ gm.nodes[ mac[cc].target_node ].mesh ].weights.size() ;

				//ウェイト補間はモーフ専用
				if (mac[cc].target_path == "weights")
				{
					ac[cc].typeDelta = 5;   //5:weight Array※ShapeAnimeはWeightのみだが、全フレームｘ全モーフに対応する配列ウェイトを構築

					for (int32 ff = 0; ff<maso.count/ac[cc].numMorph; ff++)    //nummorph(22) x 全フレーム409
					{
						float* weight = (float*)&mb[mbv[macso.bufferView].buffer].data.at(offset + ff * stride * ac[cc].numMorph);
						ac[cc].deltaKeyframes[ff].first = ff;

						for (int32 mm = 0; mm<ac[cc].numMorph; mm++)   //nummorph(22) x 全フレーム409
							ac[cc].deltaKeyframes[ff].second[mm] = weight[mm] ;
					}
				}

				else if (mac[cc].target_path == "translation")
				{
					ac[cc].typeDelta = 1;   //translate

					for (int32 ff = 0; ff < maso.count; ff++)
					{
						float* tra = (float*)&mb[mbv[macso.bufferView].buffer].data.at(offset + ff * stride);
						ac[cc].deltaKeyframes[ff].first = ff;
						ac[cc].deltaKeyframes[ff].second[0] = tra[0];
						ac[cc].deltaKeyframes[ff].second[1] = tra[1];
						ac[cc].deltaKeyframes[ff].second[2] = tra[2];
					}
				}

				else if (mac[cc].target_path == "rotation")
				{
					ac[cc].typeDelta = 3;

					for (int32 ff = 0; ff < maso.count; ff++)
					{
						float* rot = (float*)&mb[mbv[macso.bufferView].buffer].data.at(offset + ff * stride);
						auto qt = Quaternion(rot[0], rot[1], rot[2], rot[3]).normalize();

						ac[cc].deltaKeyframes[ff].first = ff;
						ac[cc].deltaKeyframes[ff].second[0] = qt.getX();
						ac[cc].deltaKeyframes[ff].second[1] = qt.getY();
						ac[cc].deltaKeyframes[ff].second[2] = qt.getZ();
						ac[cc].deltaKeyframes[ff].second[3] = qt.getW();
					}
				}

				else if (mac[cc].target_path == "scale")
				{
					ac[cc].typeDelta = 2;

					for (int32 ff = 0; ff < maso.count; ff++)
					{
						float* sca = (float*)&mb[mbv[macso.bufferView].buffer].data.at(offset + ff * stride);
						ac[cc].deltaKeyframes[ff].first = ff;
						ac[cc].deltaKeyframes[ff].second[0] = sca[0];
						ac[cc].deltaKeyframes[ff].second[1] = sca[1];
						ac[cc].deltaKeyframes[ff].second[2] = sca[2];
					}
				}
			}

bwt[3] = omp_get_wtime();	//GETCHANNEL

			// 形状確定
            Array<float> shapeAnimeWeightArray;									//フレームにおける全モーフターゲットのウェイト値
			for (int32 i= 0;i<ac.size(); i++)//メッシュ番号
			{
				Sampler& sa = as[ ac[i].idxSampler ];
				std::pair<int32, float> f0, f1;

				for (int32 kf = 1; kf < sa.interpolateKeyframes.size() ; kf++)   //BoneSamplerキーフレームリストから現在時刻を含む要素を選択
				{
					f0 = sa.interpolateKeyframes[kf-1];
					f1 = sa.interpolateKeyframes[kf];
					if (f0.second <= frametime && frametime < f1.second ) break;
				}

				float &lowtime = f0.second;
				float &upptime = f1.second;
				const int32 &lowframe = f0.first;
				const int32 &uppframe = f1.first;

				// 再生時刻を正規化して中間フレーム時刻算出
				const float mix = (frametime - lowtime) / (upptime - lowtime);

				//キーフレーム間ウェイト、補間モード、下位フレーム/時刻、上位フレーム/時刻から姿勢確定
				auto& interpol = mas[ ac[i].idxSampler ].interpolation;
				if      (interpol == "STEP")        gltfInterpolateStep  (gm, ac[i], lowframe, nodeAniParams );
				else if (interpol == "LINEAR")      gltfInterpolateLinear(gm, ac[i], lowframe, uppframe, mix, nodeAniParams );
				else if (interpol == "CUBICSPLINE") gltfInterpolateSpline(gm, ac[i], lowframe, uppframe, lowtime, upptime, mix, nodeAniParams );

				if (ac[i].idxSampler == -1) continue;

				Array<float>& l = ac[i].deltaKeyframes[lowframe].second;
				Array<float>& u = ac[i].deltaKeyframes[uppframe].second;

				shapeAnimeWeightArray.resize(ac[i].numMorph);
				for (int32 mm = 0; mm < ac[i].numMorph; mm++)
				{
					float weight = l[mm] * (1.0 - mix) + u[mm] * mix;
					shapeAnimeWeightArray[mm] = weight;
				}
			}
bwt[4] = omp_get_wtime();	//FIXPOSE

			// 姿勢確定(Bone)→姿勢変換行列生成(matlocal)
			for (int32 nn = 0; nn < gm.nodes.size(); nn++)
			{
				auto& mn = gm.nodes[nn];
				for (int32 cc = 0; cc < mn.children.size(); cc++)
					gltfCalcSkeleton(gm, Mat4x4::Identity(), mn.children[cc], nodeAniParams );
			}

bwt[5] = omp_get_wtime();	//SKELETON

			//フラット(ツリー構造でない)なスキニング行列(JOINT)リストを生成
			Array<Array<Mat4x4>> Joints( gm.skins.size() );
			for (int32 nn = 0; nn < gm.nodes.size(); nn++)
			{
				auto& mn = gm.nodes[nn];
				for (int32 cc = 0; cc < mn.children.size(); cc++)
				{
					auto& node = gm.nodes[mn.children[cc]];
					if (node.skin < 0) continue;         //LightとCameraをスキップ

					auto& msns = gm.skins[node.skin];
					auto& ibma = gm.accessors[msns.inverseBindMatrices];
					auto& ibmbv = gm.bufferViews[ibma.bufferView];
					auto  ibmd = gm.buffers[ibmbv.buffer].data.data() + ibma.byteOffset + ibmbv.byteOffset;

					Joints[node.skin].resize( msns.joints.size() );
					for (int32 ii = 0; ii < msns.joints.size(); ii++)
					{
						Mat4x4 ibm = *(Mat4x4*)&ibmd[ii * sizeof(Mat4x4)];
						Mat4x4 matworld = nodeAniParams[ msns.joints[ii] ].matWorld;
						Joints[node.skin][ii] = ibm * matworld;
					}
				}
			}

bwt[6] = omp_get_wtime();	//FLATJOINT
			for (int32 nn = 0; nn < gm.nodes.size(); nn++)
			{
				for (int32 cc = 0; cc < gm.nodes[nn].children.size(); cc++)
				{
					auto& node = gm.nodes[ gm.nodes[nn].children[cc] ];
					if (node.mesh >= 0)//メッシュノード　
					{
						gltfComputeMesh( gm, cf, animeid, node, shapeAnimeWeightArray, Joints,bwt );
					}
				}
			}
bwt[7] = omp_get_wtime();	//VERTICE

			Joints.clear();
			shapeAnimeWeightArray.clear();
			nodeAniParams.clear();

			//頂点の最大最小からサイズを計算してAABBを設定
			Float3 vmin = { FLT_MAX,FLT_MAX,FLT_MAX };
			Float3 vmax = { FLT_MIN,FLT_MIN,FLT_MIN };

			//アニメモデルはアニメ番号、フレーム番号で衝突判定かわる。
			for (int32 i = 0; i < aniModel.precAnimes[ animeid ].Frames[cf].MeshDatas.size(); i++)
			{
				for (int32 ii = 0; ii < aniModel.precAnimes[ animeid ].Frames[cf].MeshDatas[i].vertices.size(); ii++)
				{
					Vertex3D& mv = aniModel.precAnimes[ animeid ].Frames[cf].MeshDatas[i].vertices[ii];
					if (vmin.x > mv.pos.x) vmin.x = mv.pos.x;
					if (vmin.y > mv.pos.y) vmin.y = mv.pos.y;
					if (vmin.z > mv.pos.z) vmin.z = mv.pos.z;
					if (vmax.x < mv.pos.x) vmax.x = mv.pos.x;
					if (vmax.y < mv.pos.y) vmax.y = mv.pos.y;
					if (vmax.z < mv.pos.z) vmax.z = mv.pos.z;
				}
			}

			//アニメモデルは再生時にフレームのサイズをローカル変換してOBBに反映
			aniModel.precAnimes[ animeid ].Frames[cf].obbSize = (vmax - vmin);
			aniModel.precAnimes[ animeid ].Frames[cf].obbCenter = vmin + (vmax - vmin)/2;
			bwt[8] = omp_get_wtime();	//OBB
				
//			LOG_INFO(U"END:Thread[{}] Frame[{}] Time[{:.3f}s] 1:{:.3f}s 2:{:.3f}s 3:{:.3f}s 4:{:.3f}s 5:{:.3f}s 6:{:.3f}s 7:{:.3f}s 8:{:.3f}s"_fmt(
//				      th,cf,omp_get_wtime()-bwt[0],
//			          bwt[0]-bwt[1],bwt[1]-bwt[2],bwt[2]-bwt[3],bwt[3]-bwt[4],bwt[4]-bwt[5],bwt[5]-bwt[6],bwt[6]-bwt[7],bwt[7]-bwt[8]));
		}

		for (int32 th = 0; th < tmax; th++)
		{
			auto& as = aniModel.precAnimes[animeid].Samplers[th];
			auto& ac = aniModel.precAnimes[animeid].Channels[th];
			for (int32 ss = 0; ss < mas.size(); ss++) as[ss].interpolateKeyframes.clear();
			for (int32 cc = 0; cc < ac.size(); cc++) ac[cc].deltaKeyframes.clear();
			for (int32 ss = 0; ss < mas.size(); ss++) as[ss].interpolateKeyframes.shrink_to_fit();
			for (int32 cc = 0; cc < ac.size(); cc++) ac[cc].deltaKeyframes.shrink_to_fit();
			ac.clear();
			as.clear();
			ac.shrink_to_fit();
			as.shrink_to_fit();
		}

		for (int32 cf = 0; cf < cycleframe; cf++)
			aniModel.precAnimes[animeid].Frames[cf].MeshDatas.shrink_to_fit();

		aniModel.precAnimes[animeid].Samplers.clear();
		aniModel.precAnimes[animeid].Channels.clear();
		aniModel.precAnimes[animeid].Samplers.shrink_to_fit();
		aniModel.precAnimes[animeid].Channels.shrink_to_fit();

	}

	void gltfInterpolateStep(tinygltf::Model& model, Channel& ch, int32 lowframe, Array<NodeParam>& _nodeParams )
    {
		Array<float>& v = ch.deltaKeyframes[lowframe].second;
		if		(ch.typeDelta == 1) _nodeParams[ch.idxNode].posePos = Float3{ v[0], v[1], v[2] };
		else if (ch.typeDelta == 3) _nodeParams[ch.idxNode].poseRot = Float4{ v[0], v[1], v[2], v[3] };
		else if	(ch.typeDelta == 2) _nodeParams[ch.idxNode].poseSca = Float3{ v[0], v[1], v[2] };
	}

    void gltfInterpolateLinear(tinygltf::Model& model, Channel& ch, int32 lowframe, int32 uppframe, float tt, Array<NodeParam>& _nodeParams)
    {
		Array<float>& l = ch.deltaKeyframes[lowframe].second;
		Array<float>& u = ch.deltaKeyframes[uppframe].second;
		if (ch.typeDelta == 1)		//Translation
        {
			Float3 low{ l[0], l[1], l[2] };
			Float3 upp{ u[0], u[1], u[2] };
			_nodeParams[ch.idxNode].posePos = low * (1.0 - tt) + upp * tt;
		}

		else if (ch.typeDelta == 3)		//Rotation
        {
			Float4 low{ l[0], l[1], l[2], l[3] };
			Float4 upp{ u[0], u[1], u[2], l[3] };
            Quaternion lr = Quaternion(low.x, low.y, low.z, low.w);
            Quaternion ur = Quaternion(upp.x, upp.y, upp.z, upp.w);
            Quaternion mx = lr.slerp(ur, tt).normalize();
			_nodeParams[ch.idxNode].poseRot = Float4{ mx.getX(), mx.getY(), mx.getZ(), mx.getW() };
		}

		else if (ch.typeDelta == 2)		//Scale
        {
			Float3 low{ l[0], l[1], l[2] };
			Float3 upp{ u[0], u[1], u[2] };
			_nodeParams[ch.idxNode].poseSca = low * (1.0 - tt) + upp * tt;
        }
    }

    //付録C：スプライン補間
	//(https://github.com/KhronosGroup/glTF/tree/master/specification/2.0#appendix-c-spline-interpolation)
    template <typename T> T cubicSpline(float tt, T v0, T bb, T v1, T aa)
    {
        const auto t2 = tt * tt;
        const auto t3 = t2 * tt;
        return (2 * t3 - 3 * t2 + 1) * v0 + (t3 - 2 * t2 + tt) * bb + (-2 * t3 + 3 * t2) * v1 + (t3 - t2) * aa;
    }

    void gltfInterpolateSpline(tinygltf::Model& model, Channel& ch, int32 lowframe,
		                       int32 uppframe, float lowtime, float upptime, float tt, Array<NodeParam>& _nodeParams)
    {
		float delta = upptime - lowtime;

		Array<float>& l = ch.deltaKeyframes[3 * lowframe + 1].second;
		Float4 v0{ l[0],l[1],l[2],l[3] };

		Array<float>& a = ch.deltaKeyframes[3 * uppframe + 0].second;
		Float4 aa = delta * Float4{ a[0],a[1],a[2],a[3] };

		Array<float>& b = ch.deltaKeyframes[3 * lowframe + 2].second;
		Float4 bb = delta * Float4{ b[0],b[1],b[2],b[3] };

		Array<float>& u = ch.deltaKeyframes[3 * uppframe + 1].second;
		Float4 v1{ u[0],u[1],u[2],u[3] };

		if (ch.typeDelta == 1) //Translate
			_nodeParams[ch.idxNode].posePos = cubicSpline(tt, v0, bb, v1, aa).xyz();

		else if (ch.typeDelta == 3) //Rotation
        {
            Float4 val = cubicSpline(tt, v0, bb, v1, aa);
            Quaternion qt = Quaternion(val.x, val.y, val.z, val.w).normalize();
			_nodeParams[ch.idxNode].poseRot = qt.toFloat4();
		}

		else if (ch.typeDelta == 2) //Scale
			_nodeParams[ch.idxNode].poseSca = cubicSpline(tt, v0, bb, v1, aa).xyz();
    }
    
	//glTFノード番号をハンドルとして取得
	//ノード番号がJoint番号になるので、この関数の戻り値はJOINT番号。model->skins->joints[JOINT番号]に行列あるので
	//それをいじればボーン操作できる。
	int32 gltfGetJoint( String jointname )
	{
		for (int32 nn = 0; nn < gltfModel.nodes.size(); nn++)
		{
			auto& msn = gltfModel.nodes[nn];
			if (Unicode::FromUTF8(msn.name) == jointname ) return nn;
		}
		return -1;
	}


	//直でJoint行列を変更するのは都合よくない。基準行列+補正行列
	void gltfSetJoint(int32 handle, Float3 trans, Float3 rotate = { 0,0,0 }, Float3 scale = { 1,1,1 } )
	{
		if ( handle < 0 ) return;
		auto& node = gltfModel.nodes[handle];
		__m128 qrot = XMQuaternionRotationRollPitchYaw(ToRadians(rotate.x), ToRadians(rotate.y), ToRadians(rotate.z)) ;
		Mat4x4 matmodify = Mat4x4::Identity().Scale(scale) * Mat4x4(qrot) *Mat4x4::Identity().Translate(trans);
		nodeParams[handle].matModify = matmodify;
		nodeParams[handle].update = true;
	}

	void gltfSetJoint(int32 handle, Float3 trans, Quaternion rotate = { 0,0,0,1 }, Float3 scale = { 1,1,1 })
	{
		if ( handle < 0 ) return;
		auto& node = gltfModel.nodes[handle];
		Mat4x4 matmodify = Mat4x4::Identity().Scale(scale) * Mat4x4(rotate) *Mat4x4::Identity().Translate(trans);
		nodeParams[handle].matModify = matmodify;
		nodeParams[handle].update = true;
	}
	Mat4x4 gltfGetMatrix(int32 handle)
	{
		if ( handle < 0 ) return Mat4x4::Identity();
		auto& node = gltfModel.nodes[handle];
		return nodeParams[handle].matWorld;
	} 


    void gltfComputeMesh( tinygltf::Model gm, uint32 cf, int32 animeid, tinygltf::Node& node,
		                  Array<float>& weights, Array<Array<Mat4x4>> &Joints, double *bwt )
    {
        uint32 morphidx = 0;                     //モーフありメッシュの個数

		//このノードのモーフ構築(初回フレームのみ)
		if (cf == 0)
			gltfSetupMorph( node, aniModel.morphMesh); //モーフィング情報(ShapeBuffers)取得

		int32 prsize = gltfModel.meshes[node.mesh].primitives.size();
		for (int32 pp = 0; pp < prsize; pp++)
		{
bwt[0] = omp_get_wtime();
			PrecAnime& precanime = aniModel.precAnimes[animeid];
			auto& pr = gm.meshes[node.mesh].primitives[pp];
			auto& map = gm.accessors[pr.attributes["POSITION"]];

			int32 opos = 0, otex = 0, onor = 0, ojoints = 0, oweights = 0, oidx = 0;
			int32 type_p = 0, type_t = 0, type_n = 0, type_j = 0, type_w = 0, type_i = 0;
			int32 stride_p = 0, stride_t = 0, stride_n = 0, stride_j = 0, stride_w = 0, stride_i = 0;

            //Meshes->Primitive->Accessor(p,i)->BufferView(p,i)->Buffer(p,i)
			auto& bpos = *getBuffer(gm, pr, "POSITION", &opos, &stride_p, &type_p);        //type_p=5126(float)
			auto& btex = *getBuffer(gm, pr, "TEXCOORD_0", &otex, &stride_t, &type_t);	  //type_t=5126(float)
			auto& bnormal = *getBuffer(gm, pr, "NORMAL", &onor, &stride_n, &type_n);       //type_n=5126(float)
			auto& bjoint = *getBuffer(gm, pr, "JOINTS_0", &ojoints, &stride_j, &type_j);   //type_j=5121(uint8)/5123(uint16)
			auto& bweight = *getBuffer(gm, pr, "WEIGHTS_0", &oweights, &stride_w, &type_w);//type_n=5126(float)
			auto& bidx = *getBuffer(gm, pr, &oidx, &stride_i, &type_i);                    //type_j=5123(uint16)
bwt[1] = omp_get_wtime();
			Array<Vertex3D> vertices;
			for (int32 vv = 0; vv < map.count; vv++)	//頂点
			{
				Vertex3D mv;
				float* basispos = (float*)&bpos.data.at(vv * stride_p + opos);
				float* basistex = (float*)&btex.data.at(vv * stride_t + otex);
				float* basisnor = (float*)&bnormal.data.at(vv * stride_n + onor);

				mv.pos = Float3(basispos[0], basispos[1], basispos[2]);
				mv.tex = Float2(basistex[0], basistex[1]);
				mv.normal = Float3(basisnor[0], basisnor[1], basisnor[2]);

				if ( pr.targets.size() && weights.size() )//BASIS:SHAPESｘWeightで頂点座標(モーフあり)
				{
					for (int32 tt = 0; tt < weights.size(); tt++)
					{
						if (weights[tt] == 0) continue;

//Meshes->Primitive->Target->POSITION->Accessor(p,i)->BufferView(p,i)->Buffer(p,i)
						int32 offsetpos, offsetnor;
						auto& mtpos = *getBuffer(gm, pr, tt, "POSITION", &offsetpos, &stride_p, &type_p);
						auto& mtnor = *getBuffer(gm, pr, tt, "NORMAL", &offsetnor, &stride_n, &type_n);
                        float* spos = (float*)&mtpos.data.at(vv * stride_p + offsetpos);
                        float* snor = (float*)&mtnor.data.at(vv * stride_n + offsetnor);
						Float3 shapepos = Float3(spos[0], spos[1], spos[2]);
                        Float3 shapenor = Float3(snor[0], snor[1], snor[2]);
						mv.pos += shapepos * weights[tt];
						mv.normal += shapenor * weights[tt];
					}
				}

				//CPUスキニング
				if( node.skin >= 0 )
				{
					uint8* jb = (uint8*)&bjoint.data.at(vv * stride_j + ojoints); //1頂点あたり4JOINT
					uint16* jw = (uint16*)&bjoint.data.at(vv * stride_j + ojoints);
					float* wf = (float*)&bweight.data.at(vv * stride_w + oweights);
					Word4 j4 = (type_j == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) ? Word4(jw[0], jw[1], jw[2], jw[3]) :
						                                                            Word4(jb[0], jb[1], jb[2], jb[3]) ;
					Float4 w4 = Float4(wf[0], wf[1], wf[2], wf[3]);

					//4ジョイント合成
					Mat4x4 matskin = w4.x * Joints[node.skin][j4.x] +
									 w4.y * Joints[node.skin][j4.y] +
									 w4.z * Joints[node.skin][j4.z] +
									 w4.w * Joints[node.skin][j4.w];

					if (pr.targets.size() > 0)						//モーフありメッシュ
					{
						precanime.Frames[cf].morphMatBuffers.emplace_back(matskin);
						if (aniModel.morphMesh.TexCoord.size() < aniModel.morphMesh.TexCoordCount)
							aniModel.morphMesh.TexCoord.emplace_back(mv.tex);    //初回アニメの初回フレームで蓄積
					}

					SIMD_Float4 vec4pos = DirectX::XMVector4Transform(SIMD_Float4(mv.pos, 1.0f), matskin);

					Mat4x4 matnor = Mat4x4::Identity();
					if (!(std::abs(matskin.determinant()) < 10e-10))
						matnor = matskin.inverse().transposed();
					
					mv.pos = vec4pos.xyz() /vec4pos.getW();
					mv.normal = SIMD_Float4{ DirectX::XMVector4Transform(SIMD_Float4(mv.normal, 1.0f), matskin) }.xyz();
				}
				vertices.emplace_back(mv);							//頂点座標を登録
			}
bwt[2] = omp_get_wtime();

			MeshData md;
			if (pr.indices > 0)										//頂点インデクサ生成
			{
				auto& mapi = gm.accessors[pr.indices];
				Array<s3d::TriangleIndex32> indices;

				for (int32 ii = 0; ii < mapi.count; ii += 3)
				{
					s3d::TriangleIndex32 idx;
					if (mapi.componentType == 5123)
					{
						uint16* ibuf = (uint16*)&bidx.data.at(ii * 2 + oidx); //16bit
						idx.i0 = ibuf[0]; idx.i1 = ibuf[1]; idx.i2 = ibuf[2];
					}
					else if (mapi.componentType == 5125)
					{
						uint32* ibuf = (uint32*)&bidx.data.at(ii * 4 + oidx); //32bit
						idx.i0 = ibuf[0]; idx.i1 = ibuf[1]; idx.i2 = ibuf[2];
					}
					indices.emplace_back(idx);
				}

				//基準モデルのメッシュを生成
				md = MeshData(vertices, indices);
				vertices.clear();
				indices.clear();
			}
bwt[3] = omp_get_wtime();

			int32 usetex = 0;// テクスチャ頂点
			Texture tex;
            ColorF col = ColorF(1);

			//Meshes->Primitive->Metarial->baseColorTexture.json_double_value.index->images
			if (pr.material >= 0)
			{
				auto& nt = gm.materials[pr.material].additionalValues["normalTexture"];  //法線マップ
				int32 idx = -1;
				auto& mmv = gm.materials[pr.material].values;
				auto& bcf = mmv["baseColorFactor"];                                         //色

				if (mmv.count("baseColorTexture"))
				{
					int32 texidx = mmv["baseColorTexture"].json_double_value["index"];
					idx = gm.textures[texidx].source;
				}
				//頂点色を登録
				if (bcf.number_array.size()) col = ColorF(bcf.number_array[0],
															bcf.number_array[1],
															bcf.number_array[2],
															bcf.number_array[3]);
				else                         col = ColorF(1);

				//テクスチャを登録
				//materials->textures->images
				if (idx >= 0 && gm.images.size())
				{
					if (cf == 0)  //テクスチャと頂点色の登録はフレーム0に保存。
					{
						tex = Texture();
						if (gm.images[idx].bufferView >= 0)
						{
							auto& bgfx = gm.bufferViews[gm.images[idx].bufferView];
							auto bimg = &gm.buffers[bgfx.buffer].data.at(bgfx.byteOffset);
							tex = Texture(MemoryReader{ bimg,bgfx.byteLength }, TextureDesc::MippedSRGB);
						}
						else
						{
                              auto& mii = gm.images[idx].image;
							  tex = Texture( MemoryReader { (void*)&mii,mii.size()}, TextureDesc::MippedSRGB);
						}

						//TODO テクスチャ共有だしカラーのみテクスチャ無しもあるので必要最低限とするのが正しいけど、
						//現状の実装はマテリアル＝テクスチャなので、下でないとうごかない。メモリもったいない
						//						precanime.meshTexs.emplace_back(tex);           // テクスチャ追加
						//						precanime.meshColors.emplace_back(col);         // 頂点色追加
					}
					usetex = 1;
				}
			}
bwt[4] = omp_get_wtime();

			//ここにあるとテクスチャがメッシュ分(16)登録されてしまう。実際のテクスチャは2
			if (cf == 0)  //テクスチャと頂点色の登録はフレーム0に保存
			{
				precanime.meshTexs.emplace_back(tex);           // テクスチャ追加
				precanime.meshColors.emplace_back(col);         // 頂点色追加
			}

			precanime.Frames[cf].MeshDatas.emplace_back(md);				// メッシュ追加(後でOBB設定用/デバッグ用)
			precanime.Frames[cf].Meshes.emplace_back(DynamicMesh{ md });	// メッシュ追加(静的)※動的は頂点座標のみ別バッファ→DynamicMesh生成で対応
			precanime.Frames[cf].useTex.emplace_back(usetex);				// テクスチャ頂点色識別子追加
bwt[5] = omp_get_wtime();	

//			LOG_INFO(U"END:Thread[{}] Frame[{}] Time[{:.3f}s] 1:{:.3f}s 2:{:.3f}s 3:{:.3f}s 4:{:.3f}s 5:{:.3f}s"_fmt(
//				      th,cf,omp_get_wtime()-bwt[0],
//			          bwt[0]-bwt[1],bwt[1]-bwt[2],bwt[2]-bwt[3],bwt[3]-bwt[4],bwt[4]-bwt[5]));

		}
bwt[6] = omp_get_wtime();
int32 th = omp_get_thread_num();
//		LOG_INFO(U"END:Thread[{}] Frame[{}] Time[{:.3f}ms] 1:{:.3f}ms 2:{:.3f}ms 3:{:.3f}ms 4:{:.3f}ms 5:{:.3f}ms 6:{:.3f}ms"_fmt(
//				    th,cf,omp_get_wtime()-bwt[0],
//			        (bwt[0]-bwt[1])*1000,(bwt[1]-bwt[2])*1000,(bwt[2]-bwt[3])*1000,(bwt[3]-bwt[4])*1000,(bwt[4]-bwt[5])*1000,(bwt[0]-bwt[6])*1000));

	}

    //drawframe = -1:時間フレーム(アニメ) -1以外:固定フレーム
    void drawAnime( int32 drawframe = NOTUSE, ColorF usrColor=ColorF(NOTUSE), int32 istart = NOTUSE, int32 icount = NOTUSE)
    {
        Rect rectdraw = camera.getSceneRect();
        matVP = camera.getViewProj();

        AnimeModel& ani = aniModel;
		auto anime_no = 0;// ent.Maple->ArgI;    //アニメ番号取得
        __m128 qrot = XMQuaternionRotationRollPitchYaw(ToRadians(eRot.x), ToRadians(eRot.y), ToRadians(eRot.z));
        Mat4x4 mrot =  Mat4x4(Quaternion(qrot));

        Float3 t = lPos + rPos;

//GL系メッシュをDX系で描画時はXミラーして逆カリング
		//Mat4x4 mat = Mat4x4::Identity().scaled(lSca) * mrot * Mat4x4::Identity().translated(t) ;
		Mat4x4 mat = Mat4x4::Identity().Scale(Float3{ -lSca.x,lSca.y,lSca.z }) * mrot * Mat4x4::Identity().Translate(t);        

        PrecAnime& anime = ani.precAnimes[(anime_no == -1) ? 0 : anime_no];
        int32& cf = (drawframe == -1) ? currentFrame : drawframe;

        Frame& frame = anime.Frames[cf];

        uint32 morphidx = 0;								//モーフありメッシュの個数
        uint32 tid = 0;

		for (uint32 i = 0; i < frame.Meshes.size(); i++)	//1フレームを構成するメッシュ数
        {
            int32& morphs = ani.morphMesh.Targets[i];

			//動的モーフありの場合は、BASISと指定4CHのモーフを
			//BasisBuffersとShapeBuffersの間でウェイトを掛けたメッシュに差し替え。
			//アニメの各フレームに対応する4ジョイント合成行列を適用してDynamicMeshで差し替え
			//作動指示はメイン側で実施。

            if (morphs > 0 && morphTargetInfo.size() )        //モーフありメッシュのみモーフ事前処理
            {
                Array<Vertex3D> morphmv = ani.morphMesh.BasisBuffers[morphidx]; //元メッシュのコピーを作業用で確保
                Array<Array<Vertex3D>>& buf = ani.morphMesh.ShapeBuffers;

                for (uint32 ii = 0; ii < morphmv.size(); ii++)
                {
                    for (uint32 iii = 0; iii < NUMMIX; iii++)   //4chモーフ合成(目、口、歯、表情を非同期で動かす)
                    {
                        int32& now = morphTargetInfo[iii].NowTarget;
                        int32& dst = morphTargetInfo[iii].DstTarget;
                        int32& idx = morphTargetInfo[iii].IndexTrans;
                        Array<float>& wt =  morphTargetInfo[iii].WeightTrans;

                        if (now == -1) continue;    //NowTargetが-1の場合はモーフ無効

                        if (idx == -1)              //IndexTransが-1の場合はWeightTrans[0]でマニュアルウェイト
                        {
                            morphmv[ii].pos += buf[morphidx * NUMMIX + iii][ii].pos * (1 - wt[0]) +
                                               buf[morphidx * NUMMIX + iii][ii].pos * wt[0];
                            morphmv[ii].normal += buf[morphidx * NUMMIX + iii][ii].normal * (1 - wt[0]) +
                                                  buf[morphidx * NUMMIX + iii][ii].normal * wt[0];
                        }

						else						//マニュアルウェイト以外(ウェイトテーブルによるブリンクと表情遷移)
                        {
                            auto weight = (wt[idx] < 0) ? 0 : wt[idx];
                            morphmv[ii].pos += buf[morphidx * NUMMIX + now][ii].pos * (1 - weight) +
                                               buf[morphidx * NUMMIX + dst][ii].pos * weight;
                            morphmv[ii].normal += buf[morphidx * NUMMIX + now][ii].normal * (1 - weight) +
                                                  buf[morphidx * NUMMIX + dst][ii].normal * weight;
                        }
                    }

					//アニメの各フレームに対応する4ジョイント合成行列を適用
                    Mat4x4& matskin = frame.morphMatBuffers[ii];
                    SIMD_Float4 vec4pos = DirectX::XMVector4Transform(SIMD_Float4(morphmv[ii].pos, 1.0f), matskin);
                    Mat4x4 matnor = matskin.inverse().transposed();

					morphmv[ii].pos = vec4pos.xyz() /vec4pos.getW();
					morphmv[ii].normal = SIMD_Float4{ DirectX::XMVector4Transform(SIMD_Float4(morphmv[ii].normal, 1.0f), matnor) }.xyz();
                    morphmv[ii].tex = ani.morphMesh.TexCoord[i];
                }

				frame.Meshes[i].fill(morphmv);				//頂点座標を再計算後にすげ替え
                morphidx++;
            }

			if (istart == NOTUSE)	//部分描画なし
			{
				if (frame.useTex[i])	//テクスチャ色
					frame.Meshes[i].draw(mat, anime.meshTexs[i], anime.meshColors[i]);

				else				//マテリアル色
				{
					if ( usrColor.a >= USRCOLOR_NOTUSE )		//ユーザー色指定なし
						frame.Meshes[i].draw(mat, anime.meshColors[i]);
					else if	( usrColor.a == USRCOLOR_OFFSET )	//ユーザー色相対指定
						frame.Meshes[i].draw(mat, anime.meshColors[i] + ColorF(usrColor.rgb(),1) );
					else if	( usrColor.a == USRCOLOR )	//ユーザー色絶対指定
						frame.Meshes[i].draw(mat, ColorF(usrColor.rgb(),1) );
				}
			}
			else				//部分描画あり
			{
				if (frame.useTex[i])
					frame.Meshes[i].drawSubset(istart, icount, mat, anime.meshTexs[tid++]);
				else				//マテリアル色
				{
					if ( usrColor.a >= USRCOLOR_NOTUSE )		//ユーザー色指定なし
						frame.Meshes[i].drawSubset(istart, icount, mat, anime.meshColors[i]);
					else if	( usrColor.a == USRCOLOR_OFFSET )	//ユーザー色相対指定
						frame.Meshes[i].drawSubset(istart, icount,mat, anime.meshColors[i] + ColorF(usrColor.rgb(),1) );
					else if	( usrColor.a == USRCOLOR )	//ユーザー色絶対指定
						frame.Meshes[i].drawSubset(istart, icount,mat, ColorF(usrColor.rgb(),1) );
				}
            }


        }

        obb.setPos( anime.Frames[cf].obbCenter*lSca ).setSize( anime.Frames[cf].obbSize ).scaled(lSca).movedBy( lPos ) ;
		if (obbDebug) obb.draw( matVP, ColorF(0,0,1,0.1) );

    }

    void nextFrame( uint32 anime_no )
    {
        currentFrame++;
        PrecAnime &anime = aniModel.precAnimes[anime_no];
        if ( currentFrame >= anime.Frames.size() ) currentFrame = 0;
        // else if ( currentFrame < 0) currentFrame = anime.Frames.size() + currentFrame;
    }
};

