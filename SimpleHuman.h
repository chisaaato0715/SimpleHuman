/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
**/


#ifndef  _SIMPLE_HUMAN_H_
#define  _SIMPLE_HUMAN_H_


//
//  行列・ベクトルの表現には vecmath C++ライブラリ（http://objectclub.jp/download/vecmath1）を使用
//
#include <Vector3.h>
#include <Point3.h>
#include <Matrix3.h>
#include <Matrix4.h>


#include <vector>
#include <map>
#include <string>

using namespace  std;

struct  Segment;
struct  Joint;


//
//  多関節体の体節を表す構造体
//
struct  Segment
{
	// 体節番号・名前
	int                  index;
	string               name;

    // 接続関節
    vector< Joint * >    joints;

    // 各関節の接続位置（体節のローカル座標系）
    vector< Point3f >    joint_positions;

	// 体節の末端位置
	bool                 has_site;
	Point3f              site_position;
};


//
//  多関節体の関節を表す構造体
//
struct  Joint
{
	// 関節番号・名前
	int                  index;
	string               name;

    // 接続体節
    Segment *            segments[ 2 ];
};


//
//  多関節体の骨格を表す構造体
//
struct  Skeleton
{
    // 体節・関節の配列
    vector< Segment * >  segments;
    vector< Joint * >    joints;
};


//
//  多関節体の姿勢を表す構造体
//
struct  Posture
{
    Skeleton *           body;
    Point3f              root_pos;        // ルートの位置
    Matrix3f             root_ori;        // ルートの向き（回転行列表現）
    vector< Matrix3f >   joint_rotations; // 各関節の相対回転（回転行列表現）[関節番号]
};


//
//  多関節体の骨格・姿勢・動作の基本処理
//

// BVH動作から骨格モデルを生成
Skeleton *  CoustructBVHSkeleton( class BVH * bvh );

// 姿勢の初期化
void  InitPosture( Posture & posture, Skeleton * body = NULL );

// BVH動作から姿勢を取得
void  GetBVHPosture( const class BVH * bvh, int frame_no, Posture & posture );

// 順運動学計算
void  ForwardKinematics( const Posture & posture, vector< Matrix4f > & seg_frame_array, vector< Point3f > & joi_pos_array );

// 順運動学計算
void  ForwardKinematics( const Posture & posture, vector< Matrix4f > & seg_frame_array );

// 姿勢の描画（スティックフィギュアで描画）
void  DrawPosture( const Posture & posture );



#endif // _SIMPLE_HUMAN_H_
