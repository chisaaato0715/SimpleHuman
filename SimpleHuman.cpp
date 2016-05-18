/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
**/


#define  _USE_MATH_DEFINES
#include <math.h>

#include <gl/glut.h>

#include "bvh.h"
#include "SimpleHuman.h"


// BVHファイルの位置情報に適用するスケーリング比率（デフォルトでは cm→m への変換）
const float  bvh_scale = 0.01f;


//
//  BVH動作から骨格モデルを生成
//
Skeleton *  CoustructBVHSkeleton( class BVH * bvh )
{
	Skeleton *  body = new Skeleton();

	int  num_segments, num_joints;

	// 体節・関節数の決定
	num_segments = bvh->GetNumJoint();
	num_joints = num_segments - 1;

	body->segments.resize( num_segments );
	for ( int i=0; i<num_segments; i++ )
		body->segments[ i ] = new Segment();
	body->joints.resize( num_joints );
	for ( int i=0; i<num_joints; i++ )
		body->joints[ i ] = new Joint();

	int  num_connecting_joints;

	// 体節を初期化
	for ( int i=0; i<num_segments; i++ )
	{
		Segment *  segment = body->segments[ i ];

		// 体節に対応する BVH の関節を取得
		const BVH::Joint *  parts = bvh->GetJoint( i );

		// 体節番号・名前を設定
		segment->index = i;
		segment->name = parts->name;

		// 関節番号・名前を設定
		if ( i != 0 )
		{
			body->joints[ i - 1 ]->index = i - 1;
			body->joints[ i - 1 ]->name = parts->name;
		}

		// 体節に接続する関節数（子ノード数 ＋ ルートノード）
		bool  is_root = ( i == 0 );
		if ( is_root )
			num_connecting_joints = parts->children.size();
		else
			num_connecting_joints = parts->children.size() + 1;

		// 各接続関節・接続位置を取得
		segment->joints.resize( num_connecting_joints );
		segment->joint_positions.resize( num_connecting_joints );
		if ( !is_root )
		{
			segment->joint_positions[ 0 ].set( 0.0f, 0.0f, 0.0f );
			segment->joints[ 0 ] = body->joints[ parts->index - 1 ];
		}
		for ( int j = (is_root ? 0 : 1), c=0; j<num_connecting_joints; j++, c++ )
		{
			const BVH::Joint *  child = parts->children[ c ];
			segment->joints[ j ] = body->joints[ child->index - 1 ];
			segment->joint_positions[ j ].set( child->offset[ 0 ], child->offset[ 1], child->offset[ 2 ] );
		}

		// 末端位置のオフセットを取得
		segment->has_site = parts->has_site;
		if ( parts->has_site )
			segment->site_position.set( parts->site[ 0 ], parts->site[ 1], parts->site[ 2 ] );

		// 各関節の接続位置を全接続位置の中心からの相対位置に変換（ルート体節以外）
		if ( !is_root )
		{
			Vector3f  center( 0.0f, 0.0f, 0.0f );
			for ( int j=0; j<num_connecting_joints; j++ )
				center.add( segment->joint_positions[ j ] );
			if ( parts->has_site )
				center.add( segment->site_position );
			if ( parts->has_site )
				center.scale( 1.0f / (float)( num_connecting_joints + 1.0 ) );
			else
				center.scale( 1.0f / (float)num_connecting_joints );
			for ( int j=0; j<num_connecting_joints; j++ )
				segment->joint_positions[ j ].sub( center );
			if ( parts->has_site )
				segment->site_position -= center;
		}
		for ( int j=0; j<num_connecting_joints; j++ )
			segment->joint_positions[ j ].scale( bvh_scale );
		if ( parts->has_site )
			segment->site_position.scale( bvh_scale );

		// 関節の接続体節情報を設定
		for ( int j = (is_root ? 0 : 1), c=0; j<num_connecting_joints; j++, c++ )
		{
			Joint *  joint = segment->joints[ j ];
			const BVH::Joint *  child = parts->children[ c ];
			Segment *  child_segment = body->segments[ child->index ];
			joint->segments[ 0 ] = segment;
			joint->segments[ 1 ] = child_segment;
		}
	}

	return  body;
}


//
//  姿勢の初期化
//
void  InitPosture( Posture & posture, Skeleton * body )
{
	// 骨格情報を設定
	if ( body )
		posture.body = body;
	if ( !posture.body )
		return;

	// 姿勢情報を初期化
	posture.root_pos.set( 0.0f, 0.0f, 0.0f );
	posture.root_ori.setIdentity();
	posture.joint_rotations.resize( posture.body->joints.size() );
	for ( int i=0; i<posture.body->joints.size(); i++ )
		posture.joint_rotations[ i ].setIdentity();

	// 適当な腰の高さを計算・設定
	//（本来は一度計算した高さを記録しておけば毎回計算する必要はない）
	vector< Matrix4f >  seg_frame_array;
	ForwardKinematics( posture, seg_frame_array );
	float  root_height = 0.0f;
	for ( int i=0; i<posture.body->segments.size(); i++ )
		if ( root_height > seg_frame_array[ i ].m13 )
			root_height = seg_frame_array[ i ].m13;
	posture.root_pos.y = - root_height + 0.05f; // 適当なマージンを加算
}


//
//  BVH動作の回転を計算（オイラー角表現から回転行列表現に変換）
//
void  ComputeBVHJointRotation( int num_channels, const BVH::Channel * const * channels, const float * angles, Matrix3f & rot )
{
	Matrix3f  axis_rot;
	rot.setIdentity();
	for ( int i=0; i<num_channels; i++ )
	{
		switch ( channels[ i ]->type )
		{
		  case BVH::X_ROTATION:
			axis_rot.rotX( angles[ i ] );
			break;
		  case BVH::Y_ROTATION:
			axis_rot.rotY( angles[ i ] );
			break;
		  case BVH::Z_ROTATION:
			axis_rot.rotZ( angles[ i ] );
			break;
		  default:
			axis_rot.setIdentity();
		}
		rot.mul( rot, axis_rot );
	}
}


//
//  BVH動作から姿勢を取得
//
void  GetBVHPosture( const BVH * bvh, int frame_no, Posture & posture )
{
	if ( !bvh || !bvh->IsLoadSuccess() || !posture.body )
		return;
	if ( bvh->GetNumJoint() < posture.body->joints.size() )
		return;

	const Skeleton *  body = posture.body;

	// ルートの位置・向きを取得

	Vector3f  root_pos;
	Matrix3f  rot;
	BVH::Channel *  root_rot_channels[ 3 ];
	float  angles[ 3 ];


	// ルート関節の位置・向きを取得
	const BVH::Joint *  bvh_root = bvh->GetJoint( 0 );
	int  c = 0;
	for ( int j=0; j<bvh_root->channels.size(); j++ )
	{
		switch ( bvh_root->channels[ j ]->type )
		{
			case BVH::X_POSITION:
			root_pos.x = bvh->GetMotion( frame_no, bvh_root->channels[ j ]->index );
			break;
			case BVH::Y_POSITION:
			root_pos.y = bvh->GetMotion( frame_no, bvh_root->channels[ j ]->index );
			break;
			case BVH::Z_POSITION:
			root_pos.z = bvh->GetMotion( frame_no, bvh_root->channels[ j ]->index );
			break;
			case BVH::X_ROTATION:
			root_rot_channels[ c++ ] = bvh_root->channels[ j ];
			break;
			case BVH::Y_ROTATION:
			root_rot_channels[ c++ ] = bvh_root->channels[ j ];
			break;
			case BVH::Z_ROTATION:
			root_rot_channels[ c++ ] = bvh_root->channels[ j ];
			break;
		}
	}
	if ( c == 3 )
	{
		for ( int j=0; j<3; j++ )
			angles[ j ] = bvh->GetMotion( frame_no, root_rot_channels[ j ]->index ) * M_PI / 180.0f;
		ComputeBVHJointRotation( 3, root_rot_channels, angles, rot );
	}
	else
		rot.setIdentity();

	// ルート関節の位置・向きを設定
	root_pos.scale( bvh_scale );
	posture.root_pos = root_pos;
	posture.root_ori = rot;

	// 各関節の回転を取得
	for ( int i=0; i<body->joints.size(); i++ )
	{
		const BVH::Joint *  bvh_joint = bvh->GetJoint( i + 1 );
		int  num_channels = bvh_joint->channels.size();

		// 関節の回転を取得
		for ( int j=0; j<num_channels; j++ )
			angles[ j ] = bvh->GetMotion( frame_no, bvh_joint->channels[ j ]->index ) * M_PI / 180.0f;
		ComputeBVHJointRotation( num_channels, &bvh_joint->channels.front(), angles, rot );

		// 関節の回転を設定
		posture.joint_rotations[ i ] = rot;
	}
}


//
//  Forward Kinematics 計算のための反復計算（ルート体節から末端体節に向かって繰り返し計算）
//
void  ForwardKinematicsIteration( 
	Segment *  segment, Segment * prev_segment, 
	const Posture & posture, Matrix4f * seg_frame_array, Point3f * joi_pos_array = NULL )
{
	const Skeleton *  body = posture.body;
	Joint *  next_joint;
	Segment *  next_segment;
	Matrix4f  mat;
	Vector3f  trans;
	Matrix4f  rot;
	
	// 各接続関節ごとに反復
	for ( int j=0; j<segment->joints.size(); j++ )
	{
		// 次の関節・次の体節を取得
		next_joint = segment->joints[ j ];
		if ( next_joint->segments[ 0 ] != segment )
			next_segment = next_joint->segments[ 0 ];
		else
			next_segment = next_joint->segments[ 1 ];

		// 前の体節側の関節はスキップ
		if ( next_segment == prev_segment )
			continue;

		// 現在の体節の変換行列を取得
		mat = seg_frame_array[ segment->index ];
		
		// 接続関節への座標系へ平行移動
		segment->joint_positions[ j ].get( &trans );
		mat.m03 += mat.m00 * trans.x + mat.m01 * trans.y + mat.m02 * trans.z;
		mat.m13 += mat.m10 * trans.x + mat.m11 * trans.y + mat.m12 * trans.z;
		mat.m23 += mat.m20 * trans.x + mat.m21 * trans.y + mat.m22 * trans.z;

		// 接続関節の位置を設定
		if ( joi_pos_array )
			joi_pos_array[ next_joint->index ].set( mat.m03, mat.m13, mat.m23 );

		// 関節の回転行列をかける
		rot.set( posture.joint_rotations[ next_joint->index ] );
		mat.mul( mat, rot );

		// 次の体節の重心座標系に平行移動
		next_segment->joint_positions[ 0 ].get( &trans );
		mat.m03 -= mat.m00 * trans.x + mat.m01 * trans.y + mat.m02 * trans.z;
		mat.m13 -= mat.m10 * trans.x + mat.m11 * trans.y + mat.m12 * trans.z;
		mat.m23 -= mat.m20 * trans.x + mat.m21 * trans.y + mat.m22 * trans.z;
		
		// 次の体節の変換行列を設定
		seg_frame_array[ next_segment->index ] = mat;

		// 次の体節を呼び出す
		ForwardKinematicsIteration( next_segment, segment, posture, seg_frame_array, joi_pos_array );
	}
}


//
//  順運動学計算
//
void  ForwardKinematics( const Posture & posture, vector< Matrix4f > & seg_frame_array, vector< Point3f > & joi_pos_array )
{
	// 配列初期化
	seg_frame_array.resize( posture.body->segments.size() );
	joi_pos_array.resize( posture.body->joints.size() );

	// ルート体節の位置・向きを設定
	seg_frame_array[ 0 ].set( posture.root_ori, posture.root_pos, 1.0f );

	// Forward Kinematics 計算のための反復計算（ルート体節から末端体節に向かって繰り返し計算）
	ForwardKinematicsIteration( posture.body->segments[ 0 ], NULL, posture, &seg_frame_array.front(), &joi_pos_array.front() );
}


//
//  順運動学計算
//
void  ForwardKinematics( const Posture & posture, vector< Matrix4f > & seg_frame_array )
{
	// 配列初期化
	seg_frame_array.resize( posture.body->segments.size() );

	// ルート体節の位置・向きを設定
	seg_frame_array[ 0 ].set( posture.root_ori, posture.root_pos, 1.0f );

	// Forward Kinematics 計算のための反復計算（ルート体節から末端体節に向かって繰り返し計算）
	ForwardKinematicsIteration( posture.body->segments[ 0 ], NULL, posture, &seg_frame_array.front() );
}


//
//  骨格モデルの１本のリンクを楕円体で描画
//
void  DrawBone( float x0, float y0, float z0, float x1, float y1, float z1, float radius )
{
	// 与えられた２点を結ぶ円柱を描画

	// 円柱の２端点の情報を原点・向き・長さの情報に変換
	GLdouble  dir_x = x1 - x0;
	GLdouble  dir_y = y1 - y0;
	GLdouble  dir_z = z1 - z0;
	GLdouble  bone_length = sqrt( dir_x*dir_x + dir_y*dir_y + dir_z*dir_z );

	// 描画パラメタの設定
	static GLUquadricObj *  quad_obj = NULL;
	if ( quad_obj == NULL )
		quad_obj = gluNewQuadric();
	gluQuadricDrawStyle( quad_obj, GLU_FILL );
	gluQuadricNormals( quad_obj, GLU_SMOOTH );

	glPushMatrix();

	// 平行移動を設定
	glTranslated( ( x0 + x1 ) * 0.5f, ( y0 + y1 ) * 0.5f, ( z0 + z1 ) * 0.5f );

	// 以下、回転を表す行列を計算

	// ｚ軸を単位ベクトルに正規化
	double  length;
	length = sqrt( dir_x*dir_x + dir_y*dir_y + dir_z*dir_z );
	if ( length < 0.0001 ) { 
		dir_x = 0.0; dir_y = 0.0; dir_z = 1.0;  length = 1.0;
	}
	dir_x /= length;  dir_y /= length;  dir_z /= length;

	// 基準とするｙ軸の向きを設定
	GLdouble  up_x, up_y, up_z;
	up_x = 0.0;
	up_y = 1.0;
	up_z = 0.0;

	// ｚ軸とｙ軸の外積からｘ軸の向きを計算
	double  side_x, side_y, side_z;
	side_x = up_y * dir_z - up_z * dir_y;
	side_y = up_z * dir_x - up_x * dir_z;
	side_z = up_x * dir_y - up_y * dir_x;

	// ｘ軸を単位ベクトルに正規化
	length = sqrt( side_x*side_x + side_y*side_y + side_z*side_z );
	if ( length < 0.0001 ) {
		side_x = 1.0; side_y = 0.0; side_z = 0.0;  length = 1.0;
	}
	side_x /= length;  side_y /= length;  side_z /= length;

	// ｚ軸とｘ軸の外積からｙ軸の向きを計算
	up_x = dir_y * side_z - dir_z * side_y;
	up_y = dir_z * side_x - dir_x * side_z;
	up_z = dir_x * side_y - dir_y * side_x;

	// 回転行列を設定
	GLdouble  m[16] = { side_x, side_y, side_z, 0.0,
	                    up_x,   up_y,   up_z,   0.0,
	                    dir_x,  dir_y,  dir_z,  0.0,
	                    0.0,    0.0,    0.0,    1.0 };
	glMultMatrixd( m );

	// 円柱の設定
	GLdouble slices = 16.0; // 円柱の放射状の細分数（デフォルト12）
	GLdouble stack = 16.0;  // 円柱の輪切りの細分数（デフォルト１）

	// 楕円体を描画
	glScalef( radius, radius, bone_length * 0.5f );
	glEnable( GL_NORMALIZE );
	glutSolidSphere( 1.0f, slices, stack );
	glDisable( GL_NORMALIZE );

	glPopMatrix();
}


//
//  姿勢の描画（スティックフィギュアで描画）
//
void  DrawPosture( const Posture & posture )
{
	if ( !posture.body )
		return;

	// 順運動学計算
	vector< Matrix4f >  seg_frame_array;
	vector< Point3f >  joi_pos_array;
	ForwardKinematics( posture, seg_frame_array, joi_pos_array );

	float  radius = 0.05f;
	Matrix4f  mat;
	Vector3f  v1, v2;

	// 各体節の描画
	for ( int i=0; i<seg_frame_array.size(); i++ )
	{
		const Segment *  segment = posture.body->segments[i];
		const int  num_joints = segment->joints.size();

		// 体節の中心の位置・向きを基準とする変換行列を適用
		glPushMatrix();
		mat.transpose( seg_frame_array[ i ] );
		glMultMatrixf( & mat.m00 );

		// １つの関節から末端点へのバーを描画
		if ( ( num_joints == 1 ) && segment->has_site )
		{
			v1 = segment->joint_positions[ 0 ];
			v2 = segment->site_position;
			DrawBone( v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, radius );
		}
		// １つの関節から仮の末端点（重心へのベクトルを２倍した位置）へバーを描画
		else if ( ( num_joints == 1 ) && !segment->has_site )
		{
			v1 = segment->joint_positions[ 0 ];
			v2.negate( v1 );
			DrawBone( v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, radius );
		}
		// ２つの関節を接続するバーを描画
		else if ( num_joints == 2 )
		{
			v1 = segment->joint_positions[ 0 ];
			v2 = segment->joint_positions[ 1 ];
			DrawBone( v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, radius );
		}
		// 重心から各関節へのバーを描画
		else if ( num_joints > 2 )
		{
			v1.set( 0.0f, 0.0f, 0.0f );
			for ( int j=0; j<num_joints; j++ )
			{
				v2 = segment->joint_positions[ j ];
			DrawBone( v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, radius );
			}
		}

		glPopMatrix();
	}
}



// End of BVH.cpp
