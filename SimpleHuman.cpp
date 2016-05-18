/**
***  �L�����N�^�A�j���[�V�����̂��߂̐l�̃��f���̕\���E��{����
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
**/


#define  _USE_MATH_DEFINES
#include <math.h>

#include <gl/glut.h>

#include "bvh.h"
#include "SimpleHuman.h"


// BVH�t�@�C���̈ʒu���ɓK�p����X�P�[�����O�䗦�i�f�t�H���g�ł� cm��m �ւ̕ϊ��j
const float  bvh_scale = 0.01f;


//
//  BVH���삩�獜�i���f���𐶐�
//
Skeleton *  CoustructBVHSkeleton( class BVH * bvh )
{
	Skeleton *  body = new Skeleton();

	int  num_segments, num_joints;

	// �̐߁E�֐ߐ��̌���
	num_segments = bvh->GetNumJoint();
	num_joints = num_segments - 1;

	body->segments.resize( num_segments );
	for ( int i=0; i<num_segments; i++ )
		body->segments[ i ] = new Segment();
	body->joints.resize( num_joints );
	for ( int i=0; i<num_joints; i++ )
		body->joints[ i ] = new Joint();

	int  num_connecting_joints;

	// �̐߂�������
	for ( int i=0; i<num_segments; i++ )
	{
		Segment *  segment = body->segments[ i ];

		// �̐߂ɑΉ����� BVH �̊֐߂��擾
		const BVH::Joint *  parts = bvh->GetJoint( i );

		// �̐ߔԍ��E���O��ݒ�
		segment->index = i;
		segment->name = parts->name;

		// �֐ߔԍ��E���O��ݒ�
		if ( i != 0 )
		{
			body->joints[ i - 1 ]->index = i - 1;
			body->joints[ i - 1 ]->name = parts->name;
		}

		// �̐߂ɐڑ�����֐ߐ��i�q�m�[�h�� �{ ���[�g�m�[�h�j
		bool  is_root = ( i == 0 );
		if ( is_root )
			num_connecting_joints = parts->children.size();
		else
			num_connecting_joints = parts->children.size() + 1;

		// �e�ڑ��֐߁E�ڑ��ʒu���擾
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

		// ���[�ʒu�̃I�t�Z�b�g���擾
		segment->has_site = parts->has_site;
		if ( parts->has_site )
			segment->site_position.set( parts->site[ 0 ], parts->site[ 1], parts->site[ 2 ] );

		// �e�֐߂̐ڑ��ʒu��S�ڑ��ʒu�̒��S����̑��Έʒu�ɕϊ��i���[�g�̐߈ȊO�j
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

		// �֐߂̐ڑ��̐ߏ���ݒ�
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
//  �p���̏�����
//
void  InitPosture( Posture & posture, Skeleton * body )
{
	// ���i����ݒ�
	if ( body )
		posture.body = body;
	if ( !posture.body )
		return;

	// �p������������
	posture.root_pos.set( 0.0f, 0.0f, 0.0f );
	posture.root_ori.setIdentity();
	posture.joint_rotations.resize( posture.body->joints.size() );
	for ( int i=0; i<posture.body->joints.size(); i++ )
		posture.joint_rotations[ i ].setIdentity();

	// �K���ȍ��̍������v�Z�E�ݒ�
	//�i�{���͈�x�v�Z�����������L�^���Ă����Ζ���v�Z����K�v�͂Ȃ��j
	vector< Matrix4f >  seg_frame_array;
	ForwardKinematics( posture, seg_frame_array );
	float  root_height = 0.0f;
	for ( int i=0; i<posture.body->segments.size(); i++ )
		if ( root_height > seg_frame_array[ i ].m13 )
			root_height = seg_frame_array[ i ].m13;
	posture.root_pos.y = - root_height + 0.05f; // �K���ȃ}�[�W�������Z
}


//
//  BVH����̉�]���v�Z�i�I�C���[�p�\�������]�s��\���ɕϊ��j
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
//  BVH���삩��p�����擾
//
void  GetBVHPosture( const BVH * bvh, int frame_no, Posture & posture )
{
	if ( !bvh || !bvh->IsLoadSuccess() || !posture.body )
		return;
	if ( bvh->GetNumJoint() < posture.body->joints.size() )
		return;

	const Skeleton *  body = posture.body;

	// ���[�g�̈ʒu�E�������擾

	Vector3f  root_pos;
	Matrix3f  rot;
	BVH::Channel *  root_rot_channels[ 3 ];
	float  angles[ 3 ];


	// ���[�g�֐߂̈ʒu�E�������擾
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

	// ���[�g�֐߂̈ʒu�E������ݒ�
	root_pos.scale( bvh_scale );
	posture.root_pos = root_pos;
	posture.root_ori = rot;

	// �e�֐߂̉�]���擾
	for ( int i=0; i<body->joints.size(); i++ )
	{
		const BVH::Joint *  bvh_joint = bvh->GetJoint( i + 1 );
		int  num_channels = bvh_joint->channels.size();

		// �֐߂̉�]���擾
		for ( int j=0; j<num_channels; j++ )
			angles[ j ] = bvh->GetMotion( frame_no, bvh_joint->channels[ j ]->index ) * M_PI / 180.0f;
		ComputeBVHJointRotation( num_channels, &bvh_joint->channels.front(), angles, rot );

		// �֐߂̉�]��ݒ�
		posture.joint_rotations[ i ] = rot;
	}
}


//
//  Forward Kinematics �v�Z�̂��߂̔����v�Z�i���[�g�̐߂��疖�[�̐߂Ɍ������ČJ��Ԃ��v�Z�j
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
	
	// �e�ڑ��֐߂��Ƃɔ���
	for ( int j=0; j<segment->joints.size(); j++ )
	{
		// ���̊֐߁E���̑̐߂��擾
		next_joint = segment->joints[ j ];
		if ( next_joint->segments[ 0 ] != segment )
			next_segment = next_joint->segments[ 0 ];
		else
			next_segment = next_joint->segments[ 1 ];

		// �O�̑̐ߑ��̊֐߂̓X�L�b�v
		if ( next_segment == prev_segment )
			continue;

		// ���݂̑̐߂̕ϊ��s����擾
		mat = seg_frame_array[ segment->index ];
		
		// �ڑ��֐߂ւ̍��W�n�֕��s�ړ�
		segment->joint_positions[ j ].get( &trans );
		mat.m03 += mat.m00 * trans.x + mat.m01 * trans.y + mat.m02 * trans.z;
		mat.m13 += mat.m10 * trans.x + mat.m11 * trans.y + mat.m12 * trans.z;
		mat.m23 += mat.m20 * trans.x + mat.m21 * trans.y + mat.m22 * trans.z;

		// �ڑ��֐߂̈ʒu��ݒ�
		if ( joi_pos_array )
			joi_pos_array[ next_joint->index ].set( mat.m03, mat.m13, mat.m23 );

		// �֐߂̉�]�s���������
		rot.set( posture.joint_rotations[ next_joint->index ] );
		mat.mul( mat, rot );

		// ���̑̐߂̏d�S���W�n�ɕ��s�ړ�
		next_segment->joint_positions[ 0 ].get( &trans );
		mat.m03 -= mat.m00 * trans.x + mat.m01 * trans.y + mat.m02 * trans.z;
		mat.m13 -= mat.m10 * trans.x + mat.m11 * trans.y + mat.m12 * trans.z;
		mat.m23 -= mat.m20 * trans.x + mat.m21 * trans.y + mat.m22 * trans.z;
		
		// ���̑̐߂̕ϊ��s���ݒ�
		seg_frame_array[ next_segment->index ] = mat;

		// ���̑̐߂��Ăяo��
		ForwardKinematicsIteration( next_segment, segment, posture, seg_frame_array, joi_pos_array );
	}
}


//
//  ���^���w�v�Z
//
void  ForwardKinematics( const Posture & posture, vector< Matrix4f > & seg_frame_array, vector< Point3f > & joi_pos_array )
{
	// �z�񏉊���
	seg_frame_array.resize( posture.body->segments.size() );
	joi_pos_array.resize( posture.body->joints.size() );

	// ���[�g�̐߂̈ʒu�E������ݒ�
	seg_frame_array[ 0 ].set( posture.root_ori, posture.root_pos, 1.0f );

	// Forward Kinematics �v�Z�̂��߂̔����v�Z�i���[�g�̐߂��疖�[�̐߂Ɍ������ČJ��Ԃ��v�Z�j
	ForwardKinematicsIteration( posture.body->segments[ 0 ], NULL, posture, &seg_frame_array.front(), &joi_pos_array.front() );
}


//
//  ���^���w�v�Z
//
void  ForwardKinematics( const Posture & posture, vector< Matrix4f > & seg_frame_array )
{
	// �z�񏉊���
	seg_frame_array.resize( posture.body->segments.size() );

	// ���[�g�̐߂̈ʒu�E������ݒ�
	seg_frame_array[ 0 ].set( posture.root_ori, posture.root_pos, 1.0f );

	// Forward Kinematics �v�Z�̂��߂̔����v�Z�i���[�g�̐߂��疖�[�̐߂Ɍ������ČJ��Ԃ��v�Z�j
	ForwardKinematicsIteration( posture.body->segments[ 0 ], NULL, posture, &seg_frame_array.front() );
}


//
//  ���i���f���̂P�{�̃����N��ȉ~�̂ŕ`��
//
void  DrawBone( float x0, float y0, float z0, float x1, float y1, float z1, float radius )
{
	// �^����ꂽ�Q�_�����ԉ~����`��

	// �~���̂Q�[�_�̏������_�E�����E�����̏��ɕϊ�
	GLdouble  dir_x = x1 - x0;
	GLdouble  dir_y = y1 - y0;
	GLdouble  dir_z = z1 - z0;
	GLdouble  bone_length = sqrt( dir_x*dir_x + dir_y*dir_y + dir_z*dir_z );

	// �`��p�����^�̐ݒ�
	static GLUquadricObj *  quad_obj = NULL;
	if ( quad_obj == NULL )
		quad_obj = gluNewQuadric();
	gluQuadricDrawStyle( quad_obj, GLU_FILL );
	gluQuadricNormals( quad_obj, GLU_SMOOTH );

	glPushMatrix();

	// ���s�ړ���ݒ�
	glTranslated( ( x0 + x1 ) * 0.5f, ( y0 + y1 ) * 0.5f, ( z0 + z1 ) * 0.5f );

	// �ȉ��A��]��\���s����v�Z

	// ������P�ʃx�N�g���ɐ��K��
	double  length;
	length = sqrt( dir_x*dir_x + dir_y*dir_y + dir_z*dir_z );
	if ( length < 0.0001 ) { 
		dir_x = 0.0; dir_y = 0.0; dir_z = 1.0;  length = 1.0;
	}
	dir_x /= length;  dir_y /= length;  dir_z /= length;

	// ��Ƃ��邙���̌�����ݒ�
	GLdouble  up_x, up_y, up_z;
	up_x = 0.0;
	up_y = 1.0;
	up_z = 0.0;

	// �����Ƃ����̊O�ς��炘���̌������v�Z
	double  side_x, side_y, side_z;
	side_x = up_y * dir_z - up_z * dir_y;
	side_y = up_z * dir_x - up_x * dir_z;
	side_z = up_x * dir_y - up_y * dir_x;

	// ������P�ʃx�N�g���ɐ��K��
	length = sqrt( side_x*side_x + side_y*side_y + side_z*side_z );
	if ( length < 0.0001 ) {
		side_x = 1.0; side_y = 0.0; side_z = 0.0;  length = 1.0;
	}
	side_x /= length;  side_y /= length;  side_z /= length;

	// �����Ƃ����̊O�ς��炙���̌������v�Z
	up_x = dir_y * side_z - dir_z * side_y;
	up_y = dir_z * side_x - dir_x * side_z;
	up_z = dir_x * side_y - dir_y * side_x;

	// ��]�s���ݒ�
	GLdouble  m[16] = { side_x, side_y, side_z, 0.0,
	                    up_x,   up_y,   up_z,   0.0,
	                    dir_x,  dir_y,  dir_z,  0.0,
	                    0.0,    0.0,    0.0,    1.0 };
	glMultMatrixd( m );

	// �~���̐ݒ�
	GLdouble slices = 16.0; // �~���̕��ˏ�̍ו����i�f�t�H���g12�j
	GLdouble stack = 16.0;  // �~���̗֐؂�̍ו����i�f�t�H���g�P�j

	// �ȉ~�̂�`��
	glScalef( radius, radius, bone_length * 0.5f );
	glEnable( GL_NORMALIZE );
	glutSolidSphere( 1.0f, slices, stack );
	glDisable( GL_NORMALIZE );

	glPopMatrix();
}


//
//  �p���̕`��i�X�e�B�b�N�t�B�M���A�ŕ`��j
//
void  DrawPosture( const Posture & posture )
{
	if ( !posture.body )
		return;

	// ���^���w�v�Z
	vector< Matrix4f >  seg_frame_array;
	vector< Point3f >  joi_pos_array;
	ForwardKinematics( posture, seg_frame_array, joi_pos_array );

	float  radius = 0.05f;
	Matrix4f  mat;
	Vector3f  v1, v2;

	// �e�̐߂̕`��
	for ( int i=0; i<seg_frame_array.size(); i++ )
	{
		const Segment *  segment = posture.body->segments[i];
		const int  num_joints = segment->joints.size();

		// �̐߂̒��S�̈ʒu�E��������Ƃ���ϊ��s���K�p
		glPushMatrix();
		mat.transpose( seg_frame_array[ i ] );
		glMultMatrixf( & mat.m00 );

		// �P�̊֐߂��疖�[�_�ւ̃o�[��`��
		if ( ( num_joints == 1 ) && segment->has_site )
		{
			v1 = segment->joint_positions[ 0 ];
			v2 = segment->site_position;
			DrawBone( v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, radius );
		}
		// �P�̊֐߂��牼�̖��[�_�i�d�S�ւ̃x�N�g�����Q�{�����ʒu�j�փo�[��`��
		else if ( ( num_joints == 1 ) && !segment->has_site )
		{
			v1 = segment->joint_positions[ 0 ];
			v2.negate( v1 );
			DrawBone( v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, radius );
		}
		// �Q�̊֐߂�ڑ�����o�[��`��
		else if ( num_joints == 2 )
		{
			v1 = segment->joint_positions[ 0 ];
			v2 = segment->joint_positions[ 1 ];
			DrawBone( v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, radius );
		}
		// �d�S����e�֐߂ւ̃o�[��`��
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
