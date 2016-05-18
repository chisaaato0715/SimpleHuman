/**
***  �L�����N�^�A�j���[�V�����̂��߂̐l�̃��f���̕\���E��{����
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
**/


#ifndef  _SIMPLE_HUMAN_H_
#define  _SIMPLE_HUMAN_H_


//
//  �s��E�x�N�g���̕\���ɂ� vecmath C++���C�u�����ihttp://objectclub.jp/download/vecmath1�j���g�p
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
//  ���֐ߑ̂̑̐߂�\���\����
//
struct  Segment
{
	// �̐ߔԍ��E���O
	int                  index;
	string               name;

    // �ڑ��֐�
    vector< Joint * >    joints;

    // �e�֐߂̐ڑ��ʒu�i�̐߂̃��[�J�����W�n�j
    vector< Point3f >    joint_positions;

	// �̐߂̖��[�ʒu
	bool                 has_site;
	Point3f              site_position;
};


//
//  ���֐ߑ̂̊֐߂�\���\����
//
struct  Joint
{
	// �֐ߔԍ��E���O
	int                  index;
	string               name;

    // �ڑ��̐�
    Segment *            segments[ 2 ];
};


//
//  ���֐ߑ̂̍��i��\���\����
//
struct  Skeleton
{
    // �̐߁E�֐߂̔z��
    vector< Segment * >  segments;
    vector< Joint * >    joints;
};


//
//  ���֐ߑ̂̎p����\���\����
//
struct  Posture
{
    Skeleton *           body;
    Point3f              root_pos;        // ���[�g�̈ʒu
    Matrix3f             root_ori;        // ���[�g�̌����i��]�s��\���j
    vector< Matrix3f >   joint_rotations; // �e�֐߂̑��Ή�]�i��]�s��\���j[�֐ߔԍ�]
};


//
//  ���֐ߑ̂̍��i�E�p���E����̊�{����
//

// BVH���삩�獜�i���f���𐶐�
Skeleton *  CoustructBVHSkeleton( class BVH * bvh );

// �p���̏�����
void  InitPosture( Posture & posture, Skeleton * body = NULL );

// BVH���삩��p�����擾
void  GetBVHPosture( const class BVH * bvh, int frame_no, Posture & posture );

// ���^���w�v�Z
void  ForwardKinematics( const Posture & posture, vector< Matrix4f > & seg_frame_array, vector< Point3f > & joi_pos_array );

// ���^���w�v�Z
void  ForwardKinematics( const Posture & posture, vector< Matrix4f > & seg_frame_array );

// �p���̕`��i�X�e�B�b�N�t�B�M���A�ŕ`��j
void  DrawPosture( const Posture & posture );



#endif // _SIMPLE_HUMAN_H_
