/**
***  �L�����N�^�A�j���[�V�����̂��߂̐l�̃��f���̕\���E��{�����̃T���v���v���O����
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
**/


#ifdef  WIN32
	#include <windows.h>
#endif

#include <GL/glut.h>

#include "BVH.h"
#include "SimpleHuman.h"



//
//  �J�����EGLUT�̓��͏����Ɋւ���O���[�o���ϐ�
//

// �J�����̉�]�̂��߂̕ϐ�
float   camera_yaw = 0.0f;      // �x���𒆐S�Ƃ����]�p�x
float   camera_pitch = -20.0f;  // �w���𒆐S�Ƃ����]�p�x
float   camera_distance = 5.0f; // ���S����J�����̋���

// �}�E�X�̃h���b�O�̂��߂̕ϐ�
int     drag_mouse_r = 0; // �E�{�^�����h���b�O�����ǂ����̃t���O�i1:�h���b�O��, 0:��h���b�O���j
int     drag_mouse_l = 0; // ���{�^�����h���b�O�����ǂ����̃t���O�i1:�h���b�O��, 0:��h���b�O���j
int     drag_mouse_m = 0; // ���{�^�����h���b�O�����ǂ����̃t���O�i1:�h���b�O��, 0:��h���b�O���j
int     last_mouse_x, last_mouse_y; // �Ō�ɋL�^���ꂽ�}�E�X�J�[�\���̍��W

// �E�B���h�E�̃T�C�Y
int     win_width, win_height;


//
//  �A�j���[�V�����֘A�̃O���[�o���ϐ�
//

// �A�j���[�V���������ǂ�����\���t���O
bool    on_animation = true;

// �A�j���[�V�����̍Đ�����
float   animation_time = 0.0f;

// ���݂̕\���t���[���ԍ�
int     frame_no = 0;

// BVH����f�[�^
BVH *   bvh = NULL;


//
//  �L�����N�^�̎p���E���쏈���̃O���[�o���ϐ�
//

// �L�����N�^�̍��i�E�p��
Skeleton *  body = NULL;
Posture *   curr_posture = NULL;

// �֐ߓ_�̈ʒu�iIK�v�Z�p�j
vector< Point3f >  joint_world_positions;
vector< Point3f >  joint_screen_positions;

// �x�_�E���[�֐߁iIK�v�Z�p�j
int    base_joint_no = -1;
int    ee_joint_no = -1;

// �J�����p�����^�̍X�V�t���O
bool   view_updated = true;

// �f���̃��[�h��\���񋓌^�ϐ�
enum  DemoModeEnum
{
	MODE_MOTION,        // ����Đ��i�P�ꓮ��̍Đ��j
	MODE_IK,            // IK�v�Z�iCCD�@�ɂ��v�Z�j

	// �@�\��ǉ�����ꍇ�́A�K�X�A���[�h��ǉ�����
//	MODE_TRANSITION,    // ����J�ځE�ڑ��i�����̓���̊Ԃ�J�ځE�ڑ����čĐ��j
//	MODE_INTERPOLATION, // �����ԁi�����̓�����Ԃ��čĐ��j
//	MODE_IK_PSEUDO,     // IK�v�Z�i�^���t�s��ɂ��v�Z�j
//	MODE_IK_ANALYTICAL, // IK�v�Z�i�^���t�s��ɂ��v�Z�j

	NUM_MODES
};
DemoModeEnum  mode = MODE_MOTION;



///////////////////////////////////////////////////////////////////////////////
//
//  ����Đ��֘A�̏���
//


//
//  BVH����f�[�^�̓ǂݍ��݁A���i�E�p���̏�����
//
void  LoadBVH( const char * file_name )
{
	// ����f�[�^��ǂݍ���
	if ( bvh )
		delete  bvh;
	bvh = new BVH( file_name );

	// �ǂݍ��݂Ɏ��s������I��
	if ( !bvh->IsLoadSuccess() )
	{
		delete  bvh;
		bvh = NULL;
		body = NULL;
		return;
	}

	// BVH���삩�獜�i���f���𐶐�
	Skeleton *  new_body = CoustructBVHSkeleton( bvh );
	if ( !new_body )
	{
		return;
	}
	body = new_body;

	// �p���̏�����
	curr_posture = new Posture();
	InitPosture( *curr_posture, body );
}


//
//  ����Đ��J�n
//
void  StartAnimation()
{
	animation_time = 0.0f;
	frame_no = 0;
}


//
//  ����Đ��A�p���擾
//
void  UpdateAnimation( float delta_time )
{
	// ���Ԃ�i�߂�
	animation_time += delta_time;

	// ���݂̃t���[���ԍ����v�Z
	if ( bvh )
	{
		frame_no = animation_time / bvh->GetInterval();
		frame_no = frame_no % bvh->GetNumFrame();
	}
	else
		frame_no = 0;

	// BVH����̎p�����擾
	if ( bvh )
	{
		GetBVHPosture( bvh, frame_no, *curr_posture );
	}

	// �����Ԃ⓮��J�ځE�ڑ����s���ꍇ�́A�K�؂ȏ��������s����
	// ���|�[�g�ۑ�A�e������

}


//
//  �t�@�C���_�C�A���O��\������BVH�t�@�C����I���E�ǂݍ���
//
void  OpenNewBVH()
{
#ifdef  WIN32
	const int  file_name_len = 256;
	char  file_name[ file_name_len ] = "";

	// �t�@�C���_�C�A���O�̐ݒ�
	OPENFILENAME	open_file;
	memset( &open_file, 0, sizeof(OPENFILENAME) );
	open_file.lStructSize = sizeof(OPENFILENAME);
	open_file.hwndOwner = NULL;
	open_file.lpstrFilter = "BVH Motion Data (*.bvh)\0*.bvh\0All (*.*)\0*.*\0";
	open_file.nFilterIndex = 1;
	open_file.lpstrFile = file_name;
	open_file.nMaxFile = file_name_len;
	open_file.lpstrTitle = "Select a BVH file";
	open_file.lpstrDefExt = "bvh";
	open_file.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;

	// �t�@�C���_�C�A���O��\��
	BOOL  ret = GetOpenFileName( &open_file );

	// �t�@�C�����w�肳�ꂽ��V���������ݒ�
	if( ret )
	{

		// BVH����f�[�^�̓ǂݍ��݁A���i�E�p���̏�����
		LoadBVH( file_name );

		// ����Đ��̊J�n
		StartAnimation();
	}
#endif // WIN32
}



///////////////////////////////////////////////////////////////////////////////
//
//  Inverse Kinematics �v�Z�֘A�̏���
//


void  UpdateJointPositions( const Posture & posture );


//
//  Inverse Kinematics ���[�h�̊J�n
//
void  StartInverseKinematics()
{
	if ( !body || !curr_posture )
		return;

	// �p��������
	InitPosture( *curr_posture );

	// �֐ߓ_�̍X�V
	UpdateJointPositions( *curr_posture );
}


//
//  Inverse Kinematics �v�Z
//  ���o�͎p���A�x�_�֐ߔԍ��i-1�̏ꍇ�̓��[�g���x�_�Ƃ���j�A���[�֐ߔԍ��A���[�֐߂̖ڕW�ʒu���w��
//
void  ApplyInverseKinematics( Posture & posture, int base_joint_no, int ee_joint_no, Point3f ee_joint_position )
{
	// ���|�[�g�ۑ�A�e������

}


//
//  �֐ߓ_�̍X�V
//
void  UpdateJointPositions( const Posture & posture )
{
	// ���^���w�v�Z
	vector< Matrix4f >  seg_frame_array;
	ForwardKinematics( posture, seg_frame_array, joint_world_positions );

	// OpenGL �̕ϊ��s����擾
	double  model_view_matrix[ 16 ];
	double  projection_matrix[ 16 ];
	int  viewport_param[ 4 ];
	glGetDoublev( GL_MODELVIEW_MATRIX, model_view_matrix );
	glGetDoublev( GL_PROJECTION_MATRIX, projection_matrix );
	glGetIntegerv( GL_VIEWPORT, viewport_param );

	// ��ʏ�̊e�֐ߓ_�̈ʒu���v�Z
	int  num_joints = joint_world_positions.size();
	GLdouble  spx, spy, spz;
	joint_screen_positions.resize( num_joints );
	for ( int i=0; i<num_joints; i++ )
	{
		const Point3f &  wp = joint_world_positions[ i ];
		Point3f &  sp = joint_screen_positions[ i ];

		gluProject( wp.x, wp.y, wp.z,
			model_view_matrix, projection_matrix, viewport_param,
			&spx, &spy, &spz );
		sp.x = spx;
		sp.y = viewport_param[ 3 ] - spy;
	}
}


//
//  �֐ߓ_�̕`��
//
void   DrawJoint()
{
	// �f�v�X�e�X�g�𖳌��ɂ��āA�O�ʂɏ㏑������
	glDisable( GL_DEPTH_TEST );

	// �֐ߓ_��`��i����`��j
	for ( int i=0; i<joint_world_positions.size(); i++ )
	{
		// �x�_�֐߂͐Ԃŕ`��
		if ( i == base_joint_no )
			glColor3f( 1.0f, 0.0f, 0.0f );
		// ���[�֐߂͗΂ŕ`��
		else if ( i == ee_joint_no )
			glColor3f( 0.0f, 1.0f, 0.0f );
		// ���̊֐߂͐ŕ`��
		else
			glColor3f( 0.0f, 0.0f, 1.0f );

		// �֐߈ʒu�ɋ���`��
		const Point3f &  pos = joint_world_positions[ i ];
		glPushMatrix();
			glTranslatef( pos.x, pos.y, pos.z );
			glutSolidSphere( 0.025f, 16, 16 );
		glPopMatrix();
	}

	// �x�_�֐߂��w�肳��Ă��Ȃ��ꍇ�́A���[�g�̐߂��x�_�Ƃ���i���[�g�̐߂̈ʒu�ɋ���`��j
	if ( base_joint_no == -1 )
	{
		// ���[�g�̐߈ʒu�ɋ���`��
		glColor3f( 1.0f, 0.0f, 0.0f );
		const Point3f &  pos = curr_posture->root_pos;
		glPushMatrix();
			glTranslatef( pos.x, pos.y, pos.z );
			glutSolidSphere( 0.025f, 16, 16 );
		glPopMatrix();
	}

	glEnable( GL_DEPTH_TEST );
}


//
//  �֐ߓ_�̑I��
//
void   SelectJoint( int mouse_x, int mouse_y, bool ee_or_base )
{
	const float  distance_threthold = 20.0f;
	float  distance, min_distance = -1.0f;
	int  closesed_joint_no = -1;
	float  dx, dy;

	// ���͍��W�ƍł��߂��ʒu�ɂ���֐߂�T��
	for ( int i=0; i<joint_screen_positions.size(); i++ )
	{
		dx = joint_screen_positions[ i ].x - mouse_x;
		dy = joint_screen_positions[ i ].y - mouse_y;
		distance = sqrt( dx * dx + dy * dy );

		if ( ( i == 0 ) || ( distance < min_distance ) )
		{
			min_distance = distance;
			closesed_joint_no = i;
		}
	}

	// ������臒l�ȉ��ł���ΑI��
	if ( ee_or_base )
	{
		if ( min_distance < distance_threthold )
			ee_joint_no = closesed_joint_no;
		else
			ee_joint_no = -1;
	}
	else
	{
		if ( min_distance < distance_threthold )
			base_joint_no = closesed_joint_no;
		else
			base_joint_no = -1;
	}
}


//
//  �֐ߓ_�̈ړ��i�����ɐ����ȕ��ʏ�ŏ㉺���E�Ɉړ�����j
//
void   MoveJoint( int mouse_dx, int mouse_dy )
{
	// ���[�֐߂��I������Ă��Ȃ���ΏI��
	if ( ee_joint_no == -1 )
		return;

	// ��ʏ�̈ړ��ʂƂR������Ԃł̈ړ��ʂ̔䗦
	const float  mouse_pos_scale = 0.01f;

	// OpenGL �̕ϊ��s����擾
	double  model_view_matrix[ 16 ];
	glGetDoublev( GL_MODELVIEW_MATRIX, model_view_matrix );

	Vector3f  vec;
	Point3f &  pos = joint_world_positions[ ee_joint_no ];

	// �J�������W�n��X�������Ɉړ�
	vec.set( model_view_matrix[ 0 ], model_view_matrix[ 4 ], model_view_matrix[ 8 ] );
	pos.scaleAdd( mouse_dx * mouse_pos_scale, vec, pos );

	// �J�������W�n��X�������Ɉړ�
	vec.set( model_view_matrix[ 1 ], model_view_matrix[ 5 ], model_view_matrix[ 9 ] );
	pos.scaleAdd( - mouse_dy * mouse_pos_scale, vec, pos );

	// Inverse Kinematics �v�Z��K�p
	ApplyInverseKinematics( *curr_posture, base_joint_no, ee_joint_no, pos );

	// �֐ߓ_�̍X�V
//	UpdateJointPositions( *curr_posture );
}



///////////////////////////////////////////////////////////////////////////////
//
//  �A�v���P�[�V�����̃C�x���g����
//


//
//  �e�L�X�g��`��
//
void  drawMessage( int line_no, const char * message )
{
	int   i;
	if ( message == NULL )
		return;

	// �ˉe�s����������i�������̑O�Ɍ��݂̍s���ޔ��j
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D( 0.0, win_width, win_height, 0.0 );

	// ���f���r���[�s����������i�������̑O�Ɍ��݂̍s���ޔ��j
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();

	// �y�o�b�t�@�E���C�e�B���O�̓I�t�ɂ���
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );

	// ���b�Z�[�W�̕`��
	glColor3f( 1.0, 0.0, 0.0 );
	glRasterPos2i( 8, 24 + 18 * line_no );
	for ( i=0; message[i]!='\0'; i++ )
		glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18, message[i] );

	// �ݒ��S�ĕ���
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
	glMatrixMode( GL_PROJECTION );
	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );
	glPopMatrix();
}


//
//  �E�B���h�E�ĕ`�掞�ɌĂ΂��R�[���o�b�N�֐�
//
void  display( void )
{
	// ��ʂ��N���A
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );

	// �ϊ��s���ݒ�i���f�����W�n���J�������W�n�j
    glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	glTranslatef( 0.0, 0.0, - camera_distance );
	glRotatef( - camera_pitch, 1.0, 0.0, 0.0 );
	glRotatef( - camera_yaw, 0.0, 1.0, 0.0 );
	glTranslatef( 0.0, -0.5, 0.0 );

	// �����ʒu���Đݒ�
	float  light0_position[] = { 10.0, 10.0, 10.0, 1.0 };
	glLightfv( GL_LIGHT0, GL_POSITION, light0_position );

	// �n�ʂ�`��
	float  size = 1.5f;
	int  num_x = 10, num_z = 10;
	double  ox, oz;
	glBegin( GL_QUADS );
		glNormal3d( 0.0, 1.0, 0.0 );
		ox = -(num_x * size) / 2;
		for ( int x=0; x<num_x; x++, ox+=size )
		{
			oz = -(num_z * size) / 2;
			for ( int z=0; z<num_z; z++, oz+=size )
			{
				if ( ((x + z) % 2) == 0 )
					glColor3f( 1.0, 1.0, 1.0 );
				else
					glColor3f( 0.8, 0.8, 0.8 );
				glVertex3d( ox, 0.0, oz );
				glVertex3d( ox, 0.0, oz+size );
				glVertex3d( ox+size, 0.0, oz+size );
				glVertex3d( ox+size, 0.0, oz );
			}
		}
	glEnd();

	// �L�����N�^��`��
	if ( curr_posture )
	{
		glColor3f( 1.0f, 1.0f, 1.0f );
		DrawPosture( *curr_posture );
	}

	// ���_���X�V���ꂽ��֐ߓ_�̈ʒu���X�V
	if ( curr_posture && view_updated && ( mode == MODE_IK ) )
	{
		UpdateJointPositions( *curr_posture );
		view_updated = false;
	}

	// �֐ߓ_��`��
	if ( curr_posture && ( mode == MODE_IK ) )
	{
		DrawJoint();
	}

	// ���݂̃��[�h�A���ԁE�t���[���ԍ���\��
	char  message[ 64 ];
	if ( mode == MODE_MOTION )
	{
		if ( bvh )
			sprintf( message, "%.2f (%d)", animation_time, frame_no );
		else
			sprintf( message, "Press 'L' key to Load a BVH file" );
		drawMessage( 0, message );
	}
	else
	if ( mode == MODE_IK )
	{
		drawMessage( 0, "IK Mode" );
	}

	// �o�b�N�o�b�t�@�ɕ`�悵����ʂ��t�����g�o�b�t�@�ɕ\��
    glutSwapBuffers();
}


//
//  �E�B���h�E�T�C�Y�ύX���ɌĂ΂��R�[���o�b�N�֐�
//
void  reshape( int w, int h )
{
	// �E�B���h�E���̕`����s���͈͂�ݒ�i�����ł̓E�B���h�E�S�̂ɕ`��j
    glViewport(0, 0, w, h);
	
	// �J�������W�n���X�N���[�����W�n�ւ̕ϊ��s���ݒ�
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective( 45, (double)w/h, 1, 500 );

	// �E�B���h�E�̃T�C�Y���L�^�i�e�L�X�g�`�揈���̂��߁j
	win_width = w;
	win_height = h;
}


//
// �}�E�X�N���b�N���ɌĂ΂��R�[���o�b�N�֐�
//
void  mouse( int button, int state, int mx, int my )
{
	// ���{�^���������ꂽ��h���b�O�J�n
	if ( ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_DOWN ) )
		drag_mouse_l = 1;
	// ���{�^���������ꂽ��h���b�O�I��
	else if ( ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_UP ) )
		drag_mouse_l = 0;

	// �E�{�^���������ꂽ��h���b�O�J�n
	if ( ( button == GLUT_RIGHT_BUTTON ) && ( state == GLUT_DOWN ) )
		drag_mouse_r = 1;
	// �E�{�^���������ꂽ��h���b�O�I��
	else if ( ( button == GLUT_RIGHT_BUTTON ) && ( state == GLUT_UP ) )
		drag_mouse_r = 0;

	// ���{�^���������ꂽ��h���b�O�J�n
	if ( ( button == GLUT_MIDDLE_BUTTON ) && ( state == GLUT_DOWN ) )
		drag_mouse_m = 1;
	// ���{�^���������ꂽ��h���b�O�I��
	else if ( ( button == GLUT_MIDDLE_BUTTON ) && ( state == GLUT_UP ) )
		drag_mouse_m = 0;

	// ���{�^���������ꂽ��AIK�̎x�_�E���[�֐߂�I��
	if ( ( mode == MODE_IK ) && ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_DOWN ) )
	{
		// Shift�L�[��������Ă���΁A�x�_�֐߂�I��
		if ( glutGetModifiers() & GLUT_ACTIVE_SHIFT )
			SelectJoint( mx, my, false );
		// Shift�L�[��������Ă��Ȃ���΁A���[�֐߂�I��
		else
			SelectJoint( mx, my, true );
	}

	// �ĕ`��
	glutPostRedisplay();

	// ���݂̃}�E�X���W���L�^
	last_mouse_x = mx;
	last_mouse_y = my;
}


//
// �}�E�X�h���b�O���ɌĂ΂��R�[���o�b�N�֐�
//
void  motion( int mx, int my )
{
	// ���{�^���̃h���b�O���́AIK�̖��[�֐߂̖ڕW�ʒu�𑀍�
	if ( drag_mouse_l )
	{
		MoveJoint( mx - last_mouse_x, my - last_mouse_y );
	}

	// �E�{�^���̃h���b�O���͎��_����]����
	if ( drag_mouse_r )
	{
		// �O��̃}�E�X���W�ƍ���̃}�E�X���W�̍��ɉ����Ď��_����]

		// �}�E�X�̉��ړ��ɉ����Ăx���𒆐S�ɉ�]
		camera_yaw -= ( mx - last_mouse_x ) * 1.0;
		if ( camera_yaw < 0.0 )
			camera_yaw += 360.0;
		else if ( camera_yaw > 360.0 )
			camera_yaw -= 360.0;

		// �}�E�X�̏c�ړ��ɉ����Ăw���𒆐S�ɉ�]
		camera_pitch -= ( my - last_mouse_y ) * 1.0;
		if ( camera_pitch < -90.0 )
			camera_pitch = -90.0;
		else if ( camera_pitch > 90.0 )
			camera_pitch = 90.0;

		// ���_�̍X�V�t���O��ݒ�
		view_updated = true;
	}
	// ���{�^���̃h���b�O���͎��_�ƃJ�����̋�����ύX����
	if ( drag_mouse_m )
	{
		// �O��̃}�E�X���W�ƍ���̃}�E�X���W�̍��ɉ����Ď��_����]

		// �}�E�X�̏c�ړ��ɉ����ċ������ړ�
		camera_distance += ( my - last_mouse_y ) * 0.2;
		if ( camera_distance < 2.0 )
			camera_distance = 2.0;

		// ���_�̍X�V�t���O��ݒ�
		view_updated = true;
	}

	// ����̃}�E�X���W���L�^
	last_mouse_x = mx;
	last_mouse_y = my;

	// �ĕ`��
	glutPostRedisplay();
}


//
//  �L�[�{�[�h�̃L�[�������ꂽ�Ƃ��ɌĂ΂��R�[���o�b�N�֐�
//
void  keyboard( unsigned char key, int mx, int my )
{
	// m �L�[�Ń��[�h�̐؂�ւ�
	if ( key == 'm' )
	{
		mode = (DemoModeEnum)( ( mode + 1 ) % NUM_MODES );

		// �J�n����
		if ( mode == MODE_MOTION )
			StartAnimation();
		if ( mode == MODE_IK )
			StartInverseKinematics();
	}

	// s �L�[�ŃA�j���[�V�����̒�~�E�ĊJ
	if ( key == 's' )
		on_animation = !on_animation;
	// n �L�[�Ŏ��̃t���[��
	if ( ( key == 'n' ) && !on_animation )
	{
		animation_time += bvh->GetInterval();
		frame_no ++;
		frame_no = frame_no % bvh->GetNumFrame();
	}
	// p �L�[�őO�̃t���[��
	if ( ( key == 'p' ) && !on_animation && ( frame_no > 0 ) && bvh )
	{
		animation_time -= bvh->GetInterval();
		frame_no --;
		frame_no = frame_no % bvh->GetNumFrame();
	}
	// r �L�[�ŃA�j���[�V�����̃��Z�b�g
	if ( key == 'r' )
	{
		StartAnimation();
	}

	// l �L�[�ōĐ�����̕ύX
	if ( key == 'l' )
	{
		// �t�@�C���_�C�A���O��\������BVH�t�@�C����I���E�ǂݍ���
		OpenNewBVH();
	}

	glutPostRedisplay();
}


//
//  �A�C�h�����ɌĂ΂��R�[���o�b�N�֐�
//
void  idle( void )
{
	// �A�j���[�V��������
	if ( ( mode == MODE_MOTION ) && on_animation )
	{
#ifdef  WIN32
		// �V�X�e�����Ԃ��擾���A�O�񂩂�̌o�ߎ��Ԃɉ����ă���������
		static DWORD  last_time = 0;
		DWORD  curr_time = timeGetTime();
		float  delta = ( curr_time - last_time ) * 0.001f;
		if ( delta > 0.03f )
			delta = 0.03f;
		last_time = curr_time;
#else
		// �Œ�̃������g�p
		float  delta = 0.03f;
#endif

		// ����Đ��A�p���擾
		UpdateAnimation( delta );

		// �ĕ`��̎w�����o���i���̌�ōĕ`��̃R�[���o�b�N�֐����Ă΂��j
		glutPostRedisplay();
	}
}


//
//  ���������֐�
//
void  initEnvironment( void )
{
	// �������쐬����
	float  light0_position[] = { 10.0, 10.0, 10.0, 1.0 };
	float  light0_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
	float  light0_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	float  light0_ambient[] = { 0.1, 0.1, 0.1, 1.0 };
	glLightfv( GL_LIGHT0, GL_POSITION, light0_position );
	glLightfv( GL_LIGHT0, GL_DIFFUSE, light0_diffuse );
	glLightfv( GL_LIGHT0, GL_SPECULAR, light0_specular );
	glLightfv( GL_LIGHT0, GL_AMBIENT, light0_ambient );
	glEnable( GL_LIGHT0 );

	// �����v�Z��L���ɂ���
	glEnable( GL_LIGHTING );

	// ���̂̐F����L���ɂ���
	glEnable( GL_COLOR_MATERIAL );

	// �y�e�X�g��L���ɂ���
	glEnable( GL_DEPTH_TEST );

	// �w�ʏ�����L���ɂ���
	glCullFace( GL_BACK );
	glEnable( GL_CULL_FACE );

	// �w�i�F��ݒ�
	glClearColor( 0.5, 0.5, 0.8, 0.0 );

	// �T���v��BVH����f�[�^��ǂݍ���
	LoadBVH( "sample_walking1.bvh" );
}


//
//  ���C���֐��i�v���O�����͂�������J�n�j
//
int  main( int argc, char ** argv )
{
	// GLUT�̏�����
    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_STENCIL );
	glutInitWindowSize( 640, 640 );
	glutInitWindowPosition( 0, 0 );
    glutCreateWindow("Simple Human Sample");
	
	// �R�[���o�b�N�֐��̓o�^
    glutDisplayFunc( display );
    glutReshapeFunc( reshape );
	glutMouseFunc( mouse );
	glutMotionFunc( motion );
	glutKeyboardFunc( keyboard );
	glutIdleFunc( idle );

	// ��������
	initEnvironment();

	// GLUT�̃��C�����[�v�ɏ������ڂ�
    glutMainLoop();
    return 0;
}

