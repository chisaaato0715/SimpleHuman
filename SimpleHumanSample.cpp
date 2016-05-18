/**
***  キャラクタアニメーションのための人体モデルの表現・基本処理のサンプルプログラム
***  Copyright (c) 2015-, Masaki OSHITA (www.oshita-lab.org)
**/


#ifdef  WIN32
	#include <windows.h>
#endif

#include <GL/glut.h>

#include "BVH.h"
#include "SimpleHuman.h"



//
//  カメラ・GLUTの入力処理に関するグローバル変数
//

// カメラの回転のための変数
float   camera_yaw = 0.0f;      // Ｙ軸を中心とする回転角度
float   camera_pitch = -20.0f;  // Ｘ軸を中心とする回転角度
float   camera_distance = 5.0f; // 中心からカメラの距離

// マウスのドラッグのための変数
int     drag_mouse_r = 0; // 右ボタンがドラッグ中かどうかのフラグ（1:ドラッグ中, 0:非ドラッグ中）
int     drag_mouse_l = 0; // 左ボタンがドラッグ中かどうかのフラグ（1:ドラッグ中, 0:非ドラッグ中）
int     drag_mouse_m = 0; // 中ボタンがドラッグ中かどうかのフラグ（1:ドラッグ中, 0:非ドラッグ中）
int     last_mouse_x, last_mouse_y; // 最後に記録されたマウスカーソルの座標

// ウィンドウのサイズ
int     win_width, win_height;


//
//  アニメーション関連のグローバル変数
//

// アニメーション中かどうかを表すフラグ
bool    on_animation = true;

// アニメーションの再生時間
float   animation_time = 0.0f;

// 現在の表示フレーム番号
int     frame_no = 0;

// BVH動作データ
BVH *   bvh = NULL;


//
//  キャラクタの姿勢・動作処理のグローバル変数
//

// キャラクタの骨格・姿勢
Skeleton *  body = NULL;
Posture *   curr_posture = NULL;

// 関節点の位置（IK計算用）
vector< Point3f >  joint_world_positions;
vector< Point3f >  joint_screen_positions;

// 支点・末端関節（IK計算用）
int    base_joint_no = -1;
int    ee_joint_no = -1;

// カメラパラメタの更新フラグ
bool   view_updated = true;

// デモのモードを表す列挙型変数
enum  DemoModeEnum
{
	MODE_MOTION,        // 動作再生（単一動作の再生）
	MODE_IK,            // IK計算（CCD法による計算）

	// 機能を追加する場合は、適宜、モードを追加する
//	MODE_TRANSITION,    // 動作遷移・接続（複数の動作の間を遷移・接続して再生）
//	MODE_INTERPOLATION, // 動作補間（複数の動作を補間して再生）
//	MODE_IK_PSEUDO,     // IK計算（疑似逆行列による計算）
//	MODE_IK_ANALYTICAL, // IK計算（疑似逆行列による計算）

	NUM_MODES
};
DemoModeEnum  mode = MODE_MOTION;



///////////////////////////////////////////////////////////////////////////////
//
//  動作再生関連の処理
//


//
//  BVH動作データの読み込み、骨格・姿勢の初期化
//
void  LoadBVH( const char * file_name )
{
	// 動作データを読み込み
	if ( bvh )
		delete  bvh;
	bvh = new BVH( file_name );

	// 読み込みに失敗したら終了
	if ( !bvh->IsLoadSuccess() )
	{
		delete  bvh;
		bvh = NULL;
		body = NULL;
		return;
	}

	// BVH動作から骨格モデルを生成
	Skeleton *  new_body = CoustructBVHSkeleton( bvh );
	if ( !new_body )
	{
		return;
	}
	body = new_body;

	// 姿勢の初期化
	curr_posture = new Posture();
	InitPosture( *curr_posture, body );
}


//
//  動作再生開始
//
void  StartAnimation()
{
	animation_time = 0.0f;
	frame_no = 0;
}


//
//  動作再生、姿勢取得
//
void  UpdateAnimation( float delta_time )
{
	// 時間を進める
	animation_time += delta_time;

	// 現在のフレーム番号を計算
	if ( bvh )
	{
		frame_no = animation_time / bvh->GetInterval();
		frame_no = frame_no % bvh->GetNumFrame();
	}
	else
		frame_no = 0;

	// BVH動作の姿勢を取得
	if ( bvh )
	{
		GetBVHPosture( bvh, frame_no, *curr_posture );
	}

	// 動作補間や動作遷移・接続を行う場合は、適切な処理を実行する
	// レポート課題、各自実装

}


//
//  ファイルダイアログを表示してBVHファイルを選択・読み込み
//
void  OpenNewBVH()
{
#ifdef  WIN32
	const int  file_name_len = 256;
	char  file_name[ file_name_len ] = "";

	// ファイルダイアログの設定
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

	// ファイルダイアログを表示
	BOOL  ret = GetOpenFileName( &open_file );

	// ファイルが指定されたら新しい動作を設定
	if( ret )
	{

		// BVH動作データの読み込み、骨格・姿勢の初期化
		LoadBVH( file_name );

		// 動作再生の開始
		StartAnimation();
	}
#endif // WIN32
}



///////////////////////////////////////////////////////////////////////////////
//
//  Inverse Kinematics 計算関連の処理
//


void  UpdateJointPositions( const Posture & posture );


//
//  Inverse Kinematics モードの開始
//
void  StartInverseKinematics()
{
	if ( !body || !curr_posture )
		return;

	// 姿勢初期化
	InitPosture( *curr_posture );

	// 関節点の更新
	UpdateJointPositions( *curr_posture );
}


//
//  Inverse Kinematics 計算
//  入出力姿勢、支点関節番号（-1の場合はルートを支点とする）、末端関節番号、末端関節の目標位置を指定
//
void  ApplyInverseKinematics( Posture & posture, int base_joint_no, int ee_joint_no, Point3f ee_joint_position )
{
	// レポート課題、各自実装

}


//
//  関節点の更新
//
void  UpdateJointPositions( const Posture & posture )
{
	// 順運動学計算
	vector< Matrix4f >  seg_frame_array;
	ForwardKinematics( posture, seg_frame_array, joint_world_positions );

	// OpenGL の変換行列を取得
	double  model_view_matrix[ 16 ];
	double  projection_matrix[ 16 ];
	int  viewport_param[ 4 ];
	glGetDoublev( GL_MODELVIEW_MATRIX, model_view_matrix );
	glGetDoublev( GL_PROJECTION_MATRIX, projection_matrix );
	glGetIntegerv( GL_VIEWPORT, viewport_param );

	// 画面上の各関節点の位置を計算
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
//  関節点の描画
//
void   DrawJoint()
{
	// デプステストを無効にして、前面に上書きする
	glDisable( GL_DEPTH_TEST );

	// 関節点を描画（球を描画）
	for ( int i=0; i<joint_world_positions.size(); i++ )
	{
		// 支点関節は赤で描画
		if ( i == base_joint_no )
			glColor3f( 1.0f, 0.0f, 0.0f );
		// 末端関節は緑で描画
		else if ( i == ee_joint_no )
			glColor3f( 0.0f, 1.0f, 0.0f );
		// 他の関節は青で描画
		else
			glColor3f( 0.0f, 0.0f, 1.0f );

		// 関節位置に球を描画
		const Point3f &  pos = joint_world_positions[ i ];
		glPushMatrix();
			glTranslatef( pos.x, pos.y, pos.z );
			glutSolidSphere( 0.025f, 16, 16 );
		glPopMatrix();
	}

	// 支点関節が指定されていない場合は、ルート体節を支点とする（ルート体節の位置に球を描画）
	if ( base_joint_no == -1 )
	{
		// ルート体節位置に球を描画
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
//  関節点の選択
//
void   SelectJoint( int mouse_x, int mouse_y, bool ee_or_base )
{
	const float  distance_threthold = 20.0f;
	float  distance, min_distance = -1.0f;
	int  closesed_joint_no = -1;
	float  dx, dy;

	// 入力座標と最も近い位置にある関節を探索
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

	// 距離が閾値以下であれば選択
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
//  関節点の移動（視線に垂直な平面上で上下左右に移動する）
//
void   MoveJoint( int mouse_dx, int mouse_dy )
{
	// 末端関節が選択されていなければ終了
	if ( ee_joint_no == -1 )
		return;

	// 画面上の移動量と３次元空間での移動量の比率
	const float  mouse_pos_scale = 0.01f;

	// OpenGL の変換行列を取得
	double  model_view_matrix[ 16 ];
	glGetDoublev( GL_MODELVIEW_MATRIX, model_view_matrix );

	Vector3f  vec;
	Point3f &  pos = joint_world_positions[ ee_joint_no ];

	// カメラ座標系のX軸方向に移動
	vec.set( model_view_matrix[ 0 ], model_view_matrix[ 4 ], model_view_matrix[ 8 ] );
	pos.scaleAdd( mouse_dx * mouse_pos_scale, vec, pos );

	// カメラ座標系のX軸方向に移動
	vec.set( model_view_matrix[ 1 ], model_view_matrix[ 5 ], model_view_matrix[ 9 ] );
	pos.scaleAdd( - mouse_dy * mouse_pos_scale, vec, pos );

	// Inverse Kinematics 計算を適用
	ApplyInverseKinematics( *curr_posture, base_joint_no, ee_joint_no, pos );

	// 関節点の更新
//	UpdateJointPositions( *curr_posture );
}



///////////////////////////////////////////////////////////////////////////////
//
//  アプリケーションのイベント処理
//


//
//  テキストを描画
//
void  drawMessage( int line_no, const char * message )
{
	int   i;
	if ( message == NULL )
		return;

	// 射影行列を初期化（初期化の前に現在の行列を退避）
	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D( 0.0, win_width, win_height, 0.0 );

	// モデルビュー行列を初期化（初期化の前に現在の行列を退避）
	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();

	// Ｚバッファ・ライティングはオフにする
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );

	// メッセージの描画
	glColor3f( 1.0, 0.0, 0.0 );
	glRasterPos2i( 8, 24 + 18 * line_no );
	for ( i=0; message[i]!='\0'; i++ )
		glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18, message[i] );

	// 設定を全て復元
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
	glMatrixMode( GL_PROJECTION );
	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );
	glPopMatrix();
}


//
//  ウィンドウ再描画時に呼ばれるコールバック関数
//
void  display( void )
{
	// 画面をクリア
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT );

	// 変換行列を設定（モデル座標系→カメラ座標系）
    glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	glTranslatef( 0.0, 0.0, - camera_distance );
	glRotatef( - camera_pitch, 1.0, 0.0, 0.0 );
	glRotatef( - camera_yaw, 0.0, 1.0, 0.0 );
	glTranslatef( 0.0, -0.5, 0.0 );

	// 光源位置を再設定
	float  light0_position[] = { 10.0, 10.0, 10.0, 1.0 };
	glLightfv( GL_LIGHT0, GL_POSITION, light0_position );

	// 地面を描画
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

	// キャラクタを描画
	if ( curr_posture )
	{
		glColor3f( 1.0f, 1.0f, 1.0f );
		DrawPosture( *curr_posture );
	}

	// 視点が更新されたら関節点の位置を更新
	if ( curr_posture && view_updated && ( mode == MODE_IK ) )
	{
		UpdateJointPositions( *curr_posture );
		view_updated = false;
	}

	// 関節点を描画
	if ( curr_posture && ( mode == MODE_IK ) )
	{
		DrawJoint();
	}

	// 現在のモード、時間・フレーム番号を表示
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

	// バックバッファに描画した画面をフロントバッファに表示
    glutSwapBuffers();
}


//
//  ウィンドウサイズ変更時に呼ばれるコールバック関数
//
void  reshape( int w, int h )
{
	// ウィンドウ内の描画を行う範囲を設定（ここではウィンドウ全体に描画）
    glViewport(0, 0, w, h);
	
	// カメラ座標系→スクリーン座標系への変換行列を設定
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective( 45, (double)w/h, 1, 500 );

	// ウィンドウのサイズを記録（テキスト描画処理のため）
	win_width = w;
	win_height = h;
}


//
// マウスクリック時に呼ばれるコールバック関数
//
void  mouse( int button, int state, int mx, int my )
{
	// 左ボタンが押されたらドラッグ開始
	if ( ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_DOWN ) )
		drag_mouse_l = 1;
	// 左ボタンが離されたらドラッグ終了
	else if ( ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_UP ) )
		drag_mouse_l = 0;

	// 右ボタンが押されたらドラッグ開始
	if ( ( button == GLUT_RIGHT_BUTTON ) && ( state == GLUT_DOWN ) )
		drag_mouse_r = 1;
	// 右ボタンが離されたらドラッグ終了
	else if ( ( button == GLUT_RIGHT_BUTTON ) && ( state == GLUT_UP ) )
		drag_mouse_r = 0;

	// 中ボタンが押されたらドラッグ開始
	if ( ( button == GLUT_MIDDLE_BUTTON ) && ( state == GLUT_DOWN ) )
		drag_mouse_m = 1;
	// 中ボタンが離されたらドラッグ終了
	else if ( ( button == GLUT_MIDDLE_BUTTON ) && ( state == GLUT_UP ) )
		drag_mouse_m = 0;

	// 左ボタンが押されたら、IKの支点・末端関節を選択
	if ( ( mode == MODE_IK ) && ( button == GLUT_LEFT_BUTTON ) && ( state == GLUT_DOWN ) )
	{
		// Shiftキーが押されていれば、支点関節を選択
		if ( glutGetModifiers() & GLUT_ACTIVE_SHIFT )
			SelectJoint( mx, my, false );
		// Shiftキーが押されていなければ、末端関節を選択
		else
			SelectJoint( mx, my, true );
	}

	// 再描画
	glutPostRedisplay();

	// 現在のマウス座標を記録
	last_mouse_x = mx;
	last_mouse_y = my;
}


//
// マウスドラッグ時に呼ばれるコールバック関数
//
void  motion( int mx, int my )
{
	// 左ボタンのドラッグ中は、IKの末端関節の目標位置を操作
	if ( drag_mouse_l )
	{
		MoveJoint( mx - last_mouse_x, my - last_mouse_y );
	}

	// 右ボタンのドラッグ中は視点を回転する
	if ( drag_mouse_r )
	{
		// 前回のマウス座標と今回のマウス座標の差に応じて視点を回転

		// マウスの横移動に応じてＹ軸を中心に回転
		camera_yaw -= ( mx - last_mouse_x ) * 1.0;
		if ( camera_yaw < 0.0 )
			camera_yaw += 360.0;
		else if ( camera_yaw > 360.0 )
			camera_yaw -= 360.0;

		// マウスの縦移動に応じてＸ軸を中心に回転
		camera_pitch -= ( my - last_mouse_y ) * 1.0;
		if ( camera_pitch < -90.0 )
			camera_pitch = -90.0;
		else if ( camera_pitch > 90.0 )
			camera_pitch = 90.0;

		// 視点の更新フラグを設定
		view_updated = true;
	}
	// 中ボタンのドラッグ中は視点とカメラの距離を変更する
	if ( drag_mouse_m )
	{
		// 前回のマウス座標と今回のマウス座標の差に応じて視点を回転

		// マウスの縦移動に応じて距離を移動
		camera_distance += ( my - last_mouse_y ) * 0.2;
		if ( camera_distance < 2.0 )
			camera_distance = 2.0;

		// 視点の更新フラグを設定
		view_updated = true;
	}

	// 今回のマウス座標を記録
	last_mouse_x = mx;
	last_mouse_y = my;

	// 再描画
	glutPostRedisplay();
}


//
//  キーボードのキーが押されたときに呼ばれるコールバック関数
//
void  keyboard( unsigned char key, int mx, int my )
{
	// m キーでモードの切り替え
	if ( key == 'm' )
	{
		mode = (DemoModeEnum)( ( mode + 1 ) % NUM_MODES );

		// 開始処理
		if ( mode == MODE_MOTION )
			StartAnimation();
		if ( mode == MODE_IK )
			StartInverseKinematics();
	}

	// s キーでアニメーションの停止・再開
	if ( key == 's' )
		on_animation = !on_animation;
	// n キーで次のフレーム
	if ( ( key == 'n' ) && !on_animation )
	{
		animation_time += bvh->GetInterval();
		frame_no ++;
		frame_no = frame_no % bvh->GetNumFrame();
	}
	// p キーで前のフレーム
	if ( ( key == 'p' ) && !on_animation && ( frame_no > 0 ) && bvh )
	{
		animation_time -= bvh->GetInterval();
		frame_no --;
		frame_no = frame_no % bvh->GetNumFrame();
	}
	// r キーでアニメーションのリセット
	if ( key == 'r' )
	{
		StartAnimation();
	}

	// l キーで再生動作の変更
	if ( key == 'l' )
	{
		// ファイルダイアログを表示してBVHファイルを選択・読み込み
		OpenNewBVH();
	}

	glutPostRedisplay();
}


//
//  アイドル時に呼ばれるコールバック関数
//
void  idle( void )
{
	// アニメーション処理
	if ( ( mode == MODE_MOTION ) && on_animation )
	{
#ifdef  WIN32
		// システム時間を取得し、前回からの経過時間に応じてΔｔを決定
		static DWORD  last_time = 0;
		DWORD  curr_time = timeGetTime();
		float  delta = ( curr_time - last_time ) * 0.001f;
		if ( delta > 0.03f )
			delta = 0.03f;
		last_time = curr_time;
#else
		// 固定のΔｔを使用
		float  delta = 0.03f;
#endif

		// 動作再生、姿勢取得
		UpdateAnimation( delta );

		// 再描画の指示を出す（この後で再描画のコールバック関数が呼ばれる）
		glutPostRedisplay();
	}
}


//
//  環境初期化関数
//
void  initEnvironment( void )
{
	// 光源を作成する
	float  light0_position[] = { 10.0, 10.0, 10.0, 1.0 };
	float  light0_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
	float  light0_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	float  light0_ambient[] = { 0.1, 0.1, 0.1, 1.0 };
	glLightfv( GL_LIGHT0, GL_POSITION, light0_position );
	glLightfv( GL_LIGHT0, GL_DIFFUSE, light0_diffuse );
	glLightfv( GL_LIGHT0, GL_SPECULAR, light0_specular );
	glLightfv( GL_LIGHT0, GL_AMBIENT, light0_ambient );
	glEnable( GL_LIGHT0 );

	// 光源計算を有効にする
	glEnable( GL_LIGHTING );

	// 物体の色情報を有効にする
	glEnable( GL_COLOR_MATERIAL );

	// Ｚテストを有効にする
	glEnable( GL_DEPTH_TEST );

	// 背面除去を有効にする
	glCullFace( GL_BACK );
	glEnable( GL_CULL_FACE );

	// 背景色を設定
	glClearColor( 0.5, 0.5, 0.8, 0.0 );

	// サンプルBVH動作データを読み込み
	LoadBVH( "sample_walking1.bvh" );
}


//
//  メイン関数（プログラムはここから開始）
//
int  main( int argc, char ** argv )
{
	// GLUTの初期化
    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_STENCIL );
	glutInitWindowSize( 640, 640 );
	glutInitWindowPosition( 0, 0 );
    glutCreateWindow("Simple Human Sample");
	
	// コールバック関数の登録
    glutDisplayFunc( display );
    glutReshapeFunc( reshape );
	glutMouseFunc( mouse );
	glutMotionFunc( motion );
	glutKeyboardFunc( keyboard );
	glutIdleFunc( idle );

	// 環境初期化
	initEnvironment();

	// GLUTのメインループに処理を移す
    glutMainLoop();
    return 0;
}

