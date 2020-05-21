#include "LBEmulator.h"

void InitPs(unsigned int camId, double depthResolution, Matrix4d extrinsic, LadybugData lbd);
void LBEmulator::RenderPinhole(Matrix4d camPara, unsigned int cidx) {

	PIXELFORMATDESCRIPTOR _pfd = {
sizeof(PIXELFORMATDESCRIPTOR),	//	Size of this struct
1,	//	Versin of this structure
PFD_DRAW_TO_BITMAP | PFD_SUPPORT_OPENGL |
PFD_GENERIC_ACCELERATED,
//		PFD_GENERIC_FORMAT,
		//	Pixel buffer flags
		PFD_TYPE_RGBA,	//	Type of pixel data
		24,	//	The number of color bitplanes
		0, 0, 0, 0, 0, 0,	//	Number of each color bitplanes and shift count
		0, 0,	//	Number of alpha bitplanes and shift count
		0, 0, 0, 0, 0,	//	Number of accumulation bits
		32,	//	Z depth
		0,	//	Stencil depth
		0,	//	Number of auxiliary buffers
		PFD_MAIN_PLANE,	//	Ignored
		0,	//	Reserved
		0,	//	Ignored
		0,	//	Transparent color value
		0,	//	Ignored
	};
	GLint view[4];
	HDC		_hdc_ = CreateCompatibleDC(NULL);
	viewHeight = 1232;//viewSize;
	DWORD m_DIBWidth = 1616;
	DWORD m_DIBHeight = 1232;
	DWORD m_BPP = 24;

	// Create BITMAPINFOHEADER
	BITMAPINFOHEADER* m_PBIH = new BITMAPINFOHEADER;
	int iSize = sizeof(BITMAPINFOHEADER);
	::memset(m_PBIH, 0, iSize);

	m_PBIH->biSize = sizeof(BITMAPINFOHEADER);
	m_PBIH->biWidth = m_DIBWidth;
	m_PBIH->biHeight = m_DIBHeight;
	m_PBIH->biPlanes = 1;
	m_PBIH->biBitCount = m_BPP;
	m_PBIH->biCompression = BI_RGB;

	// Create DIB
	void* m_PBits;
	HBITMAP m_hbitmap_old;
	HBITMAP m_hbitmap = ::CreateDIBSection(
		_hdc_,
		(BITMAPINFO*)m_PBIH, DIB_RGB_COLORS,
		&m_PBits, NULL, 0
	);

	m_hbitmap_old = (HBITMAP)::SelectObject(_hdc_, m_hbitmap);
	DWORD dwLength;
	if ((m_DIBWidth * 3) % 4 == 0) /* バッファの１ラインの長さを計算 */
		dwLength = m_DIBWidth * 3;
	else
		dwLength = m_DIBWidth * 3 + (4 - (m_DIBWidth * 3) % 4);
	//	LPBYTE lpPixel,lpBuf;
	//	lpBuf=(LPBYTE)GlobalAlloc(GPTR,sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER)+dwLength*m_DIBHeight);
	//	lpPixel=lpBuf+sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER);

	int		_pfid = ChoosePixelFormat(_hdc_, &_pfd);
	::SetPixelFormat(_hdc_, _pfid, &_pfd);
	HGLRC	_hrc = ::wglCreateContext(_hdc_);
	::wglMakeCurrent(_hdc_, _hrc);
	InitPs(0,depthResolution,camPara,lbdata[cidx]);
	glGetIntegerv(GL_VIEWPORT, view);
	for (int i = 0;i < dataNum;i++) {
		//float* trsVert = (float*)malloc(sizeof(float)*vtNumArray[i] * 3);
		float* vertexp = vertexPointers[i];
		glBegin(GL_TRIANGLES);
		for (int j = 0;j < meshNumArray[i];j++) {
			int index1 = facePointers[i][j * 3];
			int index2 = facePointers[i][j * 3 + 1];
			int index3 = facePointers[i][j * 3 + 2];
			//gr=reflectance[index1];
			glColor3ub(rgbaPointers[i][index1 * 4], rgbaPointers[i][index1 * 4 + 1], rgbaPointers[i][index1 * 4 + 2]);
			glVertex3f(vertexp[index1 * 3], vertexp[index1 * 3 + 1], vertexp[index1 * 3 + 2]);
			glColor3ub(rgbaPointers[i][index2 * 4], rgbaPointers[i][index2 * 4 + 1], rgbaPointers[i][index2 * 4 + 2]);
			glVertex3f(vertexp[index2 * 3], vertexp[index2 * 3 + 1], vertexp[index2 * 3 + 2]);
			glColor3ub(rgbaPointers[i][index3 * 4], rgbaPointers[i][index3 * 4 + 1], rgbaPointers[i][index3 * 4 + 2]);
			glVertex3f(vertexp[index3 * 3], vertexp[index3 * 3 + 1], vertexp[index3 * 3 + 2]);
		}
		glEnd();
		//displayrgb(trsVert, facePointers[i], rgbaPointers[i], meshNumArray[i], vtNumArray[i]);
		//free(trsVert);
	}

	//	BitBlt(dhdc,0,0,m_DIBWidth,m_DIBHeight,_hdc_,0,0,SRCCOPY);
	//	SelectObject(dhdc,m_hbitmap_old);
	//	GetDIBits(_hdc_,m_hbitmap,0,m_DIBHeight,lpPixel,(LPBITMAPINFO)m_PBIH,DIB_RGB_COLORS);
	colorImage = (GLubyte*)malloc(sizeof(GLubyte)*view[2] * view[3] * 3);
	glReadPixels(view[0], view[1], view[2], view[3], GL_RGB, GL_UNSIGNED_BYTE, colorImage);
	depthArray = (GLfloat*)malloc(sizeof(GLfloat)*view[2] * view[3]);
	glReadPixels(view[0], view[1], view[2], view[3], GL_DEPTH_COMPONENT, GL_FLOAT, depthArray);
	normArray = (GLfloat*)malloc(sizeof(GLfloat)*view[2] * view[3]);

	double th = M_PI / viewHeight;
	Vector3d zdrc;
	int viewWidth = 1616;
	for (int x = 0;x < viewWidth;x++) {
		for (int y = 0;y < viewHeight;y++) {
			int idx = y * viewWidth + x;
			if (y == 0 || y == viewHeight - 1) { normArray[idx] = 0;continue; }
			int prexidx = x != 0 ? y * viewWidth + x - 1 : y * viewWidth + viewWidth - 1;
			int postxidx = x != viewWidth - 1 ? y * viewWidth + x + 1 : y * viewWidth;
			int preyidx = idx - viewWidth;
			int postyidx = idx + viewWidth;
			if (depthArray[idx] == 1.0 || depthArray[prexidx] == 1.0 || depthArray[postxidx] == 1.0 || depthArray[preyidx] == 1.0 || depthArray[postyidx] == 1.0) {
				normArray[idx] = 0;
				continue;
			}

			/*double t=M_PI*(depthArray[postxidx]+depthArray[prexidx])*125/viewHeight;//125= viewOrtho far/4

			double d=(depthArray[postxidx]-depthArray[prexidx])*500;
			double t2=M_PI*(depthArray[postyidx]+depthArray[preyidx])*125/viewHeight;//125= viewOrtho far/4
			double d2=(depthArray[postyidx]-depthArray[preyidx])*500;*/
			Vector3d v1, v2;
			v1 << (depthArray[prexidx] + depthArray[postxidx])*sin(th) * 500, 0, (depthArray[prexidx] - depthArray[postxidx])*cos(th) * 500;
			v2 << 0, (depthArray[preyidx] + depthArray[postyidx])*sin(th) * 500, (depthArray[preyidx] - depthArray[postyidx])*cos(th) * 500;
			Vector3d n;
			n = v1.cross(v2);
			if (n(2) < 0)n = -n;
			n.normalize();
			normArray[idx] = n(2);//max...2.0
		}
	}
	//	GetObject(m_hbitmap,
	::wglMakeCurrent(0, 0);
	//	GlobalFree(lpBuf);
	//	GlobalFree(lpBuf);
	//	free(buffer);
	// return 0;


}

void LBEmulator::RenderPinhole(Matrix4d camPara) {

	PIXELFORMATDESCRIPTOR _pfd = {
sizeof(PIXELFORMATDESCRIPTOR),	//	Size of this struct
1,	//	Versin of this structure
PFD_DRAW_TO_BITMAP | PFD_SUPPORT_OPENGL |
PFD_GENERIC_ACCELERATED,
//		PFD_GENERIC_FORMAT,
		//	Pixel buffer flags
		PFD_TYPE_RGBA,	//	Type of pixel data
		24,	//	The number of color bitplanes
		0, 0, 0, 0, 0, 0,	//	Number of each color bitplanes and shift count
		0, 0,	//	Number of alpha bitplanes and shift count
		0, 0, 0, 0, 0,	//	Number of accumulation bits
		32,	//	Z depth
		0,	//	Stencil depth
		0,	//	Number of auxiliary buffers
		PFD_MAIN_PLANE,	//	Ignored
		0,	//	Reserved
		0,	//	Ignored
		0,	//	Transparent color value
		0,	//	Ignored
	};
	GLint view[4];
	HDC		_hdc_ = CreateCompatibleDC(NULL);
	viewHeight = 1232;//viewSize;
	DWORD m_DIBWidth = 1616;
	DWORD m_DIBHeight = 1232;
	DWORD m_BPP = 24;

	// Create BITMAPINFOHEADER
	BITMAPINFOHEADER* m_PBIH = new BITMAPINFOHEADER;
	int iSize = sizeof(BITMAPINFOHEADER);
	::memset(m_PBIH, 0, iSize);

	m_PBIH->biSize = sizeof(BITMAPINFOHEADER);
	m_PBIH->biWidth = m_DIBWidth;
	m_PBIH->biHeight = m_DIBHeight;
	m_PBIH->biPlanes = 1;
	m_PBIH->biBitCount = m_BPP;
	m_PBIH->biCompression = BI_RGB;

	// Create DIB
	void* m_PBits;
	HBITMAP m_hbitmap_old;
	HBITMAP m_hbitmap = ::CreateDIBSection(
		_hdc_,
		(BITMAPINFO*)m_PBIH, DIB_RGB_COLORS,
		&m_PBits, NULL, 0
	);

	m_hbitmap_old = (HBITMAP)::SelectObject(_hdc_, m_hbitmap);
	DWORD dwLength;
	if ((m_DIBWidth * 3) % 4 == 0) /* バッファの１ラインの長さを計算 */
		dwLength = m_DIBWidth * 3;
	else
		dwLength = m_DIBWidth * 3 + (4 - (m_DIBWidth * 3) % 4);
	//	LPBYTE lpPixel,lpBuf;
	//	lpBuf=(LPBYTE)GlobalAlloc(GPTR,sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER)+dwLength*m_DIBHeight);
	//	lpPixel=lpBuf+sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER);

	int		_pfid = ChoosePixelFormat(_hdc_, &_pfd);
	::SetPixelFormat(_hdc_, _pfid, &_pfd);
	HGLRC	_hrc = ::wglCreateContext(_hdc_);
	::wglMakeCurrent(_hdc_, _hrc);

	colorImage = (GLubyte*)malloc(sizeof(GLubyte)*1232 * 1616 * 3 * 6);
	depthArray = (GLfloat*)malloc(sizeof(GLfloat)* 1232 * 1616 * 6);
	for (int cidx = 0;cidx < 6;cidx++) {
		InitPs(0, depthResolution, camPara, lbdata[cidx]);
		glGetIntegerv(GL_VIEWPORT, view);
		const GLfloat lightPos[] = { 0 , 0 , 0 , 1.0 };
		const GLfloat lightCol[] = { 1 , 0 , 0 , 1.0 };
	//	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
		//glLightfv(GL_LIGHT0, GL_DIFFUSE, lightCol);
		//glEnable(GL_LIGHTING);
		//glEnable(GL_LIGHT0);

		for (int i = 0;i < dataNum;i++) {
			//float* trsVert = (float*)malloc(sizeof(float)*vtNumArray[i] * 3);
			float* vertexp = vertexPointers[i];
			glBegin(GL_TRIANGLES);
			for (int j = 0;j < meshNumArray[i];j++) {
				int index1 = facePointers[i][j * 3];
				int index2 = facePointers[i][j * 3 + 1];
				int index3 = facePointers[i][j * 3 + 2];
				//gr=reflectance[index1];
//				glNormal3f(norm[j * 3], norm[j * 3+1], norm[j * 3+2]);
				glColor3ub(rgbaPointers[i][index1 * 4], rgbaPointers[i][index1 * 4 + 1], rgbaPointers[i][index1 * 4 + 2]);
				glVertex3f(vertexp[index1 * 3], vertexp[index1 * 3 + 1], vertexp[index1 * 3 + 2]);
				glColor3ub(rgbaPointers[i][index2 * 4], rgbaPointers[i][index2 * 4 + 1], rgbaPointers[i][index2 * 4 + 2]);
				glVertex3f(vertexp[index2 * 3], vertexp[index2 * 3 + 1], vertexp[index2 * 3 + 2]);
				glColor3ub(rgbaPointers[i][index3 * 4], rgbaPointers[i][index3 * 4 + 1], rgbaPointers[i][index3 * 4 + 2]);
				glVertex3f(vertexp[index3 * 3], vertexp[index3 * 3 + 1], vertexp[index3 * 3 + 2]);
			}
			glEnd();
			//displayrgb(trsVert, facePointers[i], rgbaPointers[i], meshNumArray[i], vtNumArray[i]);
			//free(trsVert);
		}

		//	BitBlt(dhdc,0,0,m_DIBWidth,m_DIBHeight,_hdc_,0,0,SRCCOPY);
		//	SelectObject(dhdc,m_hbitmap_old);
		//	GetDIBits(_hdc_,m_hbitmap,0,m_DIBHeight,lpPixel,(LPBITMAPINFO)m_PBIH,DIB_RGB_COLORS);
		glReadPixels(view[0], view[1], view[2], view[3], GL_RGB, GL_UNSIGNED_BYTE, colorImage + view[2] * view[3] * 3 * cidx);
		glReadPixels(view[0], view[1], view[2], view[3], GL_DEPTH_COMPONENT, GL_FLOAT, depthArray + view[2]*view[3]*cidx );
	}
	//normArray = (GLfloat*)malloc(sizeof(GLfloat)*view[2] * view[3]);

	//double th = M_PI / viewHeight;
	//Vector3d zdrc;
	//int viewWidth = 1616;
	//for (int x = 0;x < viewWidth;x++) {
	//	for (int y = 0;y < viewHeight;y++) {
	//		int idx = y * viewWidth + x;
	//		if (y == 0 || y == viewHeight - 1) { normArray[idx] = 0;continue; }
	//		int prexidx = x != 0 ? y * viewWidth + x - 1 : y * viewWidth + viewWidth - 1;
	//		int postxidx = x != viewWidth - 1 ? y * viewWidth + x + 1 : y * viewWidth;
	//		int preyidx = idx - viewWidth;
	//		int postyidx = idx + viewWidth;
	//		if (depthArray[idx] == 1.0 || depthArray[prexidx] == 1.0 || depthArray[postxidx] == 1.0 || depthArray[preyidx] == 1.0 || depthArray[postyidx] == 1.0) {
	//			normArray[idx] = 0;
	//			continue;
	//		}

	//		/*double t=M_PI*(depthArray[postxidx]+depthArray[prexidx])*125/viewHeight;//125= viewOrtho far/4

	//		double d=(depthArray[postxidx]-depthArray[prexidx])*500;
	//		double t2=M_PI*(depthArray[postyidx]+depthArray[preyidx])*125/viewHeight;//125= viewOrtho far/4
	//		double d2=(depthArray[postyidx]-depthArray[preyidx])*500;*/
	//		Vector3d v1, v2;
	//		v1 << (depthArray[prexidx] + depthArray[postxidx])*sin(th) * 500, 0, (depthArray[prexidx] - depthArray[postxidx])*cos(th) * 500;
	//		v2 << 0, (depthArray[preyidx] + depthArray[postyidx])*sin(th) * 500, (depthArray[preyidx] - depthArray[postyidx])*cos(th) * 500;
	//		Vector3d n;
	//		n = v1.cross(v2);
	//		if (n(2) < 0)n = -n;
	//		n.normalize();
	//		normArray[idx] = n(2);//max...2.0
	//	}
	//}
	//	GetObject(m_hbitmap,
	::wglMakeCurrent(0, 0);
	//	GlobalFree(lpBuf);
	//	GlobalFree(lpBuf);
	//	free(buffer);
	// return 0;


}




void InitPs(unsigned int camId, double depthResolution, Matrix4d extrinsic, LadybugData lbd) {
	glViewport(0, 0, 1616, 1232);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
//	gluPerspective(90.0, (double)300 / (double)400, 0.0, 100.0); //透視投影法の視体積gluPerspactive(th, w/h, near, far);

	glClearColor(1.0, 1.0, 1.0, 1.0);
	glEnable(GL_DEPTH_TEST);
//	gluPerspective(90.0, (double)300/(double)400, 0.0, 100.0); //透視投影法の視体積gluPerspactive(th, w/h, near, far);

	GLfloat m[16];
	glGetFloatv(GL_PROJECTION_MATRIX, m);
	printf("m[0]:% 7.5f m[4]:% 7.5f m[8] :% 7.5f m[12]:% 7.5f\n", m[0], m[4], m[8], m[12]);
	printf("m[1]:% 7.5f m[5]:% 7.5f m[9] :% 7.5f m[13]:% 7.5f\n", m[1], m[5], m[9], m[13]);
	printf("m[2]:% 7.5f m[6]:% 7.5f m[10]:% 7.5f m[14]:% 7.5f\n", m[2], m[6], m[10], m[14]);
	printf("m[3]:% 7.5f m[7]:% 7.5f m[11]:% 7.5f m[16]:% 7.5f\n", m[3], m[7], m[11], m[15]);

	//glLoadIdentity();



	//glOrtho(-PI_VAL, PI_VAL, -PI_VAL, 0, 0, depthResolution * 256);
	//------------------------------------------------
	//gluLookAt(
	//	0.0, 0.0, 0.0, // 視点の位置x,y,z;
	//	1.0, 0.0, 0.0,   // 視界の中心位置の参照点座標x,y,z
	//	0.0, 0.0, 1.0);  //視界の上方向のベクトルx,y,z*/
	//glGetFloatv(GL_PROJECTION_MATRIX, m);
	//printf("m[0]:% 7.5f m[4]:% 7.5f m[8] :% 7.5f m[12]:% 7.5f\n", m[0], m[4], m[8], m[12]);
	//printf("m[1]:% 7.5f m[5]:% 7.5f m[9] :% 7.5f m[13]:% 7.5f\n", m[1], m[5], m[9], m[13]);
	//printf("m[2]:% 7.5f m[6]:% 7.5f m[10]:% 7.5f m[14]:% 7.5f\n", m[2], m[6], m[10], m[14]);
	//printf("m[3]:% 7.5f m[7]:% 7.5f m[11]:% 7.5f m[16]:% 7.5f\n", m[3], m[7], m[11], m[15]);
	double f, cx, cy, w, h;
	w = 1616;
	h = 1232;
	f = lbd.f;
	cx = lbd.cx;
	cy = lbd.cy;

	double znear=100.00;
	double zfar = 0.003;

	Matrix4d m1_,rev,r2l;//projection
	m1_ << 
		2 * f / w	,0			, (w - 2 * cx) / w	, 0,
		0			,- 2 * f / h	, (h - 2 * cy) / h	, 0,
		0			, 0			, (-zfar-znear)/(zfar - znear)				, -2*zfar*znear/(zfar - znear),
		0			,0			, -1				, 0;





	//r2l << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1;

	Matrix4d m2_;//extrinsic

	double cRx =cos(lbd.rotation[0]);
	double cRy = cos(lbd.rotation[1]);
	double cRz = cos(lbd.rotation[2]);
	double sRx = sin(lbd.rotation[0]);
	double sRy = sin(lbd.rotation[1]);
	double sRz = sin(lbd.rotation[2]);
	m2_ <<
		cRz * cRy, cRz*sRy*sRx - sRz * cRx, cRz*sRy*cRx + sRz * sRx, lbd.translation[0],
		sRz*cRy, sRz*sRy*sRx + cRz * cRx, sRz*sRy*cRx - cRz * sRx, lbd.translation[1],
		-sRy, cRy*sRx, cRy*cRx, lbd.translation[2],
		0, 0, 0, 1;

	r2l <<
		1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, -1, 0
		, 0, 0, 0, 1;
	rev <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, -1, 0
		, 0, 0, 0, 1;
	Matrix4d m3 = rev * m1_*r2l* m2_.inverse()*extrinsic;



	//GLdouble m1[16] = {
	//m1_(0,0),m1_(0,1),m1_(0,2),m1_(0,3),
	//	m1_(1,0),m1_(1,1),m1_(1,2),m1_(1,3),
	//		m1_(2,0),m1_(2,1),m1_(2,2),m1_(2,3),
	//			m1_(3,0),m1_(3,1),m1_(3,2),m1_(3,3),
	//};
	GLdouble m2[16];
	//m3=m3.transpose();
	memcpy(m2, m3.data(), sizeof(double) * 16);

	glMultMatrixd(m2);
//	glMultMatrixd(m2);

	//glGetFloatv(GL_PROJECTION_MATRIX, m);
	//printf("m[0]:% 7.5f m[4]:% 7.5f m[8] :% 7.5f m[12]:% 7.5f\n", m[0], m[4], m[8], m[12]);
	//printf("m[1]:% 7.5f m[5]:% 7.5f m[9] :% 7.5f m[13]:% 7.5f\n", m[1], m[5], m[9], m[13]);
	//printf("m[2]:% 7.5f m[6]:% 7.5f m[10]:% 7.5f m[14]:% 7.5f\n", m[2], m[6], m[10], m[14]);
	//printf("m[3]:% 7.5f m[7]:% 7.5f m[11]:% 7.5f m[16]:% 7.5f\n", m[3], m[7], m[11], m[15]);


	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//	GLfloat light_position[]
	//	glLigntfv(GL_LIGHT0,GL_POSITION,);

}

POINT_3D getNorm_(POINT_3D p1, POINT_3D p2, POINT_3D p3) {
	double v1x = p2.x - p1.x, v1y = p2.y - p1.y, v1z = p2.z - p1.z,
		v2x = p3.x - p2.x, v2y = p3.y - p2.y, v2z = p3.z - p2.z;
	Vector3d nc_;
	POINT_3D nc;
	nc_(0) = v1y * v2z - v1z * v2y;
	nc_(1) = v1z * v2x - v1x * v2z;
	nc_(2) = v1x * v2y - v1y * v2x;
	nc_.normalize();
	nc.x = nc_(0);
	nc.y = nc_(1);
	nc.z = nc_(2);
	return nc;
};

void LBEmulator::meshNormCompute() {
	if (norm != NULL) {
		free(norm);
	}
	int i_ = 0;
	norm = (float*)malloc(sizeof(float)* meshNumArray[i_] * 3);
	memset(norm, 0, sizeof(float)* meshNumArray[i_] * 3);
	float* vec = vertexPointers[i_];
	unsigned int* faces = facePointers[i_];
	for (int i = 0;i < meshNumArray[i];i++) {


		POINT_3D p1, p2, p3;
		p1.x = vec[faces[i * 3] * 3];			p1.y = vec[faces[i * 3] * 3 + 1];			p1.z = vec[faces[i * 3] * 3 + 2];
		p2.x = vec[faces[i * 3 + 1] * 3];			p2.y = vec[faces[i * 3 + 1] * 3 + 1];			p2.z = vec[faces[i * 3 + 1] * 3 + 2];
		p3.x = vec[faces[i * 3 + 2] * 3];			p3.y = vec[faces[i * 3 + 2] * 3 + 1];			p3.z = vec[faces[i * 3 + 2] * 3 + 2];

		POINT_3D nc = getNorm_(p1, p2, p3);
		norm[i * 3] = nc.x;norm[i * 3 + 1] = nc.y;norm[i * 3 + 2] = nc.z;
	}

}