#pragma once
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#ifdef CMATH3D_ASSERTS
#include <assert.h>
#endif
#ifndef M_PI_F
#define M_PI_F   (3.14159265358979323846f)
#define M_1_PI_F (0.31830988618379067154f)
#define M_PI_2_F (1.57079632679f)
#endif
static inline float fsqr(float x) { return x * x; }
static inline float radians(float degrees) { return (M_PI_F / 180.0f) * degrees; }
static inline float degrees(float radians) { return (180.0f / M_PI_F) * radians; }
static inline float clamp(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}
static inline bool fcloseulps(float a, float b, int ulps) {
	if ((a < 0.0f) != (b < 0.0f)) {
		if (a == b) {
			return true;
		}
		return false;
	}
	int ia = *((int *)&a);
	int ib = *((int *)&b);
	return fabsf(ia - ib) <= ulps;
}
struct vec {
	float x; float y; float z;
};
static inline struct vec mkvec(float x, float y, float z) {
	struct vec v;
	v.x = x; v.y = y; v.z = z;
	return v;
}
static inline struct vec vrepeat(float x) {
	return mkvec(x, x, x);
}
static inline struct vec vzero(void) {
	return vrepeat(0.0f);
}
static inline struct vec vbasis(int i) {
	float a[3] = {0.0f, 0.0f, 0.0f};
	a[i] = 1.0f;
	return mkvec(a[0], a[1], a[2]);
}
static inline struct vec vscl(float s, struct vec v) {
	return mkvec(s * v.x , s * v.y, s * v.z);
}
static inline struct vec vneg(struct vec v) {
	return mkvec(-v.x, -v.y, -v.z);
}
static inline struct vec vdiv(struct vec v, float s) {
	return vscl(1.0f/s, v);
}
static inline struct vec vadd(struct vec a, struct vec b) {
	return mkvec(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline struct vec vsub(struct vec a, struct vec b) {
	return vadd(a, vneg(b));
}
static inline float vdot(struct vec a, struct vec b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline struct vec veltmul(struct vec a, struct vec b) {
	return mkvec(a.x * b.x, a.y * b.y, a.z * b.z);
}
static inline struct vec veltdiv(struct vec a, struct vec b) {
	return mkvec(a.x / b.x, a.y / b.y, a.z / b.z);
}
static inline struct vec veltrecip(struct vec a) {
	return mkvec(1.0f / a.x, 1.0f / a.y, 1.0f / a.z);
}
static inline float vmag2(struct vec v) {
	return vdot(v, v);
}
static inline float vmag(struct vec v) {
	return sqrtf(vmag2(v));
}
static inline float vdist2(struct vec a, struct vec b) {
  return vmag2(vsub(a, b));
}
static inline float vdist(struct vec a, struct vec b) {
  return sqrtf(vdist2(a, b));
}
static inline struct vec vnormalize(struct vec v) {
	return vdiv(v, vmag(v));
}
static inline struct vec vclampnorm(struct vec v, float maxnorm) {
	float const norm = vmag(v);
	if (norm > maxnorm) {
		return vscl(maxnorm / norm, v);
	}
	return v;
}
static inline struct vec vcross(struct vec a, struct vec b) {
	return mkvec(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
static inline struct vec vprojectunit(struct vec a, struct vec b_unit) {
	return vscl(vdot(a, b_unit), b_unit);
}
static inline struct vec vorthunit(struct vec a, struct vec b_unit) {
	return vsub(a, vprojectunit(a, b_unit));
}
static inline struct vec vabs(struct vec v) {
	return mkvec(fabsf(v.x), fabsf(v.y), fabsf(v.z));
}
static inline struct vec vmin(struct vec a, struct vec b) {
	return mkvec(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
}
static inline struct vec vmax(struct vec a, struct vec b) {
	return mkvec(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
}
static inline struct vec vclamp(struct vec v, struct vec lower, struct vec upper) {
	return vmin(upper, vmax(v, lower));
}
static inline struct vec vclampabs(struct vec v, struct vec abs_upper) {
	return vclamp(v, vneg(abs_upper), abs_upper);
}
static inline float vmaxelt(struct vec v) {
	return fmax(fmax(v.x, v.y), v.z);
}
static inline float vminelt(struct vec v) {
	return fmin(fmin(v.x, v.y), v.z);
}
static inline float vnorm1(struct vec v) {
	return fabsf(v.x) + fabsf(v.y) + fabsf(v.z);
}
static inline bool veq(struct vec a, struct vec b) {
	return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
}
static inline bool vneq(struct vec a, struct vec b) {
	return !veq(a, b);
}
static inline bool veqepsilon(struct vec a, struct vec b, float epsilon) {
	struct vec diffs = vabs(vsub(a, b));
	return diffs.x < epsilon && diffs.y < epsilon && diffs.z < epsilon;
}
static inline bool vless(struct vec a, struct vec b) {
	return (a.x < b.x) && (a.y < b.y) && (a.z < b.z);
}
static inline bool vleq(struct vec a, struct vec b) {
	return (a.x <= b.x) && (a.y <= b.y) && (a.z <= b.z);
}
static inline bool vgreater(struct vec a, struct vec b) {
	return (a.x > b.x) && (a.y > b.y) && (a.z > b.z);
}
static inline bool vgeq(struct vec a, struct vec b) {
	return (a.x >= b.x) && (a.y >= b.y) && (a.z >= b.z);
}
static inline bool visnan(struct vec v) {
	return isnan(v.x) || isnan(v.y) || isnan(v.z);
}
static inline struct vec vadd3(struct vec a, struct vec b, struct vec c) {
	return vadd(vadd(a, b), c);
}
static inline struct vec vadd4(struct vec a, struct vec b, struct vec c, struct vec d) {
	return vadd(vadd(a, b), vadd(c, d));
}
static inline struct vec vsub2(struct vec a, struct vec b, struct vec c) {
	return vadd3(a, vneg(b), vneg(c));
}
static inline struct vec vload(double const *d) {
	return mkvec(d[0], d[1], d[2]);
}
static inline void vstore(struct vec v, double *d) {
	d[0] = (double)v.x; d[1] = (double)v.y; d[2] = (double)v.z;
}
static inline struct vec vloadf(float const *f) {
	return mkvec(f[0], f[1], f[2]);
}
static inline void vstoref(struct vec v, float *f) {
	f[0] = v.x; f[1] = v.y; f[2] = v.z;
}
static inline float vindex(struct vec v, int i) {
	return ((float const *)&v.x)[i];
}
struct mat33 {
	float m[3][3];
};
static inline struct mat33 mzero(void) {
	struct mat33 m;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			m.m[i][j] = 0;
		}
	}
	return m;
}
static inline struct mat33 mdiag(float a, float b, float c) {
	struct mat33 m = mzero();
	m.m[0][0] = a;
	m.m[1][1] = b;
	m.m[2][2] = c;
	return m;
}
static inline struct mat33 meyescl(float a) {
	return mdiag(a, a, a);
}
static inline struct mat33 meye(void) {
	return meyescl(1.0f);
}
static inline struct mat33 mcolumns(struct vec a, struct vec b, struct vec c) {
	struct mat33 m;
	m.m[0][0] = a.x;
	m.m[1][0] = a.y;
	m.m[2][0] = a.z;
	m.m[0][1] = b.x;
	m.m[1][1] = b.y;
	m.m[2][1] = b.z;
	m.m[0][2] = c.x;
	m.m[1][2] = c.y;
	m.m[2][2] = c.z;
	return m;
}
static inline struct mat33 mrows(struct vec a, struct vec b, struct vec c) {
	struct mat33 m;
	vstoref(a, m.m[0]);
	vstoref(b, m.m[1]);
	vstoref(c, m.m[2]);
	return m;
}
static inline struct mat33 mcrossmat(struct vec v) {
	struct mat33 m;
	m.m[0][0] = 0;
	m.m[0][1] = -v.z;
	m.m[0][2] = v.y;
	m.m[1][0] = v.z;
	m.m[1][1] = 0;
	m.m[1][2] = -v.x;
	m.m[2][0] = -v.y;
	m.m[2][1] = v.x;
	m.m[2][2] = 0;
	return m;
}
static inline struct vec mcolumn(struct mat33 m, int col) {
	return mkvec(m.m[0][col], m.m[1][col], m.m[2][col]);
}
static inline struct vec mrow(struct mat33 m, int row) {
	return vloadf(m.m[row]);
}
static inline struct mat33 mtranspose(struct mat33 m) {
	struct mat33 mt;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			mt.m[i][j] = m.m[j][i];
		}
	}
	return mt;
}
static inline struct mat33 mscl(float s, struct mat33 a) {
	struct mat33 sa;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			sa.m[i][j] = s * a.m[i][j];
		}
	}
	return sa;
}
static inline struct mat33 mneg(struct mat33 a) {
	return mscl(-1.0, a);
}
static inline struct mat33 madd(struct mat33 a, struct mat33 b) {
	struct mat33 c;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			c.m[i][j] = a.m[i][j] + b.m[i][j];
		}
	}
	return c;
}
static inline struct mat33 msub(struct mat33 a, struct mat33 b) {
	struct mat33 c;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			c.m[i][j] = a.m[i][j] - b.m[i][j];
		}
	}
	return c;
}
static inline struct vec mvmul(struct mat33 a, struct vec v) {
	float x = a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z;
	float y = a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z;
	float z = a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z;
	return mkvec(x, y, z);
}
static inline struct mat33 mmul(struct mat33 a, struct mat33 b) {
	struct mat33 ab;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			float accum = 0;
			for (int k = 0; k < 3; ++k) {
				accum += a.m[i][k] * b.m[k][j];
			}
			ab.m[i][j] = accum;
		}
	}
	return ab;
}
static inline struct mat33 maddridge(struct mat33 a, float d) {
	a.m[0][0] += d;
	a.m[1][1] += d;
	a.m[2][2] += d;
	return a;
}
static inline bool misnan(struct mat33 m) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			if (isnan(m.m[i][j])) {
				return true;
			}
		}
	}
	return false;
}
static inline void set_block33_rowmaj(float *block, int stride, struct mat33 const *m) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			block[j] = m->m[i][j];
		}
		block += stride;
	}
}
static inline struct mat33 madd3(struct mat33 a, struct mat33 b, struct mat33 c) {
	return madd(madd(a, b), c);
}
static inline struct mat33 maxisangle(struct vec axis, float angle) {
	struct mat33 const K = mcrossmat(axis);
	return madd3(
		meye(),
		mscl(sinf(angle), K),
		mscl(1.0f - cosf(angle), mmul(K, K))
	);
}
static inline struct mat33 mrotx(float angle) {
	return maxisangle(mkvec(1.0f, 0.0f, 0.0f), angle);
}
static inline struct mat33 mroty(float angle) {
	return maxisangle(mkvec(0.0f, 1.0f, 0.0f), angle);
}
static inline struct mat33 mrotz(float angle) {
	return maxisangle(mkvec(0.0f, 0.0f, 1.0f), angle);
}
struct quat {
	float x;
	float y;
	float z;
	float w;
};
static inline struct quat mkquat(float x, float y, float z, float w) {
	struct quat q;
	q.x = x; q.y = y; q.z = z; q.w = w;
	return q;
}
static inline struct quat quatvw(struct vec v, float w) {
	struct quat q;
	q.x = v.x; q.y = v.y; q.z = v.z;
	q.w = w;
	return q;
}
static inline struct quat qeye(void) {
	return mkquat(0, 0, 0, 1);
}
static inline struct quat qaxisangle(struct vec axis, float angle) {
	float scale = sinf(angle / 2) / vmag(axis);
	struct quat q;
	q.x = scale * axis.x;
	q.y = scale * axis.y;
	q.z = scale * axis.z;
	q.w = cosf(angle/2);
	return q;
}
static inline struct quat qnormalize(struct quat q);
static inline struct quat qvectovec(struct vec a, struct vec b) {
	struct vec const cross = vcross(a, b);
	float const sinangle = vmag(cross);
	float const cosangle = vdot(a, b);
	float const EPS_ANGLE = 1e-6;
	if (sinangle < EPS_ANGLE) {
		if (cosangle > 0.0f) return qeye();
		else return mkquat(0.0f, 0.0f, 0.0f, 0.0f); 
	}
	float const halfcos = 0.5f * cosangle;
	float const sinhalfangle = sqrtf(fmax(0.5f - halfcos, 0.0f));
	float const coshalfangle = sqrtf(fmax(0.5f + halfcos, 0.0f));
	struct vec const qimag = vscl(sinhalfangle / sinangle, cross);
	float const qreal = coshalfangle;
	return quatvw(qimag, qreal);
}
static inline struct quat rpy2quat(struct vec rpy) {
	float r = rpy.x;
	float p = rpy.y;
	float y = rpy.z;
	float cr = cosf(r / 2.0f); float sr = sinf(r / 2.0f);
	float cp = cosf(p / 2.0f); float sp = sinf(p / 2.0f);
	float cy = cosf(y / 2.0f); float sy = sinf(y / 2.0f);
	float qx = sr * cp * cy -  cr * sp * sy;
	float qy = cr * sp * cy +  sr * cp * sy;
	float qz = cr * cp * sy -  sr * sp * cy;
	float qw = cr * cp * cy +  sr * sp * sy;
	return mkquat(qx, qy, qz, qw);
}
static inline struct quat rpy2quat_small(struct vec rpy) {
	float q2 = vmag2(rpy) / 4.0f;
	if (q2 < 1) {
		return quatvw(vdiv(rpy, 2), sqrtf(1.0f - q2));
	}
	else {
		float w = 1.0f / sqrtf(1.0f + q2);
		return quatvw(vscl(w/2, rpy), w);
	}
}
static inline struct quat mat2quat(struct mat33 m) {
	float w = sqrtf(fmax(0.0f, 1.0f + m.m[0][0] + m.m[1][1] + m.m[2][2])) / 2.0f;
	float x = sqrtf(fmax(0.0f, 1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2])) / 2.0f;
	float y = sqrtf(fmax(0.0f, 1.0f - m.m[0][0] + m.m[1][1] - m.m[2][2])) / 2.0f;
	float z = sqrtf(fmax(0.0f, 1.0f - m.m[0][0] - m.m[1][1] + m.m[2][2])) / 2.0f;
	x = copysign(x, m.m[2][1] - m.m[1][2]);
	y = copysign(y, m.m[0][2] - m.m[2][0]);
	z = copysign(z, m.m[1][0] - m.m[0][1]);
	return mkquat(x, y, z, w);
}
static inline struct vec quat2rpy(struct quat q) {
	struct vec v;
	v.x = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1 - 2 * (fsqr(q.x) + fsqr(q.y))); 
	v.y = asinf(2.0f * (q.w * q.y - q.x * q.z)); 
	v.z = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1 - 2 * (fsqr(q.y) + fsqr(q.z))); 
	return v;
}
static inline struct vec quat2axis(struct quat q) {
	float s = 1.0f / sqrtf(1.0f - q.w * q.w);
	return vscl(s, mkvec(q.x, q.y, q.z));
}
static inline float quat2angle(struct quat q) {
	float angle = 2 * acosf(q.w);
	if (angle > M_PI_F) {
		angle -= 2.0f * M_PI_F;
	}
	return angle;
}
static inline struct vec quatimagpart(struct quat q) {
	return mkvec(q.x, q.y, q.z);
}
static inline struct mat33 quat2rotmat(struct quat q) {
	float x = q.x;
	float y = q.y;
	float z = q.z;
	float w = q.w;
	struct mat33 m;
	m.m[0][0] = 1 - 2*y*y - 2*z*z;
	m.m[0][1] = 2*x*y - 2*z*w;
	m.m[0][2] = 2*x*z + 2*y*w,
	m.m[1][0] = 2*x*y + 2*z*w;
	m.m[1][1] = 1 - 2*x*x - 2*z*z;
	m.m[1][2] = 2*y*z - 2*x*w,
	m.m[2][0] = 2*x*z - 2*y*w;
	m.m[2][1] = 2*y*z + 2*x*w;
	m.m[2][2] = 1 - 2*x*x - 2*y*y;
	return m;
}
static inline struct vec qvrot(struct quat q, struct vec v) {
	struct vec qv = mkvec(q.x, q.y, q.z);
	return vadd3(
		vscl(2.0f * vdot(qv, v), qv),
		vscl(q.w * q.w - vmag2(qv), v),
		vscl(2.0f * q.w, vcross(qv, v))
	);
}
static inline struct quat qqmul(struct quat q, struct quat p) {
	float x =  q.w*p.x + q.z*p.y - q.y*p.z + q.x*p.w;
	float y = -q.z*p.x + q.w*p.y + q.x*p.z + q.y*p.w;
	float z =  q.y*p.x - q.x*p.y + q.w*p.z + q.z*p.w;
	float w = -q.x*p.x - q.y*p.y - q.z*p.z + q.w*p.w;
	return mkquat(x, y, z, w);
}
static inline struct quat qinv(struct quat q) {
	return mkquat(-q.x, -q.y, -q.z, q.w);
}
static inline struct quat qneg(struct quat q) {
	return mkquat(-q.x, -q.y, -q.z, -q.w);
}
static inline struct quat qposreal(struct quat q) {
	if (q.w < 0) return qneg(q);
	return q;
}
static inline float qdot(struct quat a, struct quat b) {
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}
static inline float qanglebetween(struct quat a, struct quat b) {
	float const dot = qdot(qposreal(a), qposreal(b));
	if (dot > 1.0f - 1e9f) return 0.0f;
	if (dot < -1.0f + 1e9f) return M_PI_F;
	return acosf(dot);
}
static inline bool qeq(struct quat a, struct quat b) {
	return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}
static inline struct quat qnormalize(struct quat q) {
	float s = 1.0f / sqrtf(qdot(q, q));
	return mkquat(s*q.x, s*q.y, s*q.z, s*q.w);
}
static inline struct quat quat_gyro_update(struct quat quat, struct vec gyro, float const dt) {
	struct quat q1;
	float const r = (dt / 2) * gyro.x;
	float const p = (dt / 2) * gyro.y;
	float const y = (dt / 2) * gyro.z;
	q1.x =    quat.x + y*quat.y - p*quat.z + r*quat.w;
	q1.y = -y*quat.x +   quat.y + r*quat.z + p*quat.w;
	q1.z =  p*quat.x - r*quat.y +   quat.z + y*quat.w;
	q1.w = -r*quat.x - p*quat.y - y*quat.z +   quat.w;
	return q1;
}
static inline struct quat qnlerp(struct quat a, struct quat b, float t) {
	float s = 1.0f - t;
	return qnormalize(mkquat(
		s*a.x + t*b.x, s*a.y + t*b.y, s*a.z + t*b.z, s*a.w + t*b.w));
}
static inline struct quat qslerp(struct quat a, struct quat b, float t)
{
	float dp = qdot(a, b);
	if (dp < 0) {
		dp = -dp;
		b = qneg(b);
	}
	if (dp > 0.99f) {
		return qnlerp(a, b, t);
	}
	else {
		float theta = acosf(dp);
		float s = sinf(theta * (1 - t)) / sinf(theta);
		t = sinf(theta * t) / sinf(theta);
		return mkquat(
			s*a.x + t*b.x, s*a.y + t*b.y, s*a.z + t*b.z, s*a.w + t*b.w);
	}
}
static inline struct quat qload(double const *d) {
	return mkquat(d[0], d[1], d[2], d[3]);
}
static inline void qstore(struct quat q, double *d) {
	d[0] = (double)q.x; d[1] = (double)q.y; d[2] = (double)q.z; d[3] = (double)q.w;
}
static inline struct quat qloadf(float const *f) {
	return mkquat(f[0], f[1], f[2], f[3]);
}
static inline void qstoref(struct quat q, float *f) {
	f[0] = q.x; f[1] = q.y; f[2] = q.z; f[3] = q.w;
}
static inline struct vec vprojecthalfspace(struct vec x, struct vec a_unit, float b) {
	float ax = vdot(a_unit, x);
	if (ax <= b) {
		return x;
	}
	return vadd(x, vscl(b - ax, a_unit));
}
static inline bool vinpolytope(struct vec v, float const A[], float const b[], int n, float tolerance)
{
	for (int i = 0; i < n; ++i) {
		struct vec a = vloadf(A + 3 * i);
		if (vdot(a, v) > b[i] + tolerance) {
			return false;
		}
	}
	return true;
}
static inline float rayintersectpolytope(struct vec origin, struct vec direction, float const A[], float const b[], int n, int *active_row)
{
	#ifdef CMATH3D_ASSERTS
	for (int i = 0; i < n; ++i) {
		struct vec a = vloadf(A + 3 * i);
		assert(fabsf(vmag2(a) - 1.0f) < 1e-6f);
	}
	#endif
	float min_s = INFINITY;
	int min_row = -1;
	for (int i = 0; i < n; ++i) {
		struct vec a = vloadf(A + 3 * i);
		float a_dir = vdot(a, direction);
		if (a_dir <= 0.0f) {
			continue;
		}
		float s = (b[i] - vdot(a, origin)) / a_dir;
		if (s < min_s) {
			min_s = s;
			min_row = i;
		}
	}
	if (active_row != NULL) {
		*active_row = min_row;
	}
	return min_s;
}
static inline struct vec vprojectpolytope(struct vec v, float const A[], float const b[], float work[], int n, float tolerance, int maxiters)
{
	if (vinpolytope(v, A, b, n, tolerance)) {
		return v;
	}
	#ifdef CMATH3D_ASSERTS
	for (int i = 0; i < n; ++i) {
		struct vec a = vloadf(A + 3 * i);
		assert(fabsf(vmag2(a) - 1.0f) < 1e-6f);
	}
	#endif
	float *z = work;
	for (int i = 0; i < 3 * n; ++i) {
		z[i] = 0.0f;
	}
	float const tolerance2 = n * fsqr(tolerance) / 10.0f;
	struct vec x = v;
	for (int iter = 0; iter < maxiters; ++iter) {
		float c = 0.0f;
		for (int i = 0; i < n; ++i) {
			struct vec x_old = x;
			struct vec ai = vloadf(A + 3 * i);
			struct vec zi_old = vloadf(z + 3 * i);
			x = vprojecthalfspace(vsub(x_old, zi_old), ai, b[i]);
			struct vec zi = vadd3(x, vneg(x_old), zi_old);
			vstoref(zi, z + 3 * i);
			c += vdist2(zi_old, zi);
		}
		if (c < tolerance2) {
			return x;
		}
	}
	return x;
}