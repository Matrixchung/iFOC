#include "foc_math.h"
/*
    @brief Clark transform for three phase input
    @param abc_t input a, b, c
    @retval transformed alphabeta_t
*/
alphabeta_t FOC_Clark_ABC(abc_t input)
{
    alphabeta_t output;
    output.alpha = ((2.0f * input.a) - input.b - input.c) / 3.0f;
    output.beta = (input.b - input.c) * divSQRT_3;
    return output;
}
/*
    @brief Clark transform for two phase input
    @param abc_t input a, b, c
    @retval transformed alphabeta_t
*/
alphabeta_t FOC_Clark_AB(ab_t input)
{
    alphabeta_t output;
    output.alpha = input.a;
    output.beta = (input.a + (2.0f * input.b)) * divSQRT_3;
    return output;
}
/*
    @brief Park transform
    @param alphabeta_t input alpha, beta
    @param theta: radian, [0, 2PI]
    @retval transformed qd_t
*/
qd_t FOC_Park(alphabeta_t input, float theta)
{
    float _sin = arm_sin_f32(theta);
    float _cos = arm_cos_f32(theta);
    qd_t output;
    output.q = _cos * input.beta - _sin * input.alpha;
    output.d = _cos * input.alpha + _sin * input.beta;
    return output;
}
/*
    @brief Reverse Park transform
    @param input: qd_t q, d
    @param theta: radian, [0, 2PI]
    @retval reverse transformed alphabeta_t
*/
alphabeta_t FOC_Rev_Park(qd_t input, float theta)
{
    float _sin = arm_sin_f32(theta);
    float _cos = arm_cos_f32(theta);
    alphabeta_t output;
    output.alpha = _cos * input.d - _sin * input.q;
    output.beta = _cos * input.q + _sin * input.d;
    return output;
}

void FOC_SVPWM(svpwm_t *svpwm, float Vbus)
{
    svpwm->sector = 0;
    float Ualpha_Pu = svpwm->Ualphabeta.alpha / Vbus;
    float Ubeta_Pu = svpwm->Ualphabeta.beta / Vbus;
    Ualpha_Pu = _constrain(Ualpha_Pu, -1.0f, 1.0f);
    Ubeta_Pu = _constrain(Ubeta_Pu, -1.0f, 1.0f);
    float Uabc_Pu[3] = {Ualpha_Pu, 0.0f, 0.0f};
    Ualpha_Pu *= -0.5f;
    Ubeta_Pu *= SQRT_3div2;
    Uabc_Pu[1] = Ualpha_Pu + Ubeta_Pu;
    Uabc_Pu[2] = Ualpha_Pu - Ubeta_Pu;
    if(Uabc_Pu[0] > 0.0f) svpwm->sector = 1;
    if(Uabc_Pu[1] > 0.0f) svpwm->sector += 2;
    if(Uabc_Pu[2] > 0.0f) svpwm->sector += 4;
    float Ucom_Pu = 0.5f * (max3(Uabc_Pu) + min3(Uabc_Pu));
    float tX[3] = {Uabc_Pu[0] - Ucom_Pu + 0.5f, Uabc_Pu[1] - Ucom_Pu + 0.5f, Uabc_Pu[2] - Ucom_Pu + 0.5f};
    float tMin = min3(tX);
    if(tMin < 0.0f) // Minimum-phase-error Modulation
    {
        if(tMin == tX[0])
        {
            tX[0] = 0.0f;
            tX[1] = tX[1] / (tX[1] + tX[2]);
            tX[2] = 1.0f - tX[1];
        }
        else if(tMin == tX[1])
        {
            tX[1] = 0.0f;
            tX[0] = tX[0] / (tX[0] + tX[2]);
            tX[2] = 1.0f - tX[0];
        }
        else
        {
            tX[2] = 0.0f;
            tX[1] = tX[1] / (tX[0] + tX[1]);
            tX[0] = 1.0f - tX[1];
        }
    }
    svpwm->compare_a = (uint16_t)(tX[0] * (float)svpwm->max_compare);
    svpwm->compare_b = (uint16_t)(tX[1] * (float)svpwm->max_compare);
    svpwm->compare_c = (uint16_t)(tX[2] * (float)svpwm->max_compare);
}

float normalize_rad(float radian)
{
    float a = fmodf(radian, PI2);
    return a >= 0 ? a : (a + PI2);
}
/**
 * Fast atan2
 */
float fast_atan2f(float y, float x)
{
	float abs_y = fabsf(y) + 1e-20; // kludge to prevent 0/0 condition
	float angle = 0.0f;
	if (x >= 0)
	{
		float r = (x - abs_y) / (x + abs_y);
		float rsq = r * r;
		angle = ((0.1963f * rsq) - 0.9817f) * r + (PI / 4.0f);
	}
	else
	{
		float r = (x + abs_y) / (abs_y - x);
		float rsq = r * r;
		angle = ((0.1963f * rsq) - 0.9817f) * r + (3.0f * PI / 4.0f);
	}
	if(y < 0) return (-angle);
    return angle;
}
float max3(float *data)
{
    if(data[0] >= data[1] && data[0] >= data[2]) return data[0];
    else if(data[2] >= data[0] && data[2] >= data[1]) return data[2];
    return data[1];
}
float min3(float *data)
{
    if(data[0] <= data[1] && data[0] <= data[2]) return data[0];
    else if(data[2] <= data[0] && data[2] <= data[1]) return data[2];
    return data[1];
}
float max(float *data, uint16_t len)
{
    float result = *data;
    data++;
    len--;
    while(len--)
    {
        if(*data >= result) result = *data;
        data++;
    }
    return result;
}
float min(float *data, uint16_t len)
{
    float result = *data;
    data++;
    len--;
    while(len--)
    {
        if(*data <= result) result = *data;
        data++;
    }
    return result;
}
#define _div30_PI (float)(9.5492965855137201461330258023509f)
#define _divPI_30 (float)(0.1047197551196597746154214461093f)
float rad_speed_to_RPM(float omega, uint8_t pole_pair)
{
    return (float)(omega * _div30_PI / (float)pole_pair);
}
float RPM_speed_to_rad(float rpm, uint8_t pole_pair)
{
    return (float)(rpm * _divPI_30 * (float)pole_pair);
}
float sat(float s, float delta)
{
	if(s > delta)
		return 1.0f;
	else if(s < -delta)
		return -1.0f;
	else
		return s/delta;
}
const float atanTab[512] = {
0.000000f, 0.001957f, 0.003914f, 0.005871f, 0.007828f, 0.009784f, 0.011741f, 0.013698f, 0.015654f, 0.017611f, 
0.019567f, 0.021523f, 0.023479f, 0.025435f, 0.027390f, 0.029346f, 0.031301f, 0.033256f, 0.035210f, 0.037165f, 
0.039119f, 0.041073f, 0.043026f, 0.044979f, 0.046932f, 0.048885f, 0.050837f, 0.052788f, 0.054740f, 0.056691f, 
0.058641f, 0.060591f, 0.062541f, 0.064490f, 0.066438f, 0.068386f, 0.070334f, 0.072281f, 0.074227f, 0.076173f, 
0.078119f, 0.080063f, 0.082007f, 0.083951f, 0.085894f, 0.087836f, 0.089778f, 0.091718f, 0.093659f, 0.095598f, 
0.097537f, 0.099475f, 0.101412f, 0.103349f, 0.105284f, 0.107219f, 0.109153f, 0.111087f, 0.113019f, 0.114951f, 
0.116882f, 0.118812f, 0.120741f, 0.122669f, 0.124596f, 0.126522f, 0.128447f, 0.130372f, 0.132295f, 0.134218f, 
0.136139f, 0.138059f, 0.139979f, 0.141897f, 0.143814f, 0.145731f, 0.147646f, 0.149560f, 0.151473f, 0.153385f, 
0.155295f, 0.157205f, 0.159113f, 0.161020f, 0.162926f, 0.164831f, 0.166735f, 0.168637f, 0.170539f, 0.172439f, 
0.174337f, 0.176235f, 0.178131f, 0.180026f, 0.181919f, 0.183811f, 0.185702f, 0.187592f, 0.189480f, 0.191367f, 
0.193252f, 0.195137f, 0.197019f, 0.198900f, 0.200780f, 0.202659f, 0.204536f, 0.206411f, 0.208285f, 0.210158f, 
0.212029f, 0.213898f, 0.215766f, 0.217633f, 0.219498f, 0.221361f, 0.223223f, 0.225083f, 0.226942f, 0.228799f, 
0.230654f, 0.232508f, 0.234360f, 0.236211f, 0.238060f, 0.239907f, 0.241753f, 0.243597f, 0.245439f, 0.247280f, 
0.249118f, 0.250956f, 0.252791f, 0.254625f, 0.256457f, 0.258287f, 0.260115f, 0.261942f, 0.263767f, 0.265590f, 
0.267411f, 0.269230f, 0.271048f, 0.272864f, 0.274677f, 0.276489f, 0.278300f, 0.280108f, 0.281914f, 0.283719f, 
0.285522f, 0.287322f, 0.289121f, 0.290918f, 0.292713f, 0.294506f, 0.296297f, 0.298086f, 0.299874f, 0.301659f, 
0.303442f, 0.305223f, 0.307002f, 0.308780f, 0.310555f, 0.312328f, 0.314099f, 0.315868f, 0.317635f, 0.319400f, 
0.321163f, 0.322924f, 0.324683f, 0.326440f, 0.328195f, 0.329947f, 0.331698f, 0.333446f, 0.335192f, 0.336936f, 
0.338678f, 0.340418f, 0.342156f, 0.343891f, 0.345625f, 0.347356f, 0.349085f, 0.350812f, 0.352537f, 0.354259f, 
0.355980f, 0.357698f, 0.359414f, 0.361128f, 0.362839f, 0.364548f, 0.366256f, 0.367960f, 0.369663f, 0.371363f, 
0.373062f, 0.374757f, 0.376451f, 0.378142f, 0.379831f, 0.381518f, 0.383203f, 0.384885f, 0.386565f, 0.388243f, 
0.389918f, 0.391591f, 0.393262f, 0.394930f, 0.396596f, 0.398260f, 0.399921f, 0.401581f, 0.403237f, 0.404892f, 
0.406544f, 0.408194f, 0.409841f, 0.411486f, 0.413129f, 0.414769f, 0.416407f, 0.418043f, 0.419676f, 0.421307f, 
0.422935f, 0.424561f, 0.426185f, 0.427806f, 0.429425f, 0.431042f, 0.432656f, 0.434268f, 0.435877f, 0.437484f, 
0.439088f, 0.440690f, 0.442290f, 0.443887f, 0.445482f, 0.447075f, 0.448665f, 0.450252f, 0.451837f, 0.453420f, 
0.455000f, 0.456578f, 0.458153f, 0.459726f, 0.461297f, 0.462865f, 0.464430f, 0.465993f, 0.467554f, 0.469112f, 
0.470668f, 0.472221f, 0.473772f, 0.475320f, 0.476866f, 0.478410f, 0.479951f, 0.481489f, 0.483025f, 0.484559f, 
0.486090f, 0.487618f, 0.489144f, 0.490668f, 0.492189f, 0.493708f, 0.495224f, 0.496738f, 0.498249f, 0.499758f, 
0.501264f, 0.502768f, 0.504269f, 0.505768f, 0.507265f, 0.508759f, 0.510250f, 0.511739f, 0.513225f, 0.514709f, 
0.516191f, 0.517670f, 0.519146f, 0.520620f, 0.522092f, 0.523561f, 0.525027f, 0.526491f, 0.527953f, 0.529412f, 
0.530868f, 0.532323f, 0.533774f, 0.535223f, 0.536670f, 0.538114f, 0.539556f, 0.540995f, 0.542432f, 0.543866f, 
0.545298f, 0.546727f, 0.548154f, 0.549578f, 0.551000f, 0.552419f, 0.553836f, 0.555250f, 0.556662f, 0.558071f, 
0.559478f, 0.560883f, 0.562285f, 0.563684f, 0.565081f, 0.566476f, 0.567868f, 0.569258f, 0.570645f, 0.572030f, 
0.573412f, 0.574792f, 0.576169f, 0.577544f, 0.578916f, 0.580286f, 0.581653f, 0.583019f, 0.584381f, 0.585741f, 
0.587099f, 0.588454f, 0.589807f, 0.591157f, 0.592505f, 0.593850f, 0.595193f, 0.596534f, 0.597872f, 0.599208f, 
0.600541f, 0.601872f, 0.603200f, 0.604526f, 0.605850f, 0.607171f, 0.608490f, 0.609806f, 0.611120f, 0.612431f, 
0.613740f, 0.615047f, 0.616351f, 0.617653f, 0.618952f, 0.620249f, 0.621544f, 0.622836f, 0.624126f, 0.625414f, 
0.626699f, 0.627981f, 0.629262f, 0.630539f, 0.631815f, 0.633088f, 0.634359f, 0.635627f, 0.636893f, 0.638157f, 
0.639418f, 0.640677f, 0.641934f, 0.643188f, 0.644440f, 0.645689f, 0.646936f, 0.648181f, 0.649424f, 0.650664f, 
0.651902f, 0.653137f, 0.654370f, 0.655601f, 0.656830f, 0.658056f, 0.659280f, 0.660501f, 0.661720f, 0.662937f, 
0.664152f, 0.665364f, 0.666574f, 0.667782f, 0.668987f, 0.670190f, 0.671391f, 0.672589f, 0.673786f, 0.674980f, 
0.676171f, 0.677361f, 0.678548f, 0.679733f, 0.680915f, 0.682095f, 0.683274f, 0.684449f, 0.685623f, 0.686794f, 
0.687963f, 0.689130f, 0.690295f, 0.691457f, 0.692617f, 0.693775f, 0.694931f, 0.696084f, 0.697235f, 0.698384f, 
0.699531f, 0.700676f, 0.701818f, 0.702958f, 0.704096f, 0.705232f, 0.706366f, 0.707497f, 0.708626f, 0.709753f, 
0.710878f, 0.712001f, 0.713121f, 0.714240f, 0.715356f, 0.716470f, 0.717582f, 0.718691f, 0.719799f, 0.720904f, 
0.722008f, 0.723109f, 0.724208f, 0.725305f, 0.726399f, 0.727492f, 0.728582f, 0.729671f, 0.730757f, 0.731841f, 
0.732923f, 0.734003f, 0.735081f, 0.736157f, 0.737230f, 0.738302f, 0.739371f, 0.740439f, 0.741504f, 0.742567f, 
0.743628f, 0.744687f, 0.745744f, 0.746799f, 0.747852f, 0.748903f, 0.749952f, 0.750999f, 0.752044f, 0.753086f, 
0.754127f, 0.755165f, 0.756202f, 0.757237f, 0.758269f, 0.759300f, 0.760328f, 0.761355f, 0.762379f, 0.763402f, 
0.764422f, 0.765441f, 0.766457f, 0.767472f, 0.768484f, 0.769495f, 0.770504f, 0.771510f, 0.772515f, 0.773518f, 
0.774518f, 0.775517f, 0.776514f, 0.777509f, 0.778502f, 0.779493f, 0.780482f, 0.781469f, 0.782454f, 0.783437f, 
0.784419f, 0.785398f
};
float fast_atanf(float x)
{
	unsigned short idx;
	float atan_val;
	if (x > 1)
	{
		idx = 512 * (x - 1) / (x + 1);
		atan_val = atanTab[idx] + 0.785398f;
	}
	else if (x >= 0 && x <= 1)
	{
		idx = 512 * x;
		atan_val = atanTab[idx];
	}
	else if (x >= -1 && x < 0)
	{
		idx = 512 * (-x);
		atan_val = -atanTab[idx];
	}
	else
	{
		idx = 512 * (-x - 1) / (-x + 1);
		atan_val = -atanTab[idx] - 0.785398f;
	}
	return atan_val;
}

static const uint8_t crc8_table[256] =
{
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
    0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
    0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
    0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
    0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
    0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
    0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
    0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
    0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
    0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
    0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};
uint8_t getCRC8(uint8_t* data, uint16_t len)
{
  uint16_t i = 0;
  uint8_t crc = 0x0;
  for (i = 0; i < len; i++) 
  {
    crc = crc8_table[(uint8_t)crc ^ *data];
    data++;
  }
  return crc;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
 
float fast_inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
#pragma GCC diagnostic pop

float origin_to_shaft(float origin, float gear_ratio)
{
    return (float)(origin / gear_ratio);
}
float shaft_to_origin(float shaft, float gear_ratio)
{
    return (float)(shaft * gear_ratio);
}