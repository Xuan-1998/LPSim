#version 420
#extension GL_EXT_gpu_shader4 : enable

in vec3 outColor;
in vec3 outUV;
in vec3 origVertex;// L

in vec3 varyingNormal;
in vec4 varyingLightVertexPosition;//position respect the camera view

out vec4 outputF;

uniform int mode;
uniform sampler2D tex0;
uniform sampler2DArray tex_2DA;
uniform sampler3D tex_3D;
//uniform sampler3D tex_3D;


uniform sampler2D shadowMap;
uniform int shadowState;

// terrain
uniform vec4 terrain_size;
uniform sampler2D terrain_tex;
uniform int terrain_vis_type;

// temp/pres
uniform float vis3_z;
uniform float vis_transp;
uniform int vis_type;
 
uniform vec3 lightDir;
uniform mat4 light_mvpMatrix;
uniform mat4 light_biasMatrix;	//transfrom to [0,1] tex coordinates

const float ambientColor=0.1;
const float diffuseColor=1.0;
const float specularColor=1.0;

/////////////////////////////////////////////////////////////////
// SHADOW
///////////////////////////////////////////////////////////////////

vec2 poissonDisk4[4] = vec2[](
  vec2( -0.94201624, -0.39906216 ),
  vec2( 0.94558609, -0.76890725 ),
  vec2( -0.094184101, -0.92938870 ),
  vec2( 0.34495938, 0.29387760 )
);

vec2 poissonDisk16[16] = vec2[]( 
   vec2( -0.94201624, -0.39906216 ), 
   vec2( 0.94558609, -0.76890725 ), 
   vec2( -0.094184101, -0.92938870 ), 
   vec2( 0.34495938, 0.29387760 ), 
   vec2( -0.91588581, 0.45771432 ), 
   vec2( -0.81544232, -0.87912464 ), 
   vec2( -0.38277543, 0.27676845 ), 
   vec2( 0.97484398, 0.75648379 ), 
   vec2( 0.44323325, -0.97511554 ), 
   vec2( 0.53742981, -0.47373420 ), 
   vec2( -0.26496911, -0.41893023 ), 
   vec2( 0.79197514, 0.19090188 ), 
   vec2( -0.24188840, 0.99706507 ), 
   vec2( -0.81409955, 0.91437590 ), 
   vec2( 0.19984126, 0.78641367 ), 
   vec2( 0.14383161, -0.14100790 ) 
);

// Returns a random number based on a vec3 and an int.
float random(vec3 seed, int i){
	vec4 seed4 = vec4(seed,i);
	float dot_product = dot(seed4, vec4(12.9898,78.233,45.164,94.673));
	return fract(sin(dot_product) * 43758.5453);
}

float shadowCoef(){
	vec4 shadow_coord2=light_mvpMatrix*vec4(origVertex,1.0);
	vec3 ProjCoords = shadow_coord2.xyz / shadow_coord2.w;
    vec2 UVCoords;
    UVCoords.x = 0.5 * ProjCoords.x + 0.5;
    UVCoords.y = 0.5 * ProjCoords.y + 0.5;
    float z = 0.5 * ProjCoords.z + 0.5;
	
	/// D
	const bool HDShadow=false;
	float visibility=1.0f;
	if(HDShadow==true){
		// HDShadow
		for (int i=0;i<8;i++){
			int index = int(16.0*random(origVertex.xyz, i))%16;
			if ( texture2D( shadowMap, UVCoords + poissonDisk16[index]/3500.0 ).z  <  z ){
				visibility-=0.1;
			}
		}
	}else{
		// Low Shadow
		for (int i=0;i<4;i++){
			int index = int(4.0*random(origVertex.xyz, i))%4;
			if ( texture2D( shadowMap, UVCoords + poissonDisk4[index]/3500.0 ).z  <  z ){
				visibility-=0.2;
			}
		}

	}
	return visibility;
}//

///////////////////////////////////////////////////////////////////
// SKY FOR WATER
///////////////////////////////////////////////////////////////////

// sun
uniform vec2 sunValues;//alt azimuth

const float PI = 3.14159265358979323846;
const float PI_2 = 1.57079632679489661923;
const float PI_4 = 0.785398163397448309616;

vec3 vGammaPowerInverse = vec3(2.2);
vec3 vGammaPower = vec3(0.454545);

vec3 linear_to_gamma(vec3 c){
	return pow(clamp(c, 0.0, 1.0), vGammaPower);
}

vec3 gamma_to_linear(vec3 c){
	return pow(clamp(c, 0.0, 1.0), vGammaPowerInverse);
}

// SCENE PARAMETERS

// North is pointed by x
// East is pointed by z
// Sea Level : y = 0.0
	
struct planet_t{
	float rs;  	// sea level radius
	float ra; 	// atmosphere radius
};

planet_t earth = planet_t(6360.0e3,6420.0e3);

struct sun_t{  
    vec3 beta_r;	// rayleigh scattering coefs at sea level
	vec3 beta_m; 	// mie scattering coefs at sea level
	float sh_r; 	// rayleigh scale height
	float sh_m; 	// mie scale height
	float i;		// sun intensity
	float mc;	    // mean cosine
	float azi; 		// azimuth
	float alt; 		// altitude
	float ad; 		// angular diameter (between 0.5244 and 5.422 for our sun)
    vec3 color;
};

sun_t sun = sun_t(vec3(5.5e-6, 13.0e-6, 22.1e-6),
				  vec3(21.0e-6),
				  7994.0,
				  1200.0,
				  20.0,
				  0.76,
				  4.4,
				  PI_2,
				  0.53,
				  vec3(1.0, 1.0, 1.0));

// Use www.esrl.noaa.gov/gmd/grad/solcalc/azel.html to get azimuth and altitude
// of our sun at a specific place and time

// RAY MARCHING STUFF

// pred : rd is normalized
bool intersect_with_atmosphere(in vec3 ro, in vec3 rd, in planet_t planet, out float tr){
	float c = length(ro); // distance from center of the planet :)
	vec3 up_dir = ro / c;
	float beta = PI - acos(dot(rd, up_dir)); 
	float sb = sin(beta);
	float b = planet.ra;
	float bt = planet.rs - 10.0;
	tr = sqrt((b * b) - (c * c) * (sb * sb)) + c * cos(beta); // sinus law
	if (sqrt((bt * bt) - (c * c) * (sb * sb)) + c * cos(beta) > 0.0)
		return false;
	return true;
}

const int SKYLIGHT_NB_VIEWDIR_SAMPLES = 12;
const int SKYLIGHT_NB_SUNDIR_SAMPLES = 6;

float compute_sun_visibility(in sun_t sun, float alt){
	float vap = 0.0;
	float h, a;
	float vvp = clamp((0.5 + alt / sun.ad), 0.0, 1.0); // vertically visible percentage
	if (vvp == 0.0)
		return 0.0;
	else if (vvp == 1.0)
		return 1.0;		
	bool is_sup;
	if (vvp > 0.5){
		is_sup = true;
		h = (vvp - 0.5) * 2.0;
	}else{
		is_sup = false;
		h = (0.5 - vvp) * 2.0;
	}
	float alpha = acos(h) * 2.0;
	a = (alpha - sin(alpha)) / (2.0 * PI);
	if (is_sup)
		vap = 1.0 - a;
	else
		vap = a;
	return vap;
}

// pred : rd is normalized
vec3 compute_sky_light(in vec3 ro, in vec3 rd, in planet_t planet, in sun_t sun){
    float t1;
    if (!intersect_with_atmosphere(ro, rd, planet, t1) || t1 < 0.0)
		return vec3(0.2);
    float sl = t1 / float(SKYLIGHT_NB_VIEWDIR_SAMPLES); // seg length
    float t = 0.0;
	float calt = cos(sun.alt);
	vec3 sun_dir = vec3(cos(sun.azi) * calt,
						sin(sun.alt),
						sin(sun.azi) * calt);
	float mu = dot(rd, sun_dir);
	float mu2 = mu * mu;
	float mc2 = sun.mc * sun.mc;
	// rayleigh stuff
	vec3 sumr = vec3(0.0);
    float odr = 0.0; // optical depth
	float phase_r = (3.0 / (16.0 * PI)) * (1.0 + mu2);
	// mie stuff
	vec3 summ = vec3(0.0);
	float odm = 0.0; // optical depth
	float phase_m = ((3.0 / (8.0 * PI)) * ((1.0 - mc2) * (1.0 + mu2))) /
		            ((2.0 + mc2) * pow(1.0 + mc2 - 2.0 * sun.mc * mu, 1.5));
    for (int i = 0; i < SKYLIGHT_NB_VIEWDIR_SAMPLES; ++i){
		vec3 sp = ro + rd * (t + 0.5 * sl);
        float h = length(sp) - planet.rs;
        float hr = exp(-h / sun.sh_r) * sl;
		odr += hr;
        float hm = exp(-h / sun.sh_m) * sl;
        odm += hm;
		float tm;
		float sp_alt = PI_2 - asin(planet.rs / length(sp));
		sp_alt += acos(normalize(sp).y) + sun.alt;
		float coef = compute_sun_visibility(sun, sp_alt);
		if (intersect_with_atmosphere(sp, sun_dir, planet, tm) || coef > 0.0){
			float sll = tm / float(SKYLIGHT_NB_SUNDIR_SAMPLES);
			float tl = 0.0;
			float odlr = 0.0, odlm = 0.0;
			for (int j = 0; j < SKYLIGHT_NB_SUNDIR_SAMPLES; ++j){
				vec3 spl = sp + sun_dir * (tl + 0.5 * sll);
				float spl_alt = PI_2 - asin(planet.rs / length(spl));
				spl_alt += acos(normalize(spl).y) + sun.alt;
				float coefl = compute_sun_visibility(sun, spl_alt);
				float hl = length(spl) - planet.rs;
				odlr += exp(-hl / sun.sh_r) * sll * (1.0 - log(coefl + 0.000001));
				odlm += exp(-hl / sun.sh_m) * sll * (1.0 - log(coefl + 0.000001));
				tl += sll;
			}
			vec3 tau = sun.beta_r * (odr + odlr) + sun.beta_m * 1.05 * (odm + odlm);
			vec3 attenuation = vec3(exp(-tau.x), exp(-tau.y), exp(-tau.z));
			sumr +=  hr * attenuation * coef;
			summ +=  hm * attenuation * coef;
		}
        t += sl;
    }
    return sun.i * (sumr * phase_r * sun.beta_r + summ * phase_m * sun.beta_m);
}

vec4 computerSkyRay(vec3 camPosXZY,vec3 camDirXZY){

	vec3 rd = camDirXZY;
	vec3 gp = camPosXZY + vec3(0.0, earth.rs + 1.0, 0.0);

	//sun.alt = 2.6;
	sun.alt=sunValues.x;
	sun.azi=sunValues.y;

	vec3 res = compute_sky_light(gp, rd, earth, sun);
	res = linear_to_gamma(res);
	
	//res *= 0.5 + 0.5 * pow(16.0 * q.x * q.y * (1.0 - q.x) * (1.0 - q.y), 0.1);

	return vec4(res,1.0);
}

///////////////////////////////////////////////////////////////////////

vec4 hsv_to_rgb(float h, float s, float v, float a)
{
	float c = v * s;
	h = mod((h * 6.0), 6.0);
	float x = c * (1.0 - abs(mod(h, 2.0) - 1.0));
	vec4 color;
 
	if (0.0 <= h && h < 1.0) {
		color = vec4(c, x, 0.0, a);
	} else if (1.0 <= h && h < 2.0) {
		color = vec4(x, c, 0.0, a);
	} else if (2.0 <= h && h < 3.0) {
		color = vec4(0.0, c, x, a);
	} else if (3.0 <= h && h < 4.0) {
		color = vec4(0.0, x, c, a);
	} else if (4.0 <= h && h < 5.0) {
		color = vec4(x, 0.0, c, a);
	} else if (5.0 <= h && h < 6.0) {
		color = vec4(c, 0.0, x, a);
	} else {
		color = vec4(0.0, 0.0, 0.0, a);
	}
 
	color.rgb += v - c;
 
	return color;
}//

void main(){

	outputF =vec4(outColor,1.0);

	// SHADOW: From light
	if(shadowState==2){// no tex/color/shadows
		return;
	}

	/////////////////
	// 2D RENDER
	if((mode&0x0FF)==14){
		return;//just outColor
	}
	if((mode&0x0FF)==11){
		// B&W
		/*outputF=texture(tex0,outUV.rg);
		vec4 col=texture(tex0,outUV.rg);
		float value=col.r;//+col.g+col.b;
		outputF=vec4(value,value,value,1.0);
		return;*/
		// iso of 0.2
		/*float value=texture(tex0,outUV.rg).r;
		const vec3 colorA[4]=vec3[4](
			vec3(0,0.97,0.06),
			vec3(1.00,0.95,0),
			vec3(0.80,0.50,0.19),
			vec3(0.15,0.13,0.09)
			);
		value=mod(value,0.2);
		outputF=value<0.01||value>0.19?vec4(0):vec4(1.0);*/
		float value=texture(tex0,outUV.rg).r;
		float h=value/3.0;
		outputF=hsv_to_rgb(h,1.0,0.5,1.0);
		//vec4 col=texture(tex0,outUV.rg);
		//float value=col.r+col.g+col.b+col.a;
		//outputF=vec4(value,value,value,1.0);
		//if(value!=0)
		//	outputF=vec4(1,1,1,1.0);
		//else
		//	outputF=vec4(0,0,0,1.0);
		/*float value=255.0*texture(tex0,outUV.rg).r;
		//outputF=vec4(value,value,value,1.0);
		const vec3 colorA[4]=vec3[4](
			vec3(0,0.97,0.06),
			vec3(1.00,0.95,0),
			vec3(0.80,0.50,0.19),
			vec3(0.15,0.13,0.09)
			);
		int texInd=int(value/85);
		float interpTex=mod(value,85.0);
		value=mod(value,25.0);
		outputF=value<1.5||value>23.5?vec4(mix(
			colorA[texInd],
			colorA[texInd+1],
			interpTex/85.0),1.0):vec4(1.0);*/
		return;
	}
	// 2D Render Terrein
	if((mode&0x0FF)==12){
		//color
		/*outputF=texture(terrain_tex,outUV.rg);*/
		const vec3 colorA[4]=vec3[4](
			vec3(0,0.97,0.06),
			vec3(1.00,0.95,0),
			vec3(0.80,0.50,0.19),
			vec3(0.15,0.13,0.09)
			);
		float heightZ=texture(terrain_tex,outUV.rg).r;
		if(heightZ<0.01){//water
			outputF=vec4(0,0,1,1);
			return;
		}
		if(heightZ>=1.0){//top montain
			outputF=vec4(1.0);
			return;
		}
		int texInd=int(heightZ/0.25);
		float interpTex=mod(heightZ,0.25);
		
		heightZ=mod(heightZ,0.1);
		outputF=heightZ<0.007||heightZ>0.093?vec4(mix(
			colorA[texInd],
			colorA[texInd+1],
			interpTex/0.25),1.0):vec4(1.0);
		return;
	}
	// 3D Render Temperature/Pressure
	if((mode&0x0FF)==13){
		// TEMPERATURE
		if(vis_type==0){
			const vec3 colorA[11]=vec3[11](
				vec3(0.51,0.51,0.51),//dark grey
				vec3(0.80,0.80,0.80),//grey
				vec3(0.55,0.00,0.68),//purple
				vec3(0.12,0.24,1.00),//dark blue
				vec3(0.59,0.90,1.00),//light blue
				vec3(0.00,0.47,0.00),//dark green
				vec3(0.55,1.00,0.55),//light green
				vec3(0.96,0.86,0.35),//yellow
				vec3(0.94,0.55,0.08),//orange
				vec3(0.86,0.00,0.00),//red
				vec3(0.55,0.00,0.00)//dark red
				);
			float temp=texture( tex0, outUV.rg ).r-273.15f;//to celsius
			temp=clamp(temp,-49.9999,49.9999);

			int texInd=int((temp+50.0)/10);//0-10 (we have 11 in the vec for +1)
			float interpTex=mod(temp+50.0,10.00);
			outputF=vec4(mix(
				colorA[texInd],
				colorA[texInd+1],
				interpTex/10.00),vis_transp);//transparency defined in uniform
			//outputF = texture( tex0, outUV.rg );//vec4(1,0,1,1);// texture( tex0, outUV.rg );
			return;
		}
		// PRESSURE
		if(vis_type==1){
			const vec3 colorA[2]=vec3[2](
				vec3(0.12,0.24,1.00),//dark blue
				vec3(0.86,0.00,0.00)//red
				);
			float pres=texture( tex0, outUV.rg ).r/1000.0f;//kPa
			pres=clamp(pres,98.0,105.0f);
	
			outputF=vec4(mix(
				colorA[0],
				colorA[1],
				(pres-98.0f)/(105.0f-98.0f)),vis_transp);//transparency defined in uniform
			//outputF = texture( tex0, outUV.rg );//vec4(1,0,1,1);// texture( tex0, outUV.rg );
			return;
		}
		if(vis_type==2){
			float pres=texture( tex0, outUV.rg ).r/100.0f;//mb
			pres=clamp(pres,980.0,1052.0);
			pres=mod(pres,4.0);//iso each 4
			if(pres<0.2||pres>3.8){
				discard;
				return;
			}
			outputF=vec4(1.0);
			return;
		}
		/////////////////////////
		// 3D
		if(vis_type==4){
			//outputF =vec4(vis3_z,vis3_z,vis3_z,1.0);
			//float temp2=texture( tex_3D, vec3(outUV.rg,vis3_z) ).r;
			//float temp2=texture( tex_3D, vec3(outUV.rg,vis3_z) ).r;
			//outputF =vec4(temp2,temp2,temp2,1.0);
			//return;
			const vec3 colorA[11]=vec3[11](
				vec3(0.51,0.51,0.51),//dark grey
				vec3(0.80,0.80,0.80),//grey
				vec3(0.55,0.00,0.68),//purple
				vec3(0.12,0.24,1.00),//dark blue
				vec3(0.59,0.90,1.00),//light blue
				vec3(0.00,0.47,0.00),//dark green
				vec3(0.55,1.00,0.55),//light green
				vec3(0.96,0.86,0.35),//yellow
				vec3(0.94,0.55,0.08),//orange
				vec3(0.86,0.00,0.00),//red
				vec3(0.55,0.00,0.00)//dark red
				);
			float temp=texture( tex_3D, vec3(outUV.rg,vis3_z) ).r-273.15f;//to celsius
			temp=clamp(temp,-49.9999,49.9999);

			int texInd=int((temp+50.0)/10);//0-10 (we have 11 in the vec for +1)
			float interpTex=mod(temp+50.0,10.00);
			outputF=vec4(mix(
				colorA[texInd],
				colorA[texInd+1],
				interpTex/10.00),vis_transp);//transparency defined in uniform
			//outputF = texture( tex0, outUV.rg );//vec4(1,0,1,1);// texture( tex0, outUV.rg );
			return;
		}
	}
	///////////////////////////////////////////////////////
	//////////////////////////////////////////////////////
	// TEXTURE
	if((mode&0xFF)==2||/*(mode&0xFF)==4||*/(mode&0xFF)==6){// tex / water / model texture
		outputF = texture( tex0, outUV.rg );
	}
	
	//////////////
	// TERRAIN
	if((mode&0xFF)==3){
		const vec3 colorA[4]=vec3[4](
			vec3(0,0.97,0.06),
			vec3(1.00,0.95,0),
			vec3(0.80,0.50,0.19),
			vec3(0.15,0.13,0.09)
			);

		const float maxHeight=7.0;//7=255*7 1500m (change in vertex shader as well)
		float height=100.0f*(origVertex.z/maxHeight)/255.0;//0-100
		height=clamp(height,0.0,99.999999);//0-0.99
		int texInd=int(height/25);
		float interpTex=mod(height,25.0);
		// texture
		outputF=mix(
			texture( tex_2DA, vec3(outUV.rg,texInd) ),
			texture( tex_2DA, vec3(outUV.rg,texInd+1) ),
			interpTex/25.0);
		if(terrain_vis_type==0){
			// texture color
			const float maxHeight=7.0;//7=255*7 1500m (change in vertex shader as well)
			float height=100.0f*(origVertex.z/maxHeight)/255.0;//0-100
			height=clamp(height,0.0,99.999999);//0-0.99
			int texInd=int(height/25);
			float interpTex=mod(height,25.0);
			// texture
			outputF=mix(
				texture( tex_2DA, vec3(outUV.rg,texInd) ),
				texture( tex_2DA, vec3(outUV.rg,texInd+1) ),
				interpTex/25.0);
		}else{
			if(terrain_vis_type==1){//line for big 3D
				// color lines
				float heightZ=origVertex.z;
				heightZ=mod(heightZ,40.0);
				outputF=heightZ<1.0||heightZ>39.0?vec4(mix(
					colorA[texInd],
					colorA[texInd+1],
					interpTex/25.0),1.0):vec4(1.0);
			}else{
				if(terrain_vis_type==2){//line for small 2D
					// color lines
					float heightZ=origVertex.z;
					heightZ=mod(heightZ,100.0);
					outputF=heightZ<5.0||heightZ>95.0?vec4(mix(
						colorA[texInd],
						colorA[texInd+1],
						interpTex/25.0),1.0):vec4(1.0);
					//varyingLightDirection=vec3(0,0,-1.0);
				}
			}
		}
		/*// color terrain
		outputF=vec4(mix(
			colorA[texInd],
			colorA[texInd+1],
			interpTex/25.0),1.0);*/
		// black lines
		/*float heightZ=origVertex.z;
		heightZ=mod(heightZ,40.0);
		outputF=heightZ<1.0||heightZ>39.0?vec4(0.0):vec4(1.0);*/
		
		/*float heightZ=origVertex.z;
		heightZ=mod(heightZ,40.0);
		outputF=heightZ<1.0||heightZ>39.0?vec4(mix(
			colorA[texInd],
			colorA[texInd+1],
			interpTex/25.0),1.0):vec4(1.0);*/
		
		
	}

	

	// LIGHTING
	vec4 ambientIllumination=vec4(0.05,0.05,0.05,0.0);
	vec4 diffuseIllumination=vec4(0.95,0.95,0.95,1.0);
	if(((mode&0x0200)==0x0200)||((mode&0xFF)==0x03)){//terran also gets lighting
		vec3 normal=varyingNormal;
		if(((mode&0x0FF)==0x05)||((mode&0xFF)==0x06))//seems that it needs it
			normal=normalize(normal);
		ambientIllumination = ambientColor*vec4(1.0,1.0,1.0,1.0);
		diffuseIllumination = (diffuseColor*vec4(1.0,1.0,1.0,1.0)) * max(0.0, dot(-lightDir, normal));
		//outputF=(ambientIllumination+diffuseIllumination)*outputF;
	}

	// WATER (simple lighting)
	if((mode&0xFF)==0x04){
		outputF=texture( tex0, outUV.rg );
		outputF=outputF* max(0.0, dot(-lightDir, vec3(0,0,1)));
		//outputF=computerSkyRay(origVertex.xzy,varyingNormal.xzy);
		/*ambientIllumination = ambientColor*vec4(1.0,1.0,1.0,1.0);
		diffuseIllumination = (diffuseColor*vec4(1.0,1.0,1.0,1.0)) * max(0.0, dot(-lightDir, varyingNormal));
		outputF=vec4(1);//(ambientIllumination+diffuseIllumination);*/
		
		//outputF=vec4(varyingNormal,1.0);
		//outputF=vec4(0,0,1,0);
		//return;
	}

	// SHADOW Disable
	if(shadowState==0){// 0 SHADOW Disable
		outputF=(ambientIllumination+diffuseIllumination)*outputF;
		return;
	}

	// SHADOW Render normal with shadows
	if(shadowState==1){// 1 SHADOW Render Normal
		float shadow_coef=0.95;
		shadow_coef= shadowCoef();
		outputF=(ambientIllumination+(shadow_coef+0.05)*diffuseIllumination)*outputF;
		return;
	}

}//

