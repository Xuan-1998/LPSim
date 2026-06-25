#version 420

layout(location = 0)in vec3 vertex;
layout(location = 1)in vec3 color;
layout(location = 2)in vec3 normal;
layout(location = 3)in vec3 uv;


out vec3 outColor;
out vec3 outUV;
out vec3 origVertex;// L

out vec3 varyingNormal;

// UNIFORM
uniform int mode;

// MODE 1--> color
// MODE 2--> texture
// MODE 3--> terrain
// MODE 4--> water
// MODE 5--> model obj: one color
// MODE 6--> model obj: texture
// MODE 11--> 2D Render
// MODE 12--> 2D Render (iso terrain)
// MODE 13--> 3D Temperature/Pression
// MODE 14--> 2D 0-1 Color Render

//0x0100 --> adapt terrain
//0x0200 --> lighting

uniform mat4 mvpMatrix;
uniform mat4 mvMatrix;
uniform mat3 normalMatrix;

uniform mat4 light_mvpMatrix;
uniform mat4 light_biasMatrix;	//transfrom to [0,1] tex coordinates

uniform int shadowState;
// 0 DISABLE
// 1 SHADOW Render Normal
// 2 SHADOW Render from light

// terrain
uniform vec4 terrain_size;
uniform sampler2D terrain_tex;

// model
uniform vec3 justOneColor;
uniform mat4 modelTransf;

void main(){
	
	outColor=color;
	outUV=uv;
	origVertex=vertex;

	// 0. 2D TEX RENDER
	if((mode&0x0FF)==11||(mode&0x0FF)==12){
		outUV= vec3(0.5)*vertex+vec3(0.5);
		gl_Position = vec4(vertex,1.0);
		return;
	}
	// 0. 2D COLOR RENDER
	if((mode&0x0FF)==14){
		//outUV=vertex;
		gl_Position=vec4(vec3(2.0)*vertex+vec3(-1.0),1.0);//vertex 0-1--> Out -1->1 (screen coord)
		return;
	}

	// 1. TRANSFORM MODEL
	if(((mode&0x0FF)==0x05)||((mode&0xFF)==0x06)){
		outColor=justOneColor;
		origVertex=origVertex.xzy;//change model Y<->Z
		origVertex=(modelTransf*vec4(origVertex,1.0)).xyz;//note 1.0

	}
	// 2. ADAPT TO TERRAIN
	if(((mode&0xFF)==0x03)||((mode&0x0100)==0x0100)){// terrain or adapt to terrain
		const float maxHeight=7.0;//7=255*7 1785m (change in fragment as well)
		vec2 terrainTexCoord=vec2(
			(origVertex.x-terrain_size.x)/terrain_size.z,
			(origVertex.y-terrain_size.y)/terrain_size.w
			);
		//float height=255.0f*length(texture(terrain_tex,terrainTexCoord.rg));
		float height=maxHeight*255.0f*texture(terrain_tex,terrainTexCoord.rg).r;
		origVertex.z+=height;
		//if(height<15.0f)//water height
		//	origVertex.z=-100.0f;
		if((mode&0xFF)==0x03){// terrain
			// computer normal from heightmap
			const vec2 size = vec2(1.0,0.0);
			const ivec3 off = ivec3(-1,0,1);

			float s01 = textureOffset(terrain_tex, terrainTexCoord.rg, off.xy).r;
			float s21 = textureOffset(terrain_tex, terrainTexCoord.rg, off.zy).r;
			float s10 = textureOffset(terrain_tex, terrainTexCoord.rg, off.yx).r;
			float s12 = textureOffset(terrain_tex, terrainTexCoord.rg, off.yz).r;
			vec3 va = normalize(vec3(size.xy,10*(s21-s01)));
			vec3 vb = normalize(vec3(size.yx,10*(s12-s10)));
			//vec3 va = normalize(vec3(size.x,s21-s01,size.y));
			//vec3 vb = normalize(vec3(size.y,s12-s10,size.x));

			varyingNormal=cross(va,vb);
			//varyingNormal=10*vec3(s21-s01,s21-s01,s21-s01);
			//varyingNormal=vec3(s12-s10,s12-s10,s12-s10);
			//varyingNormal=va;

		}
	}

	// SHADOW: From light
	if(shadowState==2){
		gl_Position = light_mvpMatrix * vec4(origVertex,1.0);
		return;
	}

	// WATER
	if((mode&0xFF)==0x04){
		vec3 u = normalize( vec3(mvMatrix * vec4(origVertex,1.0)) );
		vec3 n = normalize( normalMatrix * normal );
		vec3 r = reflect( u, n );
		float m = 2.0 * sqrt( r.x*r.x + r.y*r.y + (r.z+1.0)*(r.z+1.0) );
		m*=2.0;// NACHO
		outUV.s = r.x/m + 0.5;
		outUV.t = r.y/m + 0.5;
		/*//varyingNormal=-r;//reflect ray (for sky)*/
		//varyingNormal=normalize(normal);
	}

	// LIGHTING
	if((mode&0x0200)==0x0200){
		//varyingNormal=normalMatrix*normal;
		varyingNormal=normalize(normal);//here to avoid doing it in the fragment
		// TRANSFORM MODEL (it should change its normal too)
		if(((mode&0x0FF)==0x05)||((mode&0xFF)==0x06)){
			varyingNormal=normal.xzy;
			varyingNormal=(modelTransf*vec4(varyingNormal,0.0)).xyz;//note 0.0
		}
	}

	gl_Position = mvpMatrix * vec4(origVertex,1.0);

}