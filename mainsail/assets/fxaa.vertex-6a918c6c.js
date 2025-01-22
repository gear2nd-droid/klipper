import{S as r}from"./Viewer-cb1d5b7c.js";import"./index-b9f0f775.js";import"./vuetify-f4a6879d.js";import"./overlayscrollbars-44d87bcf.js";import"./echarts-ff51454d.js";import"./codemirror-0a1db0c7.js";const e="fxaaVertexShader",o="attribute vec2 position;uniform vec2 texelSize;varying vec2 vUV;varying vec2 sampleCoordS;varying vec2 sampleCoordE;varying vec2 sampleCoordN;varying vec2 sampleCoordW;varying vec2 sampleCoordNW;varying vec2 sampleCoordSE;varying vec2 sampleCoordNE;varying vec2 sampleCoordSW;const vec2 madd=vec2(0.5,0.5);\n#define CUSTOM_VERTEX_DEFINITIONS\nvoid main(void) {\n#define CUSTOM_VERTEX_MAIN_BEGIN\nvUV=(position*madd+madd);sampleCoordS=vUV+vec2( 0.0,1.0)*texelSize;sampleCoordE=vUV+vec2( 1.0,0.0)*texelSize;sampleCoordN=vUV+vec2( 0.0,-1.0)*texelSize;sampleCoordW=vUV+vec2(-1.0,0.0)*texelSize;sampleCoordNW=vUV+vec2(-1.0,-1.0)*texelSize;sampleCoordSE=vUV+vec2( 1.0,1.0)*texelSize;sampleCoordNE=vUV+vec2( 1.0,-1.0)*texelSize;sampleCoordSW=vUV+vec2(-1.0,1.0)*texelSize;gl_Position=vec4(position,0.0,1.0);\n#define CUSTOM_VERTEX_MAIN_END\n}";r.ShadersStore[e]=o;const s={name:e,shader:o};export{s as fxaaVertexShader};
