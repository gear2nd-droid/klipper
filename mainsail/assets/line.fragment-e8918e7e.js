import{S as t}from"./Viewer-cb1d5b7c.js";import"./clipPlaneFragment-315cb900.js";import"./logDepthDeclaration-24838e5a.js";import"./logDepthFragment-3de3050a.js";import"./index-b9f0f775.js";import"./vuetify-f4a6879d.js";import"./overlayscrollbars-44d87bcf.js";import"./echarts-ff51454d.js";import"./codemirror-0a1db0c7.js";const e="linePixelShader",r="#include<clipPlaneFragmentDeclaration>\nuniform color: vec4f;\n#include<logDepthDeclaration>\n#define CUSTOM_FRAGMENT_DEFINITIONS\n@fragment\nfn main(input: FragmentInputs)->FragmentOutputs {\n#define CUSTOM_FRAGMENT_MAIN_BEGIN\n#include<logDepthFragment>\n#include<clipPlaneFragment>\nfragmentOutputs.color=uniforms.color;\n#define CUSTOM_FRAGMENT_MAIN_END\n}";t.ShadersStoreWGSL[e]=r;const d={name:e,shader:r};export{d as linePixelShaderWGSL};