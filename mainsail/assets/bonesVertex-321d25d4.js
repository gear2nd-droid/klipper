import{S as e}from"./Viewer-cb1d5b7c.js";const t="bonesDeclaration",n="#if NUM_BONE_INFLUENCERS>0\nattribute matricesIndices : vec4<f32>;attribute matricesWeights : vec4<f32>;\n#if NUM_BONE_INFLUENCERS>4\nattribute matricesIndicesExtra : vec4<f32>;attribute matricesWeightsExtra : vec4<f32>;\n#endif\n#ifndef BAKED_VERTEX_ANIMATION_TEXTURE\n#ifdef BONETEXTURE\nvar boneSampler : texture_2d<f32>;uniform boneTextureWidth : f32;\n#else\nuniform mBones : array<mat4x4,BonesPerMesh>;\n#ifdef BONES_VELOCITY_ENABLED\nuniform mPreviousBones : array<mat4x4,BonesPerMesh>;\n#endif\n#endif\n#ifdef BONETEXTURE\nfn readMatrixFromRawSampler(smp : texture_2d<f32>,index : f32)->mat4x4<f32>\n{let offset=i32(index) *4; \nlet m0=textureLoad(smp,vec2<i32>(offset+0,0),0);let m1=textureLoad(smp,vec2<i32>(offset+1,0),0);let m2=textureLoad(smp,vec2<i32>(offset+2,0),0);let m3=textureLoad(smp,vec2<i32>(offset+3,0),0);return mat4x4<f32>(m0,m1,m2,m3);}\n#endif\n#endif\n#endif\n";e.IncludesShadersStoreWGSL[t]=n;const i="bonesVertex",r="#ifndef BAKED_VERTEX_ANIMATION_TEXTURE\n#if NUM_BONE_INFLUENCERS>0\nvar influence : mat4x4<f32>;\n#ifdef BONETEXTURE\ninfluence=readMatrixFromRawSampler(boneSampler,vertexInputs.matricesIndices[0])*vertexInputs.matricesWeights[0];\n#if NUM_BONE_INFLUENCERS>1\ninfluence=influence+readMatrixFromRawSampler(boneSampler,vertexInputs.matricesIndices[1])*vertexInputs.matricesWeights[1];\n#endif \n#if NUM_BONE_INFLUENCERS>2\ninfluence=influence+readMatrixFromRawSampler(boneSampler,vertexInputs.matricesIndices[2])*vertexInputs.matricesWeights[2];\n#endif \n#if NUM_BONE_INFLUENCERS>3\ninfluence=influence+readMatrixFromRawSampler(boneSampler,vertexInputs.matricesIndices[3])*vertexInputs.matricesWeights[3];\n#endif \n#if NUM_BONE_INFLUENCERS>4\ninfluence=influence+readMatrixFromRawSampler(boneSampler,vertexInputs.matricesIndicesExtra[0])*vertexInputs.matricesWeightsExtra[0];\n#endif \n#if NUM_BONE_INFLUENCERS>5\ninfluence=influence+readMatrixFromRawSampler(boneSampler,vertexInputs.matricesIndicesExtra[1])*vertexInputs.matricesWeightsExtra[1];\n#endif \n#if NUM_BONE_INFLUENCERS>6\ninfluence=influence+readMatrixFromRawSampler(boneSampler,vertexInputs.matricesIndicesExtra[2])*vertexInputs.matricesWeightsExtra[2];\n#endif \n#if NUM_BONE_INFLUENCERS>7\ninfluence=influence+readMatrixFromRawSampler(boneSampler,vertexInputs.matricesIndicesExtra[3])*vertexInputs.matricesWeightsExtra[3];\n#endif \n#else \ninfluence=uniforms.mBones[int(vertexInputs.matricesIndices[0])]*vertexInputs.matricesWeights[0];\n#if NUM_BONE_INFLUENCERS>1\ninfluence=influence+uniforms.mBones[int(vertexInputs.matricesIndices[1])]*vertexInputs.matricesWeights[1];\n#endif \n#if NUM_BONE_INFLUENCERS>2\ninfluence=influence+uniforms.mBones[int(vertexInputs.matricesIndices[2])]*vertexInputs.matricesWeights[2];\n#endif \n#if NUM_BONE_INFLUENCERS>3\ninfluence=influence+uniforms.mBones[int(vertexInputs.matricesIndices[3])]*vertexInputs.matricesWeights[3];\n#endif \n#if NUM_BONE_INFLUENCERS>4\ninfluence=influence+uniforms.mBones[int(vertexInputs.matricesIndicesExtra[0])]*vertexInputs.matricesWeightsExtra[0];\n#endif \n#if NUM_BONE_INFLUENCERS>5\ninfluence=influence+uniforms.mBones[int(vertexInputs.matricesIndicesExtra[1])]*vertexInputs.matricesWeightsExtra[1];\n#endif \n#if NUM_BONE_INFLUENCERS>6\ninfluence=influence+uniforms.mBones[int(vertexInputs.matricesIndicesExtra[2])]*vertexInputs.matricesWeightsExtra[2];\n#endif \n#if NUM_BONE_INFLUENCERS>7\ninfluence=influence+uniforms.mBones[int(vertexInputs.matricesIndicesExtra[3])]*vertexInputs.matricesWeightsExtra[3];\n#endif \n#endif\nfinalWorld=finalWorld*influence;\n#endif\n#endif\n";e.IncludesShadersStoreWGSL[i]=r;
