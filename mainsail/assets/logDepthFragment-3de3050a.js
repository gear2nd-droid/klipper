import{S as t}from"./Viewer-cb1d5b7c.js";const e="logDepthFragment",r="#ifdef LOGARITHMICDEPTH\nfragmentOutputs.fragDepth=log2(fragmentInputs.vFragmentDepth)*uniforms.logarithmicDepthConstant*0.5;\n#endif\n";t.IncludesShadersStoreWGSL[e]=r;
