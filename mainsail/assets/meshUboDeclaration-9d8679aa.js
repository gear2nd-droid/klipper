import{S as o}from"./Viewer-cb1d5b7c.js";const e="sceneUboDeclaration",i="layout(std140,column_major) uniform;uniform Scene {mat4 viewProjection;\n#ifdef MULTIVIEW\nmat4 viewProjectionR;\n#endif \nmat4 view;mat4 projection;vec4 vEyePosition;};\n";o.IncludesShadersStore[e]=i;const t="meshUboDeclaration",n="#ifdef WEBGL2\nuniform mat4 world;uniform float visibility;\n#else\nlayout(std140,column_major) uniform;uniform Mesh\n{mat4 world;float visibility;};\n#endif\n#define WORLD_UBO\n";o.IncludesShadersStore[t]=n;
