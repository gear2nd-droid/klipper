import{c as r}from"./Viewer-cb1d5b7c.js";import{DDSTools as n}from"./dds-34bdd6d8.js";import"./index-b9f0f775.js";import"./vuetify-f4a6879d.js";import"./overlayscrollbars-44d87bcf.js";import"./echarts-ff51454d.js";import"./codemirror-0a1db0c7.js";import"./cubemapToSphericalPolynomial-0f5d0d00.js";class w{constructor(){this.supportCascades=!0}loadCubeData(e,a,l,o){const p=a.getEngine();let i,m=!1,t=1e3;if(Array.isArray(e))for(let s=0;s<e.length;s++){const d=e[s];i=n.GetDDSInfo(d),a.width=i.width,a.height=i.height,m=(i.isRGB||i.isLuminance||i.mipmapCount>1)&&a.generateMipMaps,p._unpackFlipY(i.isCompressed),n.UploadDDSLevels(p,a,d,i,m,6,-1,s),!i.isFourCC&&i.mipmapCount===1?p.generateMipMapsForCubemap(a):t=i.mipmapCount-1}else{const s=e;i=n.GetDDSInfo(s),a.width=i.width,a.height=i.height,l&&(i.sphericalPolynomial=new r),m=(i.isRGB||i.isLuminance||i.mipmapCount>1)&&a.generateMipMaps,p._unpackFlipY(i.isCompressed),n.UploadDDSLevels(p,a,s,i,m,6),!i.isFourCC&&i.mipmapCount===1?p.generateMipMapsForCubemap(a,!1):t=i.mipmapCount-1}p._setCubeMapTextureParams(a,m,t),a.isReady=!0,a.onLoadedObservable.notifyObservers(a),a.onLoadedObservable.clear(),o&&o({isDDS:!0,width:a.width,info:i,data:e,texture:a})}loadData(e,a,l){const o=n.GetDDSInfo(e),p=(o.isRGB||o.isLuminance||o.mipmapCount>1)&&a.generateMipMaps&&Math.max(o.width,o.height)>>o.mipmapCount-1===1;l(o.width,o.height,p,o.isFourCC,()=>{n.UploadDDSLevels(a.getEngine(),a,e,o,p,1)})}}export{w as _DDSTextureLoader};