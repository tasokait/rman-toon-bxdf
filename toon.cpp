#include "RixBxdf.h"
#include "RixShading.h"
#include "RixShadingBuiltin.h"
#include "RixShadingUtils.h"
#include "RixIntegrator.h"

struct ToonInstanceData {
    int sssEnabled;
    int sssSI;
    int sssinv;
    int lineEnabled;
    int silhouetteEnabled;
    float sssdfswitch;
    float sssbias;
    float IRETA;
    float unit;
    float ior = -1;
    bool nbconnected = false;
};
class ToonBxdf : public RixBxdf {
public:
    ToonBxdf(RixShadingContext const* sCtx, RixBxdfFactory* factory, int sssEnabled, int lineEnabled, int silhouetteEnabled,bool isnb)
        : RixBxdf(sCtx, factory),
          m_numPts(sCtx->numPts),
          m_sssEnabled(sssEnabled),
          m_lineEnabled(lineEnabled),
          m_silhouetteEnabled(silhouetteEnabled){
            sCtx->EvalParam(0, -1, &m_baseColor, nullptr, true);
            sCtx->EvalParam(1, -1, &m_shadowColor, nullptr, true);
            sCtx->EvalParam(2, -1, &m_shadowLevels, nullptr, true);
            sCtx->EvalParam(3, -1, &m_shadowOpacity, nullptr, true);
            sCtx->EvalParam(5, -1, &m_sssColor, nullptr, true);
            sCtx->EvalParam(6, -1, &m_sssopas, nullptr, true);
            sCtx->EvalParam(7, -1, &m_ior, nullptr, true);
            sCtx->EvalParam(16, -1, &m_linedegree, nullptr, true);
            sCtx->EvalParam(17, -1, &m_linecolor, nullptr, true);
            sCtx->EvalParam(19, -1, &m_silamount, nullptr, true);
            sCtx->EvalParam(20, -1, &m_minsilamount, nullptr, true);
            sCtx->GetBuiltinVar(RixShadingContext::BuiltinVar::k_Ngn, &Ngn);
            sCtx->GetBuiltinVar(RixShadingContext::BuiltinVar::k_Vn, &m_vn);
            sCtx->GetBuiltinVar(RixShadingContext::BuiltinVar::k_Tn, &m_Tn);
            if(isnb){
                sCtx->EvalParam(21, -1, &ShN, nullptr, true);
                return;
            }
            sCtx->GetBuiltinVar(RixShadingContext::BuiltinVar::k_Nn, &ShN);
        }

    bool EmitLocal(RtColorRGB* result) override {return false;}
    void EvaluateSample(RixBXTransportTrait transportTrait,
                        RixBXLobeTraits const* lobesWanted,
                        RixRNG* rng,
                        RixBXLobeTraits* lobesEvaluated,
                        RtVector3 const* directions,
                        RixBXLobeWeights& weights,
                        float* forwardPdfs,
                        float* reversePdfs) override {

        for (int i = 0; i < m_numPts; i++) {
            if(!lobesWanted[i].HasAnyDiffSpec()){
                lobesEvaluated[i].SetValid(false);
                continue;
            }
            bool isnsss = (shadingCtx->transmission[i].r == 1 && shadingCtx->transmission[i].g == 1 && shadingCtx->transmission[i].b == 1) || (m_sssopas[i] == 0 || !m_sssEnabled);
            RixBXLobeSampled sampled = RixBXLobeSampled(false, false, true, false, 0, (isnsss)?0:k_RixBXSubsurfaceLobeId1);
            RtColorRGB* diffuseWeight = weights.AddActiveLobe(sampled);
            float NdotL = ShN[i].Dot(directions[i]);
            NdotL = (NdotL)/(m_shadowOpacity[i]+((1-m_shadowOpacity[i])*NdotL));
            float q = Quantize(NdotL,i);
            float PIcal = (q / M_PI)/(m_shadowOpacity[i]+((1-m_shadowOpacity[i])*(q / M_PI)));
            RtColorRGB color = Lerp(q,i);
            float totalfactor = ((m_lineEnabled && ((acosf(std::max(-1.0f, std::min(1.0f, ShN[i].Dot(Ngn[i])))) * (180.0f / M_PI)) > m_linedegree[i]))||(m_silhouetteEnabled && m_vn[i].Dot(ShN[i]) < m_silamount[i] && m_vn[i].Dot(ShN[i]) > m_minsilamount[i])) ? 1.0f : 0.0f;
            forwardPdfs[i] = PIcal;
            reversePdfs[i] = PIcal;
            lobesEvaluated[i] = RixBXLobeTraits(sampled);
            if(!isnsss){
                lobesEvaluated[i].SetTransmit(true);
                diffuseWeight[i] = ((((1-totalfactor)*color)+(totalfactor*m_linecolor[i])) * (shadingCtx->transmission[i]/(m_sssopas[i] + ((1-m_sssopas[i])*shadingCtx->transmission[i]))) + ((((m_sssColor[i]*(1-q))*(1-totalfactor))+(totalfactor * m_linecolor[i]))*((1-shadingCtx->transmission[i]) * m_sssopas[i])))*PIcal;
                continue;
            }
            diffuseWeight[i] = (((1-totalfactor)*color)+(totalfactor*m_linecolor[i])) * PIcal;
        }

    }
    void EvaluateSamplesAtIndex(RixBXTransportTrait transportTrait,
                                RixBXLobeTraits const& lobesWanted,
                                RixRNG* rng,
                                int index,
                                int numSamples,
                                RixBXLobeTraits* lobesEvaluated,
                                RtVector3 const* directions,
                                RixBXLobeWeights& weights,
                                float* forwardPdfs,
                                float* reversePdfs) override {
        if(!lobesWanted.HasAnyDiffSpec()){
            for (int i = 0; i < numSamples; i++){
                lobesEvaluated[i].SetValid(false);
            }
            return;
        }
        bool isnsss = (shadingCtx->transmission[index].r == 1 && shadingCtx->transmission[index].g == 1 && shadingCtx->transmission[index].b == 1) || (m_sssopas[index] == 0 || !m_sssEnabled);
        float NdotL = ShN[index].Dot(directions[index]);
        NdotL = NdotL/(m_shadowOpacity[index]+((1-m_shadowOpacity[index])*NdotL));
        float q = Quantize(NdotL,index);
        float PIcal = (q / M_PI)/(m_shadowOpacity[index]+((1-m_shadowOpacity[index])*(q / M_PI)));
        RtColorRGB color = Lerp(q,index);
        float totalfactor = ((m_lineEnabled && ((acosf(std::max(-1.0f, std::min(1.0f, ShN[index].Dot(Ngn[index])))) * (180.0f / M_PI)) > m_linedegree[index]))||(m_silhouetteEnabled && m_vn[index].Dot(ShN[index]) < m_silamount[index] && m_vn[index].Dot(ShN[index]) > m_minsilamount[index])) ? 1.0f : 0.0f;
        RixBXLobeSampled sampled = RixBXLobeSampled(false, false, true, false, 0, (isnsss)?0:k_RixBXSubsurfaceLobeId1);
        RtColorRGB* diffuseWeight = weights.AddActiveLobe(sampled);
        RixBXLobeTraits lobesEvaluatedop = RixBXLobeTraits(sampled);
        RtColorRGB diffusecolor = isnsss?((1-totalfactor)*color)+(totalfactor*m_linecolor[index]):(((((1-totalfactor)*color)+(totalfactor*m_linecolor[index])) * (shadingCtx->transmission[index]/(m_sssopas[index] + ((1-m_sssopas[index])*shadingCtx->transmission[index])))) + ((((m_sssColor[index]*(1-q))*(1-totalfactor))+(totalfactor * m_linecolor[index]))*((1-shadingCtx->transmission[index]) * m_sssopas[index])))*PIcal;
        for (int i = 0; i < numSamples; i++) {
            forwardPdfs[i] = PIcal;
            reversePdfs[i] = PIcal;
            diffuseWeight[i] = diffusecolor;
            lobesEvaluated[i] = lobesEvaluatedop;
            if(!isnsss){
                lobesEvaluated[i].SetTransmit(true);
            }
        }
    }
    void GenerateSample(RixBXTransportTrait transportTrait,
                        RixBXLobeTraits const* lobesWanted,
                        RixRNG* rng,
                        RixBXLobeSampled* lobesSampled,
                        RtVector3* directions,
                        RixBXLobeWeights& weights,
                        float* forwardPdfs,
                        float* reversePdfs,
                        RtColorRGB* compTrans) override {
        for (int i = 0; i < m_numPts; i++) {
            if(!lobesWanted[i].HasAnyDiffSpec()){
                lobesSampled[i].SetValid(false);
                continue;
            }
            
            bool ifsss = m_sssEnabled && ifsssen && m_sssopas[i] != 0;
            RtColorRGB* diffuseWeight = weights.AddActiveLobe(RixBXLobeSampled(false, false, true, false, 0, (ifsss)?k_RixBXSubsurfaceLobeId1:0));
            RtFloat2 rnged = rng->GenerateSample2D(i);
            float phi = 2.f * M_PI * rnged.x;
            float cosTheta = sqrtf(rnged.y);
            float sinTheta = sqrtf(fmax(0.f,1.0f - (cosTheta*cosTheta)));
            RtVector3 TX, TY;
            RixComputeShadingBasis(ShN[i], m_Tn[i], TX, TY);
            RtVector3 localDir(sinTheta * cosf(phi),sinTheta * sinf(phi),cosTheta);
            directions[i] = (m_shadowOpacity[i]*(localDir.x * TX + localDir.y * TY + localDir.z * ShN[i])) + ((1 - m_shadowOpacity[i])*localDir);
            directions[i].Normalize();

            float NdotL = ShN[i].Dot(directions[i]);
            NdotL = (NdotL)/(m_shadowOpacity[i]+((1-m_shadowOpacity[i])*NdotL));
            float q = Quantize(NdotL,i);
            float PIcal = (q / M_PI)/(m_shadowOpacity[i]+((1-m_shadowOpacity[i])*(q / M_PI)));
            RtColorRGB color = Lerp(q,i);
            float totalfactor = ((m_lineEnabled && ((acosf(std::max(-1.0f, std::min(1.0f, ShN[i].Dot(Ngn[i])))) * (180.0f / M_PI)) > m_linedegree[i]))||(m_silhouetteEnabled && m_vn[i].Dot(ShN[i]) < m_silamount[i] && m_vn[i].Dot(ShN[i]) > m_minsilamount[i])) ? 1.0f : 0.0f;
            forwardPdfs[i] = PIcal;
            reversePdfs[i] = PIcal;
            lobesSampled[i].Set(false, false, true, false, false, 0, (ifsss)?k_RixBXSubsurfaceLobeId1:0);
            if(ifsss){
                lobesSampled[i].SetTransmit(true);
                diffuseWeight[i]= (((m_sssColor[i]*(1-q))*((1 - shadingCtx->transmission[i])*m_sssopas[i])) + ((((1-totalfactor)*color)+(totalfactor*m_linecolor[i])) * (shadingCtx->transmission[i]/(m_sssopas[i] + ((1-m_sssopas[i])*shadingCtx->transmission[i]))))) * PIcal;
                continue;
            }
            diffuseWeight[i] = (((1-totalfactor)*color)+(totalfactor*m_linecolor[i])) * PIcal;
        }
        ifsssen = !ifsssen;
    }
    void GetAggregateLobeTraits(RixBXLobeTraits* t) override {
        *t = k_RixBXTraitsAllLobe;
    }
    RixBXEvaluateDomain GetEvaluateDomain() override {
        return k_RixBXBoth;
    }
    RixSCDetail GetProperty(BxdfProperty property, void const** result) const {
        static float ior = 1.0f;
        switch (property) {
            case k_Albedo:
                return k_RixSCVarying;
            case k_MaterialIor:
                if(m_sssEnabled){
                    *result = &m_ior;
                    return k_RixSCVarying;
                }
                *result = &ior;
                return k_RixSCUniform;
            default:
                return k_RixSCInvalidDetail;
        }
    }
private:
    RtColorRGB const *m_sssColor, *m_baseColor, *m_shadowColor,*m_linecolor;
    int m_numPts, m_sssEnabled, m_lineEnabled, m_silhouetteEnabled;
    float const *m_shadowOpacity,*m_shadowLevels,*m_ior,*m_linedegree,*m_silamount, *m_minsilamount, *m_sssopas;
    RtVector3 const* ShN,*m_Tn,*Ngn,*m_vn;
    bool ifsssen = false;
    float Quantize(float x, int i) const {
        if (std::fmax(1.f,m_shadowLevels[i]) <= 1) return x;  // No quantization if levels <= 1
        return floorf(x * m_shadowLevels[i]) / m_shadowLevels[i];
    }
    RtColorRGB Lerp(float t,int i) const {
        return ((m_shadowColor[i] * (1.0f - t))*m_shadowOpacity[i]) + (m_baseColor[i] * (t/(m_shadowOpacity[i]+((1-m_shadowOpacity[i])*t))));
    }
};

class ToonBxdfFactory : public RixBxdfFactory {
public:
    ToonBxdfFactory() {}
    ~ToonBxdfFactory() override {}
    int Init(RixContext& ctx, RtUString const pluginPath) override {return 0;}
    void Finalize(RixContext& ctx) override {}
    RixSCParamInfo const* GetParamTable() override {
        static RixSCParamInfo s_ptable[] = {
            RixSCParamInfo(RtUString("baseColor"), k_RixSCColor, k_RixSCInput),
            RixSCParamInfo(RtUString("shadowColor"), k_RixSCColor, k_RixSCInput),
            RixSCParamInfo(RtUString("shadowLevels"), k_RixSCFloat, k_RixSCInput),
            RixSCParamInfo(RtUString("shadowOpacity"), k_RixSCFloat, k_RixSCInput),
            RixSCParamInfo(RtUString("sssEnabled"), k_RixSCInteger, k_RixSCInput),
            RixSCParamInfo(RtUString("sssColor"), k_RixSCColor, k_RixSCInput),
            //RixSCParamInfo(RtUString("ssslevel"), k_RixSCFloat, k_RixSCInput),
            RixSCParamInfo(RtUString("sssopacity"), k_RixSCFloat, k_RixSCInput),
            RixSCParamInfo(RtUString("sssior"), k_RixSCFloat, k_RixSCInput),
            RixSCParamInfo(RtUString("sssanisotropy"), k_RixSCFloat, k_RixSCInput),
            RixSCParamInfo(RtUString("sssdiffuseSwitch"), k_RixSCFloat, k_RixSCInput),
            RixSCParamInfo(RtUString("sssbias"), k_RixSCFloat, k_RixSCInput),
            RixSCParamInfo(RtUString("sssresolveSI"), k_RixSCInteger, k_RixSCInput),
            RixSCParamInfo(RtUString("sssinternalRETA"), k_RixSCFloat, k_RixSCInput),
            RixSCParamInfo(RtUString("unitLength"), k_RixSCFloat, k_RixSCInput),
            RixSCParamInfo(RtUString("sssdbl"), k_RixSCInteger, k_RixSCInput),
            RixSCParamInfo(RtUString("lineEnabled"), k_RixSCInteger, k_RixSCInput),
            RixSCParamInfo(RtUString("linesharpdegree"), k_RixSCFloat, k_RixSCInput),
            RixSCParamInfo(RtUString("linecolor"), k_RixSCColor, k_RixSCInput),
            RixSCParamInfo(RtUString("silhouetteEnabled"), k_RixSCInteger, k_RixSCInput),
            RixSCParamInfo(RtUString("silhouetteAM"), k_RixSCFloat, k_RixSCInput),
            RixSCParamInfo(RtUString("minsilhouetteAM"), k_RixSCFloat, k_RixSCInput),
            RixSCParamInfo(RtUString("bumpNormal"), k_RixSCNormal, k_RixSCInput),
            RixSCParamInfo()
        };
        return s_ptable;
    }
    void Synchronize(RixContext& ctx, RixSCSyncMsg syncMsg, RixParameterList const* syncParams) override {}
    RixVolumeIntegrator* BeginInterior(RixShadingContext const*, RixSCShadingMode, void*, void*) override {return nullptr;}
    RixOpacity* BeginOpacity(RixShadingContext const*, RixSCShadingMode, void*, void*) override {
        return nullptr;}
    RixBxdf* BeginScatter(RixShadingContext const* sCtx, RixBXLobeTraits const& hints, RixSCShadingMode,void*, void* instanceData) override {
        ToonInstanceData* data = static_cast<ToonInstanceData*>(instanceData);
        return new ToonBxdf(sCtx, this, data->sssEnabled, data->lineEnabled, data->silhouetteEnabled,data->nbconnected);
    }
    RixVolumeIntegrator* BeginSubsurface(RixShadingContext const* sCtx, RixSCShadingMode mode, void* parentData, void* instanceData) override {
        ToonInstanceData* data = static_cast<ToonInstanceData*>(instanceData);
        if (data->sssEnabled) {
            RixShadingContext::Allocator pool(sCtx);
            void *mem = pool.AllocForVolume<RixSSPathTraced::Params>(1);
            RixSSPathTraced::Params *ssParams = new (mem) RixSSPathTraced::Params();
            sCtx->EvalParam(0, -1, &ssParams->albedo, nullptr, true);
            sCtx->EvalParam(5, -1, &ssParams->dmfp, nullptr, true);
            sCtx->EvalParam(7, -1, &ssParams->ior, nullptr, true);
            sCtx->EvalParam(8, -1, &ssParams->g, nullptr, true);
            ssParams->albedoDetail = k_RixSCVarying;
            ssParams->dmfpDetail = k_RixSCVarying;
            ssParams->iorDetail = k_RixSCVarying;
            ssParams->gDetail = k_RixSCVarying;
            ssParams->diffuseSwitch = data->sssdfswitch;
            ssParams->bias = data->sssbias;
            ssParams->internalReflectionsEta = data->IRETA;
            ssParams->unitLength = data->unit;
            ssParams->resolveSelfIntersections = true;
            ssParams->disableAlbedoInversion = data->sssinv;
            data->ior = ssParams->ior[0];
            if(data->nbconnected){
                sCtx->EvalParam(21, -1, &ssParams->bumpNormal, nullptr, true);
            }
            RixVolumeIntegrator* v = sCtx->GetBuiltinVolume(RixShadingContext::k_SSPathTraced, this, instanceData);
            v->SetParameters(ssParams);
            return v;
        }
        return nullptr;
    }
    void EndInterior(RixVolumeIntegrator*) override {}
    void EndOpacity(RixOpacity*) override {}
    void EndScatter(RixBxdf* bxdf) override {delete bxdf;}
    void EndSubsurface(RixVolumeIntegrator* vl) override {//delete vl;
    }
    float GetIndexOfRefraction(void* instanceData) const override {
        ToonInstanceData* data = static_cast<ToonInstanceData*>(instanceData);
        if (data->sssEnabled && data->ior != -1){
            return data->ior;
        }
        return 1.0f;
    }
    int GetInstanceHints(void* instanceData) const {
        ToonInstanceData* data = static_cast<ToonInstanceData*>(instanceData);
        return data->sssEnabled ? k_ComputesSubsurface|k_TriviallyOpaque : 0;
    }
    void RegisterTemporalVolumeParams(void*, std::vector<int>&) const override {}
    void CreateInstanceData(RixContext& rixCtx, RtUString const handle, RixParameterList const* parameterList, InstanceData* instanceData) override {
        ToonInstanceData* data = new ToonInstanceData();
        parameterList->EvalParam(4, 0, &data->sssEnabled);
        parameterList->EvalParam(7, 0, &data->ior);
        parameterList->EvalParam(9, 0, &data->sssdfswitch);
        parameterList->EvalParam(10, 0, &data->sssbias);
        parameterList->EvalParam(11, 0, &data->sssSI);
        parameterList->EvalParam(12, 0, &data->IRETA);
        parameterList->EvalParam(13, 0, &data->unit);
        parameterList->EvalParam(14, 0, &data->sssinv);
        parameterList->EvalParam(15, 0, &data->lineEnabled);
        parameterList->EvalParam(18, 0, &data->silhouetteEnabled);
        RixSCType dumptype;
        parameterList->GetParamInfo(21,&dumptype,&data->nbconnected);
        instanceData->data = data;
        instanceData->datalen = sizeof(data);
        instanceData->freefunc = [](void* ptr) { delete static_cast<ToonInstanceData*>(ptr);};
    }
    void SynchronizeInstanceData(RixContext& rixCtx, RtUString const handle,RixParameterList const* instanceParams,uint32_t const editHints,InstanceData* instanceData){}
    void DestroyInstanceData(void* instanceData) {delete static_cast<ToonInstanceData*>(instanceData);}};
RIX_BXDFPLUGINCREATE {return new ToonBxdfFactory();}
RIX_BXDFPLUGINDESTROY{delete ((ToonBxdfFactory*)bxdf);}
