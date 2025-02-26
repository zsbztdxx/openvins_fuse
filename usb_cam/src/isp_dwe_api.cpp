#include "isp_dwe/isp_dwe_api.h"

namespace usb_cam
{

static inline int xioctl(int fd, int request, void *arg)
{
    int r;

    do
        r = ioctl(fd, request, arg);
    while (-1 == r && EINTR == errno);

    return r;
}

ISP_API::ISP_API(std::string fileName)
{
    fd_isp = open(fileName.c_str(), O_RDWR, 0);
}

ISP_API::~ISP_API()
{
    if (fd_isp >= 0)
    {
        close(fd_isp);
    }
}

int ISP_API::Reset(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_RESET, nullptr);
    }
    return -1;
}

int ISP_API::Set_Input(struct isp_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_INPUT, context);
    }
    return -1;
}

int ISP_API::Enable(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_ENABLE, nullptr);
    }
    return -1;
}

int ISP_API::Disable(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_DISABLE, nullptr);
    }
    return -1;
}

int ISP_API::Is_Enable(bool *enable)
{
    if (fd_isp >= 0 && enable != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_ISP_STATUS, enable);
    }
    return -1;
}

int ISP_API::Start_Stream(uint32_t *framenum)
{
    if (fd_isp >= 0 && framenum != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_START_CAPTURE, framenum);
    }
    return -1;
}

int ISP_API::Stop_Stream(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_ISP_STOP, nullptr);
    }
    return -1;
}

int ISP_API::IOC_Disable_ISP_Off(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_DISABLE_ISP_OFF, nullptr);
    }
    return -1;
}

int ISP_API::Set_Buffer(struct isp_buffer_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_SET_BUFFER, context);
    }
    return -1;
}

int ISP_API::Set_Bp_Buffer(struct isp_bp_buffer_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_SET_BP_BUFFER, context);
    }
    return -1;
}

int ISP_API::IOC_Start_Dma_Read(struct isp_dma_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_START_DMA_READ, context);
    }
    return -1;
}

int ISP_API::MI_Start(struct isp_mi_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_MI_START, context);
    }
    return -1;
}

int ISP_API::MI_Stop(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_MI_STOP, nullptr);
    }
    return -1;
}

int ISP_API::Enable_TPG(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_ENABLE_TPG, nullptr);
    }
    return -1;
}

int ISP_API::Disable_TPG(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_DISABLE_TPG, nullptr);
    }
    return -1;
}

int ISP_API::Set_TPG(struct isp_tpg_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_TPG, context);
    }
    return -1;
}

int ISP_API::Set_MCM(struct isp_mcm_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_MCM, context);
    }
    return -1;
}

int ISP_API::Enable_BLS(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_ENABLE_BLS, nullptr);
    }
    return -1;
}

int ISP_API::Disable_BLS(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_DISABLE_BLS, nullptr);
    }
    return -1;
}

int ISP_API::Set_BLS(struct isp_bls_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_BLS, context);
    }
    return -1;
}

int ISP_API::Set_MUX(struct isp_mux_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_MUX, context);
    }
    return -1;
}

int ISP_API::Enable_AWB(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_ENABLE_AWB, nullptr);
    }
    return -1;
}

int ISP_API::Disable_AWB(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_DISABLE_AWB, nullptr);
    }
    return -1;
}

int ISP_API::Set_AWB(struct isp_awb_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_AWB, context);
    }
    return -1;
}

int ISP_API::Get_AWB_Mean(struct isp_awb_mean *mean)
{
    if (fd_isp >= 0 && mean != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_G_AWBMEAN, mean);
    }
    return -1;
}

int ISP_API::Set_IS(struct isp_is_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_IS, context);
    }
    return -1;
}

int ISP_API::Set_Raw_IS(struct isp_is_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_RAW_IS, context);
    }
    return -1;
}

int ISP_API::Set_CNR(struct isp_cnr_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_CNR, context);
    }
    return -1;
}

int ISP_API::Set_CC(struct isp_cc_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_CC, context);
    }
    return -1;
}

int ISP_API::Set_Xtalk(struct isp_xtalk_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_XTALK, context);
    }
    return -1;
}

int ISP_API::Set_Gamma_Out(struct isp_gamma_out_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_GAMMA_OUT, context);
    }
    return -1;
}

int ISP_API::Enable_LSC(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_ENABLE_LSC, nullptr);
    }
    return -1;
}

int ISP_API::Disable_LSC(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_DISABLE_LSC, nullptr);
    }
    return -1;
}

int ISP_API::Set_LSC_TBL(struct isp_lsc_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_LSC_TBL, context);
    }
    return -1;
}

int ISP_API::Set_LSC_SEC(struct isp_lsc_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_LSC_SEC, context);
    }
    return -1;
}

int ISP_API::Set_DPF(struct isp_dpf_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_DPF, context);
    }
    return -1;
}

//EE:锐化
int ISP_API::Set_EE(struct isp_ee_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_EE, context);
    }
    return -1;
}

int ISP_API::Set_EXP(struct isp_exp_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_EXP, context);
    }
    return -1;
}

//u8 mean[25];
int ISP_API::Get_EXP_Mean(uint8_t *mean)
{
    if (fd_isp >= 0 && mean != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_G_EXPMEAN, mean);
    }
    return -1;
}

int ISP_API::Set_HIST(struct isp_hist_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_HIST, context);
    }
    return -1;
}

//u32 mean[256];
int ISP_API::Get_HIST_Mean(uint32_t *mean)
{
    if (fd_isp >= 0 && mean != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_G_HISTMEAN, mean);
    }
    return -1;
}

int ISP_API::Set_DPCC(struct isp_dpcc_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_DPCC, context);
    }
    return -1;
}

int ISP_API::Set_FLT(struct isp_flt_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_FLT, context);
    }
    return -1;
}

int ISP_API::Set_CAC(struct isp_cac_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_CAC, context);
    }
    return -1;
}

int ISP_API::Set_DEG(struct isp_deg_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_DEG, context);
    }
    return -1;
}

int ISP_API::Set_AFM(struct isp_afm_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_AFM, context);
    }
    return -1;
}

int ISP_API::Get_AFM(struct isp_afm_result *afm)
{
    if (fd_isp >= 0 && afm != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_G_AFM, afm);
    }
    return -1;
}

int ISP_API::Set_VSM(struct isp_vsm_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_VSM, context);
    }
    return -1;
}

int ISP_API::Get_VSM(struct isp_vsm_result *vsm)
{
    if (fd_isp >= 0 && vsm != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_G_VSM, vsm);
    }
    return -1;
}

int ISP_API::Set_IE(struct isp_ie_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_IE, context);
    }
    return -1;
}

int ISP_API::Enable_WDR3(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_ENABLE_WDR3, nullptr);
    }
    return -1;
}

int ISP_API::Disable_WDR3(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_DISABLE_WDR3, nullptr);
    }
    return -1;
}

int ISP_API::Update_WDR3(struct isp_wdr3_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_U_WDR3, context);
    }
    return -1;
}

int ISP_API::Set_WDR3(struct isp_wdr3_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_WDR3, context);
    }
    return -1;
}

int ISP_API::Set_EXP2(struct isp_exp2_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_EXP2, context);
    }
    return -1;
}

int ISP_API::Set_2dnr(struct isp_2dnr_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_2DNR, context);
    }
    return -1;
}

int ISP_API::Set_3dnr(struct isp_3dnr_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_3DNR, context);
    }
    return -1;
}

int ISP_API::Get_3dnr(uint32_t *avg)
{
    if (fd_isp >= 0 && avg != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_G_3DNR, avg);
    }
    return -1;
}

int ISP_API::Update_3dnr(struct isp_3dnr_update *dnr3_update)
{
    if (fd_isp >= 0 && dnr3_update != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_U_3DNR, dnr3_update);
    }
    return -1;
}

int ISP_API::Reset_3dnr(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_R_3DNR, nullptr);
    }
    return -1;
}

int ISP_API::Set_3dnr_Cmp(struct isp_3dnr_compress_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_3DNR_CMP, context);
    }
    return -1;
}

int ISP_API::Set_HDR(struct isp_hdr_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_HDR, context);
    }
    return -1;
}

int ISP_API::Set_COMP(struct isp_comp_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_COMP, context);
    }
    return -1;
}

int ISP_API::Set_CPROC(struct isp_cproc_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_CPROC, context);
    }
    return -1;
}

int ISP_API::Set_SIMP(struct isp_simp_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_SIMP, context);
    }
    return -1;
}

int ISP_API::Set_ELAWB(struct isp_elawb_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_ELAWB, context);
    }
    return -1;
}

int ISP_API::Set_HDR_WB(struct isp_hdr_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_HDR_WB, context);
    }
    return -1;
}

int ISP_API::Set_HDR_BLS(struct isp_hdr_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_HDR_BLS, context);
    }
    return -1;
}

int ISP_API::Enable_WB(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_ENABLE_WB, nullptr);
    }
    return -1;
}

int ISP_API::Disable_WB(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_DISABLE_WB, nullptr);
    }
    return -1;
}

int ISP_API::Enable_HDR(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_ENABLE_HDR, nullptr);
    }
    return -1;
}

int ISP_API::Disable_HDR(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_DISABLE_HDR, nullptr);
    }
    return -1;
}

int ISP_API::Enable_Gamma_Out(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_ENABLE_GAMMA_OUT, nullptr);
    }
    return -1;
}

int ISP_API::Disable_Gamma_Out(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_DISABLE_GAMMA_OUT, nullptr);
    }
    return -1;
}

int ISP_API::IOC_Get_Status(uint32_t *status)
{
    if (fd_isp >= 0 && status != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_G_STATUS, status);
    }
    return -1;
}

int ISP_API::IOC_Get_Feature(uint32_t *feature)
{
    if (fd_isp >= 0 && feature != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_G_FEATURE, feature);
    }
    return -1;
}

int ISP_API::IOC_Get_Version(uint32_t *version)
{
    if (fd_isp >= 0 && version != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_G_FEATURE_VERSION, version);
    }
    return -1;
}

int ISP_API::Enable_GC_MONO(struct isp_gcmono_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_ENABLE_GCMONO, context);
    }
    return -1;
}

int ISP_API::Disable_GC_MONO(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_DISABLE_GCMONO, nullptr);
    }
    return -1;
}

int ISP_API::Set_GC_MONO(struct isp_gcmono_data *data)
{
    if (fd_isp >= 0 && data != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_GCMONO, data);
    }
    return -1;
}

int ISP_API::Enable_RGB_Gamma(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_ENABLE_RGBGAMMA, nullptr);
    }
    return -1;
}

int ISP_API::Disable_RGB_Gamma(void)
{
    if (fd_isp >= 0)
    {
        return xioctl(fd_isp, ISPIOC_DISABLE_RGBGAMMA, nullptr);
    }
    return -1;
}

int ISP_API::Set_RGB_Gamma(struct isp_rgbgamma_data *data)
{
    if (fd_isp >= 0 && data != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_RGBGAMMA, data);
    }
    return -1;
}

int ISP_API::Set_Demosaic(struct isp_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_DEMOSAIC, context);
    }
    return -1;
}

int ISP_API::Set_DMSC_Intp(struct isp_interplaion_thr_cxt *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_DMSC_INTP, context);
    }
    return -1;
}

int ISP_API::Set_DMSC_Skin(struct isp_skin_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_DMSC_SKIN, context);
    }
    return -1;
}

int ISP_API::Set_DMSC_CAC(struct isp_cac_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_DMSC_CAC, context);
    }
    return -1;
}

int ISP_API::Set_DMSC_Sharpen(struct isp_shap_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_DMSC_SHAP, context);
    }
    return -1;
}

int ISP_API::Set_DMSC_Dmoi(struct isp_dmoi_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_DMSC_DMOI, context);
    }
    return -1;
}

int ISP_API::Set_DMSC(struct isp_dmsc_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_DMSC, context);
    }
    return -1;
}

int ISP_API::Set_Green_Equilibrate(struct isp_ge_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_GREENEQUILIBRATE, context);
    }
    return -1;
}

int ISP_API::Set_Color_Adjust(struct isp_ca_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_COLOR_ADJUST, context);
    }
    return -1;
}

int ISP_API::Set_Digital_Gain(struct isp_digital_gain_cxt *gain)
{
    if (fd_isp >= 0 && gain != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_DIGITAL_GAIN, gain);
    }
    return -1;
}

int ISP_API::Get_Ext_Mem(struct isp_extmem_info *info)
{
    if (fd_isp >= 0 && info != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_G_QUERY_EXTMEM, info);
    }
    return -1;
}

int ISP_API::Set_WDR(struct isp_wdr_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_WDR_CONFIG, context);
    }
    return -1;
}

int ISP_API::Set_WDR_Curve(struct isp_wdr_context *context)
{
    if (fd_isp >= 0 && context != nullptr)
    {
        return xioctl(fd_isp, ISPIOC_S_WDR_CURVE, context);
    }
    return -1;
}

DWE_API::DWE_API(std::string fileName)
{
    fd_dwe = open(fileName.c_str(), O_RDWR, 0);
}

DWE_API::~DWE_API()
{
    if (fd_dwe >= 0)
    {
        close(fd_dwe);
    }
}

int DWE_API::Reset(void)
{
    if (fd_dwe >= 0)
    {
        return xioctl(fd_dwe, DWEIOC_RESET, nullptr);
    }
    return -1;
}

int DWE_API::Set_Params(struct dwe_hw_info *info)
{
    if (fd_dwe >= 0 && info != nullptr)
    {
        return xioctl(fd_dwe, DWEIOC_S_PARAMS, info);
    }
    return -1;
}

int DWE_API::Enable_Bus(void)
{
    if (fd_dwe >= 0)
    {
        return xioctl(fd_dwe, DWEIOC_ENABLE_BUS, nullptr);
    }
    return -1;
}

int DWE_API::Disable_Bus(void)
{
    if (fd_dwe >= 0)
    {
        return xioctl(fd_dwe, DWEIOC_DISABLE_BUS, nullptr);
    }
    return -1;
}

int DWE_API::Disable_IRQ(void)
{
    if (fd_dwe >= 0)
    {
        return xioctl(fd_dwe, DWEIOC_DISABLE_IRQ, nullptr);
    }
    return -1;
}

int DWE_API::Clear_IRQ(void)
{
    if (fd_dwe >= 0)
    {
        return xioctl(fd_dwe, DWEIOC_CLEAR_IRQ, nullptr);
    }
    return -1;
}

int DWE_API::Read_IRQ(uint32_t *ret)
{
    if (fd_dwe >= 0 && ret != nullptr)
    {
        return xioctl(fd_dwe, DWEIOC_READ_IRQ, ret);
    }
    return -1;
}

int DWE_API::Start(void)
{
    if (fd_dwe >= 0)
    {
        return xioctl(fd_dwe, DWEIOC_START, nullptr);
    }
    return -1;
}

int DWE_API::Stop(void)
{
    if (fd_dwe >= 0)
    {
        return xioctl(fd_dwe, DWEIOC_STOP, nullptr);
    }
    return -1;
}

int DWE_API::Start_Dma_Read(uint64_t *addr)
{
    if (fd_dwe >= 0 && addr != nullptr)
    {
        return xioctl(fd_dwe, DWEIOC_START_DMA_READ, addr);
    }
    return -1;
}

int DWE_API::Set_Buffer(uint64_t *addr)
{
    if (fd_dwe >= 0 && addr != nullptr)
    {
        return xioctl(fd_dwe, DWEIOC_SET_BUFFER, addr);
    }
    return -1;
}

int DWE_API::Set_LUT(struct lut_info *info)
{
    if (fd_dwe >= 0 && info != nullptr)
    {
        return xioctl(fd_dwe, DWEIOC_SET_LUT, info);
    }
    return -1;
}

} // namespace usb_cam
