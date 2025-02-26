#ifndef _ISP_DWE_APP_H_
#define _ISP_DWE_APP_H_

#include <string>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <fcntl.h> /* low-level i/o */


#include "isp_dwe/dwe_dev.h"
#include "isp_dwe/dwe_ioctl.h"
#include "isp_dwe/dwe_regs.h"
#include "isp_dwe/ic_dev.h"
#include "isp_dwe/isp_ioctl.h"
#include "isp_dwe/isp_types.h"
#include "isp_dwe/isp_version.h"
#include "isp_dwe/mrv_all_bits.h"
#include "isp_dwe/mrv_all_regs.h"
#include "isp_dwe/viv_video_kevent.h"
#include "isp_dwe/vvcsioc.h"
#include "isp_dwe/vvctrl.h"
#include "isp_dwe/vvdefs.h"
#include "isp_dwe/vvsensor.h"

namespace usb_cam
{

class ISP_API
{
public:
    ISP_API(std::string fileName);
    virtual ~ISP_API();
    int Reset(void);
    int Set_Input(struct isp_context *context);
    int Enable(void);
    int Disable(void);
    int Is_Enable(bool *enable);
    int Start_Stream(uint32_t *framenum);
    int Stop_Stream(void);
    int IOC_Disable_ISP_Off(void);
    int IOC_Start_Dma_Read(struct isp_dma_context *context);
    int IOC_Get_Status(uint32_t *status);
    int IOC_Get_Feature(uint32_t *feature);
    int IOC_Get_Version(uint32_t *version);
    int Set_Buffer(struct isp_buffer_context *context);
    int Set_Bp_Buffer(struct isp_bp_buffer_context *context);
    //MI:Memory Interface
    int MI_Start(struct isp_mi_context *context);
    int MI_Stop(void);
    //TPG:Test Patten Generator
    int Enable_TPG(void);
    int Disable_TPG(void);
    int Set_TPG(struct isp_tpg_context *context);
    //MCM:TODO???????  ,包含Vsync帧长(不占用曝光时间,但是会影响帧率),Hsync行长(会增加曝光时间,会影响帧率)
    int Set_MCM(struct isp_mcm_context *context);
    //BLS:Black Level Subtraction,传感器有暗电流，全黑环境也不能输出全黑（全0）图像，简单做法减去固定值
    int Enable_BLS(void);
    int Disable_BLS(void);
    int Set_BLS(struct isp_bls_context *context);
    //MUX:vi_dpcl,Data path control register
    int Set_MUX(struct isp_mux_context *context);
    //AWB:Auto White Balance
    int Enable_AWB(void);
    int Disable_AWB(void);
    int Set_AWB(struct isp_awb_context *context);
    int Get_AWB_Mean(struct isp_awb_mean *mean);
    //IS:Image Stabilization 防抖
    int Set_IS(struct isp_is_context *context);
    int Set_Raw_IS(struct isp_is_context *context);
    //CNR:Chroma Noise Reduction
    int Set_CNR(struct isp_cnr_context *context);
    //CC:Color Correction 
    int Set_CC(struct isp_cc_context *context);
    //Xtalk:(Crosstalk），串扰，即CCM
    int Set_Xtalk(struct isp_xtalk_context *context);
    //LSC(Lens Shading Correction)镜头阴影校正， 由于相机在成像距离较远时，随着视场角慢慢增大，能够通过照相机镜头的斜光束将慢慢减少，从而使得获得的图像中间比较亮，边缘比较暗，这个现象就是光学系统中的渐晕。由于渐晕现象带来的图像亮度不均会影响后续处理的准确性
    int Enable_LSC(void);
    int Disable_LSC(void);
    int Set_LSC_TBL(struct isp_lsc_context *context);
    int Set_LSC_SEC(struct isp_lsc_context *context);
    //DPF:Denoising Pre-Filter
    int Set_DPF(struct isp_dpf_context *context);
    //EE:锐化
    int Set_EE(struct isp_ee_context *context);
    //exp:Exposure Measurement
    int Set_EXP(struct isp_exp_context *context);
    int Get_EXP_Mean(uint8_t *mean); //u8 mean[25];
    //HIST:Histogram
    int Set_HIST(struct isp_hist_context *context);
    int Get_HIST_Mean(uint32_t *mean); //u32 mean[256];
    //DPCC:Defect Pixel Cluster Correction
    int Set_DPCC(struct isp_dpcc_context *context);
    //FLT:Filter Module
    int Set_FLT(struct isp_flt_context *context);
    //CAC:Chromatic Aberration Correction
    int Set_CAC(struct isp_cac_context *context);
    //DEG:de-gamma
    int Set_DEG(struct isp_deg_context *context);
    //AFM:Auto Focus Measurement
    int Set_AFM(struct isp_afm_context *context);
    int Get_AFM(struct isp_afm_result *afm);
    //VSM:Video Stablization Measurement 视频稳定
    int Set_VSM(struct isp_vsm_context *context);
    int Get_VSM(struct isp_vsm_result *vsm);
    //IE:Image Effects
    int Set_IE(struct isp_ie_context *context);
    //WDR3:Wide Dynamic Range
    int Enable_WDR3(void);
    int Disable_WDR3(void);
    int Update_WDR3(struct isp_wdr3_context *context);
    int Set_WDR3(struct isp_wdr3_context *context);
    //EXP2:Exposure
    int Set_EXP2(struct isp_exp2_context *context);
    //2dnr: 2d Noise Reduction
    int Set_2dnr(struct isp_2dnr_context *context);
    //3dnr: 3d Noise Reduction ,是结合空域滤波和时域滤波的一种降噪算法
    int Set_3dnr(struct isp_3dnr_context *context);
    int Get_3dnr(uint32_t *avg);
    int Update_3dnr(struct isp_3dnr_update *dnr3_update);
    int Reset_3dnr(void);
    int Set_3dnr_Cmp(struct isp_3dnr_compress_context *context);
    //COMP:compand
    int Set_COMP(struct isp_comp_context *context);
    //CPROC:Color Processing Module
    int Set_CPROC(struct isp_cproc_context *context);
    //SIMP:Superimpose叠加
    int Set_SIMP(struct isp_simp_context *context);
    //ELAWB:Elliptic Auto White Balance
    int Set_ELAWB(struct isp_elawb_context *context);
    int Enable_WB(void);
    int Disable_WB(void);
    //HDR:
    int Enable_HDR(void);
    int Disable_HDR(void);
    int Set_HDR(struct isp_hdr_context *context);
    int Set_HDR_WB(struct isp_hdr_context *context);
    int Set_HDR_BLS(struct isp_hdr_context *context);
    //GC_MONO：Control of gamma correction for mono sensor
    int Enable_GC_MONO(struct isp_gcmono_context *context);
    int Disable_GC_MONO(void);
    int Set_GC_MONO(struct isp_gcmono_data *data); /* set curve */
    int Enable_RGB_Gamma(void);
    int Disable_RGB_Gamma(void);
    int Set_RGB_Gamma(struct isp_rgbgamma_data *data);
    //Gamma_Out:TODO???????
    int Enable_Gamma_Out(void);
    int Disable_Gamma_Out(void);
    int Set_Gamma_Out(struct isp_gamma_out_context *context);
    //Demosaic:颜色插值
    int Set_Demosaic(struct isp_context *context);
    //DMSC:Demosaic parameters
    int Set_DMSC_Intp(struct isp_interplaion_thr_cxt *context);
    int Set_DMSC_Skin(struct isp_skin_context *context);
    int Set_DMSC_CAC(struct isp_cac_context *context);
    int Set_DMSC_Sharpen(struct isp_shap_context *context);
    int Set_DMSC_Dmoi(struct isp_dmoi_context *context);
    int Set_DMSC(struct isp_dmsc_context *context);
    //
    int Set_Green_Equilibrate(struct isp_ge_context *context);
    int Set_Color_Adjust(struct isp_ca_context *context);
    int Set_Digital_Gain(struct isp_digital_gain_cxt *gain);
    int Get_Ext_Mem(struct isp_extmem_info *info);
    int Set_WDR(struct isp_wdr_context *context);
    int Set_WDR_Curve(struct isp_wdr_context *context);
public:
    uint32_t isp_feature = 0;   //ISP_EE_SUPPORT ~ ISP_HDR_STITCH_SUPPORT
private:
    int fd_isp = -1;
};

class DWE_API
{
public:
    DWE_API(std::string fileName);
    virtual ~DWE_API();
    int Reset(void);
    int Set_Params(struct dwe_hw_info *info);
    int Enable_Bus(void);
    int Disable_Bus(void);
    int Disable_IRQ(void);
    int Clear_IRQ(void);
    int Read_IRQ(uint32_t *ret);
    int Start(void);
    int Stop(void);
    int Start_Dma_Read(uint64_t *addr);
    int Set_Buffer(uint64_t *addr);
    int Set_LUT(struct lut_info *info);
private:
    int fd_dwe = -1;
};

} // namespace usb_cam

#endif
