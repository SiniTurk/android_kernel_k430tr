#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include "kd_camera_hw.h"

#include <mach/board_lge.h>

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, args...)    pr_debug(PFX  fmt, ##args)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG(fmt, args...) pr_debug(PFX  fmt, ##args)
#define PK_ERR(fmt, arg...)         pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...)  pr_debug(PFX  fmt, ##args);
#else
#define PK_DBG(a, ...)
#define PK_ERR(a, ...)
#define PK_XLOG_INFO(fmt, args...)
#endif

/* GPIO Pin control*/
struct platform_device *cam_plt_dev = NULL;
struct pinctrl *camctrl = NULL;
struct pinctrl_state *cam0_rst_h = NULL;
struct pinctrl_state *cam0_rst_l = NULL;
struct pinctrl_state *cam0_vsync = NULL;
struct pinctrl_state *cam1_rst_h = NULL;
struct pinctrl_state *cam1_rst_l = NULL;
struct pinctrl_state *cam2_rst_h = NULL;
struct pinctrl_state *cam2_rst_l = NULL;
struct pinctrl_state *cam_ldo_avdd_h = NULL;
struct pinctrl_state *cam_ldo_avdd_l = NULL;
struct pinctrl_state *cam_ldo_dvdd_h = NULL;
struct pinctrl_state *cam_ldo_dvdd_l = NULL;
struct pinctrl_state *cam_ldo_iovdd_h = NULL;
struct pinctrl_state *cam_ldo_iovdd_l = NULL;
struct pinctrl_state *cam_mipisw_sel_h = NULL;
struct pinctrl_state *cam_mipisw_sel_l = NULL;
struct pinctrl_state *cam_mipisw_en_n_h = NULL;
struct pinctrl_state *cam_mipisw_en_n_l = NULL;

int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	if (lge_get_board_revno() == HW_REV_EVB1) {
		camctrl = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(camctrl)) {
			dev_err(&pdev->dev, "Cannot find camera pinctrl!");
			ret = PTR_ERR(camctrl);
		}

		/* Cam0 Rst Pin initialization */
		cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
		if (IS_ERR(cam0_rst_h)) {
			ret = PTR_ERR(cam0_rst_h);
			pr_debug("%s : pinctrl err, cam0_rst_h\n", __func__);
		}
		cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
		if (IS_ERR(cam0_rst_l)) {
			ret = PTR_ERR(cam0_rst_l);
			pr_debug("%s : pinctrl err, cam0_rst_l\n", __func__);
		}

		/* Cam0 vsync Pin initialization */
		cam0_vsync = pinctrl_lookup_state(camctrl, "cam0_vsync");
		if (IS_ERR(cam0_vsync)) {
			ret = PTR_ERR(cam0_vsync);
			pr_debug("%s : pinctrl err, cam0_vsync\n", __func__);
		}

		/* Cam1 Rst Pin initialization */
		cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
		if (IS_ERR(cam1_rst_h)) {
			ret = PTR_ERR(cam1_rst_h);
			pr_debug("%s : pinctrl err, cam1_rst_h\n", __func__);
		}
		cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
		if (IS_ERR(cam1_rst_l)) {
			ret = PTR_ERR(cam1_rst_l);
			pr_debug("%s : pinctrl err, cam1_rst_l\n", __func__);
		}

		/* Cam2 Rst Pin initialization */
		cam2_rst_h = pinctrl_lookup_state(camctrl, "cam2_rst1");
		if (IS_ERR(cam2_rst_h)) {
			ret = PTR_ERR(cam2_rst_h);
			pr_debug("%s : pinctrl err, cam2_rst_h\n", __func__);
		}
		cam2_rst_l = pinctrl_lookup_state(camctrl, "cam2_rst0");
		if (IS_ERR(cam2_rst_l)) {
			ret = PTR_ERR(cam2_rst_l);
			pr_debug("%s : pinctrl err, cam2_rst_l\n", __func__);
		}

		/* externel LDO (AVDD, DVDD, IOVDD) enable */
		/* AVDD */
		cam_ldo_avdd_h = pinctrl_lookup_state(camctrl, "cam_ldo_avdd1");
		if (IS_ERR(cam_ldo_avdd_h)) {
			ret = PTR_ERR(cam_ldo_avdd_h);
			pr_debug("%s : pinctrl err, cam_ldo_avdd_h\n", __func__);
		}
		cam_ldo_avdd_l = pinctrl_lookup_state(camctrl, "cam_ldo_avdd0");
		if (IS_ERR(cam_ldo_avdd_l)) {
			ret = PTR_ERR(cam_ldo_avdd_l);
			pr_debug("%s : pinctrl err, cam_ldo_avdd_l\n", __func__);
		}

		/* DVDD */
		cam_ldo_dvdd_h = pinctrl_lookup_state(camctrl, "cam_ldo_dvdd1");
		if (IS_ERR(cam_ldo_dvdd_h)) {
			ret = PTR_ERR(cam_ldo_dvdd_h);
			pr_debug("%s : pinctrl err, cam_ldo_dvdd_h\n", __func__);
		}
		cam_ldo_dvdd_l = pinctrl_lookup_state(camctrl, "cam_ldo_dvdd0");
		if (IS_ERR(cam_ldo_dvdd_l)) {
			ret = PTR_ERR(cam_ldo_dvdd_l);
			pr_debug("%s : pinctrl err, cam_ldo_dvdd_l\n", __func__);
		}

		/* IOVDD */
		cam_ldo_iovdd_h = pinctrl_lookup_state(camctrl, "cam_ldo_iovdd1");
		if (IS_ERR(cam_ldo_iovdd_h)) {
			ret = PTR_ERR(cam_ldo_iovdd_h);
			pr_debug("%s : pinctrl err, cam_ldo_iovdd_h\n", __func__);
		}
		cam_ldo_iovdd_l = pinctrl_lookup_state(camctrl, "cam_ldo_iovdd0");
		if (IS_ERR(cam_ldo_iovdd_l)) {
			ret = PTR_ERR(cam_ldo_iovdd_l);
			pr_debug("%s : pinctrl err, cam_ldo_iovdd_l\n", __func__);
		}

		/* MIPI Switch sel (CAM1/CAM2) Pin initialization */
		cam_mipisw_sel_h = pinctrl_lookup_state(camctrl, "cam_mipisw_sel1");
		if (IS_ERR(cam_mipisw_sel_h)) {
			ret = PTR_ERR(cam_mipisw_sel_h);
			pr_debug("%s : pinctrl err, cam_mipisw_sel_h\n", __func__);
		}
		cam_mipisw_sel_l = pinctrl_lookup_state(camctrl, "cam_mipisw_sel0");
		if (IS_ERR(cam_mipisw_sel_l)) {
			ret = PTR_ERR(cam_mipisw_sel_l);
			pr_debug("%s : pinctrl err, cam_mipisw_sel_l\n", __func__);
		}
	} else { // (lge_get_board_revno() >= HW_REV_A)
		camctrl = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(camctrl)) {
			dev_err(&pdev->dev, "Cannot find camera pinctrl!");
			ret = PTR_ERR(camctrl);
		}

		/* Cam0 Rst Pin initialization */
		cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
		if (IS_ERR(cam0_rst_h)) {
			ret = PTR_ERR(cam0_rst_h);
			pr_debug("%s : pinctrl err, cam0_rst_h\n", __func__);
		}
		cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
		if (IS_ERR(cam0_rst_l)) {
			ret = PTR_ERR(cam0_rst_l);
			pr_debug("%s : pinctrl err, cam0_rst_l\n", __func__);
		}

		/* Cam0 vsync Pin initialization */
		cam0_vsync = pinctrl_lookup_state(camctrl, "cam0_vsync");
		if (IS_ERR(cam0_vsync)) {
			ret = PTR_ERR(cam0_vsync);
			pr_debug("%s : pinctrl err, cam0_vsync\n", __func__);
		}

		/* Cam1 Rst Pin initialization */
		cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
		if (IS_ERR(cam1_rst_h)) {
			ret = PTR_ERR(cam1_rst_h);
			pr_debug("%s : pinctrl err, cam1_rst_h\n", __func__);
		}
		cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
		if (IS_ERR(cam1_rst_l)) {
			ret = PTR_ERR(cam1_rst_l);
			pr_debug("%s : pinctrl err, cam1_rst_l\n", __func__);
		}

		/* Cam2 Rst Pin initialization */
		cam2_rst_h = pinctrl_lookup_state(camctrl, "cam2_rst1");
		if (IS_ERR(cam2_rst_h)) {
			ret = PTR_ERR(cam2_rst_h);
			pr_debug("%s : pinctrl err, cam2_rst_h\n", __func__);
		}
		cam2_rst_l = pinctrl_lookup_state(camctrl, "cam2_rst0");
		if (IS_ERR(cam2_rst_l)) {
			ret = PTR_ERR(cam2_rst_l);
			pr_debug("%s : pinctrl err, cam2_rst_l\n", __func__);
		}

		/* MIPI Switch sel (CAM1/CAM2) Pin initialization */
		cam_mipisw_sel_h = pinctrl_lookup_state(camctrl, "cam_mipisw_sel1");
		if (IS_ERR(cam_mipisw_sel_h)) {
			ret = PTR_ERR(cam_mipisw_sel_h);
			pr_debug("%s : pinctrl err, cam_mipisw_sel_h\n", __func__);
		}
		cam_mipisw_sel_l = pinctrl_lookup_state(camctrl, "cam_mipisw_sel0");
		if (IS_ERR(cam_mipisw_sel_l)) {
			ret = PTR_ERR(cam_mipisw_sel_l);
			pr_debug("%s : pinctrl err, cam_mipisw_sel_l\n", __func__);
		}

		/* MIPI Switch enable_N  Pin initialization */
		cam_mipisw_en_n_h = pinctrl_lookup_state(camctrl, "cam_mipisw_en_n1");
		if (IS_ERR(cam_mipisw_en_n_h)) {
			ret = PTR_ERR(cam_mipisw_en_n_h);
			pr_debug("%s : pinctrl err, cam_mipisw_en_n_h\n", __func__);
		}
		cam_mipisw_en_n_l = pinctrl_lookup_state(camctrl, "cam_mipisw_en_n0");
		if (IS_ERR(cam_mipisw_en_n_l)) {
			ret = PTR_ERR(cam_mipisw_en_n_l);
			pr_debug("%s : pinctrl err, cam_mipisw_en_n_l\n", __func__);
		}
	}
	return ret;
}

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;

	if (lge_get_board_revno() == HW_REV_EVB1) {
		switch (PwrType) {
		case CAMRST:
			if (PinIdx == 0) {
				if (Val == 0)
					pinctrl_select_state(camctrl, cam0_rst_l);
				else
					pinctrl_select_state(camctrl, cam0_rst_h);
			} else if (PinIdx == 1) {
				if (Val == 0)
					pinctrl_select_state(camctrl, cam1_rst_l);
				else
					pinctrl_select_state(camctrl, cam1_rst_h);
			} else {
				if (Val == 0)
					pinctrl_select_state(camctrl, cam2_rst_l);
				else
					pinctrl_select_state(camctrl, cam2_rst_h);
			}
			break;
		case CAMLDO_AVDD:
			if (Val == 0)
				pinctrl_select_state(camctrl, cam_ldo_avdd_l);
			else
				pinctrl_select_state(camctrl, cam_ldo_avdd_h);
			break;
		case CAMLDO_DVDD:
			if (Val == 0)
				pinctrl_select_state(camctrl, cam_ldo_dvdd_l);
			else
				pinctrl_select_state(camctrl, cam_ldo_dvdd_h);
			break;
		case CAMLDO_IOVDD:
			if (Val == 0)
				pinctrl_select_state(camctrl, cam_ldo_iovdd_l);
			else
				pinctrl_select_state(camctrl, cam_ldo_iovdd_h);
			break;
		case CAMMIPISW_SEL:
			if (Val == 0)
				pinctrl_select_state(camctrl, cam_mipisw_sel_l);
			else
				pinctrl_select_state(camctrl, cam_mipisw_sel_h);
			break;
		case CAMMIPISW_EN_N:
			if (Val == 0)
				pinctrl_select_state(camctrl, cam_mipisw_en_n_l);
			else
				pinctrl_select_state(camctrl, cam_mipisw_en_n_h);
			break;
		default:
			PK_DBG("PwrType(%d) is invalid !!\n", PwrType);
			break;
		};
	} else { // (lge_get_board_revno() >= HW_REV_A)
		switch (PwrType) {
		case CAMRST:
			if (PinIdx == 0) {
				if (Val == 0)
					pinctrl_select_state(camctrl, cam0_rst_l);
				else
					pinctrl_select_state(camctrl, cam0_rst_h);
			} else if (PinIdx == 1) {
				if (Val == 0)
					pinctrl_select_state(camctrl, cam1_rst_l);
				else
					pinctrl_select_state(camctrl, cam1_rst_h);
			} else {
				if (Val == 0)
					pinctrl_select_state(camctrl, cam2_rst_l);
				else
					pinctrl_select_state(camctrl, cam2_rst_h);
			}
			break;
		case CAMMIPISW_SEL:
			if (Val == 0)
				pinctrl_select_state(camctrl, cam_mipisw_sel_l);
			else
				pinctrl_select_state(camctrl, cam_mipisw_sel_h);
			break;
		case CAMMIPISW_EN_N:
			if (Val == 0)
				pinctrl_select_state(camctrl, cam_mipisw_en_n_l);
			else
				pinctrl_select_state(camctrl, cam_mipisw_en_n_h);
			break;
		default:
			PK_DBG("PwrType(%d) is invalid !!\n", PwrType);
			break;
		};
	}

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}


int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, bool On, char *mode_name)
{
#define MAX_CAM_NUM    3
#define VOL_2800    2800000
#define VOL_1800    1800000
#define VOL_1500    1500000
#define VOL_1200    1200000
#define VOL_1000    1000000

	u32 pinSetIdx = 0;/* default main sensor */
	u32 i = 0;

	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx)
		pinSetIdx = 0;
	else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx)
		pinSetIdx = 1;
	else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx)
		pinSetIdx = 2;

	/* power ON */
	if (On) {
		if (pinSetIdx == 0)
			ISP_MCLK1_EN(1);
		else if (pinSetIdx == 1)
			ISP_MCLK2_EN(1);
		else
			ISP_MCLK2_EN(1);

		PK_DBG("[PowerON]pinSetIdx:%d, currSensorName: %s\n", pinSetIdx, currSensorName);

		if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX258_MIPI_RAW, currSensorName))) {
			/* First Reset Pin Low */
			for (i = 0; i < MAX_CAM_NUM; i++)
				mtkcam_gpio_set(i, CAMRST, GPIO_OUT_ZERO);

			/* AF_VCC */
			if (TRUE != _hwPowerOn(VCAMAF, VOL_2800)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_AF, power id = %d\n", VCAMAF);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);

			/* VCAM_A */
			if (TRUE != _hwPowerOn(VCAMA, VOL_2800)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_A, power id = %d\n", VCAMA);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);

			/* VCAM_IO */
			if (TRUE != _hwPowerOn(VCAMIO, VOL_1800)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_IO, power id = %d\n", VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(2);

			/* VCAM_D */
			if (TRUE != _hwPowerOn(VCAMD, VOL_1200)) {
				PK_DBG("[CAMERA SENSOR] Fail to enable VCAM_D, power id = %d\n", VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);

			/* enable active sensor */
			mtkcam_gpio_set(pinSetIdx, CAMRST, GPIO_OUT_ONE);
			mdelay(15);

		} else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV8858_MIPI_RAW, currSensorName))) {
			if (lge_get_board_revno() == HW_REV_EVB1) {
				/* EVB : to enable I2C pullup power -  VCAM_IO */
				_hwPowerOn(VCAMIO, VOL_1800);
				mdelay(1);

				/* MIPI switch */
				mtkcam_gpio_set(0, CAMMIPISW_SEL, GPIO_OUT_ZERO);  // 0:ov8858, 1:hi553
				mdelay(1);

				/* First Reset Pin Low */
				for (i = 0; i < MAX_CAM_NUM; i++)
					mtkcam_gpio_set(i, CAMRST, GPIO_OUT_ZERO);
				mdelay(1);

				/* CAMLDO_AVDD */
				mtkcam_gpio_set(0, CAMLDO_AVDD, GPIO_OUT_ONE);
				mdelay(1);

				/* CAMLDO_IOVDD */
				mtkcam_gpio_set(0, CAMLDO_IOVDD, GPIO_OUT_ONE);
				mdelay(1);

				/* CAMLDO_DVDD */
				mtkcam_gpio_set(0, CAMLDO_DVDD, GPIO_OUT_ONE);
				mdelay(1);

				/* enable active sensor */
				mtkcam_gpio_set(pinSetIdx, CAMRST, GPIO_OUT_ONE);
				mdelay(5);
			} else { // (lge_get_board_revno() >= HW_REV_A)
				/* MIPI switch */
				mtkcam_gpio_set(0, CAMMIPISW_SEL, GPIO_OUT_ZERO);  // 0:ov8858, 1:hi553
				mtkcam_gpio_set(0, CAMMIPISW_EN_N, GPIO_OUT_ZERO); // 0:enable, 1:disable
				mdelay(1);

				/* First Reset Pin Low */
				for (i = 0; i < MAX_CAM_NUM; i++)
					mtkcam_gpio_set(i, CAMRST, GPIO_OUT_ZERO);
				mdelay(1);

				/* AVDD (VTCXO_1) */
				if (TRUE != _hwPowerOn(VTCXO_1, VOL_2800)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable AVDD (VTCXO_1), power id = %d\n", VTCXO_1);
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(1);

				/* IOVDD (VGP1) */
				if (TRUE != _hwPowerOn(VGP1, VOL_1800)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable IOVDD (VGP1), power id = %d\n", VGP1);
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(1);

				/* DVDD (VRF18_1) */
				if (TRUE != _hwPowerOn(VRF18_1, VOL_1200)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable DVDD (VRF18_1), power id = %d\n", VRF18_1);
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(1);

				/* enable active sensor */
				mtkcam_gpio_set(pinSetIdx, CAMRST, GPIO_OUT_ONE);
				mdelay(5);
			}
		} else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI553_MIPI_RAW, currSensorName))) {
			if (lge_get_board_revno() == HW_REV_EVB1) {
				/* EVB : to enable I2C pullup power -  VCAM_IO */
				_hwPowerOn(VCAMIO, VOL_1800);
				mdelay(1);

				/* MIPI switch */
				mtkcam_gpio_set(0, CAMMIPISW_SEL, GPIO_OUT_ONE);   // 0:ov8858, 1:hi553
				mdelay(1);

				/* First Reset Pin Low */
				for (i = 0; i < MAX_CAM_NUM; i++)
					mtkcam_gpio_set(i, CAMRST, GPIO_OUT_ZERO);
				mdelay(1);

				/* CAMLDO_IOVDD */
				mtkcam_gpio_set(0, CAMLDO_IOVDD, GPIO_OUT_ONE);
				mdelay(1);

				/* CAMLDO_AVDD */
				mtkcam_gpio_set(0, CAMLDO_AVDD, GPIO_OUT_ONE);
				mdelay(1);

				/* CAMLDO_DVDD */
				mtkcam_gpio_set(0, CAMLDO_DVDD, GPIO_OUT_ONE);
				mdelay(1);

				/* enable active sensor */
				mtkcam_gpio_set(pinSetIdx, CAMRST, GPIO_OUT_ONE);
				mdelay(5);
			} else { // (lge_get_board_revno() >= HW_REV_A)
				/* MIPI switch */
				mtkcam_gpio_set(0, CAMMIPISW_SEL, GPIO_OUT_ONE);   // 0:ov8858, 1:hi553
				mtkcam_gpio_set(0, CAMMIPISW_EN_N, GPIO_OUT_ZERO); // 0:enable, 1:disable
				mdelay(1);

				/* First Reset Pin Low */
				for (i = 0; i < MAX_CAM_NUM; i++)
					mtkcam_gpio_set(i, CAMRST, GPIO_OUT_ZERO);
				mdelay(1);

				/* IOVDD (VGP1) */
				if (TRUE != _hwPowerOn(VGP1, VOL_1800)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable IOVDD (VGP1), power id = %d\n", VGP1);
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(1);

				/* AVDD (VTCXO_1) */
				if (TRUE != _hwPowerOn(VTCXO_1, VOL_2800)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable AVDD (VTCXO_1), power id = %d\n", VTCXO_1);
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(1);

				/* DVDD (VRF18_1) */
				if (TRUE != _hwPowerOn(VRF18_1, VOL_1200)) {
					PK_DBG("[CAMERA SENSOR] Fail to enable DVDD (VRF18_1), power id = %d\n", VRF18_1);
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(1);

				/* enable active sensor */
				mtkcam_gpio_set(pinSetIdx, CAMRST, GPIO_OUT_ONE);
				mdelay(5);
			}
		}
	} else { /* power OFF */

		PK_DBG("[PowerOFF]pinSetIdx:%d\n", pinSetIdx);

		if (pinSetIdx == 0)
			ISP_MCLK1_EN(0);
		else if (pinSetIdx == 1)
			ISP_MCLK2_EN(0);
		else
			ISP_MCLK2_EN(0);

		PK_DBG("[PowerON]pinSetIdx:%d, currSensorName: %s\n", pinSetIdx, currSensorName);

		if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX258_MIPI_RAW, currSensorName))) {

			/* Set Reset Pin Low */
			for (i = 0; i < MAX_CAM_NUM; i++)
				mtkcam_gpio_set(i, CAMRST, GPIO_OUT_ZERO);
			mdelay(1);

			/* AF_VCC */
			if (TRUE != _hwPowerDown(VCAMAF)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF VCAM_AF,power id = %d\n", VCAMAF);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);

			/* VCAM_A */
			if (TRUE != _hwPowerDown(VCAMA)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF VCAM_A,power id= (%d)\n", VCAMA);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);

			/* VCAM_D */
			if (TRUE != _hwPowerDown(VCAMD)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF VCAM_D,power id = %d\n", VCAMD);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);

			/* VCAM_IO */
			if (TRUE != _hwPowerDown(VCAMIO)) {
				PK_DBG("[CAMERA SENSOR] Fail to OFF VCAM_IO,power id = %d\n", VCAMIO);
				goto _kdCISModulePowerOn_exit_;
			}
			mdelay(1);

		} else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV8858_MIPI_RAW, currSensorName))) {
			if (lge_get_board_revno() == HW_REV_EVB1) {
				/* Reset Pin Low */
				for (i = 0; i < MAX_CAM_NUM; i++)
					mtkcam_gpio_set(i, CAMRST, GPIO_OUT_ZERO);
				mdelay(1);

				/* CAMLDO_DVDD */
				mtkcam_gpio_set(0, CAMLDO_DVDD, GPIO_OUT_ZERO);
				mdelay(1);

				/* CAMLDO_IOVDD */
				mtkcam_gpio_set(0, CAMLDO_IOVDD, GPIO_OUT_ZERO);
				mdelay(1);

				/* CAMLDO_AVDD */
				mtkcam_gpio_set(0, CAMLDO_AVDD, GPIO_OUT_ZERO);
				mdelay(1);

				/* EVB : to disable I2C pullup power -  VCAM_IO */
				_hwPowerDown(VCAMIO);
				mdelay(1);
			} else { // (lge_get_board_revno() >= HW_REV_A)
				/* Reset Pin Low */
				for (i = 0; i < MAX_CAM_NUM; i++)
					mtkcam_gpio_set(i, CAMRST, GPIO_OUT_ZERO);
				mdelay(1);

				/* DVDD (VRF18_1) */
				if (TRUE != _hwPowerDown(VRF18_1)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF DVDD (VRF18_1),power id = %d\n", VRF18_1);
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(1);

				/* IOVDD (VGP1) */
				if (TRUE != _hwPowerDown(VGP1)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF IOVDD (VGP1),power id = %d\n", VGP1);
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(1);

				/* AVDD (VTCXO_1) */
				if (TRUE != _hwPowerDown(VTCXO_1)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF AVDD (VTCXO_1),power id = %d\n", VTCXO_1);
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(1);

				/* MIPI switch */
				mtkcam_gpio_set(0, CAMMIPISW_EN_N, GPIO_OUT_ONE);  // 0:enable, 1:disable
				mdelay(1);
			}
		} else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI553_MIPI_RAW, currSensorName))) {
			if (lge_get_board_revno() == HW_REV_EVB1) {
				/* Reset Pin Low */
				for (i = 0; i < MAX_CAM_NUM; i++)
					mtkcam_gpio_set(i, CAMRST, GPIO_OUT_ZERO);
				mdelay(1);

				/* CAMLDO_DVDD */
				mtkcam_gpio_set(0, CAMLDO_DVDD, GPIO_OUT_ZERO);
				mdelay(1);

				/* CAMLDO_IOVDD */
				mtkcam_gpio_set(0, CAMLDO_IOVDD, GPIO_OUT_ZERO);
				mdelay(1);

				/* CAMLDO_AVDD */
				mtkcam_gpio_set(0, CAMLDO_AVDD, GPIO_OUT_ZERO);
				mdelay(1);

				/* EVB : to disable I2C pullup power -  VCAM_IO */
				_hwPowerDown(VCAMIO);
				mdelay(1);
			} else { // (lge_get_board_revno() >= HW_REV_A)
				/* Reset Pin Low */
				for (i = 0; i < MAX_CAM_NUM; i++)
					mtkcam_gpio_set(i, CAMRST, GPIO_OUT_ZERO);
				mdelay(1);

				/* DVDD (VRF18_1) */
				if (TRUE != _hwPowerDown(VRF18_1)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF DVDD (VRF18_1),power id = %d\n", VRF18_1);
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(1);

				/* IOVDD (VGP1) */
				if (TRUE != _hwPowerDown(VGP1)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF IOVDD (VGP1),power id = %d\n", VGP1);
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(1);

				/* AVDD (VTCXO_1) */
				if (TRUE != _hwPowerDown(VTCXO_1)) {
					PK_DBG("[CAMERA SENSOR] Fail to OFF AVDD (VTCXO_1),power id = %d\n", VTCXO_1);
					goto _kdCISModulePowerOn_exit_;
				}
				mdelay(1);

				/* MIPI switch */
				mtkcam_gpio_set(0, CAMMIPISW_EN_N, GPIO_OUT_ONE);  // 0:enable, 1:disable
				mdelay(1);
			}
		}
	}

	return 0;

_kdCISModulePowerOn_exit_:
	if (lge_get_board_revno() >= HW_REV_A) {
		/* MIPI switch */
		mtkcam_gpio_set(0, CAMMIPISW_EN_N, GPIO_OUT_ONE);  // 0:enable, 1:disable
	}
	return -EIO;

}

EXPORT_SYMBOL(kdCISModulePowerOn);

/* !-- */
/*  */

