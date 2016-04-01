/* include/linux/usb/msm_hsusb.h
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 * Copyright (c) 2009-2018, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_HSUSB_H
#define __ASM_ARCH_MSM_HSUSB_H

#include <linux/types.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/clk.h>
#include <linux/pm_qos.h>
#include <linux/hrtimer.h>
#include <linux/power_supply.h>
#include <linux/cdev.h>
#include <linux/usb_bam.h>
#include <linux/extcon.h>
#include <linux/regulator/driver.h>
/**
 * Requested USB votes for NOC frequency
 *
 * USB_NOC_NOM_VOTE    Vote for NOM set of NOC frequencies
 * USB_NOC_SVS_VOTE    Vote for SVS set of NOC frequencies
 *
 */
enum usb_noc_mode {
	USB_NOC_NOM_VOTE = 0,
	USB_NOC_SVS_VOTE,
	USB_NOC_NUM_VOTE,
};

/**
 * Different states involved in USB charger detection.
 *
 * USB_CHG_STATE_UNDEFINED	USB charger is not connected or detection
 *                              process is not yet started.
 * USB_CHG_STATE_IN_PROGRESS	Charger detection in progress
 * USB_CHG_STATE_WAIT_FOR_DCD	Waiting for Data pins contact.
 * USB_CHG_STATE_DCD_DONE	Data pin contact is detected.
 * USB_CHG_STATE_PRIMARY_DONE	Primary detection is completed (Detects
 *                              between SDP and DCP/CDP).
 * USB_CHG_STATE_SECONDARY_DONE	Secondary detection is completed (Detects
 *                              between DCP and CDP).
 * USB_CHG_STATE_DETECTED	USB charger type is determined.
 * USB_CHG_STATE_QUEUE_SM_WORK	SM work to start/stop gadget is queued.
 *
 */
enum usb_chg_state {
	USB_CHG_STATE_UNDEFINED = 0,
	USB_CHG_STATE_IN_PROGRESS,
	USB_CHG_STATE_WAIT_FOR_DCD,
	USB_CHG_STATE_DCD_DONE,
	USB_CHG_STATE_PRIMARY_DONE,
	USB_CHG_STATE_SECONDARY_DONE,
	USB_CHG_STATE_DETECTED,
	USB_CHG_STATE_QUEUE_SM_WORK,
};

/**
 * USB charger types
 *
 * USB_INVALID_CHARGER	Invalid USB charger.
 * USB_SDP_CHARGER	Standard downstream port. Refers to a downstream port
 *                      on USB2.0 compliant host/hub.
 * USB_DCP_CHARGER	Dedicated charger port (AC charger/ Wall charger).
 * USB_CDP_CHARGER	Charging downstream port. Enumeration can happen and
 *                      IDEV_CHG_MAX can be drawn irrespective of USB state.
 * USB_NONCOMPLIANT_CHARGER A non-compliant charger pull DP and DM to specific
 *			    voltages between 2.0-3.3v for identification.
 *
 */
enum usb_chg_type {
	USB_INVALID_CHARGER = 0,
	USB_SDP_CHARGER,
	USB_DCP_CHARGER,
	USB_CDP_CHARGER,
	USB_NONCOMPLIANT_CHARGER,
	USB_FLOATED_CHARGER,
};

/**
 * Maintain state for hvdcp external charger status
 * DEFAULT	This is used when DCP is detected
 * ACTIVE	This is used when ioctl is called to block LPM
 * INACTIVE	This is used when ioctl is called to unblock LPM
 */

enum usb_ext_chg_status {
	DEFAULT = 1,
	ACTIVE,
	INACTIVE,
};

/**
 * USB ID state
 */
enum usb_id_state {
	USB_ID_GROUND = 0,
	USB_ID_FLOAT,
};

/**
 * struct msm_otg_platform_data - platform device data
 *              for msm_otg driver.
 * @phy_init_seq: PHY configuration sequence. val, reg pairs
 *              terminated by -1.
 * @vbus_power: VBUS power on/off routine.It should return result
 *		as success(zero value) or failure(non-zero value).
 * @power_budget: VBUS power budget in mA (0 will be treated as 500mA).
 * @mode: Supported mode (OTG/peripheral/host).
 * @otg_control: OTG switch controlled by user/Id pin
 * @default_mode: Default operational mode. Applicable only if
 *              OTG switch is controller by user.
 * @pmic_id_irq: IRQ number assigned for PMIC USB ID line.
 * @mpm_otgsessvld_int: MPM wakeup pin assigned for OTG SESSVLD
 *              interrupt. Used when .otg_control == OTG_PHY_CONTROL.
 * @mpm_dpshv_int: MPM wakeup pin assigned for DP SHV interrupt.
 *		Used during host bus suspend.
 * @mpm_dmshv_int: MPM wakeup pin assigned for DM SHV interrupt.
 *		Used during host bus suspend.
 * @mhl_enable: indicates MHL connector or not.
 * @disable_reset_on_disconnect: perform USB PHY and LINK reset
 *              on USB cable disconnection.
 * @pnoc_errata_fix: workaround needed for PNOC hardware bug that
 *              affects USB performance.
 * @enable_lpm_on_suspend: Enable the USB core to go into Low
 *              Power Mode, when USB bus is suspended but cable
 *              is connected.
 * @core_clk_always_on_workaround: Don't disable core_clk when
 *              USB enters LPM.
 * @delay_lpm_on_disconnect: Use a delay before entering LPM
 *              upon USB cable disconnection.
 * @enable_sec_phy: Use second HSPHY with USB2 core
 * @bus_scale_table: parameters for bus bandwidth requirements
 * @mhl_dev_name: MHL device name used to register with MHL driver.
 * @log2_itc: value of 2^(log2_itc-1) will be used as the
 *              interrupt threshold (ITC), when log2_itc is
 *              between 1 to 7.
 * @l1_supported: enable link power management support.
 * @dpdm_pulldown_added: Indicates whether pull down resistors are
 *		connected on data lines or not.
 * @vddmin_gpio: dedictaed gpio in the platform that is used for
 *		pullup the D+ line in case of bus suspend with
 *		phy retention.
 * @rw_during_lpm_workaround: Determines whether remote-wakeup
 *		during low-power mode workaround will be
 *		applied.
 * @enable_ahb2ahb_bypass: Indicates whether enable AHB2AHB BYPASS
 *		mode with controller in device mode.
 * @bool disable_retention_with_vdd_min: Indicates whether to enable
		allowing VDDmin without putting PHY into retention.
 * @usb_id_gpio: Gpio used for USB ID detection.
 * @hub_reset_gpio: Gpio used for hub reset.
 * @switch_sel_gpio: Gpio used for controlling switch that
		routing D+/D- from the USB HUB to the USB jack type B
		for peripheral mode.
 * @bool phy_dvdd_always_on: PHY DVDD is supplied by always on PMIC LDO.
 * @bool emulation: Indicates whether we are running on emulation platform.
 * @bool enable_streaming: Indicates whether streaming to be enabled by default.
 * @bool enable_axi_prefetch: Indicates whether AXI Prefetch interface is used
		for improving data performance.
 */
struct msm_otg_platform_data {
	int *phy_init_seq;
	int (*vbus_power)(bool on);
	unsigned power_budget;
	enum usb_mode_type mode;
	enum otg_control_type otg_control;
	enum usb_mode_type default_mode;
	enum msm_usb_phy_type phy_type;
	int pmic_id_irq;
	unsigned int mpm_otgsessvld_int;
	unsigned int mpm_dpshv_int;
	unsigned int mpm_dmshv_int;
	bool mhl_enable;
	bool disable_reset_on_disconnect;
	bool pnoc_errata_fix;
	bool enable_lpm_on_dev_suspend;
	bool core_clk_always_on_workaround;
	bool delay_lpm_on_disconnect;
	bool dp_manual_pullup;
	bool enable_sec_phy;
	struct msm_bus_scale_pdata *bus_scale_table;
	const char *mhl_dev_name;
	int log2_itc;
	bool l1_supported;
	bool dpdm_pulldown_added;
	int vddmin_gpio;
	bool rw_during_lpm_workaround;
	bool enable_ahb2ahb_bypass;
	bool disable_retention_with_vdd_min;
	int usb_id_gpio;
	int hub_reset_gpio;
	int switch_sel_gpio;
	int usbid_switch;
	bool phy_dvdd_always_on;
	bool emulation;
	bool enable_streaming;
	bool enable_axi_prefetch;
	struct clk *system_clk;
	struct clk *pclk;
};

/* phy related flags */
#define ENABLE_DP_MANUAL_PULLUP		BIT(0)
#define ENABLE_SECONDARY_PHY		BIT(1)
#define PHY_HOST_MODE			BIT(2)
#define PHY_CHARGER_CONNECTED		BIT(3)
#define PHY_VBUS_VALID_OVERRIDE		BIT(4)

/* Timeout (in msec) values (min - max) associated with OTG timers */

#define TA_WAIT_VRISE	100	/* ( - 100)  */
#define TA_WAIT_VFALL	500	/* ( - 1000) */

/*
 * This option is set for embedded hosts or OTG devices in which leakage
 * currents are very minimal.
 */
#ifdef CONFIG_USB_OTG
#define TA_WAIT_BCON	30000	/* (1100 - 30000) */
#else
#define TA_WAIT_BCON	-1
#endif

#define TA_AIDL_BDIS	500	/* (200 - ) */
#define TA_BIDL_ADIS	155	/* (155 - 200) */
#define TB_SRP_FAIL	6000	/* (5000 - 6000) */
#define TB_ASE0_BRST	200	/* (155 - ) */

/* TB_SSEND_SRP and TB_SE0_SRP are combined */
#define TB_SRP_INIT	2000	/* (1500 - ) */

#define TA_TST_MAINT	10100	/* (9900 - 10100) */
#define TB_TST_SRP	3000	/* ( - 5000) */
#define TB_TST_CONFIG	300

/* Timeout variables */

#define A_WAIT_VRISE	0
#define A_WAIT_VFALL	1
#define A_WAIT_BCON	2
#define A_AIDL_BDIS	3
#define A_BIDL_ADIS	4
#define B_SRP_FAIL	5
#define B_ASE0_BRST	6
#define A_TST_MAINT	7
#define B_TST_SRP	8
#define B_TST_CONFIG	9

#define USB_NUM_BUS_CLOCKS      3

/**
 * struct msm_otg: OTG driver data. Shared by HCD and DCD.
 * @otg: USB OTG Transceiver structure.
 * @pdata: otg device platform data.
 * @irq: IRQ number assigned for HSUSB controller.
 * @async_irq: IRQ number used by some controllers during low power state
 * @phy_irq: IRQ number assigned for PHY to notify events like id and line
		state changes.
 * @pclk: clock struct of iface_clk.
 * @core_clk: clock struct of core_bus_clk.
 * @sleep_clk: clock struct of sleep_clk for USB PHY.
 * @phy_reset_clk: clock struct of phy_reset_clk for USB PHY. This clock is
		a reset only clock and resets the PHY, ULPI bridge and
		CSR wrapper.
 * @phy_por_clk: clock struct of phy_por_clk for USB PHY. This clock is
		a reset only clock and resets only the PHY (POR).
 * @phy_csr_clk: clock struct of phy_csr_clk for USB PHY. This clock is
		required to access PHY CSR registers via AHB2PHY interface.
 * @bus_clks: bimc/snoc/pcnoc clock struct.
 * @core_reset: Reset control for core_clk
 * @phy_reset: Reset control for phy_reset_clk
 * @phy_por_reset: Reset control for phy_por_clk
 * @default_noc_mode: default frequency for NOC clocks - SVS or NOM
 * @core_clk_rate: core clk max frequency
 * @regs: ioremapped register base address.
 * @usb_phy_ctrl_reg: relevant PHY_CTRL_REG register base address.
 * @inputs: OTG state machine inputs(Id, SessValid etc).
 * @sm_work: OTG state machine work.
 * @sm_work_pending: OTG state machine work is pending, queued post pm_resume
 * @resume_pending: USB h/w lpm_exit pending. Done on next sm_work run
 * @pm_suspended: OTG device is system(PM) suspended.
 * @pm_notify: Notifier to receive system wide PM transition events.
		It is used to defer wakeup events processing until
		system is RESUMED.
 * @in_lpm: indicates low power mode (LPM) state.
 * @async_int: IRQ line on which ASYNC interrupt arrived in LPM.
 * @cur_power: The amount of mA available from downstream port.
 * @otg_wq: Strict order otg workqueue for OTG works (SM/ID/SUSPEND).
 * @chg_work: Charger detection work.
 * @chg_state: The state of charger detection process.
 * @chg_type: The type of charger attached.
 * @chg_detection: True if PHY is doing charger type detection.
 * @bus_perf_client: Bus performance client handle to request BUS bandwidth
 * @host_bus_suspend: indicates host bus suspend or not.
 * @device_bus_suspend: indicates device bus suspend or not.
 * @bus_clks_enabled: indicates pcnoc/snoc/bimc clocks are on or not.
 * @is_ext_chg_dcp: To indicate whether charger detected by external entity
		SMB hardware is DCP charger or not.
 * @ext_id_irq: IRQ for ID interrupt.
 * @phy_irq_pending: Gets set when PHY IRQ arrives in LPM.
 * @id_state: Indicates USBID line status.
 * @rm_pulldown: Indicates pulldown status on D+ and D- data lines.
 * @extcon_vbus: Used for VBUS notification registration.
 * @extcon_id: Used for ID notification registration.
 * @vbus_nb: Notification callback for VBUS event.
 * @id_nb: Notification callback for ID event.
 * @extcon_registered: indicates if extcon notifier registered or not.
 * @dpdm_desc: Regulator descriptor for D+ and D- voting.
 * @dpdm_rdev: Regulator class device for dpdm regulator.
 * @dbg_idx: Dynamic debug buffer Index.
 * @dbg_lock: Dynamic debug buffer Lock.
 * @buf: Dynamic Debug Buffer.
 * @max_nominal_system_clk_rate: max freq at which system clock can run in
		nominal mode.
 * @sdp_check: SDP detection work in case of USB_FLOAT power supply
 * @notify_charger_work: Charger notification work.
 * @extcon_register_work: Extcon registration work.
 * @psy_nb: Notification callback for PSY registration.
 */
struct msm_otg {
	struct usb_phy phy;
	struct msm_otg_platform_data *pdata;
	struct platform_device *pdev;
	int irq;
	int async_irq;
	int phy_irq;
	struct clk *xo_clk;
	struct clk *pclk;
	struct clk *core_clk;
	struct clk *sleep_clk;
	struct clk *phy_reset_clk;
	struct clk *phy_por_clk;
	struct clk *phy_csr_clk;
	struct clk *bus_clks[USB_NUM_BUS_CLOCKS];
	struct clk *phy_ref_clk;
	struct reset_control *core_reset;
	struct reset_control *phy_reset;
	struct reset_control *phy_por_reset;
	long core_clk_rate;
	long core_clk_svs_rate;
	long core_clk_nominal_rate;
	enum usb_noc_mode default_noc_mode;
	struct resource *io_res;
	void __iomem *regs;
	void __iomem *phy_csr_regs;
	void __iomem *usb_phy_ctrl_reg;
#define ID		0
#define B_SESS_VLD	1
#define A_BUS_SUSPEND	14
	unsigned long inputs;
	struct work_struct sm_work;
	bool sm_work_pending;
	bool resume_pending;
	atomic_t pm_suspended;
	struct notifier_block pm_notify;
	atomic_t in_lpm;
	bool err_event_seen;
	int async_int;
	unsigned int cur_power;
	struct workqueue_struct *otg_wq;
	struct delayed_work chg_work;
	struct delayed_work id_status_work;
	enum usb_chg_state chg_state;
	enum usb_chg_type chg_type;
	bool chg_detection;
	unsigned int dcd_time;
	unsigned long caps;
	uint32_t bus_perf_client;
	bool host_bus_suspend;
	bool device_bus_suspend;
	bool bus_clks_enabled;
	/*
	 * Allowing PHY power collpase turns off the HSUSB 3.3v and 1.8v
	 * analog regulators while going to low power mode.
	 * Currently only 28nm PHY has the support to allowing PHY
	 * power collapse since it doesn't have leakage currents while
	 * turning off the power rails.
	 */
#define ALLOW_PHY_POWER_COLLAPSE	BIT(0)
	/*
	 * Allow PHY RETENTION mode before turning off the digital
	 * voltage regulator(VDDCX).
	 */
#define ALLOW_PHY_RETENTION		BIT(1)
	/*
	 * Allow putting the core in Low Power mode, when
	 * USB bus is suspended but cable is connected.
	 */
#define ALLOW_LPM_ON_DEV_SUSPEND	BIT(2)
	/*
	 * Allowing PHY regulators LPM puts the HSUSB 3.3v and 1.8v
	 * analog regulators into LPM while going to USB low power mode.
	 */
#define ALLOW_PHY_REGULATORS_LPM	BIT(3)
	/*
	 * Allow PHY RETENTION mode before turning off the digital
	 * voltage regulator(VDDCX) during host mode.
	 */
#define ALLOW_HOST_PHY_RETENTION	BIT(4)
	/*
	 * Allow VDD minimization without putting PHY into retention
	 * for fixing PHY current leakage issue when LDOs ar turned off.
	 */
#define ALLOW_VDD_MIN_WITH_RETENTION_DISABLED BIT(5)

	/*
	 * PHY can keep D+ pull-up during peripheral bus suspend and
	 * D+/D- pull-down during host bus suspend without any
	 * re-work. This is possible only when PHY DVDD is supplied
	 * by a PMIC LDO (unlike VDDCX/VDDMX).
	 */
#define ALLOW_BUS_SUSPEND_WITHOUT_REWORK BIT(6)
	unsigned long lpm_flags;
#define PHY_PWR_COLLAPSED		BIT(0)
#define PHY_RETENTIONED			BIT(1)
#define XO_SHUTDOWN			BIT(2)
#define CLOCKS_DOWN			BIT(3)
#define PHY_REGULATORS_LPM	BIT(4)
	int reset_counter;
	unsigned int online;

	dev_t ext_chg_dev;
	struct pinctrl *phy_pinctrl;
	bool is_ext_chg_dcp;
	struct qpnp_vadc_chip	*vadc_dev;
	int ext_id_irq;
	bool phy_irq_pending;
	enum usb_id_state id_state;
	bool rm_pulldown;
	struct extcon_dev       *extcon_vbus;
	struct extcon_dev       *extcon_id;
	struct notifier_block   vbus_nb;
	struct notifier_block   id_nb;
	bool			extcon_registered;
	struct regulator_desc	dpdm_rdesc;
	struct regulator_dev	*dpdm_rdev;
/* Maximum debug message length */
#define DEBUG_MSG_LEN   128UL
/* Maximum number of messages */
#define DEBUG_MAX_MSG   256UL
	unsigned int dbg_idx;
	rwlock_t dbg_lock;

	char (buf[DEBUG_MAX_MSG])[DEBUG_MSG_LEN];   /* buffer */
	unsigned int vbus_state;
	unsigned int usb_irq_count;
	int pm_qos_latency;
	unsigned int notify_current_mA;
	struct pm_qos_request pm_qos_req_dma;
	struct delayed_work perf_vote_work;
	struct delayed_work sdp_check;
	struct work_struct notify_charger_work;
	struct work_struct extcon_register_work;
	struct notifier_block psy_nb;
};

struct ci13xxx_platform_data {
	u8 usb_core_id;
	/*
	 * value of 2^(log2_itc-1) will be used as the interrupt threshold
	 * (ITC), when log2_itc is between 1 to 7.
	 */
	int log2_itc;
	bool l1_supported;
	bool enable_ahb2ahb_bypass;
	bool enable_streaming;
	bool enable_axi_prefetch;
};

#ifdef CONFIG_USB_BAM
void msm_bam_set_usb_host_dev(struct device *dev);
int msm_do_bam_disable_enable(enum usb_ctrl ctrl);
#else
static inline void msm_bam_set_usb_host_dev(struct device *dev) {}
int msm_do_bam_disable_enable(enum usb_ctrl ctrl) { return true; }
#endif
#ifdef CONFIG_USB_CI13XXX_MSM
void msm_hw_soft_reset(void);
#else
static inline void msm_hw_soft_reset(void)
{
}
#endif

#endif
