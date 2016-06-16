#ifndef ___RT9536_CHARGER_H
#define __RT9536_CHARGER_H

#define RT9536_DEBUG(format, args...) pr_debug(format, ##args)
#define RT9536_INFO(format, args...) pr_warn(format, ##args)
#define RT9536_ERROR(format, args...) pr_err(format, ##args)

enum rt9536_mode {
	RT9536_MODE_USB500,
	RT9536_MODE_ISET,
	RT9536_MODE_USB100,
	RT9536_MODE_PTM,
	RT9536_MODE_DENACTIVE,
	RT9536_MODE_UNKNOWN
};

enum rt9536_pin_level {
	RT9536_PIN_ZERO,
	RT9536_PIN_ONE
};

struct rt9536_info {
	struct platform_device *pdev;
	struct power_supply psy;

	/* gpio */
	int en_set;
	int chgsb;
	int pgb;

	struct pinctrl *pin;
	struct pinctrl_state *boot;
	struct pinctrl_state *charging;
	struct pinctrl_state *not_charging;

	/* current */
	int iset;

	/* status */
	int status;
	int enable;
	enum rt9536_mode mode;

	/* lock */
	struct mutex mode_lock;
	spinlock_t pulse_lock;

};


/* Function Prototype */
enum power_supply_type get_rt9536_status(void);

extern void rt9536_active_default(void);
extern void rt9536_deactive(void);
extern void rt9536_charging_enable(unsigned int set_current, unsigned int enable);
extern unsigned char rt9536_check_eoc_status(void);

#endif				/* __LINUX_MAX8971_CHARGER_H */
