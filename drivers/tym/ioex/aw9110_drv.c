/* kernel: 4.1.9
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <stdbool.h>
#include <linux/delay.h>


#define SET_BITS(unit,bits)  	(unit |= bits)
#define RESET_BITS(unit,bits)  	(unit &= ~bits)

/* gpio subsystem */
#define AW9110_GPIO_BASE 200
#define MAX_PINS 10

/* ------------------------------------------------------------------------------------------------ */
/* aw9110 registers description */

//for REG_xx_A, REG_xx_B
#define AW9110_BIT_PIN_OFFSET               4
#define AW9110_MAX_BRE_PIN                  5   // OUT0~OUT5: support intelligent breathing mode

//API: pin: range: 0-9
#define GET_PIN_BIT(pin)                    BIT((pin >= AW9110_BIT_PIN_OFFSET) ? (pin - AW9110_BIT_PIN_OFFSET) : pin)
#define ONE_OF_PORT(pin,portA,portB)        ((pin >= AW9110_BIT_PIN_OFFSET) ? portA : portB)
#define AW9110_RESET_PORT_BIT(val,pin)      (val &= ~GET_PIN_BIT(pin))
#define AW9110_SET_PORT_BIT(val, pin)       (val |= GET_PIN_BIT(pin))

#define AW9110_REG_GPIO_OUTPUT_A            0x02
#define AW9110_REG_GPIO_OUTPUT_B            0x03
    #define GET_OUTPUT_REG(pin)             ONE_OF_PORT(pin, AW9110_REG_GPIO_OUTPUT_A, AW9110_REG_GPIO_OUTPUT_B)

//gpio mode: 0: output; 1: input
//led mode: 0: smart-fade mode; 1: BLINK mode
#define AW9110_REG_CFG_A                    0x04
#define AW9110_REG_CFG_B                    0x05
    #define AW9110_REG_CFG_DEFAULT_VAL      0x00
    #define GET_CFG_REG(pin)                ONE_OF_PORT(pin, AW9110_REG_CFG_A, AW9110_REG_CFG_B)

#define AW9110_REG_CTRL                     0x11
#define AW9110_REG_GPMD_A                   0x12
#define AW9110_REG_GPMD_B                   0x13
    #define AW9110_REG_GPMD_DEFAULT_VAL     0xff
    #define GET_MODE_REG(pin)               ONE_OF_PORT(pin, AW9110_REG_GPMD_A, AW9110_REG_GPMD_B)

#define AW9110_REG_BRE_EN                   0x14  //bit[0-5]: pin0 - pin5
#define AW9110_REG_FADE_TMR                 0x15
#define AW9110_REG_FULL_TMR                 0x16
// #define AW9110_REG_DLY_BRE_BASE        0x17  //delay time after enable breathing mode. Not used.
#define AW9110_REG_DIM_BASE                 0x20  //brightness. 0x20-0x29: pin0 - pin9

#define AW9110_REG_RESET                    0x7F
    #define AW9110_VAL_RESET                0x00


typedef enum {
    OUTPUT_MODE_OPEN_DRAIN = 0,
    OUTPUT_MODE_PUSH_PULL,
} eOutputMode; //for AW9110_REG_GPMD_A, AW9110_REG_GPMD_B


typedef enum {
    PIN_MODE_LED = 0,
    PIN_MODE_GPIO, //default
} ePinMode; //for AW9110_REG_GPMD_A, AW9110_REG_GPMD_B

typedef enum {
    MAX_I_37_MA = 0,
    MAX_I_27p75_MA,
    MAX_I_18p5_MA,
    MAX_I_9p25_MA,
} eMaxI; //for AW9110_BIT_DIM_RANGE

typedef enum {
    FADE_MODE_SMART_FADE_MODE,
    FADE_MODE_BLINK_MODE,
} eFadeMode; //for AW9110_REG_CFG_A, AW9110_REG_CFG_A, pin0 - pin5

typedef enum {
    FADE_TIME_0_MS = 0,
    FADE_TIME_315_MS,
    FADE_TIME_630_MS,
    FADE_TIME_1260_MS,
    FADE_TIME_2520_MS,
    FADE_TIME_5040_MS,
    FADE_TIME_MAX,
} eFadeTime; //3bits, for AW9110_REG_FADE_TMR

typedef enum {
    FULL_TIME_0_MS = 0,
    FULL_TIME_315_MS,
    FULL_TIME_630_MS,
    FULL_TIME_1260_MS,
    FULL_TIME_2520_MS,
    FULL_TIME_5040_MS,
    FULL_TIME_10080_MS,
    FULL_TIME_20160_MS,
    FULL_TIME_MAX,
} eFullTime; //3bits, for AW9110_REG_FULL_TMR


struct aw9110_reg {
    /* public */
    union
    {
        struct {
            eMaxI max_I :2;
            unsigned char reserved0 :2;
            eOutputMode output_mode :1;
            unsigned char reserved :2;
            bool blink_en :1;
        } reg_dscp;
        unsigned char val;
    } global_ctrl; //AW9110_REG_CTRL

    union
    {
        struct {
            eFadeTime fadeOn_time :3;
            eFadeTime fadeOff_time :3;
            unsigned char reserved :2;
        } reg_dscp;
        unsigned char val;
    } fade_timer; //AW9110_REG_FADE_TMR

    union
    {
        struct {
            eFadeTime fullOn_time :3;
            eFadeTime fullOff_time :3;
            unsigned char reserved :2;
        } reg_dscp;
        unsigned char val;
    } full_timer; //AW9110_REG_FULL_TMR

    /* private */
    unsigned char breath_en;
    unsigned char config_portA;
    unsigned char config_portB;
    unsigned char gpmode_portA;
    unsigned char gpmode_portB;
    unsigned char output_portA;
    unsigned char output_portB;
};

struct aw9110_drv {
    struct gpio_chip    *gpio_chip;
    struct device       *dev;
    struct regmap       *regmap;
    struct mutex        lock;
    struct led_trigger  *trigger;

    struct aw9110_led {
        struct aw9110_drv       *me;
        struct led_classdev     cdev;
    } leds[MAX_PINS];

    struct aw9110_reg  regs;
    bool pin_configured[MAX_PINS];
};


// static int aw9110_read_reg(struct aw9110_drv *me, unsigned char reg, unsigned char *pVal)

static int aw9110_write_reg(struct aw9110_drv *me, unsigned char reg, unsigned char val)
{
    return regmap_write(me->regmap, (unsigned int)reg, (unsigned int)val);
}

/* ------------------------------------------------------------------------------------------------ */
/* registers opr */

static void aw9110_reg_init(struct aw9110_drv* me)
{
    //AD0 & AD1 high: default: pin0-pin3: 1, pin4-pin8: Hi-Z
    aw9110_write_reg(me, AW9110_REG_RESET, AW9110_VAL_RESET);

    // breathing pattern:
    me->regs.fade_timer.reg_dscp.fadeOn_time = FADE_TIME_1260_MS;
    me->regs.full_timer.reg_dscp.fullOn_time = FULL_TIME_0_MS;
    me->regs.fade_timer.reg_dscp.fadeOff_time = FADE_TIME_1260_MS;
    me->regs.full_timer.reg_dscp.fullOff_time = FULL_TIME_0_MS;
    aw9110_write_reg(me, AW9110_REG_FADE_TMR, me->regs.fade_timer.val);
    aw9110_write_reg(me, AW9110_REG_FULL_TMR, me->regs.full_timer.val);

    me->regs.global_ctrl.reg_dscp.max_I = MAX_I_18p5_MA;
    me->regs.global_ctrl.reg_dscp.output_mode = OUTPUT_MODE_PUSH_PULL;
    me->regs.global_ctrl.reg_dscp.blink_en = true;
    aw9110_write_reg(me, AW9110_REG_CTRL, me->regs.global_ctrl.val);
}

static inline int aw9110_reg_blink_go(struct aw9110_drv* me)
{
    return aw9110_write_reg(me, AW9110_REG_CTRL, me->regs.global_ctrl.val);
}

static inline int aw9110_reg_set_brightness(struct aw9110_drv *me, unsigned int pin, unsigned int brightness)
{
    return aw9110_write_reg(me, AW9110_REG_DIM_BASE + pin, brightness);
}

static int aw9110_reg_set_pin_mode(struct aw9110_drv* me, unsigned int pin, ePinMode mode)
{
    unsigned char *pVal = ONE_OF_PORT(pin, &me->regs.gpmode_portA, &me->regs.gpmode_portB);
    (mode == PIN_MODE_LED) ? AW9110_RESET_PORT_BIT(*pVal, pin) : AW9110_SET_PORT_BIT(*pVal, pin);
    return aw9110_write_reg(me, GET_MODE_REG(pin), *pVal);
}

static int aw9110_reg_config_led_mode(struct aw9110_drv *me, unsigned int pin, eFadeMode mode)
{
    unsigned char *pVal = ONE_OF_PORT(pin, &me->regs.config_portA, &me->regs.config_portB);
    (mode == FADE_MODE_SMART_FADE_MODE) ? AW9110_RESET_PORT_BIT(*pVal, pin) : AW9110_SET_PORT_BIT(*pVal, pin);
    return aw9110_write_reg(me, GET_CFG_REG(pin), *pVal);
}

static int aw9110_reg_pin_blink_mode_en(struct aw9110_drv* me, unsigned int pin, bool en)
{
    unsigned char *pVal = &me->regs.breath_en;
    int ret = 0;

    if(pin <= AW9110_MAX_BRE_PIN)
    {
        (en == true) ? SET_BITS(*pVal, BIT(pin)) : RESET_BITS(*pVal, BIT(pin));
        ret |= aw9110_write_reg(me, AW9110_REG_BRE_EN, *pVal);
        ret |= aw9110_reg_blink_go(me);
        return ret;
    }
    else
        return -EINVAL;
}

static int aw9110_reg_set_gpio(struct aw9110_drv* me, unsigned int pin, bool value)
{
    unsigned char *pVal = ONE_OF_PORT(pin, &me->regs.output_portA, &me->regs.output_portB);
    (value == 1) ? AW9110_SET_PORT_BIT(*pVal, pin) : AW9110_RESET_PORT_BIT(*pVal, pin);
    return aw9110_write_reg(me, GET_OUTPUT_REG(pin), *pVal);
}

/* ------------------------------------------------------------------------------------------------ */
/* gpio subsystem */

static int aw9110_gpio_request(struct gpio_chip *chip, unsigned offset)
{
    struct aw9110_drv *me = gpiochip_get_data(chip);
    if(me->pin_configured[offset] == true)
    {
        return -EBUSY;
    }
    return 0;
}

static void aw9110_gpio_free(struct gpio_chip *chip, unsigned offset)
{
}

static int aw9110_direction_input(struct gpio_chip *chip, unsigned offset)
{
    return 0; //not support
}

static int aw9110_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
    return 0; //not support
}

static int aw9110_get(struct gpio_chip *chip, unsigned offset)
{
    return 0; //not support
}

static void aw9110_set(struct gpio_chip *chip, unsigned offset, int value)
{
    struct aw9110_drv *me = gpiochip_get_data(chip);

    mutex_lock(&me->lock);
    aw9110_reg_set_gpio(me, offset, (bool)value);
    mutex_unlock(&me->lock);
}


struct gpio_chip aw9110_gpio_chip = {
    .label = "aw9110-gpiochip",
    .base = AW9110_GPIO_BASE,
    .ngpio = MAX_PINS,
    .request = aw9110_gpio_request,
    .free = aw9110_gpio_free,

    .get = aw9110_get,
    .set = aw9110_set,
    .direction_input = aw9110_direction_input,
    .direction_output = aw9110_direction_output,
};

/* ------------------------------------------------------------------------------------------------ */
/* led subsystem */

static void aw9110_brightness_set(struct led_classdev *cdev, enum led_brightness brightness)
{
    struct aw9110_led *led = container_of(cdev, struct aw9110_led, cdev);
    struct aw9110_drv *me = led->me;
    int pin = led - me->leds;

    mutex_lock(&me->lock);
    aw9110_reg_set_brightness(me, pin, brightness);
    mutex_unlock(&me->lock);
}

#ifdef CONFIG_LEDS_TRIGGERS
static int aw9110_led_trigger_activate(struct led_classdev *cdev)
{
    struct aw9110_led *led = container_of(cdev, struct aw9110_led, cdev);
    struct aw9110_drv *me = led->me;
    int pin = led - me->leds;

    mutex_lock(&me->lock);
    aw9110_reg_pin_blink_mode_en(me, pin, true);
    mutex_unlock(&me->lock);
    return 0;
}

static void aw9110_led_trigger_deactivate(struct led_classdev *cdev)
{
    struct aw9110_led *led = container_of(cdev, struct aw9110_led, cdev);
    struct aw9110_drv *me = led->me;
    int pin = led - me->leds;

    mutex_lock(&me->lock);
    aw9110_reg_pin_blink_mode_en(me, pin, false);
    mutex_unlock(&me->lock);
}

static struct led_trigger aw9110_trigger = {
    .name = "breathing",
    .activate = aw9110_led_trigger_activate,
    .deactivate = aw9110_led_trigger_deactivate,
};
#endif

/* ------------------------------------------------------------------------------------------------ */
static int aw9110_parse_dt_and_led_register(struct device *dev, struct aw9110_drv* me)
{
    struct device_node *np = dev->of_node, *child;
    int ret;

    // int count = of_get_child_count(np);
    // printk("of_get_child_count: %d\n", count);

    for_each_child_of_node(np, child) {
        struct aw9110_led *led;
        unsigned int pin;

        int gpio_id = of_get_named_gpio(child, "led-gpios", 0);
        pin = gpio_id - AW9110_GPIO_BASE;

        led = &me->leds[pin];

        ret = of_property_read_string(child, "label", &led->cdev.name);
        if(ret < 0)
        {
            printk(KERN_INFO "aw9110: of_property_read_string(): ret=%d\n", ret);
            return ret;
        }

        led->me = me;
        led->cdev.brightness_set = aw9110_brightness_set;
        me->pin_configured[pin] = true;

        aw9110_reg_set_pin_mode(me, pin, PIN_MODE_LED);
        aw9110_reg_config_led_mode(me, pin, FADE_MODE_BLINK_MODE);
        // default: led off
        aw9110_reg_set_brightness(me, pin, 0);

        ret = devm_led_classdev_register(dev, &led->cdev);
        if(ret < 0)
        {
            printk(KERN_INFO "aw9110: devm_led_classdev_register(): failed.\n");
            return ret;
        }
    }

    return 0;
}

static struct regmap_config regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = 0xff,
};

static int aw9110_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct aw9110_drv *me;
    int ret;

    me = devm_kzalloc(&client->dev, sizeof(*me), GFP_KERNEL);
    if (!me)
        return -ENOMEM;

    mutex_init(&me->lock);

    /* reg value init */
    me->regs.gpmode_portA = AW9110_REG_GPMD_DEFAULT_VAL;
    me->regs.gpmode_portB = AW9110_REG_GPMD_DEFAULT_VAL;
    me->regs.output_portA = AW9110_REG_CFG_DEFAULT_VAL;
    me->regs.output_portB = AW9110_REG_CFG_DEFAULT_VAL;
    me->regs.breath_en = 0;

    me->regmap = devm_regmap_init_i2c(client, &regmap_config);
    if (IS_ERR(me->regmap))
    {
        ret = PTR_ERR(me->regmap);
        goto free_mutex;
    }

    aw9110_reg_init(me);

    /* gpio subsystem */
    me->gpio_chip = &aw9110_gpio_chip;
    me->gpio_chip->parent = &client->dev;
    me->gpio_chip->owner = THIS_MODULE;
    ret = devm_gpiochip_add_data(&client->dev, me->gpio_chip, me);
    if(ret < 0)
    {
        printk(KERN_INFO "aw9110_probe: gpiochip_add(): failed.\n");
        return ret;
    }

    /* led subsystem */
    ret = aw9110_parse_dt_and_led_register(&client->dev, me);
    if(ret < 0)
    {
        return ret;
    }
#ifdef CONFIG_LEDS_TRIGGERS
    me->trigger = &aw9110_trigger;
    led_trigger_register(me->trigger);
#endif

    i2c_set_clientdata(client, me);

    printk(KERN_INFO "aw9110_probe: success\n");
    return 0;

free_mutex:
    mutex_destroy(&me->lock);
    return ret;
}

static int aw9110_remove(struct i2c_client *client)
{
    struct aw9110_drv *me = i2c_get_clientdata(client);
    // aw200xx_reset(aw);
#ifdef CONFIG_LEDS_TRIGGERS
    led_trigger_unregister(me->trigger);
#endif
    mutex_destroy(&me->lock);
    printk(KERN_INFO "aw9110_removed\n");
    return 0;
}

static const struct of_device_id aw9110_of_match[] = {
    { .compatible = "awinic,aw9110", },
    { /* sentinel */ }
};

static const struct i2c_device_id aw9110_i2c_id[] = {
    { "aw9110" },
    { /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, aw9110_i2c_id);

static struct i2c_driver aw9110_i2c_driver = {
    .driver = {
        .name    = "aw9110",
        .of_match_table = of_match_ptr(aw9110_of_match),
    },
    .probe       = aw9110_probe,
    .remove      = aw9110_remove,
    .id_table    = aw9110_i2c_id,
};

module_i2c_driver(aw9110_i2c_driver);
MODULE_LICENSE("GPL");
