/*

 */

#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "clk-si5340d_audio.h"

struct si5340d_driver_data;

struct si5340d_hw_data {
	struct clk_hw			hw;
	struct si5340d_driver_data	*drvdata;
	unsigned char			num;
	int    pll_master;
};

struct si5340d_driver_data {
	struct i2c_client	*client;
	struct regmap		*regmap;

	struct si5340d_hw_data	*msynth;
	struct si5340d_hw_data	*clkout;
	size_t			num_clkout;
	int		clkmode;
};

static const char * const si5340d_msynth_names[] = {
	"ms0"
};
static const char * const si5340d_clkout_names[] = {
	"clk0", "clk1", "clk2", "clk3"
};

/*
 * Si5351 i2c regmap
 */
static inline u8 si5340d_reg_read(struct si5340d_driver_data *drvdata, u16 reg)
{
	u32 val;
	int ret;

	ret = regmap_read(drvdata->regmap, reg, &val);
	if (ret) {
		dev_err(&drvdata->client->dev,
			"unable to read from reg%02x\n", reg);
		return 0;
	}

	return (u8)val;
}

static inline int si5340d_bulk_read(struct si5340d_driver_data *drvdata,
				   u16 reg, u8 count, u8 *buf)
{
	return regmap_bulk_read(drvdata->regmap, reg, buf, count);
}

static inline int si5340d_reg_write(struct si5340d_driver_data *drvdata,
				   u16 reg, u8 val)
{
	return regmap_write(drvdata->regmap, reg, val);
}

static inline int si5340d_bulk_write(struct si5340d_driver_data *drvdata,
				    u16 reg, u8 count, const u8 *buf)
{
	return regmap_raw_write(drvdata->regmap, reg, buf, count);
}

static inline int si5340d_block_write(struct si5340d_driver_data *drvdata,
				    const si5340_revd_register_t *table, int count)
{
	int i;
	for( i = 0; i < count; i++){
		si5340d_reg_write(drvdata, table[i].address, table[i].value);
	}
	return 0;
}


static inline int si5340d_set_bits(struct si5340d_driver_data *drvdata,
				  u16 reg, u8 mask, u8 val)
{
	return regmap_update_bits(drvdata->regmap, reg, mask, val);
}

static inline u16 si5340d_rdiv_params_address(int num)
{
	switch(num){
	case 0:
		return SI5340_R0_REG;
		break;
	case 1:
		return SI5340_R1_REG;
		break;
	case 2:
		return SI5340_R2_REG;
		break;
	case 3:
		return SI5340_R3_REG;
		break;
	}
	return 0;
}

static inline u16 si5340d_clkcnf_params_address(int num)
{
	switch(num){
	case 0:
		return SI5340_CLKOUT0_CONFIG;
		break;
	case 1:
		return SI5340_CLKOUT1_CONFIG;
		break;
	case 2:
		return SI5340_CLKOUT2_CONFIG;
		break;
	case 3:
		return SI5340_CLKOUT3_CONFIG;
		break;
	}
	return 0;
}

#if 0
static bool si5340d_regmap_is_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SI5351_DEVICE_STATUS:
	case SI5351_INTERRUPT_STATUS:
	case SI5351_PLL_RESET:
		return true;
	}
	return false;
}

static bool si5340d_regmap_is_writeable(struct device *dev, unsigned int reg)
{
	/* reserved registers */
	if (reg >= 4 && reg <= 8)
		return false;
	if (reg >= 10 && reg <= 14)
		return false;
	if (reg >= 173 && reg <= 176)
		return false;
	if (reg >= 178 && reg <= 182)
		return false;
	/* read-only */
	if (reg == SI5351_DEVICE_STATUS)
		return false;
	return true;
}
#endif
static const struct regmap_range_cfg si5340d_range = {
	.name = "Pages",
	.range_min = 0,
	.range_max = SI5340_REVD_REG_CONFIG_MAX_REGS,
	.selector_reg = SI5340_PAGEREG,
	.selector_mask = 0xff,
	.window_start = 0, .window_len = 0x100,
};

static const struct regmap_config si5340d_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,

	.ranges = &si5340d_range,
	.num_ranges = 1,
	.max_register = SI5340_REVD_REG_CONFIG_MAX_REGS,

//	.writeable_reg = si5340d_regmap_is_writeable,
//	.volatile_reg = si5340d_regmap_is_volatile,
};




/*
 * Si5340d multisync divider
 * for audio apl
 */


static unsigned long si5340d_msynth_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct si5340d_hw_data *hwdata =
		container_of(hw, struct si5340d_hw_data, hw);
	unsigned long long rate = 0;

	if ( hwdata->drvdata->clkmode == 1){
		rate = SI5340D_REVD_44_FREQ;
	}else
	if ( hwdata->drvdata->clkmode == 0){
		rate = SI5340D_REVD_48_FREQ;
	}

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: parent_rate = %lu, rate = %lu\n",
		__func__, clk_hw_get_name(hw),
		parent_rate, (unsigned long)rate);

	return (unsigned long)rate;
}

static long si5340d_msynth_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	struct si5340d_hw_data *hwdata =
		container_of(hw, struct si5340d_hw_data, hw);
	unsigned long rdiv;

	/* request frequency if multisync master */
	if (!(SI5340D_REVD_48_FREQ % rate)) {
		*parent_rate = SI5340D_REVD_48_FREQ;
	}else
	if (!(SI5340D_REVD_44_FREQ % rate)) {
		*parent_rate = SI5340D_REVD_44_FREQ;
	}
	rdiv = (*parent_rate / rate) & (~0x01);
	rate = *parent_rate;

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: rdiv = %lu, parent_rate = %lu, rate = %lu\n",
		__func__, clk_hw_get_name(hw), rdiv,
		*parent_rate, rate);

	return rate;
}

static int si5340d_msynth_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct si5340d_hw_data *hwdata =
		container_of(hw, struct si5340d_hw_data, hw);

	if ( rate == SI5340D_REVD_48_FREQ && hwdata->drvdata->clkmode == 1){
		si5340d_block_write(hwdata->drvdata, si5340_revd_48_base , sizeof(si5340_revd_48_base) /sizeof(si5340_revd_register_t));
		hwdata->drvdata->clkmode = 0;
	}else
	if ( rate == SI5340D_REVD_44_FREQ && hwdata->drvdata->clkmode == 0){
		si5340d_block_write(hwdata->drvdata, si5340_revd_44_base , sizeof(si5340_revd_44_base) /sizeof(si5340_revd_register_t));
		hwdata->drvdata->clkmode = 1;
	}

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: parent_rate = %lu, rate = %lu \n",
		__func__, clk_hw_get_name(hw), parent_rate, rate);

	return 0;
}

static const struct clk_ops si5340d_msynth_ops = {
	.recalc_rate = si5340d_msynth_recalc_rate,
	.round_rate = si5340d_msynth_round_rate,
	.set_rate = si5340d_msynth_set_rate,
};

/*
 * Si5351 clkout divider
 */
static int _si5340d_clkout_reparent(struct si5340d_driver_data *drvdata,
				   int num, int parent)
{
	return 0;
}

static int si5340d_clkout_prepare(struct clk_hw *hw)
{
	struct si5340d_hw_data *hwdata =
		container_of(hw, struct si5340d_hw_data, hw);
	u16 reg = si5340d_clkcnf_params_address(hwdata->num);


//	si5340d_set_bits(hwdata->drvdata, reg, SI5340_CONFIG_CLKPDN, 0);
	si5340d_set_bits(hwdata->drvdata, reg, SI5340_CONFIG_CLKOE, SI5340_CONFIG_CLKOE);
	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: %04x:%02x \n",	__func__, clk_hw_get_name(hw), reg, SI5340_CONFIG_CLKOE);
	return 0;
}

static void si5340d_clkout_unprepare(struct clk_hw *hw)
{
	struct si5340d_hw_data *hwdata =
		container_of(hw, struct si5340d_hw_data, hw);
	u16 reg = si5340d_clkcnf_params_address(hwdata->num);

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s:  \n",	__func__, clk_hw_get_name(hw));

//	si5340d_set_bits(hwdata->drvdata, reg, SI5340_CONFIG_CLKPDN, SI5340_CONFIG_CLKPDN);
	si5340d_set_bits(hwdata->drvdata, reg, SI5340_CONFIG_CLKOE, 0);
}

static u8 si5340d_clkout_get_parent(struct clk_hw *hw)
{
	return 0;
}

static int si5340d_clkout_set_parent(struct clk_hw *hw, u8 index)
{
	return 0;
}

static unsigned long si5340d_clkout_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct si5340d_hw_data *hwdata =
		container_of(hw, struct si5340d_hw_data, hw);
	u16 reg;
	u8  param[3];
	unsigned long rdiv;

	reg = si5340d_rdiv_params_address(hwdata->num);
	si5340d_bulk_read(hwdata->drvdata, reg, 3, param);

	rdiv = param[0] | param[1] << 8 | param[2] << 16;
	rdiv = (rdiv + 1) << 1;

	return parent_rate / rdiv;
}

static long si5340d_clkout_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	struct si5340d_hw_data *hwdata =
		container_of(hw, struct si5340d_hw_data, hw);
	unsigned long rdiv;

	/* request frequency if multisync master */
	if (clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT) {
		dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: pll-master\n", __func__, clk_hw_get_name(hw) );
		if (!(SI5340D_REVD_48_FREQ % rate)) {
			*parent_rate = SI5340D_REVD_48_FREQ;
		}else
		if (!(SI5340D_REVD_44_FREQ % rate)) {
			*parent_rate = SI5340D_REVD_44_FREQ;
		}
	}
	rdiv = (*parent_rate / rate) & (~0x01);
	rate = *parent_rate / rdiv;

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: rdiv = %lu, parent_rate = %lu, rate = %lu\n",
		__func__, clk_hw_get_name(hw), rdiv,
		*parent_rate, rate);

	return rate;
}

static int si5340d_clkout_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct si5340d_hw_data *hwdata =
		container_of(hw, struct si5340d_hw_data, hw);
	unsigned long rerr, rdiv, rreg;
	u16 reg;
	u8  param[3];

	/* round to closed rdiv */
	rerr = parent_rate % rate;
	rdiv = parent_rate / rate;
	rreg = (rdiv >> 1) - 1;

	/* write output divider */
	reg = si5340d_rdiv_params_address(hwdata->num);
	param[0] = rreg & 0xff;
	param[1] = rreg >> 8  & 0xff;
	param[2] = rreg >> 16 & 0xff;
	si5340d_bulk_write(hwdata->drvdata, reg, 3, param);

	/* powerup clkout */
	reg = si5340d_clkcnf_params_address(hwdata->num);
	si5340d_set_bits(hwdata->drvdata, reg, SI5340_CONFIG_CLKPDN, 0);

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: rdiv = %lu, parent_rate = %lu, rate = %lu, err = %lu\n",
		__func__, clk_hw_get_name(hw), rdiv,
		parent_rate, rate, rerr);

	return 0;
}

static const struct clk_ops si5340d_clkout_ops = {
	.prepare = si5340d_clkout_prepare,
	.unprepare = si5340d_clkout_unprepare,
	.set_parent = si5340d_clkout_set_parent,
	.get_parent = si5340d_clkout_get_parent,
	.recalc_rate = si5340d_clkout_recalc_rate,
	.round_rate = si5340d_clkout_round_rate,
	.set_rate = si5340d_clkout_set_rate,
};

/*
 * Si5351 i2c probe and DT
 */
#ifdef CONFIG_OF
static const struct of_device_id si5340d_dt_ids[] = {
	{ .compatible = "silabs,si5340d_audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, si5340d_dt_ids);

static int si5340d_dt_parse(struct i2c_client *client , struct si5340d_hw_data *clkout)
{
	struct device_node *child, *np = client->dev.of_node;
	int num = 0;

	if (np == NULL)
		return 0;

	/* per clkout properties */
	for_each_child_of_node(np, child) {
		if (of_property_read_u32(child, "reg", &num)) {
			dev_err(&client->dev, "missing reg property of %s\n",
				child->name);
			goto put_child;
		}

		if (num >= 4) {
			dev_err(&client->dev, "invalid clkout %d\n", num);
			goto put_child;
		}

		clkout[num].pll_master =
			of_property_read_bool(child, "silabs,pll-master");
	}

	return 0;
put_child:
	of_node_put(child);
	return -EINVAL;
}

static struct clk_hw *
si5340_of_clk_get(struct of_phandle_args *clkspec, void *data)
{
	struct si5340d_driver_data *drvdata = data;
	unsigned int idx = clkspec->args[0];

	if (idx >= drvdata->num_clkout) {
		pr_err("%s: invalid index %u\n", __func__, idx);
		return ERR_PTR(-EINVAL);
	}

	return &drvdata->clkout[idx].hw;
}
#else
static int si5340d_dt_parse(struct i2c_client *client, enum si5340d_variant variant)
{
	return 0;
}

static struct clk_hw *
si5340_of_clk_get(struct of_phandle_args *clkspec, void *data)
{
	return NULL;
}
#endif /* CONFIG_OF */

static int si5340d_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
//	struct si5340d_platform_data *pdata;
	struct si5340d_driver_data *drvdata;
	struct clk_init_data init;
	const char *parent_names[4];
	u8 num_parents, num_clocks;
	int ret, n;
	unsigned int val;

	drvdata = devm_kzalloc(&client->dev, sizeof(*drvdata), GFP_KERNEL);
	if (drvdata == NULL) {
		dev_err(&client->dev, "unable to allocate driver data\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, drvdata);
	drvdata->client = client;

	drvdata->regmap = devm_regmap_init_i2c(client, &si5340d_regmap_config);
	if (IS_ERR(drvdata->regmap)) {
		dev_err(&client->dev, "failed to allocate register map\n");
		return PTR_ERR(drvdata->regmap);
	}

	// write default
	si5340d_block_write(drvdata, si5340_revd_preamble , sizeof(si5340_revd_preamble) /sizeof(si5340_revd_register_t));
	usleep_range(300000, 500000);
	si5340d_block_write(drvdata, si5340_revd_config   , sizeof(si5340_revd_config)   /sizeof(si5340_revd_register_t));
	si5340d_block_write(drvdata, si5340_revd_postamble, sizeof(si5340_revd_postamble)/sizeof(si5340_revd_register_t));

	/* register clk multisync and clk out divider */
	num_clocks = 4;

	drvdata->msynth = devm_kzalloc(&client->dev, 1 *
				       sizeof(*drvdata->msynth), GFP_KERNEL);
	drvdata->clkout = devm_kzalloc(&client->dev, num_clocks *
				       sizeof(*drvdata->clkout), GFP_KERNEL);
	drvdata->num_clkout = num_clocks;

	if (WARN_ON(!drvdata->msynth || !drvdata->clkout)) {
		ret = -ENOMEM;
		goto err_clk;
	}

	ret = si5340d_dt_parse(client, drvdata->clkout);
	if (ret)
		return ret;

	{
		int n = 0;
		drvdata->msynth[n].num = n;
		drvdata->msynth[n].drvdata = drvdata;
		drvdata->msynth[n].hw.init = &init;
		memset(&init, 0, sizeof(init));
		init.name = si5340d_msynth_names[n];
		init.ops = &si5340d_msynth_ops;
		init.flags = 0;
//		if (pdata->clkout[n].pll_master)
			init.flags |= CLK_SET_RATE_PARENT;
//		init.parent_names = parent_names;
//		init.num_parents = 2;
		init.parent_names = NULL;
		init.num_parents = 0;
		ret = devm_clk_hw_register(&client->dev,
					   &drvdata->msynth[n].hw);
		if (ret) {
			dev_err(&client->dev, "unable to register %s\n",
				init.name);
			goto err_clk;
		}
	}

	num_parents = 4;
	for (n = 0; n < num_clocks; n++) {
		parent_names[0] = si5340d_msynth_names[0];

		drvdata->clkout[n].num = n;
		drvdata->clkout[n].drvdata = drvdata;
		drvdata->clkout[n].hw.init = &init;
		memset(&init, 0, sizeof(init));
		init.name = si5340d_clkout_names[n];
		init.ops = &si5340d_clkout_ops;
		init.flags = 0;
//		if (pdata->clkout[n].clkout_src == SI5351_CLKOUT_SRC_MSYNTH_N)
//			init.flags |= CLK_SET_RATE_PARENT;
		if (drvdata->clkout[n].pll_master) {
			dev_dbg(&client->dev, "%s: pll-master", init.name);
			init.flags |= CLK_SET_RATE_PARENT;
		}
		init.parent_names = parent_names;
		init.num_parents = 1;
		ret = devm_clk_hw_register(&client->dev,
					   &drvdata->clkout[n].hw);
		if (ret) {
			dev_err(&client->dev, "unable to register %s\n",
				init.name);
			goto err_clk;
		}
	}

	ret = of_clk_add_hw_provider(client->dev.of_node, si5340_of_clk_get,
				     drvdata);
	if (ret) {
		dev_err(&client->dev, "unable to add clk provider\n");
		goto err_clk;
	}

	return 0;

err_clk:
	return ret;
}

static const struct i2c_device_id si5340d_i2c_ids[] = {
	{ "si5340d", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, si5340d_i2c_ids);

static struct i2c_driver si5340d_driver = {
	.driver = {
		.name = "si5340d",
		.of_match_table = of_match_ptr(si5340d_dt_ids),
	},
	.probe = si5340d_i2c_probe,
	.id_table = si5340d_i2c_ids,
};
module_i2c_driver(si5340d_driver);

MODULE_LICENSE("GPL");
