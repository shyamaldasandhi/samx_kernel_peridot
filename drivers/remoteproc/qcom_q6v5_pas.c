// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm ADSP/SLPI Peripheral Image Loader for MSM8974 and MSM8996
 *
 * Copyright (C) 2016 Linaro Ltd
 * Copyright (C) 2014 Sony Mobile Communications AB
 * Copyright (c) 2012-2013, 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2025 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/dma-mapping.h>
#include <linux/panic_notifier.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/qcom_scm.h>
#include <linux/regulator/consumer.h>
#include <linux/remoteproc.h>
#include <linux/interconnect.h>
#include <linux/soc/qcom/mdt_loader.h>
#include <linux/soc/qcom/smem.h>
#include <linux/soc/qcom/smem_state.h>
#include <linux/soc/qcom/qcom_aoss.h>
#include <soc/qcom/secure_buffer.h>

#include <trace/events/rproc_qcom.h>
#include <soc/qcom/qcom_ramdump.h>
#include <trace/hooks/remoteproc.h>
#include <linux/iopoll.h>
#include <linux/timer.h>

#include "qcom_common.h"
#include "qcom_pil_info.h"
#include "qcom_q6v5.h"
#include "remoteproc_internal.h"

#define XO_FREQ		19200000
#define PIL_TZ_AVG_BW	UINT_MAX
#define PIL_TZ_PEAK_BW	UINT_MAX

#define ADSP_DECRYPT_SHUTDOWN_DELAY_MS	100
#define RPROC_HANDOVER_POLL_DELAY_MS	1

static struct icc_path *scm_perf_client;
static int scm_pas_bw_count;
static DEFINE_MUTEX(q6v5_pas_mutex);
bool timeout_disabled;
static bool global_sync_mem_setup;
static bool recovery_set_cb;

#define to_rproc(d) container_of(d, struct rproc, dev)

#define SOCCP_SLEEP_US  100
#define SOCCP_TIMEOUT_US  10000
#define SOCCP_STATE_MASK 0x600
#define SPARE_REG_SOCCP_D0  0x1
#define SOCCP_D0  0x2
#define SOCCP_D1  0x4
#define SOCCP_D3  0x8

#define EARLY_BOOT_RETRY_COUNT 5
#define EARLY_BOOT_RETRY_INTERVAL_MS 1000

struct adsp_data {
	int crash_reason_smem;
	const char *firmware_name;
	const char *dtb_firmware_name;
	int pas_id;
	int dtb_pas_id;
	bool free_after_auth_reset;
	unsigned int minidump_id;
	bool both_dumps;
	bool uses_elf64;
	bool auto_boot;
	bool dma_phys_below_32b;
	bool decrypt_shutdown;
	bool hyp_assign_mem;
	bool ssr_hyp_assign_mem;

	char **active_pd_names;
	char **proxy_pd_names;

	const char *ssr_name;
	const char *sysmon_name;
	const char *qmp_name;
	int ssctl_id;
	unsigned int smem_host_id;
	bool check_status;
	bool early_boot;
};

struct qcom_adsp {
	struct device *dev;
	struct device *minidump_dev;
	struct rproc *rproc;

	struct qcom_q6v5 q6v5;

	struct clk *xo;
	struct clk *aggre2_clk;

	struct regulator *cx_supply;
	struct regulator *px_supply;
	struct reg_info *regs;
	int reg_cnt;

	struct device *active_pds[1];
	struct device *proxy_pds[3];
	const char *qmp_name;
	struct qmp *qmp;

	int active_pd_count;
	int proxy_pd_count;

	int pas_id;
	int dtb_pas_id;
	const char *dtb_fw_name;
	struct qcom_mdt_metadata *mdata;
	struct qcom_mdt_metadata dtb_mdata;
	unsigned int minidump_id;
	bool both_dumps;
	bool retry_shutdown;
	struct icc_path *bus_client;
	int crash_reason_smem;
	unsigned int smem_host_id;
	bool dma_phys_below_32b;
	bool decrypt_shutdown;
	const char *info_name;

	struct completion start_done;
	struct completion stop_done;
	struct timer_list boot_timer;
	u32 count_get_irq;

	phys_addr_t dtb_mem_phys;
	phys_addr_t dtb_mem_reloc;
	void *dtb_mem_region;
	size_t dtb_mem_size;

	phys_addr_t mem_phys;
	phys_addr_t mem_reloc;
	void *mem_region;
	size_t mem_size;

	struct qcom_rproc_glink glink_subdev;
	struct qcom_rproc_subdev smd_subdev;
	struct qcom_rproc_ssr ssr_subdev;
	struct qcom_sysmon *sysmon;
	const struct firmware *dtb_firmware;
	bool subsys_recovery_disabled;

	bool hyp_assign_mem;
	bool ssr_hyp_assign_mem;
	phys_addr_t *hyp_assign_phy;
	size_t *hyp_assign_mem_size;
	int hyp_assign_mem_cnt;

	struct qcom_smem_state *wake_state;
	struct qcom_smem_state *sleep_state;
	struct notifier_block panic_blk;
	struct mutex adsp_lock;
	unsigned int wake_bit;
	unsigned int sleep_bit;
	int current_users;
	void *tcsr_addr;
	void *spare_reg_addr;
	bool check_status;
	bool rproc_ddr_set_icc_low_svs;
	unsigned int rproc_ddr_lowsvs_icc_bw;

	bool ready_irq;
	bool crash_irq;
};

static ssize_t txn_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct qcom_adsp *adsp = (struct qcom_adsp *)platform_get_drvdata(pdev);

	return sysfs_emit(buf, "%zu\n", qcom_sysmon_get_txn_id(adsp->sysmon));
}
static DEVICE_ATTR_RO(txn_id);

static inline bool is_mss_ssr_hyp_assign_en(struct qcom_adsp *adsp)
{
	return (adsp->ssr_hyp_assign_mem && !strcmp(adsp->dtb_fw_name, "modem_dtb.mdt"));
}

static int adsp_custom_segment_dump(struct qcom_adsp *adsp,
				    struct rproc_dump_segment *segment,
				    void *dest, size_t offset, size_t size)
{
	int len = strlen("md_dbg_buf");
	void __iomem *base;
	int total_offset;
	bool valid = false;
	int i;

	if (segment->priv && strnlen(segment->priv, len + 1) == len &&
		    !strcmp(segment->priv, "md_dbg_buf"))
		goto custom_segment_dump;

	if (!is_mss_ssr_hyp_assign_en(adsp))
		return -EINVAL;

	/*
	 * Also, do second level of check for custom segments in
	 * adsp_custom_segment_dump(), which checks if the segment
	 * lies outside the subsystem region range.
	 */
	for (i = 0; i < adsp->hyp_assign_mem_cnt; i++) {
		total_offset = segment->da + segment->offset +
			       offset - adsp->hyp_assign_phy[i];
		if (!(total_offset < 0 ||
		    total_offset + size > adsp->hyp_assign_mem_size[i])) {
			valid = true;
			break;
		}
	}

	if (!valid)
		return -EINVAL;

custom_segment_dump:
	base = ioremap((unsigned long)le64_to_cpu(segment->da) + offset, size);
	if (!base) {
		dev_err(adsp->dev, "failed to map custom_segment region\n");
		return -EINVAL;
	}

	memcpy_fromio(dest, base, size);
	iounmap(base);
	return 0;
}

void adsp_segment_dump(struct rproc *rproc, struct rproc_dump_segment *segment,
		     void *dest, size_t offset, size_t size)
{
	struct qcom_adsp *adsp = rproc->priv;
	int total_offset;

	total_offset = segment->da + segment->offset + offset - adsp->mem_phys;
	if (!(total_offset < 0 || total_offset + size > adsp->mem_size)) {
		memcpy_fromio(dest, adsp->mem_region + total_offset, size);
		return;
	} else if (!adsp_custom_segment_dump(adsp, segment, dest, offset, size)) {
		return;
	}

	dev_err(adsp->dev,
		"invalid copy request for segment %pad with offset %zu and size %zu)\n",
		&segment->da, offset, size);
	memset(dest, 0xff, size);
}

static void adsp_minidump(struct rproc *rproc)
{
	struct qcom_adsp *adsp = rproc->priv;

	trace_rproc_qcom_event(dev_name(adsp->dev), "adsp_minidump", "enter");

	if (rproc->dump_conf == RPROC_COREDUMP_DISABLED)
		goto exit;

	qcom_minidump(rproc, adsp->minidump_dev, adsp->minidump_id, adsp_segment_dump,
			adsp->both_dumps);

exit:
	trace_rproc_qcom_event(dev_name(adsp->dev), "adsp_minidump", "exit");
}

static int adsp_pds_enable(struct qcom_adsp *adsp, struct device **pds,
			   size_t pd_count)
{
	int ret;
	int i;

	for (i = 0; i < pd_count; i++) {
		dev_pm_genpd_set_performance_state(pds[i], INT_MAX);
		ret = pm_runtime_get_sync(pds[i]);
		if (ret < 0) {
			pm_runtime_put_noidle(pds[i]);
			dev_pm_genpd_set_performance_state(pds[i], 0);
			goto unroll_pd_votes;
		}
	}

	return 0;

unroll_pd_votes:
	for (i--; i >= 0; i--) {
		dev_pm_genpd_set_performance_state(pds[i], 0);
		pm_runtime_put(pds[i]);
	}

	return ret;
};

static void adsp_pds_disable(struct qcom_adsp *adsp, struct device **pds,
			     size_t pd_count)
{
	int i;

	for (i = 0; i < pd_count; i++) {
		dev_pm_genpd_set_performance_state(pds[i], 0);
		pm_runtime_put(pds[i]);
	}
}

static int adsp_shutdown_poll_decrypt(struct qcom_adsp *adsp)
{
	unsigned int retry_num = 50;
	int ret;

	do {
		msleep(ADSP_DECRYPT_SHUTDOWN_DELAY_MS);
		ret = qcom_scm_pas_shutdown(adsp->pas_id);
	} while (ret == -EINVAL && --retry_num);

	return ret;
}

static int scm_pas_enable_bw(void)
{
	int ret = 0;

	if (IS_ERR(scm_perf_client))
		return -EINVAL;

	mutex_lock(&q6v5_pas_mutex);
	if (!scm_pas_bw_count) {
		ret = icc_set_bw(scm_perf_client, PIL_TZ_AVG_BW,
						PIL_TZ_PEAK_BW);
		if (ret)
			goto err_bus;
	}

	scm_pas_bw_count++;
	mutex_unlock(&q6v5_pas_mutex);
	return ret;

err_bus:
	pr_err("scm-pas: Bandwidth request failed (%d)\n", ret);
	icc_set_bw(scm_perf_client, 0, 0);

	mutex_unlock(&q6v5_pas_mutex);
	return ret;
}

static void scm_pas_disable_bw(void)
{
	if (IS_ERR(scm_perf_client))
		return;

	mutex_lock(&q6v5_pas_mutex);
	if (scm_pas_bw_count-- == 1)
		icc_set_bw(scm_perf_client, 0, 0);
	mutex_unlock(&q6v5_pas_mutex);
}

static void adsp_add_coredump_segments(struct qcom_adsp *adsp, const struct firmware *fw)
{
	struct rproc *rproc = adsp->rproc;
	struct rproc_dump_segment *entry;
	struct elf32_hdr *ehdr = (struct elf32_hdr *)fw->data;
	struct elf32_phdr *phdr, *phdrs = (struct elf32_phdr *)(fw->data + ehdr->e_phoff);
	uint32_t elf_min_addr = U32_MAX;
	bool relocatable = false;
	int ret;
	int i;

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &phdrs[i];
		if (phdr->p_type != PT_LOAD ||
		   (phdr->p_flags & QCOM_MDT_TYPE_MASK) == QCOM_MDT_TYPE_HASH ||
		   !phdr->p_memsz)
			continue;

		if (phdr->p_flags & QCOM_MDT_RELOCATABLE)
			relocatable = true;

		elf_min_addr = min(phdr->p_paddr, elf_min_addr);

		ret = rproc_coredump_add_segment(rproc, phdr->p_paddr, phdr->p_memsz);
		if (ret) {
			dev_err(adsp->dev, "failed to add rproc segment: %d\n", ret);
			rproc_coredump_cleanup(adsp->rproc);
			return;
		}
	}

	list_for_each_entry(entry, &rproc->dump_segments, node)
		entry->da = adsp->mem_phys + entry->da - elf_min_addr;

	if (relocatable)
		adsp->mem_reloc = adsp->mem_phys + adsp->mem_reloc - elf_min_addr;
}

static int adsp_load(struct rproc *rproc, const struct firmware *fw)
{
	struct qcom_adsp *adsp = (struct qcom_adsp *)rproc->priv;
	int ret;

	trace_rproc_qcom_event(dev_name(adsp->dev), "adsp_load", "enter");

	rproc_coredump_cleanup(adsp->rproc);

	scm_pas_enable_bw();

	if (!adsp->dtb_pas_id || !adsp->dtb_fw_name) {
		scm_pas_disable_bw();
		return 0;
	}

	ret = request_firmware(&adsp->dtb_firmware, adsp->dtb_fw_name, adsp->dev);
	if (ret) {
		dev_err(adsp->dev, "request_firmware failed for %s: %d\n", adsp->dtb_fw_name, ret);
		goto exit;
	}

	ret = qcom_mdt_load_no_free(adsp->dev, adsp->dtb_firmware, adsp->dtb_fw_name,
				adsp->dtb_pas_id, adsp->dtb_mem_region, adsp->dtb_mem_phys,
				adsp->dtb_mem_size, &adsp->dtb_mem_reloc, adsp->dma_phys_below_32b,
				&adsp->dtb_mdata);
	if (ret) {
		dev_err(adsp->dev, "failed to load %s: %d\n", adsp->dtb_fw_name, ret);
		release_firmware(adsp->dtb_firmware);
		goto exit;
	}

exit:
	trace_rproc_qcom_event(dev_name(adsp->dev), "adsp_load", "exit");
	scm_pas_disable_bw();

	return ret;
}

static void disable_regulators(struct qcom_adsp *adsp)
{
	int i;

	for (i = (adsp->reg_cnt - 1); i >= 0; i--) {
		regulator_set_voltage(adsp->regs[i].reg, 0, INT_MAX);
		regulator_set_load(adsp->regs[i].reg, 0);
		regulator_disable(adsp->regs[i].reg);
	}
}

static int enable_regulators(struct qcom_adsp *adsp)
{
	int i, rc = 0;

	for (i = 0; i < adsp->reg_cnt; i++) {
		regulator_set_voltage(adsp->regs[i].reg, adsp->regs[i].uV, INT_MAX);
		regulator_set_load(adsp->regs[i].reg, adsp->regs[i].uA);
		rc = regulator_enable(adsp->regs[i].reg);
		if (rc) {
			dev_err(adsp->dev, "Regulator enable failed(rc:%d)\n",
				rc);
			goto err_enable;
		}
	}
	return rc;

err_enable:
	disable_regulators(adsp);
	return rc;
}

static int do_bus_scaling(struct qcom_adsp *adsp, bool enable)
{
	int rc = 0;
	u32 avg_bw = enable
			? adsp->rproc_ddr_set_icc_low_svs
				? adsp->rproc_ddr_lowsvs_icc_bw
				: PIL_TZ_AVG_BW
			: 0;
	u32 peak_bw = enable
			? adsp->rproc_ddr_set_icc_low_svs
				? adsp->rproc_ddr_lowsvs_icc_bw
				: PIL_TZ_PEAK_BW
			: 0;

	if (IS_ERR(adsp->bus_client)) {
		dev_err(adsp->dev, "Bus scaling not setup for %s\n",
			adsp->rproc->name);
	} else {
		rc = icc_set_bw(adsp->bus_client, avg_bw, peak_bw);
	}
	if (rc)
		dev_err(adsp->dev, "bandwidth request failed(rc:%d)\n", rc);

	return rc;
}

static int setup_mpss_dsm_mem(struct qcom_adsp *adsp)
{
	struct of_phandle_iterator it;
	struct resource res;
	int ret;
	int i = 0;

	ret = of_property_count_elems_of_size(adsp->dev->of_node,
					      "mpss_dsm_mem_reg", sizeof(phandle));
	if (ret < 0) {
		dev_err(adsp->dev, "mpss_dsm_mem_reg is not defined properly\n");
		return ret;
	}

	adsp->hyp_assign_phy = devm_kzalloc(adsp->dev,
					     sizeof(phys_addr_t) * ret, GFP_KERNEL);
	if (!adsp->hyp_assign_phy)
		return -ENOMEM;

	adsp->hyp_assign_mem_size = devm_kzalloc(adsp->dev,
					     sizeof(size_t) * ret, GFP_KERNEL);
	if (!adsp->hyp_assign_mem_size)
		return -ENOMEM;

	of_for_each_phandle(&it, ret, adsp->dev->of_node, "mpss_dsm_mem_reg", NULL, 0) {
		ret = of_address_to_resource(it.node, 0, &res);
		if (ret) {
			dev_err(adsp->dev,
				"address to resource failed for mpss_dsm_mem_reg[%d]\n",
				it.cur_count);
			return ret;
		}

		adsp->hyp_assign_phy[i] = res.start;
		adsp->hyp_assign_mem_size[i] = resource_size(&res);
		i++;
	}

	adsp->hyp_assign_mem_cnt = i;

	return 0;
}

static int mpss_dsm_hyp_assign_control(struct qcom_adsp *adsp, bool start)
{
	struct qcom_scm_vmperm newvm[1];
	u64 curr_perm;
	int ret;
	int i;

	for (i = 0; i < adsp->hyp_assign_mem_cnt; i++) {
		if (start) {
			newvm[0].vmid = QCOM_SCM_VMID_MSS_MSA;
			curr_perm = BIT(QCOM_SCM_VMID_HLOS);
		} else {
			newvm[0].vmid = QCOM_SCM_VMID_HLOS;
			curr_perm = BIT(QCOM_SCM_VMID_MSS_MSA);
		}

		newvm[0].perm = QCOM_SCM_PERM_RW;
		ret = qcom_scm_assign_mem(adsp->hyp_assign_phy[i],
					  adsp->hyp_assign_mem_size[i],
					  &curr_perm, newvm, 1);
		/*
		 * There is no point of reclaiming the successful
		 * hyp assigned memory as already something bad
		 * happened.
		 */
		if (ret) {
			dev_err(adsp->dev,
				"hyp assign for mpss_dsm_mem_reg[%d]\n", i);
			return ret;
		}
	}

	return 0;
}

static void add_mpss_dsm_mem_ssr_dump(struct qcom_adsp *adsp)
{
	struct rproc *rproc = adsp->rproc;
	struct device_node *np;
	struct resource imem;
	void __iomem *base;
	int ret = 0, i;
	const char *prop = "qcom,msm-imem-mss-dsm";
	dma_addr_t da;
	size_t size;

	np = of_find_compatible_node(NULL, NULL, prop);
	if (!np) {
		pr_err("%s entry missing!\n", prop);
		return;
	}

	ret = of_address_to_resource(np, 0, &imem);
	of_node_put(np);
	if (ret < 0) {
		pr_err("address to resource conversion failed for %s\n", prop);
		return;
	}

	base = ioremap(imem.start, resource_size(&imem));
	if (!base) {
		pr_err("failed to map MSS DSM region\n");
		return;
	}

	/*
	 * There can be multiple DSM partitions based on the Modem flavor.
	 * Each DSM partition start address and size are written to IMEM by Modem and each
	 * partition consumes 4 bytes (2 bytes for address and 2 bytes for size) of IMEM.
	 *
	 * Modem physical address range has to be in the low 4G (32 bits only) and low 2
	 * bytes will be zeros, so, left shift by 16 to get proper address & size.
	 */
	for (i = 0; i < resource_size(&imem); i = i + 4) {
		da = (u32)(__raw_readw(base + i) << 16);
		size = (u32)(__raw_readw(base + (i + 2)) << 16);
		if (da && size)
			rproc_coredump_add_custom_segment(rproc,
				da, size, adsp_segment_dump, NULL);
	}

	iounmap(base);
}

static int qcom_rproc_alloc_dtb_firmware(struct qcom_adsp *adsp,
					const char *dtb_firmware)
{
	const char *p;

	if (!dtb_firmware)
		return 0;

	p = kstrdup_const(dtb_firmware, GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	adsp->dtb_fw_name = p;
	return 0;
}

int qcom_rproc_set_dtb_firmware(struct rproc *rproc, const char *dtb_fw_name)
{
	struct qcom_adsp *adsp;
	struct device *dev;
	int ret, len;
	char *p;

	if (!rproc || !dtb_fw_name)
		return -EINVAL;

	dev = rproc->dev.parent;
	adsp = (struct qcom_adsp *)rproc->priv;
	ret = mutex_lock_interruptible(&rproc->lock);
	if (ret) {
		dev_err(dev, "can't lock rproc %s: %d\n", rproc->name, ret);
		return -EINVAL;
	}

	if (rproc->state != RPROC_OFFLINE) {
		dev_err(dev, "can't change firmware while running\n");
		ret = -EBUSY;
		goto out;
	}

	len = strcspn(dtb_fw_name, "\n");
	if (!len) {
		dev_err(dev, "can't provide empty string for DTB firmware name\n");
		ret = -EINVAL;
		goto out;
	}

	p = kstrndup(dtb_fw_name, len, GFP_KERNEL);
	if (!p) {
		ret = -ENOMEM;
		goto out;
	}
	if (adsp->dtb_fw_name)
		kfree_const(adsp->dtb_fw_name);
	adsp->dtb_fw_name = p;

out:
	mutex_unlock(&rproc->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(qcom_rproc_set_dtb_firmware);

static int setup_global_sync_mem(struct qcom_adsp *adsp)
{
	struct qcom_scm_vmperm newvm[2];
	struct device_node *node;
	struct resource res;
	phys_addr_t mem_phys;
	u64 curr_perm;
	u64 mem_size;
	int ret;

	curr_perm = BIT(QCOM_SCM_VMID_HLOS);
	newvm[0].vmid = QCOM_SCM_VMID_HLOS;
	newvm[0].perm = QCOM_SCM_PERM_RW;
	newvm[1].vmid = QCOM_SCM_VMID_CDSP;
	newvm[1].perm = QCOM_SCM_PERM_RW;

	node = of_parse_phandle(adsp->dev->of_node, "global-sync-mem-reg", 0);
	if (!node) {
		dev_err(adsp->dev, "global sync mem region is missing\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(node, 0, &res);
	if (ret) {
		dev_err(adsp->dev, "address to resource failed for global sync mem\n");
		return ret;
	}

	mem_phys = res.start;
	mem_size = resource_size(&res);
	ret = qcom_scm_assign_mem(mem_phys, mem_size, &curr_perm, newvm, ARRAY_SIZE(newvm));
	if (ret) {
		dev_err(adsp->dev, "hyp assign for global sync mem failed\n");
		return ret;
	}

	global_sync_mem_setup = true;
	return 0;
}

static int adsp_start(struct rproc *rproc)
{
	struct qcom_adsp *adsp = (struct qcom_adsp *)rproc->priv;
	int i, ret;
	const struct firmware *fw = NULL;

	trace_rproc_qcom_event(dev_name(adsp->dev), "adsp_start", "enter");

	if (adsp->check_status)
		adsp->current_users = 0;

	qcom_q6v5_prepare(&adsp->q6v5);

	if (adsp->hyp_assign_mem && adsp->pas_id == 18) {
		ret = setup_global_sync_mem(adsp);
		if (ret) {
			dev_err(adsp->dev, "failed to setup global sync mem\n");
			return -EINVAL;
		}
	}

	if (is_mss_ssr_hyp_assign_en(adsp)) {
		ret = mpss_dsm_hyp_assign_control(adsp, true);
		if (ret) {
			dev_err(adsp->dev, "failed to hyp assign mpss dsm mem\n");
			goto disable_irqs;
		}
	}

	ret = do_bus_scaling(adsp, true);
	if (ret < 0)
		goto disable_irqs;

	ret = adsp_pds_enable(adsp, adsp->active_pds, adsp->active_pd_count);
	if (ret < 0)
		goto unscale_bus;

	ret = adsp_pds_enable(adsp, adsp->proxy_pds, adsp->proxy_pd_count);
	if (ret < 0)
		goto disable_active_pds;

	if (adsp->qmp) {
		ret = qcom_rproc_toggle_load_state(adsp->qmp, adsp->qmp_name, true);
		if (ret)
			goto disable_proxy_pds;
	}

	ret = clk_prepare_enable(adsp->xo);
	if (ret)
		goto disable_load_state;

	ret = clk_prepare_enable(adsp->aggre2_clk);
	if (ret)
		goto disable_xo_clk;

	ret = enable_regulators(adsp);
	if (ret)
		goto disable_aggre2_clk;

	scm_pas_enable_bw();
	trace_rproc_qcom_event(dev_name(adsp->dev), "dtb_auth_reset", "enter");
	if (adsp->dtb_pas_id || adsp->dtb_fw_name) {
		ret = qcom_scm_pas_auth_and_reset(adsp->dtb_pas_id);
		if (ret)
			panic("Panicking, auth and reset failed for remoteproc %s dtb ret=%d\n",
				rproc->name, ret);
	}

	trace_rproc_qcom_event(dev_name(adsp->dev), "Q6_firmware_loading", "enter");
	ret = request_firmware(&fw, rproc->firmware, adsp->dev);
	if (ret)
		goto free_metadata_dtb;

	ret = qcom_mdt_load_no_free(adsp->dev, fw, rproc->firmware, adsp->pas_id,
				    adsp->mem_region, adsp->mem_phys, adsp->mem_size,
				    &adsp->mem_reloc, adsp->dma_phys_below_32b, adsp->mdata);
	if (ret)
		goto free_firmware;

	qcom_pil_info_store(adsp->info_name, adsp->mem_phys, adsp->mem_size);

	adsp_add_coredump_segments(adsp, fw);
	trace_rproc_qcom_event(dev_name(adsp->dev), "Q6_auth_reset", "enter");

	ret = qcom_scm_pas_auth_and_reset(adsp->pas_id);
	if (ret)
		panic("Panicking, auth and reset failed for remoteproc %s ret=%d\n",
				rproc->name, ret);
	trace_rproc_qcom_event(dev_name(adsp->dev), "Q6_auth_reset", "exit");

	/* if needed, signal Q6 to continute booting */
	if (adsp->q6v5.rmb_base) {
		for (i = 0; i < RMB_POLL_MAX_TIMES || timeout_disabled; i++) {
			if (readl_relaxed(adsp->q6v5.rmb_base + RMB_BOOT_WAIT_REG)) {
				writel_relaxed(1, adsp->q6v5.rmb_base + RMB_BOOT_CONT_REG);
				break;
			}
			msleep(20);
		}

		if (!readl_relaxed(adsp->q6v5.rmb_base + RMB_BOOT_WAIT_REG)) {
			dev_err(adsp->dev, "Didn't get rmb signal from  %s\n", rproc->name);
			goto free_metadata;
		}
	}

	if (!timeout_disabled) {
		ret = qcom_q6v5_wait_for_start(&adsp->q6v5, msecs_to_jiffies(5000));
		if (rproc->recovery_disabled && ret)
			panic("Panicking, remoteproc %s failed to bootup.\n", adsp->rproc->name);
		else if (ret == -ETIMEDOUT)
			dev_err(adsp->dev, "start timed out\n");
	}

	adsp->q6v5.seq++;

free_metadata:
	qcom_mdt_free_metadata(adsp->dev, adsp->pas_id, adsp->mdata,
					adsp->dma_phys_below_32b, ret);
free_firmware:
	if (fw)
		release_firmware(fw);

free_metadata_dtb:
	if (adsp->dtb_pas_id || adsp->dtb_fw_name) {
		qcom_mdt_free_metadata(adsp->dev, adsp->dtb_pas_id,
					&adsp->dtb_mdata, adsp->dma_phys_below_32b, ret);
		release_firmware(adsp->dtb_firmware);
	}

	scm_pas_disable_bw();
	if (!ret)
		goto exit;

	disable_regulators(adsp);
disable_aggre2_clk:
	clk_disable_unprepare(adsp->aggre2_clk);
disable_xo_clk:
	clk_disable_unprepare(adsp->xo);
disable_load_state:
	if (adsp->qmp)
		qcom_rproc_toggle_load_state(adsp->qmp, adsp->qmp_name, false);
disable_proxy_pds:
	adsp_pds_disable(adsp, adsp->proxy_pds, adsp->proxy_pd_count);
disable_active_pds:
	adsp_pds_disable(adsp, adsp->active_pds, adsp->active_pd_count);
unscale_bus:
	do_bus_scaling(adsp, false);
disable_irqs:
	qcom_q6v5_unprepare(&adsp->q6v5);
exit:
	trace_rproc_qcom_event(dev_name(adsp->dev), "adsp_start", "exit");
	return ret;
}

/**
 * rproc_config_check() - Check back the config register
 * @state: new state of the rproc
 *
 * Polled read on a register till with a 5ms timeout and 100-200Us interval.
 * Returns immediately if the expected value is read back from the addr.
 *
 * state: new state of the rproc
 *
 * addr: Address to poll on
 *
 * return: 0 if the expected value is read back from the address
 * -ETIMEDOUT is the value was not read in 5ms
 */
static int rproc_config_check(struct qcom_adsp *adsp, u32 state, void *addr)
{
	unsigned int retry_num = 50;
	u32 val;

	do {
		usleep_range(SOCCP_SLEEP_US, SOCCP_SLEEP_US + 100);
		val = readl(addr);
	} while (!(val & state) && --retry_num);

	return (val & state) ? 0 : -ETIMEDOUT;
}
static int rproc_config_check_atomic(struct qcom_adsp *adsp, u32 state, void *addr)
{
	u32 val;

	return readx_poll_timeout_atomic(readl, addr, val,
				val == state, SOCCP_SLEEP_US, SOCCP_TIMEOUT_US);
}

/**
 * rproc_find_status_register() - Find the power control regs and INT's
 *
 * Call this function to calculated the tcsr config register, which
 * is the register to be chacked to read the current state of the rproc.
 *
 * Return: 0 for success
 */
static int rproc_find_status_register(struct qcom_adsp *adsp)
{
	struct device_node *tcsr;
	struct device_node *np = adsp->dev->of_node;
	struct resource res;
	u32 offset, addr;
	int ret;
	void *tcsr_base;

	tcsr = of_parse_phandle(np, "soccp-tcsr", 0);
	if (!tcsr) {
		dev_err(adsp->dev, "Unable to find the soccp config register\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(tcsr, 0, &res);
	of_node_put(tcsr);
	if (ret) {
		dev_err(adsp->dev, "Unable to find the tcsr base addr\n");
		return ret;
	}

	tcsr_base = ioremap_wc(res.start, resource_size(&res));
	if (!tcsr_base) {
		dev_err(adsp->dev, "Unable to find the tcsr base addr\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32_index(np, "soccp-tcsr", 1, &offset);
	if (ret < 0) {
		dev_err(adsp->dev, "Unable to find the tcsr config offset addr\n");
		iounmap(tcsr_base);
		return ret;
	}
	adsp->tcsr_addr = tcsr_base + offset;

	ret = of_property_read_u32(np, "soccp-spare", &addr);
	if (!addr) {
		dev_err(adsp->dev, "Unable to find the running config register\n");
		return -EINVAL;
	}

	adsp->spare_reg_addr = ioremap_wc(addr, 4);
	if (!adsp->spare_reg_addr) {
		dev_err(adsp->dev, "Unable to find the tcsr base addr\n");
		return -ENOMEM;
	}

	return 0;
}

static bool rproc_poll_handover(struct qcom_adsp *adsp)
{
	unsigned int retry_num = 50;

	do {
		msleep(RPROC_HANDOVER_POLL_DELAY_MS);
	} while (!adsp->q6v5.handover_issued && --retry_num);

	return adsp->q6v5.handover_issued;
}

/**
 * rproc_set_state() - Request the SOCCP to change state
 * @state: 1 to set state to RUNNING (D3 to D0)
 *         0 to set state to SUSPEND (D0 to D3)
 *
 * Function to request the SOCCP to move to Running/Dormant.
 * Blocking API, where the MAX timeout is 5 seconds.
 *
 * return: 0 if status is set, else -ETIMEOUT
 */
int rproc_set_state(struct rproc *rproc, bool state)
{
	int ret = 0;
	int users;
	struct qcom_adsp *adsp;

	if (!rproc || !rproc->priv) {
		pr_err("no rproc or adsp\n");
		return -EINVAL;
	}

	adsp = (struct qcom_adsp *)rproc->priv;
	if (!adsp->q6v5.running) {
		dev_err(adsp->dev, "rproc is not running\n");
		return -EINVAL;
	} else if (!adsp->q6v5.handover_issued) {
		dev_err(adsp->dev, "rproc is running but handover is not received\n");
		if (!rproc_poll_handover(adsp)) {
			dev_err(adsp->dev, "retry for handover timedout\n");
			return -EINVAL;
		}
	}

	mutex_lock(&adsp->adsp_lock);
	users = adsp->current_users;
	if (state) {
		if (users >= 1) {
			adsp->current_users++;
			ret = 0;
			goto soccp_out;
		}

		ret = enable_regulators(adsp);
		if (ret) {
			dev_err(adsp->dev, "failed to enable regulators\n");
			goto soccp_out;
		}

		ret = do_bus_scaling(adsp, true);
		if (ret) {
			dev_err(adsp->q6v5.dev, "failed to set bandwidth request\n");
			goto soccp_out;
		}

		ret = clk_prepare_enable(adsp->xo);
		if (ret) {
			dev_err(adsp->dev, "failed to enable clks\n");
			goto soccp_out;
		}

		ret = qcom_smem_state_update_bits(adsp->wake_state,
					    SOCCP_STATE_MASK,
					    BIT(adsp->wake_bit));
		if (ret) {
			dev_err(adsp->dev, "failed to update smem bits for D3 to D0\n");
			goto soccp_out;
		}

		ret = rproc_config_check(adsp, SOCCP_D0 | SOCCP_D1, adsp->tcsr_addr);
		if (ret) {
			dev_err(adsp->dev, "%s requested D3->D0: soccp failed to update tcsr val=%d\n",
				current->comm, readl(adsp->tcsr_addr));
			goto soccp_out;
		}

		ret = rproc_config_check(adsp, SPARE_REG_SOCCP_D0, adsp->spare_reg_addr);
		if (ret) {
			ret = qcom_smem_state_update_bits(adsp->wake_state,
						    SOCCP_STATE_MASK,
						    !BIT(adsp->wake_bit));
			if (ret)
				dev_err(adsp->dev, "failed to clear smem bits after a failed D0 request\n");

			dev_err(adsp->dev, "%s requested D3->D0: soccp failed to update spare reg val=%d\n",
				current->comm, readl(adsp->spare_reg_addr));
			goto soccp_out;
		}

		adsp->current_users = 1;
	} else {
		if (users > 1) {
			adsp->current_users--;
			ret = 0;
			goto soccp_out;
		} else if (users == 1) {
			ret = qcom_smem_state_update_bits(adsp->sleep_state,
					    SOCCP_STATE_MASK,
					    BIT(adsp->sleep_bit));
			if (ret) {
				dev_err(adsp->dev, "failed to update smem bits for D0 to D3\n");
				goto soccp_out;
			}

			ret = rproc_config_check(adsp, SOCCP_D3, adsp->tcsr_addr);
			if (ret) {
				ret = qcom_smem_state_update_bits(adsp->sleep_state,
						    SOCCP_STATE_MASK,
						    !BIT(adsp->sleep_bit));
				if (ret)
					dev_err(adsp->dev, "failed to clear smem bits after a failed D3 request\n");

				dev_err(adsp->dev, "%s requested D0->D3 failed: TCSR value:%d\n",
					current->comm, readl(adsp->tcsr_addr));
				goto soccp_out;
			}
			disable_regulators(adsp);
			clk_disable_unprepare(adsp->xo);
			ret = do_bus_scaling(adsp, false);
			if (ret < 0) {
				dev_err(adsp->q6v5.dev, "failed to set bandwidth request\n");
				goto soccp_out;
			}

			adsp->current_users = 0;
		}
	}

soccp_out:
	if (ret && (adsp->rproc->state != RPROC_RUNNING)) {
		dev_err(adsp->dev, "SOCCP has crashed while processing a D transition req by %s\n",
			current->comm);
		ret = -EBUSY;
	}

	mutex_unlock(&adsp->adsp_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(rproc_set_state);

static int rproc_panic_handler(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	struct qcom_adsp *adsp = container_of(this, struct qcom_adsp, panic_blk);
	int ret;

	if (!adsp)
		return NOTIFY_DONE;
	/* wake up SOCCP during panic to run error handlers on SOCCP */
	dev_info(adsp->dev, "waking SOCCP from panic path\n");
	ret = qcom_smem_state_update_bits(adsp->wake_state,
				    SOCCP_STATE_MASK,
				    BIT(adsp->wake_bit));
	if (ret) {
		dev_err(adsp->dev, "failed to update smem bits for D3 to D0\n");
		goto done;
	}
	ret = rproc_config_check_atomic(adsp, SOCCP_D0, adsp->tcsr_addr);
	if (ret)
		dev_err(adsp->dev, "failed to change to D0\n");

	ret = rproc_config_check_atomic(adsp, SPARE_REG_SOCCP_D0, adsp->spare_reg_addr);
	if (ret)
		dev_err(adsp->dev, "failed to change to D0\n");
done:
	return NOTIFY_DONE;
}

static void qcom_pas_handover(struct qcom_q6v5 *q6v5)
{
	struct qcom_adsp *adsp = container_of(q6v5, struct qcom_adsp, q6v5);
	int ret;

	if (adsp->check_status) {
		ret = rproc_config_check(adsp, SOCCP_D3, adsp->tcsr_addr);
		if (ret)
			dev_err(adsp->dev, "state not changed in handover TCSR val = %d\n",
				readl(adsp->tcsr_addr));
		else
			dev_info(adsp->dev, "state changed in handover for soccp! TCSR val = %d\n",
					readl(adsp->tcsr_addr));
	}
	disable_regulators(adsp);
	clk_disable_unprepare(adsp->aggre2_clk);
	clk_disable_unprepare(adsp->xo);
	adsp_pds_disable(adsp, adsp->proxy_pds, adsp->proxy_pd_count);
	do_bus_scaling(adsp, false);
}

static int adsp_stop(struct rproc *rproc)
{
	struct qcom_adsp *adsp = (struct qcom_adsp *)rproc->priv;
	int handover;
	int ret;

	trace_rproc_qcom_event(dev_name(adsp->dev), "adsp_stop", "enter");

	ret = qcom_q6v5_request_stop(&adsp->q6v5, adsp->sysmon);
	if (ret == -ETIMEDOUT)
		dev_err(adsp->dev, "timed out on wait\n");

	scm_pas_enable_bw();
	if (adsp->retry_shutdown)
		ret = qcom_scm_pas_shutdown_retry(adsp->pas_id);
	else
		ret = qcom_scm_pas_shutdown(adsp->pas_id);

	if (ret && adsp->decrypt_shutdown)
		ret = adsp_shutdown_poll_decrypt(adsp);

	if (ret)
		panic("Panicking, remoteproc %s failed to shutdown.\n", rproc->name);

	if (adsp->dtb_pas_id) {
		ret = qcom_scm_pas_shutdown(adsp->dtb_pas_id);
		if (ret)
			panic("Panicking, remoteproc %s dtb failed to shutdown.\n", rproc->name);
	}

	scm_pas_disable_bw();
	adsp_pds_disable(adsp, adsp->active_pds, adsp->active_pd_count);
	if (adsp->qmp)
		qcom_rproc_toggle_load_state(adsp->qmp, adsp->qmp_name, false);
	handover = qcom_q6v5_unprepare(&adsp->q6v5);
	if (handover)
		qcom_pas_handover(&adsp->q6v5);

	if (adsp->smem_host_id)
		ret = qcom_smem_bust_hwspin_lock_by_host(adsp->smem_host_id);

	if (is_mss_ssr_hyp_assign_en(adsp)) {
		add_mpss_dsm_mem_ssr_dump(adsp);
		ret = mpss_dsm_hyp_assign_control(adsp, false);
		if (ret)
			dev_err(adsp->dev, "failed to reclaim mpss dsm mem\n");
	}

	adsp->q6v5.seq++;
	trace_rproc_qcom_event(dev_name(adsp->dev), "adsp_stop", "exit");

	return ret;
}

/*
 * read_early_boot_register: Read Slave kernel bits
 *
 * Function to read the slave kernel bits and update the rproc state
 * based off the read bits.
 *
 * return:
 *        -EINVAL if the slave kernel bits have not been set
 *        -ENODATA if the slave kernel is read but the device has
 *              not crashed or boot up
 *        0 if the value has been read and the state has been set
 *
 */
void read_early_boot_register(struct timer_list *timer)
{
	struct qcom_adsp *adsp = NULL;
	int ret = ENODATA;

	adsp = container_of(timer, struct qcom_adsp, boot_timer);
	if (!adsp)
		return;

	ret = irq_get_irqchip_state(adsp->q6v5.fatal_irq,
				IRQCHIP_STATE_LINE_LEVEL, &adsp->crash_irq);
	if (adsp->crash_irq) {
		dev_err(adsp->dev, "Sub system has crashed before driver probe\n");
		adsp->rproc->state = RPROC_CRASHED;
	}

	ret = irq_get_irqchip_state(adsp->q6v5.ready_irq,
			IRQCHIP_STATE_LINE_LEVEL, &adsp->ready_irq);

	if (adsp->ready_irq) {
		dev_info(adsp->dev, "Sub system has boot-up before driver probe\n");
		adsp->rproc->state = RPROC_DETACHED;
	}

	if ((adsp->count_get_irq++ < EARLY_BOOT_RETRY_COUNT) && (ret == -ENODEV))
		mod_timer(&adsp->boot_timer,
			jiffies + msecs_to_jiffies(EARLY_BOOT_RETRY_INTERVAL_MS));
	else
		complete(&adsp->q6v5.subsys_booted);

}

static int adsp_attach(struct rproc *rproc)
{
	struct qcom_adsp *adsp = (struct qcom_adsp *)rproc->priv;
	const struct firmware *fw;
	int ret = 0;
	int i;
	if (adsp->q6v5.early_boot)
		goto timer_setup;

	/* try to register fw for dumps; continue if we fail */
	ret = request_firmware(&fw, rproc->firmware, &rproc->dev);
	if (ret < 0) {
		dev_err(adsp->dev, "Failed to request DSP firmware\n");
		dev_err(adsp->dev, "Dumps will not be available\n");
		goto begin_attach;
	}

	ret = qcom_register_dump_segments(rproc, fw);
	if (ret) {
		dev_err(adsp->dev, "Failed to register dump segments\n");
		dev_err(adsp->dev, "Dumps will not be available\n");
	}
	release_firmware(fw);

begin_attach:
	qcom_q6v5_prepare(&adsp->q6v5);

	ret = do_bus_scaling(adsp, true);
	if (ret < 0)
		goto disable_irqs;

	ret = adsp_pds_enable(adsp, adsp->active_pds, adsp->active_pd_count);
	if (ret < 0)
		goto unscale_bus;

	if (!adsp->q6v5.rmb_base ||
	    !readl_relaxed(adsp->q6v5.rmb_base + RMB_BOOT_WAIT_REG)) {
		dev_err(adsp->dev, "Remote proc is not ready to attach\n");
		adsp_stop(rproc);
		ret = -EBUSY;
		goto disable_active_pds;
	}

	ret = adsp_pds_enable(adsp, adsp->proxy_pds, adsp->proxy_pd_count);
	if (ret < 0)
		goto disable_active_pds;

	ret = qcom_rproc_toggle_load_state(adsp->qmp, adsp->qmp_name, true);
	if (ret)
		goto disable_proxy_pds;

	ret = clk_prepare_enable(adsp->xo);
	if (ret)
		goto disable_load_state;

	ret = clk_prepare_enable(adsp->aggre2_clk);
	if (ret)
		goto disable_xo_clk;

	ret = enable_regulators(adsp);
	if (ret)
		goto disable_aggre2_clk;

	/* Signal the Q6 to continue booting */
	for (i = 0; i < RMB_POLL_MAX_TIMES || timeout_disabled; i++) {
		if (readl_relaxed(adsp->q6v5.rmb_base + RMB_BOOT_WAIT_REG)) {
			writel_relaxed(1, adsp->q6v5.rmb_base + RMB_BOOT_CONT_REG);
			break;
		}
		msleep(20);
	}

	if (!readl_relaxed(adsp->q6v5.rmb_base + RMB_BOOT_WAIT_REG)) {
		dev_err(adsp->dev, "Didn't get rmb signal from %s\n", rproc->name);
		goto disable_regs;
	}

	if (!timeout_disabled) {
		ret = qcom_q6v5_wait_for_start(&adsp->q6v5, msecs_to_jiffies(5000));
		if (rproc->recovery_disabled && ret) {
			panic("Panicking, remoteproc %s failed to bootup.\n", adsp->rproc->name);
		} else if (ret == -ETIMEDOUT) {
			dev_err(adsp->dev, "start timed out\n");
			goto disable_regs;
		}
	}

	return ret;

disable_regs:
	disable_regulators(adsp);
disable_aggre2_clk:
	clk_disable_unprepare(adsp->aggre2_clk);
disable_xo_clk:
	clk_disable_unprepare(adsp->xo);
disable_load_state:
	qcom_rproc_toggle_load_state(adsp->qmp, adsp->qmp_name, false);
disable_proxy_pds:
	adsp_pds_disable(adsp, adsp->proxy_pds, adsp->proxy_pd_count);
disable_active_pds:
	adsp_pds_disable(adsp, adsp->active_pds, adsp->active_pd_count);
unscale_bus:
	do_bus_scaling(adsp, false);
disable_irqs:
	qcom_q6v5_unprepare(&adsp->q6v5);
timer_setup:
	timer_setup(&adsp->boot_timer, read_early_boot_register, 0);
	init_completion(&adsp->q6v5.subsys_booted);

	read_early_boot_register(&(adsp->boot_timer));

	wait_for_completion(&adsp->q6v5.subsys_booted);
	del_timer(&adsp->boot_timer);

	ret = ping_subsystem(&adsp->q6v5);

	if (ret) {
		dev_err(adsp->dev, "Timed out on ping/pong, assuming device crashed\n");
		rproc->state = RPROC_CRASHED;
	}
	adsp->q6v5.running = true;
	return ret;
}

static void *adsp_da_to_va(struct rproc *rproc, u64 da, size_t len, bool *is_iomem)
{
	struct qcom_adsp *adsp = (struct qcom_adsp *)rproc->priv;
	int offset;

	offset = da - adsp->mem_reloc;
	if (offset < 0 || offset + len > adsp->mem_size)
		return NULL;

	if (is_iomem)
		*is_iomem = true;

	return adsp->mem_region + offset;
}

static unsigned long adsp_panic(struct rproc *rproc)
{
	struct qcom_adsp *adsp = (struct qcom_adsp *)rproc->priv;

	return qcom_q6v5_panic(&adsp->q6v5);
}

static const struct rproc_ops adsp_ops = {
	.attach = adsp_attach,
	.start = adsp_start,
	.stop = adsp_stop,
	.da_to_va = adsp_da_to_va,
	.load = adsp_load,
	.panic = adsp_panic,
};

static const struct rproc_ops adsp_minidump_ops = {
	.attach = adsp_attach,
	.start = adsp_start,
	.stop = adsp_stop,
	.da_to_va = adsp_da_to_va,
	.parse_fw = qcom_register_dump_segments,
	.load = adsp_load,
	.panic = adsp_panic,
	.coredump = adsp_minidump,
};

static int adsp_init_clock(struct qcom_adsp *adsp)
{
	int ret;

	adsp->xo = devm_clk_get(adsp->dev, "xo");
	if (IS_ERR(adsp->xo)) {
		ret = PTR_ERR(adsp->xo);
		if (ret != -EPROBE_DEFER)
			dev_err(adsp->dev, "failed to get xo clock");
		return ret;
	}

	adsp->aggre2_clk = devm_clk_get_optional(adsp->dev, "aggre2");
	if (IS_ERR(adsp->aggre2_clk)) {
		ret = PTR_ERR(adsp->aggre2_clk);
		if (ret != -EPROBE_DEFER)
			dev_err(adsp->dev,
				"failed to get aggre2 clock");
		return ret;
	}

	return 0;
}

static int adsp_init_regulator(struct qcom_adsp *adsp)
{
	int len;
	int i, rc;
	char uv_ua[50];
	u32 uv_ua_vals[2];
	const char *reg_name;

	adsp->reg_cnt = of_property_count_strings(adsp->dev->of_node,
						  "reg-names");
	if (adsp->reg_cnt <= 0) {
		dev_err(adsp->dev, "No regulators added!\n");
		return 0;
	}

	adsp->regs = devm_kzalloc(adsp->dev,
				  sizeof(struct reg_info) * adsp->reg_cnt,
				  GFP_KERNEL);
	if (!adsp->regs)
		return -ENOMEM;

	for (i = 0; i < adsp->reg_cnt; i++) {
		of_property_read_string_index(adsp->dev->of_node, "reg-names",
					      i, &reg_name);

		adsp->regs[i].reg = devm_regulator_get(adsp->dev, reg_name);
		if (IS_ERR(adsp->regs[i].reg)) {
			dev_err(adsp->dev, "failed to get %s reg\n", reg_name);
			return PTR_ERR(adsp->regs[i].reg);
		}

		/* Read current(uA) and voltage(uV) value */
		snprintf(uv_ua, sizeof(uv_ua), "%s-uV-uA", reg_name);
		if (!of_find_property(adsp->dev->of_node, uv_ua, &len))
			continue;

		rc = of_property_read_u32_array(adsp->dev->of_node, uv_ua,
						uv_ua_vals,
						ARRAY_SIZE(uv_ua_vals));
		if (rc) {
			dev_err(adsp->dev, "Failed to read uVuA value(rc:%d)\n",
				rc);
			return rc;
		}

		if (uv_ua_vals[0] > 0)
			adsp->regs[i].uV = uv_ua_vals[0];
		if (uv_ua_vals[1] > 0)
			adsp->regs[i].uA = uv_ua_vals[1];
	}
	return 0;
}

static void adsp_init_bus_scaling(struct qcom_adsp *adsp)
{
	int ret;
	bool rproc_ddr_set_icc_low_svs;
	unsigned int rproc_ddr_lowsvs_icc_bw;

	if (scm_perf_client)
		goto get_rproc_client;

	scm_perf_client = of_icc_get(adsp->dev, "crypto_ddr");
	if (IS_ERR(scm_perf_client))
		dev_warn(adsp->dev, "Crypto scaling not setup\n");

get_rproc_client:
	adsp->bus_client = of_icc_get(adsp->dev, "rproc_ddr");
	if (IS_ERR(adsp->bus_client))
		dev_warn(adsp->dev, "%s: No bus client\n", __func__);

	rproc_ddr_set_icc_low_svs = of_property_read_bool(adsp->dev->of_node,
				     "rproc-ddr-set-icc-low-svs");

	if (rproc_ddr_set_icc_low_svs) {
		adsp->rproc_ddr_set_icc_low_svs = rproc_ddr_set_icc_low_svs;
		ret = of_property_read_u32(adsp->dev->of_node,
				"rproc-ddr-lowsvs-icc-bw", &rproc_ddr_lowsvs_icc_bw);
		if (!rproc_ddr_lowsvs_icc_bw) {
			dev_err(adsp->dev, "Low SVS ICC Bw is not available. Falling back to Turbo..\n");
			adsp->rproc_ddr_lowsvs_icc_bw = UINT_MAX;
		} else {
			adsp->rproc_ddr_lowsvs_icc_bw = rproc_ddr_lowsvs_icc_bw;
			dev_info(adsp->dev, "LowSVS Bus BW: %d\n", rproc_ddr_lowsvs_icc_bw);
		}
	} else {
		adsp->rproc_ddr_set_icc_low_svs = false;
		dev_info(adsp->dev, "Low SVS vote for ICC is not set\n");
	}
}

static int adsp_pds_attach(struct device *dev, struct device **devs,
			   char **pd_names)
{
	size_t num_pds = 0;
	int ret;
	int i;

	if (!pd_names)
		return 0;

	while (pd_names[num_pds])
		num_pds++;

	/* Handle single power domain */
	if (num_pds == 1 && dev->pm_domain) {
		devs[0] = dev;
		pm_runtime_enable(dev);
		return 1;
	}

	for (i = 0; i < num_pds; i++) {
		devs[i] = dev_pm_domain_attach_by_name(dev, pd_names[i]);
		if (IS_ERR_OR_NULL(devs[i])) {
			ret = PTR_ERR(devs[i]) ? : -ENODATA;
			goto unroll_attach;
		}
	}

	return num_pds;

unroll_attach:
	for (i--; i >= 0; i--)
		dev_pm_domain_detach(devs[i], false);

	return ret;
};

static void adsp_pds_detach(struct qcom_adsp *adsp, struct device **pds,
			    size_t pd_count)
{
	struct device *dev = adsp->dev;
	int i;

	/* Handle single power domain */
	if (pd_count == 1 && dev->pm_domain) {
		pm_runtime_disable(dev);
		return;
	}

	for (i = 0; i < pd_count; i++)
		dev_pm_domain_detach(pds[i], false);
}

static int adsp_alloc_memory_region(struct qcom_adsp *adsp)
{
	struct device_node *node;
	struct resource r;
	int ret;

	node = of_parse_phandle(adsp->dev->of_node, "memory-region", 0);
	if (!node) {
		dev_err(adsp->dev, "no memory-region specified\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(node, 0, &r);
	of_node_put(node);
	if (ret)
		return ret;

	adsp->mem_phys = adsp->mem_reloc = r.start;
	adsp->mem_size = resource_size(&r);
	adsp->mem_region = devm_ioremap_wc(adsp->dev, adsp->mem_phys, adsp->mem_size);
	if (!adsp->mem_region) {
		dev_err(adsp->dev, "unable to map memory region: %pa+%zx\n",
			&r.start, adsp->mem_size);
		return -EBUSY;
	}

	if (!adsp->dtb_pas_id)
		return 0;

	node = of_parse_phandle(adsp->dev->of_node, "memory-region", 1);
	if (!node) {
		dev_err(adsp->dev, "no dtb memory-region specified\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(node, 0, &r);
	if (ret)
		return ret;

	adsp->dtb_mem_phys = adsp->dtb_mem_reloc = r.start;
	adsp->dtb_mem_size = resource_size(&r);
	adsp->dtb_mem_region = devm_ioremap_wc(adsp->dev, adsp->dtb_mem_phys, adsp->dtb_mem_size);
	if (!adsp->dtb_mem_region) {
		dev_err(adsp->dev, "unable to map memory region: %pa+%zx\n",
			&r.start, adsp->dtb_mem_size);
		return -EBUSY;
	}

	return 0;
}

static int adsp_setup_32b_dma_allocs(struct qcom_adsp *adsp)
{
	int ret;

	if (!adsp->dma_phys_below_32b)
		return 0;

	ret = of_reserved_mem_device_init_by_idx(adsp->dev, adsp->dev->of_node, 2);
	if (ret) {
		dev_err(adsp->dev,
			"Unable to get the CMA area for performing dma_alloc_* calls\n");
		goto out;
	}

	ret = dma_set_mask_and_coherent(adsp->dev, DMA_BIT_MASK(32));
	if (ret)
		dev_err(adsp->dev, "Unable to set the coherent mask to 32-bits!\n");

out:
	return ret;
}

static void android_vh_rproc_recovery_set(void *data, struct rproc *rproc)
{
	struct qcom_adsp *adsp = (struct qcom_adsp *)rproc->priv;

	if (strstr(rproc->name, "spss"))
		return;
	adsp->subsys_recovery_disabled = rproc->recovery_disabled;
}

void qcom_rproc_update_recovery_status(struct rproc *rproc, bool enable)
{
	struct qcom_adsp *adsp;

	if (!rproc)
		return;

	adsp = (struct qcom_adsp *)rproc->priv;
	mutex_lock(&rproc->lock);
	if (enable) {
		/* Save recovery flag */
		adsp->subsys_recovery_disabled = rproc->recovery_disabled;
		rproc->recovery_disabled = !enable;
		pr_info("qcom rproc: %s: recovery enabled by kernel client\n", rproc->name);
	} else {
		/* Restore recovery flag */
		rproc->recovery_disabled = adsp->subsys_recovery_disabled;
		pr_info("qcom rproc: %s: recovery disabled by kernel client\n", rproc->name);
	}
	mutex_unlock(&rproc->lock);
}
EXPORT_SYMBOL(qcom_rproc_update_recovery_status);

static int adsp_probe(struct platform_device *pdev)
{
	const struct adsp_data *desc;
	struct qcom_adsp *adsp;
	struct rproc *rproc;
	const char *fw_name;
	const struct rproc_ops *ops = &adsp_ops;
	char md_dev_name[32];
	int ret;
	bool signal_aop;

	desc = of_device_get_match_data(&pdev->dev);
	if (!desc)
		return -EINVAL;

	if (!qcom_scm_is_available())
		return -EPROBE_DEFER;

	fw_name = desc->firmware_name;
	ret = of_property_read_string(pdev->dev.of_node, "firmware-name",
				      &fw_name);
	if (ret < 0 && ret != -EINVAL)
		return ret;


	if (desc->minidump_id)
		ops = &adsp_minidump_ops;

	rproc = rproc_alloc(&pdev->dev, pdev->name, ops, fw_name, sizeof(*adsp));

	if (!rproc) {
		dev_err(&pdev->dev, "unable to allocate remoteproc\n");
		return -ENOMEM;
	}

	rproc->recovery_disabled = true;
	rproc->auto_boot = desc->auto_boot;
	if (desc->uses_elf64)
		rproc_coredump_set_elf_info(rproc, ELFCLASS64, EM_NONE);
	else
		rproc_coredump_set_elf_info(rproc, ELFCLASS32, EM_NONE);

	adsp = (struct qcom_adsp *)rproc->priv;
	adsp->dev = &pdev->dev;
	adsp->rproc = rproc;
	adsp->minidump_id = desc->minidump_id;
	adsp->pas_id = desc->pas_id;
	adsp->dtb_pas_id = desc->dtb_pas_id;
	adsp->hyp_assign_mem = desc->hyp_assign_mem;
	ret = qcom_rproc_alloc_dtb_firmware(adsp, desc->dtb_firmware_name);
	if (ret)
		goto free_rproc;
	adsp->info_name = desc->sysmon_name;
	adsp->smem_host_id = desc->smem_host_id;
	adsp->decrypt_shutdown = desc->decrypt_shutdown;
	adsp->qmp_name = desc->qmp_name;
	adsp->dma_phys_below_32b = desc->dma_phys_below_32b;
	adsp->both_dumps = desc->both_dumps;
	adsp->subsys_recovery_disabled = true;
	adsp->check_status = desc->check_status;

	if (desc->free_after_auth_reset) {
		adsp->mdata = devm_kzalloc(adsp->dev, sizeof(struct qcom_mdt_metadata), GFP_KERNEL);
		adsp->retry_shutdown = true;
	}

	if (desc->ssr_hyp_assign_mem) {
		ret = setup_mpss_dsm_mem(adsp);
		if (ret) {
			dev_err(adsp->dev, "failed to parse mpss dsm mem\n");
			goto free_dtb_firmware;
		}
		adsp->ssr_hyp_assign_mem = true;
	}

	platform_set_drvdata(pdev, adsp);

	ret = device_init_wakeup(adsp->dev, true);
	if (ret)
		goto free_dtb_firmware;

	ret = adsp_alloc_memory_region(adsp);
	if (ret)
		goto deinit_wakeup_source;

	ret = adsp_setup_32b_dma_allocs(adsp);
	if (ret)
		goto deinit_wakeup_source;

	ret = adsp_init_clock(adsp);
	if (ret)
		goto deinit_wakeup_source;

	ret = adsp_init_regulator(adsp);
	if (ret)
		goto deinit_wakeup_source;

	adsp_init_bus_scaling(adsp);

	ret = adsp_pds_attach(&pdev->dev, adsp->active_pds,
			      desc->active_pd_names);
	if (ret < 0)
		goto deinit_wakeup_source;
	adsp->active_pd_count = ret;

	ret = adsp_pds_attach(&pdev->dev, adsp->proxy_pds,
			      desc->proxy_pd_names);
	if (ret < 0)
		goto detach_active_pds;
	adsp->proxy_pd_count = ret;

	signal_aop = of_property_read_bool(pdev->dev.of_node,
			"qcom,signal-aop");

	if (signal_aop) {
		adsp->qmp = qmp_get(adsp->dev);
		if (IS_ERR_OR_NULL(adsp->qmp))
			goto detach_proxy_pds;
	}

	ret = qcom_q6v5_init(&adsp->q6v5, pdev, rproc, desc->crash_reason_smem,
			     desc->early_boot, qcom_pas_handover);

	if (ret)
		goto detach_proxy_pds;

	if (adsp->check_status) {
		if (rproc_find_status_register(adsp))
			goto detach_proxy_pds;
		adsp->wake_state = devm_qcom_smem_state_get(&pdev->dev, "wakeup", &adsp->wake_bit);

		if (IS_ERR(adsp->wake_state)) {
			dev_err(&pdev->dev, "failed to acquire wake state\n");
			goto detach_proxy_pds;
		}

		adsp->sleep_state = devm_qcom_smem_state_get(&pdev->dev, "sleep", &adsp->sleep_bit);

		if (IS_ERR(adsp->sleep_state)) {
			dev_err(&pdev->dev, "failed to acquire sleep state\n");
			goto detach_proxy_pds;
		}

		mutex_init(&adsp->adsp_lock);

		adsp->current_users = 0;
	}

	qcom_q6v5_register_ssr_subdev(&adsp->q6v5, &adsp->ssr_subdev.subdev);

	if (adsp->q6v5.rmb_base &&
			readl_relaxed(adsp->q6v5.rmb_base + RMB_Q6_BOOT_STATUS_REG))
		rproc->state = RPROC_DETACHED;

	timeout_disabled = qcom_pil_timeouts_disabled();
	qcom_add_glink_subdev(rproc, &adsp->glink_subdev, desc->ssr_name);
	qcom_add_smd_subdev(rproc, &adsp->smd_subdev);
	adsp->sysmon = qcom_add_sysmon_subdev(rproc,
					      desc->sysmon_name,
					      desc->ssctl_id);
	if (IS_ERR(adsp->sysmon)) {
		ret = PTR_ERR(adsp->sysmon);
		goto detach_proxy_pds;
	}

	qcom_add_ssr_subdev(rproc, &adsp->ssr_subdev, desc->ssr_name);
	ret = device_create_file(adsp->dev, &dev_attr_txn_id);
	if (ret)
		goto remove_subdevs;

	snprintf(md_dev_name, ARRAY_SIZE(md_dev_name), "%s-md", pdev->dev.of_node->name);
	adsp->minidump_dev = qcom_create_ramdump_device(md_dev_name, NULL);
	if (!adsp->minidump_dev)
		dev_err(&pdev->dev, "Unable to create %s minidump device.\n", md_dev_name);


	/*
	 * Read back the smp2p slave bits to check if the Subsystem has been
	 * brought out of reset by another entitiy before kernel entry
	 */
	if (adsp->q6v5.early_boot) {
		adsp->rproc->state = RPROC_DETACHED;
		ret = ping_subsystem_init(&adsp->q6v5, pdev);
		if (ret) {
			dev_err(&pdev->dev, "Unable to find ping/pong bits\n");
			qcom_remove_sysmon_subdev(adsp->sysmon);
			return ret;
		}
	}

	ret = rproc_add(rproc);
	if (ret)
		goto destroy_minidump_dev;

	mutex_lock(&q6v5_pas_mutex);
	if (!recovery_set_cb) {
		ret = register_trace_android_vh_rproc_recovery_set(android_vh_rproc_recovery_set,
											NULL);
		if (ret) {
			dev_err(&pdev->dev, "Unable to register with rproc_recovery_set trace hook\n");
			mutex_unlock(&q6v5_pas_mutex);
			goto remove_rproc;
		}
		recovery_set_cb = true;
	}
	mutex_unlock(&q6v5_pas_mutex);

	if (adsp->check_status) {
		adsp->panic_blk.priority = INT_MAX - 2;
		adsp->panic_blk.notifier_call = rproc_panic_handler;
		atomic_notifier_chain_register(&panic_notifier_list, &adsp->panic_blk);
	}

	return 0;

remove_rproc:
	rproc_del(rproc);
destroy_minidump_dev:
	if (adsp->minidump_dev)
		qcom_destroy_ramdump_device(adsp->minidump_dev);

	device_remove_file(adsp->dev, &dev_attr_txn_id);
remove_subdevs:
	qcom_remove_sysmon_subdev(adsp->sysmon);
detach_proxy_pds:
	adsp_pds_detach(adsp, adsp->proxy_pds, adsp->proxy_pd_count);
detach_active_pds:
	adsp_pds_detach(adsp, adsp->active_pds, adsp->active_pd_count);
deinit_wakeup_source:
	device_init_wakeup(adsp->dev, false);
free_dtb_firmware:
	if (adsp->dtb_fw_name)
		kfree_const(adsp->dtb_fw_name);
free_rproc:
	device_init_wakeup(adsp->dev, false);
	rproc_free(rproc);

	return ret;
}

static int adsp_remove(struct platform_device *pdev)
{
	struct qcom_adsp *adsp = platform_get_drvdata(pdev);

	unregister_trace_android_vh_rproc_recovery_set(android_vh_rproc_recovery_set, NULL);
	if (adsp->dtb_fw_name)
		kfree_const(adsp->dtb_fw_name);
	rproc_del(adsp->rproc);
	if (adsp->minidump_dev)
		qcom_destroy_ramdump_device(adsp->minidump_dev);
	device_remove_file(adsp->dev, &dev_attr_txn_id);
	qcom_remove_glink_subdev(adsp->rproc, &adsp->glink_subdev);
	qcom_remove_sysmon_subdev(adsp->sysmon);
	qcom_remove_smd_subdev(adsp->rproc, &adsp->smd_subdev);
	qcom_remove_ssr_subdev(adsp->rproc, &adsp->ssr_subdev);
	if (adsp->check_status)
		atomic_notifier_chain_unregister(&panic_notifier_list, &adsp->panic_blk);
	adsp_pds_detach(adsp, adsp->proxy_pds, adsp->proxy_pd_count);
	device_init_wakeup(adsp->dev, false);
	rproc_free(adsp->rproc);

	return 0;
}

static const struct adsp_data adsp_resource_init = {
		.crash_reason_smem = 423,
		.firmware_name = "adsp.mdt",
		.pas_id = 1,
		.auto_boot = true,
		.ssr_name = "lpass",
		.sysmon_name = "adsp",
		.ssctl_id = 0x14,
};

static const struct adsp_data sm6150_adsp_resource = {
		.crash_reason_smem = 423,
		.firmware_name = "adsp.mdt",
		.pas_id = 1,
		.minidump_id = 5,
		.uses_elf64 = true,
		.auto_boot = true,
		.ssr_name = "lpass",
		.sysmon_name = "adsp",
		.qmp_name = "adsp",
		.ssctl_id = 0x14,
};

static const struct adsp_data sm6150_cdsp_resource = {
		.crash_reason_smem = 601,
		.firmware_name = "cdsp.mdt",
		.pas_id = 18,
		.minidump_id = 7,
		.uses_elf64 = true,
		.auto_boot = true,
		.ssr_name = "cdsp",
		.sysmon_name = "cdsp",
		.qmp_name = "cdsp",
		.ssctl_id = 0x17,
};

static const struct adsp_data sm8150_adsp_resource = {
		.crash_reason_smem = 423,
		.firmware_name = "adsp.mdt",
		.pas_id = 1,
		.minidump_id = 5,
		.uses_elf64 = true,
		.auto_boot = true,
		.ssr_name = "lpass",
		.sysmon_name = "adsp",
		.qmp_name = "adsp",
		.ssctl_id = 0x14,
};

static const struct adsp_data sm8150_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.minidump_id = 3,
	.has_aggre2_clk = false,
	.auto_boot = true,
	.ssr_name = "mpss",
	.qmp_name = "modem",
	.sysmon_name = "modem",
	.ssctl_id = 0x12,
};

static const struct adsp_data sm8250_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.minidump_id = 5,
	.auto_boot = true,
	.active_pd_names = (char*[]){
		"load_state",
		NULL
	},
	.proxy_pd_names = (char*[]){
		"lcx",
		"lmx",
		NULL
	},
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data sm8350_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.auto_boot = true,
	.active_pd_names = (char*[]){
		"load_state",
		NULL
	},
	.proxy_pd_names = (char*[]){
		"lcx",
		"lmx",
		NULL
	},
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data waipio_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.minidump_id = 5,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.qmp_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data kalama_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.dtb_firmware_name = "adsp_dtb.mdt",
	.pas_id = 1,
	.dtb_pas_id = 0x24,
	.minidump_id = 5,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.qmp_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data pineapple_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.dtb_firmware_name = "adsp_dtb.mdt",
	.pas_id = 1,
	.dtb_pas_id = 0x24,
	.minidump_id = 5,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.qmp_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data niobe_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.dtb_firmware_name = "adsp_dtb.mdt",
	.pas_id = 1,
	.dtb_pas_id = 0x24,
	.minidump_id = 5,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.qmp_name = "adsp",
	.ssctl_id = 0x14,
	.smem_host_id = 2,
};

static const struct adsp_data seraph_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.dtb_firmware_name = "adsp_dtb.mdt",
	.pas_id = 1,
	.dtb_pas_id = 0x24,
	.minidump_id = 5,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.qmp_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data cliffs_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.dtb_firmware_name = "adsp_dtb.mdt",
	.pas_id = 1,
	.dtb_pas_id = 0x24,
	.minidump_id = 5,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.qmp_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data volcano_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.dtb_firmware_name = "adsp_dtb.mdt",
	.pas_id = 1,
	.dtb_pas_id = 0x24,
	.minidump_id = 5,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.qmp_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data khaje_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.minidump_id = 5,
	.uses_elf64 = false,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data msm8998_adsp_resource = {
		.crash_reason_smem = 423,
		.firmware_name = "adsp.mdt",
		.pas_id = 1,
		.auto_boot = true,
		.proxy_pd_names = (char*[]){
			"cx",
			NULL
		},
		.ssr_name = "lpass",
		.sysmon_name = "adsp",
		.ssctl_id = 0x14,
};

static const struct adsp_data blair_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.minidump_id = 5,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.qmp_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data holi_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.minidump_id = 5,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.qmp_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data pitti_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.minidump_id = 5,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.qmp_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data cdsp_resource_init = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.auto_boot = true,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data sm8150_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.minidump_id = 7,
	.uses_elf64 = true,
	.auto_boot = true,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.qmp_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data sm8250_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.auto_boot = true,
	.active_pd_names = (char*[]){
		"load_state",
		NULL
	},
	.proxy_pd_names = (char*[]){
		"cx",
		NULL
	},
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data sm8350_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.has_aggre2_clk = false,
	.auto_boot = true,
	.active_pd_names = (char*[]){
		"load_state",
		NULL
	},
	.proxy_pd_names = (char*[]){
		"cx",
		"mxc",
		NULL
	},
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data waipio_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.minidump_id = 7,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.qmp_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data sc8280xp_nsp0_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"nsp",
		NULL
	},
	.ssr_name = "cdsp0",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data sc8280xp_nsp1_resource = {
	.crash_reason_smem = 633,
	.firmware_name = "cdsp.mdt",
	.pas_id = 30,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"nsp",
		NULL
	},
	.ssr_name = "cdsp1",
	.sysmon_name = "cdsp1",
	.ssctl_id = 0x20,
};

static const struct adsp_data kalama_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.dtb_firmware_name = "cdsp_dtb.mdt",
	.pas_id = 18,
	.dtb_pas_id = 0x25,
	.minidump_id = 7,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.qmp_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data pineapple_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.dtb_firmware_name = "cdsp_dtb.mdt",
	.pas_id = 18,
	.dtb_pas_id = 0x25,
	.minidump_id = 7,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.hyp_assign_mem = true,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.qmp_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data niobe_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.dtb_firmware_name = "cdsp_dtb.mdt",
	.pas_id = 18,
	.dtb_pas_id = 0x25,
	.minidump_id = 7,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.hyp_assign_mem = true,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.qmp_name = "cdsp",
	.ssctl_id = 0x17,
	.smem_host_id = 5,
};

static const struct adsp_data seraph_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.dtb_firmware_name = "cdsp_dtb.mdt",
	.pas_id = 18,
	.dtb_pas_id = 0x25,
	.minidump_id = 7,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.hyp_assign_mem = false,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.qmp_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data cliffs_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.dtb_firmware_name = "cdsp_dtb.mdt",
	.pas_id = 18,
	.dtb_pas_id = 0x25,
	.minidump_id = 7,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.hyp_assign_mem = true,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.qmp_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data volcano_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.dtb_firmware_name = "cdsp_dtb.mdt",
	.pas_id = 18,
	.dtb_pas_id = 0x25,
	.minidump_id = 7,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.qmp_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data anorak_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.minidump_id = 5,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.qmp_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data anorak_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.minidump_id = 7,
	.uses_elf64 = true,
	.auto_boot = false,
	.hyp_assign_mem = true,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.qmp_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data neo_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.minidump_id = 5,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.qmp_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data neo_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.minidump_id = 7,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.hyp_assign_mem = true,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.qmp_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data khaje_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.minidump_id = 7,
	.uses_elf64 = false,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data blair_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.minidump_id = 7,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.qmp_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data holi_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.minidump_id = 7,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.qmp_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data mpss_resource_init = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.minidump_id = 3,
	.auto_boot = false,
	.active_pd_names = (char*[]){
		"load_state",
		NULL
	},
	.proxy_pd_names = (char*[]){
		"cx",
		"mss",
		NULL
	},
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.ssctl_id = 0x12,
};

static const struct adsp_data waipio_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.free_after_auth_reset = true,
	.minidump_id = 3,
	.uses_elf64 = true,
	.auto_boot = false,
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.qmp_name = "modem",
	.ssctl_id = 0x12,
};

static const struct adsp_data kalama_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.dtb_firmware_name = "modem_dtb.mdt",
	.pas_id = 4,
	.dtb_pas_id = 0x26,
	.free_after_auth_reset = true,
	.minidump_id = 3,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.qmp_name = "modem",
	.ssctl_id = 0x12,
	.dma_phys_below_32b = true,
};

static const struct adsp_data pineapple_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.dtb_firmware_name = "modem_dtb.mdt",
	.pas_id = 4,
	.dtb_pas_id = 0x26,
	.free_after_auth_reset = true,
	.minidump_id = 3,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_hyp_assign_mem = true,
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.qmp_name = "modem",
	.ssctl_id = 0x12,
	.dma_phys_below_32b = true,
	.both_dumps = true,
};

static const struct adsp_data cliffs_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.dtb_firmware_name = "modem_dtb.mdt",
	.pas_id = 4,
	.dtb_pas_id = 0x26,
	.free_after_auth_reset = true,
	.minidump_id = 3,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_hyp_assign_mem = true,
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.qmp_name = "modem",
	.ssctl_id = 0x12,
	.dma_phys_below_32b = true,
	.both_dumps = true,
};

static const struct adsp_data volcano_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.free_after_auth_reset = true,
	.minidump_id = 3,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.qmp_name = "modem",
	.ssctl_id = 0x12,
	.dma_phys_below_32b = true,
	.both_dumps = true,
};

static const struct adsp_data cinder_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.free_after_auth_reset = true,
	.minidump_id = 3,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.qmp_name = "modem",
	.ssctl_id = 0x12,
};

static const struct adsp_data khaje_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.free_after_auth_reset = true,
	.minidump_id = 3,
	.uses_elf64 = true,
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.ssctl_id = 0x12,
};

static const struct adsp_data blair_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.free_after_auth_reset = true,
	.minidump_id = 3,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.qmp_name = "modem",
	.ssctl_id = 0x12,
};

static const struct adsp_data holi_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.free_after_auth_reset = true,
	.minidump_id = 3,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.qmp_name = "modem",
	.ssctl_id = 0x12,
};

static const struct adsp_data pitti_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.free_after_auth_reset = true,
	.minidump_id = 3,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.qmp_name = "modem",
	.ssctl_id = 0x12,
};

static const struct adsp_data slpi_resource_init = {
		.crash_reason_smem = 424,
		.firmware_name = "slpi.mdt",
		.pas_id = 12,
		.auto_boot = true,
		.ssr_name = "dsps",
		.sysmon_name = "slpi",
		.ssctl_id = 0x16,
};

static const struct adsp_data sm8150_slpi_resource = {
		.crash_reason_smem = 424,
		.firmware_name = "slpi.mdt",
		.pas_id = 12,
		.has_aggre2_clk = false,
		.auto_boot = true,
		.active_pd_names = (char*[]){
			"load_state",
			NULL
		},
		.proxy_pd_names = (char*[]){
			"lcx",
			"lmx",
			NULL
		},
		.ssr_name = "dsps",
		.sysmon_name = "slpi",
		.ssctl_id = 0x16,
};

static const struct adsp_data sm8250_slpi_resource = {
	.crash_reason_smem = 424,
	.firmware_name = "slpi.mdt",
	.pas_id = 12,
	.has_aggre2_clk = false,
	.auto_boot = true,
	.active_pd_names = (char*[]){
		"load_state",
		NULL
	},
	.proxy_pd_names = (char*[]){
		"lcx",
		"lmx",
		NULL
	},
	.ssr_name = "dsps",
	.sysmon_name = "slpi",
	.ssctl_id = 0x16,
};

static const struct adsp_data sm8350_slpi_resource = {
	.crash_reason_smem = 424,
	.firmware_name = "slpi.mdt",
	.pas_id = 12,
	.has_aggre2_clk = false,
	.auto_boot = true,
	.active_pd_names = (char*[]){
		"load_state",
		NULL
	},
	.proxy_pd_names = (char*[]){
		"lcx",
		"lmx",
		NULL
	},
	.ssr_name = "dsps",
	.sysmon_name = "slpi",
	.ssctl_id = 0x16,
};

static const struct adsp_data waipio_slpi_resource = {
	.crash_reason_smem = 424,
	.firmware_name = "slpi.mdt",
	.pas_id = 12,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "dsps",
	.sysmon_name = "slpi",
	.qmp_name = "slpi",
	.ssctl_id = 0x16,
};

static const struct adsp_data msm8998_slpi_resource = {
		.crash_reason_smem = 424,
		.firmware_name = "slpi.mdt",
		.pas_id = 12,
		.has_aggre2_clk = true,
		.auto_boot = true,
		.proxy_pd_names = (char*[]){
			"ssc_cx",
			NULL
		},
		.ssr_name = "dsps",
		.sysmon_name = "slpi",
		.ssctl_id = 0x16,
};

static const struct adsp_data wcss_resource_init = {
	.crash_reason_smem = 421,
	.firmware_name = "wcnss.mdt",
	.pas_id = 6,
	.auto_boot = true,
	.ssr_name = "mpss",
	.sysmon_name = "wcnss",
	.ssctl_id = 0x12,
};

static const struct adsp_data sdx55_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.auto_boot = true,
	.proxy_pd_names = (char*[]){
		"cx",
		"mss",
		NULL
	},
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.ssctl_id = 0x22,
};

static const struct adsp_data sc8180x_mpss_resource = {
	.crash_reason_smem = 421,
	.firmware_name = "modem.mdt",
	.pas_id = 4,
	.auto_boot = false,
	.active_pd_names = (char*[]){
		"load_state",
		NULL
	},
	.proxy_pd_names = (char*[]){
		"cx",
		NULL
	},
	.ssr_name = "mpss",
	.sysmon_name = "modem",
	.ssctl_id = 0x12,
};

static const struct adsp_data sdmshrike_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.minidump_id = 5,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = true,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.qmp_name = "adsp",
	.ssctl_id = 0x14,
};

static const struct adsp_data sdmshrike_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp.mdt",
	.pas_id = 18,
	.minidump_id = 7,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = true,
	.ssr_name = "lpass",
	.sysmon_name = "cdsp",
	.qmp_name = "cdsp",
	.ssctl_id = 0x17,
};

static const struct adsp_data monaco_auto_adsp_resource = {
	.crash_reason_smem = 423,
	.firmware_name = "adsp.mdt",
	.pas_id = 1,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "lpass",
	.sysmon_name = "adsp",
	.qmp_name = "adsp",
	.ssctl_id = 0x14,
	.minidump_id = 5,
};

static const struct adsp_data monaco_auto_cdsp_resource = {
	.crash_reason_smem = 601,
	.firmware_name = "cdsp0.mdt",
	.pas_id = 18,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "cdsp",
	.sysmon_name = "cdsp",
	.qmp_name = "cdsp",
	.ssctl_id = 0x17,
	.minidump_id = 7,
};

static const struct adsp_data niobe_soccp_resource = {
	.crash_reason_smem = 656,
	.firmware_name = "soccp.mbn",
	.pas_id = 51,
	.ssr_name = "soccp",
	.sysmon_name = "soccp",
	.check_status = true,
	.auto_boot = true,
};

static const struct adsp_data seraph_soccp_resource = {
	.crash_reason_smem = 656,
	.firmware_name = "soccp.mbn",
	.pas_id = 51,
	.ssr_name = "soccp",
	.sysmon_name = "soccp",
	.check_status = true,
	.early_boot = true,
	.auto_boot = true,
};

static const struct adsp_data monaco_auto_gpdsp_resource = {
	.crash_reason_smem = 640,
	.firmware_name = "gpdsp0.mdt",
	.pas_id = 39,
	.uses_elf64 = true,
	.has_aggre2_clk = false,
	.auto_boot = false,
	.ssr_name = "gpdsp0",
	.sysmon_name = "gpdsp0",
	.qmp_name = "gpdsp0",
	.ssctl_id = 0x21,
	.minidump_id = 21,
};

static const struct adsp_data cliffs_wpss_resource = {
	.crash_reason_smem = 626,
	.firmware_name = "wpss.mdt",
	.pas_id = 6,
	.minidump_id = 4,
	.uses_elf64 = true,
	.ssr_name = "wpss",
	.sysmon_name = "wpss",
	.qmp_name = "wpss",
	.ssctl_id = 0x19,
};

static const struct adsp_data pitti_wpss_resource = {
	.crash_reason_smem = 626,
	.firmware_name = "wpss.mdt",
	.pas_id = 6,
	.minidump_id = 4,
	.uses_elf64 = true,
	.ssr_name = "wpss",
	.sysmon_name = "wpss",
	.qmp_name = "wpss",
	.ssctl_id = 0x19,
};

static const struct adsp_data volcano_wpss_resource = {
	.crash_reason_smem = 626,
	.firmware_name = "wpss.mdt",
	.pas_id = 6,
	.minidump_id = 4,
	.uses_elf64 = true,
	.ssr_name = "wpss",
	.sysmon_name = "wpss",
	.qmp_name = "wpss",
	.ssctl_id = 0x19,
};

static const struct of_device_id adsp_of_match[] = {
	{ .compatible = "qcom,msm8226-adsp-pil", .data = &adsp_resource_init},
	{ .compatible = "qcom,msm8974-adsp-pil", .data = &adsp_resource_init},
	{ .compatible = "qcom,msm8996-adsp-pil", .data = &adsp_resource_init},
	{ .compatible = "qcom,msm8996-slpi-pil", .data = &slpi_resource_init},
	{ .compatible = "qcom,msm8998-adsp-pas", .data = &msm8998_adsp_resource},
	{ .compatible = "qcom,msm8998-slpi-pas", .data = &msm8998_slpi_resource},
	{ .compatible = "qcom,qcs404-adsp-pas", .data = &adsp_resource_init },
	{ .compatible = "qcom,qcs404-cdsp-pas", .data = &cdsp_resource_init },
	{ .compatible = "qcom,qcs404-wcss-pas", .data = &wcss_resource_init },
	{ .compatible = "qcom,sc7180-mpss-pas", .data = &mpss_resource_init},
	{ .compatible = "qcom,sc8180x-adsp-pas", .data = &sm8150_adsp_resource},
	{ .compatible = "qcom,sc8180x-cdsp-pas", .data = &sm8150_cdsp_resource},
	{ .compatible = "qcom,sc8180x-mpss-pas", .data = &sc8180x_mpss_resource},
	{ .compatible = "qcom,sc8280xp-adsp-pas", .data = &sm8250_adsp_resource},
	{ .compatible = "qcom,sc8280xp-nsp0-pas", .data = &sc8280xp_nsp0_resource},
	{ .compatible = "qcom,sc8280xp-nsp1-pas", .data = &sc8280xp_nsp1_resource},
	{ .compatible = "qcom,sdm660-adsp-pas", .data = &adsp_resource_init},
	{ .compatible = "qcom,sdm845-adsp-pas", .data = &adsp_resource_init},
	{ .compatible = "qcom,sdm845-cdsp-pas", .data = &cdsp_resource_init},
	{ .compatible = "qcom,sdx55-mpss-pas", .data = &sdx55_mpss_resource},
	{ .compatible = "qcom,sm6150-adsp-pas", .data = &sm6150_adsp_resource},
	{ .compatible = "qcom,sm6150-cdsp-pas", .data = &sm6150_cdsp_resource},
	{ .compatible = "qcom,sm8150-adsp-pas", .data = &sm8150_adsp_resource},
	{ .compatible = "qcom,sm8150-cdsp-pas", .data = &sm8150_cdsp_resource},
	{ .compatible = "qcom,sm8150-mpss-pas", .data = &sm8150_mpss_resource},
	{ .compatible = "qcom,sm8150-slpi-pas", .data = &sm8150_slpi_resource},
	{ .compatible = "qcom,sm8250-adsp-pas", .data = &sm8250_adsp_resource},
	{ .compatible = "qcom,sm8250-cdsp-pas", .data = &sm8250_cdsp_resource},
	{ .compatible = "qcom,sm8250-slpi-pas", .data = &sm8250_slpi_resource},
	{ .compatible = "qcom,sm8350-adsp-pas", .data = &sm8350_adsp_resource},
	{ .compatible = "qcom,sm8350-cdsp-pas", .data = &sm8350_cdsp_resource},
	{ .compatible = "qcom,sm8350-slpi-pas", .data = &sm8350_slpi_resource},
	{ .compatible = "qcom,sm8350-mpss-pas", .data = &mpss_resource_init},
	{ .compatible = "qcom,waipio-adsp-pas", .data = &waipio_adsp_resource},
	{ .compatible = "qcom,waipio-cdsp-pas", .data = &waipio_cdsp_resource},
	{ .compatible = "qcom,waipio-slpi-pas", .data = &waipio_slpi_resource},
	{ .compatible = "qcom,waipio-modem-pas", .data = &waipio_mpss_resource},
	{ .compatible = "qcom,kalama-adsp-pas", .data = &kalama_adsp_resource},
	{ .compatible = "qcom,kalama-cdsp-pas", .data = &kalama_cdsp_resource},
	{ .compatible = "qcom,kalama-modem-pas", .data = &kalama_mpss_resource},
	{ .compatible = "qcom,pineapple-adsp-pas", .data = &pineapple_adsp_resource},
	{ .compatible = "qcom,pineapple-modem-pas", .data = &pineapple_mpss_resource},
	{ .compatible = "qcom,pineapple-cdsp-pas", .data = &pineapple_cdsp_resource},
	{ .compatible = "qcom,niobe-adsp-pas", .data = &niobe_adsp_resource},
	{ .compatible = "qcom,niobe-cdsp-pas", .data = &niobe_cdsp_resource},
	{ .compatible = "qcom,seraph-adsp-pas", .data = &seraph_adsp_resource},
	{ .compatible = "qcom,seraph-cdsp-pas", .data = &seraph_cdsp_resource},
	{ .compatible = "qcom,cinder-modem-pas", .data = &cinder_mpss_resource},
	{ .compatible = "qcom,khaje-adsp-pas", .data = &khaje_adsp_resource},
	{ .compatible = "qcom,khaje-cdsp-pas", .data = &khaje_cdsp_resource},
	{ .compatible = "qcom,khaje-modem-pas", .data = &khaje_mpss_resource},
	{ .compatible = "qcom,sdmshrike-adsp-pas", .data = &sdmshrike_adsp_resource},
	{ .compatible = "qcom,sdmshrike-cdsp-pas", .data = &sdmshrike_cdsp_resource},
	{ .compatible = "qcom,blair-adsp-pas", .data = &blair_adsp_resource},
	{ .compatible = "qcom,blair-cdsp-pas", .data = &blair_cdsp_resource},
	{ .compatible = "qcom,blair-modem-pas", .data = &blair_mpss_resource},
	{ .compatible = "qcom,monaco_auto-adsp-pas", .data = &monaco_auto_adsp_resource},
	{ .compatible = "qcom,monaco_auto-cdsp-pas", .data = &monaco_auto_cdsp_resource},
	{ .compatible = "qcom,monaco_auto-gpdsp-pas", .data = &monaco_auto_gpdsp_resource},
	{ .compatible = "qcom,holi-adsp-pas", .data = &holi_adsp_resource},
	{ .compatible = "qcom,holi-cdsp-pas", .data = &holi_cdsp_resource},
	{ .compatible = "qcom,holi-modem-pas", .data = &holi_mpss_resource},
	{ .compatible = "qcom,cliffs-adsp-pas", .data = &cliffs_adsp_resource},
	{ .compatible = "qcom,cliffs-modem-pas", .data = &cliffs_mpss_resource},
	{ .compatible = "qcom,cliffs-cdsp-pas", .data = &cliffs_cdsp_resource},
	{ .compatible = "qcom,cliffs-wpss-pas", .data = &cliffs_wpss_resource},
	{ .compatible = "qcom,pitti-wpss-pas", .data = &pitti_wpss_resource},
	{ .compatible = "qcom,pitti-adsp-pas", .data = &pitti_adsp_resource},
	{ .compatible = "qcom,pitti-modem-pas", .data = &pitti_mpss_resource},
	{ .compatible = "qcom,niobe-soccp-pas", .data = &niobe_soccp_resource},
	{ .compatible = "qcom,seraph-soccp-pas", .data = &seraph_soccp_resource},
	{ .compatible = "qcom,volcano-wpss-pas", .data = &volcano_wpss_resource},
	{ .compatible = "qcom,volcano-adsp-pas", .data = &volcano_adsp_resource},
	{ .compatible = "qcom,volcano-modem-pas", .data = &volcano_mpss_resource},
	{ .compatible = "qcom,volcano-cdsp-pas", .data = &volcano_cdsp_resource},
	{ .compatible = "qcom,anorak-adsp-pas", .data = &anorak_adsp_resource},
	{ .compatible = "qcom,anorak-cdsp-pas", .data = &anorak_cdsp_resource},
	{ .compatible = "qcom,neo-adsp-pas", .data = &neo_adsp_resource},
	{ .compatible = "qcom,neo-cdsp-pas", .data = &neo_cdsp_resource},
	{ },
};
MODULE_DEVICE_TABLE(of, adsp_of_match);

static struct platform_driver adsp_driver = {
	.probe = adsp_probe,
	.remove = adsp_remove,
	.driver = {
		.name = "qcom_q6v5_pas",
		.of_match_table = adsp_of_match,
	},
};

module_platform_driver(adsp_driver);
MODULE_DESCRIPTION("Qualcomm Hexagon v5 Peripheral Authentication Service driver");
MODULE_LICENSE("GPL v2");
