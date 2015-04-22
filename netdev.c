/*******************************************************************************

  Intel PRO/1000 Linux driver
  Copyright(c) 1999 - 2013 Intel Corporation.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Contact Information:
  Linux NICS <linux.nics@intel.com>
  e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497

*******************************************************************************/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/vmalloc.h>
#include <linux/pagemap.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/tcp.h>
#include <linux/ipv6.h>
#include <linux/slab.h>
#include <net/checksum.h>
#include <net/ip6_checksum.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/pm_qos.h>
#include <linux/pm_runtime.h>
#include <linux/aer.h>
#include <linux/prefetch.h>

#include "e1000.h"

#define _E1000E_DRV_VERSION "2.3.2-k"
char e1000e_driver_name[] = "e1000e";
const char e1000e_driver_version[] = _E1000E_DRV_VERSION;

#define DEFAULT_MSG_ENABLE (NETIF_MSG_DRV|NETIF_MSG_PROBE|NETIF_MSG_LINK)
static int debug = -1;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug level (0=none,...,16=all)");

#define SPEED_MODE_BIT (1 << 21)

// code moved to _netdev_dbg.h
static void e1000e_dump(struct e1000_adapter *adapter);
static int  e1000_desc_unused(struct e1000_ring *ring);

// code moved to _netdev_rxtx.h
static void e1000e_set_rx_mode(struct net_device *netdev);
static void e1000_configure_rx(struct e1000_adapter *adapter);
static void e1000_setup_rctl(struct e1000_adapter *adapter);
static void e1000_configure_tx(struct e1000_adapter *adapter);

static void e1000_restore_vlan(struct e1000_adapter *adapter);
static void e1000_init_manageability_pt(struct e1000_adapter *adapter);
static void e1000_update_mng_vlan(struct e1000_adapter *adapter);

static int  e1000_vlan_rx_add_vid(struct net_device *netdev, __always_unused __be16 proto, u16 vid);
static int  e1000_vlan_rx_kill_vid(struct net_device *netdev, __always_unused __be16 proto, u16 vid);

static void e1000e_setup_rss_hash(struct e1000_adapter *adapter);

static void e1000_clean_rx_ring(struct e1000_ring *rx_ring);
static void e1000_clean_tx_ring(struct e1000_ring *tx_ring);

static int  e1000_alloc_queues(struct e1000_adapter *adapter);

static netdev_tx_t e1000_xmit_frame(struct sk_buff *skb, struct net_device *netdev);

static void e1000_tx_timeout(struct net_device *netdev);

static void e1000_configure_msix(struct e1000_adapter *adapter);
static void e1000_irq_disable(struct e1000_adapter *adapter);
static void e1000_irq_enable(struct e1000_adapter *adapter);
static int  e1000_request_irq(struct e1000_adapter *adapter);
static void e1000_free_irq(struct e1000_adapter *adapter);

static int  e1000_test_msi(struct e1000_adapter *adapter);

static int  e1000e_poll(struct napi_struct *napi, int weight);
static void e1000e_downshift_workaround(struct work_struct *work);

#ifdef CONFIG_NET_POLL_CONTROLLER
static void e1000_netpoll(struct net_device *netdev);
#endif/*CONFIG_NET_POLL_CONTROLLER*/

// code moved to _netdev_wd.h
static void e1000_watchdog(unsigned long data);
static void e1000_watchdog_task(struct work_struct *work);

static const struct e1000_info *e1000_info_tbl[] = {
	[board_ich8lan]		= &e1000_ich8_info,
	[board_ich9lan]		= &e1000_ich9_info,
	[board_ich10lan]	= &e1000_ich10_info,
	[board_pchlan]		= &e1000_pch_info,
	[board_pch2lan]		= &e1000_pch2_info,
	[board_pch_lpt]		= &e1000_pch_lpt_info,
};

/**
 * e1000e_systim_to_hwtstamp - convert system time value to hw time stamp
 * @adapter: board private structure
 * @hwtstamps: time stamp structure to update
 * @systim: unsigned 64bit system time value.
 *
 * Convert the system time value stored in the RX/TXSTMP registers into a
 * hwtstamp which can be used by the upper level time stamping functions.
 *
 * The 'systim_lock' spinlock is used to protect the consistency of the
 * system time value. This is needed because reading the 64 bit time
 * value involves reading two 32 bit registers. The first read latches the
 * value.
 **/
static void e1000e_systim_to_hwtstamp(struct e1000_adapter *adapter,
				      struct skb_shared_hwtstamps *hwtstamps,
				      u64 systim)
{
	u64 ns;
	unsigned long flags;

	spin_lock_irqsave(&adapter->systim_lock, flags);
	ns = timecounter_cyc2time(&adapter->tc, systim);
	spin_unlock_irqrestore(&adapter->systim_lock, flags);

	memset(hwtstamps, 0, sizeof(*hwtstamps));
	hwtstamps->hwtstamp = ns_to_ktime(ns);
}

static void e1000_print_hw_hang(struct work_struct *work)
{
	struct e1000_adapter *adapter = container_of(work,
						     struct e1000_adapter,
						     print_hang_task);
	struct net_device *netdev = adapter->netdev;
	struct e1000_ring *tx_ring = adapter->tx_ring;
	unsigned int i = tx_ring->next_to_clean;
	unsigned int eop = tx_ring->buffer_info[i].next_to_watch;
	struct e1000_tx_desc *eop_desc = E1000_TX_DESC(*tx_ring, eop);
	struct e1000_hw *hw = &adapter->hw;
	u16 phy_status, phy_1000t_status, phy_ext_status;
	u16 pci_status;

	if (test_bit(__E1000_DOWN, &adapter->state))
		return;

	if (!adapter->tx_hang_recheck && (adapter->flags2 & FLAG2_DMA_BURST)) {
		/* May be block on write-back, flush and detect again
		 * flush pending descriptor writebacks to memory
		 */
		ew32(TIDV, adapter->tx_int_delay | E1000_TIDV_FPD);
		/* execute the writes immediately */
		e1e_flush();
		/* Due to rare timing issues, write to TIDV again to ensure
		 * the write is successful
		 */
		ew32(TIDV, adapter->tx_int_delay | E1000_TIDV_FPD);
		/* execute the writes immediately */
		e1e_flush();
		adapter->tx_hang_recheck = true;
		return;
	}
	/* Real hang detected */
	adapter->tx_hang_recheck = false;
	netif_stop_queue(netdev);

	e1e_rphy(hw, MII_BMSR, &phy_status);
	e1e_rphy(hw, MII_STAT1000, &phy_1000t_status);
	e1e_rphy(hw, MII_ESTATUS, &phy_ext_status);

	pci_read_config_word(adapter->pdev, PCI_STATUS, &pci_status);

	/* detected Hardware unit hang */
	e_err("Detected Hardware Unit Hang:\n"
	      "  TDH                  <%x>\n"
	      "  TDT                  <%x>\n"
	      "  next_to_use          <%x>\n"
	      "  next_to_clean        <%x>\n"
	      "buffer_info[next_to_clean]:\n"
	      "  time_stamp           <%lx>\n"
	      "  next_to_watch        <%x>\n"
	      "  jiffies              <%lx>\n"
	      "  next_to_watch.status <%x>\n"
	      "MAC Status             <%x>\n"
	      "PHY Status             <%x>\n"
	      "PHY 1000BASE-T Status  <%x>\n"
	      "PHY Extended Status    <%x>\n"
	      "PCI Status             <%x>\n",
	      readl(tx_ring->head), readl(tx_ring->tail), tx_ring->next_to_use,
	      tx_ring->next_to_clean, tx_ring->buffer_info[eop].time_stamp,
	      eop, jiffies, eop_desc->upper.fields.status, er32(STATUS),
	      phy_status, phy_1000t_status, phy_ext_status, pci_status);

	/* Suggest workaround for known h/w issue */
	if ((hw->mac.type == e1000_pchlan) && (er32(CTRL) & E1000_CTRL_TFCE))
		e_err("Try turning off Tx pause (flow control) via ethtool\n");
}

/**
 * e1000e_rx_hwtstamp - utility function which checks for Rx time stamp
 * @adapter: board private structure
 * @status: descriptor extended error and status field
 * @skb: particular skb to include time stamp
 *
 * If the time stamp is valid, convert it into the timecounter ns value
 * and store that result into the shhwtstamps structure which is passed
 * up the network stack.
 **/
static void e1000e_rx_hwtstamp(struct e1000_adapter *adapter, u32 status,
			       struct sk_buff *skb)
{
	struct e1000_hw *hw = &adapter->hw;
	u64 rxstmp;

	if (!(adapter->flags & FLAG_HAS_HW_TIMESTAMP) ||
	    !(status & E1000_RXDEXT_STATERR_TST) ||
	    !(er32(TSYNCRXCTL) & E1000_TSYNCRXCTL_VALID))
		return;

	/* The Rx time stamp registers contain the time stamp.  No other
	 * received packet will be time stamped until the Rx time stamp
	 * registers are read.  Because only one packet can be time stamped
	 * at a time, the register values must belong to this packet and
	 * therefore none of the other additional attributes need to be
	 * compared.
	 */
	rxstmp = (u64)er32(RXSTMPL);
	rxstmp |= (u64)er32(RXSTMPH) << 32;
	e1000e_systim_to_hwtstamp(adapter, skb_hwtstamps(skb), rxstmp);

	adapter->flags2 &= ~FLAG2_CHECK_RX_HWTSTAMP;
}

/**
 * e1000e_tx_hwtstamp_work - check for Tx time stamp
 * @work: pointer to work struct
 *
 * This work function polls the TSYNCTXCTL valid bit to determine when a
 * timestamp has been taken for the current stored skb.  The timestamp must
 * be for this skb because only one such packet is allowed in the queue.
 */
static void e1000e_tx_hwtstamp_work(struct work_struct *work)
{
	struct e1000_adapter *adapter = container_of(work, struct e1000_adapter,
						     tx_hwtstamp_work);
	struct e1000_hw *hw = &adapter->hw;

	if (!adapter->tx_hwtstamp_skb)
		return;

	if (er32(TSYNCTXCTL) & E1000_TSYNCTXCTL_VALID) {
		struct skb_shared_hwtstamps shhwtstamps;
		u64 txstmp;

		txstmp = er32(TXSTMPL);
		txstmp |= (u64)er32(TXSTMPH) << 32;

		e1000e_systim_to_hwtstamp(adapter, &shhwtstamps, txstmp);

		skb_tstamp_tx(adapter->tx_hwtstamp_skb, &shhwtstamps);
		dev_kfree_skb_any(adapter->tx_hwtstamp_skb);
		adapter->tx_hwtstamp_skb = NULL;
	} else {
		/* reschedule to check later */
		schedule_work(&adapter->tx_hwtstamp_work);
	}
}

/**
 * e1000e_get_base_timinca - get default SYSTIM time increment attributes
 * @adapter: board private structure
 * @timinca: pointer to returned time increment attributes
 *
 * Get attributes for incrementing the System Time Register SYSTIML/H at
 * the default base frequency, and set the cyclecounter shift value.
 **/
s32 e1000e_get_base_timinca(struct e1000_adapter *adapter, u32 *timinca)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 incvalue, incperiod, shift;

	/* Make sure clock is enabled on I217 before checking the frequency */
	if ((hw->mac.type == e1000_pch_lpt) &&
	    !(er32(TSYNCTXCTL) & E1000_TSYNCTXCTL_ENABLED) &&
	    !(er32(TSYNCRXCTL) & E1000_TSYNCRXCTL_ENABLED)) {
		u32 fextnvm7 = er32(FEXTNVM7);

		if (!(fextnvm7 & (1 << 0))) {
			ew32(FEXTNVM7, fextnvm7 | (1 << 0));
			e1e_flush();
		}
	}

	switch (hw->mac.type) {
	case e1000_pch2lan:
	case e1000_pch_lpt:
		/* On I217, the clock frequency is 25MHz or 96MHz as
		 * indicated by the System Clock Frequency Indication
		 */
		if ((hw->mac.type != e1000_pch_lpt) ||
		    (er32(TSYNCRXCTL) & E1000_TSYNCRXCTL_SYSCFI)) {
			/* Stable 96MHz frequency */
			incperiod = INCPERIOD_96MHz;
			incvalue = INCVALUE_96MHz;
			shift = INCVALUE_SHIFT_96MHz;
			adapter->cc.shift = shift + INCPERIOD_SHIFT_96MHz;
			break;
		}
		/* fall-through */
		break;
	default:
		return -EINVAL;
	}

	*timinca = ((incperiod << E1000_TIMINCA_INCPERIOD_SHIFT) |
		    ((incvalue << shift) & E1000_TIMINCA_INCVALUE_MASK));

	return 0;
}

/**
 * e1000e_config_hwtstamp - configure the hwtstamp registers and enable/disable
 * @adapter: board private structure
 *
 * Outgoing time stamping can be enabled and disabled. Play nice and
 * disable it when requested, although it shouldn't cause any overhead
 * when no packet needs it. At most one packet in the queue may be
 * marked for time stamping, otherwise it would be impossible to tell
 * for sure to which packet the hardware time stamp belongs.
 *
 * Incoming time stamping has to be configured via the hardware filters.
 * Not all combinations are supported, in particular event type has to be
 * specified. Matching the kind of event packet is not supported, with the
 * exception of "all V2 events regardless of level 2 or 4".
 **/
static int e1000e_config_hwtstamp(struct e1000_adapter *adapter,
				  struct hwtstamp_config *config)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 tsync_tx_ctl = E1000_TSYNCTXCTL_ENABLED;
	u32 tsync_rx_ctl = E1000_TSYNCRXCTL_ENABLED;
	u32 rxmtrl = 0;
	u16 rxudp = 0;
	bool is_l4 = false;
	bool is_l2 = false;
	u32 regval;
	s32 ret_val;

	if (!(adapter->flags & FLAG_HAS_HW_TIMESTAMP))
		return -EINVAL;

	/* flags reserved for future extensions - must be zero */
	if (config->flags)
		return -EINVAL;

	switch (config->tx_type) {
	case HWTSTAMP_TX_OFF:
		tsync_tx_ctl = 0;
		break;
	case HWTSTAMP_TX_ON:
		break;
	default:
		return -ERANGE;
	}

	switch (config->rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		tsync_rx_ctl = 0;
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L4_V1;
		rxmtrl = E1000_RXMTRL_PTP_V1_SYNC_MESSAGE;
		is_l4 = true;
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L4_V1;
		rxmtrl = E1000_RXMTRL_PTP_V1_DELAY_REQ_MESSAGE;
		is_l4 = true;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
		/* Also time stamps V2 L2 Path Delay Request/Response */
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L2_V2;
		rxmtrl = E1000_RXMTRL_PTP_V2_SYNC_MESSAGE;
		is_l2 = true;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
		/* Also time stamps V2 L2 Path Delay Request/Response. */
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L2_V2;
		rxmtrl = E1000_RXMTRL_PTP_V2_DELAY_REQ_MESSAGE;
		is_l2 = true;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
		/* Hardware cannot filter just V2 L4 Sync messages;
		 * fall-through to V2 (both L2 and L4) Sync.
		 */
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
		/* Also time stamps V2 Path Delay Request/Response. */
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L2_L4_V2;
		rxmtrl = E1000_RXMTRL_PTP_V2_SYNC_MESSAGE;
		is_l2 = true;
		is_l4 = true;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		/* Hardware cannot filter just V2 L4 Delay Request messages;
		 * fall-through to V2 (both L2 and L4) Delay Request.
		 */
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		/* Also time stamps V2 Path Delay Request/Response. */
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_L2_L4_V2;
		rxmtrl = E1000_RXMTRL_PTP_V2_DELAY_REQ_MESSAGE;
		is_l2 = true;
		is_l4 = true;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
		/* Hardware cannot filter just V2 L4 or L2 Event messages;
		 * fall-through to all V2 (both L2 and L4) Events.
		 */
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_EVENT_V2;
		config->rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		is_l2 = true;
		is_l4 = true;
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
		/* For V1, the hardware can only filter Sync messages or
		 * Delay Request messages but not both so fall-through to
		 * time stamp all packets.
		 */
	case HWTSTAMP_FILTER_ALL:
		is_l2 = true;
		is_l4 = true;
		tsync_rx_ctl |= E1000_TSYNCRXCTL_TYPE_ALL;
		config->rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	default:
		return -ERANGE;
	}

	adapter->hwtstamp_config = *config;

	/* enable/disable Tx h/w time stamping */
	regval = er32(TSYNCTXCTL);
	regval &= ~E1000_TSYNCTXCTL_ENABLED;
	regval |= tsync_tx_ctl;
	ew32(TSYNCTXCTL, regval);
	if ((er32(TSYNCTXCTL) & E1000_TSYNCTXCTL_ENABLED) !=
	    (regval & E1000_TSYNCTXCTL_ENABLED)) {
		e_err("Timesync Tx Control register not set as expected\n");
		return -EAGAIN;
	}

	/* enable/disable Rx h/w time stamping */
	regval = er32(TSYNCRXCTL);
	regval &= ~(E1000_TSYNCRXCTL_ENABLED | E1000_TSYNCRXCTL_TYPE_MASK);
	regval |= tsync_rx_ctl;
	ew32(TSYNCRXCTL, regval);
	if ((er32(TSYNCRXCTL) & (E1000_TSYNCRXCTL_ENABLED |
				 E1000_TSYNCRXCTL_TYPE_MASK)) !=
	    (regval & (E1000_TSYNCRXCTL_ENABLED |
		       E1000_TSYNCRXCTL_TYPE_MASK))) {
		e_err("Timesync Rx Control register not set as expected\n");
		return -EAGAIN;
	}

	/* L2: define ethertype filter for time stamped packets */
	if (is_l2)
		rxmtrl |= ETH_P_1588;

	/* define which PTP packets get time stamped */
	ew32(RXMTRL, rxmtrl);

	/* Filter by destination port */
	if (is_l4) {
		rxudp = PTP_EV_PORT;
		cpu_to_be16s(&rxudp);
	}
	ew32(RXUDP, rxudp);

	e1e_flush();

	/* Clear TSYNCRXCTL_VALID & TSYNCTXCTL_VALID bit */
	er32(RXSTMPH);
	er32(TXSTMPH);

	/* Get and set the System Time Register SYSTIM base frequency */
	ret_val = e1000e_get_base_timinca(adapter, &regval);
	if (ret_val)
		return ret_val;
	ew32(TIMINCA, regval);

	/* reset the ns time counter */
	timecounter_init(&adapter->tc, &adapter->cc,
			 ktime_to_ns(ktime_get_real()));

	return 0;
}

/**
 * e1000_configure - configure the hardware for Rx and Tx
 * @adapter: private board structure
 **/
static void e1000_configure(struct e1000_adapter *adapter)
{
	struct e1000_ring *rx_ring = adapter->rx_ring;

	e1000e_set_rx_mode(adapter->netdev);

	e1000_restore_vlan(adapter);
	e1000_init_manageability_pt(adapter);

	e1000_configure_tx(adapter);

	if (adapter->netdev->features & NETIF_F_RXHASH)
		e1000e_setup_rss_hash(adapter);
	e1000_setup_rctl(adapter);
	e1000_configure_rx(adapter);
	adapter->alloc_rx_buf(rx_ring, e1000_desc_unused(rx_ring), GFP_KERNEL);
}

/**
 * e1000e_power_up_phy - restore link in case the phy was powered down
 * @adapter: address of board private structure
 *
 * The phy may be powered down to save power and turn off link when the
 * driver is unloaded and wake on lan is not enabled (among others)
 * *** this routine MUST be followed by a call to e1000e_reset ***
 **/
void e1000e_power_up_phy(struct e1000_adapter *adapter)
{
	if (adapter->hw.phy.ops.power_up)
		adapter->hw.phy.ops.power_up(&adapter->hw);

	adapter->hw.mac.ops.setup_link(&adapter->hw);
}

/**
 * e1000_power_down_phy - Power down the PHY
 *
 * Power down the PHY so no link is implied when interface is down.
 * The PHY cannot be powered down if management or WoL is active.
 */
static void e1000_power_down_phy(struct e1000_adapter *adapter)
{
	/* WoL is enabled */
	if (adapter->wol)
		return;

	if (adapter->hw.phy.ops.power_down)
		adapter->hw.phy.ops.power_down(&adapter->hw);
}

/**
 * e1000e_reset - bring the hardware into a known good state
 *
 * This function boots the hardware and enables some settings that
 * require a configuration cycle of the hardware - those cannot be
 * set/changed during runtime. After reset the device needs to be
 * properly configured for Rx, Tx etc.
 */
void e1000e_reset(struct e1000_adapter *adapter)
{
	struct e1000_mac_info *mac = &adapter->hw.mac;
	struct e1000_fc_info *fc = &adapter->hw.fc;
	struct e1000_hw *hw = &adapter->hw;
	u32 tx_space, min_tx_space, min_rx_space;
	u32 pba = adapter->pba;
	u16 hwm;

	/* reset Packet Buffer Allocation to default */
	ew32(PBA, pba);

	if (adapter->max_frame_size > ETH_FRAME_LEN + ETH_FCS_LEN) {
		/* To maintain wire speed transmits, the Tx FIFO should be
		 * large enough to accommodate two full transmit packets,
		 * rounded up to the next 1KB and expressed in KB.  Likewise,
		 * the Rx FIFO should be large enough to accommodate at least
		 * one full receive packet and is similarly rounded up and
		 * expressed in KB.
		 */
		pba = er32(PBA);
		/* upper 16 bits has Tx packet buffer allocation size in KB */
		tx_space = pba >> 16;
		/* lower 16 bits has Rx packet buffer allocation size in KB */
		pba &= 0xffff;
		/* the Tx fifo also stores 16 bytes of information about the Tx
		 * but don't include ethernet FCS because hardware appends it
		 */
		min_tx_space = (adapter->max_frame_size +
				sizeof(struct e1000_tx_desc) - ETH_FCS_LEN) * 2;
		min_tx_space = ALIGN(min_tx_space, 1024);
		min_tx_space >>= 10;
		/* software strips receive CRC, so leave room for it */
		min_rx_space = adapter->max_frame_size;
		min_rx_space = ALIGN(min_rx_space, 1024);
		min_rx_space >>= 10;

		/* If current Tx allocation is less than the min Tx FIFO size,
		 * and the min Tx FIFO size is less than the current Rx FIFO
		 * allocation, take space away from current Rx allocation
		 */
		if ((tx_space < min_tx_space) &&
		    ((min_tx_space - tx_space) < pba)) {
			pba -= min_tx_space - tx_space;

			/* if short on Rx space, Rx wins and must trump Tx
			 * adjustment
			 */
			if (pba < min_rx_space)
				pba = min_rx_space;
		}

		ew32(PBA, pba);
	}

	/* flow control settings
	 *
	 * The high water mark must be low enough to fit one full frame
	 * (or the size used for early receive) above it in the Rx FIFO.
	 * Set it to the lower of:
	 * - 90% of the Rx FIFO size, and
	 * - the full Rx FIFO size minus one full frame
	 */
	if (adapter->flags & FLAG_DISABLE_FC_PAUSE_TIME)
		fc->pause_time = 0xFFFF;
	else
		fc->pause_time = E1000_FC_PAUSE_TIME;
	fc->send_xon = true;
	fc->current_mode = fc->requested_mode;

	switch (hw->mac.type) {
	case e1000_ich9lan:
	case e1000_ich10lan:
		if (adapter->netdev->mtu > ETH_DATA_LEN) {
			pba = 14;
			ew32(PBA, pba);
			fc->high_water = 0x2800;
			fc->low_water = fc->high_water - 8;
			break;
		}
		/* fall-through */
	default:
		hwm = min(((pba << 10) * 9 / 10),
			  ((pba << 10) - adapter->max_frame_size));

		fc->high_water = hwm & E1000_FCRTH_RTH;	/* 8-byte granularity */
		fc->low_water = fc->high_water - 8;
		break;
	case e1000_pchlan:
		/* Workaround PCH LOM adapter hangs with certain network
		 * loads.  If hangs persist, try disabling Tx flow control.
		 */
		if (adapter->netdev->mtu > ETH_DATA_LEN) {
			fc->high_water = 0x3500;
			fc->low_water = 0x1500;
		} else {
			fc->high_water = 0x5000;
			fc->low_water = 0x3000;
		}
		fc->refresh_time = 0x1000;
		break;
	case e1000_pch2lan:
	case e1000_pch_lpt:
		fc->refresh_time = 0x0400;

		if (adapter->netdev->mtu <= ETH_DATA_LEN) {
			fc->high_water = 0x05C20;
			fc->low_water = 0x05048;
			fc->pause_time = 0x0650;
			break;
		}

		pba = 14;
		ew32(PBA, pba);
		fc->high_water = ((pba << 10) * 9 / 10) & E1000_FCRTH_RTH;
		fc->low_water = ((pba << 10) * 8 / 10) & E1000_FCRTL_RTL;
		break;
	}

	/* Alignment of Tx data is on an arbitrary byte boundary with the
	 * maximum size per Tx descriptor limited only to the transmit
	 * allocation of the packet buffer minus 96 bytes with an upper
	 * limit of 24KB due to receive synchronization limitations.
	 */
	adapter->tx_fifo_limit = min_t(u32, ((er32(PBA) >> 16) << 10) - 96,
				       24 << 10);

	/* Disable Adaptive Interrupt Moderation if 2 full packets cannot
	 * fit in receive buffer.
	 */
	if (adapter->itr_setting & 0x3) {
		if ((adapter->max_frame_size * 2) > (pba << 10)) {
			if (!(adapter->flags2 & FLAG2_DISABLE_AIM)) {
				dev_info(&adapter->pdev->dev,
					 "Interrupt Throttle Rate off\n");
				adapter->flags2 |= FLAG2_DISABLE_AIM;
				e1000e_write_itr(adapter, 0);
			}
		} else if (adapter->flags2 & FLAG2_DISABLE_AIM) {
			dev_info(&adapter->pdev->dev,
				 "Interrupt Throttle Rate on\n");
			adapter->flags2 &= ~FLAG2_DISABLE_AIM;
			adapter->itr = 20000;
			e1000e_write_itr(adapter, adapter->itr);
		}
	}

	/* Allow time for pending master requests to run */
	mac->ops.reset_hw(hw);

	/* For parts with AMT enabled, let the firmware know
	 * that the network interface is in control
	 */
	if (adapter->flags & FLAG_HAS_AMT)
		e1000e_get_hw_control(adapter);

	ew32(WUC, 0);

	if (mac->ops.init_hw(hw))
		e_err("Hardware Error\n");

	e1000_update_mng_vlan(adapter);

	/* Enable h/w to recognize an 802.1Q VLAN Ethernet packet */
	ew32(VET, ETH_P_8021Q);

	e1000e_reset_adaptive(hw);

	/* initialize systim and reset the ns time counter */
	e1000e_config_hwtstamp(adapter, &adapter->hwtstamp_config);

	/* Set EEE advertisement as appropriate */
	if (adapter->flags2 & FLAG2_HAS_EEE) {
		s32 ret_val;
		u16 adv_addr;

		switch (hw->phy.type) {
		case e1000_phy_82579:
			adv_addr = I82579_EEE_ADVERTISEMENT;
			break;
		case e1000_phy_i217:
			adv_addr = I217_EEE_ADVERTISEMENT;
			break;
		default:
			dev_err(&adapter->pdev->dev,
				"Invalid PHY type setting EEE advertisement\n");
			return;
		}

		ret_val = hw->phy.ops.acquire(hw);
		if (ret_val) {
			dev_err(&adapter->pdev->dev,
				"EEE advertisement - unable to acquire PHY\n");
			return;
		}

		e1000_write_emi_reg_locked(hw, adv_addr,
					   hw->dev_spec.ich8lan.eee_disable ?
					   0 : adapter->eee_advert);

		hw->phy.ops.release(hw);
	}

	if (!netif_running(adapter->netdev) &&
	    !test_bit(__E1000_TESTING, &adapter->state)) {
		e1000_power_down_phy(adapter);
		return;
	}

	e1000_get_phy_info(hw);

	if ((adapter->flags & FLAG_HAS_SMART_POWER_DOWN) &&
	    !(adapter->flags & FLAG_SMART_POWER_DOWN)) {
		u16 phy_data = 0;
		/* speed up time to link by disabling smart power down, ignore
		 * the return value of this function because there is nothing
		 * different we would do if it failed
		 */
		e1e_rphy(hw, IGP02E1000_PHY_POWER_MGMT, &phy_data);
		phy_data &= ~IGP02E1000_PM_SPD;
		e1e_wphy(hw, IGP02E1000_PHY_POWER_MGMT, phy_data);
	}
}

int e1000e_up(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;

	/* hardware has been reset, we need to reload some things */
	e1000_configure(adapter);

	clear_bit(__E1000_DOWN, &adapter->state);

	if (adapter->msix_entries)
		e1000_configure_msix(adapter);
	e1000_irq_enable(adapter);

	netif_start_queue(adapter->netdev);

	/* fire a link change interrupt to start the watchdog */
	if (adapter->msix_entries)
		ew32(ICS, E1000_ICS_LSC | E1000_ICR_OTHER);
	else
		ew32(ICS, E1000_ICS_LSC);

	return 0;
}

static void e1000e_flush_descriptors(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;

	if (!(adapter->flags2 & FLAG2_DMA_BURST))
		return;

	/* flush pending descriptor writebacks to memory */
	ew32(TIDV, adapter->tx_int_delay | E1000_TIDV_FPD);
	ew32(RDTR, adapter->rx_int_delay | E1000_RDTR_FPD);

	/* execute the writes immediately */
	e1e_flush();

	/* due to rare timing issues, write to TIDV/RDTR again to ensure the
	 * write is successful
	 */
	ew32(TIDV, adapter->tx_int_delay | E1000_TIDV_FPD);
	ew32(RDTR, adapter->rx_int_delay | E1000_RDTR_FPD);

	/* execute the writes immediately */
	e1e_flush();
}

static void e1000e_update_stats(struct e1000_adapter *adapter);

void e1000e_down(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	struct e1000_hw *hw = &adapter->hw;
	u32 tctl, rctl;

	/* signal that we're down so the interrupt handler does not
	 * reschedule our watchdog timer
	 */
	set_bit(__E1000_DOWN, &adapter->state);

	/* disable receives in the hardware */
	rctl = er32(RCTL);
	if (!(adapter->flags2 & FLAG2_NO_DISABLE_RX))
		ew32(RCTL, rctl & ~E1000_RCTL_EN);
	/* flush and sleep below */

	netif_stop_queue(netdev);

	/* disable transmits in the hardware */
	tctl = er32(TCTL);
	tctl &= ~E1000_TCTL_EN;
	ew32(TCTL, tctl);

	/* flush both disables and wait for them to finish */
	e1e_flush();
	usleep_range(10000, 20000);

	e1000_irq_disable(adapter);

	napi_synchronize(&adapter->napi);

	del_timer_sync(&adapter->watchdog_timer);
	del_timer_sync(&adapter->phy_info_timer);

	netif_carrier_off(netdev);

	spin_lock(&adapter->stats64_lock);
	e1000e_update_stats(adapter);
	spin_unlock(&adapter->stats64_lock);

	e1000e_flush_descriptors(adapter);
	e1000_clean_tx_ring(adapter->tx_ring);
	e1000_clean_rx_ring(adapter->rx_ring);

	adapter->link_speed = 0;
	adapter->link_duplex = 0;

	/* Disable Si errata workaround on PCHx for jumbo frame flow */
	if ((hw->mac.type >= e1000_pch2lan) &&
	    (adapter->netdev->mtu > ETH_DATA_LEN) &&
	    e1000_lv_jumbo_workaround_ich8lan(hw, false))
		e_dbg("failed to disable jumbo frame workaround mode\n");

	if (!pci_channel_offline(adapter->pdev))
		e1000e_reset(adapter);

	/* TODO: for power management, we could drop the link and
	 * pci_disable_device here.
	 */
}

void e1000e_reinit_locked(struct e1000_adapter *adapter)
{
	might_sleep();
	while (test_and_set_bit(__E1000_RESETTING, &adapter->state))
		usleep_range(1000, 2000);
	e1000e_down(adapter);
	e1000e_up(adapter);
	clear_bit(__E1000_RESETTING, &adapter->state);
}

/**
 * e1000e_cyclecounter_read - read raw cycle counter (used by time counter)
 * @cc: cyclecounter structure
 **/
static cycle_t e1000e_cyclecounter_read(const struct cyclecounter *cc)
{
	struct e1000_adapter *adapter = container_of(cc, struct e1000_adapter,
						     cc);
	struct e1000_hw *hw = &adapter->hw;
	cycle_t systim;

	/* latch SYSTIMH on read of SYSTIML */
	systim = (cycle_t)er32(SYSTIML);
	systim |= (cycle_t)er32(SYSTIMH) << 32;

	return systim;
}

/**
 * e1000_sw_init - Initialize general software structures (struct e1000_adapter)
 * @adapter: board private structure to initialize
 *
 * e1000_sw_init initializes the Adapter private data structure.
 * Fields are initialized based on PCI device information and
 * OS network device settings (MTU size).
 **/
static int e1000_sw_init(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;

	adapter->rx_buffer_len = ETH_FRAME_LEN + VLAN_HLEN + ETH_FCS_LEN;
	adapter->rx_ps_bsize0 = 128;
	adapter->max_frame_size = netdev->mtu + ETH_HLEN + ETH_FCS_LEN;
	adapter->min_frame_size = ETH_ZLEN + ETH_FCS_LEN;
	adapter->tx_ring_count = E1000_DEFAULT_TXD;
	adapter->rx_ring_count = E1000_DEFAULT_RXD;

	spin_lock_init(&adapter->stats64_lock);

	e1000e_set_interrupt_capability(adapter);

	if (e1000_alloc_queues(adapter))
		return -ENOMEM;

	/* Setup hardware time stamping cyclecounter */
	if (adapter->flags & FLAG_HAS_HW_TIMESTAMP) {
		adapter->cc.read = e1000e_cyclecounter_read;
		adapter->cc.mask = CLOCKSOURCE_MASK(64);
		adapter->cc.mult = 1;
		/* cc.shift set in e1000e_get_base_tininca() */

		spin_lock_init(&adapter->systim_lock);
		INIT_WORK(&adapter->tx_hwtstamp_work, e1000e_tx_hwtstamp_work);
	}

	/* Explicitly disable IRQ since the NIC can be in any state. */
	e1000_irq_disable(adapter);

	set_bit(__E1000_DOWN, &adapter->state);
	return 0;
}

/**
 * e1000_open - Called when a network interface is made active
 * @netdev: network interface device structure
 *
 * Returns 0 on success, negative value on failure
 *
 * The open entry point is called when a network interface is made
 * active by the system (IFF_UP).  At this point all resources needed
 * for transmit and receive operations are allocated, the interrupt
 * handler is registered with the OS, the watchdog timer is started,
 * and the stack is notified that the interface is ready.
 **/
static int e1000_open(struct net_device *netdev)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	struct pci_dev *pdev = adapter->pdev;
	int err;

	/* disallow open during test */
	if (test_bit(__E1000_TESTING, &adapter->state))
		return -EBUSY;

	pm_runtime_get_sync(&pdev->dev);

	netif_carrier_off(netdev);

	/* allocate transmit descriptors */
	err = e1000e_setup_tx_resources(adapter->tx_ring);
	if (err)
		goto err_setup_tx;

	/* allocate receive descriptors */
	err = e1000e_setup_rx_resources(adapter->rx_ring);
	if (err)
		goto err_setup_rx;

	/* If AMT is enabled, let the firmware know that the network
	 * interface is now open and reset the part to a known state.
	 */
	if (adapter->flags & FLAG_HAS_AMT) {
		e1000e_get_hw_control(adapter);
		e1000e_reset(adapter);
	}

	e1000e_power_up_phy(adapter);

	adapter->mng_vlan_id = E1000_MNG_VLAN_NONE;
	if ((adapter->hw.mng_cookie.status & E1000_MNG_DHCP_COOKIE_STATUS_VLAN))
		e1000_update_mng_vlan(adapter);

	/* DMA latency requirement to workaround jumbo issue */
	pm_qos_add_request(&adapter->netdev->pm_qos_req, PM_QOS_CPU_DMA_LATENCY,
			   PM_QOS_DEFAULT_VALUE);

	/* before we allocate an interrupt, we must be ready to handle it.
	 * Setting DEBUG_SHIRQ in the kernel makes it fire an interrupt
	 * as soon as we call pci_request_irq, so we have to setup our
	 * clean_rx handler before we do so.
	 */
	e1000_configure(adapter);

	err = e1000_request_irq(adapter);
	if (err)
		goto err_req_irq;

	/* Work around PCIe errata with MSI interrupts causing some chipsets to
	 * ignore e1000e MSI messages, which means we need to test our MSI
	 * interrupt now
	 */
	if (adapter->int_mode != E1000E_INT_MODE_LEGACY) {
		err = e1000_test_msi(adapter);
		if (err) {
			e_err("Interrupt allocation failed\n");
			goto err_req_irq;
		}
	}

	/* From here on the code is the same as e1000e_up() */
	clear_bit(__E1000_DOWN, &adapter->state);

	napi_enable(&adapter->napi);

	e1000_irq_enable(adapter);

	adapter->tx_hang_recheck = false;
	netif_start_queue(netdev);

	adapter->idle_check = true;
	hw->mac.get_link_status = true;
	pm_runtime_put(&pdev->dev);

	/* fire a link status change interrupt to start the watchdog */
	if (adapter->msix_entries)
		ew32(ICS, E1000_ICS_LSC | E1000_ICR_OTHER);
	else
		ew32(ICS, E1000_ICS_LSC);

	return 0;

err_req_irq:
	e1000e_release_hw_control(adapter);
	e1000_power_down_phy(adapter);
	e1000e_free_rx_resources(adapter->rx_ring);
err_setup_rx:
	e1000e_free_tx_resources(adapter->tx_ring);
err_setup_tx:
	e1000e_reset(adapter);
	pm_runtime_put_sync(&pdev->dev);

	return err;
}

/**
 * e1000_close - Disables a network interface
 * @netdev: network interface device structure
 *
 * Returns 0, this is not allowed to fail
 *
 * The close entry point is called when an interface is de-activated
 * by the OS.  The hardware is still under the drivers control, but
 * needs to be disabled.  A global MAC reset is issued to stop the
 * hardware, and all transmit and receive resources are freed.
 **/
static int e1000_close(struct net_device *netdev)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct pci_dev *pdev = adapter->pdev;
	int count = E1000_CHECK_RESET_COUNT;

	while (test_bit(__E1000_RESETTING, &adapter->state) && count--)
		usleep_range(10000, 20000);

	WARN_ON(test_bit(__E1000_RESETTING, &adapter->state));

	pm_runtime_get_sync(&pdev->dev);

	if (!test_bit(__E1000_DOWN, &adapter->state)) {
		e1000e_down(adapter);
		e1000_free_irq(adapter);
	}

	napi_disable(&adapter->napi);

	e1000_power_down_phy(adapter);

	e1000e_free_tx_resources(adapter->tx_ring);
	e1000e_free_rx_resources(adapter->rx_ring);

	/* kill manageability vlan ID if supported, but not if a vlan with
	 * the same ID is registered on the host OS (let 8021q kill it)
	 */
	if (adapter->hw.mng_cookie.status & E1000_MNG_DHCP_COOKIE_STATUS_VLAN)
		e1000_vlan_rx_kill_vid(netdev, htons(ETH_P_8021Q),
				       adapter->mng_vlan_id);

	/* If AMT is enabled, let the firmware know that the network
	 * interface is now closed
	 */
	if ((adapter->flags & FLAG_HAS_AMT) &&
	    !test_bit(__E1000_TESTING, &adapter->state))
		e1000e_release_hw_control(adapter);

	pm_qos_remove_request(&adapter->netdev->pm_qos_req);

	pm_runtime_put_sync(&pdev->dev);

	return 0;
}

/**
 * e1000_set_mac - Change the Ethernet Address of the NIC
 * @netdev: network interface device structure
 * @p: pointer to an address structure
 *
 * Returns 0 on success, negative on failure
 **/
static int e1000_set_mac(struct net_device *netdev, void *p)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);
	memcpy(adapter->hw.mac.addr, addr->sa_data, netdev->addr_len);

	hw->mac.ops.rar_set(&adapter->hw, adapter->hw.mac.addr, 0);

	return 0;
}

/**
 * e1000e_update_phy_task - work thread to update phy
 * @work: pointer to our work struct
 *
 * this worker thread exists because we must acquire a
 * semaphore to read the phy, which we could msleep while
 * waiting for it, and we can't msleep in a timer.
 **/
static void e1000e_update_phy_task(struct work_struct *work)
{
	struct e1000_adapter *adapter = container_of(work,
						     struct e1000_adapter,
						     update_phy_task);

	if (test_bit(__E1000_DOWN, &adapter->state))
		return;

	e1000_get_phy_info(&adapter->hw);
}

/**
 * e1000_update_phy_info - timre call-back to update PHY info
 * @data: pointer to adapter cast into an unsigned long
 *
 * Need to wait a few seconds after link up to get diagnostic information from
 * the phy
 **/
static void e1000_update_phy_info(unsigned long data)
{
	struct e1000_adapter *adapter = (struct e1000_adapter *)data;

	if (test_bit(__E1000_DOWN, &adapter->state))
		return;

	schedule_work(&adapter->update_phy_task);
}

/**
 * e1000e_update_phy_stats - Update the PHY statistics counters
 * @adapter: board private structure
 *
 * Read/clear the upper 16-bit PHY registers and read/accumulate lower
 **/
static void e1000e_update_phy_stats(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	s32 ret_val;
	u16 phy_data;

	ret_val = hw->phy.ops.acquire(hw);
	if (ret_val)
		return;

	/* A page set is expensive so check if already on desired page.
	 * If not, set to the page with the PHY status registers.
	 */
	hw->phy.addr = 1;
	ret_val = e1000e_read_phy_reg_mdic(hw, IGP01E1000_PHY_PAGE_SELECT,
					   &phy_data);
	if (ret_val)
		goto release;
	if (phy_data != (HV_STATS_PAGE << IGP_PAGE_SHIFT)) {
		ret_val = hw->phy.ops.set_page(hw,
					       HV_STATS_PAGE << IGP_PAGE_SHIFT);
		if (ret_val)
			goto release;
	}

	/* Single Collision Count */
	hw->phy.ops.read_reg_page(hw, HV_SCC_UPPER, &phy_data);
	ret_val = hw->phy.ops.read_reg_page(hw, HV_SCC_LOWER, &phy_data);
	if (!ret_val)
		adapter->stats.scc += phy_data;

	/* Excessive Collision Count */
	hw->phy.ops.read_reg_page(hw, HV_ECOL_UPPER, &phy_data);
	ret_val = hw->phy.ops.read_reg_page(hw, HV_ECOL_LOWER, &phy_data);
	if (!ret_val)
		adapter->stats.ecol += phy_data;

	/* Multiple Collision Count */
	hw->phy.ops.read_reg_page(hw, HV_MCC_UPPER, &phy_data);
	ret_val = hw->phy.ops.read_reg_page(hw, HV_MCC_LOWER, &phy_data);
	if (!ret_val)
		adapter->stats.mcc += phy_data;

	/* Late Collision Count */
	hw->phy.ops.read_reg_page(hw, HV_LATECOL_UPPER, &phy_data);
	ret_val = hw->phy.ops.read_reg_page(hw, HV_LATECOL_LOWER, &phy_data);
	if (!ret_val)
		adapter->stats.latecol += phy_data;

	/* Collision Count - also used for adaptive IFS */
	hw->phy.ops.read_reg_page(hw, HV_COLC_UPPER, &phy_data);
	ret_val = hw->phy.ops.read_reg_page(hw, HV_COLC_LOWER, &phy_data);
	if (!ret_val)
		hw->mac.collision_delta = phy_data;

	/* Defer Count */
	hw->phy.ops.read_reg_page(hw, HV_DC_UPPER, &phy_data);
	ret_val = hw->phy.ops.read_reg_page(hw, HV_DC_LOWER, &phy_data);
	if (!ret_val)
		adapter->stats.dc += phy_data;

	/* Transmit with no CRS */
	hw->phy.ops.read_reg_page(hw, HV_TNCRS_UPPER, &phy_data);
	ret_val = hw->phy.ops.read_reg_page(hw, HV_TNCRS_LOWER, &phy_data);
	if (!ret_val)
		adapter->stats.tncrs += phy_data;

release:
	hw->phy.ops.release(hw);
}

/**
 * e1000e_update_stats - Update the board statistics counters
 * @adapter: board private structure
 **/
static void e1000e_update_stats(struct e1000_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	struct e1000_hw *hw = &adapter->hw;
	struct pci_dev *pdev = adapter->pdev;

	/* Prevent stats update while adapter is being reset, or if the pci
	 * connection is down.
	 */
	if (adapter->link_speed == 0)
		return;
	if (pci_channel_offline(pdev))
		return;

	adapter->stats.crcerrs += er32(CRCERRS);
	adapter->stats.gprc += er32(GPRC);
	adapter->stats.gorc += er32(GORCL);
	er32(GORCH);		/* Clear gorc */
	adapter->stats.bprc += er32(BPRC);
	adapter->stats.mprc += er32(MPRC);
	adapter->stats.roc += er32(ROC);

	adapter->stats.mpc += er32(MPC);

	/* Half-duplex statistics */
	if (adapter->link_duplex == HALF_DUPLEX) {
		if (adapter->flags2 & FLAG2_HAS_PHY_STATS) {
			e1000e_update_phy_stats(adapter);
		} else {
			adapter->stats.scc += er32(SCC);
			adapter->stats.ecol += er32(ECOL);
			adapter->stats.mcc += er32(MCC);
			adapter->stats.latecol += er32(LATECOL);
			adapter->stats.dc += er32(DC);

			hw->mac.collision_delta = er32(COLC);
		}
		adapter->stats.colc += hw->mac.collision_delta;
	}

	adapter->stats.xonrxc += er32(XONRXC);
	adapter->stats.xontxc += er32(XONTXC);
	adapter->stats.xoffrxc += er32(XOFFRXC);
	adapter->stats.xofftxc += er32(XOFFTXC);
	adapter->stats.gptc += er32(GPTC);
	adapter->stats.gotc += er32(GOTCL);
	er32(GOTCH);		/* Clear gotc */
	adapter->stats.rnbc += er32(RNBC);
	adapter->stats.ruc += er32(RUC);

	adapter->stats.mptc += er32(MPTC);
	adapter->stats.bptc += er32(BPTC);

	/* used for adaptive IFS */

	hw->mac.tx_packet_delta = er32(TPT);
	adapter->stats.tpt += hw->mac.tx_packet_delta;

	adapter->stats.algnerrc += er32(ALGNERRC);
	adapter->stats.rxerrc += er32(RXERRC);
	adapter->stats.cexterr += er32(CEXTERR);
	adapter->stats.tsctc += er32(TSCTC);
	adapter->stats.tsctfc += er32(TSCTFC);

	/* Fill out the OS statistics structure */
	netdev->stats.multicast = adapter->stats.mprc;
	netdev->stats.collisions = adapter->stats.colc;

	/* Rx Errors */

	/* RLEC on some newer hardware can be incorrect so build
	 * our own version based on RUC and ROC
	 */
	netdev->stats.rx_errors = adapter->stats.rxerrc +
	    adapter->stats.crcerrs + adapter->stats.algnerrc +
	    adapter->stats.ruc + adapter->stats.roc + adapter->stats.cexterr;
	netdev->stats.rx_length_errors = adapter->stats.ruc +
	    adapter->stats.roc;
	netdev->stats.rx_crc_errors = adapter->stats.crcerrs;
	netdev->stats.rx_frame_errors = adapter->stats.algnerrc;
	netdev->stats.rx_missed_errors = adapter->stats.mpc;

	/* Tx Errors */
	netdev->stats.tx_errors = adapter->stats.ecol + adapter->stats.latecol;
	netdev->stats.tx_aborted_errors = adapter->stats.ecol;
	netdev->stats.tx_window_errors = adapter->stats.latecol;
	netdev->stats.tx_carrier_errors = adapter->stats.tncrs;

	/* Tx Dropped needs to be maintained elsewhere */

	/* Management Stats */
	adapter->stats.mgptc += er32(MGTPTC);
	adapter->stats.mgprc += er32(MGTPRC);
	adapter->stats.mgpdc += er32(MGTPDC);

	/* Correctable ECC Errors */
	if (hw->mac.type == e1000_pch_lpt) {
		u32 pbeccsts = er32(PBECCSTS);
		adapter->corr_errors +=
		    pbeccsts & E1000_PBECCSTS_CORR_ERR_CNT_MASK;
		adapter->uncorr_errors +=
		    (pbeccsts & E1000_PBECCSTS_UNCORR_ERR_CNT_MASK) >>
		    E1000_PBECCSTS_UNCORR_ERR_CNT_SHIFT;
	}
}

/**
 * e1000_phy_read_status - Update the PHY register status snapshot
 * @adapter: board private structure
 **/
static void e1000_phy_read_status(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	struct e1000_phy_regs *phy = &adapter->phy_regs;

	if (!pm_runtime_suspended((&adapter->pdev->dev)->parent) &&
	    (er32(STATUS) & E1000_STATUS_LU) &&
	    (adapter->hw.phy.media_type == e1000_media_type_copper)) {
		int ret_val;

		ret_val = e1e_rphy(hw, MII_BMCR, &phy->bmcr);
		ret_val |= e1e_rphy(hw, MII_BMSR, &phy->bmsr);
		ret_val |= e1e_rphy(hw, MII_ADVERTISE, &phy->advertise);
		ret_val |= e1e_rphy(hw, MII_LPA, &phy->lpa);
		ret_val |= e1e_rphy(hw, MII_EXPANSION, &phy->expansion);
		ret_val |= e1e_rphy(hw, MII_CTRL1000, &phy->ctrl1000);
		ret_val |= e1e_rphy(hw, MII_STAT1000, &phy->stat1000);
		ret_val |= e1e_rphy(hw, MII_ESTATUS, &phy->estatus);
		if (ret_val)
			e_warn("Error reading PHY register\n");
	} else {
		/* Do not read PHY registers if link is not up
		 * Set values to typical power-on defaults
		 */
		phy->bmcr = (BMCR_SPEED1000 | BMCR_ANENABLE | BMCR_FULLDPLX);
		phy->bmsr = (BMSR_100FULL | BMSR_100HALF | BMSR_10FULL |
			     BMSR_10HALF | BMSR_ESTATEN | BMSR_ANEGCAPABLE |
			     BMSR_ERCAP);
		phy->advertise = (ADVERTISE_PAUSE_ASYM | ADVERTISE_PAUSE_CAP |
				  ADVERTISE_ALL | ADVERTISE_CSMA);
		phy->lpa = 0;
		phy->expansion = EXPANSION_ENABLENPAGE;
		phy->ctrl1000 = ADVERTISE_1000FULL;
		phy->stat1000 = 0;
		phy->estatus = (ESTATUS_1000_TFULL | ESTATUS_1000_THALF);
	}
}

static void e1000_print_link_info(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl = er32(CTRL);

	/* Link status message must follow this format for user tools */
	pr_info("%s NIC Link is Up %d Mbps %s Duplex, Flow Control: %s\n",
		adapter->netdev->name, adapter->link_speed,
		adapter->link_duplex == FULL_DUPLEX ? "Full" : "Half",
		(ctrl & E1000_CTRL_TFCE) && (ctrl & E1000_CTRL_RFCE) ? "Rx/Tx" :
		(ctrl & E1000_CTRL_RFCE) ? "Rx" :
		(ctrl & E1000_CTRL_TFCE) ? "Tx" : "None");
}

static bool e1000e_has_link(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	bool link_active = false;
	s32 ret_val = 0;

	/* get_link_status is set on LSC (link status) interrupt or
	 * Rx sequence error interrupt.  get_link_status will stay
	 * false until the check_for_link establishes link
	 * for copper adapters ONLY
	 */
	switch (hw->phy.media_type) {
	case e1000_media_type_copper:
		if (hw->mac.get_link_status) {
			ret_val = hw->mac.ops.check_for_link(hw);
			link_active = !hw->mac.get_link_status;
		} else {
			link_active = true;
		}
		break;
	case e1000_media_type_fiber:
		ret_val = hw->mac.ops.check_for_link(hw);
		link_active = !!(er32(STATUS) & E1000_STATUS_LU);
		break;
	case e1000_media_type_internal_serdes:
		ret_val = hw->mac.ops.check_for_link(hw);
		link_active = adapter->hw.mac.serdes_has_link;
		break;
	default:
	case e1000_media_type_unknown:
		break;
	}

	if ((ret_val == E1000_ERR_PHY) && (hw->phy.type == e1000_phy_igp_3) &&
	    (er32(CTRL) & E1000_PHY_CTRL_GBE_DISABLE)) {
		/* See e1000_kmrn_lock_loss_workaround_ich8lan() */
		e_info("Gigabit has been disabled, downgrading speed\n");
	}

	return link_active;
}

static void e1000e_enable_receives(struct e1000_adapter *adapter)
{
	/* make sure the receive unit is started */
	if ((adapter->flags & FLAG_RX_NEEDS_RESTART) &&
	    (adapter->flags & FLAG_RESTART_NOW)) {
		struct e1000_hw *hw = &adapter->hw;
		u32 rctl = er32(RCTL);
		ew32(RCTL, rctl | E1000_RCTL_EN);
		adapter->flags &= ~FLAG_RESTART_NOW;
	}
}

static void e1000_reset_task(struct work_struct *work)
{
	struct e1000_adapter *adapter;
	adapter = container_of(work, struct e1000_adapter, reset_task);

	/* don't run the task if already down */
	if (test_bit(__E1000_DOWN, &adapter->state))
		return;

	if (!(adapter->flags & FLAG_RESTART_NOW)) {
		e1000e_dump(adapter);
		e_err("Reset adapter unexpectedly\n");
	}
	e1000e_reinit_locked(adapter);
}

/**
 * e1000_get_stats64 - Get System Network Statistics
 * @netdev: network interface device structure
 * @stats: rtnl_link_stats64 pointer
 *
 * Returns the address of the device statistics structure.
 **/
struct rtnl_link_stats64 *e1000e_get_stats64(struct net_device *netdev,
					     struct rtnl_link_stats64 *stats)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);

	memset(stats, 0, sizeof(struct rtnl_link_stats64));
	spin_lock(&adapter->stats64_lock);
	e1000e_update_stats(adapter);
	/* Fill out the OS statistics structure */
	stats->rx_bytes = adapter->stats.gorc;
	stats->rx_packets = adapter->stats.gprc;
	stats->tx_bytes = adapter->stats.gotc;
	stats->tx_packets = adapter->stats.gptc;
	stats->multicast = adapter->stats.mprc;
	stats->collisions = adapter->stats.colc;

	/* Rx Errors */

	/* RLEC on some newer hardware can be incorrect so build
	 * our own version based on RUC and ROC
	 */
	stats->rx_errors = adapter->stats.rxerrc +
	    adapter->stats.crcerrs + adapter->stats.algnerrc +
	    adapter->stats.ruc + adapter->stats.roc + adapter->stats.cexterr;
	stats->rx_length_errors = adapter->stats.ruc + adapter->stats.roc;
	stats->rx_crc_errors = adapter->stats.crcerrs;
	stats->rx_frame_errors = adapter->stats.algnerrc;
	stats->rx_missed_errors = adapter->stats.mpc;

	/* Tx Errors */
	stats->tx_errors = adapter->stats.ecol + adapter->stats.latecol;
	stats->tx_aborted_errors = adapter->stats.ecol;
	stats->tx_window_errors = adapter->stats.latecol;
	stats->tx_carrier_errors = adapter->stats.tncrs;

	/* Tx Dropped needs to be maintained elsewhere */

	spin_unlock(&adapter->stats64_lock);
	return stats;
}

/**
 * e1000_change_mtu - Change the Maximum Transfer Unit
 * @netdev: network interface device structure
 * @new_mtu: new value for maximum frame size
 *
 * Returns 0 on success, negative on failure
 **/
static int e1000_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	int max_frame = new_mtu + ETH_HLEN + ETH_FCS_LEN;

	/* Jumbo frame support */
	if ((max_frame > ETH_FRAME_LEN + ETH_FCS_LEN) &&
	    !(adapter->flags & FLAG_HAS_JUMBO_FRAMES)) {
		e_err("Jumbo Frames not supported.\n");
		return -EINVAL;
	}

	/* Supported frame sizes */
	if ((new_mtu < ETH_ZLEN + ETH_FCS_LEN + VLAN_HLEN) ||
	    (max_frame > adapter->max_hw_frame_size)) {
		e_err("Unsupported MTU setting\n");
		return -EINVAL;
	}

	/* Jumbo frame workaround on 82579 and newer requires CRC be stripped */
	if ((adapter->hw.mac.type >= e1000_pch2lan) &&
	    !(adapter->flags2 & FLAG2_CRC_STRIPPING) &&
	    (new_mtu > ETH_DATA_LEN)) {
		e_err("Jumbo Frames not supported on this device when CRC stripping is disabled.\n");
		return -EINVAL;
	}

	while (test_and_set_bit(__E1000_RESETTING, &adapter->state))
		usleep_range(1000, 2000);
	/* e1000e_down -> e1000e_reset dependent on max_frame_size & mtu */
	adapter->max_frame_size = max_frame;
	e_info("changing MTU from %d to %d\n", netdev->mtu, new_mtu);
	netdev->mtu = new_mtu;
	if (netif_running(netdev))
		e1000e_down(adapter);

	/* NOTE: netdev_alloc_skb reserves 16 bytes, and typically NET_IP_ALIGN
	 * means we reserve 2 more, this pushes us to allocate from the next
	 * larger slab size.
	 * i.e. RXBUFFER_2048 --> size-4096 slab
	 * However with the new *_jumbo_rx* routines, jumbo receives will use
	 * fragmented skbs
	 */

	if (max_frame <= 2048)
		adapter->rx_buffer_len = 2048;
	else
		adapter->rx_buffer_len = 4096;

	/* adjust allocation if LPE protects us, and we aren't using SBP */
	if ((max_frame == ETH_FRAME_LEN + ETH_FCS_LEN) ||
	    (max_frame == ETH_FRAME_LEN + VLAN_HLEN + ETH_FCS_LEN))
		adapter->rx_buffer_len = ETH_FRAME_LEN + VLAN_HLEN
		    + ETH_FCS_LEN;

	if (netif_running(netdev))
		e1000e_up(adapter);
	else
		e1000e_reset(adapter);

	clear_bit(__E1000_RESETTING, &adapter->state);

	return 0;
}

static int e1000_mii_ioctl(struct net_device *netdev, struct ifreq *ifr,
			   int cmd)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct mii_ioctl_data *data = if_mii(ifr);

	if (adapter->hw.phy.media_type != e1000_media_type_copper)
		return -EOPNOTSUPP;

	switch (cmd) {
	case SIOCGMIIPHY:
		data->phy_id = adapter->hw.phy.addr;
		break;
	case SIOCGMIIREG:
		e1000_phy_read_status(adapter);

		switch (data->reg_num & 0x1F) {
		case MII_BMCR:
			data->val_out = adapter->phy_regs.bmcr;
			break;
		case MII_BMSR:
			data->val_out = adapter->phy_regs.bmsr;
			break;
		case MII_PHYSID1:
			data->val_out = (adapter->hw.phy.id >> 16);
			break;
		case MII_PHYSID2:
			data->val_out = (adapter->hw.phy.id & 0xFFFF);
			break;
		case MII_ADVERTISE:
			data->val_out = adapter->phy_regs.advertise;
			break;
		case MII_LPA:
			data->val_out = adapter->phy_regs.lpa;
			break;
		case MII_EXPANSION:
			data->val_out = adapter->phy_regs.expansion;
			break;
		case MII_CTRL1000:
			data->val_out = adapter->phy_regs.ctrl1000;
			break;
		case MII_STAT1000:
			data->val_out = adapter->phy_regs.stat1000;
			break;
		case MII_ESTATUS:
			data->val_out = adapter->phy_regs.estatus;
			break;
		default:
			return -EIO;
		}
		break;
	case SIOCSMIIREG:
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

/**
 * e1000e_hwtstamp_ioctl - control hardware time stamping
 * @netdev: network interface device structure
 * @ifreq: interface request
 *
 * Outgoing time stamping can be enabled and disabled. Play nice and
 * disable it when requested, although it shouldn't cause any overhead
 * when no packet needs it. At most one packet in the queue may be
 * marked for time stamping, otherwise it would be impossible to tell
 * for sure to which packet the hardware time stamp belongs.
 *
 * Incoming time stamping has to be configured via the hardware filters.
 * Not all combinations are supported, in particular event type has to be
 * specified. Matching the kind of event packet is not supported, with the
 * exception of "all V2 events regardless of level 2 or 4".
 **/
static int e1000e_hwtstamp_ioctl(struct net_device *netdev, struct ifreq *ifr)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct hwtstamp_config config;
	int ret_val;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	ret_val = e1000e_config_hwtstamp(adapter, &config);
	if (ret_val)
		return ret_val;

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		/* With V2 type filters which specify a Sync or Delay Request,
		 * Path Delay Request/Response messages are also time stamped
		 * by hardware so notify the caller the requested packets plus
		 * some others are time stamped.
		 */
		config.rx_filter = HWTSTAMP_FILTER_SOME;
		break;
	default:
		break;
	}

	return copy_to_user(ifr->ifr_data, &config,
			    sizeof(config)) ? -EFAULT : 0;
}

static int e1000_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		return e1000_mii_ioctl(netdev, ifr, cmd);
	case SIOCSHWTSTAMP:
		return e1000e_hwtstamp_ioctl(netdev, ifr);
	default:
		return -EOPNOTSUPP;
	}
}

static int e1000_init_phy_wakeup(struct e1000_adapter *adapter, u32 wufc)
{
	struct e1000_hw *hw = &adapter->hw;
	u32 i, mac_reg;
	u16 phy_reg, wuc_enable;
	int retval;

	/* copy MAC RARs to PHY RARs */
	e1000_copy_rx_addrs_to_phy_ich8lan(hw);

	retval = hw->phy.ops.acquire(hw);
	if (retval) {
		e_err("Could not acquire PHY\n");
		return retval;
	}

	/* Enable access to wakeup registers on and set page to BM_WUC_PAGE */
	retval = e1000_enable_phy_wakeup_reg_access_bm(hw, &wuc_enable);
	if (retval)
		goto release;

	/* copy MAC MTA to PHY MTA - only needed for pchlan */
	for (i = 0; i < adapter->hw.mac.mta_reg_count; i++) {
		mac_reg = E1000_READ_REG_ARRAY(hw, E1000_MTA, i);
		hw->phy.ops.write_reg_page(hw, BM_MTA(i),
					   (u16)(mac_reg & 0xFFFF));
		hw->phy.ops.write_reg_page(hw, BM_MTA(i) + 1,
					   (u16)((mac_reg >> 16) & 0xFFFF));
	}

	/* configure PHY Rx Control register */
	hw->phy.ops.read_reg_page(&adapter->hw, BM_RCTL, &phy_reg);
	mac_reg = er32(RCTL);
	if (mac_reg & E1000_RCTL_UPE)
		phy_reg |= BM_RCTL_UPE;
	if (mac_reg & E1000_RCTL_MPE)
		phy_reg |= BM_RCTL_MPE;
	phy_reg &= ~(BM_RCTL_MO_MASK);
	if (mac_reg & E1000_RCTL_MO_3)
		phy_reg |= (((mac_reg & E1000_RCTL_MO_3) >> E1000_RCTL_MO_SHIFT)
			    << BM_RCTL_MO_SHIFT);
	if (mac_reg & E1000_RCTL_BAM)
		phy_reg |= BM_RCTL_BAM;
	if (mac_reg & E1000_RCTL_PMCF)
		phy_reg |= BM_RCTL_PMCF;
	mac_reg = er32(CTRL);
	if (mac_reg & E1000_CTRL_RFCE)
		phy_reg |= BM_RCTL_RFCE;
	hw->phy.ops.write_reg_page(&adapter->hw, BM_RCTL, phy_reg);

	/* enable PHY wakeup in MAC register */
	ew32(WUFC, wufc);
	ew32(WUC, E1000_WUC_PHY_WAKE | E1000_WUC_PME_EN);

	/* configure and enable PHY wakeup in PHY registers */
	hw->phy.ops.write_reg_page(&adapter->hw, BM_WUFC, wufc);
	hw->phy.ops.write_reg_page(&adapter->hw, BM_WUC, E1000_WUC_PME_EN);

	/* activate PHY wakeup */
	wuc_enable |= BM_WUC_ENABLE_BIT | BM_WUC_HOST_WU_BIT;
	retval = e1000_disable_phy_wakeup_reg_access_bm(hw, &wuc_enable);
	if (retval)
		e_err("Could not set PHY Host Wakeup bit\n");
release:
	hw->phy.ops.release(hw);

	return retval;
}

static int __e1000_shutdown(struct pci_dev *pdev, bool runtime)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u32 ctrl, ctrl_ext, rctl, status;
	/* Runtime suspend should only enable wakeup for link changes */
	u32 wufc = runtime ? E1000_WUFC_LNKC : adapter->wol;
	int retval = 0;

	netif_device_detach(netdev);

	if (netif_running(netdev)) {
		int count = E1000_CHECK_RESET_COUNT;

		while (test_bit(__E1000_RESETTING, &adapter->state) && count--)
			usleep_range(10000, 20000);

		WARN_ON(test_bit(__E1000_RESETTING, &adapter->state));
		e1000e_down(adapter);
		e1000_free_irq(adapter);
	}
	e1000e_reset_interrupt_capability(adapter);

	status = er32(STATUS);
	if (status & E1000_STATUS_LU)
		wufc &= ~E1000_WUFC_LNKC;

	if (wufc) {
		e1000_setup_rctl(adapter);
		e1000e_set_rx_mode(netdev);

		/* turn on all-multi mode if wake on multicast is enabled */
		if (wufc & E1000_WUFC_MC) {
			rctl = er32(RCTL);
			rctl |= E1000_RCTL_MPE;
			ew32(RCTL, rctl);
		}

		ctrl = er32(CTRL);
		ctrl |= E1000_CTRL_ADVD3WUC;
		if (!(adapter->flags2 & FLAG2_HAS_PHY_WAKEUP))
			ctrl |= E1000_CTRL_EN_PHY_PWR_MGMT;
		ew32(CTRL, ctrl);

		if (adapter->hw.phy.media_type == e1000_media_type_fiber ||
		    adapter->hw.phy.media_type ==
		    e1000_media_type_internal_serdes) {
			/* keep the laser running in D3 */
			ctrl_ext = er32(CTRL_EXT);
			ctrl_ext |= E1000_CTRL_EXT_SDP3_DATA;
			ew32(CTRL_EXT, ctrl_ext);
		}

		if (adapter->flags & FLAG_IS_ICH)
			e1000_suspend_workarounds_ich8lan(&adapter->hw);

		/* Allow time for pending master requests to run */
		e1000e_disable_pcie_master(&adapter->hw);

		if (adapter->flags2 & FLAG2_HAS_PHY_WAKEUP) {
			/* enable wakeup by the PHY */
			retval = e1000_init_phy_wakeup(adapter, wufc);
			if (retval)
				return retval;
		} else {
			/* enable wakeup by the MAC */
			ew32(WUFC, wufc);
			ew32(WUC, E1000_WUC_PME_EN);
		}
	} else {
		ew32(WUC, 0);
		ew32(WUFC, 0);
	}

	if (adapter->hw.phy.type == e1000_phy_igp_3)
		e1000e_igp3_phy_powerdown_workaround_ich8lan(&adapter->hw);

	/* Release control of h/w to f/w.  If f/w is AMT enabled, this
	 * would have already happened in close and is redundant.
	 */
	e1000e_release_hw_control(adapter);

	pci_clear_master(pdev);

	/* The pci-e switch on some quad port adapters will report a
	 * correctable error when the MAC transitions from D0 to D3.  To
	 * prevent this we need to mask off the correctable errors on the
	 * downstream port of the pci-e switch.
	 *
	 * We don't have the associated upstream bridge while assigning
	 * the PCI device into guest. For example, the KVM on power is
	 * one of the cases.
	 */
	if (adapter->flags & FLAG_IS_QUAD_PORT) {
		struct pci_dev *us_dev = pdev->bus->self;
		u16 devctl;

		if (!us_dev)
			return 0;

		pcie_capability_read_word(us_dev, PCI_EXP_DEVCTL, &devctl);
		pcie_capability_write_word(us_dev, PCI_EXP_DEVCTL,
					   (devctl & ~PCI_EXP_DEVCTL_CERE));

		pci_save_state(pdev);
		pci_prepare_to_sleep(pdev);

		pcie_capability_write_word(us_dev, PCI_EXP_DEVCTL, devctl);
	}

	return 0;
}

/**
 * e1000e_disable_aspm - Disable ASPM states
 * @pdev: pointer to PCI device struct
 * @state: bit-mask of ASPM states to disable
 *
 * Some devices *must* have certain ASPM states disabled per hardware errata.
 **/
static void e1000e_disable_aspm(struct pci_dev *pdev, u16 state)
{
	struct pci_dev *parent = pdev->bus->self;
	u16 aspm_dis_mask = 0;
	u16 pdev_aspmc, parent_aspmc;

	switch (state) {
	case PCIE_LINK_STATE_L0S:
	case PCIE_LINK_STATE_L0S | PCIE_LINK_STATE_L1:
		aspm_dis_mask |= PCI_EXP_LNKCTL_ASPM_L0S;
		/* fall-through - can't have L1 without L0s */
	case PCIE_LINK_STATE_L1:
		aspm_dis_mask |= PCI_EXP_LNKCTL_ASPM_L1;
		break;
	default:
		return;
	}

	pcie_capability_read_word(pdev, PCI_EXP_LNKCTL, &pdev_aspmc);
	pdev_aspmc &= PCI_EXP_LNKCTL_ASPMC;

	if (parent) {
		pcie_capability_read_word(parent, PCI_EXP_LNKCTL,
					  &parent_aspmc);
		parent_aspmc &= PCI_EXP_LNKCTL_ASPMC;
	}

	/* Nothing to do if the ASPM states to be disabled already are */
	if (!(pdev_aspmc & aspm_dis_mask) &&
	    (!parent || !(parent_aspmc & aspm_dis_mask)))
		return;

	dev_info(&pdev->dev, "Disabling ASPM %s %s\n",
		 (aspm_dis_mask & pdev_aspmc & PCI_EXP_LNKCTL_ASPM_L0S) ?
		 "L0s" : "",
		 (aspm_dis_mask & pdev_aspmc & PCI_EXP_LNKCTL_ASPM_L1) ?
		 "L1" : "");

#ifdef CONFIG_PCIEASPM
	pci_disable_link_state_locked(pdev, state);

	/* Double-check ASPM control.  If not disabled by the above, the
	 * BIOS is preventing that from happening (or CONFIG_PCIEASPM is
	 * not enabled); override by writing PCI config space directly.
	 */
	pcie_capability_read_word(pdev, PCI_EXP_LNKCTL, &pdev_aspmc);
	pdev_aspmc &= PCI_EXP_LNKCTL_ASPMC;

	if (!(aspm_dis_mask & pdev_aspmc))
		return;
#endif

	/* Both device and parent should have the same ASPM setting.
	 * Disable ASPM in downstream component first and then upstream.
	 */
	pcie_capability_clear_word(pdev, PCI_EXP_LNKCTL, aspm_dis_mask);

	if (parent)
		pcie_capability_clear_word(parent, PCI_EXP_LNKCTL,
					   aspm_dis_mask);
}

#ifdef CONFIG_PM
static bool e1000e_pm_ready(struct e1000_adapter *adapter)
{
	return !!adapter->tx_ring->buffer_info;
}

static int __e1000_resume(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u16 aspm_disable_flag = 0;
	u32 err;

	if (adapter->flags2 & FLAG2_DISABLE_ASPM_L0S)
		aspm_disable_flag = PCIE_LINK_STATE_L0S;
	if (adapter->flags2 & FLAG2_DISABLE_ASPM_L1)
		aspm_disable_flag |= PCIE_LINK_STATE_L1;
	if (aspm_disable_flag)
		e1000e_disable_aspm(pdev, aspm_disable_flag);

	pci_set_master(pdev);

	e1000e_set_interrupt_capability(adapter);
	if (netif_running(netdev)) {
		err = e1000_request_irq(adapter);
		if (err)
			return err;
	}

	if (hw->mac.type >= e1000_pch2lan)
		e1000_resume_workarounds_pchlan(&adapter->hw);

	e1000e_power_up_phy(adapter);

	/* report the system wakeup cause from S3/S4 */
	if (adapter->flags2 & FLAG2_HAS_PHY_WAKEUP) {
		u16 phy_data;

		e1e_rphy(&adapter->hw, BM_WUS, &phy_data);
		if (phy_data) {
			e_info("PHY Wakeup cause - %s\n",
			       phy_data & E1000_WUS_EX ? "Unicast Packet" :
			       phy_data & E1000_WUS_MC ? "Multicast Packet" :
			       phy_data & E1000_WUS_BC ? "Broadcast Packet" :
			       phy_data & E1000_WUS_MAG ? "Magic Packet" :
			       phy_data & E1000_WUS_LNKC ?
			       "Link Status Change" : "other");
		}
		e1e_wphy(&adapter->hw, BM_WUS, ~0);
	} else {
		u32 wus = er32(WUS);
		if (wus) {
			e_info("MAC Wakeup cause - %s\n",
			       wus & E1000_WUS_EX ? "Unicast Packet" :
			       wus & E1000_WUS_MC ? "Multicast Packet" :
			       wus & E1000_WUS_BC ? "Broadcast Packet" :
			       wus & E1000_WUS_MAG ? "Magic Packet" :
			       wus & E1000_WUS_LNKC ? "Link Status Change" :
			       "other");
		}
		ew32(WUS, ~0);
	}

	e1000e_reset(adapter);

	e1000_init_manageability_pt(adapter);

	if (netif_running(netdev))
		e1000e_up(adapter);

	netif_device_attach(netdev);

	/* If the controller has AMT, do not set DRV_LOAD until the interface
	 * is up.  For all other cases, let the f/w know that the h/w is now
	 * under the control of the driver.
	 */
	if (!(adapter->flags & FLAG_HAS_AMT))
		e1000e_get_hw_control(adapter);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int e1000_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);

	return __e1000_shutdown(pdev, false);
}

static int e1000_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);

	if (e1000e_pm_ready(adapter))
		adapter->idle_check = true;

	return __e1000_resume(pdev);
}
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM_RUNTIME
static int e1000_runtime_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);

	if (!e1000e_pm_ready(adapter))
		return 0;

	return __e1000_shutdown(pdev, true);
}

static int e1000_idle(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);

	if (!e1000e_pm_ready(adapter))
		return 0;

	if (adapter->idle_check) {
		adapter->idle_check = false;
		if (!e1000e_has_link(adapter))
			pm_schedule_suspend(dev, MSEC_PER_SEC);
	}

	return -EBUSY;
}

static int e1000_runtime_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);

	if (!e1000e_pm_ready(adapter))
		return 0;

	adapter->idle_check = !dev->power.runtime_auto;
	return __e1000_resume(pdev);
}
#endif /* CONFIG_PM_RUNTIME */
#endif /* CONFIG_PM */

static void e1000_shutdown(struct pci_dev *pdev)
{
	__e1000_shutdown(pdev, false);
}

/**
 * e1000_io_error_detected - called when PCI error is detected
 * @pdev: Pointer to PCI device
 * @state: The current pci connection state
 *
 * This function is called after a PCI bus error affecting
 * this device has been detected.
 */
static pci_ers_result_t e1000_io_error_detected(struct pci_dev *pdev,
						pci_channel_state_t state)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);

	netif_device_detach(netdev);

	if (state == pci_channel_io_perm_failure)
		return PCI_ERS_RESULT_DISCONNECT;

	if (netif_running(netdev))
		e1000e_down(adapter);
	pci_disable_device(pdev);

	/* Request a slot slot reset. */
	return PCI_ERS_RESULT_NEED_RESET;
}

/**
 * e1000_io_slot_reset - called after the pci bus has been reset.
 * @pdev: Pointer to PCI device
 *
 * Restart the card from scratch, as if from a cold-boot. Implementation
 * resembles the first-half of the e1000_resume routine.
 */
static pci_ers_result_t e1000_io_slot_reset(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);
	struct e1000_hw *hw = &adapter->hw;
	u16 aspm_disable_flag = 0;
	int err;
	pci_ers_result_t result;

	if (adapter->flags2 & FLAG2_DISABLE_ASPM_L0S)
		aspm_disable_flag = PCIE_LINK_STATE_L0S;
	if (adapter->flags2 & FLAG2_DISABLE_ASPM_L1)
		aspm_disable_flag |= PCIE_LINK_STATE_L1;
	if (aspm_disable_flag)
		e1000e_disable_aspm(pdev, aspm_disable_flag);

	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev,
			"Cannot re-enable PCI device after reset.\n");
		result = PCI_ERS_RESULT_DISCONNECT;
	} else {
		pdev->state_saved = true;
		pci_restore_state(pdev);
		pci_set_master(pdev);

		pci_enable_wake(pdev, PCI_D3hot, 0);
		pci_enable_wake(pdev, PCI_D3cold, 0);

		e1000e_reset(adapter);
		ew32(WUS, ~0);
		result = PCI_ERS_RESULT_RECOVERED;
	}

	pci_cleanup_aer_uncorrect_error_status(pdev);

	return result;
}

/**
 * e1000_io_resume - called when traffic can start flowing again.
 * @pdev: Pointer to PCI device
 *
 * This callback is called when the error recovery driver tells us that
 * its OK to resume normal operation. Implementation resembles the
 * second-half of the e1000_resume routine.
 */
static void e1000_io_resume(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);

	e1000_init_manageability_pt(adapter);

	if (netif_running(netdev)) {
		if (e1000e_up(adapter)) {
			dev_err(&pdev->dev,
				"can't bring device back up after reset\n");
			return;
		}
	}

	netif_device_attach(netdev);

	/* If the controller has AMT, do not set DRV_LOAD until the interface
	 * is up.  For all other cases, let the f/w know that the h/w is now
	 * under the control of the driver.
	 */
	if (!(adapter->flags & FLAG_HAS_AMT))
		e1000e_get_hw_control(adapter);
}

static void e1000_print_device_info(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	struct net_device *netdev = adapter->netdev;
	u32 ret_val;
	u8 pba_str[E1000_PBANUM_LENGTH];

	/* print bus type/speed/width info */
	e_info("(PCI Express:2.5GT/s:%s) %pM\n",
	       /* bus width */
	       ((hw->bus.width == e1000_bus_width_pcie_x4) ? "Width x4" :
		"Width x1"),
	       /* MAC address */
	       netdev->dev_addr);
	e_info("Intel(R) PRO/%s Network Connection\n",
	       (hw->phy.type == e1000_phy_ife) ? "10/100" : "1000");
	ret_val = e1000_read_pba_string_generic(hw, pba_str,
						E1000_PBANUM_LENGTH);
	if (ret_val)
		strlcpy((char *)pba_str, "Unknown", sizeof(pba_str));
	e_info("MAC: %d, PHY: %d, PBA No: %s\n",
	       hw->mac.type, hw->phy.type, pba_str);
}

static void e1000_eeprom_checks(struct e1000_adapter *adapter)
{
	struct e1000_hw *hw = &adapter->hw;
	int ret_val;
	u16 buf = 0;

	ret_val = e1000_read_nvm(hw, NVM_INIT_CONTROL2_REG, 1, &buf);
	le16_to_cpus(&buf);
	if (!ret_val && (!(buf & (1 << 0)))) {
		/* Deep Smart Power Down (DSPD) */
		dev_warn(&adapter->pdev->dev,
			 "Warning: detected DSPD enabled in EEPROM\n");
	}
}

static int e1000_set_features(struct net_device *netdev,
			      netdev_features_t features)
{
	struct e1000_adapter *adapter = netdev_priv(netdev);
	netdev_features_t changed = features ^ netdev->features;

	if (changed & (NETIF_F_TSO | NETIF_F_TSO6))
		adapter->flags |= FLAG_TSO_FORCE;

	if (!(changed & (NETIF_F_HW_VLAN_CTAG_RX | NETIF_F_HW_VLAN_CTAG_TX |
			 NETIF_F_RXCSUM | NETIF_F_RXHASH | NETIF_F_RXFCS |
			 NETIF_F_RXALL)))
		return 0;

	if (changed & NETIF_F_RXFCS) {
		if (features & NETIF_F_RXFCS) {
			adapter->flags2 &= ~FLAG2_CRC_STRIPPING;
		} else {
			/* We need to take it back to defaults, which might mean
			 * stripping is still disabled at the adapter level.
			 */
			if (adapter->flags2 & FLAG2_DFLT_CRC_STRIPPING)
				adapter->flags2 |= FLAG2_CRC_STRIPPING;
			else
				adapter->flags2 &= ~FLAG2_CRC_STRIPPING;
		}
	}

	netdev->features = features;

	if (netif_running(netdev))
		e1000e_reinit_locked(adapter);
	else
		e1000e_reset(adapter);

	return 0;
}

static const struct net_device_ops e1000e_netdev_ops = {
	.ndo_open		= e1000_open,
	.ndo_stop		= e1000_close,
	.ndo_start_xmit		= e1000_xmit_frame,
	.ndo_get_stats64	= e1000e_get_stats64,
	.ndo_set_rx_mode	= e1000e_set_rx_mode,
	.ndo_set_mac_address	= e1000_set_mac,
	.ndo_change_mtu		= e1000_change_mtu,
	.ndo_do_ioctl		= e1000_ioctl,
	.ndo_tx_timeout		= e1000_tx_timeout,
	.ndo_validate_addr	= eth_validate_addr,

	.ndo_vlan_rx_add_vid	= e1000_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= e1000_vlan_rx_kill_vid,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= e1000_netpoll,
#endif
	.ndo_set_features = e1000_set_features,
};

/**
 * e1000_probe - Device Initialization Routine
 * @pdev: PCI device information struct
 * @ent: entry in e1000_pci_tbl
 *
 * Returns 0 on success, negative on failure
 *
 * e1000_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
static int e1000_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct net_device *netdev;
	struct e1000_adapter *adapter;
	struct e1000_hw *hw;
	const struct e1000_info *ei = e1000_info_tbl[ent->driver_data];
	resource_size_t mmio_start, mmio_len;
	resource_size_t flash_start, flash_len;
	static int cards_found;
	u16 aspm_disable_flag = 0;
	int bars, i, err, pci_using_dac;
	u16 eeprom_data = 0;
	u16 eeprom_apme_mask = E1000_EEPROM_APME;

	if (ei->flags2 & FLAG2_DISABLE_ASPM_L0S)
		aspm_disable_flag = PCIE_LINK_STATE_L0S;
	if (ei->flags2 & FLAG2_DISABLE_ASPM_L1)
		aspm_disable_flag |= PCIE_LINK_STATE_L1;
	if (aspm_disable_flag)
		e1000e_disable_aspm(pdev, aspm_disable_flag);

	err = pci_enable_device_mem(pdev);
	if (err)
		return err;

	pci_using_dac = 0;
	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (!err) {
		pci_using_dac = 1;
	} else {
		err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (err) {
			dev_err(&pdev->dev,
				"No usable DMA configuration, aborting\n");
			goto err_dma;
		}
	}

	bars = pci_select_bars(pdev, IORESOURCE_MEM);
	err = pci_request_selected_regions_exclusive(pdev, bars,
						     e1000e_driver_name);
	if (err)
		goto err_pci_reg;

	/* AER (Advanced Error Reporting) hooks */
	pci_enable_pcie_error_reporting(pdev);

	pci_set_master(pdev);
	/* PCI config space info */
	err = pci_save_state(pdev);
	if (err)
		goto err_alloc_etherdev;

	err = -ENOMEM;
	netdev = alloc_etherdev(sizeof(struct e1000_adapter));
	if (!netdev)
		goto err_alloc_etherdev;

	SET_NETDEV_DEV(netdev, &pdev->dev);

	netdev->irq = pdev->irq;

	pci_set_drvdata(pdev, netdev);
	adapter = netdev_priv(netdev);
	hw = &adapter->hw;
	adapter->netdev = netdev;
	adapter->pdev = pdev;
	adapter->ei = ei;
	adapter->pba = ei->pba;
	adapter->flags = ei->flags;
	adapter->flags2 = ei->flags2;
	adapter->hw.adapter = adapter;
	adapter->hw.mac.type = ei->mac;
	adapter->max_hw_frame_size = ei->max_hw_frame_size;
	adapter->msg_enable = netif_msg_init(debug, DEFAULT_MSG_ENABLE);

	mmio_start = pci_resource_start(pdev, 0);
	mmio_len = pci_resource_len(pdev, 0);

	err = -EIO;
	adapter->hw.hw_addr = ioremap(mmio_start, mmio_len);
	if (!adapter->hw.hw_addr)
		goto err_ioremap;

	if ((adapter->flags & FLAG_HAS_FLASH) &&
	    (pci_resource_flags(pdev, 1) & IORESOURCE_MEM)) {
		flash_start = pci_resource_start(pdev, 1);
		flash_len = pci_resource_len(pdev, 1);
		adapter->hw.flash_address = ioremap(flash_start, flash_len);
		if (!adapter->hw.flash_address)
			goto err_flashmap;
	}

	/* Set default EEE advertisement */
	if (adapter->flags2 & FLAG2_HAS_EEE)
		adapter->eee_advert = MDIO_EEE_100TX | MDIO_EEE_1000T;

	/* construct the net_device struct */
	netdev->netdev_ops = &e1000e_netdev_ops;
	e1000e_set_ethtool_ops(netdev);
	netdev->watchdog_timeo = 5 * HZ;
	netif_napi_add(netdev, &adapter->napi, e1000e_poll, 64);
	strlcpy(netdev->name, pci_name(pdev), sizeof(netdev->name));

	netdev->mem_start = mmio_start;
	netdev->mem_end = mmio_start + mmio_len;

	adapter->bd_number = cards_found++;

	e1000e_check_options(adapter);

	/* setup adapter struct */
	err = e1000_sw_init(adapter);
	if (err)
		goto err_sw_init;

	memcpy(&hw->mac.ops, ei->mac_ops, sizeof(hw->mac.ops));
	memcpy(&hw->nvm.ops, ei->nvm_ops, sizeof(hw->nvm.ops));
	memcpy(&hw->phy.ops, ei->phy_ops, sizeof(hw->phy.ops));

	err = ei->get_variants(adapter);
	if (err)
		goto err_hw_init;

	if ((adapter->flags & FLAG_IS_ICH) &&
	    (adapter->flags & FLAG_READ_ONLY_NVM))
		e1000e_write_protect_nvm_ich8lan(&adapter->hw);

	hw->mac.ops.get_bus_info(&adapter->hw);

	adapter->hw.phy.autoneg_wait_to_complete = 0;

	/* Copper options */
	if (adapter->hw.phy.media_type == e1000_media_type_copper) {
		adapter->hw.phy.mdix = AUTO_ALL_MODES;
		adapter->hw.phy.disable_polarity_correction = 0;
		adapter->hw.phy.ms_type = e1000_ms_hw_default;
	}

	if (hw->phy.ops.check_reset_block && hw->phy.ops.check_reset_block(hw))
		dev_info(&pdev->dev,
			 "PHY reset is blocked due to SOL/IDER session.\n");

	/* Set initial default active device features */
	netdev->features = (NETIF_F_SG |
			    NETIF_F_HW_VLAN_CTAG_RX |
			    NETIF_F_HW_VLAN_CTAG_TX |
			    NETIF_F_TSO |
			    NETIF_F_TSO6 |
			    NETIF_F_RXHASH |
			    NETIF_F_RXCSUM |
			    NETIF_F_HW_CSUM);

	/* Set user-changeable features (subset of all device features) */
	netdev->hw_features = netdev->features;
	netdev->hw_features |= NETIF_F_RXFCS;
	netdev->priv_flags |= IFF_SUPP_NOFCS;
	netdev->hw_features |= NETIF_F_RXALL;

	if (adapter->flags & FLAG_HAS_HW_VLAN_FILTER)
		netdev->features |= NETIF_F_HW_VLAN_CTAG_FILTER;

	netdev->vlan_features |= (NETIF_F_SG |
				  NETIF_F_TSO |
				  NETIF_F_TSO6 |
				  NETIF_F_HW_CSUM);

	netdev->priv_flags |= IFF_UNICAST_FLT;

	if (pci_using_dac) {
		netdev->features |= NETIF_F_HIGHDMA;
		netdev->vlan_features |= NETIF_F_HIGHDMA;
	}

	if (e1000e_enable_mng_pass_thru(&adapter->hw))
		adapter->flags |= FLAG_MNG_PT_ENABLED;

	/* before reading the NVM, reset the controller to
	 * put the device in a known good starting state
	 */
	adapter->hw.mac.ops.reset_hw(&adapter->hw);

	/* systems with ASPM and others may see the checksum fail on the first
	 * attempt. Let's give it a few tries
	 */
	for (i = 0;; i++) {
		if (e1000_validate_nvm_checksum(&adapter->hw) >= 0)
			break;
		if (i == 2) {
			dev_err(&pdev->dev, "The NVM Checksum Is Not Valid\n");
			err = -EIO;
			goto err_eeprom;
		}
	}

	e1000_eeprom_checks(adapter);

	/* copy the MAC address */
	if (e1000e_read_mac_addr(&adapter->hw))
		dev_err(&pdev->dev,
			"NVM Read Error while reading MAC address\n");

	memcpy(netdev->dev_addr, adapter->hw.mac.addr, netdev->addr_len);

	if (!is_valid_ether_addr(netdev->dev_addr)) {
		dev_err(&pdev->dev, "Invalid MAC Address: %pM\n",
			netdev->dev_addr);
		err = -EIO;
		goto err_eeprom;
	}

	init_timer(&adapter->watchdog_timer);
	adapter->watchdog_timer.function = e1000_watchdog;
	adapter->watchdog_timer.data = (unsigned long)adapter;

	init_timer(&adapter->phy_info_timer);
	adapter->phy_info_timer.function = e1000_update_phy_info;
	adapter->phy_info_timer.data = (unsigned long)adapter;

	INIT_WORK(&adapter->reset_task, e1000_reset_task);
	INIT_WORK(&adapter->watchdog_task, e1000_watchdog_task);
	INIT_WORK(&adapter->downshift_task, e1000e_downshift_workaround);
	INIT_WORK(&adapter->update_phy_task, e1000e_update_phy_task);
	INIT_WORK(&adapter->print_hang_task, e1000_print_hw_hang);

	/* Initialize link parameters. User can change them with ethtool */
	adapter->hw.mac.autoneg = 1;
	adapter->fc_autoneg = true;
	adapter->hw.fc.requested_mode = e1000_fc_default;
	adapter->hw.fc.current_mode = e1000_fc_default;
	adapter->hw.phy.autoneg_advertised = 0x2f;

	/* Initial Wake on LAN setting - If APM wake is enabled in
	 * the EEPROM, enable the ACPI Magic Packet filter
	 */
	if (adapter->flags & FLAG_APME_IN_WUC) {
		/* APME bit in EEPROM is mapped to WUC.APME */
		eeprom_data = er32(WUC);
		eeprom_apme_mask = E1000_WUC_APME;
		if ((hw->mac.type > e1000_ich10lan) &&
		    (eeprom_data & E1000_WUC_PHY_WAKE))
			adapter->flags2 |= FLAG2_HAS_PHY_WAKEUP;
	} else if (adapter->flags & FLAG_APME_IN_CTRL3) {
		if (adapter->flags & FLAG_APME_CHECK_PORT_B &&
		    (adapter->hw.bus.func == 1))
			e1000_read_nvm(&adapter->hw, NVM_INIT_CONTROL3_PORT_B,
				       1, &eeprom_data);
		else
			e1000_read_nvm(&adapter->hw, NVM_INIT_CONTROL3_PORT_A,
				       1, &eeprom_data);
	}

	/* fetch WoL from EEPROM */
	if (eeprom_data & eeprom_apme_mask)
		adapter->eeprom_wol |= E1000_WUFC_MAG;

	/* now that we have the eeprom settings, apply the special cases
	 * where the eeprom may be wrong or the board simply won't support
	 * wake on lan on a particular port
	 */
	if (!(adapter->flags & FLAG_HAS_WOL))
		adapter->eeprom_wol = 0;

	/* initialize the wol settings based on the eeprom settings */
	adapter->wol = adapter->eeprom_wol;

	/* make sure adapter isn't asleep if manageability is enabled */
	if (adapter->wol || (adapter->flags & FLAG_MNG_PT_ENABLED) ||
	    (hw->mac.ops.check_mng_mode(hw)))
		device_wakeup_enable(&pdev->dev);

	/* save off EEPROM version number */
	e1000_read_nvm(&adapter->hw, 5, 1, &adapter->eeprom_vers);

	/* reset the hardware with the new settings */
	e1000e_reset(adapter);

	/* If the controller has AMT, do not set DRV_LOAD until the interface
	 * is up.  For all other cases, let the f/w know that the h/w is now
	 * under the control of the driver.
	 */
	if (!(adapter->flags & FLAG_HAS_AMT))
		e1000e_get_hw_control(adapter);

	strlcpy(netdev->name, "eth%d", sizeof(netdev->name));
	err = register_netdev(netdev);
	if (err)
		goto err_register;

	/* carrier off reporting is important to ethtool even BEFORE open */
	netif_carrier_off(netdev);

	/* init PTP hardware clock */
	e1000e_ptp_init(adapter);

	e1000_print_device_info(adapter);

	if (pci_dev_run_wake(pdev))
		pm_runtime_put_noidle(&pdev->dev);

	return 0;

err_register:
	if (!(adapter->flags & FLAG_HAS_AMT))
		e1000e_release_hw_control(adapter);
err_eeprom:
	if (hw->phy.ops.check_reset_block && !hw->phy.ops.check_reset_block(hw))
		e1000_phy_hw_reset(&adapter->hw);
err_hw_init:
	kfree(adapter->tx_ring);
	kfree(adapter->rx_ring);
err_sw_init:
	if (adapter->hw.flash_address)
		iounmap(adapter->hw.flash_address);
	e1000e_reset_interrupt_capability(adapter);
err_flashmap:
	iounmap(adapter->hw.hw_addr);
err_ioremap:
	free_netdev(netdev);
err_alloc_etherdev:
	pci_release_selected_regions(pdev,
				     pci_select_bars(pdev, IORESOURCE_MEM));
err_pci_reg:
err_dma:
	pci_disable_device(pdev);
	return err;
}

/**
 * e1000_remove - Device Removal Routine
 * @pdev: PCI device information struct
 *
 * e1000_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.  The could be caused by a
 * Hot-Plug event, or because the driver is going to be removed from
 * memory.
 **/
static void e1000_remove(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct e1000_adapter *adapter = netdev_priv(netdev);
	bool down = test_bit(__E1000_DOWN, &adapter->state);

	e1000e_ptp_remove(adapter);

	/* The timers may be rescheduled, so explicitly disable them
	 * from being rescheduled.
	 */
	if (!down)
		set_bit(__E1000_DOWN, &adapter->state);
	del_timer_sync(&adapter->watchdog_timer);
	del_timer_sync(&adapter->phy_info_timer);

	cancel_work_sync(&adapter->reset_task);
	cancel_work_sync(&adapter->watchdog_task);
	cancel_work_sync(&adapter->downshift_task);
	cancel_work_sync(&adapter->update_phy_task);
	cancel_work_sync(&adapter->print_hang_task);

	if (adapter->flags & FLAG_HAS_HW_TIMESTAMP) {
		cancel_work_sync(&adapter->tx_hwtstamp_work);
		if (adapter->tx_hwtstamp_skb) {
			dev_kfree_skb_any(adapter->tx_hwtstamp_skb);
			adapter->tx_hwtstamp_skb = NULL;
		}
	}

	if (!(netdev->flags & IFF_UP))
		e1000_power_down_phy(adapter);

	/* Don't lie to e1000_close() down the road. */
	if (!down)
		clear_bit(__E1000_DOWN, &adapter->state);
	unregister_netdev(netdev);

	if (pci_dev_run_wake(pdev))
		pm_runtime_get_noresume(&pdev->dev);

	/* Release control of h/w to f/w.  If f/w is AMT enabled, this
	 * would have already happened in close and is redundant.
	 */
	e1000e_release_hw_control(adapter);

	e1000e_reset_interrupt_capability(adapter);
	kfree(adapter->tx_ring);
	kfree(adapter->rx_ring);

	iounmap(adapter->hw.hw_addr);
	if (adapter->hw.flash_address)
		iounmap(adapter->hw.flash_address);
	pci_release_selected_regions(pdev,
				     pci_select_bars(pdev, IORESOURCE_MEM));

	free_netdev(netdev);

	/* AER disable */
	pci_disable_pcie_error_reporting(pdev);

	pci_disable_device(pdev);
}

/* PCI Error Recovery (ERS) */
static const struct pci_error_handlers e1000_err_handler = {
	.error_detected = e1000_io_error_detected,
	.slot_reset = e1000_io_slot_reset,
	.resume = e1000_io_resume,
};

static DEFINE_PCI_DEVICE_TABLE(e1000_pci_tbl) = {
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH8_IFE), board_ich8lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH8_IFE_G), board_ich8lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH8_IFE_GT), board_ich8lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH8_IGP_AMT), board_ich8lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH8_IGP_C), board_ich8lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH8_IGP_M), board_ich8lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH8_IGP_M_AMT), board_ich8lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH8_82567V_3), board_ich8lan },

	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH9_IFE), board_ich9lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH9_IFE_G), board_ich9lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH9_IFE_GT), board_ich9lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH9_IGP_AMT), board_ich9lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH9_IGP_C), board_ich9lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH9_BM), board_ich9lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH9_IGP_M), board_ich9lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH9_IGP_M_AMT), board_ich9lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH9_IGP_M_V), board_ich9lan },

	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH10_R_BM_LM), board_ich9lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH10_R_BM_LF), board_ich9lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH10_R_BM_V), board_ich9lan },

	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH10_D_BM_LM), board_ich10lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH10_D_BM_LF), board_ich10lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_ICH10_D_BM_V), board_ich10lan },

	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_PCH_M_HV_LM), board_pchlan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_PCH_M_HV_LC), board_pchlan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_PCH_D_HV_DM), board_pchlan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_PCH_D_HV_DC), board_pchlan },

	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_PCH2_LV_LM), board_pch2lan },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_PCH2_LV_V), board_pch2lan },

	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_PCH_LPT_I217_LM), board_pch_lpt },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_PCH_LPT_I217_V), board_pch_lpt },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_PCH_LPTLP_I218_LM), board_pch_lpt },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_PCH_LPTLP_I218_V), board_pch_lpt },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_PCH_I218_LM2), board_pch_lpt },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_PCH_I218_V2), board_pch_lpt },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_PCH_I218_LM3), board_pch_lpt },
	{ PCI_VDEVICE(INTEL, E1000_DEV_ID_PCH_I218_V3), board_pch_lpt },

	{ 0, 0, 0, 0, 0, 0, 0 }	/* terminate list */
};
MODULE_DEVICE_TABLE(pci, e1000_pci_tbl);

static const struct dev_pm_ops e1000_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(e1000_suspend, e1000_resume)
	SET_RUNTIME_PM_OPS(e1000_runtime_suspend, e1000_runtime_resume,
			   e1000_idle)
};

/* PCI Device API Driver */
static struct pci_driver e1000_driver = {
	.name     = e1000e_driver_name,
	.id_table = e1000_pci_tbl,
	.probe    = e1000_probe,
	.remove   = e1000_remove,
	.driver   = {
		.pm = &e1000_pm_ops,
	},
	.shutdown = e1000_shutdown,
	.err_handler = &e1000_err_handler
};

/**
 * e1000_init_module - Driver Registration Routine
 *
 * e1000_init_module is the first routine called when the driver is
 * loaded. All it does is register with the PCI subsystem.
 **/
static int __init e1000_init_module(void)
{
	int ret;
	pr_info("Intel(R) PRO/1000 Network Driver - %s\n",
		e1000e_driver_version);
	pr_info("Copyright(c) 1999 - 2013 Intel Corporation.\n");
	ret = pci_register_driver(&e1000_driver);

	return ret;
}
module_init(e1000_init_module);

/**
 * e1000_exit_module - Driver Exit Cleanup Routine
 *
 * e1000_exit_module is called just before the driver is removed
 * from memory.
 **/
static void __exit e1000_exit_module(void)
{
	pci_unregister_driver(&e1000_driver);
}
module_exit(e1000_exit_module);

MODULE_AUTHOR("Intel Corporation, <linux.nics@intel.com>");
MODULE_DESCRIPTION("Intel(R) PRO/1000 Network Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(_E1000E_DRV_VERSION);

/* netdev.c */

#include "_netdev_rxtx.h"
#include "_netdev_intr.h"
#include "_netdev_vlan.h"
#include "_netdev_wd.h"

#include "_netdev_dbg.h"
